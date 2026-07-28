"""
Smart Star Glasses — ESP32 MicroPython  main.py
================================================
Hardware bill of materials:
  - ESP32 dev board (MicroPython firmware >= 1.22)
  - MPU-6050 6-axis IMU  (I2C: SDA=GPIO21, SCL=GPIO22)
  - ILI9341 or ST7789 SPI LCD  240x240 or 240x320
    (MOSI=GPIO23, SCLK=GPIO18, CS=GPIO5, DC=GPIO2, RST=GPIO4)
  - Optional: DS3231 RTC (same I2C bus)
  - Optional: NEO-6M GPS  (UART for lat/lon + time)
  - Optional: QMC5883L compass (I2C, corrects yaw drift)

Project overview:
  1. Read gyro + accel from MPU-6050
  2. Complementary filter → pitch / roll / yaw
  3. Map head orientation to sky viewing direction (alt / az)
  4. Compute Local Sidereal Time from UTC + observer longitude
  5. Convert star catalogue RA/Dec → Alt/Az
  6. Gnomonic-project visible stars onto LCD pixels
  7. Draw constellation lines, star glyphs, HUD overlay, crosshair

How to flash:
  ampy --port /dev/ttyUSB0 put main.py
  or use Thonny IDE
"""

import math
import time
import struct
from machine import I2C, SPI, Pin

# =============================================================
#  CONFIGURATION  — edit to match your location and hardware
# =============================================================

OBSERVER_LAT = 36.8    # degrees North  (default: Tunis, Tunisia)
OBSERVER_LON = 10.18   # degrees East

LCD_WIDTH  = 240       # LCD horizontal resolution in pixels
LCD_HEIGHT = 240       # LCD vertical   resolution in pixels

# Half-angle of the rendered field of view (degrees)
# Total FOV = 2 * FOV_H  x  2 * FOV_V  (default 60 x 60 deg)
FOV_H = 30.0
FOV_V = 30.0

# Faintest stars to display (lower mag = brighter)
MAG_LIMIT = 5.0

# ── Complementary filter weight (0=trust accel only, 1=trust gyro only)
ALPHA = 0.98

# ── Gyro/accel calibration offsets in deg/s and g respectively.
#    Run calibrate_gyro() once, keep device flat, paste printed values here.
GYRO_OFFSET  = (0.0, 0.0, 0.0)
ACCEL_OFFSET = (0.0, 0.0, 0.0)


# =============================================================
#  STAR CATALOGUE — abbreviated Yale Bright Star Catalog
#  Format: (RA_hours, Dec_degrees, apparent_magnitude, common_name)
# =============================================================
STARS = [
    # Orion
    (5.9195,   7.4069,  0.12, "Rigel"),
    (5.2788,   6.3497,  0.45, "Betelgeus"),
    (5.6355,  -1.9426,  1.70, "Alnilam"),
    (5.6032,  -1.2019,  1.74, "Alnitak"),
    (5.5333,  -0.2991,  2.23, "Mintaka"),
    (5.7950,  -9.6697,  1.65, "Saiph"),
    (4.8299,   6.9612,  1.64, "Bellatrix"),
    # Ursa Major
    (11.0622,  61.7511, 1.79, "Dubhe"),
    (11.0307,  56.3824, 2.37, "Merak"),
    (12.2570,  57.0326, 2.44, "Phecda"),
    (12.9004,  55.9598, 3.32, "Megrez"),
    (13.3992,  54.9254, 1.76, "Alioth"),
    (13.7923,  49.3133, 2.23, "Mizar"),
    (14.0651,  49.3133, 1.85, "Alkaid"),
    # Cassiopeia
    (0.6751,   56.5373, 2.23, "Schedar"),
    (0.1453,   59.1498, 2.27, "Caph"),
    (0.9451,   60.7165, 2.15, "GammaCas"),
    (1.4305,   60.2353, 2.66, "Ruchbah"),
    (1.9062,   63.6701, 3.35, "Segin"),
    # Leo
    (10.1395,  11.9672, 1.35, "Regulus"),
    (11.8176,  14.5720, 2.14, "Denebola"),
    (10.3328,  19.8417, 2.61, "Algieba"),
    (11.2352,  20.5238, 3.44, "Zosma"),
    # Scorpius
    (16.4905, -26.4320, 0.96, "Antares"),
    (17.5622, -37.1038, 1.62, "Shaula"),
    (16.8361, -34.2934, 2.31, "Sargas"),
    (15.9808, -22.6217, 2.89, "Graffias"),
    (16.0053, -11.3723, 3.21, "Dschubba"),
    (17.7080, -39.0292, 2.39, "Lesath"),
    # Sagittarius
    (18.3499, -29.8281, 1.79, "KausAust"),
    (18.9281, -26.2963, 2.72, "Nunki"),
    # Virgo
    (13.4199, -11.1614, 0.98, "Spica"),
    (12.6942,  -1.4494, 2.83, "Porrima"),
    # Gemini
    (7.7553,   28.0262, 1.14, "Pollux"),
    (7.5764,   31.8883, 1.58, "Castor"),
    (6.7548,   25.1312, 1.93, "Alhena"),
    (7.0680,   20.5703, 3.15, "Tejat"),
    # Taurus
    (4.5987,   16.5093, 0.85, "Aldebaran"),
    # Auriga
    (5.2782,   45.9980, 0.08, "Capella"),
    (5.9921,   44.9474, 1.90, "Menkalinan"),
    # Canis Major
    (6.7524,  -16.7161,-1.46, "Sirius"),
    (6.3783,  -17.9559, 1.50, "Adhara"),
    (7.1397,  -26.3932, 1.98, "Wezen"),
    # Canis Minor
    (7.6550,    5.2250, 0.34, "Procyon"),
    # Bootes
    (14.2610,  19.1824,-0.04, "Arcturus"),
    (14.5344,  38.3083, 2.35, "Izar"),
    # Lyra
    (18.6157,  38.7836, 0.03, "Vega"),
    # Cygnus
    (20.6905,  45.2803, 1.25, "Deneb"),
    (19.5121,  27.9597, 2.46, "Albireo"),
    (20.3705,  40.2567, 2.23, "Sadr"),
    # Aquila
    (19.8461,   8.8683, 0.77, "Altair"),
    (19.7747,  10.6137, 2.72, "Tarazed"),
    # Southern Cross
    (12.4433, -63.0990, 0.77, "Acrux"),
    (12.5194, -57.1128, 1.25, "Mimosa"),
    (12.2523, -58.7489, 1.59, "Gacrux"),
    # Centaurus
    (14.6600, -60.8353,-0.01, "RigilKent"),
    (14.0637, -60.3730, 0.61, "Hadar"),
    (13.6643, -53.4663, 2.06, "Menkent"),
    # Eridanus
    (1.6286,  -57.2367, 0.46, "Achernar"),
    # Piscis Austrinus
    (22.9608, -29.6223, 1.16, "Fomalhaut"),
    # Pegasus
    (22.6910,  10.8315, 2.38, "Enif"),
    (0.2206,   15.1836, 2.83, "Alpheratz"),
    (23.0629,  28.0827, 2.42, "Scheat"),
    (23.0629,  15.2053, 2.49, "Markab"),
    # Andromeda
    (0.6552,   30.8611, 2.06, "Mirach"),
    (1.1622,   35.6203, 2.10, "Almach"),
    # Aries
    (2.1195,   23.4624, 2.00, "Hamal"),
    # Ophiuchus
    (17.7223,   4.5667, 2.08, "Rasalhague"),
    # Corona Borealis
    (15.5780,  26.7148, 2.23, "Alphecca"),
    # Draco
    (17.9436,  51.4889, 2.23, "Eltanin"),
    # Ursa Minor
    (2.5301,   89.2641, 2.02, "Polaris"),
    (14.8450,  74.1554, 2.08, "Kochab"),
    # Cetus
    (2.7220,   -3.2358, 2.04, "Diphda"),
    # Carina
    (6.3992,  -52.6957,-0.72, "Canopus"),
    (9.2199,  -69.7172, 1.67, "Miaplacidus"),
    # Perseus
    (3.0793,   40.9557, 1.79, "Mirfak"),
    # Hercules
    (17.2442,  14.3901, 2.78, "Kornephoros"),
    # Libra
    (15.2839,  -9.3829, 2.61, "Zubenelg"),
]


# =============================================================
#  MPU-6050  I2C DRIVER
# =============================================================
class MPU6050:
    """Minimal MPU-6050 driver for MicroPython."""

    ADDR         = 0x68
    REG_PWR_MGMT = 0x6B
    REG_SMPLRT   = 0x19
    REG_CONFIG   = 0x1A
    REG_GYRO_CFG = 0x1B
    REG_ACCL_CFG = 0x1C
    REG_ACCEL    = 0x3B
    REG_GYRO     = 0x43
    REG_TEMP     = 0x41

    GYRO_SCALE   = 131.0    # LSB / (deg/s) at +/-250 deg/s
    ACCEL_SCALE  = 16384.0  # LSB / g       at +/-2 g

    def __init__(self, i2c, addr=None):
        self.i2c  = i2c
        self.addr = addr or self.ADDR
        self._boot()

    def _wr(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes([val]))

    def _rd(self, reg, n):
        return self.i2c.readfrom_mem(self.addr, reg, n)

    def _boot(self):
        self._wr(self.REG_PWR_MGMT, 0x00)   # wake up
        self._wr(self.REG_SMPLRT,   0x07)   # 125 Hz sample rate
        self._wr(self.REG_CONFIG,   0x00)   # no DLPF
        self._wr(self.REG_GYRO_CFG, 0x00)   # +/-250 deg/s
        self._wr(self.REG_ACCL_CFG, 0x00)   # +/-2 g

    def read_accel(self):
        """Return calibrated (ax, ay, az) in units of g."""
        ax, ay, az = struct.unpack(">hhh", self._rd(self.REG_ACCEL, 6))
        s = self.ACCEL_SCALE
        return (ax/s - ACCEL_OFFSET[0],
                ay/s - ACCEL_OFFSET[1],
                az/s - ACCEL_OFFSET[2])

    def read_gyro(self):
        """Return calibrated (gx, gy, gz) in deg/s."""
        gx, gy, gz = struct.unpack(">hhh", self._rd(self.REG_GYRO, 6))
        s = self.GYRO_SCALE
        return (gx/s - GYRO_OFFSET[0],
                gy/s - GYRO_OFFSET[1],
                gz/s - GYRO_OFFSET[2])

    def temperature(self):
        """Return die temperature in Celsius."""
        raw = struct.unpack(">h", self._rd(self.REG_TEMP, 2))[0]
        return raw / 340.0 + 36.53


# =============================================================
#  ILI9341 / ST7789  SPI LCD DRIVER
# =============================================================
class LCD:
    """
    Minimal SPI LCD driver compatible with ILI9341 (240x320)
    and ST7789 (240x240).  Uses direct SPI pixel writes — no
    intermediate frame buffer — to keep RAM usage low on ESP32.
    """

    # Command codes (ILI9341 / ST7789 compatible)
    _SWRST  = 0x01
    _SLPOUT = 0x11
    _DISPON = 0x29
    _CASET  = 0x2A
    _RASET  = 0x2B
    _RAMWR  = 0x2C
    _MADCTL = 0x36
    _COLMOD = 0x3A

    # Colour palette (RGB565)
    BLACK    = 0x0000
    WHITE    = 0xFFFF
    RED      = 0xF800
    GREEN    = 0x07E0
    BLUE     = 0x001F
    YELLOW   = 0xFFE0
    CYAN     = 0x07FF
    MAGENTA  = 0xF81F
    SKY_DARK = 0x0010   # near-black deep navy for night sky

    # Star spectral colours (RGB565)
    COL_HOT    = 0xAEFF   # O/B blue-white
    COL_WHITE  = 0xFFFF   # A white
    COL_SOLAR  = 0xFF84   # F/G yellow-white (sun-like)
    COL_ORANGE = 0xFC60   # K orange giant
    COL_RED    = 0xF800   # M red giant

    def __init__(self, spi, cs, dc, rst, w=240, h=240):
        self.spi = spi
        self.cs  = cs
        self.dc  = dc
        self.rst = rst
        self.w   = w
        self.h   = h
        self._init()

    # ── low-level helpers ──────────────────────────────────────
    def _cmd(self, c):
        self.cs(0); self.dc(0)
        self.spi.write(bytes([c]))
        self.cs(1)

    def _dat(self, d):
        self.cs(0); self.dc(1)
        self.spi.write(d if isinstance(d, (bytes, bytearray)) else bytes([d]))
        self.cs(1)

    def _cd(self, c, *args):
        self._cmd(c)
        if args:
            self._dat(bytes(args))

    def _init(self):
        self.rst(0); time.sleep_ms(100)
        self.rst(1); time.sleep_ms(100)
        self._cmd(self._SWRST);  time.sleep_ms(150)
        self._cmd(self._SLPOUT); time.sleep_ms(500)
        self._cd(self._COLMOD, 0x55)  # 16-bit RGB565
        self._cd(self._MADCTL, 0x00)  # portrait, no mirror
        self._cmd(self._DISPON)

    # ── drawing primitives ────────────────────────────────────
    def _window(self, x0, y0, x1, y1):
        self._cd(self._CASET, x0>>8, x0&0xFF, x1>>8, x1&0xFF)
        self._cd(self._RASET, y0>>8, y0&0xFF, y1>>8, y1&0xFF)
        self._cmd(self._RAMWR)

    def fill(self, colour):
        self._window(0, 0, self.w-1, self.h-1)
        chunk = bytes([(colour>>8)&0xFF, colour&0xFF]) * 64
        self.cs(0); self.dc(1)
        for _ in range(self.w * self.h // 64):
            self.spi.write(chunk)
        self.cs(1)

    def pixel(self, x, y, colour):
        if 0 <= x < self.w and 0 <= y < self.h:
            self._window(x, y, x, y)
            self.cs(0); self.dc(1)
            self.spi.write(bytes([(colour>>8)&0xFF, colour&0xFF]))
            self.cs(1)

    def hline(self, x, y, length, colour):
        if not (0 <= y < self.h): return
        x0 = max(0, x); x1 = min(self.w-1, x+length-1)
        if x0 > x1: return
        self._window(x0, y, x1, y)
        n = x1 - x0 + 1
        self.cs(0); self.dc(1)
        self.spi.write(bytes([(colour>>8)&0xFF, colour&0xFF]) * n)
        self.cs(1)

    def vline(self, x, y, length, colour):
        for i in range(length):
            self.pixel(x, y+i, colour)

    def circle(self, cx, cy, r, colour, fill=False):
        """Bresenham midpoint circle, optionally solid."""
        if r <= 0:
            self.pixel(cx, cy, colour); return
        x, y, err = r, 0, 0
        pts = []
        while x >= y:
            for dx, dy in [(x,y),(-x,y),(x,-y),(-x,-y),
                           (y,x),(-y,x),(y,-x),(-y,-x)]:
                pts.append((cx+dx, cy+dy))
            y += 1
            if err <= 0: err += 2*y + 1
            else:        x -= 1; err += 2*(y-x) + 1
        if fill:
            # Scanline fill
            rows = {}
            for px, py in pts:
                if py not in rows: rows[py] = [px, px]
                rows[py][0] = min(rows[py][0], px)
                rows[py][1] = max(rows[py][1], px)
            for py, (x0, x1) in rows.items():
                self.hline(x0, py, x1-x0+1, colour)
        else:
            for px, py in pts:
                self.pixel(px, py, colour)

    def star_glyph(self, cx, cy, size, colour):
        """Star glyph: small filled circle + cardinal spikes."""
        self.circle(cx, cy, max(0, size-1), colour, fill=True)
        if size >= 2:
            for d in range(1, size+3):
                for dx, dy in [(d,0),(-d,0),(0,d),(0,-d)]:
                    self.pixel(cx+dx, cy+dy, colour)

    # ── 5x8 bitmap font ──────────────────────────────────────
    _F = {
        ' ':b'\x00\x00\x00\x00\x00','!':b'\x00\x5f\x00\x00\x00',
        '+':b'\x08\x08\x3e\x08\x08','-':b'\x08\x08\x08\x08\x08',
        '.':b'\x00\x60\x60\x00\x00',':':b'\x00\x36\x36\x00\x00',
        '/':b'\x20\x10\x08\x04\x02',
        '0':b'\x3e\x51\x49\x45\x3e','1':b'\x00\x42\x7f\x40\x00',
        '2':b'\x42\x61\x51\x49\x46','3':b'\x21\x41\x45\x4b\x31',
        '4':b'\x18\x14\x12\x7f\x10','5':b'\x27\x45\x45\x45\x39',
        '6':b'\x3c\x4a\x49\x49\x30','7':b'\x01\x71\x09\x05\x03',
        '8':b'\x36\x49\x49\x49\x36','9':b'\x06\x49\x49\x29\x1e',
        'A':b'\x7e\x11\x11\x11\x7e','B':b'\x7f\x49\x49\x49\x36',
        'C':b'\x3e\x41\x41\x41\x22','D':b'\x7f\x41\x41\x22\x1c',
        'E':b'\x7f\x49\x49\x49\x41','F':b'\x7f\x09\x09\x01\x01',
        'G':b'\x3e\x41\x41\x51\x32','H':b'\x7f\x08\x08\x08\x7f',
        'I':b'\x41\x7f\x41\x00\x00','J':b'\x20\x40\x41\x3f\x01',
        'K':b'\x7f\x08\x14\x22\x41','L':b'\x7f\x40\x40\x40\x40',
        'M':b'\x7f\x02\x04\x02\x7f','N':b'\x7f\x04\x08\x10\x7f',
        'O':b'\x3e\x41\x41\x41\x3e','P':b'\x7f\x09\x09\x09\x06',
        'Q':b'\x3e\x41\x51\x21\x5e','R':b'\x7f\x09\x19\x29\x46',
        'S':b'\x46\x49\x49\x49\x31','T':b'\x01\x01\x7f\x01\x01',
        'U':b'\x3f\x40\x40\x40\x3f','V':b'\x1f\x20\x40\x20\x1f',
        'W':b'\x3f\x40\x38\x40\x3f','X':b'\x63\x14\x08\x14\x63',
        'Y':b'\x07\x08\x70\x08\x07','Z':b'\x61\x51\x49\x45\x43',
    }

    def text(self, x, y, s, fg=0xFFFF, bg=0x0000):
        """Draw ASCII text using the 5x8 bitmapped font."""
        cx = x
        for ch in s.upper():
            bits = self._F.get(ch, self._F[' '])
            for col in range(5):
                b = bits[col]
                for row in range(8):
                    self.pixel(cx+col, y+row, fg if (b>>row)&1 else bg)
            cx += 6

    # ── Stellarium-style reticle ──────────────────────────────
    def crosshair(self, colour=0x07FF):
        cx = self.w // 2
        cy = self.h // 2
        gap = 8; arm = 14
        dim = 0x4228
        self.hline(cx-gap-arm, cy, arm,   colour)
        self.hline(cx+gap,     cy, arm,   colour)
        self.vline(cx, cy-gap-arm, arm,   colour)
        self.vline(cx, cy+gap,     arm,   colour)
        self.pixel(cx, cy, colour)
        for d in range(1, 5):
            self.pixel(cx+gap+d, cy, dim); self.pixel(cx-gap-d, cy, dim)
            self.pixel(cx, cy+gap+d, dim); self.pixel(cx, cy-gap-d, dim)


# =============================================================
#  ORIENTATION TRACKER  —  Complementary Filter
# =============================================================
class OrientationTracker:
    """
    Fuses gyroscope + accelerometer with a complementary filter.

    Output angles:
      pitch  — elevation above horizon  (-90 to +90 deg)
      roll   — bank angle               (-180 to +180 deg)
      yaw    — compass bearing           (0 to 360 deg)
                NOTE: yaw drifts over time without a magnetometer.
                Add QMC5883L for magnetometer correction.
    """
    def __init__(self, imu):
        self.imu   = imu
        self.pitch = 0.0
        self.roll  = 0.0
        self.yaw   = 0.0
        self._last = time.ticks_ms()

    def update(self):
        ax, ay, az = self.imu.read_accel()
        gx, gy, gz = self.imu.read_gyro()

        now = time.ticks_ms()
        dt  = max(time.ticks_diff(now, self._last) / 1000.0, 1e-4)
        self._last = now

        # Accelerometer-derived tilt (stable but noisy)
        ap = math.degrees(math.atan2(ax, math.sqrt(ay*ay + az*az)))
        ar = math.degrees(math.atan2(ay, math.sqrt(ax*ax + az*az)))

        # Complementary fusion
        self.pitch = ALPHA * (self.pitch + gx*dt) + (1-ALPHA)*ap
        self.roll  = ALPHA * (self.roll  + gy*dt) + (1-ALPHA)*ar
        self.yaw   = (self.yaw + gz*dt) % 360.0

        return self.pitch, self.roll, self.yaw


# =============================================================
#  ASTRONOMICAL CALCULATIONS
# =============================================================

def _rad(d): return d * math.pi / 180.0
def _deg(r): return r * 180.0 / math.pi


def julian_day(year, month, day, hour=0, minute=0, second=0):
    """Return Julian Day Number for the given UTC instant."""
    if month <= 2:
        year -= 1; month += 12
    A = int(year / 100)
    B = 2 - A + int(A / 4)
    return (int(365.25*(year + 4716)) +
            int(30.6001*(month + 1)) +
            day + B - 1524.5 +
            (hour + minute/60.0 + second/3600.0) / 24.0)


def greenwich_sidereal_time(jd):
    """
    Greenwich Apparent Sidereal Time in degrees.
    Accuracy ~0.1 deg — sufficient for visual star-spotting.
    """
    T = (jd - 2451545.0) / 36525.0
    return (280.46061837 +
            360.98564736629 * (jd - 2451545.0) +
            T*T * 0.000387933 -
            T*T*T / 38710000.0) % 360.0


def local_sidereal_time(jd, lon_deg):
    """Local Sidereal Time in degrees."""
    return (greenwich_sidereal_time(jd) + lon_deg) % 360.0


def ra_dec_to_alt_az(ra_h, dec_deg, lat_deg, lst_deg):
    """
    Convert equatorial (RA, Dec) to horizontal (Altitude, Azimuth).

    Parameters
    ----------
    ra_h     : Right Ascension in decimal hours
    dec_deg  : Declination in degrees
    lat_deg  : Observer geographic latitude in degrees
    lst_deg  : Local Sidereal Time in degrees

    Returns
    -------
    (altitude_deg, azimuth_deg)
    Azimuth: 0 = North, 90 = East, 180 = South, 270 = West
    """
    ha  = _rad((lst_deg - ra_h * 15.0) % 360.0)  # Hour Angle
    dec = _rad(dec_deg)
    lat = _rad(lat_deg)

    sin_alt = (math.sin(dec)*math.sin(lat) +
               math.cos(dec)*math.cos(lat)*math.cos(ha))
    alt = math.asin(max(-1.0, min(1.0, sin_alt)))

    denom   = math.cos(alt) * math.cos(lat) + 1e-9
    cos_az  = (math.sin(dec) - math.sin(alt)*math.sin(lat)) / denom
    az      = math.acos(max(-1.0, min(1.0, cos_az)))

    if math.sin(ha) > 0:
        az = 2.0*math.pi - az

    return _deg(alt), _deg(az)


def mag_to_size(mag):
    """Map apparent magnitude to pixel radius for star glyph."""
    if   mag < -0.5: return 4
    elif mag <  0.5: return 3
    elif mag <  1.5: return 2
    elif mag <  3.0: return 1
    else:            return 0


def mag_to_colour(mag, ra_h):
    """
    Heuristic star colour from magnitude + RA.
    Replace with B-V colour index lookup for photometric accuracy.
    """
    palette = [LCD.COL_HOT, LCD.COL_WHITE, LCD.COL_SOLAR,
               LCD.COL_ORANGE, LCD.COL_RED]
    return palette[int(ra_h * 7) % 5]


# =============================================================
#  GNOMONIC PROJECTION  (same model as Stellarium)
# =============================================================

def project(star_alt, star_az, view_alt, view_az, w, h, fov_h, fov_v):
    """
    Project celestial coordinates onto screen pixels via gnomonic
    (rectilinear / tangent-plane) projection.

    Parameters
    ----------
    star_alt / star_az  : star position (degrees)
    view_alt / view_az  : centre of the current view (degrees)
    w, h                : screen width and height (pixels)
    fov_h, fov_v        : half-FOV angles in each axis (degrees)

    Returns
    -------
    (px, py) pixel coordinate, or None if outside the viewport.
    """
    daz  = ((star_az - view_az + 180.0) % 360.0) - 180.0
    dalt =   star_alt - view_alt

    # Quick cull — reject stars clearly outside the FOV
    if abs(daz) > fov_h * 1.5 or abs(dalt) > fov_v * 1.5:
        return None

    c1   = math.cos(_rad(star_alt))
    s1   = math.sin(_rad(star_alt))
    c2   = math.cos(_rad(view_alt))
    s2   = math.sin(_rad(view_alt))
    cosc = s1*s2 + c1*c2*math.cos(_rad(daz))

    if cosc < 0.01:
        return None

    xt = c1 * math.sin(_rad(daz)) / cosc
    yt = (s1*c2 - c1*s2*math.cos(_rad(daz))) / cosc

    px = int(w/2 + xt * (w  / (2.0 * _rad(fov_h))))
    py = int(h/2 - yt * (h  / (2.0 * _rad(fov_v))))

    if 0 <= px < w and 0 <= py < h:
        return (px, py)
    return None


# =============================================================
#  CONSTELLATION STICK FIGURES
# =============================================================

CONSTELLATION_LINES = [
    # Orion
    ("Betelgeus", "Bellatrix"), ("Betelgeus", "Alnilam"),
    ("Rigel",     "Saiph"),     ("Saiph",     "Alnilam"),
    ("Alnilam",   "Bellatrix"), ("Alnitak",   "Alnilam"),
    ("Alnilam",   "Mintaka"),
    # Ursa Major (Big Dipper bowl + handle)
    ("Dubhe",  "Merak"),  ("Merak",  "Phecda"), ("Phecda", "Megrez"),
    ("Megrez", "Alioth"), ("Alioth", "Mizar"),  ("Mizar",  "Alkaid"),
    ("Megrez", "Dubhe"),
    # Cassiopeia
    ("Caph",     "Schedar"),  ("Schedar",  "GammaCas"),
    ("GammaCas", "Ruchbah"),  ("Ruchbah",  "Segin"),
    # Leo
    ("Regulus", "Algieba"), ("Algieba", "Zosma"), ("Zosma", "Denebola"),
    # Scorpius
    ("Antares", "Dschubba"), ("Antares", "Sargas"),
    ("Sargas",  "Shaula"),   ("Shaula",  "Lesath"),
    # Gemini
    ("Castor", "Pollux"), ("Pollux", "Alhena"), ("Alhena", "Tejat"),
    # Cygnus (Northern Cross)
    ("Deneb", "Sadr"), ("Sadr", "Albireo"),
    # Aquila
    ("Altair", "Tarazed"),
    # Bootes
    ("Arcturus", "Izar"),
]


def _star_index(stars):
    """Map star name → (ra_h, dec_deg) for constellation rendering."""
    return {name: (ra_h, dec_deg)
            for ra_h, dec_deg, _, name in stars}


def _bresenham(lcd, x0, y0, x1, y1, colour):
    """Draw a line using Bresenham's algorithm."""
    dx = abs(x1-x0); dy = abs(y1-y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    for _ in range(600):            # guard upper bound
        lcd.pixel(x0, y0, colour)
        if x0 == x1 and y0 == y1: break
        e2 = 2 * err
        if e2 > -dy: err -= dy; x0 += sx
        if e2 <  dx: err += dx; y0 += sy


def draw_constellation_lines(lcd, idx, lat, lst, view_alt, view_az):
    """Render all constellation stick-figure lines for current view."""
    DIM_CYAN = 0x294A
    for na, nb in CONSTELLATION_LINES:
        if na not in idx or nb not in idx:
            continue
        ra_a, dec_a = idx[na]
        ra_b, dec_b = idx[nb]
        alt_a, az_a = ra_dec_to_alt_az(ra_a, dec_a, lat, lst)
        alt_b, az_b = ra_dec_to_alt_az(ra_b, dec_b, lat, lst)
        pa = project(alt_a, az_a, view_alt, view_az,
                     lcd.w, lcd.h, FOV_H, FOV_V)
        pb = project(alt_b, az_b, view_alt, view_az,
                     lcd.w, lcd.h, FOV_H, FOV_V)
        if pa and pb:
            _bresenham(lcd, pa[0], pa[1], pb[0], pb[1], DIM_CYAN)


# =============================================================
#  HUD OVERLAY
# =============================================================

_CARDINALS = ["N","NE","E","SE","S","SW","W","NW"]


def draw_hud(lcd, pitch, yaw, star_count):
    """
    Render heads-up display:
      - Top centre  : compass bearing + cardinal direction
      - Bottom left : elevation angle above/below horizon
      - Bottom right: count of visible stars in current view
    """
    sky = LCD.SKY_DARK
    card = _CARDINALS[int((yaw + 22.5) % 360 / 45)]
    lcd.text(lcd.w//2 - 18, 3,
             "{:03d} {}".format(int(yaw), card),
             fg=LCD.CYAN,    bg=sky)
    lcd.text(3, lcd.h - 11,
             "EL{:+03d}".format(int(pitch)),
             fg=LCD.YELLOW,  bg=sky)
    lcd.text(lcd.w - 46, lcd.h - 11,
             "ST{:02d}".format(min(star_count, 99)),
             fg=LCD.MAGENTA, bg=sky)


# =============================================================
#  FRAME RENDERER
# =============================================================

def render_frame(lcd, stars, idx, view_alt, view_az,
                 lat, lst, labels=True):
    """
    Render one complete star-map frame onto the LCD.

    Steps:
      1. Fill sky background (deep navy)
      2. Draw constellation stick figures
      3. Draw each visible star as a sized + coloured glyph
      4. Optionally label bright stars (mag <= 2)
      5. Return count of stars rendered in this frame
    """
    lcd.fill(LCD.SKY_DARK)

    draw_constellation_lines(lcd, idx, lat, lst, view_alt, view_az)

    visible = 0
    for ra_h, dec_deg, mag, name in stars:
        if mag > MAG_LIMIT:
            continue
        alt, az = ra_dec_to_alt_az(ra_h, dec_deg, lat, lst)
        if alt < -5.0:
            continue
        pos = project(alt, az, view_alt, view_az,
                      lcd.w, lcd.h, FOV_H, FOV_V)
        if pos is None:
            continue
        px, py = pos
        lcd.star_glyph(px, py, mag_to_size(mag), mag_to_colour(mag, ra_h))
        if labels and mag <= 2.0 and 12 < py < lcd.h - 12:
            lcd.text(px + 4, py - 4, name[:6],
                     fg=0x8410, bg=LCD.SKY_DARK)
        visible += 1

    return visible


# =============================================================
#  TIME  UTILITIES
# =============================================================

def get_utc():
    """
    Return current UTC as (year, month, day, hour, minute, second).
    Reads from MicroPython's RTC; update at boot via sync_ntp().
    """
    try:
        from machine import RTC
        t = RTC().datetime()  # (Y, M, D, weekday, h, m, s, sub)
        return t[0], t[1], t[2], t[4], t[5], t[6]
    except Exception:
        return 2025, 7, 1, 21, 0, 0    # fallback — update for accuracy


def sync_ntp():
    """
    Synchronise the ESP32 RTC from an NTP server over WiFi.
    Silently skips if no WiFi connection is active.
    """
    try:
        import network, ntptime
        if network.WLAN(network.STA_IF).isconnected():
            ntptime.settime()
            print("[NTP] RTC synchronised")
        else:
            print("[NTP] No WiFi — using current RTC time")
    except Exception as e:
        print("[NTP] Failed:", e)


# =============================================================
#  GYRO CALIBRATION UTILITY
# =============================================================

def calibrate_gyro(imu, samples=300):
    """
    Compute mean gyro bias at rest over `samples` readings.
    Keep the device perfectly flat and motionless.
    Paste the printed GYRO_OFFSET tuple into the config section.
    """
    print("Calibrating gyro — hold device still for ~3 seconds ...")
    gx_s = gy_s = gz_s = 0.0
    for i in range(samples):
        gx, gy, gz = imu.read_gyro()
        gx_s += gx; gy_s += gy; gz_s += gz
        time.sleep_ms(10)
        if i % 60 == 0:
            print("  {}/{} samples".format(i, samples))
    offset = (gx_s/samples, gy_s/samples, gz_s/samples)
    print("GYRO_OFFSET =", tuple(round(v,4) for v in offset))
    return offset


# =============================================================
#  MAIN APPLICATION
# =============================================================

def main():
    print("=" * 46)
    print("   Smart Star Glasses — ESP32 MicroPython")
    print("=" * 46)

    # ── I2C bus ──────────────────────────────────────────────
    i2c  = I2C(0, sda=Pin(21), scl=Pin(22), freq=400_000)
    devs = i2c.scan()
    print("I2C scan:", [hex(a) for a in devs])

    if 0x68 not in devs and 0x69 not in devs:
        print("ERROR: MPU-6050 not found!  "
              "Check SDA/SCL wiring and 3.3 V power.")

    imu = MPU6050(i2c)

    # ── SPI LCD ──────────────────────────────────────────────
    spi = SPI(1,
              baudrate = 40_000_000,
              polarity = 0,
              phase    = 0,
              sck      = Pin(18),
              mosi     = Pin(23))
    cs  = Pin(5, Pin.OUT, value=1)
    dc  = Pin(2, Pin.OUT, value=0)
    rst = Pin(4, Pin.OUT, value=1)

    lcd = LCD(spi, cs, dc, rst, w=LCD_WIDTH, h=LCD_HEIGHT)

    # ── NTP time sync ────────────────────────────────────────
    sync_ntp()

    # ── (Optional) Gyro calibration ─────────────────────────
    #   Uncomment once, run with device flat, paste output values
    #   into GYRO_OFFSET at the top of this file.
    # global GYRO_OFFSET
    # GYRO_OFFSET = calibrate_gyro(imu)

    # ── Splash screen ────────────────────────────────────────
    lcd.fill(LCD.BLACK)
    lcd.text(10, 50, "SMART STAR GLASSES", fg=LCD.CYAN,   bg=LCD.BLACK)
    lcd.text(10, 70, "INITIALISING...",    fg=0x8410,      bg=LCD.BLACK)
    lcd.text(10, 90, "LAT:{:.1f} LON:{:.1f}".format(
             OBSERVER_LAT, OBSERVER_LON),  fg=LCD.YELLOW,  bg=LCD.BLACK)
    time.sleep(2)

    # ── Build orientation tracker ────────────────────────────
    tracker = OrientationTracker(imu)
    star_idx = _star_index(STARS)

    # ── Warm-up: prime the complementary filter ──────────────
    print("Warming up orientation filter (2 s) ...")
    for _ in range(200):
        tracker.update()
        time.sleep_ms(10)
    print("Entering render loop — aim glasses at the sky!")

    frame  = 0
    labels = True

    # ── Main render loop ─────────────────────────────────────
    while True:

        # 1. Read orientation
        pitch, roll, yaw = tracker.update()

        # 2. Compute current Local Sidereal Time
        y, mo, d, h, mi, s = get_utc()
        jd  = julian_day(y, mo, d, h, mi, s)
        lst = local_sidereal_time(jd, OBSERVER_LON)

        # 3. Viewing direction from head pose
        #    pitch > 0 → tilting head back (looking up toward zenith)
        #    yaw   = 0 → facing North (on boot; drift-corrected if compass added)
        view_alt = pitch   # −90..+90  deg (elevation above horizon)
        view_az  = yaw     # 0..360    deg (compass bearing)

        # 4. Render star map
        count = render_frame(
            lcd, STARS, star_idx,
            view_alt, view_az,
            OBSERVER_LAT, lst,
            labels=labels,
        )

        # 5. Draw crosshair reticle
        lcd.crosshair(colour=LCD.CYAN)

        # 6. Draw HUD overlay
        draw_hud(lcd, pitch, yaw, count)

        # 7. Toggle star name labels every ~5 min (9000 frames at 30 ms)
        frame += 1
        if frame % 9000 == 0:
            labels = not labels

        # 8. Brief pause — main delay is SPI pixel writes (~30 ms/frame)
        time.sleep_ms(30)


# =============================================================
#  ENTRY POINT
# =============================================================
if __name__ == "__main__":
    main()
