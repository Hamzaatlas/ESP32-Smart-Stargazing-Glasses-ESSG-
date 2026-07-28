# 🌌 Smart Star Glasses (MicroPython / ESP32)

An open-source, head-tracked heads-up display (HUD) star map for smart glasses. Built on ESP32 in MicroPython, it projects real-time star coordinates onto an optical lens / LCD display matching your exact line of sight—letting you spot constellations without holding a phone.

## ✨ Features
- **Head Orientation Tracking**: 6-axis MPU-6050 sensor fused via a complementary filter for smooth pitch/roll/yaw response.
- **Real-Time Astronomy Engine**: Converts Right Ascension & Declination to local Horizon coordinates (Alt/Az) using Julian Day & Sidereal Time calculations.
- **Stellarium-Style Projection**: Gnomonic rectilinear viewport mapping with dynamic star magnitude sizing, spectral colors, and constellation stick figures.
- **Lightweight LCD Renderer**: High-performance SPI display driver (ILI9341 / ST7789) designed for low-RAM microcontrollers.
- **Heads-Up Display (HUD)**: On-screen reticle, compass heading, elevation angle, and live visible star counter.

## 🛠️ Hardware Needed
- **MCU**: ESP32 Development Board
- **IMU**: MPU-6050 Gyroscope + Accelerometer (I2C: GPIO 21 / 22)
- **Display**: 240x240 or 240x320 SPI LCD (ST7789 / ILI9341)
- *(Optional)* DS3231 RTC Module or GPS module for automatic location & time sync.

## 🔌 Wiring summary:

| Module   |ESP32 GPIO| 
| -------- | -------- | 
| MPU-6050 SDA   | GPIO 21  |
| MPU-6050 SCL  | GPIO 22   | 
| LCD MOSI   | GPIO 23   | 
| LCD SCLK   | GPIO 18   |
| LCD CS  | GPIO 5   | 
| LCD DC  | GPIO 2   |
| LCD RST  | GPIO 4   | 


## 🚀 Quick Start
1. Flash MicroPython onto your ESP32.
```bash
# Install ampy
pip install adafruit-ampy

# Upload
ampy --port /dev/ttyUSB0 put main.py
```
2. Edit `OBSERVER_LAT` and `OBSERVER_LON` in `main.py` with your coordinates.

```bash
OBSERVER_LAT = 36.8    # degrees North  (default: Tunis, Tunisia)
OBSERVER_LON = 10.18   # degrees East
```

4. Upload `main.py` using `ampy` or Thonny IDE.
