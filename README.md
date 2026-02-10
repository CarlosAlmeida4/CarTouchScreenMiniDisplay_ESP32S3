# CarTouchScreenMiniDisplay – ESP32-S3

Touchscreen mini display project for ESP32-S3 designed for automotive or embedded dashboard applications.  
This project demonstrates how to build a responsive graphical user interface using **LVGL** and **SquareLine Studio**, running on an ESP32-S3 with a touch display.

The goal of this project is to create a small touchscreen UI to integrate into the middle display of a vehicle to act mostly as an inclinometer, but also as HMI to other vehicle information

---

## Overview

This project implements:

- ESP32-S3 based touchscreen interface
- UI designed in **SquareLine Studio**
- Graphics powered by **LVGL**
- ESP-IDF based firmware
- Modular structure suitable for future telemetry or vehicle data integration
- Over the air functionality

The UI is exported from SquareLine Studio and integrated directly into the ESP-IDF project. Application logic runs independently from the UI layer, allowing easy modification or replacement of screens without changing low-level display code.

---

## Features

- Touchscreen graphical interface
- LVGL-based UI rendering
- SquareLine Studio workflow
- ESP-IDF project structure
- Ready for automotive UI extensions
- Clean separation between:
  - Display driver
  - UI layer
  - Application logic
- **Sensor Integration**: QMI8658 IMU for inclinometer (roll/pitch), touch controller support
- **OTA Updates**: Over-the-air firmware update capability
- **Optional Sensors**: Support for magnetometer (BMM150), light sensor, RTC, vibration motor

---

# User Interface

![User Interface Design](\Designs\ScreenDesign.png)

---

## Project Structure

```
CarTouchScreenMiniDisplay_ESP32S3/
│
├── main/
│ ├── main.cpp
│ └── application logic
│
├── components/
│ ├── display/          # Display driver & LVGL integration
│ ├── ui/               # SquareLine Studio generated UI
│ ├── SensorLib/        # Sensor drivers & interfaces
│ ├── System/           # System management & task coordination
│ ├── qmi8658cInterface/# IMU sensor interface
│ ├── OTAUpdater/       # Over-the-air update handler
│ └── esp_lcd_sh8601/   # LCD controller driver
│
├── SquarelineProject/  # SquareLine Studio project source
├── CMakeLists.txt
├── sdkconfig
├── partitions.csv      # Flash partitions for OTA
└── README.md
```

### Main Components

#### `main/`

Application entry point with FreeRTOS task initialization.

#### `components/display/`

Display driver integration with LVGL, touch input handling, and rendering loop.

#### `components/ui/`

Auto-generated UI from SquareLine Studio (screens, components, fonts, images).

#### `components/SensorLib/`

Comprehensive sensor library supporting:

- **IMU**: QMI8658 (accelerometer, gyroscope)
- **Magnetometer**: QMC6310, BMM150
- **Touch**: CST226, CST92xx, GT911
- **Light**: CM32181, LTR553
- **RTC**: PCF8563, PCF85063
- **Haptic**: DRV2605 vibration motor

#### `components/System/`

System orchestration managing display, sensors, and OTA updates via FreeRTOS queues.

---

## Requirements

### Hardware

- **ESP32-S3** Waveshare board ([guide](https://www.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-1.75))
- SH8601 AMOLED display (466×466)
- CST226/CST92xx touch controller
- Optional: QMI8658 IMU sensor breakout

### Software

- **ESP-IDF** v5.0 or newer
- **Python** 3.7+ (installed by ESP-IDF)
- **SquareLine Studio** 1.6.0+ (for UI editing)
- **CMake** 3.16+

---

## Installation

### 1. Install ESP-IDF

```bash
git clone https://github.com/espressif/esp-idf.git
cd esp-idf
git checkout v5.1  # or latest stable
./install.sh
source ./export.sh
```

### 2. Clone the Repository

```bash
git clone https://github.com/CarlosAlmeida4/CarTouchScreenMiniDisplay_ESP32S3.git
cd CarTouchScreenMiniDisplay_ESP32S3
```

### 3. Configure Project

```bash
idf.py set-target esp32s3
idf.py menuconfig
```

Key settings to verify:

- **Partition scheme**: Choose one that supports OTA (e.g., "Two OTA app slots")
- **PSRAM**: Enable if connected
- **Freq**: 240MHz recommended

### 4. Build & Flash

```bash
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor  # Linux/Mac
idf.py -p COM3 flash monitor          # Windows
```

---

## Building the UI

To modify the user interface:

1. Open `SquarelineProject/CarTouchScreenMiniDisplay.slvs` in **SquareLine Studio**
2. Edit screens/components as needed
3. Export to `components/ui/` (GCC C project format)
4. Rebuild the ESP-IDF project

⚠️ **Note**: Auto-generated UI files should not be manually edited. Make all UI changes in SquareLine Studio.

---

## Configuration

Key files:

- [`sdkconfig`](sdkconfig) - ESP-IDF configuration
- [`partitions.csv`](partitions.csv) - Flash partition layout for OTA
- [`components/System/System.hpp`](components/System/System.hpp) - System integration

---

## Contributing

Contributions are welcome! Areas for enhancement:

- Additional sensor support
- Improved OTA update mechanisms
- Performance optimizations
- Additional UI screens

---

## License

This project contains multiple licenses:

- Project code: MIT License
- Fonts: SIL Open Font License (OFL) - see `SquarelineProject/assets/*/OFL.txt`
- LCD driver component: Apache 2.0 License

See individual files for copyright information.

---

## Resources

- [LVGL Documentation](https://docs.lvgl.io/)
- [ESP-IDF Official Docs](https://docs.espressif.com/projects/esp-idf/)
- [SquareLine Studio](https://www.squareline.io/)
- [Waveshare ESP32-S3 AMOLED](https://www.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-1.75)
