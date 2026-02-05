# ESP32 Matter Environmental Sensor

A compact, Matter-enabled environmental monitoring module built around the XIAO ESP32-C6 and Bosch BME680 sensor.  
This project demonstrates how to build a low-power, Wi-Fi or Thread-capable air quality sensor that integrates seamlessly into Matter-compatible ecosystems such as Apple Home, Google Home, and Home Assistant.

---

## 📦 Overview

The ESP32 Matter Environmental Sensor measures **temperature**, **humidity**, **pressure**, and **indoor air quality (IAQ)** using the Bosch BME680 environmental sensor and the BSEC2 library for accurate, compensated readings.  
An optional OLED display provides real-time readouts, while Matter support enables direct commissioning with any compatible ecosystem controller.

This project is designed for simple assembly, minimal wiring, and reliable 24/7 operation when powered from a compact USB charger.

---

## 🧰 Hardware

| Component | Description / Notes |
|------------|--------------------|
| **Seeed Studio XIAO ESP32-C6** | Main MCU; supports Wi-Fi, Thread, and BLE. Chosen for its small footprint and native Matter compatibility. |
| **Bosch BME680** | Environmental sensor measuring temperature, humidity, pressure, and gas/IAQ; connected via I²C. |
| **Apple 5W USB Charger** | Reliable, low-noise 5 V supply for continuous operation. |
| **USB-A Male Plug** | Interfaces the charger output to the device enclosure. |
| **USB-C Male Plug** | Connects to the XIAO board for power and flashing. |
| **Optional: SH1106 OLED Display (1.3")** | Displays temperature, humidity, pressure, and IAQ values locally. Connected via I²C (address `0x3C`). |

> ⚙️ *Future versions may include a VEML7700 ambient light sensor for automatic display dimming.*

---

## 🧩 Firmware & Tooling

The firmware is built with **ESP-IDF** and the **Arduino-ESP32** component, using Espressif’s **Matter** support and Bosch’s **BSEC2** library for compensated BME680 readings.

**Core Features**
- Matter over Wi-Fi or Thread
- Thread router-eligible configuration for mains-powered operation
- BME680 via BSEC2 for accurate IAQ, temperature, humidity, and pressure (s-IAQ for stationary use)
- Periodic state saving (`setState()` / `getState()`) every hour for stable IAQ tracking
- Matter Time Synchronization for local time (initial sync at commission; after reboot, sync can take up to ~10 minutes)
- OLED visualization (optional)
- Serial diagnostics (including Thread role and router eligibility)
- OLED auto-detection (prints a one-time notice if no display is found)

**Key Components**
- `managed_components/espressif__arduino-esp32` (Arduino core)
- `managed_components/espressif__esp_matter` (Matter stack)
- `components/bsec2` + `components/BME68x_Sensor_library`
- `components/oled_ssd1306_tp` (optional OLED)
- `components/qrcodeoled` (commissioning QR display)
- `components/esp32_matter_extra_endpoints` (additional Matter endpoints)

**Build & Flash**
- `idf.py set-target esp32c6`
- `idf.py build`
- `idf.py flash`
- `idf.py monitor` (watch commissioning output and Thread role)

**Hardware Flashing Guide**

App-only (device already has bootloader + partition table):
```
python -m esptool --chip esp32c6 -b 460800 --before default_reset --after hard_reset \
  write_flash 0x30000 build/WALL-Env_Sensor.bin
```

Full flash (first-time flash or after partition changes):
```
python -m esptool --chip esp32c6 -b 460800 --before default_reset --after hard_reset \
  write_flash 0x0 build/bootloader/bootloader.bin \
             0x8000 build/partition_table/partition-table.bin \
             0x30000 build/WALL-Env_Sensor.bin
```

Single image (merge bootloader + partition table + app):
```
python -m esptool --chip esp32c6 merge_bin -o build/full_flash.bin \
  0x0 build/bootloader/bootloader.bin \
  0x8000 build/partition_table/partition-table.bin \
  0x30000 build/WALL-Env_Sensor.bin
```

**Partitioning**
- Custom partition table disables OTA (single factory app slot) to free space for data/NVS.

**Device Identity IDs**
- Device vendor/product IDs and software/hardware versions are configured via `main/chip_project_config.h`.
- Update `CHIP_DEVICE_CONFIG_DEVICE_VENDOR_NAME`, `CHIP_DEVICE_CONFIG_DEVICE_PRODUCT_NAME`,
  `CHIP_DEVICE_CONFIG_DEVICE_SOFTWARE_VERSION_STRING`, and
  `CHIP_DEVICE_CONFIG_DEFAULT_DEVICE_HARDWARE_VERSION_STRING` as needed.

---

## 🧱 3D Printed Enclosure

A simple, vented enclosure designed for passive airflow and unobtrusive wall or outlet mounting.

**Files**
- `enclosure/Main Housing - Main Housing.stl`
- `enclosure/Solid Lid - Lid - Solid.stl`
- `enclosure/1.3- Screen Lid - Hosyond - Lid - OLED.stl` *(optional display version)*
- `enclosure/1.3- Screen Lid - Hosyond - OLED Retaining Bracket.stl`

**Notes**
- The case design accommodates the XIAO ESP32-C6, BME680 breakout, and optional SH1106 OLED.
- Optimized for front-facing airflow and USB-powered mounting directly on a 5 W charger.
- Printed in PLA or PETG recommended for indoor use.

---

## 🔌 Assembly & Wiring

Connect the BME680 (and optional OLED) to the XIAO ESP32-C6 over I²C. The firmware defaults to `SDA_PIN=22` and `SCL_PIN=23` (adjust in `main/MainSensor.cpp` if your board uses different pins).

| XIAO ESP32-C6 Pin | Signal | BME680 Pin | OLED Pin (optional) | Notes |
|--------------------|---------|-------------|----------------------|-------|
| **GPIO22 (D4)** | SDA | SDA | SDA | I²C data line |
| **GPIO23 (D5)** | SCL | SCL | SCL | I²C clock line |
| **3V3** | 3.3 V | VIN | VCC | Power supply |
| **GND** | Ground | GND | GND | Common ground |

**Steps**
1. Wire the BME680 (and OLED) to the XIAO as shown above.  
2. Build and flash via USB-C using ESP-IDF.  
3. Power the unit using the Apple 5 W charger with USB-A → USB-C adapter.  
4. On startup, scan the QR code shown on the OLED (or via serial output) to commission the device via a Matter controller.

---

## 🧠 Future Enhancements

- Additional Thread diagnostics and routing metrics  
- OTA updates  
- Ambient light sensor integration for display dimming  
- Expanded Matter cluster support (e.g., VOC index, CO₂, etc.)

---

## 📜 License

MIT License © 2025 [CP Waite](https://github.com/charles-waite)

---

## 🖼️ Preview

*(Add photos or renders here)*  
- Device mounted on charger  
- OLED displaying environmental data  
- Matter commissioning QR code  
- Enclosure render or assembly diagram

---
