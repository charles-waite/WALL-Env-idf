# WALL-Env ESP-IDF Environmental Sensor

A compact, Matter-over-Thread environmental monitoring module built around ESP32-C6 boards and a Bosch BME680 sensor.

This repository is the native ESP-IDF + esp-matter rewrite of the earlier Arduino/ESP-IDF WALL-Env firmware. It is not a fork of that project. The goal is a cleaner firmware stack with direct ownership of Matter behavior, predictable endpoint definitions, board-specific build outputs, OTA packaging, and less Arduino-layer coupling.

---

## 📦 Overview

WALL-Env measures indoor environmental conditions using a Bosch BME680 with Bosch BSEC2 compensation. The active firmware publishes temperature, relative humidity, air quality, CO2 equivalent, and TVOC through Matter over Thread.

An optional 1.3" SH1106 OLED shows commissioning and local status. When the device is uncommissioned, the OLED displays the Matter QR code only. After commissioning, it alternates between a Kirby bitmap screen and sensor data. Headless devices are supported; OLED initialization failures are logged but do not stop Matter or sensor operation.

The firmware is Thread-only. BLE is used for Matter commissioning, and operational traffic runs over Thread.

---

## 🧰 Hardware

| Component | Description / Notes |
|------------|--------------------|
| **Seeed Studio XIAO ESP32-C6** | Primary board target. Uses 4 MB flash and default XIAO-style I2C pins. |
| **Generic ESP32-C6 Supermini** | Secondary board profile with separate I2C pins and Product ID. |
| **Bosch BME680 / BME68x** | Environmental sensor used with BSEC2 for compensated temperature, humidity, s-IAQ, eCO2, and bVOC/TVOC. Connected over I2C at `0x77`. |
| **Optional: SH1106 OLED Display (1.3")** | Local display connected over I2C at `0x3C`. Current driver assumes a 128x64 panel with a 2-pixel SH1106 column offset. |
| **USB power source** | Designed for always-on USB power. The firmware configures the node as Thread router-capable. |

**Default XIAO wiring**

| XIAO ESP32-C6 Pin | Signal | BME680 Pin | OLED Pin (optional) | Notes |
|--------------------|---------|-------------|----------------------|-------|
| **GPIO22 / D4** | SDA | SDA | SDA | I2C data line |
| **GPIO23 / D5** | SCL | SCL | SCL | I2C clock line |
| **3V3** | 3.3 V | VIN | VCC | Power supply |
| **GND** | Ground | GND | GND | Common ground |
| **GPIO9** | Button | - | - | Factory reset button on the XIAO profile |

**Supermini profile**

The Supermini overlay uses:

```text
SDA = GPIO0
SCL = GPIO1
Factory reset button disabled
Product ID = 0x8001
```

---

## 🧩 Firmware & Tooling

The firmware uses native **ESP-IDF**, Espressif **esp-matter**, OpenThread, and Bosch **BSEC2**. Arduino is not used.

**Core features**

- Matter over Thread, with BLE commissioning.
- Thread router-capable configuration for mains-powered operation.
- BME680 via BSEC2 in low-power mode.
- Temperature and humidity on dedicated Matter sensor endpoints.
- Air Quality, CO2 concentration, and TVOC concentration on a dedicated Air Quality endpoint.
- Matter Time Synchronization cluster on endpoint 0.
- Matter OTA Requestor with dual OTA app partitions and rollback validation.
- BSEC2 state persistence in NVS after IAQ accuracy is high enough.
- Persistent temperature calibration profiles set from serial.
- Optional SH1106 OLED UI with commissioning QR code, sensor page, Kirby screen, and quiet-hours display-off behavior.
- Serial diagnostics and commands over USB Serial/JTAG.

**Key project files**

| Path | Purpose |
|------|---------|
| `main/app_main.cpp` | Matter node setup, endpoint creation, Thread setup, OTA requestor, serial commands. |
| `main/drivers/bsec2_app.cpp` | BSEC2 startup, BME680 I2C glue, BSEC state persistence, Matter sensor publishing, temperature profiles. |
| `main/drivers/oled_sh1106.cpp` | SH1106 OLED UI, QR rendering, time display, quiet-hours display control, Kirby/sensor rotation. |
| `main/chip_project_config.h` | Matter vendor/product identity and software/hardware version strings. |
| `wall_env.zap` | Local ZAP data model source. |
| `main/zap-generated/` | Generated Matter data model artifacts. |
| `sdkconfig.defaults` | Primary XIAO defaults and shared Matter/Thread/OLED settings. |
| `sdkconfig.boards/supermini.defaults` | Supermini board overlay. |
| `partitions.csv` | OTA-capable 4 MB ESP32-C6 partition table. |

**Managed and local components**

- `managed_components/espressif__esp_matter`
- `managed_components/espressif__button`
- `components/bsec2`
- `components/BME68x_Sensor_library`
- `components/app_reset`

---

## 🧱 3D Printed Enclosure

The enclosure files are kept in `enclosure/`.

**Current files**

- `enclosure/Main Housing - Main Housing.stl`
- `enclosure/Solid Lid - Lid - Solid.stl`

The OLED case lid and OLED retaining bracket designs are still a work in progress and are intentionally not treated as final release assets yet.

---

## 🔌 Build, Flash, Monitor

Prefer the repository wrapper scripts. They set the ESP-IDF environment, use the correct build directories, enable ccache when available, add heartbeat logging for long builds, and avoid overlapping `idf.py` runs.

**XIAO ESP32-C6**

```sh
tools/set-target-esp32c6.sh
tools/build.sh
tools/idf.sh flash
tools/idf.sh monitor --no-reset
```

The XIAO build output is:

```text
builds/xiao_esp32c6/wall_env_idf.bin
```

**Generic ESP32-C6 Supermini**

```sh
tools/set-target-esp32c6-supermini.sh
tools/build-supermini.sh
tools/flash-supermini.sh
```

The Supermini build output is:

```text
builds/supermini_esp32c6/wall_env_idf.bin
```

**Flash an already-built image without rebuilding**

```sh
tools/idf.sh flash-bin
```

This uses the existing build artifacts and skips CMake/Ninja rebuild checks.

---

## 📡 Matter, Thread, and Endpoints

The device is commissioned over BLE and then operates over Thread. Wi-Fi station/AP support is disabled in project defaults.

The active runtime sensor endpoint layout is:

| Endpoint | Device / Purpose | Main clusters |
|----------|------------------|---------------|
| `0` | Root Node | Basic Information, Descriptor, Access Control, Time Synchronization, OTA Requestor support. |
| `1` | Temperature Sensor | Temperature Measurement. |
| `2` | Humidity Sensor | Relative Humidity Measurement. |
| `3` | Air Quality Sensor | Air Quality, Carbon Dioxide Concentration Measurement, TVOC Concentration Measurement. |

ZAP artifacts live in `wall_env.zap` and `main/zap-generated/`. Runtime code still creates the sensor endpoints explicitly where that has produced the best controller behavior, especially in Apple Home and Home Assistant.

The firmware prints onboarding QR and manual pairing codes on every boot. If an OLED is present and the device is uncommissioned, the QR code is shown on the display.

---

## 🕒 Time and Display Behavior

Matter Time Synchronization is exposed on endpoint 0. Time can take several minutes to arrive after a cold boot or recommissioning, so the OLED treats early epoch values as unsynced.

OLED behavior:

- Boot splash for `CONFIG_WALL_ENV_OLED_SPLASH_MS` milliseconds.
- QR-only screen while uncommissioned.
- After commissioning, 25 seconds of Kirby followed by 5 seconds of sensor data.
- Time/date line on the sensor page.
- Unsynced time is marked visually until Matter time sync has populated the system clock.
- Quiet hours turn the display off from `CONFIG_WALL_ENV_OLED_QUIET_HOUR_START` to `CONFIG_WALL_ENV_OLED_QUIET_HOUR_END`.

Current default quiet hours:

```text
Off: 22:00 local time
On:  06:00 local time
```

---

## 🧪 Sensor Calibration and State

BSEC2 state is restored from NVS on boot and saved periodically after IAQ accuracy reaches the configured threshold. This helps IAQ and gas-derived outputs recover faster after normal reboots.

Temperature calibration is selected through persistent serial profiles:

```text
profile v1
profile v2
profile custom <offsetC>
```

The selected profile survives normal reboot, firmware flash, and Matter decommissioning. It is cleared by erase-flash.

---

## ⌨️ Serial Commands

Use `tools/idf.sh monitor --no-reset` from an interactive terminal, then type commands into the serial monitor.

```text
help
decom
decommission
factoryreset
reset
profile
profile v1
profile v2
profile custom <offsetC>
thdiag
threaddiag
```

`decom` removes Matter fabrics and schedules a reboot so stale operational sessions and discovery state are cleared.

---

## 🔄 OTA and Release Packaging

The current partition table uses dual OTA slots on 4 MB ESP32-C6 modules:

```text
ota_0: 0x20000,  0x1E0000
ota_1: 0x200000, 0x1E0000
```

Build Matter OTA images with:

```sh
tools/build-ota.sh xiao
tools/build-ota.sh supermini
```

The OTA script:

- Builds the selected board profile.
- Checks app size against the OTA slot with a 10% free-space margin.
- Wraps the app binary with the Matter OTA image header.
- Emits the `.ota` file and Home Assistant Matter Server local OTA `.json`.
- Uses board-specific Product IDs so XIAO and Supermini updates do not cross-apply.

Package a full "flash one file" image with:

```sh
tools/package-release.sh xiao
tools/package-release.sh supermini
```

Release and OTA process notes live in:

- `docs/matter_ota_requestor.md`
- `docs/matter_ota_release_policy.md`

---

## 🆔 Device Identity

Matter identity values are configured through `main/chip_project_config.h`, `sdkconfig.defaults`, and board overlays.

Current development IDs:

| Target | Vendor ID | Product ID |
|--------|-----------|------------|
| XIAO ESP32-C6 | `0xFFF1` | `0x8000` |
| Generic ESP32-C6 Supermini | `0xFFF1` | `0x8001` |

`0xFFF1` is a test Vendor ID. Do not use the development VID/PIDs for production distribution.

Version source of truth is `CMakeLists.txt`:

```text
PROJECT_VER
PROJECT_VER_NUMBER
```

The Matter software version number uses:

```text
major * 10000 + minor * 100 + patch
```

---

## 🗂️ Repository Layout

| Path | Contents |
|------|----------|
| `main/` | App logic, Matter integration, generated ZAP sources, OLED and sensor drivers. |
| `components/` | Local BSEC2/BME68x/app reset components. |
| `managed_components/` | ESP-IDF component manager dependencies. |
| `tools/` | Build, flash, target, OTA, release, version, and ZAP helper scripts. |
| `docs/` | OTA release policy and requestor runbook. |
| `reference/` | Datasheets, board pinouts, legacy notes, and diagnostics references. |
| `enclosure/` | Current STL enclosure assets. |
| `builds/` | Board-specific build outputs. |

---

## 📜 License

MIT License © 2025 [CP Waite](https://github.com/charles-waite)
