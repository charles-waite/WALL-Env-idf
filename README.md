# WALL-Env ESP-IDF Environmental Sensor

### A compact, Matter-over-Thread environmental monitoring module built around ESP32-C6 boards and a Bosch BME680 sensor. 
It is compatible with both **Apple Home** and **Home Assistant** but requires a Thread-capable Matter controller in order to work. Apple TV 4K (2nd/3rd Gen Wi-Fi+Ethernet), HomePod (2nd Gen), and HomePod mini are all compatible.

This repository is a rewrite of the earlier [Arduino/ESP-IDF WALL-Env firmware](https://github.com/charles-waite/ESP32-Matter-Environmental-Sensor) using native ESP-IDF + esp-matter. The goal is a cleaner codebase with better Matter integration, support for Matter over Thread, board-specific build outputs, and OTA firmware update support.



## 📦 Overview

WALL-Env measures indoor environmental conditions using a Bosch BME680 with Bosch's BSEC2 library calculating accurate sensor data. The device publishes temperature, relative humidity, air quality, CO2 equivalent, and TVOC through Matter over Thread.

Optional support for a 1.3" SH1106 OLED connected via I2C shows commissioning and local status. When the device is uncommissioned, the OLED displays the Matter QR code for simple pairing to Apple Home or Home Assistant.

The firmware is Thread-only. BLE is used for Matter commissioning using an iPhone or iPad using Apple Home or the Home Assistant app. Sensor data and communication runs over Thread.



## 🧰 Hardware

| Component | Description / Notes |
|-----------------|---------------|
| **Seeed Studio XIAO ESP32-C6** | Primary board target. Uses 4 MB flash and default XIAO-style I2C pins. |
| *OR* | | 
| **Generic ESP32-C6 Supermini** | Secondary board profile with separate I2C pins and Product ID. |
| **Bosch BME680** | Environmental sensor used with BSEC2 for compensated temperature, humidity, s-IAQ, eCO2, and bVOC/TVOC. Connected over I2C at `0x77`. |
| **Optional: 1.3" OLED display"** | Local display connected over I2C at `0x3C`. Current driver assumes a 128x64 panel with a 2-pixel SH1106 column offset. |
| **Standard Apple 5w USB charger** | Designed for always-on USB power. The firmware configures the node as Thread router-capable. |

###Default XIAO wiring

| XIAO ESP32-C6 Pin | Signal |
|--------------------|---------|
| GPIO22 / D4 | SDA |
| GPIO23 / D5 | SCL |
| 3V3 | 3.3 V out |
| GND | Ground |
###Default Supermini wiring

| Supermini Pin | Signal |
|------------------|---------|
| GPIO0 | SDA |
| GPIO1 | SCL |
| 3V3 | 3.3 V out |
| GND | Ground |



##🧩 Firmware & Tooling

The firmware uses native **ESP-IDF**, Espressif **esp-matter**, OpenThread, and Bosch **BSEC2**. Arduino is not used.

**Core features**

- Matter over Thread, with BLE commissioning.
- Thread router-capable configuration for mains-powered operation.
- BME680 sensor providing accurate Temperature, Humidity, Air Quality, CO2 concentration, and TVOC measurements.
- Accuraacty compensation done via BSEC2 in low-power mode.
- Matter Time Synchronization for automated time sync.
- Partial Matter OTA support.
- BSEC2 compensation state saves to internal storage.
- Temperature calibration configurable via Serial.
- Optional 1.3" OLED UI with commissioning QR code, live sensor data, bitmap graphic support, time-based dimming and configurable display-off schedule.
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


## 🧱 3D Printed Enclosure

The enclosure files are kept in `enclosure/`. These STL files are intended to take a standard 5W Apple USB charger with an integrated mount for a male USB-A plug that can be wired to a small male usb-C plug to the board. Serial and firmware updates can be managed through this usb-A plug using a female to male adapter. The lids are designed to securely attach using 3x2mm neodymium magnets embedded in the case and lid itself.

**Current files**
Main enclosure body:
- `enclosure/Main Housing - Main Housing.stl`

- `enclosure/Solid Lid - Lid - Solid.stl`

> The OLED case lid and OLED retaining bracket designs are still a work in progress and are intentionally not treated as final release assets yet. Will be comleted and released very soon!



## 🔌 Build, Flash, Monitor

To aid in build and evnironment setup, there are a number of helper scripts included in the `tools/` directory. They set the ESP-IDF environment, use the correct build directories, enable ccache when available, add heartbeat logging for long builds, and avoid overlapping `idf.py` runs.

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
Screen Off: 10:00 PM local time
Screen On:  6:00 AM local time
```



## 🧪 Sensor Calibration and State

BSEC2 state is restored from NVS on boot and saved periodically after IAQ accuracy reaches the configured threshold. This helps IAQ and gas-derived outputs recover faster after normal reboots.

Temperature calibration is selected through persistent serial profiles:

```text
profile v1
profile v2
profile custom <offsetC>
```

**Profile v1** is deprecated and designed for the prior V1 enclosure design. 

**Profile v2** is *enabled by default* and is intended for the V2 enclosure design with better airflow and thus lower temperature compensation. 

**Profile Custom** allows you to set a custom offset. A positive integer results in a lower temperature reading. ie. `profile custom 5` will result in a raw measured temperature of 50°C to display as 45° C. The temperature offset will alter the BSEC compensation values for RH and IAQ across the board and is critical to overall accuracy.

The selected offset profile or custom value survives normal reboot, firmware flash, and Matter decommissioning. It is only cleared by erase-flash.

There is no significant difference in measured temperature with or without the OLED installed.




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

`decom` and `decommission` removes Matter fabrics and schedules a reboot so stale operational sessions and discovery state are cleared.



## 🔄 OTA Update and Release Packaging

The current partition table uses dual OTA slots on 4 MB ESP32-C6 modules and is the same for both the XIAO and generic Supermini Boards: 

```text
ota_0: 0x20000,  0x1E0000
ota_1: 0x200000, 0x1E0000
```

Build Matter OTA images with:

```sh
./tools/build-ota.sh xiao

./tools/build-ota.sh supermini
```

The OTA script:

- Builds the selected board profile.
- Checks app size against the OTA slot with a 10% free-space margin.
- Wraps the app binary with the Matter OTA image header.
- Emits the `.ota` file and Home Assistant Matter Server local OTA `.json`.
- Uses board-specific Product IDs so XIAO and Supermini updates do not cross-apply.

Package a full "flash one file" image with:

```sh
./tools/package-release.sh xiao
./tools/package-release.sh supermini
```

Release and OTA process notes live in:

- `docs/matter_ota_requestor.md`
- `docs/matter_ota_release_policy.md`



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


MIT License © 2025 [CP Waite](https://github.com/charles-waite)
