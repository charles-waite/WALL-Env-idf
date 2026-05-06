# Repository Guidelines (WALL-Env_ESP-IDF)

## Purpose
This repository is a **clean-room ESP-IDF rewrite** of the WALL-Env sensor firmware. It is **not** a fork. The goal is to replace the Arduino-ESP-IDF hybrid with a native ESP-IDF + esp-matter implementation for better ZAP ownership, predictable cluster behavior, and fewer Arduino quirks.

## Hardware/Protocols Used
- Seeed XIAO ESP32-C6
  - 4mb Flash - Custom partition table (no OTA; more NVS/data space)
  - I2C devices connected on standard SDA & SCL (SDA=GPIO22/D4, SCL= GPIO23/D5)
  - Boot button = GPIO9
- 1.3" OLED display (Generic)
  - I2c addr=0x3C
- BME680 sensor board
  - I2C addr=0x77
- Thread-only. No Wifi.
- Commissioning over BLE w/QR-Code or commissioning code.

## Feature Targets (parity with current device)
- BME680 sensor with BSEC2 (LP profile)
  - Outputs:
     - Temperature (heat-compensated)
     - Relative Humidity (heat-compensated)
     - Pressure (raw → sea-level option)
     - s-IAQ (Static IAQ) + accuracy
     - eCO2 (CO2 equivalent)
  - BSEC2 state persistence in NVS (hourly save; restore on boot)
- Matter Endpoints
	- General data, device attributes, and time sync on Endpoint 0
	- Sensor data on Endpoint 1
- Matter Clusters:
  - Temperature Measurement
  - Relative Humidity Measurement
  - Pressure Measurement
  - Air Quality (enum from s-IAQ)
  - CO2 Concentration (from eCO2)
  - Time Synchronization cluster on endpoint 0
- OLED UI (SH1106):
  - +2px horizontal offset (default in SH1106 driver libraries)
  - Boot screen displays for 5 seconds on each boot
  - Display Matter Commissioning QR-code when uncommissioned only. Switch to sensor/kirby display after commissioning fully complete.
     - Always print QR-code URL and manual pin code in serial on boot. 
  - 16pt Temp + RH row
  - 10pt IAQ line with label and accuracy
  - 10pt CO2 line
  - 10pt Pressure line with trend arrow
  - 1-char left indent on all text rows (for aesthetic reasons)
  - Kirby bitmap screensaver alternating every 30s
  - Automatic dimming (setContrast=1) at sunset, brightening at sunrise (setContrast=255). 
- Time sync (Matter over Thread):
  - Time sync can take 5–15 minutes after boot
  - Use sunrise/sunset for OLED dimming
  - Serial logs include Local + UTC + next event
  - Timezone is PDT/PST using daylight savings corrections (zip code 98103 if that helps location awareness).
  - Can store device time in UTC but convert to local time for display if that simplifies Matter integration.
- Matter/Thread Features
	- Thread router eligibility (RX-on-when-idle)
	- Full End Device
	- User/human (me) will use zaptool to configure endpoints and clusters
- Serial inputs via keyboard
  - type "decom" in serial monitor to force Matter decommissioning and reset device 

## Project Structure (proposed)
- `main/` — app logic, Matter integration, UI, sensor pipeline
- `components/`
  - `bsec2` and BME68x driver
  - OLED driver (ESP-IDF compatible)
- `enclosure/` — STL files (copy from original repo)
- `resources/` — diagrams, datasheets, notes
- `docs/` — design notes, wiring, commissioning steps

## Build / Flash
- `idf.py set-target esp32c6`
- `idf.py build`
- `idf.py flash`
- `idf.py monitor` (use `--no-reset` if needed)

## ZAP / Data Model
- Own the data model via a local `.zap` file.
- Generate `zap-generated` into `main/` or a component.
- Use ZAP-managed clusters rather than Arduino prefab endpoints.

## Matter Conformance Defaults
- Align endpoint/device-type creation with the latest Matter spec and current ESP-Matter/CHIP SDK artifacts; do not rely on stale IDs from memory.
- Prefer ZAP-managed endpoint/cluster definitions for canonical IDs and revisions; keep runtime endpoint creation aligned with those generated definitions.
- Populate professional device identity metadata (vendor name, product name, model, software/hardware version, serial) in project config and Basic Information attributes.
- Keep Vendor ID/Product ID and project identity coherent across `main/chip_project_config.h`, `sdkconfig.defaults`, and onboarding/commissioning presentation.
- Treat placeholder/test identifiers as temporary only; if production values are unknown, explicitly mark them as TODO and do not silently invent.
- For Matter-facing changes, verify exposed endpoint IDs, device types, and core identity attributes on boot logs before finalizing.

## Coding Style
- Use ESP-IDF conventions (ESP_LOGx, esp_timer, FreeRTOS delays)
- Keep helpers `static` and close to usage
- 2-space indentation, braces on same line (match original style)

## Testing Checklist (manual)
- Commissioning succeeds (QR + manual code if needed)
- Thread role is stable and router eligible
- Time sync eventually updates (after reboot)
- BSEC state saves/restores (acc >= 2 quickly after reboot)
- OLED layout matches spec

## Notes / Gotchas
- Matter time sync can be delayed; initial time may be 2000/2001 until sync.
- `idf.py monitor` toggles reset; use `--no-reset` when diagnosing time sync.
- Partition table changes require `idf.py erase-flash`.


## Common Mistakes To Avoid
- Prefer wrapper scripts (`tools/idf.sh`, board `set-target`/`build` wrappers) over raw `idf.py` unless wrappers are missing.
- Do not change board pins in source defaults when board overlays exist; use `sdkconfig.boards/*.defaults`.
- Treat commissioning anomalies as possible stale NVS/fabric state first; validate with `erase-flash` before code changes.
- Keep AirQuality enum semantics consistent with project intent (WALL-Env uses `BSEC_OUTPUT_STATIC_IAQ` + accuracy gate).
- Do not casually edit `sdkconfig.defaults`, `sdkconfig.revisions/*.defaults`, `partitions.csv`, or `idf_component.yml` files without explicit request.
- For failures, report the first root-cause error with `file:line`, exact command, and one-line fix hypothesis.

## Codex ESP-IDF Workflow
- Prefer repo wrappers over raw `idf.py`: use `/Users/cwaite/Documents/WALL-Env-idf/tools/idf.sh` (it sets ESP-IDF + esp-matter env and serial-port fallback).
- Canonical target default is `esp32c6`; set with `/Users/cwaite/Documents/WALL-Env-idf/tools/set-target-esp32c6.sh`.
- Canonical build command is `/Users/cwaite/Documents/WALL-Env-idf/tools/build.sh`.
- Canonical flash command is `/Users/cwaite/Documents/WALL-Env-idf/tools/idf.sh flash`.
- Canonical monitor command is `/Users/cwaite/Documents/WALL-Env-idf/tools/idf.sh monitor --no-reset` for timing/commissioning diagnostics.
- Revisioned build default is `v2`; use `/Users/cwaite/Documents/WALL-Env-idf/tools/build-revision.sh v2` (or no arg for timed prompt defaulting to `v2`).
- Optional board overlay exists for Supermini: set target with `/Users/cwaite/Documents/WALL-Env-idf/tools/set-target-esp32c6-supermini.sh` and build with `/Users/cwaite/Documents/WALL-Env-idf/tools/build-supermini.sh`.
- Board/pin defaults for primary profile come from `/Users/cwaite/Documents/WALL-Env-idf/sdkconfig.defaults` (XIAO-style I2C pins `SDA=22`, `SCL=23`, reset button `GPIO9`).
- Do not change these casually unless explicitly requested: `/Users/cwaite/Documents/WALL-Env-idf/sdkconfig.defaults`, `/Users/cwaite/Documents/WALL-Env-idf/sdkconfig.revisions/*.defaults`, `/Users/cwaite/Documents/WALL-Env-idf/sdkconfig.boards/*.defaults`, `/Users/cwaite/Documents/WALL-Env-idf/partitions.csv`, `/Users/cwaite/Documents/WALL-Env-idf/main/idf_component.yml`, `/Users/cwaite/Documents/WALL-Env-idf/idf_component.yml`.
- Verification command after changes: `/Users/cwaite/Documents/WALL-Env-idf/tools/build.sh` (or board-specific build), then confirm `/Users/cwaite/Documents/WALL-Env-idf/builds/xiao_esp32c6/wall_env_idf.bin` exists.
- Failure reporting expectation: report the first root-cause compiler/runtime error with exact `file:line` (or closest location), the command run, and a one-line fix hypothesis before listing follow-on errors.

## Session Log
- If desired, track major actions in `resources/codex/SessionLog.m`.
