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

## Session Log
- If desired, track major actions in `resources/codex/SessionLog.m`.
