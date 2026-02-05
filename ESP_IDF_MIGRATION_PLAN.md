# ESP-IDF Migration Plan (Arduino-ESPIDF → Full ESP-IDF + esp-matter)

Goal: replicate the current feature set in a **new repo** using full ESP-IDF + esp-matter, with ZAP-managed clusters and no Arduino layer.

This plan is written as a handoff for another Codex agent. It assumes ESP32‑C6 target and Thread-only (no Wi‑Fi).

---
## 0) Create New Repo + Baseline

1. Create a new repo (e.g., `ESP32-Matter-EnvSensor-IDF`).
2. Initialize with ESP-IDF + esp-matter base:
   - Use an esp-matter example closest to a sensor device (e.g., `esp_matter` + minimal app).
   - Confirm `idf.py set-target esp32c6`.
   - Build once: `idf.py build`.
3. Copy over non-code assets you want to keep (README snippets, enclosure files, wiring notes).

Deliverable: New repo builds and flashes a minimal esp-matter app.

---
## 1) ZAP + Data Model Ownership

1. Create a `*.zap` file in the new repo:
   - Endpoints:
     - Temp Measurement
     - Relative Humidity
     - Pressure
     - Air Quality (IAQ enum)
     - CO2 Concentration (CO2eq)
   - Root endpoint: add Time Synchronization server cluster.
2. Generate outputs with Matter ZAP (connectedhomeip tools):
   - Generate into `main/zap-generated` or `components/zap-generated`.
3. Wire the build to include generated sources.

Deliverable: ZAP-generated sources compile and define your endpoints/clusters.

---
## 2) Hardware Bring-Up (ESP-IDF Native)

Replace Arduino APIs with ESP-IDF equivalents.

### I2C
- Replace `Wire` with ESP-IDF I2C driver (`driver/i2c_master.h`).
- Set SDA/SCL pins to 22/23, 400kHz.

### Serial Logging
- Replace `Serial` with `ESP_LOGx` or `printf`.

### Timing
- Replace `millis()` with `esp_timer_get_time()` (microseconds).
- Replace `delay()` with `vTaskDelay()`.

Deliverable: I2C is working, and you can read from the BME680 driver.

---
## 3) BME680 + BSEC2

1. Bring over `components/bsec2` and `components/BME68x_Sensor_library` (or add as managed components).
2. Port the BSEC2 setup:
   - Subscribe to:
     - `BSEC_OUTPUT_STATIC_IAQ`
     - `BSEC_OUTPUT_CO2_EQUIVALENT`
     - `BSEC_OUTPUT_BREATH_VOC_EQUIVALENT` (optional)
     - `BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE`
     - `BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY`
     - `BSEC_OUTPUT_RAW_PRESSURE`
   - Sample rate: `BSEC_SAMPLE_RATE_LP`
3. NVS state save/restore:
   - Use NVS APIs directly (`nvs_open`, `nvs_get_blob`, `nvs_set_blob`).
   - Save interval: 1 hour; save only if `acc >= 2`.
   - Restore at boot; log `[BSEC2] state restored`/`[BSEC2] state saved`.

Deliverable: Sensor data is stable, and BSEC state persists across reboots.

---
## 4) Matter Endpoint Updates

1. Wire sensor outputs to Matter attributes:
   - Temperature: °C
   - RH: %
   - Pressure: hPa (sea-level corrected if enabled)
   - Air Quality enum: map s-IAQ to Matter AQ enum (Good/Fair/Moderate/Poor/VeryPoor/ExtremelyPoor).
   - CO2: use eCO2 from BSEC.
2. Ensure `set` calls are using the correct cluster attributes.
3. Keep `airQualityEnum` as Unknown until IAQ accuracy >= 2.

Deliverable: Values visible in controller and update every interval.

---
## 5) OLED UI (Optional but Desired)

1. Move OLED driver to an ESP-IDF compatible library:
   - If the SH1106 driver is Arduino-only, replace with an ESP-IDF SH1106 driver.
2. Re-create current UI layout:
   - Line 1 (16pt): `Temp` + `RH`
   - Line 2 (10pt): `IAQ: <Label> (<value>,<acc>)`
   - Line 3 (10pt): `CO2: ####ppm`
   - Line 4 (10pt): `Pressure: ##.##inHg <trend>`
   - 1-character left indent for all rows
3. Kirby screensaver:
   - Alternate sensor view and Kirby bitmap every 30s.
4. Brightness control:
   - Dim at sunset, full at sunrise using local sun calculations.

Deliverable: OLED matches current UI + Kirby screen swap.

---
## 6) Matter Time Sync

1. Add Time Sync cluster in ZAP on endpoint 0.
2. Use a simple time sync module:
   - Create cluster after `Matter.begin()`.
   - Poll until UTC becomes valid.
   - Ignore Last Known Good time (only accept real SetUTCTime).
3. Use time to:
   - Render Local + UTC + Next event in serial logs.
   - Compute sunrise/sunset for brightness dimming.

Deliverable: time updates after boot (typically 5–15 minutes).

---
## 7) Thread + Commissioning

1. Enable Thread router eligibility:
   - `otThreadSetRouterEligible(true)`
   - `mRxOnWhenIdle = true`
2. Verify commissioning flow is consistent with current project:
   - If not commissioned: show QR once.
   - If commissioned: show sensor UI.
3. Ensure `idf.py monitor` does not reset device (`--no-reset` if needed).

Deliverable: Device commissions reliably and stays on Thread.

---
## 8) Partition Table

1. Remove OTA slots (single app, larger NVS if desired).
2. Keep NVS + keys + phy.

Deliverable: No OTA, larger NVS for BSEC state.

---
## 9) Regression Checklist (Manual Tests)

- Commissioning succeeds.
- Matter endpoints update with live values.
- s-IAQ maps to Air Quality enum.
- BSEC state saves/restores (acc jumps to 2–3 after reboot).
- OLED layout matches current build.
- Kirby alternates every 30s.
- Time sync eventually updates; logs show Local/UTC + Next event.

---
## Notes / Gotchas

- Matter time sync can take 10+ minutes after reboot.
- `idf.py monitor` toggles reset; use `--no-reset` to avoid.
- After partition changes, use `idf.py erase-flash`.
- If commissioning is slow, ensure Time Sync cluster is created after Matter init.

