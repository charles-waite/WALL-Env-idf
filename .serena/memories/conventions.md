# Conventions

- Prefer repo wrappers over raw `idf.py`; raw commands can create default `build/` and bypass board-scoped paths.
- Keep board pin changes in defaults/overlays, especially `sdkconfig.defaults` and `sdkconfig.boards/*.defaults`; do not casually edit hardware defaults.
- Treat Matter endpoint/device-type work as ZAP/data-model aligned; runtime endpoint creation must match generated definitions.
- Keep identity metadata coherent across Matter config surfaces: vendor/product/model/version fields, onboarding presentation, and OTA headers.
- Use ESP-IDF style logging/timing (`ESP_LOGx`, `esp_timer`, FreeRTOS delays); keep helpers `static` and close to use.
- C/C++ style is 2-space indentation with braces on same line, matching existing project code.
- Commissioning anomalies should be considered stale NVS/fabric state first; validate with erase-flash before code changes.
- AirQuality semantics use BSEC static IAQ with accuracy gating; avoid drifting enum meaning.