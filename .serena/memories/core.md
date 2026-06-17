# Core

- Clean-room native ESP-IDF + ESP-Matter firmware for WALL-Env; not a fork of the earlier Arduino hybrid.
- Target hardware is ESP32-C6, primarily Seeed XIAO ESP32-C6; Supermini is supported via board defaults/wrappers.
- Thread-only device: BLE commissioning plus Thread transport; Wi-Fi is not a project feature even if ESP-IDF/CHIP dependencies expose Wi-Fi config symbols.
- Top-level app code lives in `main/`; local components include BSEC2/BME68x and app reset support in `components/`.
- Matter data model is owned locally through `wall_env.zap`, `wall_env.matter`, and generated code under `main/zap-generated/`.
- Project dependency source is ESP-IDF Component Manager (`managed_components/` ignored locally), including managed `espressif/esp_matter`.
- Read `mem:tech_stack` for build/dependency details, `mem:suggested_commands` for wrapper commands, `mem:conventions` for project invariants, and `mem:task_completion` before finishing code work.