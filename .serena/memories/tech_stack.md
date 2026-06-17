# Tech Stack

- Firmware framework: ESP-IDF 5.5 era project using CMake/Ninja and ESP-IDF Component Manager.
- Main implementation language: C++17 with C support; compile flags include `-std=gnu++17`, `-Os`, and CHIP config defines in top-level CMake.
- Matter stack: managed `espressif/esp_matter`; local `esp-matter/` checkout is not required for normal build/flash/monitor.
- Protocols: BLE commissioning, Thread transport, Matter endpoints/clusters generated from ZAP artifacts.
- Local drivers/components: BSEC2/BME68x, SH1106 OLED, QR code support, app reset helper.
- Tooling languages in repo: Bash wrapper scripts in `tools/`, small Python utility scripts, YAML Component Manager manifests, JSON manifests/metadata, Markdown docs.
- Build outputs are ignored and board-scoped under `builds/`; `build/` may appear if raw `idf.py` is used.