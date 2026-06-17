# Suggested Commands

- Activate ESP-IDF manually only when needed: `source ~/esp-idf/export.sh`.
- Default XIAO build: `tools/build.sh`.
- Supermini build: `tools/build-supermini.sh`.
- Revisioned builds: `tools/build-revision.sh v2`, `tools/build-revision.sh v1`, or `tools/build-revision.sh --all`.
- OTA package build: `tools/build-ota.sh xiao` or `tools/build-ota.sh supermini`.
- XIAO flash: `tools/flash.sh`; erase plus flash: `tools/flash.sh erase-flash flash`.
- Supermini flash: `tools/flash-supermini.sh`; erase plus flash: `tools/flash-supermini.sh erase-flash flash`.
- Monitor through wrapper to avoid raw `idf.py` creating default `build/`: `tools/idf.sh monitor --no-reset`.
- Regenerate ZAP artifacts: `./tools/zap_regen.sh`.
- Darwin file-size behavior appears in scripts; prefer existing wrappers over ad hoc commands.