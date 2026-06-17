# Task Completion

- For normal firmware code changes, run `tools/build.sh` and confirm `builds/xiao_esp32c6/wall_env_idf.bin` exists.
- For Supermini-impacting changes, also run `tools/build-supermini.sh` and confirm `builds/supermini_esp32c6/wall_env_idf.bin` exists.
- For revision/default changes, run the relevant `tools/build-revision.sh` target.
- For OTA/version changes, run `tools/build-ota.sh xiao` and/or `tools/build-ota.sh supermini`; version changes should go through `tools/update-version.sh`.
- For Matter data model changes, run `./tools/zap_regen.sh` and then build.
- On failures, report the first root-cause error with command, closest file:line, and a one-line fix hypothesis.
- Runtime validation requiring hardware remains manual: commissioning, Thread role stability, time sync, BSEC state restore, OLED layout, and serial `decom` behavior.