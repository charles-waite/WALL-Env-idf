# WALL-Env project instructions

## Scope and accepted decisions

- Native ESP-IDF + esp-matter rewrite of WALL-Env. Do not reintroduce Arduino
  dependencies into the application.
- Intended distribution: public source and enclosure designs, with optional
  low-volume hobbyist assembly sales. Formal Matter/Thread certification is
  outside the current scope. Prioritize practical security, reliability,
  interoperability, and testable improvements.
- Apply global working and privacy preferences except for the explicit
  project exceptions below. Historical feature targets are not instructions
  to restore obsolete behavior.

## Hardware and configuration

- Default board: Seeed XIAO ESP32-C6, 4 MB flash. I2C SDA=GPIO22, SCL=GPIO23;
  boot/reset button=GPIO9. BME680 at 0x77; optional SH1106 OLED at 0x3C.
- Matter over Thread with BLE commissioning; Wi-Fi remains disabled.
  Preserve router eligibility and RX-on-when-idle behavior.
- Board-specific pins belong in sdkconfig.boards/ overlays; revision settings
  belong in sdkconfig.revisions/. Preserve the selected board and revision.
- Change sdkconfig.defaults, board/revision overlays, partitions.csv, or
  idf_component.yml files only when the requested work explicitly covers
  those settings. Do not use configuration changes as speculative fixes.

## Architecture and implementation references

- main/app_main.cpp: Matter endpoints, networking, identity, OTA integration,
  and serial commands.
- main/drivers/bsec2_app.cpp: sensor processing, calibration, Matter reporting,
  and BSEC state persistence.
- main/drivers/oled_sh1106.cpp and main/Kconfig.projbuild: display behavior
  and configurable timings. Preserve current QR gating, rotation, time
  validity handling, quiet hours, and dimming unless explicitly changed.
- wall_env.zap and main/zap-generated/: ZAP model and generated artifacts.
- README.md: setup and feature documentation. OTA procedures are in
  docs/matter_ota_requestor.md and docs/matter_ota_release_policy.md.

Accepted Matter architecture:

| Endpoint | Purpose |
|----------|---------|
| 0 | Root node, time synchronization, and OTA support |
| 1 | Temperature measurement |
| 2 | Humidity measurement |
| 3 | Air quality, eCO2, and TVOC |

- Runtime sensor endpoint creation and ZAP artifacts are both intentional.
  Preserve this arrangement and its controller compatibility behavior;
  do not consolidate sensors onto endpoint 1 or impose a ZAP-only model.
- The user owns ZAP model decisions. For authorized model changes, keep
  runtime definitions and generated artifacts consistent. Regenerate with
  tools/zap_regen.sh, preserving its SDK compatibility handling rather than
  manually patching generated files.
- Resolve Matter IDs, revisions, and APIs against the SDK version used by
  this project and its corresponding specification. Do not silently upgrade
  dependencies to chase the newest specification.
- AirQuality uses BSEC_OUTPUT_STATIC_IAQ with an accuracy gate. Preserve
  units, enum mapping, validity gates, and BSEC NVS save/restore behavior.
- Matter OTA is a supported feature using dual application slots and the
  existing requestor/release tooling. Preserve this architecture; distinguish
  implemented support from updates actually verified on hardware.
- Keep device identity and version values coherent across configuration,
  Basic Information, onboarding, and release artifacts. Do not invent IDs.

## Builds and verification: opt-in

- Do not automatically build firmware or verify build outputs after edits.
  Ask first unless the user has already explicitly authorized those actions
  for the current task. A request to change code alone is not authorization.
- The user may build locally and return only if an error occurs. Respect
  that workflow and its token budget; do not run checks in anticipation.
- When a build error is supplied, inspect that evidence first. Ask before
  starting a reproduction build unless it was already requested.
- Static inspection and reviewing the requested diff remain part of normal
  code work. Clearly distinguish those checks from compilation, artifact
  verification, automated tests, and hardware validation.
- When authorized, build the affected profile and verify its corresponding
  artifact. Do not broaden or repeat verification without a concrete reason.
- Run applicable hardware checks only within the authorized task: commissioning,
  Thread stability, endpoint/identity exposure, BSEC persistence, OLED/time,
  or OTA. If checks are not authorized or hardware is unavailable, report
  them as pending. Code may be handed off without claiming a hardware-validated
  or release-ready result.

## Commands and artifacts

Run from the repository root after obtaining any required task authorization.
Use wrappers; use raw idf.py only when wrappers are unavailable. Do not overlap
build, target, or flash runs.

| Profile | Build command | App binary |
|---------|---------------|------------|
| XIAO (default board) | tools/build.sh | builds/xiao_esp32c6/wall_env_idf.bin |
| Supermini | tools/build-supermini.sh | builds/supermini_esp32c6/wall_env_idf.bin |
| Revision v2 (default revision) | tools/build-revision.sh v2 | builds/revisions/v2_esp32c6/wall_env_idf.bin |

- Use an explicit revision argument for automation. v1 uses corresponding
  v1 paths. Revision builds regenerate configuration and also export artifacts
  under builds/artifacts/<revision>/.
- Set target only when needed: tools/set-target-esp32c6.sh or
  tools/set-target-esp32c6-supermini.sh.
- Flash XIAO with tools/flash.sh; Supermini with tools/flash-supermini.sh.
  For an authorized revision build-and-flash, use
  tools/build-revision.sh v2 --flash. Build authorization alone does not
  authorize flashing a device.
- Monitor with tools/idf.sh monitor --no-reset. Set IDF_BUILD_DIR to the
  selected profile's build directory when it is not the default XIAO build.
- Use tools/update-version.sh and the documented OTA/release wrappers for
  requested release work. Preserve the profile through build, flash, and review.

## Diagnosis and device state

- Capture relevant diagnostics before resetting a failing device. Treat stale
  fabric/NVS state as a possible commissioning cause, not a mandatory erase step.
- Use targeted reset or full erasure only when justified and covered by existing
  authorization. Full erasure loses commissioning and calibration state.
  For partition changes, determine the required erase or migration procedure.
- Matter time sync can take 5–15 minutes. Use --no-reset monitoring and
  distinguish unsynced placeholder time from valid synchronized time.
- Report the first root-cause error with file:line (or closest location),
  exact command, and a one-line fix hypothesis before follow-on errors.

## Private configuration and commissioning-code exception

- Keep installation-specific location and private provisioning values in ignored
  project-root secrets/, with tracked placeholder templates under examples/secrets/.
  Keep personal values out of project instructions and public artifacts.
- Project exception: the current generic Matter commissioning code and its
  corresponding QR payload may appear in local display/serial output, including
  on boot, as well as documentation and shared diagnostics. They do not require
  redaction under the global privacy rule for this project.
- This exception does not cover other credentials, keys, fabric data, unique
  device secrets, or personal information. More robust or individualized pairing
  code generation is outside the current scope; revisit only when requested.

## Code style

- Match surrounding ESP-IDF code; use ESP_LOGx, esp_timer, and FreeRTOS APIs.
- For new code, prefer two-space indentation and same-line opening braces.
  Keep private helpers static and near their use; avoid formatting churn.
