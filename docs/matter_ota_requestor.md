# Matter OTA Requestor Runbook

This project uses Matter OTA Requestor over Thread/BDX with ESP-IDF dual OTA app partitions on 4 MB flash.

## Partition And Fallback Model

The shared XIAO and Supermini partition table has `otadata`, `ota_0`, and `ota_1`; each app slot is `0x1E0000` bytes. After this partition-table change, do a full flash or erase/reflash once. App-only flashing over the old table is not sufficient.

ESP-IDF rollback is enabled with `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y`. A newly downloaded OTA image boots as pending verify. The app marks it valid only after Matter startup and sensor startup succeed. If the app crashes, resets, or fails validation before marking itself valid, the bootloader rolls back to the previous valid app partition.

## Identity And Versioning

Local bring-up keeps test VID `0xFFF1`.

- XIAO PID: `0x8000`
- Supermini PID: `0x8001`

Do not publish production OTA images with the test VID/PIDs. Replace them with assigned production values across `sdkconfig.defaults`, board defaults, `main/chip_project_config.h`-derived metadata, onboarding data, and OTA image headers.

Software version numbers are monotonic SemVer integers:

```text
major * 10000 + minor * 100 + patch
```

For example, `1.3.0` becomes `10300`. OTA providers and requestors compare the integer, not only the display string.

Version source of truth is `CMakeLists.txt`:

- `PROJECT_VER`: semantic version string.
- `PROJECT_VER_NUMBER`: Matter integer software version.

Use `tools/update-version.sh` to change release versions. It prompts for an
`X.Y.Z` semantic version, computes `PROJECT_VER_NUMBER` as
`major * 10000 + minor * 100 + patch`, and updates:

- `CMakeLists.txt` `PROJECT_VER`
- `CMakeLists.txt` `PROJECT_VER_NUMBER`
- `main/chip_project_config.h` `CHIP_DEVICE_CONFIG_DEVICE_SOFTWARE_VERSION_STRING`

Do not edit only one of these fields by hand. `tools/build-ota.sh` verifies that
the compiled Matter software version, CMake version, and OTA header version all
match before it writes an OTA image.

## Build And Package

The Matter OTA image tool is vendored with esp-matter:

```sh
managed_components/espressif__esp_matter/connectedhomeip/connectedhomeip/src/app/ota_image_tool.py
```

The tool imports CHIP Python TLV helpers from the same managed component. If Python import errors occur, initialize the ESP-IDF environment first:

```sh
source "$HOME/esp-idf/export.sh"
```

Build and package OTA images:

```sh
tools/update-version.sh
tools/build-ota.sh xiao
tools/build-ota.sh supermini
```

The script reads the version from `CMakeLists.txt`, asks for confirmation,
builds the selected board, checks that `wall_env_idf.bin` leaves at least 10%
free in the `0x1E0000` OTA slot, creates the Matter OTA image header with
SHA-256 digest, emits the local Matter Server JSON, and prints
`ota_image_tool.py show` output. Use `-f` or `--force` to skip confirmation in
automation. If `--min-version` is omitted, it defaults to `10300` so pre-OTA
baseline firmware is excluded by applicability bounds.

Raw tool form:

```sh
python3 managed_components/espressif__esp_matter/connectedhomeip/connectedhomeip/src/app/ota_image_tool.py create \
  -v 0xFFF1 -p 0x8000 -vn 10300 -vs 1.3.0 -da sha256 \
  builds/xiao_esp32c6/wall_env_idf.bin \
  builds/xiao_esp32c6/ota/wall_env_xiao_1.3.0.ota
```

Use PID `0x8001` and the Supermini build path for Supermini images.

## Provider Setup

The validated local setup uses Home Assistant OS Matter Server's built-in local
OTA flow. `python-matter-server` loads local update metadata from its
`--ota-provider-dir`, starts `chip-ota-provider-app`, commissions a temporary OTA
Provider node into the HA fabric, announces the provider to the target node, and
then stops the provider after the update.

For the current HAOS Matter Server add-on setup:

```text
Host-visible path: /addon_configs/core_matter_server/ota
Add-on path:       /config/ota
Add-on argument:   --ota-provider-dir /config/ota
```

Copy the generated `.ota` and `.json` pair from `tools/build-ota.sh` into that
directory, restart the Matter Server add-on, then use the Matter Server UI or HA
update entity to check and install the local update.

Manual `chip-tool` provider setup remains useful for isolated protocol testing,
but it is not the primary WALL-Env deployment path.

See `docs/matter_ota_release_policy.md` before publishing OTA images.

## Manual Provider Reference

Use this only for standalone CHIP testing outside the HA Matter Server local OTA
flow.

Typical provider shape:

```sh
chip-ota-provider-app \
  --filepath builds/xiao_esp32c6/ota/wall_env_xiao_1.3.0.ota \
  --discriminator 3841 \
  --passcode 20202021
```

Exact binary name and options vary by CHIP build. Use local `--help` output as canonical.

Commission the OTA Provider onto the same fabric as the sensor, then either write `DefaultOTAProviders` on the sensor or announce the provider. Fill in the operational Node IDs and fabric index from your controller:

```sh
chip-tool otasoftwareupdaterequestor write default-ota-providers \
  '[{"providerNodeID": <PROVIDER_NODE_ID>, "endpoint": 0, "fabricIndex": <FABRIC_INDEX>}]' \
  <SENSOR_NODE_ID> 0
```

Or announce the provider:

```sh
chip-tool otasoftwareupdaterequestor announce-ota-provider \
  <PROVIDER_NODE_ID> 0 0 0xFFF1 <SENSOR_NODE_ID> 0
```

If your `chip-tool` syntax differs, run:

```sh
chip-tool otasoftwareupdaterequestor --help
```

## Validation Checklist

- Full-flash once after the partition change.
- Confirm boot logs show `otadata`, `ota_0`, `ota_1`, active partition, next OTA partition, VID/PID, and software version.
- Confirm endpoint 0 exposes Root Node plus OTA Requestor device type, OTA Software Update Requestor server cluster, OTA Software Update Provider client cluster, and Binding.
- Confirm the OTA image header PID matches the target board.
- Test wrong-PID, lower/equal-version, and truncated images; the requestor should reject or fail safely.
- During a real OTA, confirm reboot into the opposite app slot, rollback validation, and preserved Matter fabric/NVS state.

Matter OTA image headers provide payload integrity through the digest. Firmware authenticity/signing is separate production hardening unless secure boot and signed images are enabled.
