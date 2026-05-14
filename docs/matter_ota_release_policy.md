# Matter OTA Release Policy

This policy defines when a WALL-Env firmware build is safe to publish through
Matter OTA. The OTA path is for routine firmware updates only. Changes that
alter the boot, flash, identity, or persistent-storage contract require a manual
flash and validation pass before OTA is used again.

## OTA Eligibility Preconditions

A device is OTA-eligible only if all of the following are true:

- The device was previously flashed with the OTA-capable partition layout
  (`otadata`, `ota_0`, `ota_1`) and successfully booted at least once.
- The running firmware has OTA Requestor enabled and endpoint 0 exposes the OTA
  Requestor cluster.
- Bootloader rollback support remains enabled and operational.
- The candidate image matches the device VID/PID and has a strictly higher
  `SoftwareVersion`.

If any condition above is false, OTA must be disallowed and the device must be
updated by manual erase/flash first.

## Supported OTA Targets

Local development uses VID `0xFFF1`.

- XIAO ESP32-C6: PID `0x8000`
- Supermini ESP32-C6: PID `0x8001`

Each board target must publish its own OTA image and metadata JSON. Do not reuse
an OTA image across PIDs, even if the ESP32-C6 chip is the same.

## Versioning Rules

`SoftwareVersion` is the authoritative update ordering field. It must increase
monotonically for every OTA-published image for a given PID.

Use this conversion:

```text
major * 10000 + minor * 100 + patch
```

Examples:

```text
1.2.0 -> 10200
1.3.0 -> 10300
1.3.1 -> 10301
```

Rules:

- Never publish two different images with the same VID, PID, and
  `SoftwareVersion`.
- Never publish an OTA image whose `SoftwareVersion` is lower than a version
  already deployed to that PID.
- Diagnostic or test builds must still use unique, increasing version numbers.
- `SoftwareVersionString` may be human-readable, but it does not replace the
  integer version.

Version source files:

- `tools/update-version.sh` is the supported way to change release versioning.
- `PROJECT_VER` sets the semantic software version string.
- `PROJECT_VER_NUMBER` sets the Matter numeric software version.
- `main/chip_project_config.h` must keep
  `CHIP_DEVICE_CONFIG_DEVICE_SOFTWARE_VERSION_STRING` aligned with
  `PROJECT_VER`.

`tools/update-version.sh` prompts for an `X.Y.Z` version, updates
`CMakeLists.txt`, updates `main/chip_project_config.h`, and computes the Matter
integer software version as `major * 10000 + minor * 100 + patch`. Rebuild every
target board after running it.

## OTA-Compatible Changes

These changes are normally acceptable for OTA, assuming the image still fits:

- Sensor/display/application logic changes.
- Matter attribute behavior changes that preserve endpoint and cluster identity.
- Logging changes.
- Non-breaking timing, retry, threshold, and calibration changes.
- Dependency updates that do not require partition, storage, bootloader, or
  factory-data changes.

## Manual-Flash-Only Changes

Do not deploy these by OTA to already-fielded devices:

- `partitions.csv` changes, including partition offsets, sizes, names, flags, or
  adding/removing partitions.
- Any change to OTA slot geometry (`ota_0`/`ota_1` offsets or sizes) or OTA data
  partition behavior.
- Bootloader configuration changes that alter rollback, secure boot, flash
  encryption, app signing, or image validation behavior.
- Changes to `CONFIG_PARTITION_TABLE_*`.
- Changes to `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE` or rollback confirmation
  semantics.
- Changes to `CONFIG_SECURE_BOOT`, `CONFIG_FLASH_ENCRYPTION_ENABLED`, signing
  keys, encryption keys, or secure-cert storage format.
- Changes to NVS partition name, NVS schema, factory-data partition, DAC/provider
  config, or Matter persistent-storage compatibility that are not explicitly
  migrated in-app.
- Changes to VID, PID, discriminator/passcode generation model, commissioning
  flow, or production identity metadata.
- Endpoint/device-type/cluster model changes that could make the existing HA
  Matter node representation invalid without re-interview or recommissioning.
- Switching Thread/Wi-Fi transport mode or disabling Matter-over-Thread support.
- Any change where a failed boot cannot be expected to roll back safely.

If one of these changes is required, manually flash one device, validate
commissioning, persistence, rollback, and HA interview behavior, then decide
whether later routine app updates can resume over OTA.

## Provider Matching Rules

The OTA provider must reject a candidate unless all conditions are met:

- `VendorID` exactly matches requestor VID.
- `ProductID` exactly matches requestor PID.
- Candidate `SoftwareVersion` is strictly greater than requestor current version.
- If set, `MinApplicableSoftwareVersion` <= requestor current version.
- If set, `MaxApplicableSoftwareVersion` >= requestor current version.

If multiple candidates are valid, serve the highest `SoftwareVersion`.

Never serve cross-PID images or equal/lower-version images.

## Required Build Checks

`tools/build-ota.sh` is the only supported packaging path for WALL-Env OTA
images. It must:

- Build the selected board profile.
- Verify the app image fits in the `0x1E0000` OTA slot with at least 10% free.
- Wrap the app binary with a Matter OTA image header.
- Generate the `python-matter-server` local-update JSON.
- Set VID/PID according to the board target.
- Set `softwareVersion`, `softwareVersionString`, `otaFileSize`,
  `otaChecksum`, and applicability range from the generated artifacts.
- Read package version from `CMakeLists.txt` (`PROJECT_VER` and
  `PROJECT_VER_NUMBER`).
- Ask for interactive confirmation before building unless `-f` or `--force` is
  used.
- Fail if runtime firmware version fields do not match the package version.
- Default `minApplicableSoftwareVersion` to `10300` unless explicitly
  overridden.

Before copying artifacts to the Matter Server OTA directory, inspect:

```sh
python3 managed_components/espressif__esp_matter/connectedhomeip/connectedhomeip/src/app/ota_image_tool.py show \
  builds/<board>/ota/wall_env_<board>_<version>.ota
```

Confirm:

- VID and PID match the physical board.
- Version is higher than the deployed version.
- Payload size fits the OTA slot.
- Digest is present.
- `SoftwareVersion` and `SoftwareVersionString` in firmware runtime config match
  the OTA header values.

## Publishing Rules

Publish OTA files as pairs:

```text
wall_env_<board>_<version>.ota
wall_env_<board>_<version>.json
```

Place both files in the Matter Server OTA provider directory. For the current
HAOS Matter Server setup, the host-visible path is:

```text
/addon_configs/core_matter_server/ota
```

The Matter Server add-on sees that directory as:

```text
/config/ota
```

The add-on must be started with:

```text
--ota-provider-dir /config/ota
```

Restart the Matter Server add-on after adding or removing local update JSON
files, because local updates are loaded at startup.

## Deployment Checklist

For each OTA release:

1. Confirm the intended target board and PID.
2. Confirm the currently deployed `SoftwareVersion`.
3. Confirm the device is already on OTA-capable partition layout and requestor
   firmware.
4. Choose the next monotonic version.
5. Run `tools/update-version.sh`.
6. Build with `tools/build-ota.sh <xiao|supermini>`.
7. Inspect the OTA header and generated JSON.
8. Verify provider-side matching will exclude wrong PID and lower/equal
   versions.
9. Copy both generated files to the Matter Server OTA directory.
10. Restart the Matter Server add-on.
11. Use the Matter Server UI or HA update entity to verify the update source is
    `local`.
12. Update one test node first.
13. Confirm download, reboot, rollback validation, and new version reporting.
14. Leave the OTA files in place only while that version should remain
    available.

## Post-Update Validation

After update, verify:

- Device returns online on the same Matter fabric.
- `BasicInformation.SoftwareVersion` matches the target integer.
- `BasicInformation.SoftwareVersionString` matches the target string.
- `OtaSoftwareUpdateRequestor.UpdateState` returns to idle.
- ESP-IDF rollback validation marks the new app valid.
- Sensor startup, display startup, BSEC startup, and Matter startup all succeed.
- HA subscriptions resume without needing recommissioning.

If the device rolls back, remove the JSON from the OTA provider directory and
restart Matter Server before investigating.
