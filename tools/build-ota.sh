#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OTA_TOOL="${ROOT_DIR}/managed_components/espressif__esp_matter/connectedhomeip/connectedhomeip/src/app/ota_image_tool.py"
SLOT_SIZE=$((0x1E0000))
MIN_FREE_PERCENT=10
VID_HEX="0xFFF1"
CD_VERSION_NUMBER=1

usage() {
    cat <<EOF
Usage: tools/build-ota.sh <xiao|supermini> --version MAJOR.MINOR.PATCH [--min-version VERSION_NUMBER] [--max-version VERSION_NUMBER]

Builds the selected board profile, enforces the OTA slot size margin, and wraps
the ESP-IDF app binary with the Matter OTA image header. Also emits a
python-matter-server local OTA metadata JSON file next to the OTA image.
EOF
}

semver_to_number() {
    local version="$1"
    if [[ ! "${version}" =~ ^([0-9]+)\.([0-9]+)\.([0-9]+)$ ]]; then
        echo "Version must be MAJOR.MINOR.PATCH, got '${version}'" >&2
        return 2
    fi
    echo $((BASH_REMATCH[1] * 10000 + BASH_REMATCH[2] * 100 + BASH_REMATCH[3]))
}

file_size() {
    local path="$1"
    if stat -f '%z' "${path}" >/dev/null 2>&1; then
        stat -f '%z' "${path}"
    else
        stat -c '%s' "${path}"
    fi
}

sha256_base64() {
    local path="$1"
    if command -v openssl >/dev/null 2>&1; then
        openssl dgst -sha256 -binary "${path}" | openssl base64
    else
        python3 - "${path}" <<'PY'
import base64
import hashlib
import pathlib
import sys

print(base64.b64encode(hashlib.sha256(pathlib.Path(sys.argv[1]).read_bytes()).digest()).decode("ascii"))
PY
    fi
}

board="${1:-}"
if [[ -z "${board}" || "${board}" == "-h" || "${board}" == "--help" ]]; then
    usage
    exit 0
fi
shift

version=""
min_version=""
max_version=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --version)
            version="${2:-}"
            shift 2
            ;;
        --min-version)
            min_version="${2:-}"
            shift 2
            ;;
        --max-version)
            max_version="${2:-}"
            shift 2
            ;;
        *)
            echo "Unsupported argument: $1" >&2
            usage
            exit 2
            ;;
    esac
done

if [[ -z "${version}" ]]; then
    echo "--version is required" >&2
    usage
    exit 2
fi

case "${board}" in
    xiao)
        build_cmd=("${ROOT_DIR}/tools/build.sh")
        build_dir="${ROOT_DIR}/builds/xiao_esp32c6"
        pid_hex="0x8000"
        ;;
    supermini)
        build_cmd=("${ROOT_DIR}/tools/build-supermini.sh")
        build_dir="${ROOT_DIR}/builds/supermini_esp32c6"
        pid_hex="0x8001"
        ;;
    *)
        echo "Unsupported board '${board}', expected xiao or supermini" >&2
        exit 2
        ;;
esac

version_number="$(semver_to_number "${version}")"
app_bin="${build_dir}/wall_env_idf.bin"
ota_dir="${build_dir}/ota"
ota_image="${ota_dir}/wall_env_${board}_${version}.ota"
ota_json="${ota_dir}/wall_env_${board}_${version}.json"

"${build_cmd[@]}"

if [[ ! -f "${app_bin}" ]]; then
    echo "Missing app binary: ${app_bin}" >&2
    exit 3
fi
if [[ ! -x "${OTA_TOOL}" && ! -f "${OTA_TOOL}" ]]; then
    echo "Missing Matter OTA tool: ${OTA_TOOL}" >&2
    exit 3
fi

sdkconfig_h="${build_dir}/config/sdkconfig.h"
if [[ ! -f "${sdkconfig_h}" ]]; then
    echo "Missing generated sdkconfig header: ${sdkconfig_h}" >&2
    exit 3
fi

firmware_version_number="$(sed -n 's/^#define CONFIG_DEVICE_SOFTWARE_VERSION_NUMBER \([0-9][0-9]*\)$/\1/p' "${sdkconfig_h}")"
firmware_version_string="$(sed -n 's/^#define CHIP_DEVICE_CONFIG_DEVICE_SOFTWARE_VERSION_STRING "\(.*\)"$/\1/p' "${ROOT_DIR}/main/chip_project_config.h")"

if [[ "${firmware_version_number}" != "${version_number}" ]]; then
    echo "Firmware runtime SoftwareVersion does not match requested OTA header:" >&2
    echo "  firmware=${firmware_version_number:-missing} requested=${version_number}" >&2
    echo "  update CONFIG_DEVICE_SOFTWARE_VERSION_NUMBER before packaging OTA" >&2
    exit 4
fi
if [[ "${firmware_version_string}" != "${version}" ]]; then
    echo "Firmware runtime SoftwareVersionString does not match requested OTA header:" >&2
    echo "  firmware=${firmware_version_string:-missing} requested=${version}" >&2
    echo "  update CHIP_DEVICE_CONFIG_DEVICE_SOFTWARE_VERSION_STRING before packaging OTA" >&2
    exit 4
fi

app_size="$(file_size "${app_bin}")"
min_free_bytes=$((SLOT_SIZE * MIN_FREE_PERCENT / 100))
max_allowed=$((SLOT_SIZE - min_free_bytes))
free_bytes=$((SLOT_SIZE - app_size))

if (( app_size > max_allowed )); then
    echo "App binary is too large for OTA margin:" >&2
    echo "  app=${app_size} bytes slot=${SLOT_SIZE} bytes free=${free_bytes} bytes required_free=${min_free_bytes} bytes" >&2
    exit 4
fi

mkdir -p "${ota_dir}"

ota_args=(
    create
    -v "${VID_HEX}"
    -p "${pid_hex}"
    -vn "${version_number}"
    -vs "${version}"
    -da sha256
)
if [[ -n "${min_version}" ]]; then
    ota_args+=("--min-version" "${min_version}")
fi
if [[ -n "${max_version}" ]]; then
    ota_args+=("--max-version" "${max_version}")
fi
ota_args+=("${app_bin}" "${ota_image}")

python3 "${OTA_TOOL}" "${ota_args[@]}"

ota_size="$(file_size "${ota_image}")"
ota_checksum="$(sha256_base64 "${ota_image}")"
min_applicable="${min_version:-0}"
max_applicable="${max_version:-$((version_number - 1))}"
vid_decimal="$((VID_HEX))"
pid_decimal="$((pid_hex))"

python3 - "${ota_json}" <<PY
import json
import pathlib

data = {
    "modelVersion": {
        "vid": ${vid_decimal},
        "pid": ${pid_decimal},
        "softwareVersion": ${version_number},
        "softwareVersionString": "${version}",
        "cdVersionNumber": ${CD_VERSION_NUMBER},
        "firmwareInformation": "",
        "softwareVersionValid": True,
        "otaUrl": "file:/${ota_image##*/}",
        "otaFileSize": "${ota_size}",
        "otaChecksum": "${ota_checksum}",
        "otaChecksumType": 1,
        "minApplicableSoftwareVersion": ${min_applicable},
        "maxApplicableSoftwareVersion": ${max_applicable},
        "releaseNotesUrl": "",
        "creator": "local",
    }
}

path = pathlib.Path("${ota_json}")
path.write_text(json.dumps(data, indent=2) + "\\n")
PY

echo
echo "OTA image:"
python3 "${OTA_TOOL}" show "${ota_image}"
echo
echo "Board: ${board}"
echo "VID: ${VID_HEX}"
echo "PID: ${pid_hex}"
echo "SoftwareVersion: ${version_number}"
echo "SoftwareVersionString: ${version}"
echo "App size: ${app_size} bytes"
echo "OTA slot free: ${free_bytes} bytes"
echo "OTA slot free percent: $((free_bytes * 100 / SLOT_SIZE))%"
echo "Output: ${ota_image}"
echo "Matter Server metadata: ${ota_json}"
