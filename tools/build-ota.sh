#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OTA_TOOL="${ROOT_DIR}/managed_components/espressif__esp_matter/connectedhomeip/connectedhomeip/src/app/ota_image_tool.py"
SLOT_SIZE=$((0x1E0000))
MIN_FREE_PERCENT=10
VID_HEX="0xFFF1"
CD_VERSION_NUMBER=1
DEFAULT_MIN_APPLICABLE_VERSION=10300

usage() {
    cat <<EOF
Usage: tools/build-ota.sh <xiao|supermini> [--min-version VERSION_NUMBER] [--max-version VERSION_NUMBER] [-f|--force]

Builds the selected board profile, enforces the OTA slot size margin, and wraps
the ESP-IDF app binary with the Matter OTA image header. Also emits a
python-matter-server local OTA metadata JSON file next to the OTA image. The
firmware version is read from CMakeLists.txt and checked against the runtime
Matter version fields before packaging.
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

min_version=""
max_version=""
force=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --min-version)
            min_version="${2:-}"
            shift 2
            ;;
        --max-version)
            max_version="${2:-}"
            shift 2
            ;;
        -f|--force)
            force=1
            shift
            ;;
        --version)
            echo "--version is no longer supported; run tools/update-version.sh to change the firmware version" >&2
            usage
            exit 2
            ;;
        *)
            echo "Unsupported argument: $1" >&2
            usage
            exit 2
            ;;
    esac
done

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

version="$(sed -n 's/^set(PROJECT_VER "\([^"]*\)")$/\1/p' "${ROOT_DIR}/CMakeLists.txt" | head -n 1)"
project_version_number="$(sed -n 's/^set(PROJECT_VER_NUMBER \([0-9][0-9]*\))$/\1/p' "${ROOT_DIR}/CMakeLists.txt" | head -n 1)"
if [[ -z "${version}" || -z "${project_version_number}" ]]; then
    echo "Could not determine PROJECT_VER and PROJECT_VER_NUMBER from CMakeLists.txt" >&2
    exit 3
fi

version_number="$(semver_to_number "${version}")"
if [[ "${project_version_number}" != "${version_number}" ]]; then
    echo "PROJECT_VER_NUMBER does not match PROJECT_VER:" >&2
    echo "  PROJECT_VER=${version} expected_number=${version_number} PROJECT_VER_NUMBER=${project_version_number}" >&2
    echo "Run tools/update-version.sh to change the version consistently, then rebuild." >&2
    exit 4
fi

if [[ -z "${min_version}" ]]; then
    min_version="${DEFAULT_MIN_APPLICABLE_VERSION}"
fi

if (( force == 0 )); then
    echo "Packaging OTA firmware version: ${version} (${version_number})"
    read -r -p "Continue? [Y/n]: " confirm
    case "${confirm}" in
        ""|y|Y|yes|YES|Yes)
            ;;
        n|N|no|NO|No)
            echo "Aborted. Run tools/update-version.sh to change the version number, then rebuild."
            exit 0
            ;;
        *)
            echo "Unsupported response: ${confirm}" >&2
            exit 2
            ;;
    esac
fi

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

project_ver_from_sdkconfig="$(sed -n 's/^#define CONFIG_APP_PROJECT_VER_FROM_CONFIG \([01]\)$/\1/p' "${sdkconfig_h}" || true)"
firmware_version_number="$(rg -o --no-filename -- '-DCHIP_CONFIG_SOFTWARE_VERSION_NUMBER=[0-9]+' "${build_dir}/compile_commands.json" | head -n 1 | sed 's/.*=//' || true)"
firmware_version_string_macro="$(sed -n 's/^#define CHIP_DEVICE_CONFIG_DEVICE_SOFTWARE_VERSION_STRING \(.*\)$/\1/p' "${ROOT_DIR}/main/chip_project_config.h" | head -n 1 || true)"
project_version="$(sed -n 's/^set(PROJECT_VER "\([^"]*\)")$/\1/p' "${ROOT_DIR}/CMakeLists.txt" | head -n 1 || true)"
project_version_number="$(sed -n 's/^set(PROJECT_VER_NUMBER \([0-9][0-9]*\))$/\1/p' "${ROOT_DIR}/CMakeLists.txt" | head -n 1 || true)"
if [[ "${project_ver_from_sdkconfig:-0}" != "0" ]]; then
    echo "CONFIG_APP_PROJECT_VER_FROM_CONFIG must remain disabled for WALL-Env OTA flow." >&2
    echo "  generated value=${project_ver_from_sdkconfig:-missing}" >&2
    exit 4
fi
if [[ -z "${firmware_version_number}" ]]; then
    echo "Could not determine CHIP_CONFIG_SOFTWARE_VERSION_NUMBER from compile_commands.json" >&2
    exit 4
fi
if [[ -z "${firmware_version_string_macro}" ]]; then
    echo "Could not determine CHIP_DEVICE_CONFIG_DEVICE_SOFTWARE_VERSION_STRING from main/chip_project_config.h" >&2
    exit 4
fi
if [[ -z "${project_version}" || -z "${project_version_number}" ]]; then
    echo "Could not determine PROJECT_VER / PROJECT_VER_NUMBER from CMakeLists.txt" >&2
    exit 4
fi
if [[ "${firmware_version_number}" != "${version_number}" ]]; then
    echo "Firmware compile-time SoftwareVersion number does not match requested OTA header:" >&2
    echo "  firmware=${firmware_version_number:-missing} requested=${version_number}" >&2
    echo "  run a clean rebuild after updating CMakeLists PROJECT_VER/PROJECT_VER_NUMBER" >&2
    exit 4
fi
if [[ "${firmware_version_string_macro}" != "\"${version}\"" ]]; then
    echo "CHIP software version string macro does not match requested OTA header version:" >&2
    echo "  main/chip_project_config.h defines: ${firmware_version_string_macro:-missing}" >&2
    echo "  requested=${version}" >&2
    exit 4
fi
if [[ "${project_version}" != "${version}" ]]; then
    echo "ESP-IDF PROJECT_VER does not match requested OTA header:" >&2
    echo "  project=${project_version:-missing} requested=${version}" >&2
    echo "  update PROJECT_VER before packaging OTA" >&2
    exit 4
fi
if [[ "${project_version_number}" != "${version_number}" ]]; then
    echo "ESP-IDF PROJECT_VER_NUMBER does not match requested OTA header:" >&2
    echo "  project=${project_version_number:-missing} requested=${version_number}" >&2
    echo "  update PROJECT_VER_NUMBER before packaging OTA" >&2
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
min_applicable="${min_version}"
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
