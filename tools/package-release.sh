#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BOARD="${1:-xiao}"
VERSION="${2:-1.3.2}"

case "${BOARD}" in
  xiao|xiao_esp32c6)
    BOARD="xiao"
    BUILD_DIR="${ROOT_DIR}/builds/xiao_esp32c6"
    ;;
  supermini|supermini_esp32c6)
    BOARD="supermini"
    BUILD_DIR="${ROOT_DIR}/builds/supermini_esp32c6"
    ;;
  *)
    echo "Usage: tools/package-release.sh [xiao|supermini] [version]"
    echo "Example: tools/package-release.sh xiao v1.2"
    exit 2
    ;;
esac

BOOTLOADER="${BUILD_DIR}/bootloader/bootloader.bin"
PARTITION_TABLE="${BUILD_DIR}/partition_table/partition-table.bin"
APP="${BUILD_DIR}/wall_env_idf.bin"
OTA_DATA="${BUILD_DIR}/ota_data_initial.bin"
OUT_DIR="${ROOT_DIR}/builds/releases/${VERSION}"
OUT_BIN="${OUT_DIR}/wall_env_${BOARD}_${VERSION}.bin"
OUT_README="${OUT_DIR}/README_${BOARD}_${VERSION}.txt"

for file in "${BOOTLOADER}" "${PARTITION_TABLE}" "${OTA_DATA}" "${APP}"; do
  if [[ ! -f "${file}" ]]; then
    echo "[package-release] Missing required build artifact: ${file}"
    echo "[package-release] Run the matching board build first."
    exit 2
  fi
done

mkdir -p "${OUT_DIR}"

source "${HOME}/esp-idf/export.sh" >/dev/null

esptool.py --chip esp32c6 merge_bin \
  --flash_mode dio \
  --flash_freq 80m \
  --flash_size 4MB \
  --target-offset 0x0 \
  --output "${OUT_BIN}" \
  0x0 "${BOOTLOADER}" \
  0xc000 "${PARTITION_TABLE}" \
  0x1d000 "${OTA_DATA}" \
  0x20000 "${APP}"

cat > "${OUT_README}" <<README
WALL-Env ${VERSION} ${BOARD} full flash image

Flash command:
  esptool.py --chip esp32c6 -p <PORT> -b 460800 --before default_reset --after hard_reset write_flash 0x0 "$(basename "${OUT_BIN}")"

macOS example:
  esptool.py --chip esp32c6 -p /dev/cu.usbmodem101 -b 460800 --before default_reset --after hard_reset write_flash 0x0 "$(basename "${OUT_BIN}")"

This merged image contains:
  0x0     bootloader.bin
  0xc000  partition-table.bin
  0x1d000 ota_data_initial.bin
  0x20000 wall_env_idf.bin
README

ls -lh "${OUT_BIN}" "${OUT_README}"
echo "[package-release] Wrote ${OUT_BIN}"
