#!/usr/bin/env bash
set -euo pipefail

# Regenerate `main/zap-generated` from `wall_env.zap`.
#
# Requires the esp-matter environment (zap-cli) which is set up by tools/idf.sh
# / esp-matter/export.sh.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export ESP_MATTER_PATH="${ESP_MATTER_PATH:-${ROOT_DIR}/esp-matter}"
source "${ESP_MATTER_PATH}/export.sh"

ZAP_CLI="${ZAP_CLI:-${ZAP_INSTALL_PATH}/zap-cli}"
ZAP_FILE="${ZAP_FILE:-${ROOT_DIR}/wall_env.zap}"
OUT_DIR="${OUT_DIR:-${ROOT_DIR}/main/zap-generated}"

mkdir -p "${OUT_DIR}"

echo "Generating ZAP artifacts:"
echo "  zap: ${ZAP_FILE}"
echo "  out: ${OUT_DIR}"

"${ZAP_CLI}" generate --packageMatch strict -o "${OUT_DIR}" "${ZAP_FILE}"

