#!/usr/bin/env bash
set -euo pipefail

# Regenerate `main/zap-generated` from `wall_env.zap`.
#
# Requires zap-cli to be installed or provided via ZAP_CLI.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ZAP_CLI="${ZAP_CLI:-$(command -v zap-cli || true)}"
ZAP_FILE="${ZAP_FILE:-${ROOT_DIR}/wall_env.zap}"
OUT_DIR="${OUT_DIR:-${ROOT_DIR}/main/zap-generated}"

if [[ -z "${ZAP_CLI}" ]]; then
    echo "error: zap-cli is not available."
    echo "Set ZAP_CLI to a zap-cli binary path or install zap-cli."
    echo "Example: ZAP_CLI=/path/to/zap-cli tools/zap_regen.sh"
    exit 1
fi

mkdir -p "${OUT_DIR}"

echo "Generating ZAP artifacts:"
echo "  zap: ${ZAP_FILE}"
echo "  out: ${OUT_DIR}"

"${ZAP_CLI}" generate --packageMatch strict -o "${OUT_DIR}" "${ZAP_FILE}"
