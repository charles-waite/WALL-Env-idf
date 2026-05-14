#!/usr/bin/env bash
set -euo pipefail

# Regenerate `main/zap-generated` from `wall_env.zap`.
#
# Requires zap-cli to be installed, provided via ZAP_CLI, or available in the
# esp-matter CIPD environment under ~/.espressif.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

DEFAULT_ZAP_CLI="${HOME}/.espressif/esp-matter/connectedhomeip/connectedhomeip/.environment/cipd/packages/zap/zap-cli"
DEFAULT_ZAP_TEMPLATES="${HOME}/.espressif/esp-matter/connectedhomeip/connectedhomeip/src/app/zap-templates"

ZAP_CLI="${ZAP_CLI:-$(command -v zap-cli || true)}"
if [[ -z "${ZAP_CLI}" && -x "${DEFAULT_ZAP_CLI}" ]]; then
    ZAP_CLI="${DEFAULT_ZAP_CLI}"
fi

ZAP_FILE="${ZAP_FILE:-${ROOT_DIR}/wall_env.zap}"
OUT_DIR="${OUT_DIR:-${ROOT_DIR}/main/zap-generated}"
ZAP_TEMPLATES_DIR="${ZAP_TEMPLATES_DIR:-${DEFAULT_ZAP_TEMPLATES}}"

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

zap_args=(generate --packageMatch strict -o "${OUT_DIR}")
if [[ -f "${ZAP_TEMPLATES_DIR}/zcl/zcl.json" && -f "${ZAP_TEMPLATES_DIR}/app-templates.json" ]]; then
    echo "  zcl: ${ZAP_TEMPLATES_DIR}/zcl/zcl.json"
    echo "  templates: ${ZAP_TEMPLATES_DIR}/app-templates.json"
    zap_args=(
        generate
        --packageMatch ignore
        --zcl "${ZAP_TEMPLATES_DIR}/zcl/zcl.json"
        --gen "${ZAP_TEMPLATES_DIR}/app-templates.json"
        -o "${OUT_DIR}"
    )
fi

"${ZAP_CLI}" "${zap_args[@]}" "${ZAP_FILE}"

# The esp-matter component currently vendored by this project uses the older
# one-argument cluster shutdown callback signature. Newer ZAP templates emit a
# MatterClusterShutdownType argument, so normalize the generated app shim.
perl -0pi -e 's/, MatterClusterShutdownType shutdownType//g; s/, shutdownType//g' \
    "${OUT_DIR}/CodeDrivenCallback.h" \
    "${OUT_DIR}/CodeDrivenInitShutdown.cpp"
