#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.boards/supermini.defaults"

exec "${ROOT_DIR}/tools/idf.sh" -D SDKCONFIG_DEFAULTS="${SDKCONFIG_DEFAULTS}" set-target esp32c6 "$@"
