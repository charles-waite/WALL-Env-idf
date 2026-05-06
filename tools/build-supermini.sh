#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.boards/supermini.defaults"
export IDF_BUILD_DIR="${IDF_BUILD_DIR:-${ROOT_DIR}/builds/supermini_esp32c6}"
SDKCONFIG="${SDKCONFIG:-${IDF_BUILD_DIR}/sdkconfig}"

exec "${ROOT_DIR}/tools/idf.sh" -D SDKCONFIG="${SDKCONFIG}" -D SDKCONFIG_DEFAULTS="${SDKCONFIG_DEFAULTS}" build "$@"
