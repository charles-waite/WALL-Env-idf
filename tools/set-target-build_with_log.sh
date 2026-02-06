#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_PATH="${ROOT_DIR}/build/build.log"

export IDF_CCACHE_ENABLE=1

source "$HOME/esp-idf/export.sh"
source "${ROOT_DIR}/esp-matter/export.sh"

mkdir -p "${ROOT_DIR}/build"

echo "Logging build to ${LOG_PATH}"

idf.py -C "${ROOT_DIR}" set-target esp32c6 build 2>&1 | tee "${LOG_PATH}"
