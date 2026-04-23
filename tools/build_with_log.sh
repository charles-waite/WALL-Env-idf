#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export IDF_BUILD_DIR="${IDF_BUILD_DIR:-${ROOT_DIR}/builds/xiao_esp32c6}"
LOG_PATH="${IDF_BUILD_DIR}/build.log"

mkdir -p "${IDF_BUILD_DIR}"

echo "Logging build to ${LOG_PATH}"
"${ROOT_DIR}/tools/idf.sh" build 2>&1 | tee "${LOG_PATH}"
