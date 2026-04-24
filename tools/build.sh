#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export IDF_BUILD_DIR="${IDF_BUILD_DIR:-${ROOT_DIR}/builds/xiao_esp32c6}"
exec "${ROOT_DIR}/tools/idf.sh" build "$@"
