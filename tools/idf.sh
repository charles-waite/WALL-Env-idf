#!/usr/bin/env bash
set -euo pipefail

# Wrapper to run idf.py with both ESP-IDF and ESP-Matter/CHIP GN environment set up.
# This avoids "gn command not found" errors when cmake regenerates CHIP libraries.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Optional: speed up builds if ccache is installed/configured.
export IDF_CCACHE_ENABLE="${IDF_CCACHE_ENABLE:-1}"

source "$HOME/esp-idf/export.sh"

# esp-matter/export.sh expects ESP_MATTER_PATH to be set (it errors under `set -u`).
export ESP_MATTER_PATH="${ESP_MATTER_PATH:-${ROOT_DIR}/esp-matter}"
source "${ESP_MATTER_PATH}/export.sh"

exec idf.py -C "${ROOT_DIR}" "$@"
