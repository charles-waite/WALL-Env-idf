#!/usr/bin/env bash

# Source this file to get ESP-IDF environment and a local idf.py alias.
# Usage:
#   source tools/activate-idf.sh
#   widf build

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "Run this script with: source tools/activate-idf.sh"
    exit 1
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source "$HOME/esp-idf/export.sh"

export WALL_ENV_IDF_ROOT="${ROOT_DIR}"
alias widf='idf.py -C "$WALL_ENV_IDF_ROOT"'
echo "Environment ready. Use: widf <command> (example: widf build)"
