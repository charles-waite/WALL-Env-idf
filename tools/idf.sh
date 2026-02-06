#!/usr/bin/env bash
set -euo pipefail

# Wrapper to run idf.py with both ESP-IDF and ESP-Matter/CHIP GN environment set up.
# This avoids "gn command not found" errors when cmake regenerates CHIP libraries.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Prevent overlapping idf.py runs (build/set-target/flash) from this wrapper.
# Keep lock artifacts outside build/ so `idf.py set-target` can run fullclean
# even when build/ does not yet contain CMake metadata.
LOCK_FILE="${ROOT_DIR}/.idf_sh.lock"
LOCK_DIR="${ROOT_DIR}/.idf_sh.lockdir"
LOCK_MODE=""
CHILD_PID=""
HEARTBEAT_PID=""

cleanup() {
    if [[ -n "${HEARTBEAT_PID}" ]]; then
        kill "${HEARTBEAT_PID}" >/dev/null 2>&1 || true
        wait "${HEARTBEAT_PID}" 2>/dev/null || true
    fi
    if [[ -n "${CHILD_PID}" ]]; then
        kill "${CHILD_PID}" >/dev/null 2>&1 || true
        wait "${CHILD_PID}" 2>/dev/null || true
    fi
    if [[ "${LOCK_MODE}" == "mkdir" ]]; then
        rm -rf "${LOCK_DIR}"
    fi
}
trap cleanup EXIT INT TERM

if command -v flock >/dev/null 2>&1; then
    exec 9>"${LOCK_FILE}"
    if ! flock -n 9; then
        echo "[idf.sh] Another idf.sh command is already running. Wait for it to finish."
        exit 2
    fi
    LOCK_MODE="flock"
else
    if mkdir "${LOCK_DIR}" 2>/dev/null; then
        printf "%s\n" "$$" > "${LOCK_DIR}/pid"
        LOCK_MODE="mkdir"
    else
        if [[ -f "${LOCK_DIR}/pid" ]]; then
            lock_pid="$(cat "${LOCK_DIR}/pid" 2>/dev/null || true)"
            if [[ -n "${lock_pid}" ]] && ! kill -0 "${lock_pid}" 2>/dev/null; then
                rm -rf "${LOCK_DIR}"
                mkdir "${LOCK_DIR}"
                printf "%s\n" "$$" > "${LOCK_DIR}/pid"
                LOCK_MODE="mkdir"
            else
                echo "[idf.sh] Another idf.sh command is already running (pid=${lock_pid:-unknown}). Wait for it to finish."
                exit 2
            fi
        else
            echo "[idf.sh] Another idf.sh command is already running. Wait for it to finish."
            exit 2
        fi
    fi
fi

# Migration safety for older wrapper behavior:
# if set-target sees a non-CMake build/ that is empty (or only old lock files),
# remove it so idf.py fullclean can proceed.
if [[ "${1:-}" == "set-target" ]] && [[ -d "${ROOT_DIR}/build" ]] && [[ ! -f "${ROOT_DIR}/build/CMakeCache.txt" ]]; then
    shopt -s nullglob dotglob
    build_entries=("${ROOT_DIR}/build"/*)
    shopt -u nullglob dotglob
    removable=true
    for entry in "${build_entries[@]}"; do
        base_name="$(basename "${entry}")"
        if [[ "${base_name}" != ".idf_sh.lock" && "${base_name}" != ".idf_sh.lockdir" ]]; then
            removable=false
            break
        fi
    done
    if [[ "${removable}" == "true" ]]; then
        rm -rf "${ROOT_DIR}/build"
    fi
fi

# Optional: speed up builds if ccache is installed/configured.
export IDF_CCACHE_ENABLE="${IDF_CCACHE_ENABLE:-1}"

source "$HOME/esp-idf/export.sh"

# esp-matter/export.sh expects ESP_MATTER_PATH to be set (it errors under `set -u`).
export ESP_MATTER_PATH="${ESP_MATTER_PATH:-${ROOT_DIR}/esp-matter}"
source "${ESP_MATTER_PATH}/export.sh"

run_idf() {
    local -r heartbeat_secs="${IDF_HEARTBEAT_SECS:-30}"
    idf.py -C "${ROOT_DIR}" "$@" &
    CHILD_PID=$!

    if [[ "${heartbeat_secs}" =~ ^[0-9]+$ ]] && [[ "${heartbeat_secs}" -gt 0 ]]; then
        (
            while kill -0 "${CHILD_PID}" >/dev/null 2>&1; do
                sleep "${heartbeat_secs}" || break
                if kill -0 "${CHILD_PID}" >/dev/null 2>&1; then
                    echo "[idf.sh] heartbeat: idf.py (pid=${CHILD_PID}) is still running..."
                fi
            done
        ) &
        HEARTBEAT_PID=$!
    fi

    set +e
    wait "${CHILD_PID}"
    local rc=$?
    set -e

    if [[ -n "${HEARTBEAT_PID}" ]]; then
        kill "${HEARTBEAT_PID}" >/dev/null 2>&1 || true
        wait "${HEARTBEAT_PID}" 2>/dev/null || true
        HEARTBEAT_PID=""
    fi
    CHILD_PID=""
    return "${rc}"
}

# For serial-port operations on this workstation, prefer usbmodem1101, then
# fallback to usbmodem101. This avoids manual retries when macOS rotates the
# device suffix.
is_port_sensitive_cmd=false
is_monitor_cmd=false
has_port_arg=false
requested_port=""
prev=""
for arg in "$@"; do
    if [[ "${arg}" == "flash" || "${arg}" == "monitor" ]]; then
        is_port_sensitive_cmd=true
    fi
    if [[ "${arg}" == "monitor" ]]; then
        is_monitor_cmd=true
    fi
    if [[ "${arg}" == "-p" || "${arg}" == "--port" ]]; then
        has_port_arg=true
        prev="${arg}"
        continue
    fi
    if [[ "${prev}" == "-p" || "${prev}" == "--port" ]]; then
        requested_port="${arg}"
    fi
    prev=""
done

if [[ "${is_monitor_cmd}" == "true" ]] && [[ ! -t 0 ]]; then
    echo "[idf.sh] monitor requires an interactive TTY."
    echo "[idf.sh] Run from a real terminal (VS Code Integrated Terminal), not from a task/output pane."
    exit 3
fi

if [[ "${is_port_sensitive_cmd}" == "true" ]]; then
    if [[ "${has_port_arg}" == "false" ]]; then
        echo "[idf.sh] Serial op: trying /dev/cu.usbmodem1101 first..."
        if run_idf -p /dev/cu.usbmodem1101 "$@"; then
            exit 0
        fi
        echo "[idf.sh] Serial op: retrying with /dev/cu.usbmodem101..."
        run_idf -p /dev/cu.usbmodem101 "$@"
        exit $?
    fi

    if [[ "${requested_port}" == "/dev/cu.usbmodem1101" ]]; then
        echo "[idf.sh] Serial op: requested /dev/cu.usbmodem1101..."
        if run_idf "$@"; then
            exit 0
        fi
        echo "[idf.sh] Serial op: /dev/cu.usbmodem1101 failed, retrying /dev/cu.usbmodem101..."

        retry_args=()
        prev=""
        for arg in "$@"; do
            if [[ "${prev}" == "-p" || "${prev}" == "--port" ]]; then
                retry_args+=("/dev/cu.usbmodem101")
                prev=""
                continue
            fi
            retry_args+=("${arg}")
            if [[ "${arg}" == "-p" || "${arg}" == "--port" ]]; then
                prev="${arg}"
            else
                prev=""
            fi
        done
        run_idf "${retry_args[@]}"
        exit $?
    fi
fi

run_idf "$@"
