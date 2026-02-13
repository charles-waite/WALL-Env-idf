#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REV_DIR="${ROOT_DIR}/sdkconfig.revisions"

usage() {
  cat <<USAGE
Usage:
  tools/build-revision.sh [v1|v2|--all] [--flash]

Examples:
  tools/build-revision.sh v1
  tools/build-revision.sh v2 --flash
  tools/build-revision.sh --all

Notes:
  - Revision overlays live in sdkconfig.revisions/<rev>.defaults
  - This script always runs set-target to regenerate sdkconfig from the selected defaults.
  - With no revision argument, a timed prompt is shown:
      A=v2, B=v1, C=both; timeout defaults to v2.
USAGE
}

choose_revision_timed() {
  local timeout_secs="${BUILD_REVISION_TIMEOUT_SECS:-5}"
  local choice=""

  echo "[build-revision] Select build target:"
  echo "  A) v2 (default)"
  echo "  B) v1"
  echo "  C) both"
  echo -n "[build-revision] Enter A/B/C within ${timeout_secs}s: "
  if ! read -r -t "${timeout_secs}" choice; then
    echo
    echo "[build-revision] No selection received. Defaulting to v2."
    echo "v2"
    return 0
  fi

  case "${choice^^}" in
    A|"") echo "v2" ;;
    B) echo "v1" ;;
    C) echo "--all" ;;
    *)
      echo "[build-revision] Invalid selection '${choice}'. Defaulting to v2." >&2
      echo "v2"
      ;;
  esac
}

build_one() {
  local rev="$1"
  local do_flash="$2"
  local overlay="${REV_DIR}/${rev}.defaults"

  if [[ ! -f "${overlay}" ]]; then
    echo "Missing revision overlay: ${overlay}"
    exit 1
  fi

  local sdk_defaults="sdkconfig.defaults;${overlay}"

  echo "[build-revision] Building revision '${rev}' with ${overlay}"
  "${ROOT_DIR}/tools/idf.sh" -D SDKCONFIG_DEFAULTS="${sdk_defaults}" set-target esp32c6
  "${ROOT_DIR}/tools/idf.sh" -D SDKCONFIG_DEFAULTS="${sdk_defaults}" build

  local out_dir="${ROOT_DIR}/build/artifacts/${rev}"
  mkdir -p "${out_dir}"
  cp "${ROOT_DIR}/build/wall_env_idf.bin" "${out_dir}/wall_env_idf_${rev}.bin"
  cp "${ROOT_DIR}/build/wall_env_idf.elf" "${out_dir}/wall_env_idf_${rev}.elf"
  cp "${ROOT_DIR}/build/partition_table/partition-table.bin" "${out_dir}/partition-table_${rev}.bin"
  cp "${ROOT_DIR}/build/bootloader/bootloader.bin" "${out_dir}/bootloader_${rev}.bin"

  echo "[build-revision] Artifacts exported to ${out_dir}"

  if [[ "${do_flash}" == "true" ]]; then
    echo "[build-revision] Flashing revision '${rev}'"
    "${ROOT_DIR}/tools/idf.sh" -D SDKCONFIG_DEFAULTS="${sdk_defaults}" flash
  fi
}

rev_arg=""
flash="false"

for arg in "$@"; do
  case "${arg}" in
    -h|--help) usage; exit 0 ;;
    --flash) flash="true" ;;
    --all|v1|v2) rev_arg="${arg}" ;;
    *)
      echo "Unknown argument: ${arg}"
      usage
      exit 2
      ;;
  esac
done

if [[ -z "${rev_arg}" ]]; then
  rev_arg="$(choose_revision_timed)"
fi

case "${rev_arg}" in
  --all)
    if [[ "${flash}" == "true" ]]; then
      echo "--flash is only supported with a single revision"
      exit 2
    fi
    build_one v1 false
    build_one v2 false
    ;;
  v1|v2)
    build_one "${rev_arg}" "${flash}"
    ;;
  *)
    usage
    exit 2
    ;;
esac
