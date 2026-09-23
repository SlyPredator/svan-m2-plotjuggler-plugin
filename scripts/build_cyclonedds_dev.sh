#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
sdk_dir="${repo_root}/third_party/m2_sdk"

if [[ ! -d "${sdk_dir}" ]]; then
  echo "Error: third_party/m2_sdk directory not found. Please clone submodules." >&2
  exit 1
fi

echo "Setting up CycloneDDS and CycloneDDS-CXX in ${sdk_dir}..."
cd "${sdk_dir}"
bash ./setup_dds_env.sh
