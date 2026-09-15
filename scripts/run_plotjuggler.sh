#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"

bundle_dir="${repo_root}/bundle/plotjuggler_m2"
pj_bin="${repo_root}/.deps/plotjuggler/lib/plotjuggler/plotjuggler"

if [[ ! -f "${pj_bin}" ]]; then
  echo "PlotJuggler binary not found at ${pj_bin}"
  exit 1
fi

export LD_LIBRARY_PATH="${repo_root}/.deps/plotjuggler/lib:${repo_root}/.deps/plotjuggler/usr/lib/x86_64-linux-gnu:${repo_root}/third_party/m2_sdk/third_party/install/lib:${LD_LIBRARY_PATH:-}"

# Satisfy PlotJuggler's ROS plugin discovery check to prevent the "Missing package [plotjuggler-ros]" modal
mkdir -p "${repo_root}/.deps/plotjuggler/share/ament_index/resource_index/packages" "${repo_root}/.deps/plotjuggler/lib/plotjuggler_ros"
touch "${repo_root}/.deps/plotjuggler/share/ament_index/resource_index/packages/plotjuggler_ros"
export AMENT_PREFIX_PATH="${repo_root}/.deps/plotjuggler:${AMENT_PREFIX_PATH:-}"

echo "=================================================="
echo " Starting PlotJuggler with Svan M2 Plugin Suite"
echo " Plugin directory: ${bundle_dir}"
echo "=================================================="

"${pj_bin}" --plugin_folders "${bundle_dir}" "$@"
