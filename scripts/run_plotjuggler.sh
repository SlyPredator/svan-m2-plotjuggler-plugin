#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"

bundle_dir="${repo_root}/bundle/plotjuggler_m2"
pj_bin=""
for candidate in \
  "${repo_root}/.deps/plotjuggler/bin/plotjuggler" \
  "${repo_root}/.deps/plotjuggler/lib/plotjuggler/plotjuggler"; do
  if [[ -f "${candidate}" ]]; then
    pj_bin="${candidate}"
    break
  fi
done

if [[ -z "${pj_bin}" ]]; then
  if command -v plotjuggler >/dev/null 2>&1; then
    pj_bin="$(command -v plotjuggler)"
  else
    echo "PlotJuggler binary not found."
    echo "Run ./scripts/build_local_bundle.sh to build PlotJuggler and plugins automatically."
    exit 1
  fi
fi

export LD_LIBRARY_PATH="${repo_root}/.deps/plotjuggler/lib:${repo_root}/.deps/plotjuggler/usr/lib/x86_64-linux-gnu:${repo_root}/third_party/m2_sdk/third_party/install/lib:${LD_LIBRARY_PATH:-}"

# Satisfy PlotJuggler's ROS plugin discovery check to prevent the "Missing package [plotjuggler-ros]" modal
mkdir -p "${repo_root}/.deps/plotjuggler/share/ament_index/resource_index/packages" "${repo_root}/.deps/plotjuggler/lib/plotjuggler_ros"
touch "${repo_root}/.deps/plotjuggler/share/ament_index/resource_index/packages/plotjuggler_ros"
export AMENT_PREFIX_PATH="${repo_root}/.deps/plotjuggler:${AMENT_PREFIX_PATH:-}"

echo "=================================================="
echo " Starting PlotJuggler with Svan M2 Plugin Suite"
echo " Plugin directory: ${bundle_dir}"
echo " Binary:           ${pj_bin}"
echo "=================================================="

buffer_size="30"  # default 30s for live stream
mode="live"
enhanced="0"
forward_args=()

for arg in "$@"; do
  if [[ "${arg}" == "--bag" ]]; then
    buffer_size="999"
    mode="bag"
  elif [[ "${arg}" == "--live" ]]; then
    buffer_size="30"
    mode="live"
  elif [[ "${arg}" == "--enhanced" ]]; then
    enhanced="1"
  else
    forward_args+=("${arg}")
  fi
done

export PLOTJUGGLER_M2_ENHANCED="${enhanced}"

if [[ "${enhanced}" == "1" ]]; then
  echo " Mode: ENHANCED  (PD torque, leg aliases, power calcs enabled)"
else
  echo " Mode: CANONICAL (raw IDL arrays, no enhancements)"
  echo " Tip:  Run with --enhanced to enable derived signals"
fi

export PLOTJUGGLER_M2_MODE="${mode}"

# Ensure PlotJuggler ini doesn't restore a stale 5s/10s buffer on startup
if [[ -f "${HOME}/.config/PlotJuggler/io.plotjuggler.PlotJuggler.ini" ]]; then
  sed -i "s/^MainWindow\.streamingBufferValue=.*/MainWindow.streamingBufferValue=${buffer_size}/" \
    "${HOME}/.config/PlotJuggler/io.plotjuggler.PlotJuggler.ini" 2>/dev/null || true
fi

# Pass --buffer_size if not already explicitly specified
if ! [[ " ${forward_args[*]:-} " =~ " --buffer_size " ]]; then
  forward_args=("--buffer_size" "${buffer_size}" "${forward_args[@]:-}")
fi

"${pj_bin}" --plugin_folders "${bundle_dir}" "${forward_args[@]:-}"
