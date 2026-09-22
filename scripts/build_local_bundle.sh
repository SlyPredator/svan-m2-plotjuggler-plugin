#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
cd "${repo_root}"

jobs="${CMAKE_BUILD_PARALLEL_LEVEL:-16}"

# 1. Initialize submodules if missing
if [[ ! -f "third_party/m2_sdk/CMakeLists.txt" || ! -f "third_party/xterra_m2_assets/m2_metal_description/urdf/m2_metal_description.urdf" ]]; then
  echo "Initializing required submodules..."
  git submodule update --init --depth 1 third_party/m2_sdk third_party/xterra_m2_assets
fi

# 2. Build CycloneDDS dependencies if not already present
if [[ ! -d "third_party/m2_sdk/third_party/install" ]]; then
  echo "CycloneDDS dependencies not found. Building locally..."
  "${script_dir}/build_cyclonedds_dev.sh"
fi

# 3. Build PlotJuggler binary & dev SDK into .deps/plotjuggler if not already present
if [[ ! -f ".deps/plotjuggler/bin/plotjuggler" && ! -f ".deps/plotjuggler/lib/plotjuggler/plotjuggler" ]]; then
  echo "PlotJuggler not found in .deps/plotjuggler. Building PlotJuggler..."
  "${script_dir}/build_plotjuggler_dev.sh"
fi

echo "Configuring and building Svan M2 PlotJuggler Plugins..."
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel "${jobs}"
ctest --test-dir build --output-on-failure

echo ""
echo "=================================================="
echo " Build successful! Runtime bundle created at:"
echo " ${repo_root}/bundle/plotjuggler_m2"
echo "=================================================="
