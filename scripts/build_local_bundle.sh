#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
cd "${repo_root}"

jobs="${CMAKE_BUILD_PARALLEL_LEVEL:-16}"

echo "Configuring and building Svan M2 PlotJuggler Plugins..."
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel "${jobs}"
ctest --test-dir build --output-on-failure

echo ""
echo "=================================================="
echo " Build successful! Runtime bundle created at:"
echo " ${repo_root}/bundle/plotjuggler_m2"
echo "=================================================="
