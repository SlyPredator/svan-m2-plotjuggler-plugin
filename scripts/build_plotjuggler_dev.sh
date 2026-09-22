#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"

ref="${PLOTJUGGLER_REF:-3.17.2}"
jobs="${CMAKE_BUILD_PARALLEL_LEVEL:-$(nproc)}"
root="${RUNNER_TEMP:-${repo_root}/.deps}/plotjuggler-dev"
src_dir="${PLOTJUGGLER_SOURCE_DIR:-${root}/src}"
build_dir="${PLOTJUGGLER_BUILD_DIR:-${root}/build}"
install_dir="${PLOTJUGGLER_INSTALL_DIR:-${repo_root}/.deps/plotjuggler}"

if [[ ! -d "${src_dir}/.git" ]]; then
  rm -rf "${src_dir}"
  echo "Cloning PlotJuggler ${ref}..."
  git clone --depth 1 --branch "${ref}" \
    https://github.com/PlotJuggler/PlotJuggler.git "${src_dir}"
fi

if [[ -f "${build_dir}/CMakeCache.txt" ]] && ! grep -q "CMAKE_GENERATOR:INTERNAL=Ninja" "${build_dir}/CMakeCache.txt" 2>/dev/null; then
  echo "Clearing CMake cache due to generator mismatch..."
  rm -rf "${build_dir}"
fi

echo "Configuring and compiling PlotJuggler ${ref}..."
cmake -S "${src_dir}" -B "${build_dir}" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${install_dir}" \
  -DBASE_AS_SHARED=ON \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5

cmake --build "${build_dir}" --target install --parallel "${jobs}"

printf '%s\n' "${install_dir}"
