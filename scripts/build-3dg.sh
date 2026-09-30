#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd "$(dirname "$0")/.." && pwd)"
deps_dir="$project_dir/.deps/root/usr"
build_dir="${THREEDG_BUILD_DIR:-$project_dir/build-local}"
if [[ ! -f "$project_dir/libs/qCC_db/extern/CCCoreLib/CMakeLists.txt" ]]; then
  echo "Missing CCCoreLib. Run from the project directory:" >&2
  echo "  git submodule update --init --recursive -- libs/qCC_db/extern/CCCoreLib" >&2
  exit 1
fi
linguist_dir="${QT5_LINGUIST_TOOLS_DIR:-$HOME/software/Qt/5.15.2/gcc_64/lib/cmake/Qt5LinguistTools}"
linguist_args=()
if [[ -f "$linguist_dir/Qt5LinguistToolsConfig.cmake" ]]; then
  linguist_args=(-DQt5LinguistTools_DIR="$linguist_dir")
fi
mkdir -p "$build_dir/.ccache"
export CCACHE_DIR="$build_dir/.ccache"
# Rediscover dependencies, including paths cached by older versions of this script.
# A local dependency prefix is optional; otherwise use the system installations.
deps_args=(-U 'Protobuf_*' -U 'Qt5Multimedia*_DIR' -U 'Qt5WebSockets_DIR')
if [[ -d "$deps_dir" ]]; then
  multiarch="$(gcc -print-multiarch)"
  export LD_LIBRARY_PATH="$deps_dir/lib/$multiarch${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  deps_args+=(-DCMAKE_PREFIX_PATH="$deps_dir")
  # Qt5Config from the system searches components beside itself, so explicitly
  # select local components only when their configuration files actually exist.
  for component in WebSockets Multimedia MultimediaWidgets; do
    component_dir="$deps_dir/lib/$multiarch/cmake/Qt5$component"
    if [[ -f "$component_dir/Qt5${component}Config.cmake" ]]; then
      deps_args+=("-DQt5${component}_DIR=$component_dir")
    fi
  done
else
  deps_args+=(-U CMAKE_PREFIX_PATH)
fi
cmake -S "$project_dir" -B "$build_dir" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DOPTION_3DG=ON -DOPTION_BUILD_CCVIEWER=OFF \
  -DPLUGIN_STANDARD_QPOISSON_RECON=ON \
  -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON "${deps_args[@]}" "${linguist_args[@]}"
cmake --build "$build_dir" --target CloudCompare QCORE_IO_PLUGIN QPOISSON_RECON_PLUGIN mission3dg_tests -j "${BUILD_JOBS:-2}"
