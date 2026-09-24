#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd "$(dirname "$0")/.." && pwd)"
deps_dir="$project_dir/.deps/root/usr"
build_dir="${THREEDG_BUILD_DIR:-$project_dir/build-local}"
linguist_dir="${QT5_LINGUIST_TOOLS_DIR:-$HOME/software/Qt/5.15.2/gcc_64/lib/cmake/Qt5LinguistTools}"
linguist_args=()
if [[ -f "$linguist_dir/Qt5LinguistToolsConfig.cmake" ]]; then
  linguist_args=(-DQt5LinguistTools_DIR="$linguist_dir")
fi
mkdir -p "$build_dir/.ccache"
export CCACHE_DIR="$build_dir/.ccache"
export LD_LIBRARY_PATH="$deps_dir/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
cmake -S "$project_dir" -B "$build_dir" -G Ninja -DCMAKE_BUILD_TYPE=Release -DOPTION_3DG=ON -DOPTION_BUILD_CCVIEWER=OFF -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON -DCMAKE_PREFIX_PATH="$deps_dir" -DQt5Multimedia_DIR="$deps_dir/lib/x86_64-linux-gnu/cmake/Qt5Multimedia" -DQt5MultimediaWidgets_DIR="$deps_dir/lib/x86_64-linux-gnu/cmake/Qt5MultimediaWidgets" -DProtobuf_INCLUDE_DIR="$deps_dir/include" -DProtobuf_LIBRARY="$deps_dir/lib/x86_64-linux-gnu/libprotobuf.so" -DProtobuf_PROTOC_EXECUTABLE="$deps_dir/bin/protoc" "${linguist_args[@]}"
cmake --build "$build_dir" --target CloudCompare QCORE_IO_PLUGIN mission3dg_tests -j "${BUILD_JOBS:-2}"
