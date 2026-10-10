#!/usr/bin/env bash
# Cross-build CloudCompare 3DG for Windows x64, then create a portable ZIP.
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${THREEDG_WINDOWS_BUILD_DIR:-$project_dir/build-windows}"
qt_dir="${THREEDG_WINDOWS_QT_DIR:-$project_dir/.deps/windows-qt/5.15.2/mingw81_64}"
protobuf_src="${THREEDG_WINDOWS_PROTOBUF_SRC:-$project_dir/.deps/protobuf-3.21.12}"
protobuf_lib="${THREEDG_WINDOWS_PROTOBUF_LIB:-$project_dir/.deps/protobuf-build-win/libprotobuf.a}"
output_dir="${THREEDG_WINDOWS_OUTPUT_DIR:-$project_dir/artifacts/windows}"
jobs="${BUILD_JOBS:-2}"

if [[ "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Usage: scripts/build-3dg-windows.sh

Cross-builds CloudCompare 3DG for Windows x64 and packages a portable ZIP.
Run this script on Linux after installing MinGW-w64 (POSIX), CMake, Ninja,
host Qt 5 build tools, protoc, Python 3, Windows Qt 5.15.2, and a Windows
Protobuf 3.21.12 static library.

Optional environment variables:
  THREEDG_WINDOWS_BUILD_DIR       default: build-windows
  THREEDG_WINDOWS_QT_DIR          default: .deps/windows-qt/5.15.2/mingw81_64
  THREEDG_WINDOWS_PROTOBUF_SRC    default: .deps/protobuf-3.21.12
  THREEDG_WINDOWS_PROTOBUF_LIB    default: .deps/protobuf-build-win/libprotobuf.a
  THREEDG_WINDOWS_OUTPUT_DIR      default: artifacts/windows
  THREEDG_WINDOWS_LRELEASE        host Qt 5 lrelease executable
  BUILD_JOBS                      default: 2
EOF
  exit 0
fi
if [[ $# -ne 0 ]]; then
  echo "Unknown argument: $1 (use --help)" >&2
  exit 2
fi

for tool in cmake ninja python3 protoc moc rcc uic \
  x86_64-w64-mingw32-gcc-posix x86_64-w64-mingw32-g++-posix \
  x86_64-w64-mingw32-windres x86_64-w64-mingw32-objdump; do
  if ! command -v "$tool" >/dev/null 2>&1; then
    echo "Missing required tool: $tool" >&2
    exit 1
  fi
done
lrelease="${THREEDG_WINDOWS_LRELEASE:-}"
if [[ -z "$lrelease" ]]; then
  for candidate in "$(command -v lrelease || true)" \
    "$(command -v lrelease-qt5 || true)" \
    /usr/lib/qt5/bin/lrelease \
    "${HOME:-}/software/Qt/5.15.2/gcc_64/bin/lrelease"; do
    if [[ -n "$candidate" && -x "$candidate" ]] && \
       "$candidate" -version >/dev/null 2>&1; then
      lrelease="$candidate"
      break
    fi
  done
fi
if [[ -z "$lrelease" || ! -x "$lrelease" ]] || \
   ! "$lrelease" -version >/dev/null 2>&1; then
  echo "Missing working host Qt 5 lrelease; set THREEDG_WINDOWS_LRELEASE" >&2
  exit 1
fi
for path in \
  "$project_dir/libs/qCC_db/extern/CCCoreLib/CMakeLists.txt" \
  "$qt_dir/lib/cmake/Qt5/Qt5Config.cmake" \
  "$qt_dir/plugins/platforms/qwindows.dll" \
  "$protobuf_src/src/google/protobuf/message.h" \
  "$protobuf_src/LICENSE" \
  "$protobuf_lib"; do
  if [[ ! -f "$path" ]]; then
    echo "Missing Windows build dependency: $path" >&2
    exit 1
  fi
done
if [[ ! "$jobs" =~ ^[1-9][0-9]*$ ]]; then
  echo "BUILD_JOBS must be a positive integer" >&2
  exit 2
fi
mingw_runtime_file="$(x86_64-w64-mingw32-g++-posix -print-file-name=libstdc++-6.dll)"
if [[ ! -f "$mingw_runtime_file" ]]; then
  echo "Missing MinGW runtime: libstdc++-6.dll" >&2
  exit 1
fi
mingw_runtime_dir="$(dirname "$mingw_runtime_file")"

mkdir -p "$build_dir/.ccache"
export CCACHE_DIR="$build_dir/.ccache"

# CMake must run Linux Qt tools even though it links against Windows Qt.
host_tools="$build_dir/qt-host-tools.cmake"
cat > "$host_tools" <<EOF
if(TARGET Qt5::moc)
  set_target_properties(Qt5::moc PROPERTIES IMPORTED_LOCATION "$(command -v moc)")
endif()
if(TARGET Qt5::rcc)
  set_target_properties(Qt5::rcc PROPERTIES IMPORTED_LOCATION "$(command -v rcc)")
endif()
if(TARGET Qt5::uic)
  set_target_properties(Qt5::uic PROPERTIES IMPORTED_LOCATION "$(command -v uic)")
endif()
EOF

cmake -S "$project_dir" -B "$build_dir" -G Ninja \
  -DCMAKE_SYSTEM_NAME=Windows \
  -DCMAKE_C_COMPILER="$(command -v x86_64-w64-mingw32-gcc-posix)" \
  -DCMAKE_CXX_COMPILER="$(command -v x86_64-w64-mingw32-g++-posix)" \
  -DCMAKE_RC_COMPILER="$(command -v x86_64-w64-mingw32-windres)" \
  -DCMAKE_PROJECT_CloudCompare_INCLUDE="$host_tools" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$qt_dir" \
  -DQt5_DIR="$qt_dir/lib/cmake/Qt5" \
  -DProtobuf_INCLUDE_DIR="$protobuf_src/src" \
  -DProtobuf_LIBRARY="$protobuf_lib" \
  -DProtobuf_PROTOC_EXECUTABLE="$(command -v protoc)" \
  -Dwin_deploy_qt=/usr/bin/true \
  -DOPTION_3DG=ON \
  -DOPTION_BUILD_CCVIEWER=OFF \
  -DPLUGIN_IO_QCORE=ON

cmake --build "$build_dir" --target CloudCompare QCORE_IO_PLUGIN -j "$jobs"
python3 "$project_dir/scripts/package-3dg-windows.py" \
  --build-dir "$build_dir" --qt-dir "$qt_dir" \
  --output-dir "$output_dir" --lrelease "$lrelease" \
  --mingw-runtime-dir "$mingw_runtime_dir" \
  --protobuf-license "$protobuf_src/LICENSE"
