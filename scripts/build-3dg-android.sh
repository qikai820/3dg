#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "$0")/.." && pwd)"
dev_dir="$(cd "$project_dir/.." && pwd)/android-dev"
if [[ ! -f "$dev_dir/env.sh" ]]; then
  echo "Android development environment is missing: $dev_dir" >&2
  exit 1
fi
source "$dev_dir/env.sh"

build_dir="${THREEDG_ANDROID_BUILD_DIR:-$dev_dir/builds/3dg-android-arm64-main}"
protobuf_source="${THREEDG_PROTOBUF_SOURCE_DIR:-$project_dir/.deps/protobuf-3.21.12}"
qt_dir="$dev_dir/qt/6.12.0"
cmake -S "$project_dir" -B "$build_dir" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE="$qt_dir/android_arm64_v8a/lib/cmake/Qt6/qt.toolchain.cmake" \
  -DQT_HOST_PATH="$qt_dir/gcc_64" \
  -DANDROID_ABI=arm64-v8a -DANDROID_PLATFORM=android-28 \
  -DTHREEDG_PROTOBUF_SOURCE_DIR="$protobuf_source"
cmake --build "$build_dir" --target ThreeDGAndroid_make_apk -j "${BUILD_JOBS:-4}"

apk="$build_dir/android/android-build/build/outputs/apk/release/android-build-release-unsigned.apk"
if [[ ! -s "$apk" ]]; then
  echo "APK was not created: $apk" >&2
  exit 1
fi
echo "Unsigned Android APK: $apk"
