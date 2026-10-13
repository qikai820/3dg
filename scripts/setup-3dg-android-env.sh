#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace_dir="$(cd "$project_dir/.." && pwd)"
dev_dir="$workspace_dir/android-dev"
sdk_dir="$dev_dir/sdk"
qt_root="$dev_dir/qt"
qt_version="6.12.0"
ndk_version="27.2.12479018"
cmdline_version="15859902"
cmdline_sha256="4e4c464f145a7512b57d088ac6c278c03c9eea610886b35a5e0804e74eedf583"
protobuf_version="3.21.12"
protobuf_source="$project_dir/.deps/protobuf-$protobuf_version"
host_dir="$dev_dir/host"
cmdline_zip="$dev_dir/cache/commandlinetools-linux-${cmdline_version}_latest.zip"
protobuf_tar="$dev_dir/cache/protobuf-cpp-${protobuf_version}.tar.gz"
aqt_venv="$dev_dir/aqt-venv"

mkdir -p "$dev_dir/cache" "$sdk_dir/cmdline-tools" "$host_dir/bin" "$project_dir/.deps"

if [[ ! -d "$sdk_dir/cmdline-tools/latest" ]]; then
  if [[ ! -s "$cmdline_zip" ]]; then
    curl -fL --retry 3 "https://dl.google.com/android/repository/commandlinetools-linux-${cmdline_version}_latest.zip" -o "$cmdline_zip"
  fi
  echo "$cmdline_sha256  $cmdline_zip" | sha256sum --check --status || {
    echo "Android command-line tools checksum mismatch" >&2
    exit 1
  }
  stage="$dev_dir/cache/cmdline-tools-stage"
  rm -rf "$stage"
  mkdir -p "$stage"
  unzip -q "$cmdline_zip" -d "$stage"
  mv "$stage/cmdline-tools" "$sdk_dir/cmdline-tools/latest"
  rmdir "$stage"
fi

export JAVA_HOME="${JAVA_HOME:-$(dirname "$(dirname "$(readlink -f "$(command -v java)")")")}"
export ANDROID_SDK_ROOT="$sdk_dir"
export ANDROID_HOME="$sdk_dir"
export ANDROID_NDK_ROOT="$sdk_dir/ndk/$ndk_version"
export PATH="$JAVA_HOME/bin:$sdk_dir/cmdline-tools/latest/bin:$sdk_dir/platform-tools:$host_dir/bin:$PATH"

{ yes || true; } | sdkmanager --sdk_root="$sdk_dir" --licenses >/dev/null
sdkmanager --sdk_root="$sdk_dir" \
  "platform-tools" "platforms;android-28" "platforms;android-36" \
  "build-tools;36.0.0" "ndk;$ndk_version"

if [[ ! -x "$aqt_venv/bin/aqt" ]]; then
  python3 -m venv "$aqt_venv"
  "$aqt_venv/bin/pip" install --upgrade pip aqtinstall
fi
if [[ ! -f "$qt_root/$qt_version/gcc_64/lib/cmake/Qt6/Qt6Config.cmake" || \
      ! -f "$qt_root/$qt_version/gcc_64/lib/cmake/Qt6WebSockets/Qt6WebSocketsConfig.cmake" ]]; then
  "$aqt_venv/bin/aqt" install-qt linux desktop "$qt_version" linux_gcc_64 -O "$qt_root" --modules qtwebsockets
fi
if [[ ! -f "$qt_root/$qt_version/android_arm64_v8a/lib/cmake/Qt6/qt.toolchain.cmake" || \
      ! -f "$qt_root/$qt_version/android_arm64_v8a/lib/cmake/Qt6WebSockets/Qt6WebSocketsConfig.cmake" ]]; then
  "$aqt_venv/bin/aqt" install-qt linux android "$qt_version" android_arm64_v8a -O "$qt_root" --modules qtwebsockets
fi

if [[ ! -d "$protobuf_source" ]]; then
  if [[ ! -s "$protobuf_tar" ]]; then
    curl -fL --retry 3 "https://github.com/protocolbuffers/protobuf/releases/download/v21.12/protobuf-cpp-${protobuf_version}.tar.gz" -o "$protobuf_tar"
  fi
  mkdir -p "$protobuf_source"
  tar -xzf "$protobuf_tar" -C "$protobuf_source" --strip-components=1
fi

protobuf_build="$dev_dir/builds/protobuf-host"
cmake -S "$protobuf_source" -B "$protobuf_build" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -Dprotobuf_BUILD_TESTS=OFF \
  -Dprotobuf_BUILD_SHARED_LIBS=OFF -Dprotobuf_BUILD_LIBPROTOC=ON \
  -Dprotobuf_BUILD_PROTOC_BINARIES=ON -Dprotobuf_WITH_ZLIB=OFF
cmake --build "$protobuf_build" --target protoc -j "${BUILD_JOBS:-4}"
install -m 0755 "$protobuf_build/protoc" "$host_dir/bin/protoc"

cat > "$dev_dir/env.sh" <<'ENV'
#!/usr/bin/env bash
_3dg_android_dev_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export JAVA_HOME="${JAVA_HOME:-$(dirname "$(dirname "$(readlink -f "$(command -v java)")")")}"
export ANDROID_SDK_ROOT="$_3dg_android_dev_dir/sdk"
export ANDROID_HOME="$ANDROID_SDK_ROOT"
export ANDROID_NDK_ROOT="$ANDROID_SDK_ROOT/ndk/27.2.12479018"
export PATH="$JAVA_HOME/bin:$ANDROID_SDK_ROOT/cmdline-tools/latest/bin:$ANDROID_SDK_ROOT/platform-tools:$_3dg_android_dev_dir/host/bin:$PATH"
ENV
chmod +x "$dev_dir/env.sh"

echo "Android toolchain installed under: $dev_dir"
echo "Qt: $qt_version (gcc_64 and android_arm64_v8a)"
echo "NDK: $ndk_version"
echo "Host protoc: $("$host_dir/bin/protoc" --version)"
