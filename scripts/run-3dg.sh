#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd "$(dirname "$0")/.." && pwd)"
build_dir="${THREEDG_BUILD_DIR:-$project_dir/build-local}"
export LD_LIBRARY_PATH="$project_dir/.deps/root/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
export QT_PLUGIN_PATH="$project_dir/.deps/root/usr/lib/x86_64-linux-gnu/qt5/plugins:${QT_PLUGIN_PATH:-}"
export CC_PLUGIN_PATH="$build_dir/plugins/core/IO/qCoreIO${CC_PLUGIN_PATH:+:$CC_PLUGIN_PATH}"
export THREEDG=1
# The onboard RTSP stream is reached directly. GStreamer's GIO resolver can
# otherwise route it through the desktop HTTP proxy, even with no_proxy set.
export GIO_USE_PROXY_RESOLVER=dummy
if [[ "${1:-}" == "--demo" ]]; then export THREEDG_DEMO=1; shift; fi
exec "$build_dir/qCC/CloudCompare" "$@"
