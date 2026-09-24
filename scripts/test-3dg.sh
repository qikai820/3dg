#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd "$(dirname "$0")/.." && pwd)"
build_dir="${THREEDG_BUILD_DIR:-$project_dir/build-local}"
export LD_LIBRARY_PATH="$project_dir/.deps/root/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
cmake --build "$build_dir" --target mission3dg_tests -j "${BUILD_JOBS:-2}"
"$build_dir/qCC/mission3dg_tests"
if [[ "${1:-}" == "--gui" ]]; then
  export THREEDG_SMOKE_DIR="$project_dir/artifacts/smoke"
  "$project_dir/scripts/run-3dg.sh" --demo
fi
