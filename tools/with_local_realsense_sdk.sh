#!/usr/bin/env bash
# Keep the D405 VGA intrinsics fix local to the camera process.
set -euo pipefail
sdk_worktree="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
sdk_runtime="$sdk_worktree/.runtime/realsense-2.56.5"
if [[ ! -f "$sdk_runtime/pyrealsense2.cpython-38-x86_64-linux-gnu.so" ]]; then
    echo "Missing isolated D405 SDK: $sdk_runtime; see docs/operations/d405-sdk-runtime.md" >&2
    exit 1
fi
export PYTHONPATH="$sdk_runtime${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$sdk_runtime/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$@"
