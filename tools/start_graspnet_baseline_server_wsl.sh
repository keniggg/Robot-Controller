#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

BASELINE_ROOT="${BASELINE_ROOT:-/home/lv/grasp6d_ws/graspnet-baseline}"
CHECKPOINT="${CHECKPOINT:-/home/lv/grasp6d_ws/checkpoints/checkpoint-rs.tar}"
HOST="${HOST:-0.0.0.0}"
PORT="${PORT:-8000}"
DEVICE="${DEVICE:-cuda:0}"
CUDA_HOME="${CUDA_HOME:-/usr/local/cuda-11.8}"

if [[ -d "${CUDA_HOME}" ]]; then
  export CUDA_HOME
  export PATH="${CUDA_HOME}/bin:${PATH}"
  export LD_LIBRARY_PATH="${CUDA_HOME}/lib64:${LD_LIBRARY_PATH:-}"
fi

if command -v python >/dev/null 2>&1; then
  PYTHON_BIN="python"
elif command -v python3 >/dev/null 2>&1; then
  PYTHON_BIN="python3"
else
  echo "ERROR: python/python3 not found. Activate the grasp6d118 conda env first." >&2
  exit 127
fi

if [[ ! -d "${BASELINE_ROOT}" ]]; then
  echo "ERROR: baseline root not found: ${BASELINE_ROOT}" >&2
  exit 2
fi

if [[ ! -f "${CHECKPOINT}" ]]; then
  echo "ERROR: checkpoint not found: ${CHECKPOINT}" >&2
  exit 2
fi

cd "${REPO_ROOT}"
exec "${PYTHON_BIN}" tools/graspnet_baseline_server.py \
  --baseline-root "${BASELINE_ROOT}" \
  --checkpoint "${CHECKPOINT}" \
  --host "${HOST}" \
  --port "${PORT}" \
  --device "${DEVICE}" \
  --warmup
