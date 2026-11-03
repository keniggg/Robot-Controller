#!/usr/bin/env bash
set -euo pipefail
candidate_root="$(cd -- "$(dirname -- "$0")" && pwd)"
export MUJOCO_GL=disable
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export PYTHONNOUSERSITE=1
export PYTHONDONTWRITEBYTECODE=1
export PYTHONUNBUFFERED=1
# No source setup.bash, ROS launch, WSL endpoint edit or implicit --serve.
exec /usr/bin/python3 -u "$candidate_root/run_candidate.py" "$@"
