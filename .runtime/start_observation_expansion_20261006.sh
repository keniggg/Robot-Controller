#!/bin/bash
set -e
cd /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
exec roslaunch --skip-log-check .runtime/observation_expansion_remote_20261006.launch
