#!/bin/bash
set -e
cd /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade
# Resolve one disk before ROS initializes logging. Inherited session paths are
# checked against the same backend/UUID binding; no ~/.ros/log fallback.
storage_args=(--shell-environment)
if [[ -n "${ROS_LOG_DIR:-}" ]]; then storage_args+=(--path "$ROS_LOG_DIR"); fi
storage_environment=$(/usr/bin/python3 -B tools/experiment_storage.py "${storage_args[@]}")
eval "$storage_environment"
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
export PYTHONUNBUFFERED=1 PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export GRASP6D_URL=http://172.23.132.97:8000
exec roslaunch --skip-log-check .runtime/full_latest_gen36_single_observation_20261016.launch
