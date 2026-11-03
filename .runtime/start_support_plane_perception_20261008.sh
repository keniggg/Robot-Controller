#!/bin/bash
set -e
cd /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade
source devel/setup.bash
test -n "$ROS_LOG_DIR"
mountpoint -q /media/zhuyupei/SanStand
case "$ROS_LOG_DIR" in /media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/*) ;; *) exit 2 ;; esac
mkdir -p "$ROS_LOG_DIR"
export ROS_MASTER_URI=http://localhost:11311
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec roslaunch --skip-log-check .runtime/support_fit_perception_20261006.launch
