#!/usr/bin/env bash
set -e
cd /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_LOG_DIR="$1"
export DISPLAY=:0
export XAUTHORITY=/run/user/1000/gdm/Xauthority
export PYTHONUNBUFFERED=1
export PYTHONNOUSERSITE=1
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
test -d /media/zhuyupei/SanStand
mountpoint -q /media/zhuyupei/SanStand
exec roslaunch --skip-log-check alicia_grasp_modes gui_modes.launch
