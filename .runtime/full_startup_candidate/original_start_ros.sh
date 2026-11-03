#!/usr/bin/env bash
set -e
cd /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade
source /home/zhuyupei/alicia_wa_full/.ros_log/unknown_grasp_20260923/sanstand_storage.sh
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_LOG_DIR="$ALICIA_SESSION_LOG_DIR"
export DISPLAY=:0
export XAUTHORITY=/run/user/1000/gdm/Xauthority
export PYTHONUNBUFFERED=1
export OPENBLAS_NUM_THREADS=1
export GRASP6D_URL=http://172.23.132.97:8000
exec roslaunch --skip-log-check "/home/zhuyupei/alicia_wa_full/.ros_log/ros_live_20260922_session/resume_modes.launch" >> "$ROS_LOG_DIR/ros.log" 2>&1
