#!/usr/bin/env bash
set -e
cd /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade
source /home/zhuyupei/alicia_wa_full/.ros_log/unknown_grasp_20260923/sanstand_storage.sh
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_LOG_DIR=/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/grasp_repair_resume_20260928T120606Z/ros
export PYTHONUNBUFFERED=1
export OPENBLAS_NUM_THREADS=1
mkdir -p "$ROS_LOG_DIR"
exec roslaunch --wait --skip-log-check .runtime/current_board_observer.launch >> /media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/grasp_repair_resume_20260928T120606Z/handeye_observer.log 2>&1
