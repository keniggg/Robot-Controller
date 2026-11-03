#!/usr/bin/env bash
set -eo pipefail
cd /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade
source /home/zhuyupei/alicia_wa_full/.ros_log/unknown_grasp_20260923/sanstand_storage.sh
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_LOG_DIR=/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_ros_latest_start_20260930T014325Z/contact_precheck_ros
export PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
mkdir -p "$ROS_LOG_DIR"
exec roslaunch --wait .runtime/model_pair_reload/remote_only.launch >> /media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_ros_latest_start_20260930T014325Z/contact_precheck_remote.log 2>&1
