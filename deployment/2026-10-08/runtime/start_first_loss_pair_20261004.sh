#!/bin/bash
set -e
cd /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade
source /home/zhuyupei/alicia_wa_full/.ros_log/unknown_grasp_20260923/sanstand_storage.sh
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export PYTHONPATH="$PWD/src/alicia_grasp_modes/src:$PWD/src/alicia_flexible_grasp_supervisor/src:$PYTHONPATH"
export ROS_LOG_DIR='/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_ros_resume_20261004T002930Z'
exec /usr/bin/python3 .runtime/collect_unknown_first_loss.py --first-loss-pair --seconds 180
