#!/usr/bin/env bash
set -e
cd /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade
source /home/zhuyupei/alicia_wa_full/.ros_log/unknown_grasp_20260923/sanstand_storage.sh
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_LOG_DIR="$ALICIA_UNKNOWN_LOG_DIR/approach_failure_20260927"
export PYTHONUNBUFFERED=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec rosrun alicia_grasp_modes mode_aware_grasp_task.py /grasp/start:=/grasp/internal_start __name:=grasp_task_node >> "$ROS_LOG_DIR/task_reloaded.log" 2>&1
