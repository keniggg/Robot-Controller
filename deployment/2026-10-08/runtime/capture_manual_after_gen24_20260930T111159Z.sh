#!/usr/bin/env bash
set -e
source /home/zhuyupei/alicia_wa_full/.ros_log/unknown_grasp_20260923/sanstand_storage.sh
source /opt/ros/noetic/setup.bash
source /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade/devel/setup.bash
export ROS_LOG_DIR="$ALICIA_SESSION_LOG_DIR" PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec rosbag record --duration=90 --buffsize=32 --quiet -O '/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/resumed_observe_return_grasp_20260930T040740Z/manual_after_gen24_20260930T111159Z.bag' /alicia_d/sdk_command /alicia_d/accepted_joint_states /joint_states /joint_commands /gui/joint_direct_mode /alicia_d/sdk_diagnostics /alicia_controller/state /alicia_controller/follow_joint_trajectory/goal /alicia_controller/follow_joint_trajectory/result /grasp/state > '/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/resumed_observe_return_grasp_20260930T040740Z/manual_after_gen24_20260930T111159Z.log' 2>&1
