#!/usr/bin/env bash
set -euo pipefail
cd /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade
source /home/zhuyupei/alicia_wa_full/.ros_log/unknown_grasp_20260923/sanstand_storage.sh
set +u
source /opt/ros/noetic/setup.bash
source devel/setup.bash
set -u
export ROS_MASTER_URI=http://localhost:11311
export ROS_LOG_DIR=/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/resumed_observe_return_grasp_20260930T040740Z/manual_settle_ros
exec rosbag record --duration=360 --buffsize=16 --chunksize=256 -O /media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/resumed_observe_return_grasp_20260930T040740Z/manual_settle_followup_20260930.bag /joint_commands /alicia_d/sdk_command /alicia_d/accepted_joint_states /alicia_d/control_reference /alicia_d/control_reference_epoch /alicia_d/sdk_diagnostics /alicia_d/actuation_status /gui/joint_direct_mode /alicia_controller/state /alicia_controller/follow_joint_trajectory/goal /rosout_agg >> /media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/resumed_observe_return_grasp_20260930T040740Z/manual_settle_followup_recorder.log 2>&1
