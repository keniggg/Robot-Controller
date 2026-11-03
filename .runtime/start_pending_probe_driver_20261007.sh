#!/usr/bin/env bash
set -e
cd /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade
mountpoint -q /media/zhuyupei/SanStand
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_LOG_DIR="$1"
exec devel/lib/alicia_d_driver/alicia_d_driver_node __name:=alicia_d_driver_node \
  _auto_torque_on_startup:=true \
  _software_temperature_monitoring_enabled:=false _temperature_poll_rate_hz:=0.0
