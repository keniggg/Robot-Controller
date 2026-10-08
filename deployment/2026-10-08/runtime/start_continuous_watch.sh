#!/usr/bin/env bash
set -e
cd /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade
source /home/zhuyupei/alicia_wa_full/.ros_log/unknown_grasp_20260923/sanstand_storage.sh
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_LOG_DIR="$ALICIA_UNKNOWN_LOG_DIR"
export PYTHONUNBUFFERED=1
while true; do
  /usr/bin/python3 -u tools/watch_grasp_attempt_readonly.py --duration 1800 >> "$ALICIA_UNKNOWN_LOG_DIR/supervision_continuous.log" 2>&1 || true
  sleep 2
done
