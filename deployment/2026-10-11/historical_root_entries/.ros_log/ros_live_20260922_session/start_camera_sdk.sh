#!/usr/bin/env bash
set -e
cd /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade
source /home/zhuyupei/alicia_wa_full/.ros_log/unknown_grasp_20260923/sanstand_storage.sh
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_LOG_DIR="$ALICIA_SESSION_LOG_DIR"
export PYTHONUNBUFFERED=1
export OPENBLAS_NUM_THREADS=1
python3 - <<'PYCFG'
import rospy
rospy.set_param('/camera/mode_depth_preset', dict(enabled=True, unknown_preset='high_accuracy', stationary_temporal_enabled=True, baseline_path='/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade/.runtime/d405_carton_depth_baseline.json'))
PYCFG
exec tools/with_local_realsense_sdk.sh python3 -u src/alicia_flexible_grasp_supervisor/scripts/camera_node.py __name:=supervisor_camera >> "$ROS_LOG_DIR/camera_sdk.log" 2>&1
