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
export PYTHONNOUSERSITE=1
export OMP_NUM_THREADS=1
export GRASP6D_URL=http://172.23.132.97:8000
/usr/bin/python3 "/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade/.runtime/full_startup_candidate/cold_start_handoff.py" --prepare >> "$ROS_LOG_DIR/ros.log" 2>&1
systemctl --user start alicia-candidate-simulation.service
/usr/bin/python3 "/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade/.runtime/full_startup_candidate/candidate_simulation_startup.py" --wait-ready >> "$ROS_LOG_DIR/ros.log" 2>&1
exec roslaunch --skip-log-check "/home/zhuyupei/alicia_wa_full/.ros_log/ros_live_20260922_session/resume_modes.launch" >> "$ROS_LOG_DIR/ros.log" 2>&1
