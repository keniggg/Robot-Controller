#!/bin/bash
set -e
source /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade/devel/setup.bash
cd /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade
export ROS_MASTER_URI=http://localhost:11311
export ROS_LOG_DIR=/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/grasp_resume_20261006T031136Z/ros
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
exec /usr/bin/python3 -u tools/run_recorded_grasp.py --root /media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records --duration 600 -- /usr/bin/python3 -u .runtime/run_confirmed_unknown_grasp_20261004.py --selection-result /media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/grasp_resume_20261006T031136Z/current_selection_adapter.json
