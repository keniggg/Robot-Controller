#!/bin/bash
set -e
test "$#" -eq 1
test -n "$ROS_LOG_DIR"
source /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade/devel/setup.bash
cd /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade
export ROS_MASTER_URI=http://localhost:11311
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
exec /usr/bin/python3 -u tools/run_recorded_grasp.py --root /media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records --duration 600 -- /usr/bin/python3 -u .runtime/run_confirmed_unknown_grasp_20261004.py --selection-result "$1"
