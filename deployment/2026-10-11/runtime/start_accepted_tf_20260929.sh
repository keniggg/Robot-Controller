#!/bin/bash
set -eo pipefail
source /home/zhuyupei/alicia_wa_full/.ros_log/unknown_grasp_20260923/sanstand_storage.sh
cd /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade
source devel/setup.bash
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONUNBUFFERED=1
exec /usr/bin/python3 .runtime/accepted_tf_handoff.py --legacy-pid 431007 --parent-pid 8526 --model-sha256 7812d6b69e854ed9f37b4e3cbd813ecd53dab3504065b2d04d287abc6fd852e0 --output /media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/accepted_tf_load_20260929T084705Z
