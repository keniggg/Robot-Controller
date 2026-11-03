#!/bin/bash
set -eo pipefail
cd /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade
test -n "${ROS_LOG_DIR:-}"
case "$(realpath -m "$ROS_LOG_DIR")" in
  /media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/*/ros) ;;
  *) echo "SanStand ROS_LOG_DIR required; no fallback" >&2; exit 2 ;;
esac
mountpoint -q /media/zhuyupei/SanStand
test -d "$ROS_LOG_DIR"
source devel/setup.bash
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
exec roslaunch --skip-log-check .runtime/support_source_task_20261006.launch
