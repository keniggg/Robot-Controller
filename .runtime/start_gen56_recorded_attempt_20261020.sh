#!/bin/bash
# A fresh reviewed scene is required; the unchanged recorded runner owns checks.
set -e
case "${1:-}" in
  /media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen56_runtime/scene_start_*) ;;
  *) exit 2 ;;
esac
test -f "$1/operator_observation.json"
cd /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export ROS_LOG_DIR="$1/recorded_attempt_ros"
exec python3 -B tools/run_recorded_grasp.py \
  --root '/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records' \
  --duration 600 -- \
  python3 -B .runtime/run_reboot_realigned_grasp_20261010.py \
  --session-dir "$1" \
  --user-message '你一直进行处理，直到成功抓取' \
  --continuation-authorization '/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/authorization.json'
