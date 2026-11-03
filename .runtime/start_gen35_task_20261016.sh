#!/bin/bash
set -eo pipefail
cd /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade
# Fail before importing ROS when the inherited external log destination is absent.
/usr/bin/python3 -B - <<'PY'
import os
from pathlib import Path
mount = Path('/media/zhuyupei/SanStand')
root = mount / '实验记录/Robot-Controller/ros_log'
logs = Path(os.environ['ROS_LOG_DIR']).resolve(strict=True)
assert mount.is_mount() and not os.statvfs(mount).f_flag & os.ST_RDONLY
assert root in logs.parents and logs.stat().st_dev == mount.stat().st_dev
PY
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec roslaunch --skip-log-check /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade/.runtime/gen35_task_consumers_20261016.launch
