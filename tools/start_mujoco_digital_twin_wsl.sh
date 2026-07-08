#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

python tools/mujoco_digital_twin_server.py \
  --baseline-root "${GRASPNET_BASELINE_ROOT:-/home/lv/grasp6d_ws/graspnet-baseline}" \
  --checkpoint "${GRASPNET_CHECKPOINT:-/home/lv/grasp6d_ws/checkpoints/checkpoint-rs.tar}" \
  --model-xml "${MUJOCO_ALICIA_MODEL_XML:-$(pwd)/src/arm-mujoco/synriard/mjcf/Alicia_D_v5_6/Alicia_D_v5_6_gripper_50mm.xml}" \
  --host "${MUJOCO_TWIN_HOST:-0.0.0.0}" \
  --port "${MUJOCO_TWIN_PORT:-8000}" \
  --device "${GRASPNET_DEVICE:-cuda:0}" \
  --ros-joint-state-topic "${ROS_JOINT_STATE_TOPIC:-/joint_states}" \
  "$@"
