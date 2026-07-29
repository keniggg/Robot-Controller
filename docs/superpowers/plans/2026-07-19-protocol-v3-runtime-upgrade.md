# Grasp6D Protocol v3 Runtime Upgrade Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run the ROS host and the WSL GraspNet + MuJoCo service from the same protocol-v3 source revision, then verify correlated 6D candidate generation without executing a grasp.

**Architecture:** Preserve the currently running v2 checkout as the rollback copy. Build and run ROS from an ignored Git worktree pinned to merged mainline commit `5ff0bdbf76c427c072b9f90127f43710589e93e4`. Run the WSL combined inference service from that exact revision so `/predict` request correlation, timing fields, candidate schema, and `/simulate_grasp` stay aligned.

**Tech Stack:** ROS 1 Noetic, catkin, Python 3, HTTP Grasp6D protocol v3, GraspNet Baseline, MuJoCo, PyQt GUI.

## Global constraints

- Keep `/home/zhuyupei/alicia_wa_full` untouched as the v2 rollback workspace.
- Pin both sides to `5ff0bdbf76c427c072b9f90127f43710589e93e4`; do not mix individual v3 files into v2.
- Preserve the existing camera, arm, tactile, calibration, network, and launch parameters.
- Keep the arm stationary during cutover. Candidate generation is allowed; grasp execution waits for a later explicit user instruction.
- If any acceptance check fails, leave the old workspace available and do not execute a grasp.

---

### Task 1: Establish the isolated v3 ROS workspace

**Files:**
- Create: `.worktrees/protocol-v3-upgrade/`
- Verify: `.worktrees/protocol-v3-upgrade/src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/remote_grasp6d_client.py`

- [x] Confirm `.worktrees/` is ignored and the current checkout is a normal Git worktree.
- [x] Create branch `codex/protocol-v3-upgrade` at exact commit `5ff0bdbf76c427c072b9f90127f43710589e93e4`.
- [x] Confirm `GRASP6D_PROTOCOL_VERSION = 3`, request correlation fields, and model assets are present.

### Task 2: Verify protocol-v3 source behavior before build

**Files:**
- Test: `src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_client.py`
- Test: `src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py`
- Test: `src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py`
- Test: `src/alicia_flexible_grasp_supervisor/tests/test_graspnet_baseline_server_protocol.py`
- Test: `src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_server_protocol.py`

- [x] Run the client, GraspNet, and MuJoCo protocol tests that do not require generated ROS messages.
- [x] Confirm the pre-build tests require integer protocol version `3`, validate `request_id` and `snapshot_stamp_sec`, and pass (`145 passed, 1 skipped`).

### Task 3: Build the complete ROS v3 workspace

**Files:**
- Generate: `.worktrees/protocol-v3-upgrade/.superpowers/protocol-v3/build/`
- Generate: `.worktrees/protocol-v3-upgrade/.superpowers/protocol-v3/devel/`

- [x] Source ROS Noetic and run `catkin_make --force-cmake` with isolated build/devel paths under `.superpowers/protocol-v3/` so the tracked legacy artifacts are not reused.
- [x] Source the generated `devel/setup.bash` and verify the remote node resolves inside the v3 worktree.
- [x] Run a Python import check proving the installed client constant is protocol `3`.
- [x] Run the remote node and streaming tests after catkin has generated `Grasp6DPlan` and related message modules (`273 passed`).

### Task 4: Start the matching WSL v3 service

**Files:**
- Use: `tools/start_mujoco_digital_twin_wsl.sh`
- Use: `tools/mujoco_digital_twin_server.py`

- [x] In WSL, preserve the old checkout and create `~/grasp6d_ws/Robot-Controller-v3` at exact commit `5ff0bdbf76c427c072b9f90127f43710589e93e4`.
- [x] Activate `grasp6d118`, export the existing GraspNet/checkpoint/device/model paths, and start the combined server on port `8000` with `--pass-score 80 --min-lift-success-m 0.015 --warmup`.
- [x] Verify `/health` reports `ok: true`, top-level `protocol_version: 3`, loaded GraspNet, and healthy MuJoCo.

### Task 5: Cut ROS over from v2 to v3

**Files:**
- Use: `src/alicia_flexible_grasp_supervisor/launch/full_system.launch`

- [x] Gracefully stop the old v2 `roslaunch` only after the v3 build and WSL health check pass.
- [x] Launch v3 with the existing ROS IP/master settings, hardware nodes, automatic joint torque, GUI, and `remote_grasp6d_url:=http://172.23.132.97:8000`.
- [x] Start a persistent filtered log monitor and verify the driver, RGB-D camera, perception, remote Grasp6D node, and GUI stay online.
- [x] Confirm `/alicia_d/motion_enabled=true`, feedback ready, and no protection latch after launch.

### Task 6: End-to-end v3 acceptance

**Files:**
- Verify: runtime ROS topics and `http://172.23.132.97:8000/health`

- [x] Have the user confirm the small carton is correctly framed and stationary; live samples stayed near `uv=(315,165)`, `bbox=67x79`, `depth=0.269 m`, and confidence above `0.91`.
- [ ] Generate one 6D candidate batch; do not click or invoke grasp execution.
- [ ] Confirm the response is protocol v3, correlation succeeds, candidates are available, and timing metrics populate without `WSL_PREDICT_FAILED`.
- [ ] Report the accepted v3 revision, live process/session information, and the next user action.

### Rollback

- [ ] If cutover fails, stop the v3 launch and relaunch `/home/zhuyupei/alicia_wa_full` with the previous parameters. The original checkout and source remain unchanged.
