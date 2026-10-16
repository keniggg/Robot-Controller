# Remote Grasp6D Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a deployable split-compute 6D grasp path where the ROS Noetic VM sends RGB-D frames to a WSL2 GPU GraspNet baseline HTTP server and publishes returned grasp poses on `/grasp_6d/plan`.

**Architecture:** Keep the existing ROS grasp execution chain unchanged by publishing the same `geometry_msgs/PoseArray` consumed by `grasp_task_node.py`. Add a small stdlib HTTP protocol for RGB-D request/response, a ROS client node for the Ubuntu VM, and a standalone WSL2 service script that imports `graspnet-baseline` plus `checkpoint-rs.tar`.

**Tech Stack:** ROS Noetic, `rospy`, `cv_bridge`, stdlib `urllib`/`http.server`, NumPy, PyTorch, GraspNet baseline.

---

### Task 1: Protocol And Client

**Files:**
- Create: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/remote_grasp6d_client.py`
- Test: `src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_client.py`

- [ ] **Step 1: Write failing tests** for RGB-D payload round trip and response candidate parsing.
- [ ] **Step 2: Run tests** with `python3 -m unittest src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_client.py` and confirm import failure.
- [ ] **Step 3: Implement minimal protocol helpers** using base64-encoded compressed NumPy archives and validated JSON candidate parsing.
- [ ] **Step 4: Re-run tests** and confirm pass.

### Task 2: ROS Remote Node

**Files:**
- Create: `src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py`
- Modify: `src/alicia_flexible_grasp_supervisor/CMakeLists.txt`
- Test: `src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py`

- [ ] **Step 1: Write failing tests** for candidate-to-plan conversion and first reachable candidate selection.
- [ ] **Step 2: Implement node** subscribing color/depth, posting RGB-D to the remote server, transforming returned camera-frame grasp pose to base frame, reachability-checking candidates, and publishing `/grasp_6d/plan`.
- [ ] **Step 3: Re-run node tests** and confirm pass.

### Task 3: WSL2 Baseline Server

**Files:**
- Create: `tools/graspnet_baseline_server.py`
- Test: `src/alicia_flexible_grasp_supervisor/tests/test_graspnet_baseline_server_protocol.py`

- [ ] **Step 1: Write failing protocol test** for `/health` and `/predict` behavior in mock mode.
- [ ] **Step 2: Implement server** with stdlib `ThreadingHTTPServer`, lazy GraspNet baseline backend loading, checkpoint validation, and mock mode for connectivity tests.
- [ ] **Step 3: Re-run protocol tests** and confirm pass.

### Task 4: Launch And Config

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/launch/grasp_system.launch`
- Modify: `src/alicia_flexible_grasp_supervisor/launch/full_system.launch`
- Modify: `src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml`

- [ ] **Step 1: Add launch args** for `use_remote_grasp6d` and `remote_grasp6d_url`.
- [ ] **Step 2: Start remote node by default** and keep local `grasp6d_node.py` available with `use_remote_grasp6d:=false`.
- [ ] **Step 3: Add remote config** under `/grasp_6d/remote`.

### Task 5: Verification

- [ ] **Step 1:** Run all affected unit tests.
- [ ] **Step 2:** Run `catkin_make`.
- [ ] **Step 3:** Run a no-hardware launch smoke test and confirm nodes start and are cleaned up.
