# RGB Dataset Collector Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and launch a minimal ROS/PyQt RealSense RGB collector with three category buttons and per-category sequence-number filenames.

**Architecture:** Reuse the existing `camera_node.py` to publish RealSense RGB images. Add one standalone collector GUI script that subscribes to the RGB topic, renders the latest frame, and saves button-selected frames into category directories.

**Tech Stack:** ROS Noetic `rospy`, `sensor_msgs/Image`, `cv_bridge`, PyQt5, OpenCV, Python 3.

## Global Constraints

- Do not start robot arm, serial, MoveIt, tactile, grasp, or torque nodes.
- Save source RGB frames from `/supervisor/camera/color/image_raw`, not GUI screenshots.
- Use category-local six-digit sequence filenames such as `000001.png`.
- Preserve unrelated dirty worktree changes.

---

### Task 1: Sequence Filename Helpers

**Files:**
- Test: `src/alicia_flexible_grasp_supervisor/tests/test_rgb_dataset_collector.py`
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/rgb_dataset_collector_gui.py`
- Test: `src/alicia_flexible_grasp_supervisor/launch/rgb_dataset_collector.launch`

**Interfaces:**
- Produces: `DatasetCategory`, `ensure_category_dirs(output_root)`, `next_sequence_path(directory)`

- [ ] **Step 1: Write failing tests**

Add tests that verify the three default category directories and that existing `000001.png`, `000003.png` lead to next file `000004.png`.

- [ ] **Step 2: Run tests and verify failure**

Run: `python src/alicia_flexible_grasp_supervisor/tests/test_rgb_dataset_collector.py -v`

- [ ] **Step 3: Implement helpers**

Create `DatasetCategory`, directory creation, numeric stem scanning, and next path generation.

- [ ] **Step 4: Run tests and verify pass**

Run: `python src/alicia_flexible_grasp_supervisor/tests/test_rgb_dataset_collector.py -v`

### Task 2: Collector GUI And Launch

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/rgb_dataset_collector_gui.py`
- Create: `src/alicia_flexible_grasp_supervisor/launch/rgb_dataset_collector.launch`
- Create: `src/alicia_flexible_grasp_supervisor/config/rgb_dataset_camera.yaml`

**Interfaces:**
- Consumes: `/supervisor/camera/color/image_raw`
- Produces: PNG files under `~/carton_dataset/raw_rgb/{positive,negative,low_sample}`

- [ ] **Step 1: Implement GUI**

Initialize a ROS node, subscribe to RGB images with `queue_size=1`, render RGB preview, and save the latest BGR frame on button click.

- [ ] **Step 2: Add launch**

Load `config/rgb_dataset_camera.yaml`, start `camera_node.py`, and start `rgb_dataset_collector_gui.py`.

- [ ] **Step 3: Verify syntax**

Run: `python -m py_compile src/alicia_flexible_grasp_supervisor/scripts/rgb_dataset_collector_gui.py`

- [ ] **Step 4: Launch runtime**

Run: `source devel/setup.bash && roslaunch alicia_flexible_grasp_supervisor rgb_dataset_collector.launch`
