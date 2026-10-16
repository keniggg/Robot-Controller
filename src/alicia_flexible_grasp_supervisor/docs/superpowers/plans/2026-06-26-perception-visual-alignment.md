# Perception Visual Alignment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a small Cartesian micro-jog panel inside the target recognition page so the operator can move the arm/camera until the object is visible and detection succeeds.

**Architecture:** Reuse the existing `/supervisor/cartesian_jog` service and `CartesianJog.srv` request shape. Keep target recognition data flow unchanged: the perception node continues publishing `/perception/object`, and the GUI only adds arm alignment controls plus status feedback.

**Tech Stack:** ROS Noetic, PyQt5, `alicia_flexible_grasp_supervisor.srv.CartesianJog`, Python `unittest`.

---

### Task 1: Add Testable Jog Request Mapping

**Files:**
- Modify: `gui/widgets/perception_widget.py`
- Test: `tests/test_perception_visual_alignment.py`

- [ ] **Step 1: Write the failing test**

```python
def test_visual_jog_values_map_axes_and_execute_flag():
    assert PerceptionWidget._visual_jog_values('X+', 0.005, True) == (0.005, 0.0, 0.0, 0.0, 0.0, 0.0, True)
    assert PerceptionWidget._visual_jog_values('Z-', 0.01, False) == (0.0, 0.0, -0.01, 0.0, 0.0, 0.0, False)
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `./devel/env.sh python3 -m unittest src/alicia_flexible_grasp_supervisor/tests/test_perception_visual_alignment.py`
Expected: FAIL because `_visual_jog_values` does not exist.

- [ ] **Step 3: Implement `_visual_jog_values`**

Add a static helper on `PerceptionWidget` that maps `X+`, `X-`, `Y+`, `Y-`, `Z+`, `Z-` to Cartesian deltas and preserves the `execute` flag.

- [ ] **Step 4: Run the test to verify it passes**

Run: `./devel/env.sh python3 -m unittest src/alicia_flexible_grasp_supervisor/tests/test_perception_visual_alignment.py`
Expected: PASS.

### Task 2: Add Perception Page Controls

**Files:**
- Modify: `gui/widgets/perception_widget.py`

- [ ] **Step 1: Add UI controls**

Add a “视觉对准控制” group with step options `1 mm`, `5 mm`, `10 mm`, a checkbox for executing motion, and six axis buttons.

- [ ] **Step 2: Call `/supervisor/cartesian_jog`**

Each button calls `_visual_jog_values`, sends `CartesianJog`, and updates the page status with the service response.

- [ ] **Step 3: Verify syntax and build**

Run:
`python3 -m py_compile src/alicia_flexible_grasp_supervisor/gui/widgets/perception_widget.py`
`catkin_make --pkg alicia_flexible_grasp_supervisor`

