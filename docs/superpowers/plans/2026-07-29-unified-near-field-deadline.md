# Unified Near-Field Deadline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> `superpowers:executing-plans` to implement this plan task-by-task.

**Goal:** Make the approved `30.0 s` near-field snapshot-and-selection budget
one phase-owned absolute deadline shared by the task, remote 6D selector, and
strict MoveIt planning services, and return a fresh exact terminal Preview
before the task can misclassify an unchecked tail.

**Architecture:** `GraspTaskNode` publishes one immutable
`NearFieldPlanningPhase` contract when the observation range first admits
near-field planning. The contract contains the phase start, absolute deadline,
and monotonically increasing task-local phase ID. The remote node binds the
current target epoch and direct snapshot to that contract, passes the same
deadline through both strict MoveIt services, and publishes either one valid
contact Preview or one fresh invalid terminal Preview. The task consumes only
a terminal Preview newer than its current phase start and carrying the
dedicated near-field terminal source marker.

**Tech Stack:** ROS Noetic, Python 3, catkin message/service generation,
`unittest`, existing `GraspTaskNode`, `RemoteGrasp6DNode`,
`MotionGateway`, and `MoveItPlanner`.

## Global Constraints

- Keep the complete near-field phase budget at `30.0 s`; do not add a second
  snapshot-based 30-second window.
- Do not remove or relax target, geometry, collision, joint-limit, complete
  sweep, strict MoveIt, plan-ID, controller, encoder, or endpoint gates.
- Do not report candidates that were not checked before the deadline as
  unreachable.
- A deadline may stop planning only; it must never publish `/grasp/stop`,
  torque-off, disable, controller-stop, or emergency-stop commands.
- All implementation and tests in this plan are offline. Powered validation
  requires a later explicit operator report that the arm is powered and
  aligned.

## Task 1: Generate the phase and deadline-aware service contracts

**Files:**

- Add: `src/alicia_flexible_grasp_supervisor/msg/NearFieldPlanningPhase.msg`
- Modify: `src/alicia_flexible_grasp_supervisor/CMakeLists.txt`
- Modify: `src/alicia_flexible_grasp_supervisor/srv/CheckPoseSequence.srv`
- Modify: `src/alicia_flexible_grasp_supervisor/srv/ResolveFreeSpaceOrientations.srv`

- [ ] Add an atomic phase message with `header`, `active`, `phase_id`, and
  absolute ROS `deadline`.
- [ ] Add an absolute ROS deadline request field to both planning-only MoveIt
  services. A zero deadline preserves non-near-field callers.
- [ ] Regenerate catkin messages/services before importing the changed
  interfaces in behavior tests.

## Task 2: Prove the old task/remote timing behavior fails

**Files:**

- Modify:
  `src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py`
- Modify:
  `src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py`

- [ ] Add a task test proving the active phase contract owns exactly one
  `start + 30.0 s` deadline.
- [ ] Add a task test proving a fresh invalid Preview with
  `candidate_source=near_field_terminal` ends the wait immediately with its
  exact allowlisted code.
- [ ] Replace the snapshot-based direct deadline test with a phase-owned
  deadline test.
- [ ] Add remote tests proving the same deadline is copied into both strict
  MoveIt service requests and that a direct terminal publishes one invalid
  Preview bound to the current ticket.
- [ ] Run the focused tests and record RED failures against the old
  production behavior.

## Task 3: Implement task/remote shared-deadline behavior

**Files:**

- Modify:
  `src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py`
- Modify:
  `src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py`

- [ ] Publish the structured contract at near-field entry and retain the
  legacy Bool only as a compatibility signal.
- [ ] Make the direct task wait use the published absolute deadline, including
  stream-start time, instead of starting a new monotonic stopwatch.
- [ ] Bind direct remote selection to the current structured phase deadline,
  not the later fused snapshot timestamp.
- [ ] Pass the exact deadline to strict sequence planning and deterministic
  free-space orientation resolution.
- [ ] Publish a fresh invalid Preview for the exact direct terminal. The task
  may consume only a current-window terminal source/code and must never treat
  an invalid plan as executable authority.

## Task 4: Make MoveIt planning services deadline-aware

**Files:**

- Modify:
  `src/alicia_flexible_grasp_supervisor/scripts/motion_gateway_node.py`
- Modify:
  `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/moveit_planner.py`
- Modify:
  `src/alicia_flexible_grasp_supervisor/tests/test_motion_gateway_controller_start.py`
- Modify:
  `src/alicia_flexible_grasp_supervisor/tests/test_moveit_planner_pose_feedback.py`

- [ ] Forward a finite positive deadline through both planning-only service
  handlers.
- [ ] Check remaining time before and after each strict sequence stage.
- [ ] Check remaining time before each deterministic orientation IK sample and
  the repeatability solve; clamp each IK request timeout to the remaining
  phase budget.
- [ ] Return `MOVEIT_TIMEOUT` with the exact current stage when the deadline is
  consumed. Do not convert an unchecked remainder into
  `MOVEIT_UNREACHABLE`.
- [ ] Keep zero-deadline legacy callers behaviorally unchanged.

## Task 5: Verify, document, commit, and push

- [ ] Run focused task, remote, gateway, and planner tests.
- [ ] Run the complete supervisor test discovery, Python compilation,
  message/service generation build, YAML/XML parsing where affected, and
  `git diff --check`.
- [ ] Append RED/GREEN evidence and implementation details to the runtime log.
- [ ] Update the technical-route status only to “offline implemented”; do not
  claim powered contact or lift.
- [ ] Commit intentionally and push
  `origin/codex/protocol-v3-upgrade`.

