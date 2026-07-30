# Observation Radial-Correction Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let a strictly submitted far-field radial correction whose controller reports failure proceed only when settled, fresh camera evidence proves that the same target now satisfies the unchanged observation-range contract.

**Architecture:** Reuse the existing observation-only recovery path in `GraspTaskNode._plan_and_execute_pose`; do not add a second recovery mechanism. The radial-correction caller will opt into that path, after which the existing task flow takes a fresh post-correction observation and either enters near field or fails with the existing range/target reason.

**Tech Stack:** ROS Noetic, Python 3, `unittest`, existing `GraspTaskNode` and `TriggerZero`/`StartGrasp` ROS interfaces.

## Global Constraints

- Keep `/robot/execution_goal_tolerance_rad=0.03` and `/robot/execution_goal_tolerance_slack_rad=0.005`; do not accept joint endpoint error above `0.035 rad`.
- Keep `/alicia_d_driver_node/endpoint_feedback_trim_enabled=false`.
- Permit controller-failure recovery only for a frozen `FAR_FIELD_OBSERVATION_PLAN`; never apply it to near-field pregrasp, approach, grasp, gripper close, or lift.
- Keep the existing limit of at most one measured radial correction.
- A recoverable controller result must still wait for settled feedback; the following fresh observation must match the frozen target and fall inside the existing inclusive camera-range contract.
- Never publish `/grasp/stop`, torque-off, disable, controller stop, or emergency-stop commands.
- Runtime deployment may replace only `/grasp_task_node`; do not restart the driver or clear measured actuation confirmation.

---

### Task 1: Wire the radial correction into the existing observation contract

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py:1379`
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py:4624`

**Interfaces:**
- Consumes: `GraspTaskNode._plan_and_execute_pose(..., allow_post_failure_observation_validation: bool) -> bool`.
- Produces: `_maybe_execute_observation_camera_retreat(...)` requests observation-only post-failure validation for its one strict radial-correction trajectory.

- [ ] **Step 1: Change the focused test to state the approved behavior**

In `test_observation_retreat_executes_one_strict_preflight_and_move`, replace the old assertion with:

```python
self.assertTrue(
    execute_kwargs[0][
        'allow_post_failure_observation_validation'
    ]
)
```

This test changes only the expected contract at the radial-correction call boundary. Existing tests continue to prove that `_plan_and_execute_pose` accepts only the exact cached-plan failure prefix, waits for feedback settle, and records the observation failure marker.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
python3 -m unittest \
  src.alicia_flexible_grasp_supervisor.tests.test_grasp_task_sequence.GraspTaskSequenceTest.test_observation_retreat_executes_one_strict_preflight_and_move \
  -v
```

Expected: `FAIL` because the production caller still supplies
`allow_post_failure_observation_validation=False`.

- [ ] **Step 3: Implement the minimal production change**

In `_maybe_execute_observation_camera_retreat`, change only the final
`_plan_and_execute_pose` argument:

```python
allow_post_failure_observation_validation=True,
```

Do not change `_plan_and_execute_pose`, the error-prefix classifier, the
range contract, joint tolerances, correction count, or any contact-stage call.

- [ ] **Step 4: Run the focused behavior and boundary tests and verify GREEN**

Run:

```bash
python3 -m unittest \
  src.alicia_flexible_grasp_supervisor.tests.test_grasp_task_sequence.GraspTaskSequenceTest.test_observation_retreat_executes_one_strict_preflight_and_move \
  src.alicia_flexible_grasp_supervisor.tests.test_grasp_task_sequence.GraspTaskSequenceTest.test_far_field_observation_may_defer_controller_failure_to_live_range \
  src.alicia_flexible_grasp_supervisor.tests.test_grasp_task_sequence.GraspTaskSequenceTest.test_observation_correction_is_blocked_after_controller_failure \
  -v
```

Expected: all three tests pass. The third test must continue to prove that a
failed main observation whose fresh distance is still out of range cannot
issue another physical correction.

- [ ] **Step 5: Run the complete task regression and static checks**

Run:

```bash
python3 -m unittest \
  src.alicia_flexible_grasp_supervisor.tests.test_grasp_task_sequence \
  -q
python3 -m py_compile \
  src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py \
  src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py
git diff --check
```

Expected: the complete task suite passes, both Python files compile, and
`git diff --check` is silent.

- [ ] **Step 6: Commit the tested implementation**

```bash
git add \
  src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py \
  src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py
git commit -m "fix: validate failed observation correction by live range"
```

### Task 2: Deploy only the task node and resume the live proof

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/logs/2026-07-23-ros-latest-node-launch.md`
- Modify only if the effective route changes: `src/alicia_flexible_grasp_supervisor/docs/grasp_task_technical_route.md`

**Interfaces:**
- Consumes: the tested `grasp_task_node.py`, existing ROS master, driver, camera, perception, remote 6D node, `/grasp_6d/request_plan`, and `/grasp/start`.
- Produces: a new live audit showing whether the failed correction settles into the unchanged observation range and whether the one-shot near-field/contact sequence completes.

- [ ] **Step 1: Record the offline RED/GREEN evidence**

Append the failing test result, minimal one-line production change, focused
test result, complete task-suite result, compile result, and diff check to the
existing runtime log. State explicitly that offline commands sent no ROS or
hardware command.

- [ ] **Step 2: Replace only `/grasp_task_node`**

Source the current worktree and start the updated node with the same ROS name:

```bash
source devel/setup.bash
rosrun alicia_flexible_grasp_supervisor grasp_task_node.py \
  __name:=grasp_task_node
```

Run it in a supervised PTY. ROS name replacement may shut down the old task
process, but must not call `/grasp/stop` and must not restart or kill the
driver, camera, MoveIt, controller, or remote 6D node.

- [ ] **Step 3: Restore the approved runtime-only calibration setting**

The parameter should remain in the ROS parameter server across task-node
replacement. If the updated node reports the persisted interlock, set only:

```bash
rosparam set /grasp/calibration_interlock_active false
```

Do not edit the persisted YAML default and do not send a hardware command.

- [ ] **Step 4: Generate and freeze a new far-field plan**

Use the already authorized remote endpoint:

```bash
rosservice call /grasp_6d/request_plan "trigger: true"
```

Wait for a fresh valid `/grasp_6d/plan_enriched`, then freeze inference without
clearing Execution readiness:

```bash
rosservice call /grasp_6d/request_plan "trigger: false"
```

Read the retained `/grasp_6d/plan_enriched/plan_id`. Never reuse
`d94be0261d64908dd67e2f38` or another pre-restart plan.

- [ ] **Step 5: Execute the exactly frozen plan**

Copy the exact literal plan ID printed in Step 4 into the `plan_id` field of
one `/grasp/start` request. Construct that command only after the value is
known; do not use shell substitution, an unresolved variable, an old ID, or
an empty ID. The request shape is `execute: true` plus that exact literal
`plan_id`.

Supervise `/grasp/state`, `/grasp/near_field_active`,
`/alicia_d/actuation_status`, `/joint_states`, task-node logs, remote 6D
metrics, and MoveIt/controller results. Do not call `/grasp/stop` on failure.

- [ ] **Step 6: Decide the live result only from the existing contracts**

For a radial-correction controller failure:

- wait for the task's settle logic;
- require a newer observation of the same target;
- continue only if the measured camera-target distance is inside the existing
  inclusive range;
- otherwise retain the exact failure and do not issue a second correction.

For near-field/contact execution, require the existing plan-ID, collision,
strict-MoveIt, controller, encoder, endpoint, close, and lift evidence without
using the observation-only recovery.

- [ ] **Step 7: Record, verify, commit, and push**

Append the complete live timeline and exact terminal evidence to the runtime
log. Update the technical-route status/changelog only if the effective route
or its validation state changed. Then run:

```bash
git diff --check
git status --short
git add \
  src/alicia_flexible_grasp_supervisor/logs/2026-07-23-ros-latest-node-launch.md \
  src/alicia_flexible_grasp_supervisor/docs/grasp_task_technical_route.md
git commit -m "docs: record observation recovery validation"
git push origin codex/protocol-v3-upgrade
```

If only the log changed, omit the unchanged technical-route path from
`git add`. Claim successful grasp only if the target is visibly carried off
the support surface through the lift endpoint and the full audit is correlated
to the fresh frozen plan.
