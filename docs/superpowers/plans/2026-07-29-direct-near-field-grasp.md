# Direct Near-Field Grasp Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the production second-stage 450-second multi-request/MuJoCo loop with one fused near-field snapshot that freezes the first hard-safe, strictly MoveIt-reachable candidate within 30 seconds and proceeds directly to contact execution.

**Architecture:** Keep the first observation stage and every physical hard gate. Add an explicit `single_snapshot_direct` policy at the remote selection boundary so current request observations replace cross-request tracker authority, MoveIt stops at the first authoritative reachable result, and MuJoCo is not called. Make the task recognize that policy, avoid duplicate simulation/final refinement, freeze one plan, stop candidate streaming, and execute the existing pregrasp/approach/grasp/close/lift chain.

**Tech Stack:** ROS 1 Noetic, Python 3, pytest/unittest, MoveIt service interfaces, YAML.

## Global Constraints

- The far-field observation move and one measured radial correction remain unchanged.
- Near field uses exactly one configured multi-frame fused RGB-D/mask/object snapshot.
- Geometry, support clearance, finite-data, gripper CAD, collision, joint-limit, strict MoveIt, target-freshness, plan-ID, and measured-endpoint gates remain mandatory.
- Direct mode does not require cross-request candidate stability.
- Direct mode checks candidates in authoritative rank order and stops on the first strict MoveIt success.
- Direct mode does not call the remote MuJoCo selector or task-level duplicate simulation.
- Direct mode skips post-rebind final visual refinement because the near-field snapshot is the visual correction.
- The direct near-field snapshot-and-selection budget is `30.0 s`.
- Legacy selection remains available only through an explicit non-production strategy.
- No failure path added here sends torque-off, stop, disable, controller-stop, or emergency-stop.

---

### Task 1: Encode the production direct strategy

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_graspnet_input_default_config.py`

**Interfaces:**
- Produces: `/grasp/near_field_strategy=single_snapshot_direct` and direct-mode production defaults consumed by both ROS Python nodes.

- [x] **Step 1: Change the persisted-config test first**

Replace the 450-second and enabled-final-refine assertions with:

```python
self.assertEqual(
    grasp['near_field_strategy'],
    'single_snapshot_direct',
)
self.assertEqual(grasp['near_field_replan_timeout_sec'], 30.0)
self.assertFalse(grasp['final_visual_refine_enabled'])
self.assertFalse(grasp['final_visual_refine_required'])
```

Keep all existing assertions for snapshot frame count, target validity,
geometry, collision, joint, and endpoint settings.

- [x] **Step 2: Run and verify the test fails**

```bash
python3 -m unittest src.alicia_flexible_grasp_supervisor.tests.test_graspnet_input_default_config -v
```

Expected: the current `450.0`, `true`, and missing strategy values fail.

- [x] **Step 3: Set production YAML**

Write:

```yaml
near_field_strategy: single_snapshot_direct
near_field_replan_timeout_sec: 30.0
final_visual_refine_enabled: false
final_visual_refine_required: false
```

Retain the legacy tuning values below them for explicit legacy-mode tests; they
have no production authority while the direct strategy is selected.

- [x] **Step 4: Run and pass the config test**

Run the command from Step 2.

Expected: all persisted-config tests pass.

- [x] **Step 5: Commit the production policy**

```bash
git add src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml src/alicia_flexible_grasp_supervisor/tests/test_graspnet_input_default_config.py
git commit -m "config: select direct near-field grasp strategy"
```

---

### Task 2: Make one near-field request authoritative and stop at first reachable

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py`

**Interfaces:**
- Consumes: `prepared.near_field`, `self.near_field_strategy`, current request observations, existing `_recheck_and_score_stable`, and `bounded_moveit_select`.
- Produces: one `PREVIEW_READY` direct candidate or a precise bounded direct-mode status.

- [ ] **Step 1: Add failing direct-mode selection tests**

Construct a near-field node fixture with:

```python
node.near_field_strategy = 'single_snapshot_direct'
node.moveit_top_n = 24
node.candidate_max_joint_delta_rad = 0.0
```

Provide three current hard-safe observations in authoritative order. Stub
`bounded_moveit_select` to capture arguments and select the second candidate.
Assert:

```python
assert captured['exhaustive'] is False
assert captured['first_reachable_by_rank'] is True
assert captured['continue_checking'] is not None
assert captured['continuation_stop_reason'] == 'NEAR_FIELD_DIRECT_TIMEOUT'
assert mujoco_calls == []
```

Set the tracker to return no cross-request stable candidates while the current
request has valid observations, then assert direct mode still reaches
`_recheck_and_score_stable` with those current observations. Verify
`snapshot_evidence.disjoint_window_required` is false and
`tracking_evidence.required_hits` is `1`.

Add failure assertions:

- no current observations -> `NEAR_FIELD_NO_HARD_SAFE_CANDIDATE`;
- deadline before a reachable result -> `NEAR_FIELD_DIRECT_TIMEOUT`;
- checked current candidates but none reachable ->
  `NEAR_FIELD_NO_REACHABLE_CANDIDATE`.

- [ ] **Step 2: Run the focused streaming tests and verify failure**

```bash
pytest -q \
  src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py \
  -k "direct_near_field or near_field_selection_policy"
```

Expected: direct policy attributes and statuses do not exist.

- [ ] **Step 3: Add the explicit strategy helpers**

Implement:

```python
def _near_field_strategy(self):
    return str(
        getattr(self, 'near_field_strategy', 'legacy_gated') or
        'legacy_gated'
    ).strip().lower()

def _direct_near_field_active(self, prepared):
    return (
        bool(getattr(prepared, 'near_field', False))
        and self._near_field_strategy() == 'single_snapshot_direct'
    )
```

Load `self.near_field_strategy` from
`/grasp/near_field_strategy` during node initialization. Legacy is the fallback
for isolated old fixtures and explicit experiments.

- [ ] **Step 4: Select from current fused-request evidence**

Inside `_accept_prediction`, define:

```python
direct_near_field = self._direct_near_field_active(prepared)
```

Still update the tracker for audit continuity. For direct near field, set:

```python
stable = tuple(observations)
```

For legacy near field, retain the existing `NEAR_FIELD_STABILITY_MIN_HITS`
filter. Record:

```python
local_funnel['snapshot_evidence']['disjoint_window_required'] = (
    near_field and not direct_near_field
)
tracking_evidence['required_hits'] = (
    1 if direct_near_field else existing_required_hits
)
```

When direct mode has zero current observations, publish
`NEAR_FIELD_NO_HARD_SAFE_CANDIDATE` without reusing a retained track.

- [ ] **Step 5: Change only direct-mode MoveIt and MuJoCo authority**

Call `bounded_moveit_select` with:

```python
exhaustive=bool(near_field and not direct_near_field),
first_reachable_by_rank=bool(
    direct_near_field or not near_field
),
continue_checking=(
    self._direct_near_field_deadline_gate(prepared)
    if direct_near_field
    else self._near_field_moveit_continuation_gate(prepared)
    if near_field
    else None
),
continuation_stop_reason=(
    'NEAR_FIELD_DIRECT_TIMEOUT'
    if direct_near_field
    else 'MUJOCO_SNAPSHOT_RESERVE_REACHED'
),
```

The direct deadline gate uses the prepared snapshot/request start time and the
configured `30.0 s` total budget; it returns false before starting another
MoveIt check once the budget is consumed. Call
`_screen_near_field_selection_with_mujoco` only when
`near_field and not direct_near_field`.

Map the remaining no-selection outcomes exactly:

```python
if direct_near_field and selection.selected is None:
    status = (
        'NEAR_FIELD_DIRECT_TIMEOUT'
        if selection.terminated_early
        else 'NEAR_FIELD_NO_REACHABLE_CANDIDATE'
    )
```

- [ ] **Step 6: Run the full streaming suite**

```bash
pytest -q src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py
```

Expected: direct and legacy behavior both pass; legacy MuJoCo tests remain
unchanged.

- [ ] **Step 7: Commit remote direct selection**

```bash
git add src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py
git commit -m "fix: select first reachable near-field grasp"
```

---

### Task 3: Freeze one direct plan and remove duplicate task authority

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py`

**Interfaces:**
- Consumes: a contact-execution preview published by Task 2.
- Produces: one frozen near-field plan followed by the existing pregrasp, linear approach, grasp, close, and lift chain.

- [ ] **Step 1: Replace the old MuJoCo expectation with direct-mode tests**

Create a direct-mode rebind test based on
`test_near_field_replan_binds_preview_and_runs_mujoco_gate`, but configure:

```python
gcfg = {
    'near_field_strategy': 'single_snapshot_direct',
    'near_field_replan_enabled': True,
    'near_field_replan_required': True,
    'near_field_replan_timeout_sec': 30.0,
    'near_field_replan_snapshot_slack_sec': 1.0,
    'plan_validity_sec': 5.0,
    'target_max_drift_m': 0.02,
    'target_observation_validity_sec': 1.5,
}
```

Assert the direct plan is frozen and returned while a simulation stub that
raises is never called. Add an execution test where a final-refine stub also
raises, and assert recorded labels are:

```python
[
    '6D near-field pregrasp',
    'linear 6D approach',
    'linear 6D grasp pose',
    'linear 6D lift',
]
```

Record one gripper-close call between grasp and lift. Add a request-stream
stub and assert it receives `True` once at near-field entry and `False` once
immediately after the exact plan is frozen.

- [ ] **Step 2: Run focused task tests and verify failure**

```bash
python3 -m unittest \
  src.alicia_flexible_grasp_supervisor.tests.test_grasp_task_sequence \
  -v
```

Expected: current code calls duplicate simulation/final refinement and does not
stop inference after freezing.

- [ ] **Step 3: Add direct-strategy branching**

Implement:

```python
@staticmethod
def _near_field_strategy(gcfg):
    return str(
        gcfg.get('near_field_strategy', 'legacy_gated') or
        'legacy_gated'
    ).strip().lower()

def _direct_near_field_enabled(self, gcfg):
    return (
        self._near_field_strategy(gcfg) ==
        'single_snapshot_direct'
    )
```

In `_maybe_rebind_near_field_grasp6d_plan`, once the direct candidate is
validated and frozen:

1. do not call `_simulate_grasp6d_plan_if_required`;
2. call a generalized `_set_near_field_preview_stream(gcfg, False)`;
3. return the frozen plan immediately.

Rename `_request_near_field_preview_stream` to accept the desired boolean and
call `/grasp_6d/request_plan` with `TriggerZero(request)` semantics already
used by the service.

- [ ] **Step 4: Skip only the redundant final-refine layer**

In `_execute_grasp6d_plan`, direct mode sets:

```python
refined = plan
```

instead of calling `_maybe_final_refine_grasp6d_plan`. Keep every existing
`_execution_checkpoint`, pregrasp comparison, strict linear movement,
gripper-close, and lift call unchanged. Preserve legacy behavior when the
strategy is not direct.

If direct mode times out, map the task result to
`NEAR_FIELD_DIRECT_TIMEOUT`; do not emit the legacy generic
`NEAR_FIELD_REPLAN_TIMEOUT`.

- [ ] **Step 5: Run the complete task suite**

```bash
python3 -m unittest src.alicia_flexible_grasp_supervisor.tests.test_grasp_task_sequence -v
```

Expected: direct tests and all legacy task sequencing tests pass.

- [ ] **Step 6: Commit task authority simplification**

```bash
git add src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py
git commit -m "fix: execute frozen near-field plan directly"
```

---

### Task 4: Verify the direct route and update its dated history

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/docs/grasp_task_technical_route.md`
- Modify: `src/alicia_flexible_grasp_supervisor/logs/2026-07-23-ros-latest-node-launch.md`

**Interfaces:**
- Consumes: Tasks 1-3 source, tests, and configuration.
- Produces: offline implementation evidence and the dated current-route status.

- [ ] **Step 1: Run complete affected offline verification**

```bash
pytest -q \
  src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py \
  src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py
python3 -m unittest \
  src.alicia_flexible_grasp_supervisor.tests.test_grasp_task_sequence \
  src.alicia_flexible_grasp_supervisor.tests.test_graspnet_input_default_config \
  -v
python3 -m py_compile \
  src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py \
  src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py
git diff --check
```

Expected: all suites pass, both scripts compile, and whitespace validation is
silent.

- [ ] **Step 2: Verify configuration syntax**

```bash
python3 -c "import pathlib,yaml; yaml.safe_load(pathlib.Path('src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml').read_text())"
```

Expected: exit code `0`.

- [ ] **Step 3: Update route and runtime records**

Change the route status to “近场直抓离线实现已验证，待真机验证” and append a
`2026-07-29（近场直抓离线实现）` entry without deleting the initial dated
entry. Append exact test counts, behavior changes, failure-code coverage, and
the absence of ROS/hardware commands to the existing runtime log.

- [ ] **Step 4: Commit verification documentation**

```bash
git add src/alicia_flexible_grasp_supervisor/docs/grasp_task_technical_route.md src/alicia_flexible_grasp_supervisor/logs/2026-07-23-ros-latest-node-launch.md
git commit -m "docs: record direct near-field verification"
```
