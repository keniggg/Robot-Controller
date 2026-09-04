# Endpoint trim continuity verification — 2026-09-03

## Scope and safety status

This is offline-only evidence. WSL and all hardware interfaces were shut down.
No ROS master/node, driver executable, serial/camera/controller interface, or
robot process was started; no ROS topic/service was published/called. There was
no powered deployment, restart, hot-load, or grasp.

Powered acceptance remains blocked. Do not restart or hot-load the real driver
while a motion command is active, and do not run a new grasp from this endpoint
plan alone. The class-agnostic precontact-geometry plan must first be green,
then the operator must later realign the target, and only then may a fresh-plan
powered acceptance be considered.

## Effective contract and transition record

The effective SDK quantum is `q = 2*pi/4096 rad` (`0.0015339807878856412 rad`).
The driver defaults are maximum step `4q` (`0.006135923151542565 rad`), minimum
directional response `2q` (`0.0030679615757712823 rad`), and response deadline
`1.0 s`. The clean committed endpoint arbiter owns a `0.25 s` GUI-edit/task
holdoff/sync boundary. An integrated broader GUI guard may impose a stricter
rule, but may never bypass that arbiter boundary.

```text
IDLE --activate--> ACTIVE_READY --bounded correction--> WAITING_RESPONSE
  ^                    ^                 | settled + stable          | release
  | explicit GUI       +-----------------+                            v
  +----------------------------------------------------------- PENDING_RELEASE
ACTIVE_READY --release--> QUIESCENT
PENDING_RELEASE --settle or deadline--> QUIESCENT
WAITING_RESPONSE --deadline without release--> FAULT
```

An explicit GUI handoff atomically installs the GUI target, clears task trim
state, and returns to `IDLE`. A normal settled release emits
`ENDPOINT_TRIM_RELEASE_SETTLED`; a deadline release emits
`ENDPOINT_TRIM_RESPONSE_TIMEOUT`; an invalid/recomposition path emits
`ENDPOINT_TRIM_CONTINUITY_VIOLATION`. Release API statuses are separately
`PENDING`, `COMPLETED`, `NOOP`, or `REJECTED`.

The live-log-derived regression input is a six-joint reference, a first Joint2
correction of `+1.3 deg`, an attempted second correction that would reach
`+2.3 deg` while the response remains outstanding, and lease release `0.37 s`
later. It requires serialized ownership, `PENDING_RELEASE`, and a composed
target that neither jumps nor reverses. Historical observations of two trim
iterations at `0.015748` then `0.009476 rad`, plus a `0.46 s` backwards ROS
time jump and stale feedback, are diagnostic regression inputs only.

## Commands and results

| Command | Exit | Result |
| --- | ---: | --- |
| `source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_ENABLE_TESTING=ON -j2` | 0 | Configure/build succeeded; no test count is emitted by this build command. |
| `source /opt/ros/noetic/setup.bash && catkin_make run_tests_alicia_d_driver_gtest_actuation_confirmation_test` | 0 | `46/46` passed: ActuationConfirmation `11`, GuiDirectHold `2`, EndpointTrimContinuity `16`, EndpointTrimDriverAdmission `5`, EndpointTrimOwnershipTransition `5`, EndpointTrimDriverOrchestration `7`. |
| `python3 -m unittest src.alicia_flexible_grasp_supervisor.tests.test_serial_driver_resilience -q` | 0 | `27/27` passed in `0.043 s`. |
| `git diff --check` | 0 | No whitespace errors in the dirty worktree after the Task 4 documentation append. |

For completeness, the full supervisor discovery command was run with the
generated workspace on `PYTHONPATH`:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 -m unittest discover -s src/alicia_flexible_grasp_supervisor/tests -q
```

It exited `1`: `725` tests ran in `2.226 s`, with `2` failures and `3` errors.
This is not claimed green. Two errors are unrelated sandbox policy failures:
the GraspNet mock-server tests cannot bind `127.0.0.1`. The three remaining
results are old brittle source-text checks in
`test_moveit_trajectory_execution_config`: two failures and one `IndexError`
expect the pre-coordinator timer-local endpoint implementation. They are
traceable to the endpoint-coordinator refactor's obsolete test expectations,
not a demonstrated focused endpoint behavior failure; the behavior-focused
driver target above remains `46/46` green. No failure was masked or changed.

The same discovery command without `source devel/setup.bash` exited `1` after
`269` tests with `2` failures and `55` import errors because generated ROS
Python packages were unavailable; it is an invalid environment invocation,
not a product result.

## Gate decision

The evidence records endpoint continuity only. It does not authorize a powered
deployment. Keep all hardware interfaces down until the class-agnostic
precontact-geometry plan is green and the operator has subsequently realigned
the target; then require a fresh plan and a separately authorized acceptance.
