# Endpoint trim continuity verification — 2026-09-03

## Current accepted result — 2026-09-04

Behavior commit: `349255c1c41fd779cf39a4727ef0862c7019cfd0`.
Final independent review found no remaining Critical/Important issue in the
endpoint scope. The definitive response goal is the immutable
`measured_baseline + actual_applied_step`, using saturating arithmetic.
The applied step includes both the per-step and accumulated-offset limits.
The SDK target remains `reference + offsets`; the next increment is admitted
only after fresh directional feedback has settled around the incremental goal.

The deployed-parameter regression uses `4q` maximum step, `2q` response minimum,
`2q` settle band, `0.30 s` stability, and `1.0 s` response deadline. A synthetic
fixed `1.3 deg` command-to-feedback bias converges in four positive increments:
`0.3515625`, `0.3515625`, `0.3515625`, and `0.2453125 deg`.
Each step waits through the stability interval; at convergence the next step
is zero and the SDK target is preserved. This is an ideal fixed-bias unit-test
model, not a measurement of hardware dynamics, speed, or grasp success.

Exact commit `349255c` was extracted with `git archive` into
`/tmp/endpoint-349255c-26oNVG`. The archive's C++ source was compiled against its
own headers with C++14 and the installed GoogleTest sources. Python ran from
the archive with ROS Noetic and the workspace's generated message environment.

| Verification | Result |
| --- | --- |
| Clean behavior-commit C++ tests | `47/47`, exit `0` |
| Clean MoveIt/source and serial contracts | `14 + 25 = 39/39`, exit `0` |
| Clean full Python discovery | `701/701`, exit `0` (`2.165 s`) |
| Integrated build and C++ tests | build exit `0`; `49/49`, exit `0` |
| Integrated full Python discovery | `725/725`, exit `0` (`2.198 s`) |

The first sandboxed clean discovery had exactly two socket-permission errors.
It was then rerun with approved localhost socket access: both mock HTTP tests
and the complete discovery passed. The integrated discovery also used that
approval. Only local test servers were started; ROS and hardware interfaces
were not started or called. Test log messages about motion/enable paths are
outputs of mocked cases, not actions on the robot.

Earlier sections below retain the investigation history and historical counts.
They are superseded by this result where they describe response-goal behavior
or incomplete full-suite verification. In particular, the intermediate sibling
`c546417` is not the evidence identifier for the accepted behavior.

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
FAULT --explicit GUI handoff--> IDLE
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

## Historical corrective verification — deployed response threshold

The original continuity test constructed a bare `EndpointTrimConfig`, whose
header default `response_min_rad` is `q`. The node's deployed parameter wiring
instead uses `2q`. The direct behavioral gtest now instantiates the deployed
values: `max_step_rad = 8*pi/4096` (`4q`), `response_min_rad = 4*pi/4096`
(`2q`), `response_deadline_sec = 1.0`, `stable_sec = 0.30`, and
`settle_error_rad = 4*pi/4096` (`2q`). The fixture starts from an existing
`-1q` trim, so after the new correction the `1q` directional feedback sample
is already exactly at the `2q` settle-error boundary. Repeating that `1q`
sample after `0.31 s` still leaves the coordinator in `WAITING_RESPONSE`;
therefore the response-minimum gate, rather than the settle-error gate, is
what rejects it. A `2q` response then begins settling, stays waiting at
`0.29 s`, and reaches `ACTIVE_READY` only after `0.31 s`. This is a
coordinator-only unit test; it does not dispatch a driver command or contact
hardware.

The outdated source-contract tests were reproduced before repair with exit
`1`: `14` ran, with `2` failures and `1` `IndexError`. They expected the
deleted timer-local trim algorithm. They now assert the committed
`EndpointTrimContinuity` / `EndpointTrimDriverAdmission` architecture,
parameter wiring, and feedback/correction/stream ordering without brittle
formatting assumptions.

| Command | Exit | Result |
| --- | ---: | --- |
| `source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_ENABLE_TESTING=ON -j2 && catkin_make run_tests_alicia_d_driver_gtest_actuation_confirmation_test` | 0 | Rebuilt and ran `47/47`: ActuationConfirmation `11`, GuiDirectHold `2`, EndpointTrimContinuity `17`, EndpointTrimDriverAdmission `5`, EndpointTrimOwnershipTransition `5`, EndpointTrimDriverOrchestration `7`. |
| staged test source compiled directly with `g++`, then `/tmp/endpoint_task4_staged_test` | 0 | The isolated staged snapshot ran `45/45`: the same endpoint suites, excluding the two unrelated unstaged GUI-hold tests. |
| `source /opt/ros/noetic/setup.bash && source devel/setup.bash && python3 -m unittest src.alicia_flexible_grasp_supervisor.tests.test_moveit_trajectory_execution_config -q` | 0 | `14/14` passed. |
| `source /opt/ros/noetic/setup.bash && source devel/setup.bash && python3 -m unittest src.alicia_flexible_grasp_supervisor.tests.test_serial_driver_resilience -q` | 0 | `27/27` passed. |
| `source /opt/ros/noetic/setup.bash && source devel/setup.bash && python3 -m unittest discover -s src/alicia_flexible_grasp_supervisor/tests -q` | 1 | `725` ran in `2.328 s`; no endpoint source-contract failures remain. The only two errors are sandbox-denied `127.0.0.1` binds in the GraspNet mock-server tests. |

The full suite is therefore not claimed green. The two remaining errors are
environmental sandbox restrictions, not endpoint-commit failures. This
corrective evidence remains offline-only and does not alter the powered
deployment gate above.

## Corrective verification — unconfounded threshold and release ordering

The deployed-threshold fixture now makes the `1q` sample satisfy the
`settle_error_rad = 2q` condition before testing the independent
`response_min_rad = 2q` condition. Both task-release paths also assert that
`EndpointTrimContinuity::request_release` appears before
`EndpointTrimCommandOrder::record_release`, so the order object records the
decision that was just produced rather than stale state.

| Command | Exit | Result |
| --- | ---: | --- |
| `source /opt/ros/noetic/setup.bash && catkin_make run_tests_alicia_d_driver_gtest_actuation_confirmation_test` | 0 | Integrated dirty-tree run passed `47/47`, including the unconfounded deployed-threshold case. |
| staged test source compiled directly with `g++`, then `/tmp/endpoint_task4_round2_staged_test` | 0 | Isolated staged snapshot passed `45/45`; unrelated GUI-hold hunks remained unstaged. |
| `source devel/setup.bash && python3 -m unittest src.alicia_flexible_grasp_supervisor.tests.test_moveit_trajectory_execution_config -q` | 0 | `14/14` passed, including both release-order assertions. |
| `source devel/setup.bash && python3 -m unittest src.alicia_flexible_grasp_supervisor.tests.test_serial_driver_resilience -q` | 0 | `27/27` passed. |
| `git diff --check` | 0 | No whitespace errors. |

## Historical final-review attempt — reference settle and sticky fault

The intermediate `7d43529` attempt used the full upstream reference as the
response goal. Review rejected that goal because a correct first `4q` response
cannot reach the reference when the initial error is `1.3 deg`. Its two
single-response tests were replaced by the deployed multi-increment regression
in `349255c`; the accepted goal is `measured_baseline + actual_applied_step`.
The following counts are historical and do not establish acceptance of that
intermediate control law.

Timeout `FAULT` is sticky across task-controller handoff. A valid task target
cannot change the preserved target or phase; malformed task input still fails
closed. Only an explicit GUI handoff clears trim fault/history and installs its
exact target in `IDLE`. Every unchanged decision now reports a zero
`applied_step`, while the release-generation tests continue to prove terminal
release delivery is exact-once.

The committed MoveIt source contract no longer depends on the operator-owned,
unstaged GUI gesture/one-hot fields. It checks only committed GUI source
classification, accepted-boundary explicit handoff ordering, task-lease
clearance, and the arbiter's inclusive `0.25 s` edit holdoff plus explicit sync
release.

| Environment and command | Exit | Result |
| --- | ---: | --- |
| current integrated worktree: `catkin_make -DCATKIN_ENABLE_TESTING=ON -j2` | 0 | Configure/build succeeded without starting a ROS node or hardware process. |
| current integrated worktree: focused driver gtest | 0 | `50/50` passed: `11` actuation, `2` operator-owned GUI hold, `20` endpoint continuity, `5` admission, `5` ownership transition, `7` orchestration. |
| clean `git archive` snapshot of `c546417`: focused driver gtest | 0 | `48/48` passed; the two unrelated unstaged GUI-hold tests are absent. |
| clean snapshot: MoveIt source contracts | 0 | `14/14` passed. |
| clean snapshot: serial contracts | 0 | `25/25` passed. |
| current integrated worktree: serial contracts | 0 | `27/27` passed, including two operator-owned unstaged cases. |
| clean snapshot: full supervisor discovery | 1 | `701` ran in `2.099 s`; the only two errors are sandbox-denied `127.0.0.1` mock-server binds. |
| current integrated worktree: full supervisor discovery | 1 | `725` ran in `2.265 s`; again only the same two sandbox bind errors remain. |
| `git diff --check` | 0 | No whitespace errors. |

Neither full discovery is claimed green: both remaining errors are separated
environmental sandbox restrictions. All corrective work and verification was
offline-only. The powered deployment gate in this document is unchanged.
