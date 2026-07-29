# Actuation Confirmation and Direct Near-Field Grasp Design

Status: approved in conversation on 2026-07-29.

## Problem statement

Two independent failures were reproduced on the powered real arm.

First, a positive torque-enable request was accepted by the ROS driver at
ROS time `1785316280.782145890`. The driver continued writing the GUI target
with Joint2 near `-28.8 deg`, while fresh encoder feedback remained near
`-16.8 deg`. The firmware status stayed at `0xE1`. Later, the same measured
temperature channel remained at `60-63 C` for at least three consecutive
samples. The driver nevertheless published `/alicia_d/motion_enabled=true`
immediately after writing the torque-on frame. The protocol exposes no
positive torque acknowledgement, so this topic currently reports a request,
not demonstrated actuation.

Second, the near-field half of the two-stage grasp no longer implements the
operator's intended sequence of "look again, correct, and grasp." It waits for
cross-request candidate stability, exhaustively checks many complete sequences
with MoveIt, screens every reachable candidate through MuJoCo, and then
re-simulates the selected plan in the task node. The latest run found batches
with four and eight strictly MoveIt-reachable sequences, but MuJoCo rejected
all of them. The task then exhausted a 450-second wait horizon without sending
a contact motion.

## Goals

1. Distinguish a positive torque-on request from measured actuator response.
2. Never claim that motion is enabled solely because a torque-on frame was
   written.
3. Prevent retained ROS/controller targets from replaying immediately after a
   serial reconnect or power cycle.
4. Preserve the operator-only torque-off policy. The repair must not send
   torque-off, stop, disable, or emergency-stop commands automatically.
5. Make the second stage one fresh near-field observation followed by the
   first current, strictly reachable grasp sequence.
6. Keep physical geometry, support clearance, collision, finite-data, joint
   limit, strict MoveIt, target-freshness, and measured-endpoint checks.
7. Remove cross-request waiting and MuJoCo contact authority from the direct
   near-field path.
8. Bound the second-stage planning wait to 30 seconds and return an exact
   failure reason instead of waiting 450 seconds.

## Non-goals

- Software will not claim to clear a firmware `0xE1` protection state. Cooling
  and a physical power cycle remain hardware recovery steps when the same
  measured temperature channel is persistently over the configured limit.
- This change will not infer a missing firmware acknowledgement.
- This change will not relax joint limits, MoveIt collision checking, support
  clearance, gripper CAD constraints, target freshness, or endpoint checks.
- Offline tests will not send a hardware command.

## 1. Driver actuation-confirmation state machine

### States

The driver will track two separate concepts:

- `motion_commands_requested`: the operator or startup policy has requested
  positive torque and command streaming is permitted internally.
- `actuation_state`: one of `DISABLED`, `PENDING`, `CONFIRMED`,
  `UNCONFIRMED`, or `OVERHEAT_BLOCKED`.

`/alicia_d/motion_enabled` will mean measured `CONFIRMED`, not merely
"torque-on was written." A latched string topic
`/alicia_d/actuation_status` will expose the exact state and reason.

The internal command-stream permission remains separate. Marking actuation
unconfirmed must not send torque-off or any other hardware command.

### Positive enable

A positive enable from startup, reconnect, or `/demonstration=false` will use
one common helper:

1. Clear retained SDK command interpolation and command-response evidence.
2. Set `motion_commands_requested=true`.
3. Set `actuation_state=PENDING` and publish
   `/alicia_d/motion_enabled=false`.
4. Write exactly the existing positive torque-on frame.
5. Do not manufacture an acknowledgement.

An explicit `/demonstration=true` remains the only runtime torque-off path and
is outside this change.

### Reconnect command admission

After process startup or serial reconnect, the driver will wait for fresh real
feedback and a synchronized ROS command before admitting a motion delta.

- Commands received before real feedback are not retained for later replay.
- The first admitted six-joint command must be within a configurable
  synchronization tolerance of current feedback. This is the command produced
  by the GUI's "同步当前关节" action or an equivalent controller
  synchronization.
- A pre-existing controller target outside that tolerance is rejected and
  audited as `STALE_COMMAND_AFTER_RECONNECT`; it is never streamed to the
  hardware.
- Once synchronization succeeds, a later operator command may establish
  measured actuation.

The default synchronization tolerance will cover the observed post-sync
quantization/residual while rejecting the retained 12-degree Joint2 target.

### Measured confirmation

For the first non-trivial command after synchronization, the driver records:

- the fresh encoder baseline;
- the exact streamed target;
- the command time; and
- the joints whose requested deltas exceed the configurable probe threshold.

Actuation becomes `CONFIRMED` only when fresh encoder feedback moves by the
minimum measurable delta in the requested direction before the response
deadline. It is not necessary to reach the endpoint merely to prove torque
response.

If no qualifying feedback change appears before the deadline, the state
becomes `UNCONFIRMED`, `/alicia_d/motion_enabled` remains false, and the driver
logs the requested and measured deltas. It continues to obey the no-automatic-
torque-off rule.

If `0xE1` or `0xE2` coincides with the existing same-channel sustained
temperature condition at or above the enable limit, the state becomes
`OVERHEAT_BLOCKED`. Further positive-enable retries are rejected until
measured temperature is below the existing limit; the driver still sends no
torque-off.

### Supervisor behavior

The grasp task will subscribe to the confirmed motion state. A new automatic
grasp start is rejected with `ACTUATION_UNCONFIRMED` unless confirmation is
fresh. Manual GUI commands remain available so the operator can synchronize
and perform the small move that supplies confirmation after power-up.

## 2. Direct near-field grasp mode

### Phase-one behavior

The current far-field observation move remains unchanged:

1. Bind the far-field observation authority.
2. Move to the camera observation pose.
3. Measure the reached endpoint and camera-to-target distance.
4. Permit at most the existing single measured radial correction.
5. Enter near-field mode only after the inclusive observation-range contract
   passes.

### Single near-field request

Production configuration will select
`near_field_strategy: single_snapshot_direct`.

On the transition into near field:

1. Reset the old far-field candidate epoch.
2. Collect one current fused RGB-D/mask/object snapshot using the configured
   multi-frame snapshot window.
3. Materialize and recheck candidates against the frozen snapshot geometry.
4. Rank the current hard-safe candidates deterministically.
5. Run strict MoveIt sequence checking in rank order.
6. Stop at the first strictly reachable candidate.
7. Publish that exact candidate as the contact execution preview and freeze
   its plan ID.

Cross-request candidate-hit stability is not required in this strategy,
because temporal evidence already comes from the fused frames in the one
request. Near-field MoveIt selection is non-exhaustive and
first-reachable-by-authoritative-rank. A 30-second phase deadline covers
snapshot preparation and MoveIt checking; no new checks start after the
deadline.

### Removed authority layers

For `single_snapshot_direct` only:

- `_screen_near_field_selection_with_mujoco` is not called.
- The task node does not call the duplicate
  `_simulate_grasp6d_plan_if_required` after rebinding the near-field plan.
- `final_visual_refine` is skipped because the fresh near-field snapshot is
  already the visual correction requested by the operator.
- Continuous inference is stopped as soon as the exact near-field plan is
  frozen.

MuJoCo remains available for offline auditing and for any explicitly selected
legacy strategy, but it has no execution authority in the production direct
near-field strategy.

### Physical execution

After the exact near-field plan is frozen, the unchanged execution checkpoints
run in this order:

1. Move to the candidate-specific pregrasp if it differs from the reached
   observation pose.
2. Execute the strict linear approach.
3. Execute the strict linear grasp pose.
4. Close the gripper with the configured position command.
5. Execute the strict linear lift.

Every stage retains plan-ID binding, target freshness/drift, strict MoveIt
collision and joint-limit authority, measured endpoint checks, and controller
result validation.

### Failure behavior

The direct near-field stage fails closed without contact motion when:

- the fused snapshot cannot be formed;
- no hard-safe candidate exists;
- no candidate passes strict MoveIt within 30 seconds;
- the target or plan becomes stale;
- actuation is not confirmed; or
- any execution checkpoint fails.

Expected terminal codes include `NEAR_FIELD_SNAPSHOT_UNAVAILABLE`,
`NEAR_FIELD_NO_HARD_SAFE_CANDIDATE`,
`NEAR_FIELD_NO_REACHABLE_CANDIDATE`, and
`NEAR_FIELD_DIRECT_TIMEOUT`. The old generic 450-second
`NEAR_FIELD_REPLAN_TIMEOUT` is not used by this strategy.

## Configuration

Production defaults will encode the approved behavior:

- `near_field_strategy: single_snapshot_direct`
- `near_field_replan_timeout_sec: 30.0`
- phase-specific near-field stability hits: one request
- near-field MoveIt selection: first reachable by authoritative rank
- MuJoCo near-field selection authority: disabled
- duplicate task-level near-field simulation: disabled
- final visual refinement after near-field rebind: disabled

Legacy behavior remains selectable only through an explicit non-production
strategy value so tests and offline experiments can still exercise it.

Driver parameters will define:

- reconnect synchronization tolerance;
- minimum requested delta for confirmation;
- minimum measured response delta;
- confirmation timeout; and
- confirmation freshness.

All thresholds are actuator/protocol quantities, not object-specific values.

## Testing

Implementation will follow test-driven development.

### Driver tests

Tests will first reproduce and then cover:

- torque-on publishes `PENDING`, not confirmed;
- a command with zero encoder response becomes `UNCONFIRMED`;
- feedback moving toward the streamed command becomes `CONFIRMED`;
- an opposite or unrelated feedback jump does not confirm actuation;
- persistent same-channel temperature at the limit reports
  `OVERHEAT_BLOCKED` without writing torque-off;
- reconnect discards pre-feedback and out-of-tolerance retained targets;
- a synchronized command unlocks later manual motion;
- all positive-enable entry points share the same state-reset behavior.

### Near-field tests

Tests will first reproduce and then cover:

- one fused near-field request can produce a preview without cross-request
  stability hits;
- candidates are checked in authoritative rank order;
- checking stops at the first strictly reachable sequence;
- MuJoCo selection is not called in direct mode;
- duplicate task-level simulation and final-refine waits are not called;
- the exact rebound plan executes pregrasp, approach, grasp, close, and lift;
- a 30-second deadline returns an exact direct-mode error;
- hard geometry, collision, joint-limit, target-freshness, plan-ID, and endpoint
  gates remain active.

### Verification

Offline verification will include focused driver, task, streaming, remote-node,
configuration, and GUI suites; Python compilation; YAML/XML parsing;
`git diff --check`; and the complete relevant workspace test suite. A catkin
build will verify the C++ driver.

## Powered validation

No powered validation occurs until the operator reports that the arm has
cooled and is powered on.

1. Launch or reconnect sends only positive torque-on.
2. Driver reports `PENDING`, not confirmed.
3. Operator clicks "同步当前关节" and makes one small GUI move.
4. Encoder response must produce `CONFIRMED`; otherwise no automatic grasp
   starts.
5. Operator aligns the target.
6. One fresh 6D task executes the unchanged observation stage.
7. The direct near-field stage must either freeze a reachable plan within
   30 seconds and proceed to grasp, or fail with an exact bounded reason.

ROS nodes remain online after terminal success or failure. No stop, disable,
or torque-off command is part of this validation.

## Documentation ownership

The current effective end-to-end route and its dated route-level change
history live in
`src/alicia_flexible_grasp_supervisor/docs/grasp_task_technical_route.md`.
Runtime evidence, individual defects, implementation changes, fixes, and
verification results continue to be appended to
`src/alicia_flexible_grasp_supervisor/logs/2026-07-23-ros-latest-node-launch.md`.
