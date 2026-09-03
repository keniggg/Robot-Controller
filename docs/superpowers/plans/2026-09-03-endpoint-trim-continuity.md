# Alicia-D Endpoint-Trim Continuity Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Eliminate the observed endpoint/wrist shake by serializing endpoint-trim corrections and preserving the composed SDK joint target across task-precision lease release or expiry.

**Architecture:** Extract the endpoint-trim transition rules into a pure C++14 coordinator. The ROS driver supplies fresh encoder samples, stable-joint evidence, reference targets, lease events, and explicit GUI ownership events. The coordinator owns correction-step admission and the `ACTIVE_READY -> WAITING_RESPONSE -> PENDING_RELEASE -> QUIESCENT/FAULT` state transitions. The driver continues to be the only SDK command writer and applies the coordinator's reference plus offset atomically under `latest_cmd_mutex_`.

**Tech Stack:** ROS 1 Noetic, roscpp, C++14, catkin gtest, Python 3 unittest, Alicia-D SDK joint protocol.

**Spec:** `docs/superpowers/specs/2026-09-03-known-to-unknown-class-agnostic-grasp-design.md`, section “Endpoint-trim continuity”.

## Global Constraints

- Do not add or invoke any automatic stop, torque-off, disable, controller-stop, emergency, `/demonstration=true`, or `/grasp/stop` path.
- Preserve the existing positive-enable state and explicit GUI command authority.
- One SDK joint position quantum is `2*pi/4096 rad`; every ownership rebase must preserve the pre-transition composed target within that quantum.
- Default per-response correction is capped at four SDK quanta (`8*pi/4096 rad`, about `0.352 deg`) per joint.
- A correction cannot be followed by another correction until a fresh encoder response has both moved in the requested direction and remained inside the settle band for the configured stable interval.
- Lease release or expiry during an outstanding response is deferred; it never immediately replaces the composed command with measured feedback.
- Offline tests must not open the serial device, start controllers, enable or move hardware.

---

### Task 1: Add a pure endpoint-trim coordinator and reproduce the live failure

**Files:**
- Create: `src/real-arm/alicia_d_driver/include/alicia_d_driver/endpoint_trim_continuity.hpp`
- Modify: `src/real-arm/alicia_d_driver/test/actuation_confirmation_test.cpp`

**Interfaces:**

```cpp
enum class EndpointTrimPhase {
    IDLE,
    ACTIVE_READY,
    WAITING_RESPONSE,
    PENDING_RELEASE,
    QUIESCENT,
    FAULT,
};

struct EndpointTrimConfig {
    size_t joint_count = 6;
    double sdk_quantum_rad = 2.0 * M_PI / 4096.0;
    double max_step_rad = 8.0 * M_PI / 4096.0;
    double response_min_rad = 2.0 * M_PI / 4096.0;
    double settle_error_rad = 4.0 * M_PI / 4096.0;
    double stable_sec = 0.30;
    double response_deadline_sec = 1.0;
    double max_total_trim_rad = 0.12;
};

struct EndpointTrimDecision {
    EndpointTrimPhase phase;
    std::vector<double> reference;
    std::vector<double> offsets;
    std::vector<double> composed_target;
    std::vector<double> applied_step;
    bool command_changed;
    bool release_completed;
    std::string code;
};

class EndpointTrimContinuity {
public:
    explicit EndpointTrimContinuity(const EndpointTrimConfig& config);
    EndpointTrimDecision activate(
        const std::vector<double>& reference,
        const std::vector<double>& offsets,
        double now_sec);
    EndpointTrimDecision request_correction(
        const std::vector<double>& measured,
        const std::vector<uint8_t>& stable_joint_mask,
        double gain,
        double now_sec);
    EndpointTrimDecision note_feedback(
        const std::vector<double>& measured,
        double feedback_stamp_sec,
        double now_sec);
    EndpointTrimDecision request_release(
        const std::vector<double>& measured,
        bool feedback_fresh,
        double now_sec);
    EndpointTrimDecision update(double now_sec);
    EndpointTrimDecision explicit_gui_handoff(
        const std::vector<double>& gui_target,
        double now_sec);
    const EndpointTrimDecision& state() const;
};
```

- [ ] **Step 1: Write the failing log-derived regression**

In `actuation_confirmation_test.cpp`, recreate a six-joint reference, the first correction with Joint2 `+1.3 deg`, a second request that would reach `+2.3 deg`, and lease release `0.37 s` later. Assert that the second request is not admitted while the first response is outstanding, release returns `PENDING_RELEASE`, and `composed_target` does not jump or reverse.

- [ ] **Step 2: Add boundary tests before implementation**

Assert all of the following:

- each element of `applied_step` is no larger than `max_step_rad`;
- feedback with the wrong direction or an old timestamp cannot settle a response;
- one fresh matching response still waits for `stable_sec`;
- pending release completes after settle or `response_deadline_sec`;
- final `reference[i] + offsets[i]` differs from the preserved target by at most `sdk_quantum_rad`;
- a response deadline produces `ENDPOINT_TRIM_RESPONSE_TIMEOUT` without changing the preserved target;
- malformed vector sizes or non-finite numbers produce `FAULT` with `ENDPOINT_TRIM_CONTINUITY_VIOLATION`;
- `explicit_gui_handoff()` immediately clears task trim state and returns the exact GUI target.

- [ ] **Step 3: Run the focused test and verify RED**

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_ENABLE_TESTING=ON -j2
catkin_make run_tests_alicia_d_driver_gtest_actuation_confirmation_test
```

Expected: compilation fails because `endpoint_trim_continuity.hpp` does not exist.

- [ ] **Step 4: Implement the minimal state machine**

Use saturating arithmetic and immutable response metadata. `request_correction()` computes `error = reference - measured`, multiplies by `gain`, clamps each joint first to `max_step_rad` and then to `max_total_trim_rad`, and records the measured response baseline and per-joint direction mask. `note_feedback()` accepts only a newer sample whose masked joints moved at least `response_min_rad` in the commanded direction; it starts a settle timer and completes only after the response remains within `settle_error_rad` for `stable_sec`.

On release, capture `preserved = reference + offsets`. Once release can complete, use `new_reference = measured` only when fresh; compute `new_offsets = preserved - new_reference`, quantize through the same SDK conversion rule, and reject any result whose recomposed target differs from `preserved` by more than one quantum. At deadline, keep `preserved` as the reference with zero offset and return the timeout diagnostic.

- [ ] **Step 5: Run the focused test and verify GREEN**

Run the commands from Step 3. Expected: every existing actuation/GUI test and every new endpoint-trim test passes without opening hardware.

- [ ] **Step 6: Commit the pure coordinator**

```bash
git add src/real-arm/alicia_d_driver/include/alicia_d_driver/endpoint_trim_continuity.hpp src/real-arm/alicia_d_driver/test/actuation_confirmation_test.cpp
git commit -m "fix: serialize endpoint trim transitions"
```

---

### Task 2: Integrate serialized correction admission into the driver timer

**Files:**
- Modify: `src/real-arm/alicia_d_driver/include/alicia_d_driver/alicia_d_driver_node.hpp`
- Modify: `src/real-arm/alicia_d_driver/src/alicia_d_driver_node.cpp`
- Modify: `src/real-arm/alicia_d_driver/launch/alicia_d_driver.launch`
- Modify: `src/real-arm/alicia_d_driver/launch/alicia_d_bringup.launch`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_serial_driver_resilience.py`

- [ ] **Step 1: Write failing source-contract tests**

Extend `test_serial_driver_resilience.py` to require the node to own one `EndpointTrimContinuity` instance, pass only accepted real feedback into `note_feedback()`, derive the transmitted joint values from `decision.composed_target`, and forbid direct writes to `endpoint_feedback_trim_offsets_` from the service callback and lease-expiry branch.

Also assert that no new torque-off/disable frame or service invocation appears in the endpoint-trim paths.

- [ ] **Step 2: Verify the new tests fail**

```bash
python3 -m unittest src.alicia_flexible_grasp_supervisor.tests.test_serial_driver_resilience -q
```

Expected: failures identify the current direct offset mutation and immediate release/expiry latch.

- [ ] **Step 3: Replace timer-local transition logic with the coordinator**

Add `EndpointTrimContinuity endpoint_trim_continuity_;` and keep all state access under `latest_cmd_mutex_`. In `send_command_timer_callback`:

1. snapshot reference, accepted feedback, stable mask, and timestamps;
2. call `note_feedback()` once for each new accepted encoder timestamp;
3. call `request_correction()` only in `ACTIVE_READY`;
4. assign the returned reference/offset pair together;
5. serialize `decision.composed_target` into the SDK frame; and
6. publish/log the transition code, response age, applied step, and target before/after.

Delete the stalled-response retry that applies another correction merely because its timeout elapsed. A timeout becomes a preserved-target diagnostic, not a new trim increment.

- [ ] **Step 4: Expose bounded parameters**

Load and forward these launch parameters:

```xml
<arg name="endpoint_feedback_trim_max_step_quantums" default="4"/>
<arg name="endpoint_feedback_trim_response_min_quantums" default="2"/>
<arg name="endpoint_feedback_trim_response_deadline_sec" default="1.0"/>
```

Reject values outside `max_step_quantums=[1,16]`, `response_min_quantums=[1,max_step_quantums]`, and deadline `[0.30,3.0]`; on rejection retain the safe defaults and log the actual effective values.

- [ ] **Step 5: Run driver tests**

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_ENABLE_TESTING=ON -j2
catkin_make run_tests_alicia_d_driver_gtest_actuation_confirmation_test
python3 -m unittest src.alicia_flexible_grasp_supervisor.tests.test_serial_driver_resilience -q
```

Expected: all pass.

- [ ] **Step 6: Commit timer integration**

```bash
git add src/real-arm/alicia_d_driver/include/alicia_d_driver/alicia_d_driver_node.hpp src/real-arm/alicia_d_driver/src/alicia_d_driver_node.cpp src/real-arm/alicia_d_driver/launch/alicia_d_driver.launch src/real-arm/alicia_d_driver/launch/alicia_d_bringup.launch src/alicia_flexible_grasp_supervisor/tests/test_serial_driver_resilience.py
git commit -m "fix: preserve endpoint target during trim response"
```

---

### Task 3: Make lease release, expiry, and GUI handoff continuous

**Files:**
- Modify: `src/real-arm/alicia_d_driver/src/alicia_d_driver_node.cpp`
- Modify: `src/real-arm/alicia_d_driver/test/actuation_confirmation_test.cpp`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_serial_driver_resilience.py`

- [ ] **Step 1: Add failing ownership-transition tests**

Cover service release during `WAITING_RESPONSE`, timer expiry during `WAITING_RESPONSE`, release with stale feedback, release after settled feedback, and an explicit GUI edit during `PENDING_RELEASE`. Assert that service release acknowledges “pending” without lying that the rebase is already complete, expiry uses the same code path, and GUI input remains immediately authoritative.

- [ ] **Step 2: Route both release sources through `request_release()`**

`set_task_endpoint_precision_callback(false)` and lease expiry must only request release. Neither may independently rebuild offsets, clear response metadata, or latch measured feedback. The send timer completes the atomic rebase and logs one of:

- `ENDPOINT_TRIM_RELEASE_SETTLED`;
- `ENDPOINT_TRIM_RESPONSE_TIMEOUT`; or
- `ENDPOINT_TRIM_CONTINUITY_VIOLATION`.

- [ ] **Step 3: Preserve explicit GUI authority**

At the accepted explicit-GUI command boundary call `explicit_gui_handoff(gui_target, now)`, atomically install its returned target, and clear task-lease/reference/response history. Do not reinterpret repeated task-controller messages as GUI commands.

- [ ] **Step 4: Run the focused and source-contract suites**

Use the commands from Task 2 Step 5. Expected: release, expiry, and GUI cases pass and the forbidden-command scan stays green.

- [ ] **Step 5: Commit ownership transitions**

```bash
git add src/real-arm/alicia_d_driver/src/alicia_d_driver_node.cpp src/real-arm/alicia_d_driver/test/actuation_confirmation_test.cpp src/alicia_flexible_grasp_supervisor/tests/test_serial_driver_resilience.py
git commit -m "fix: make endpoint trim lease handoff continuous"
```

---

### Task 4: Record evidence and gate powered deployment

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/docs/grasp_task_technical_route.md`
- Modify: `src/alicia_flexible_grasp_supervisor/logs/2026-07-23-ros-latest-node-launch.md`
- Create: `docs/superpowers/verification/2026-09-03-endpoint-trim-continuity.md`

- [ ] **Step 1: Run offline verification**

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_ENABLE_TESTING=ON -j2
catkin_make run_tests_alicia_d_driver_gtest_actuation_confirmation_test
python3 -m unittest src.alicia_flexible_grasp_supervisor.tests.test_serial_driver_resilience -q
git diff --check
```

- [ ] **Step 2: Record exact results**

Write command, exit status, pass count, effective quantum/step/deadline configuration, and the live-log regression inputs to the verification document. Update the technical route and runtime log with the new transition state diagram.

- [ ] **Step 3: Enforce the deployment gate**

Do not restart or hot-load the real driver while a motion command is active. Do not run a new grasp from this plan alone. Powered acceptance is permitted only after the class-agnostic precontact-geometry plan is also green and the operator has realigned the target.

- [ ] **Step 4: Commit verification records**

```bash
git add docs/superpowers/verification/2026-09-03-endpoint-trim-continuity.md src/alicia_flexible_grasp_supervisor/docs/grasp_task_technical_route.md src/alicia_flexible_grasp_supervisor/logs/2026-07-23-ros-latest-node-launch.md
git commit -m "docs: record endpoint trim continuity evidence"
```
