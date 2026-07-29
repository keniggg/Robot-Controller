# Alicia-D Actuation Confirmation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make positive torque enable, post-reconnect command admission, and measured actuator response distinct states so stale commands cannot replay and automatic grasp cannot start without fresh encoder-confirmed motion.

**Architecture:** Put the protocol-independent transition rules in a small C++14 `ActuationConfirmation` class and unit-test it without serial hardware. The ROS driver owns that class, publishes its latched state, uses one helper for every positive-enable entry point, and feeds it only accepted encoder samples and successfully streamed targets. The grasp task consumes the latched state and its local receipt time as an automatic-execution gate.

**Tech Stack:** ROS 1 Noetic, roscpp, std_msgs, C++14, catkin gtest, Python 3 unittest.

## Global Constraints

- A written positive torque-on frame is a request, not an acknowledgement.
- `/alicia_d/motion_enabled` is true only for fresh `CONFIRMED` measured motion.
- Startup, reconnect, and `/demonstration=false` use the same positive-enable helper.
- Commands received before fresh real feedback are dropped and never replayed.
- The first post-connect command must synchronize within the configured tolerance before a later non-trivial command can be streamed.
- `0xE1`/`0xE2` plus the existing same-channel sustained over-limit condition produces `OVERHEAT_BLOCKED`.
- This implementation never adds an automatic torque-off, stop, disable, controller-stop, or emergency-stop path.
- Offline tests must not start ROS hardware interfaces or write a serial frame.

---

### Task 1: Add the protocol-independent actuation state machine

**Files:**
- Create: `src/real-arm/alicia_d_driver/include/alicia_d_driver/actuation_confirmation.hpp`
- Create: `src/real-arm/alicia_d_driver/test/actuation_confirmation_test.cpp`
- Modify: `src/real-arm/alicia_d_driver/CMakeLists.txt`

**Interfaces:**
- Consumes: six-joint encoder samples, admitted targets, successful streamed targets, monotonic seconds, and `ActuationConfirmationConfig`.
- Produces: `ActuationState state()`, `bool motion_confirmed(double now_sec)`, `bool admit_command(...)`, `void note_feedback(...)`, `void note_streamed_target(...)`, and `std::string status_text()`.

- [ ] **Step 1: Write failing transition tests**

Add gtests that instantiate this exact public interface:

```cpp
ActuationConfirmationConfig config;
config.sync_tolerance_rad = 0.05;
config.command_probe_min_delta_rad = 0.02;
config.measured_response_min_delta_rad = 0.003;
config.response_timeout_sec = 1.0;
config.confirmation_freshness_sec = 2.0;
ActuationConfirmation state(config);
```

Cover these concrete sequences:

```cpp
state.reset_for_positive_enable(10.0);
EXPECT_EQ(state.state(), ActuationState::PENDING);
EXPECT_FALSE(state.motion_confirmed(10.1));

state.note_feedback({0, 0, 0, 0, 0, 0}, 10.1);
EXPECT_FALSE(state.admit_command({0, 0.20, 0, 0, 0, 0}, 10.2, nullptr));
EXPECT_TRUE(state.admit_command({0, 0.01, 0, 0, 0, 0}, 10.3, nullptr));
EXPECT_TRUE(state.synchronized());

state.note_streamed_target({0, 0.10, 0, 0, 0, 0}, 10.4);
state.note_feedback({0, 0.006, 0, 0, 0, 0}, 10.5);
EXPECT_EQ(state.state(), ActuationState::CONFIRMED);
EXPECT_TRUE(state.motion_confirmed(10.6));
```

Also assert:

- zero feedback through `11.5` becomes `UNCONFIRMED`;
- a `-0.006` Joint2 response to a positive Joint2 target does not confirm;
- a jump only on an uncommanded joint does not confirm;
- confirmation expires after `confirmation_freshness_sec`;
- `mark_overheat_blocked("...")` publishes `OVERHEAT_BLOCKED`;
- `reset_for_positive_enable` clears synchronization and any active probe;
- `set_disabled` is the only state-machine transition to `DISABLED`.

- [ ] **Step 2: Run the test and verify it fails**

Run:

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_ENABLE_TESTING=ON -j2
catkin_make run_tests_alicia_d_driver_actuation_confirmation_test
```

Expected: compilation fails because `actuation_confirmation.hpp` and its types do not exist.

- [ ] **Step 3: Implement the minimal C++14 state machine**

Define:

```cpp
enum class ActuationState {
    DISABLED,
    PENDING,
    CONFIRMED,
    UNCONFIRMED,
    OVERHEAT_BLOCKED,
};

struct ActuationConfirmationConfig {
    double sync_tolerance_rad = 0.05;
    double command_probe_min_delta_rad = 0.02;
    double measured_response_min_delta_rad = 0.003;
    double response_timeout_sec = 1.0;
    double confirmation_freshness_sec = 2.0;
};
```

Implement a header-only `ActuationConfirmation` with these exact signatures:

```cpp
explicit ActuationConfirmation(
    const ActuationConfirmationConfig& config =
        ActuationConfirmationConfig()
);
void reset_for_positive_enable(double now_sec);
void set_disabled(double now_sec);
void mark_overheat_blocked(const std::string& reason, double now_sec);
void note_feedback(
    const std::vector<double>& joints,
    double stamp_sec
);
bool admit_command(
    const std::vector<double>& target,
    double stamp_sec,
    std::string* rejection_reason
);
void note_streamed_target(
    const std::vector<double>& target,
    double stamp_sec
);
void update(double now_sec);
ActuationState state() const;
bool synchronized() const;
bool motion_confirmed(double now_sec) const;
std::string status_text() const;
```

Apply these deterministic rules:

```cpp
requested_delta = target[joint] - feedback_baseline[joint];
measured_delta = feedback[joint] - feedback_baseline[joint];
qualifying_request =
    std::abs(requested_delta) >= config.command_probe_min_delta_rad;
matching_response =
    std::abs(measured_delta) >= config.measured_response_min_delta_rad &&
    requested_delta * measured_delta > 0.0;
```

The first command after feedback only sets `synchronized=true` when every joint
is within `sync_tolerance_rad`; it does not start a probe. A later successfully
streamed non-trivial target starts one immutable probe. Repeated keepalive
frames do not replace its baseline or deadline. `update()` changes an expired
active probe to `UNCONFIRMED`. `status_text()` starts with the enum name and
appends the stored reason after `:`.

- [ ] **Step 4: Register and run the C++ test**

Add:

```cmake
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(
    actuation_confirmation_test
    test/actuation_confirmation_test.cpp
  )
  if(TARGET actuation_confirmation_test)
    target_include_directories(
      actuation_confirmation_test PRIVATE include ${catkin_INCLUDE_DIRS}
    )
  endif()
endif()
```

Run the two commands from Step 2.

Expected: `actuation_confirmation_test` passes with no serial device open.

- [ ] **Step 5: Commit the independently tested state machine**

```bash
git add src/real-arm/alicia_d_driver/include/alicia_d_driver/actuation_confirmation.hpp src/real-arm/alicia_d_driver/test/actuation_confirmation_test.cpp src/real-arm/alicia_d_driver/CMakeLists.txt
git commit -m "feat: add measured actuation state machine"
```

---

### Task 2: Integrate confirmation and reconnect admission into the ROS driver

**Files:**
- Modify: `src/real-arm/alicia_d_driver/include/alicia_d_driver/alicia_d_driver_node.hpp`
- Modify: `src/real-arm/alicia_d_driver/src/alicia_d_driver_node.cpp`
- Modify: `src/real-arm/alicia_d_driver/launch/alicia_d_bringup.launch`
- Modify: `src/real-arm/alicia_d_driver/launch/alicia_d_driver.launch`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_serial_driver_resilience.py`

**Interfaces:**
- Consumes: `ActuationConfirmation` from Task 1 and accepted real feedback from `parse_sdk_joint_state_frame`.
- Produces: latched `/alicia_d/actuation_status` (`std_msgs/String`) and corrected `/alicia_d/motion_enabled` (`std_msgs/Bool`).

- [ ] **Step 1: Add failing driver-structure regressions**

Extend `test_serial_driver_resilience.py` to assert:

```python
self.assertIn('std_msgs/String.h', header)
self.assertIn('ros::Publisher actuation_status_pub_', header)
self.assertIn('ActuationConfirmation actuation_confirmation_', header)
self.assertIn('request_positive_enable(', header)
self.assertIn('publish_actuation_status(', header)
```

Extract constructor, reconnect, demonstration, joint-command, send-command,
and accepted-feedback bodies with `_function_body`. Assert:

- constructor, reconnect, and `/demonstration=false` call
  `request_positive_enable(...)`;
- none of those three bodies contains its own `torque_on_frame`;
- positive enable publishes `motion_enabled=false`;
- the command callback calls `actuation_confirmation_.admit_command`;
- successful SDK streaming calls `note_streamed_target`;
- accepted encoder feedback calls `note_feedback`;
- sustained temperature plus `0xE1`/`0xE2` calls
  `mark_overheat_blocked`;
- no temperature/status body contains `torque_off_frame`;
- only the explicit `if (msg->data)` branch keeps the existing torque-off frame.

- [ ] **Step 2: Run the failing Python test**

Run:

```bash
python3 -m unittest src.alicia_flexible_grasp_supervisor.tests.test_serial_driver_resilience -v
```

Expected: new assertions fail because the shared helper and status publisher do
not yet exist.

- [ ] **Step 3: Add driver parameters and ROS state**

Add private parameters with these production defaults:

```cpp
pnh_.param<double>(
    "reconnect_sync_tolerance_rad",
    reconnect_sync_tolerance_rad_,
    0.05
);
pnh_.param<double>(
    "actuation_command_probe_min_delta_rad",
    actuation_command_probe_min_delta_rad_,
    0.02
);
pnh_.param<double>(
    "actuation_measured_response_min_delta_rad",
    actuation_measured_response_min_delta_rad_,
    0.003
);
pnh_.param<double>(
    "actuation_confirmation_timeout_sec",
    actuation_confirmation_timeout_sec_,
    1.0
);
pnh_.param<double>(
    "actuation_confirmation_freshness_sec",
    actuation_confirmation_freshness_sec_,
    2.0
);
```

Advertise:

```cpp
actuation_status_pub_ =
    nh_.advertise<std_msgs::String>("/alicia_d/actuation_status", 1, true);
```

Initialize the state machine config from these parameters after
`load_parameters()` and publish an initial `DISABLED` state.

- [ ] **Step 4: Route every positive-enable entry through one helper**

Implement:

```cpp
bool AliciaDDriverNode::request_positive_enable(
    const std::string& source
);
void AliciaDDriverNode::publish_actuation_status();
void AliciaDDriverNode::clear_retained_command_state();
```

`clear_retained_command_state()` clears `has_latest_command_`, the six-joint
target, interpolation seed, streamed-command evidence, endpoint trim state, and
the confirmation probe. `request_positive_enable()`:

1. rejects only the existing measured sustained over-temperature condition;
2. clears retained command state;
3. sets internal command permission true;
4. resets confirmation to `PENDING`;
5. publishes `motion_enabled=false` and the exact status;
6. writes exactly `{0xAA,0x05,0x00,0x01,0x01,0xF9,0xFF}`;
7. returns the serial write result without manufacturing an acknowledgement.

Replace constructor, reconnect, and `/demonstration=false` copies with this
helper. Keep the existing explicit `/demonstration=true` torque-off branch and
set the confirmation state to `DISABLED` there.

- [ ] **Step 5: Gate commands and feed only authoritative evidence**

In `joint_command_callback`, after forming the complete six-joint target but
before assigning `latest_joint_angles_`, call:

```cpp
std::string rejection_reason;
if (!actuation_confirmation_.admit_command(
        joint_angles,
        command_time.toSec(),
        &rejection_reason
    )) {
    ROS_WARN_THROTTLE(
        1.0,
        "Rejected joint command: %s",
        rejection_reason.c_str()
    );
    return;
}
```

When `admit_command` rejects an out-of-tolerance first command after startup or
reconnect, its exact reason is `STALE_COMMAND_AFTER_RECONNECT`; commands
received before a real feedback baseline use `FEEDBACK_REQUIRED_FOR_SYNC`.

After a real SDK command frame is successfully written, call
`note_streamed_target(cmd_joint_angles_, now.toSec())`. After
`candidate_joint_positions` passes all existing parser and command-consistency
filters, call `note_feedback(candidate_joint_positions, feedback_time.toSec())`.
Call `update(now.toSec())` in the send timer, then publish transitions.

When the same temperature channel reaches the existing consecutive limit and
`last_run_status_` is `0xE1` or `0xE2`, call:

```cpp
actuation_confirmation_.mark_overheat_blocked(
    "sustained same-channel temperature protection",
    ros::Time::now().toSec()
);
```

Do not change `motion_commands_enabled_` and do not write torque-off in this
path.

- [ ] **Step 6: Add launch defaults and pass focused tests**

Put the same five parameter values in both Alicia-D driver launch files.

Run:

```bash
python3 -m unittest src.alicia_flexible_grasp_supervisor.tests.test_serial_driver_resilience -v
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_ENABLE_TESTING=ON -j2
catkin_make run_tests_alicia_d_driver_actuation_confirmation_test
```

Expected: all focused Python and C++ driver tests pass.

- [ ] **Step 7: Commit the driver integration**

```bash
git add src/real-arm/alicia_d_driver/include/alicia_d_driver/alicia_d_driver_node.hpp src/real-arm/alicia_d_driver/src/alicia_d_driver_node.cpp src/real-arm/alicia_d_driver/launch/alicia_d_bringup.launch src/real-arm/alicia_d_driver/launch/alicia_d_driver.launch src/alicia_flexible_grasp_supervisor/tests/test_serial_driver_resilience.py
git commit -m "fix: confirm actuator response before reporting motion"
```

---

### Task 3: Gate automatic grasp on fresh confirmed actuation

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py`
- Modify: `src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_graspnet_input_default_config.py`

**Interfaces:**
- Consumes: latched `/alicia_d/actuation_status` text whose prefix is an `ActuationState` name.
- Produces: `ACTUATION_UNCONFIRMED` start rejection before execution authority is frozen.

- [ ] **Step 1: Write failing task-gate tests**

Add focused tests for:

```python
node.actuation_status_cb(String(data='CONFIRMED:encoder response'))
ok, reason = node._automatic_actuation_gate(
    {
        'require_actuation_confirmation': True,
        'actuation_confirmation_freshness_sec': 2.0,
    },
    now_sec=11.0,
)
self.assertTrue(ok)
```

Set the callback receipt time to `10.0` in the fixture. Assert
`PENDING`, `UNCONFIRMED`, `OVERHEAT_BLOCKED`, missing status, and a receipt
older than two seconds all return `False` with a reason beginning
`ACTUATION_UNCONFIRMED`. Assert `start_cb` returns before `_freeze_execution_plan`
or `execute` when the gate fails.

Update the persisted-config test to require:

```python
self.assertTrue(grasp['require_actuation_confirmation'])
self.assertEqual(grasp['actuation_confirmation_freshness_sec'], 2.0)
```

- [ ] **Step 2: Run the focused tests and verify failure**

Run:

```bash
python3 -m unittest \
  src.alicia_flexible_grasp_supervisor.tests.test_grasp_task_sequence \
  src.alicia_flexible_grasp_supervisor.tests.test_graspnet_input_default_config \
  -v
```

Expected: failures for the missing callback, helper, subscription, and config.

- [ ] **Step 3: Implement the task-level gate**

Subscribe in `__init__`:

```python
rospy.Subscriber(
    '/alicia_d/actuation_status',
    String,
    self.actuation_status_cb,
    queue_size=1,
)
```

Store:

```python
def actuation_status_cb(self, msg):
    self.latest_actuation_status = str(msg.data or '').strip()
    self.latest_actuation_status_time = rospy.Time.now()
```

Implement:

```python
def _automatic_actuation_gate(self, gcfg, now_sec=None):
    if not self._cfg_bool(
        gcfg, 'require_actuation_confirmation', False
    ):
        return True, ''
    status = str(
        getattr(self, 'latest_actuation_status', '') or ''
    ).strip()
    received = getattr(self, 'latest_actuation_status_time', None)
    if now_sec is None:
        now_sec = _stamp_seconds(rospy.Time.now())
    age = (
        float('inf')
        if received is None
        else float(now_sec) - _stamp_seconds(received)
    )
    freshness = max(
        0.0,
        self._cfg_float(
            gcfg, 'actuation_confirmation_freshness_sec', 2.0
        ),
    )
    if status.split(':', 1)[0] != 'CONFIRMED':
        return False, 'ACTUATION_UNCONFIRMED: %s' % (
            status or 'status missing'
        )
    if not math.isfinite(age) or age < 0.0 or age > freshness:
        return False, (
            'ACTUATION_UNCONFIRMED: confirmation age %.3fs exceeds %.3fs'
            % (age, freshness)
        )
    return True, ''
```

Call it in `start_cb` immediately after calibration interlock evaluation and
before copying or freezing any plan. Add production config values:

```yaml
require_actuation_confirmation: true
actuation_confirmation_freshness_sec: 2.0
```

- [ ] **Step 4: Run focused tests**

Run the command from Step 2.

Expected: the new gate tests and all existing task/config tests pass.

- [ ] **Step 5: Commit the automatic-execution gate**

```bash
git add src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py src/alicia_flexible_grasp_supervisor/tests/test_graspnet_input_default_config.py
git commit -m "fix: require fresh actuation proof for automatic grasp"
```

---

### Task 4: Verify the actuation track and document evidence

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/docs/grasp_task_technical_route.md`
- Modify: `src/alicia_flexible_grasp_supervisor/logs/2026-07-23-ros-latest-node-launch.md`

**Interfaces:**
- Consumes: Tasks 1-3 test and build results.
- Produces: dated route status and detailed implementation/verification log.

- [ ] **Step 1: Run the complete affected offline verification**

```bash
python3 -m unittest \
  src.alicia_flexible_grasp_supervisor.tests.test_serial_driver_resilience \
  src.alicia_flexible_grasp_supervisor.tests.test_grasp_task_sequence \
  src.alicia_flexible_grasp_supervisor.tests.test_graspnet_input_default_config \
  -v
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_ENABLE_TESTING=ON -j2
catkin_make run_tests_alicia_d_driver_actuation_confirmation_test
git diff --check
```

Expected: all focused tests and catkin build pass; `git diff --check` is silent.

- [ ] **Step 2: Update the two documentation authorities**

In the route document, change status from “正在离线实现” to “执行确认离线实现已验证，
待真机验证” and append a dated route-change entry. In the runtime log, record
exact test counts, build result, parameters, files changed, and the fact that no
ROS hardware interface was started.

- [ ] **Step 3: Commit verification records**

```bash
git add src/alicia_flexible_grasp_supervisor/docs/grasp_task_technical_route.md src/alicia_flexible_grasp_supervisor/logs/2026-07-23-ros-latest-node-launch.md
git commit -m "docs: record actuation confirmation verification"
```
