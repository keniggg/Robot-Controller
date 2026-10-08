# Paired model reload — prepared only

This directory is a concrete ROS-consumer reload draft, not a completed deployment.
Only the default review branch and pure offline tests have run. No `--execute`,
ROS parameter write, process signal, service restart or hardware motion was run.
User authorization to continue autonomous repair already exists; the current
`execution_ready=false` is an evidence/readiness condition, not a request for new
user permission.

Historical combined evidence retains the missing J3 window and held-J4 control-bound failure. Independent geometry review can qualify **idle paired loading and read-only compute verification** while preserving that failed history. It cannot authorize motion or clear a fault. `activation_review.json` remains unreleased until root's current-context, new-J4 and matching-simulation review. The selected candidate service is bound by actual `/health`, not an assumption about whether a process was started.

## Files and exact entry points

- `reload_pair.py`: defaults to a JSON review plan, without ROS/network access.
- `compute_consumers.launch`: MoveGroup and handeye publisher only; consumes
  already installed parameters without reloading defaults.
- `perception_consumers.launch`: old carton topic remaps plus unknown perception,
  remote planner and mode router. Existing task stays in its dedicated unit.
- `start_consumers.sh`: starts only one of those scoped launches with SanStand logs.
- `persistent_last_include.txt`: exact final include for root's future persistent
  launch edit. Neither this draft nor its execute branch edits/relaunches the
  full root startup file.

Review command:

```sh
python3 .runtime/model_pair_reload/reload_pair.py
```

Future root-only idle loading, after updating the pinned readiness record for this limited scope:

```sh
source devel/setup.bash
python3 .runtime/model_pair_reload/reload_pair.py --execute \
  --geometry-eligibility /absolute/idle_pair_reload_geometry_eligibility.json --geometry-sha256 SHA256 \
  --expected-live-epoch CURRENT_POWER_CYCLE_EPOCH_NS \
  --simulation-manifest /absolute/root_simulation_manifest.json --simulation-sha256 SHA256 \
  --output /media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/NEW_RELOAD_DIRECTORY
```

The independent eligibility is checked against its original combined report,
candidate manifest, frozen fit and independent review hashes, original geometry
criteria, explicit motion exclusions and preserved failed terminal/missing phase.
The old report is never relabelled or rewritten. Capture epochs are archived in
`geometry_evidence_binding.capture_epochs_ns`; the separately supplied live epoch
is matched to current feedback and checked throughout loading, including the
snapshot construction window. Reusing a pre-power-cycle pose is not supported.

The original strict interface remains available with `--prospective-report` and
`--prospective-sha256` instead of geometry arguments. It still requires complete
command coverage and the same capture/live epoch. These interfaces cannot be
mixed; neither publishes a trajectory.

The simulation manifest must pin `candidate_model_sha256`,
`candidate_handeye_sha256`, `candidate_pair_sha256`, selected `server_url`,
`model_xml_sha256`, and exact `health_bindings`. For example, binding keys can be
`digital_twin.model_xml_sha256` and `digital_twin.mujoco`. Real `/health` must
expose the selected XML hash and match every binding. The endpoint is not hardcoded
to WSL; the separate existing GraspNet endpoint remains unchanged.

## Concrete sequence

The script requires idle task/controller actions, a stable 0.8 s measured window,
unchanged held SDK reference/epoch and the currently empty MoveIt scene. Task state is a latched event, not a heartbeat: inactive content must come from the connected `/grasp_task_node` publisher with its bound PID/starttime/URI. Task restart creates a fresh subscription generation under a short lock; queued callbacks from the old generation cannot supply new-instance evidence. Freshness requirements on action status and accepted/SDK feedback are unchanged. These
are this specific deployment window's consistency conditions, not new grasp
geometry tolerances. It archives old URDF/handeye parameters, exact calibration
YAML, selection, process identities, controller ownership and policy values.

It freezes only the PID/starttime-bound idle gateway; stops the existing task
child unit and independent readonly handeye observer; shuts down only listed
nonrespawn consumers and confirms their process exit. It writes the paired URDF/X
and selected simulation URL, lets the original parent respawn RSP, starts scoped
MoveGroup/handeye, and verifies exact-source RSP FK, static X and MoveGroup FK.
It then replaces the frozen gateway through its original parent respawn, restarts
the original task child unit and starts perception/router with the same mode and
strategy. Both the published selection and successful mode status must match
the new parameter generation and post-model cutoff; parameter writes alone do
not count as reset acknowledgement.

The root ROS unit, driver, hardware interface, controller instances, SDK raw
angles/reference/epoch and camera remain running. Original speed policy,
20 g unknown-object operator bound, lift-after-close and full-path compensation
remain enabled, and simulation lift remains diagnostic. The script creates no
motion publisher and never calls grasp start, controller switch, enable/disable,
bootstrap or sync.

The existing task unit has `PartOf=alicia-ros-live-20260928.service`. The observer
unit is independent. Both are transient: if stop causes systemd to collect a
unit, its exact bound start-script SHA and same unit name are used to recreate it,
with the original PartOf distinction and `Restart=no`. Other load states or
changed scripts are rejected. The root unit is never recreated.

## Failure and bounded rollback

A failed stage records `FAILED_NO_FURTHER_ACTION` and stops orchestration. There
is no implicit motion, automatic model retry or automatic rollback. Some
consumers can remain stopped and the idle gateway can remain paused; this is
recorded, not reported as a completed rollout. The hardware reference is untouched.
Root can explicitly invoke:

```sh
python3 .runtime/model_pair_reload/reload_pair.py --execute \
  --rollback-from /absolute/previous_reload/state.json --rollback-sha256 SHA256 \
  --output /media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/NEW_ROLLBACK_DIRECTORY
```

Rollback uses the exact saved original YAML with SHA `6ab0e8…`, rather than a
mutable original filename, and rebuilds the same consumers around the old pair.
A partial rollout with MoveGroup/task already absent uses the previously recorded
empty-scene and phase evidence; it does not require a message from the stopped
task. The same root/hardware process identities and driver epoch must still hold.
If the scene is nonempty, ownership changed, or those identities are unavailable,
this limited recipe does not invent a migration or restart hardware. Rollback
restores the previous simulation URL, not a remote simulation process. A future
persistent include edit is a separate root step; this script never edits it.
No live rollout/rollback fault injection has been performed.

The execute branch runs inside a main-thread `SIGALRM` deadline of 180 seconds,
covering synchronous ROS/XMLRPC/HTTP waits as well as the orchestration loop.
The exception is distinct from polling's recoverable OSError/TimeoutError and
records `FAILED_NO_FURTHER_ACTION`; the timer/handler are restored before local
failure recording. It interrupts this client's wait, not an already accepted
remote side effect; partial-stage rollback limitations above still apply. The
default review branch never arms a timer. `ROS_LOG_DIR` is set to `--output/ros`
on mounted SanStand before ROS import/init_node; no system-disk fallback.

## Verification

`python3 -m pytest -q .runtime/model_pair_reload/test_reload_pair.py`:
97 passed, covering the original 42 contracts plus the real independent eligibility
schema, unchanged partial/failure history, historical/current epoch separation,
strict-interface preservation, main-thread blocked-read interruption without
poll-loop swallowing, timeout failure recording, ROS log configuration before
init, old latched idle state with publisher/process binding, replaced/disconnected
publishers, snapshot epoch race, and an old callback waiting across subscription
generation renewal. These are pure offline tests and source inspection, not a
claim that live loading or rollback has succeeded.

SDK message schema correction: driver `sdk_command` publishes exactly Joint1..6
in radians with frame `sdk_transmitted`; it contains no gripper value.
The same successful full wire write separately publishes `control_reference`
with the same stamp and seventh `right_finger` in meters. Snapshot retains up
to four frames per topic, captures one exact-stamp post-epoch pair no older than
500ms, and freezes all six arm values plus the actual gripper width. Subsequent
SDK/reference messages retain independent freshness and unchanged-value checks;
a gripper-only change is not discarded. Accepted feedback and RSP joint_states
keep their canonical seven-name schema. The first root loading attempt stopped
in read-only snapshot with `KeyError right_finger` before any signal/parameter
mutation; its original failure record remains archived.
