# WSL MuJoCo carton grasp execution gate

This service combines the existing GraspNet `/predict` endpoint with a
fail-closed MuJoCo `/simulate_grasp` gate. The gate consumes only the
schema-version-2 request produced by the ROS supervisor. It simulates a
segmentation-derived carton OBB, the detected support plane, and the fixed
`Alicia_D_v5_6_gripper_50mm` model before the real arm may move.

This phase does not use force or tactile feedback. Passing the digital-twin
gate is a geometric and kinematic precondition, not proof that a real grasp
will succeed.

## Install and start in WSL

The intended environment is Python 3.8 with `mujoco>=3.2`, NumPy, SciPy, and
the existing GraspNet baseline dependencies:

```bash
conda activate grasp6d118
pip install "mujoco>=3.2" numpy scipy pyyaml
cd ~/grasp6d_ws/Robot-Controller
./tools/start_mujoco_digital_twin_wsl.sh \
  --pass-score 80 \
  --min-lift-success-m 0.015 \
  --warmup
```

The default endpoint is `http://0.0.0.0:8000`. Check it without starting a
simulation:

```bash
curl http://127.0.0.1:8000/health
```

The ROS host and WSL host must use synchronized wall clocks. A request whose
ROS `snapshot_stamp_sec` is zero, more than 2.0 seconds old, or in the future
is rejected as `PLAN_STALE`. ROS simulated time is therefore not compatible
with this WSL wall-clock gate unless both sides are explicitly given the same
time source.

## Schema-version-2 request

`POST /simulate_grasp` requires all of the following:

- a non-empty `plan_id`, `model_choice`, and fresh `snapshot_stamp_sec`;
- finite, uniquely named joints containing `Joint1` through `Joint6`;
- exactly four poses, ordered pregrasp, approach, grasp, and lift;
- finite pose positions in metres and non-zero XYZW quaternions;
- `0 < required_open_width_m <= 0.050`;
- the fixed Alicia-D v5.6 50 mm gripper geometry contract;
- a `carton_box` OBB pose, dimensions, mass, and friction;
- a finite detected support plane `normal_base` and `offset_m`.

Carton dimensions must be positive, no dimension may exceed 0.600 m, and the
Z dimension may not exceed 0.500 m. The server normalizes quaternions and the
support equation `normal · point + offset = 0` before writing MuJoCo WXYZ
quaternions.

The following request is complete. Generate the timestamp immediately before
sending it so that the 2-second lease remains valid:

```bash
STAMP="$(date +%s.%N)"
curl -sS http://127.0.0.1:8000/simulate_grasp \
  -H 'Content-Type: application/json' \
  --data-binary @- <<JSON
{
  "schema_version": 2,
  "plan_id": "carton-plan-001",
  "snapshot_stamp_sec": ${STAMP},
  "model_choice": "carton_seg",
  "joint_names": ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"],
  "joint_positions": [0.0, 0.1, -0.2, 0.3, -0.1, 0.2],
  "trajectory": [
    {"position_m": [0.25, 0.00, 0.20], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
    {"position_m": [0.28, 0.00, 0.16], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
    {"position_m": [0.30, 0.00, 0.12], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
    {"position_m": [0.30, 0.00, 0.20], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]}
  ],
  "candidate_width_m": 0.040,
  "required_open_width_m": 0.045,
  "gripper": {
    "model_name": "Alicia_D_v5_6_gripper_50mm",
    "max_inner_gap_m": 0.050,
    "finger_size_xyz_m": [0.0434, 0.0286, 0.0600],
    "palm_size_xyz_m": [0.1175, 0.1550, 0.0774]
  },
  "object_model": {
    "type": "carton_box",
    "pose_base": {
      "position_m": [0.30, 0.00, 0.04],
      "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]
    },
    "size_xyz_m": [0.20, 0.10, 0.08],
    "mass_kg": 0.08,
    "friction": [1.2, 0.08, 0.02]
  },
  "support_plane": {
    "normal_base": [0.0, 0.0, 1.0],
    "offset_m": 0.0
  }
}
JSON
```

## Dynamic scene and gripper contract

The compiled-model cache key contains the carton type, mass, friction, and
full dimensions quantized to 1 mm. The XML uses those same canonical
dimensions as box half-extents, so cache results do not depend on request
order. Object and support poses are excluded from the cache key and are
written to fresh `MjData` on every request:

- the carton uses a free joint and the delivered OBB pose;
- `detected_support` is a mocap plane placed at
  `-normalized_offset * normalized_normal`;
- the legacy fixed `floor` has collision disabled for schema 2.

On model compilation the server verifies the two finger joint ranges and the
facing collision-mesh gap. The expected open-state gap is approximately
49.9375 mm and must be within 0.5 mm of the 50 mm contract. A mismatch returns
`GRIPPER_MODEL_MISMATCH`.

Inner gap mapping follows the actual MJCF directions:

```text
50 mm open -> left_finger=0.000,  right_finger=0.000
 0 mm open -> left_finger=-0.025, right_finger=+0.025
```

The close phase starts at exactly 50 mm and uses at least 35 monotonic
increments. Palm/object, robot/support, and excessive object/support
penetration fail immediately. A single-finger contact may continue only while
the carton remains stable. The first simultaneous left/right finger contact
width and its complete simulation state are retained for lift; the server
does not force the fingers to zero gap.

During lift, arm and finger qpos, qvel, and position-actuator targets are held
at every MuJoCo step. Two-sided contact and collision classification are also
checked at every step. Lift passes only if contact remains valid and the
carton moves at least `--min-lift-success-m` along the detected support normal.
This threshold belongs to the WSL server process. A request field named
`min_lift_success_m` is not part of schema 2 and cannot lower the configured
server threshold.

## Fail-closed response

Every handled simulation request returns the correlated `plan_id`, a finite
`score`, `failure_code`, `failure_reason`, and all five explicit booleans:

```json
{
  "plan_id": "carton-plan-001",
  "simulation_ok": true,
  "score": 100.0,
  "ik_success": true,
  "collision_free": true,
  "contact_success": true,
  "lift_success": true,
  "failure_code": "",
  "failure_reason": ""
}
```

`simulation_ok` is true only when IK, collision, contact, lift, and score
policy all pass. Stable failures include:

- `PLAN_INVALID`, `PLAN_STALE`, and `JOINT_STATE_INVALID`;
- `GRIPPER_TOO_NARROW` and `GRIPPER_MODEL_MISMATCH`;
- `OBB_INVALID` and `SUPPORT_PLANE_INVALID`;
- `MUJOCO_IK_FAILED`, `MUJOCO_COLLISION`,
  `MUJOCO_CONTACT_FAILED`, and `MUJOCO_LIFT_FAILED`;
- `MUJOCO_SCORE_BELOW_THRESHOLD` and `MUJOCO_INTERNAL_ERROR`.

Malformed payloads, model mismatches, missing components, non-finite results,
timeouts, and server exceptions all block execution. An HTTP 200 response or
a high score alone is never a simulation pass.

The HTTP boundary applies the same policy even if a backend is faulty: a
missing/mismatched `plan_id`, non-finite score, incomplete component set, or a
non-boolean component is converted to a correlated, component-complete
`MUJOCO_INTERNAL_ERROR`. Strict JSON is used throughout; an empty joint-state
cache is reported as JSON `null`, never `Infinity`.

## ROS deployment safety invariants

Real-arm deployment requires all of these parameters to remain in effect:

```yaml
mujoco_digital_twin:
  enabled: true
  execution_gate_enabled: true
  allow_execution_on_error: false
  send_joint_state_in_request: true
```

Both `enabled` and `execution_gate_enabled` must be `true`: disabling either
one removes the pre-execution simulation call and is not a safe degraded mode.
Keep `allow_execution_on_error: false` as a deployment invariant. The current
ROS supervisor fails closed on network errors regardless of that legacy
setting; changing it to `true` must never be treated as authorization to bypass
an unreachable WSL server, timeout, malformed response, or protocol error.

Before enabling motor power, verify the live parameters rather than only the
checked-in YAML:

```bash
rosparam get /mujoco_digital_twin/enabled
rosparam get /mujoco_digital_twin/execution_gate_enabled
rosparam get /mujoco_digital_twin/allow_execution_on_error
rosparam get /mujoco_digital_twin/send_joint_state_in_request
```

The expected values are `true`, `true`, `false`, and `true`, respectively.
Restart/re-source all ROS clients and services after deploying the schema-2
gate changes. Each execution must use the exact current `plan_id`; the
supervisor revalidates plan authority after the WSL network request, so a stop,
timeout, or replacement while the request is in flight still blocks motion.

## Verification before enabling arm motion

Run protocol and CLI checks without connecting to the robot:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B -m pytest -p no:cacheprovider \
  src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_server_protocol.py -q
python3 -B tools/mujoco_digital_twin_server.py --help
```

When MuJoCo and the Alicia-D meshes are installed on the WSL host, also run
the optional compilation smoke test:

```bash
MUJOCO_SMOKE=1 PYTHONDONTWRITEBYTECODE=1 python3 -B -m pytest \
  -p no:cacheprovider \
  src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_server_protocol.py -q
```

With `MUJOCO_SMOKE=1`, the protocol suite imports real MuJoCo, injects the
dynamic carton/support XML into the Alicia-D v5.6 MJCF, compiles the actual
mesh-backed model, validates the gripper contract, writes fresh dynamic scene
state, and runs `mj_forward`. A missing MuJoCo installation or broken mesh/XML
therefore fails the smoke run.

`--mock-mujoco` verifies only HTTP/schema wiring. Its `/simulate_grasp`
response is deliberately component-complete but always fails closed with
`MUJOCO_INTERNAL_ERROR`; it cannot authorize physical execution. It does not
validate IK, collision, contact, or lift physics and must never be used while
real-arm motion is enabled.
