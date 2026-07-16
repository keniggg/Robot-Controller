# Carton segmentation RGB-D grasp gate verification

- Verification date: 2026-07-16
- Task 11 implementation baseline: `53daf96`
- WSL URL wiring fix: `fd5508f`
- MASK freshness review fix: `e0152b4`
- Scope: segmentation checkpoint, ROS-mocked/pure regression, generated ROS interfaces,
  camera/serial availability, and remote WSL service capability
- Force/tactile feedback: out of scope for this phase
- Real-arm motion command: **not called**
- Overall decision: **REAL-ARM AUTHORIZATION: BLOCKED**

## 1. Result summary

| Check | Result | Recorded evidence |
| --- | --- | --- |
| Supplied YOLOv8-seg checkpoint | PASS | `task=segment`, class `carton`, 19/20 sampled frames contained a carton mask |
| Focused final Task 12 regression | PASS | `436 passed, 1 skipped, 3 warnings in 13.59s` |
| MuJoCo server protocol suite | PASS WITH OPTIONAL SMOKE SKIPPED | `94 passed, 1 skipped`; the skip is real `MUJOCO_SMOKE=1` |
| Full suite at Task 11 HEAD | PASS | `593 passed, 1 skipped, 3 warnings` |
| Full suite after URL wiring fix | PASS | `594 passed, 1 skipped, 3 warnings in 16.38s` |
| Full final Task 12 candidate | PASS | `602 passed, 1 skipped, 3 warnings in 14.37s` |
| Isolated catkin interface build | PASS | generated `7` messages and `9` services |
| `StartGrasp` interface | PASS | MD5 `5d246499be275f0453d1db3b1be742a6` |
| Camera-only bench acceptance | **NOT EXECUTED** | no ROS master and no `/dev/video*`; hardware was unavailable |
| Real WSL MuJoCo acceptance | **NOT EXECUTED / BLOCKED** | reachable endpoint was old GraspNet-only; `/simulate_grasp` was unknown |
| Local real-MuJoCo smoke | **NOT EXECUTED** | local Python had no `mujoco` module |
| Real-arm acceptance | **BLOCKED** | camera and real WSL MuJoCo evidence are both incomplete |

Automated PASS rows establish code and protocol behavior only. They do not replace the two
hardware-facing acceptances and do not authorize motor power.

## 2. Supplied segmentation checkpoint

The weight and uploaded video were inspected without changing or committing either file.
The weight exists in the main workspace, not inside the feature worktree:

| Artifact | Absolute path | Size | SHA-256 |
| --- | --- | ---: | --- |
| YOLOv8-seg weight | `/home/zhuyupei/alicia_wa_full/carton_segment_model/best.pt` | 23,838,964 bytes | `403ca673f0c29ac94884f64f92991a817e2e9cdf888bdcbb57dd60dc8629a4ec` |
| Uploaded motion video | `/home/zhuyupei/Videos/915d0f0873b3fa8217a0d8582932b44d.mp4` | 7,409,389 bytes | `c5f509e8ead99394322198f20ce3fc11d0f59c440fd76023fb166710ebbdd2fb` |

From the feature worktree the relative paths are respectively
`../../carton_segment_model/best.pt` and
`../../../Videos/915d0f0873b3fa8217a0d8582932b44d.mp4`.

The smoke environment was `/usr/bin/python3` 3.8.10 with Ultralytics 8.4.80,
OpenCV 4.13.0 and PyTorch 2.4.1 CPU (`torch.cuda.is_available() == False`). The video
reported 1764 frames, 720 x 1280, at approximately 29.983 FPS.

Frames `0, 15, ..., 285` were sampled. Recorded model and mask evidence:

```text
task: segment
names: {0: carton}
sampled frames: 20
frames with >=1 carton mask: 19
frames without a carton mask: [285]
first carton mask frame: 0
first carton confidence: 0.9164624214172363
first mask shape: [640, 384]
all masks on frame 0: [2, 640, 384]
```

This proves the supplied file can produce carton instance masks on the uploaded video. It
does not prove live-camera timestamp synchronization, depth quality, OBB quality, grasp
candidate availability or physical grasp success.

## 3. Automated regression evidence

Tests were run with bytecode and pytest cache output disabled. The focused command covered
the model selector, detector, RGB-D/mask synchronization, depth filtering, OBB estimation,
analytical gripper gate, remote candidate selection, rich-plan consumers and MuJoCo client/
server protocol. The freshly generated isolated Catkin Python path was prepended so the
tests could not import the stale main-workspace `StartGrasp` definition:

```bash
export PYTHONPATH=/tmp/alicia-task11-catkin.idNazc/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/zhuyupei/alicia_wa_full/devel/lib/python3/dist-packages:$PYTHONPATH

PYTHONDONTWRITEBYTECODE=1 python3 -B -m pytest -p no:cacheprovider \
  src/alicia_flexible_grasp_supervisor/tests/test_perception_model_selection.py \
  src/alicia_flexible_grasp_supervisor/tests/test_yolov8_detector.py \
  src/alicia_flexible_grasp_supervisor/tests/test_perception_depth_selection.py \
  src/alicia_flexible_grasp_supervisor/tests/test_camera_overlay.py \
  src/alicia_flexible_grasp_supervisor/tests/test_realsense_depth_scale.py \
  src/alicia_flexible_grasp_supervisor/tests/test_rgbd_snapshot.py \
  src/alicia_flexible_grasp_supervisor/tests/test_object_geometry.py \
  src/alicia_flexible_grasp_supervisor/tests/test_gripper_geometry.py \
  src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_candidate_selection.py \
  src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py \
  src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_control_widget.py \
  src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_sequence.py \
  src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py \
  src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_client.py \
  src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_server_protocol.py -q
```

Final result after the URL wiring and MASK freshness review fixes:
`436 passed, 1 skipped, 3 warnings in 13.59s`.

The protocol-only command was:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B -m pytest -p no:cacheprovider \
  src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_server_protocol.py -q
```

Result: `94 passed, 1 skipped`. The skipped test requires a host with the real MuJoCo
package and mesh-backed Alicia model; it was not converted into a pass.

The complete suite at Task 11 HEAD recorded `593 passed, 1 skipped, 3 warnings`. After
`fd5508f` bound `remote_grasp6d_url` to both
`/grasp_6d/remote/server_url` and `/mujoco_digital_twin/server_url`, the complete suite
recorded `594 passed, 1 skipped, 3 warnings in 16.38s`:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B -m pytest -p no:cacheprovider \
  src/alicia_flexible_grasp_supervisor/tests -q
```

The URL regression verifies launch wiring. Live deployment must still compare the two
rosparams after launch. With the final MASK freshness review fix included, the same complete
suite recorded `602 passed, 1 skipped, 3 warnings in 14.37s`.

The HTTP-boundary tests opened only temporary `127.0.0.1` listener ports. They required
loopback-socket permission in the test sandbox and did not contact WSL, the robot or any
external network.

## 4. ROS interface generation

An isolated catkin build at `/tmp/alicia-task11-catkin.idNazc` generated `7` message types
and `9` service types. The relevant new execution types are `ObjectGeometry`,
`Grasp6DPlan`, and the changed `StartGrasp` service:

```text
StartGrasp request:  bool execute, string plan_id
StartGrasp response: bool success, string message
StartGrasp MD5:      5d246499be275f0453d1db3b1be742a6
```

Deployment therefore requires a clean rebuild, a new `source devel/setup.bash`, and restart
of every GUI/node/service client compiled against the old service. A mixed-MD5 ROS graph is
not accepted.

## 5. Safety properties covered by tests

The automated suites cover these contracts:

- exactly one of `original`, `carton`, or `carton_segment` is active; a model change clears
  actionable perception and rich-plan state;
- `carton_segment` requires a matching non-empty instance mask and fails closed when the
  three-frame RGB/depth/mask/object window cannot be formed;
- mask absence, empty output and freshness failure are distinct: no mask ever received is
  `MASK_MISSING`; the latest all-zero mask is `MASK_EMPTY`; a previously observed non-empty
  mask that cannot participate in the required number of fresh, exact-timestamp
  RGB-D-mask-object samples is `MASK_STALE`; all three invalidate geometry and the rich
  plan;
- one request waits for three stable frames; one click initiates an attempt and does not
  guarantee a plan;
- OBB, support plane, depth quality, candidate counts and analytical gate counts remain
  bound to the same snapshot;
- required opening is OBB projection on the jaw axis plus 2 mm clearance on each side and
  must not exceed the fixed 50 mm inner-gap contract;
- `/grasp_6d/plan` is legacy visualization only; only a fresh valid
  `/grasp_6d/plan_enriched` can be execution authority;
- `plan_id` is checked before execution, after WSL network return and before physical
  actions; stop, expiry and replacement revoke the old authority;
- MuJoCo requires schema 2, current joints, four poses, the fixed gripper, carton OBB and
  detected support plane;
- a response is accepted only when the exact `plan_id`, finite score and all five explicit
  booleans pass; network and malformed-response paths fail closed regardless of
  `allow_execution_on_error`.

These are unit/integration guarantees under mocks and deterministic fixtures. They are not
evidence that the current camera or WSL host satisfies the same contracts at runtime.

## 6. Hardware and live-service checkpoint

The user confirmed the mechanical arm was powered off and its serial hardware was
disconnected. Read-only checks found:

| Check | Observation | Decision |
| --- | --- | --- |
| `rosnode list` | no reachable ROS master | no live ROS graph evidence |
| `/dev/video*` | no matching device | no camera-only acceptance possible |
| `/dev/alicia_arm`, `/dev/ttyACM*`, `/dev/ttyUSB*`, serial-by-id | no arm/TTY device | real arm remains disconnected |
| local `import mujoco` | module unavailable | real local `MUJOCO_SMOKE` not run |

Consequently, the camera-only acceptance is **NOT EXECUTED**. No success log, live contour,
header comparison, depth-quality sample, OBB, support plane or live failure matrix is
claimed in this report.

The configured WSL address was checked at `http://172.23.132.97:8000`:

| Request | Observation | Decision |
| --- | --- | --- |
| `GET /health` | reachable old GraspNet-only health response; no `digital_twin` component | not the combined service |
| `POST /simulate_grasp` | unknown path | MuJoCo gate unavailable |

Therefore real WSL MuJoCo acceptance is **NOT EXECUTED / BLOCKED**. The reachable old
GraspNet-only process must not authorize motion. No MuJoCo success response, collision/
contact/lift log, or viewer output is claimed.

## 7. Current gaps and non-claims

- `MUJOCO_SMOKE=1` remains unexecuted on a MuJoCo-equipped WSL host.
- Camera-only model switching, target-page contour, shared RGB/depth timestamps and
  three-frame stability remain unexecuted on live hardware.
- Deliberate real-backend IK, collision, contact and lift failures remain unrecorded.
- The server is headless and has no built-in viewer. Logs and `diagnosis` are not visual
  evidence; a separate viewer must be launched and identified if visual evidence is wanted.
- The uploaded video checkpoint smoke is offline evidence only.
- No force, tactile or slip behavior was evaluated.

## 8. Future WSL acceptance commands

Run these commands only after the combined code is present on WSL. Do not add any mock flag.

### 8.1 Synchronize the old WSL checkout and start the combined server

The currently reachable WSL endpoint is confirmed to be the old GraspNet-only service.
Before starting acceptance, preserve any unknown WSL-local changes and synchronize the
complete checkout to the approved feature version. Do not copy only the HTTP server file.
Record the exact WSL commit and dirty status:

```bash
cd ~/grasp6d_ws/Robot-Controller
git status --short
git rev-parse HEAD

test -x tools/start_mujoco_digital_twin_wsl.sh
test -f tools/mujoco_digital_twin_server.py
test -f src/arm-mujoco/synriard/mjcf/Alicia_D_v5_6/Alicia_D_v5_6_gripper_50mm.xml
rg -n "'/simulate_grasp'|'/predict'" tools/mujoco_digital_twin_server.py
```

If this WSL checkout has no Git remote containing the feature branch, use the project's
controlled whole-checkout transfer process. A dirty checkout must be reviewed rather than
overwritten. The acceptance remains blocked until the recorded WSL version supplies both
routes and the matching mesh/protocol tests.

Stop the old GraspNet-only process with `Ctrl-C` in its terminal, then verify that its WSL
listener is gone before starting the combined service:

```bash
ss -ltn | rg ':8000\b'
```

Expected: no output. If a listener remains, identify and normally stop that old process;
do not start a competing service on the same port.

```bash
conda activate grasp6d118
cd ~/grasp6d_ws/Robot-Controller

export GRASPNET_BASELINE_ROOT=/home/lv/grasp6d_ws/graspnet-baseline
export GRASPNET_CHECKPOINT=/home/lv/grasp6d_ws/checkpoints/checkpoint-rs.tar
export GRASPNET_DEVICE=cuda:0
export MUJOCO_ALICIA_MODEL_XML="$PWD/src/arm-mujoco/synriard/mjcf/Alicia_D_v5_6/Alicia_D_v5_6_gripper_50mm.xml"

./tools/start_mujoco_digital_twin_wsl.sh \
  --pass-score 80 \
  --min-lift-success-m 0.015 \
  --warmup
```

In a second WSL terminal:

```bash
cd ~/grasp6d_ws/Robot-Controller
curl -fsS http://127.0.0.1:8000/health | tee /tmp/alicia-mujoco-health.json

python3 - <<'PY'
import json
with open('/tmp/alicia-mujoco-health.json', 'r', encoding='utf-8') as stream:
    health = json.load(stream)
assert health.get('ok') is True, health
assert 'grasp_backend' in health, health
assert 'digital_twin' in health, health
assert health['digital_twin'].get('ok') is True, health
assert health['digital_twin'].get('backend') == 'mujoco', health
print(json.dumps(health, indent=2, ensure_ascii=False))
PY
```

Run the real compile/mesh smoke on that same host:

```bash
MUJOCO_SMOKE=1 PYTHONDONTWRITEBYTECODE=1 python3 -B -m pytest \
  -p no:cacheprovider \
  src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_server_protocol.py -q
```

The previously skipped smoke must execute and pass. A skip, missing mesh, model-contract
failure or MuJoCo import error leaves authorization blocked.

### 8.2 Send one fresh rich plan without calling a motion service

Start this helper in the Ubuntu ROS environment before generating a new plan. It listens for
a fresh valid `/plan_enriched`, builds the exact schema-2 payload from that plan and current
`/joint_states`, and calls only `/simulate_grasp`:

This requires current `Joint1`--`Joint6` state obtained while motion remains disabled. Do not
invent joint values to turn the health check into an apparent physical acceptance; if no
safe `/joint_states` source is available, record the acceptance as blocked.

```bash
source devel/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 -B - <<'PY'
import json
import threading

import rospy
from sensor_msgs.msg import JointState
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
from alicia_flexible_grasp.vision.mujoco_digital_twin_client import (
    MujocoDigitalTwinClient,
    build_mujoco_payload,
)

rospy.init_node('mujoco_acceptance_probe', anonymous=True, disable_signals=True)
ready = threading.Event()
holder = {}

def plan_cb(message):
    age = rospy.Time.now().to_sec() - message.header.stamp.to_sec()
    if message.valid and message.plan_id and 0.0 <= age <= 2.0:
        holder['plan'] = message
        ready.set()

subscriber = rospy.Subscriber('/grasp_6d/plan_enriched', Grasp6DPlan, plan_cb, queue_size=1)
print('waiting up to 30 s for a newly generated valid rich plan')
if not ready.wait(30.0):
    raise SystemExit('no fresh valid rich plan received')
subscriber.unregister()

plan = holder['plan']
joint = rospy.wait_for_message('/joint_states', JointState, timeout=3.0)
config = rospy.get_param('/mujoco_digital_twin')
payload = build_mujoco_payload(plan, joint.name, joint.position, config)
client = MujocoDigitalTwinClient(config['server_url'], timeout_sec=float(config['timeout_sec']))
response = client.simulate_grasp(payload)
print(json.dumps({'request_plan_id': plan.plan_id, 'response': response}, indent=2, ensure_ascii=False))
PY
```

While it is waiting, trigger exactly one fresh planning attempt in another Ubuntu terminal:

```bash
source devel/setup.bash
rosservice call /grasp_6d/request_plan "trigger: true"
```

This probe does not call `/grasp/start`, `/supervisor/move_to_pose`,
`/supervisor/move_to_pose_linear` or `/supervisor/set_gripper`. Record the complete response.
Do not treat it as a pass unless the echoed ID matches and every required component is true.

For negative acceptance, derive four schema-2 request fixtures from a newly generated payload
and deliberately make one request unreachable, one intersect the support, one miss the
carton, and one have no positive lift. Give them distinct non-production `plan_id` values.
The helper below replaces only `snapshot_stamp_sec` in memory immediately before each POST,
so the component test is not hidden by an earlier `PLAN_STALE` rejection:

```bash
send_case() {
  SENT="/tmp/${1%.json}-sent.json"
  python3 -B -c 'import json,sys,time; p=json.load(open(sys.argv[1], encoding="utf-8")); p["snapshot_stamp_sec"]=time.time(); json.dump(p, sys.stdout, allow_nan=False)' "$1" | \
  tee "$SENT" | \
  curl -sS http://REPLACE_WITH_WSL2_IP:8000/simulate_grasp \
    -H 'Content-Type: application/json' \
    --data-binary @- | python3 -m json.tool
  sha256sum "$SENT"
}

send_case ik-failure.json
send_case collision-failure.json
send_case contact-failure.json
send_case lift-failure.json
```

The fixture contents and hashes must be attached to the evidence. Do not infer the expected
component from the filename: verify the returned `failure_code` and all five booleans. If a
fixture fails an earlier component than intended, correct and rerun it; do not relabel the
response.

## 9. Future camera-only acceptance commands

Keep the arm powered off/disconnected or otherwise disabled at the motion gateway. Camera
acceptance must not call `/grasp/start`.

```bash
ls -l /dev/video*

cd /home/zhuyupei/alicia_wa_full
source devel/setup.bash
export GRASP6D_URL=http://REPLACE_WITH_WSL2_OR_WINDOWS_IP:8000

roslaunch alicia_flexible_grasp_supervisor full_system.launch \
  start_real_arm:=false \
  start_camera:=true \
  start_tactile:=false \
  start_gui:=true \
  use_remote_grasp6d:=true \
  remote_grasp6d_url:="$GRASP6D_URL"
```

In a second terminal:

```bash
source devel/setup.bash

PREDICT_URL="$(rosparam get /grasp_6d/remote/server_url)"
TWIN_URL="$(rosparam get /mujoco_digital_twin/server_url)"
printf 'predict=%s\ntwin=%s\n' "$PREDICT_URL" "$TWIN_URL"
test "$PREDICT_URL" = "$TWIN_URL"

rosparam get /mujoco_digital_twin/enabled
rosparam get /mujoco_digital_twin/execution_gate_enabled
rosparam get /mujoco_digital_twin/allow_execution_on_error
rosparam get /mujoco_digital_twin/send_joint_state_in_request

timeout 6 rostopic hz /supervisor/camera/color/image_raw
timeout 6 rostopic hz /supervisor/camera/depth/image_raw
rostopic echo -n 1 /supervisor/camera/color/image_raw/header
rostopic echo -n 1 /supervisor/camera/depth/image_raw/header
rostopic echo -n 1 /perception/detector_status
rostopic echo -n 1 /perception/object_mask/header
```

Use the GUI to switch through `original`, `carton`, `carton_segment`, then back through them,
waiting for the matching `ready:<choice>` status each time. Confirm that only the target
recognition page draws the timestamp-matched segment contour.

For `carton_segment`, perform one click and record the whole attempt:

```bash
rosservice call /grasp_6d/request_plan "trigger: true"
rostopic echo -n 1 /grasp_6d/status
rostopic echo -n 1 /grasp_6d/object_geometry
rostopic echo -n 1 /grasp_6d/gate_audit
rostopic echo -n 1 /grasp_6d/plan_enriched
```

Repeat with deliberate occlusion/no mask, moving camera or arm joints, excessive width and
invalid support geometry. Each failure must clear the executable rich plan. A single failed
attempt is a valid fail-closed observation, not evidence that one-click planning is broken;
fix the condition and click again for a new three-frame window.

## 10. Evidence templates

Leave unknown fields blank. Never copy a value from a unit test into a live-evidence row.

### 10.1 Environment and launch

| Field | Recorded value |
| --- | --- |
| Date/time/time zone | `[待填写]` |
| ROS commit and dirty status | `[待填写]` |
| WSL commit and dirty status | `[待填写]` |
| Camera serial / firmware | `[待填写]` |
| WSL IP / Windows forwarding address | `[待填写]` |
| Combined `/health` JSON attachment | `[待填写]` |
| `digital_twin.backend` / MuJoCo version | `[待填写]` |
| GraspNet checkpoint path + hash | `[待填写]` |
| MJCF path + hash | `[待填写]` |
| Live carton mass / friction config | `[待填写]` |
| Predict URL | `[待填写]` |
| Twin URL | `[待填写]` |
| URL equality command result | `[待填写]` |
| `allow_execution_on_error` | `[待填写；必须为 false]` |
| `StartGrasp` MD5 | `[待填写]` |

### 10.2 Camera-only attempt

| Field | Recorded value |
| --- | --- |
| Model choice / detector status | `[待填写]` |
| Color header | `[待填写]` |
| Depth header | `[待填写]` |
| Mask/object header | `[待填写]` |
| Target-page contour / other-page absence | `[待填写；附截图或视频]` |
| One-click request start/end time | `[待填写]` |
| Fused frames | `[待填写]` |
| Valid depth points / ratio | `[待填写]` |
| Depth MAD | `[待填写]` |
| Support inlier ratio | `[待填写]` |
| OBB center / orientation / size | `[待填写]` |
| Raw and per-gate candidate counts | `[待填写]` |
| Candidate / required opening | `[待填写]` |
| Rich-plan ID / valid / age | `[待填写]` |
| Complete success or failure status text | `[待填写]` |

### 10.3 Fail-closed camera cases

| Deliberate condition | Expected safety effect | Actual failure code | Rich plan cleared? | Evidence attachment |
| --- | --- | --- | --- | --- |
| No segment mask has ever arrived | no executable plan; expect `MASK_MISSING` | `[待填写]` | `[待填写]` | `[待填写]` |
| Latest published segment mask is all zero | no executable plan; expect `MASK_EMPTY` | `[待填写]` | `[待填写]` | `[待填写]` |
| Mask was seen but cannot form the fresh exact-timestamp window | no executable plan; expect `MASK_STALE` | `[待填写]` | `[待填写]` | `[待填写]` |
| Unstable mask/target/camera/joints | no executable plan | `[待填写]` | `[待填写]` | `[待填写]` |
| Insufficient/noisy depth | no executable plan | `[待填写]` | `[待填写]` | `[待填写]` |
| Invalid support plane/OBB | no executable plan | `[待填写]` | `[待填写]` | `[待填写]` |
| Required opening > 50 mm | no executable plan | `[待填写]` | `[待填写]` | `[待填写]` |
| Model switch/reload | previous plan revoked | `[待填写]` | `[待填写]` | `[待填写]` |

### 10.4 Real MuJoCo cases

| Case | Request plan ID | Echoed plan ID | IK | Collision-free | Contact | Lift | Score | Failure code/reason | Fixture/log hash |
| --- | --- | --- | --- | --- | --- | --- | ---: | --- | --- |
| Fresh valid rich plan | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` |
| Deliberate IK failure | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` |
| Deliberate collision | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` |
| Deliberate contact failure | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` |
| Deliberate lift failure | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` | `[待填写]` |

For a valid-plan pass, additionally record carton size/pose, detected support plane, compiled
open-gap measurement, first two-sided contact width, object lift displacement and configured
minimum lift. If a separate viewer was used, record its exact command/version and attach the
capture; otherwise write `viewer: NOT RUN`.

## 11. Authorization decision

The code checkpoints, tests and interface build pass, but the following mandatory gates are
still absent:

1. camera-only acceptance on the real depth camera;
2. `MUJOCO_SMOKE=1` on the configured WSL host;
3. combined WSL `/health` with a healthy `digital_twin` component;
4. one fresh matching rich-plan MuJoCo pass and deliberate component failures;
5. final review of the completed live evidence.

Until all five are recorded, keep the arm powered off or motion-disabled, keep
`allow_execution_on_error=false`, and do not call the real-arm execution service.

**REAL-ARM AUTHORIZATION: BLOCKED**
