"""Bounded diagnostic IK/FK/validity requests; no trajectory or actuation API."""
import argparse
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import time

parser = argparse.ArgumentParser()
parser.add_argument('--inputs', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--all', action='store_true')
args = parser.parse_args()
mount = Path('/media/zhuyupei/SanStand')
assert mount.is_mount() and mount in args.output.resolve().parents
args.output.mkdir(exist_ok=False)
os.environ['ROS_LOG_DIR'] = str(args.output / 'ros')
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionFK, GetPositionFKRequest, GetStateValidity, GetStateValidityRequest

data = json.loads(args.inputs.read_text())
rospy.init_node('gen14_endpoint_diagnostic_readonly', anonymous=True, disable_rosout=True)
description = rospy.get_param('/robot_description')
assert hashlib.sha256(description.encode()).hexdigest() == data['model_sha256']
services = {}
for name, kind in [('compute_ik', GetPositionIK), ('compute_fk', GetPositionFK), ('check_state_validity', GetStateValidity)]:
    rospy.wait_for_service('/' + name, timeout=3)
    services[name] = rospy.ServiceProxy('/' + name, kind)
seed = RobotState()
seed.joint_state.name = data['recorded_joint_seed']['name']
seed.joint_state.position = data['recorded_joint_seed']['position']
baseline_fk_request = GetPositionFKRequest(robot_state=seed, fk_link_names=[data['ik_link_name']])
baseline_fk_request.header.frame_id = data['frame_id']
baseline_fk = services['compute_fk'](baseline_fk_request)
assert baseline_fk.error_code.val == 1
baseline = []
for collision in (False, True):
    baseline_req = GetPositionIKRequest()
    baseline_req.ik_request.group_name = data['group_name']
    baseline_req.ik_request.ik_link_name = data['ik_link_name']
    baseline_req.ik_request.robot_state = deepcopy(seed)
    baseline_req.ik_request.pose_stamped = baseline_fk.pose_stamped[0]
    baseline_req.ik_request.pose_stamped.header.stamp = rospy.Time(0)
    baseline_req.ik_request.timeout = rospy.Duration(.15)
    baseline_req.ik_request.avoid_collisions = collision
    result = services['compute_ik'](baseline_req)
    baseline.append(dict(avoid_collisions=collision,error_code=result.error_code.val))
(args.output / 'baseline_fk_ik.json').write_text(json.dumps(baseline,indent=2)+'\n')
print('baseline', json.dumps(baseline), flush=True)
assert baseline[0]['error_code'] == 1, 'recorded seed FK->IK failed: inspect solver before interpreting targets'
rows = data['all_220_in_original_first_path_dispatch_order'] if args.all else data['selected_probes']
results = []
started = time.monotonic()
with (args.output / 'queries.jsonl').open('x') as stream:
    for row in rows:
        for seconds in ([.025] if args.all else [.02, .15]):
            for collision in (False, True):
                if time.monotonic() - started > 42:
                    break
                request = GetPositionIKRequest()
                ik = request.ik_request
                ik.group_name = data['group_name']
                ik.ik_link_name = data['ik_link_name']
                ik.robot_state = deepcopy(seed)
                ik.avoid_collisions = collision
                ik.timeout = rospy.Duration.from_sec(seconds)
                ik.pose_stamped.header.frame_id = data['frame_id']
                # Static recorded base-frame pose, no historical timestamp transformed as fresh evidence.
                values = row['target_pose_xyz_xyzw']
                assert len(values) == 7 and all(math.isfinite(v) for v in values)
                for obj, names, values_part in ((ik.pose_stamped.pose.position, 'xyz', values[:3]), (ik.pose_stamped.pose.orientation, 'xyzw', values[3:])):
                    for name, value in zip(names, values_part): setattr(obj, name, value)
                before = time.monotonic()
                result = services['compute_ik'](request)
                report = dict(candidate_index=row['candidate_index'], distance_m=row['distance_m'],
                    tilt_deg=row['tilt_deg'], roll_deg=row['roll_deg'],
                    original_strict_checked=row['original_strict_result'].get('checked', False),
                    avoid_collisions=collision, timeout_sec=seconds,
                    elapsed_sec=time.monotonic()-before, error_code=result.error_code.val)
                if result.error_code.val == 1:
                    report['joint_names'] = list(result.solution.joint_state.name)
                    report['joint_positions'] = list(result.solution.joint_state.position)
                    valid = services['check_state_validity'](GetStateValidityRequest(robot_state=result.solution, group_name=data['group_name']))
                    report['state_valid'] = valid.valid
                    report['contacts'] = [(c.contact_body_1,c.contact_body_2,c.depth) for c in valid.contacts]
                    fk_request = GetPositionFKRequest(robot_state=result.solution, fk_link_names=[data['ik_link_name']])
                    fk_request.header.frame_id = data['frame_id']
                    fk = services['compute_fk'](fk_request)
                    report['fk_error_code'] = fk.error_code.val
                    if fk.error_code.val == 1:
                        actual = fk.pose_stamped[0].pose
                        report['fk_position_error_m'] = math.sqrt(sum((getattr(actual.position,k)-v)**2 for k,v in zip('xyz',values[:3])))
                        dot = abs(sum(getattr(actual.orientation,k)*v for k,v in zip('xyzw',values[3:])))
                        report['fk_orientation_error_rad'] = 2*math.acos(min(1.,dot))
                stream.write(json.dumps(report)+'\n'); stream.flush()
                results.append(report)
                print(json.dumps(report), flush=True)
            if time.monotonic()-started > 42: break
        if time.monotonic()-started > 42: break
summary = dict(inputs=str(args.inputs), inputs_sha256=hashlib.sha256(args.inputs.read_bytes()).hexdigest(),
    baseline_fk_ik=baseline,
    model_sha256=data['model_sha256'], recorded_seed=data['recorded_joint_seed'], results=results,
    elapsed_sec=time.monotonic()-started, scene='current live collision scene; not reconstructed historical scene',
    exact_historical_service_start=False, execution_authorized=False, motion_sent=False,
    new_measurement_or_user_confirmation_claimed=False)
(args.output / 'result.json').write_text(json.dumps(summary,indent=2)+'\n')
