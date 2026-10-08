#!/usr/bin/env python3
"""Historical lift fixture through native read-only MoveIt service transports.

No task state mutations, lift service, action clients, control publishers, or
planning-scene writes. This is NOT current-scene execution authorization.
"""
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import io
import json
import os
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
STORE = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
if not Path('/media/zhuyupei/SanStand').is_mount():
    raise SystemExit('SanStand unavailable; no fallback')
OUT = STORE / 'ros_log/grasp_repair_resume_20260928T120606Z/lift_path_continuity' / ('native_readonly_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
OUT.mkdir(parents=True, exist_ok=False)
os.environ['ROS_LOG_DIR'] = str(OUT / 'roslog')

import numpy as np
import rospy
import yaml
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest, GetStateValidity
sys.path.insert(0, str(ROOT / 'src/alicia_flexible_grasp_supervisor/tests'))
from test_lift_path_continuity import recorded, point
from alicia_flexible_grasp.robot.lift_path_continuity import derive_lift_path
from alicia_flexible_grasp.robot.lift_moveit_adapter import ClosedLiftMoveItAdapter
from alicia_flexible_grasp.robot.observation_path_guard import wire_digest

calls = []
phase = 'initial_scene'
report = dict(scope='historical_fixture_native_readonly_MoveIt_transport_diagnostic',
              motion_authorized=False, current_scene_lift_authorized=False,
              actual_lift_verified=False, global_scene_mutation=False,
              task_phase_mutation=False, issued_motion_commands=0)
started = time.monotonic()


def wire_save(name, msg):
    stream = io.BytesIO(); msg.serialize(stream)
    payload = stream.getvalue(); (OUT / name).write_bytes(payload)
    return dict(path=name, sha256=hashlib.sha256(payload).hexdigest(), bytes=len(payload))


def transport(name, proxy):
    def call(req):
        n = len(calls)
        row = dict(index=n, service=name, phase=phase,
                   request=wire_save('%03d_request.roswire' % n, req))
        calls.append(row)
        then = time.monotonic()
        try:
            response = proxy(req)
            row['response'] = wire_save('%03d_response.roswire' % n, response)
            if name == '/check_state_validity':
                body = req.robot_state.attached_collision_objects[-1]
                row.update(valid=response.valid, object_frame=body.object.header.frame_id,
                           attachment_link=body.link_name, touch_links=list(body.touch_links),
                           retained_prior_attachments=len(req.robot_state.attached_collision_objects)-1,
                           is_diff=req.robot_state.is_diff,
                           joint_names=list(req.robot_state.joint_state.name),
                           positions=list(req.robot_state.joint_state.position),
                           contacts=[yaml.safe_load(str(c)) for c in response.contacts],
                           constraints=[yaml.safe_load(str(c)) for c in response.constraint_result],
                           object_pose=yaml.safe_load(str(body.object.pose)))
            return response
        except Exception as exc:
            row['error'] = str(exc)
            raise
        finally:
            row['elapsed_seconds'] = time.monotonic() - then
            (OUT / 'calls.json').write_text(json.dumps(calls, indent=2, ensure_ascii=False)+'\n')
    return call


try:
    rospy.init_node('historical_lift_readonly_diagnostic', anonymous=True, disable_signals=True)
    data, fk, plan, nominal, sample, opening = recorded()
    current_model = rospy.get_param('/robot_description')
    expected_hash = hashlib.sha256(data['robot_description'].encode()).hexdigest()
    current_hash = hashlib.sha256(current_model.encode()).hexdigest()
    report.update(historical_model_sha256=expected_hash, current_model_sha256=current_hash,
                  historical_epoch_ns=sample['epoch_ns'], historical_opening_m=opening,
                  source_bag=data['source_bag'], source_goal_id=data['source_goal_id'],
                  source_fixture_sha256=hashlib.sha256((ROOT/'src/alicia_flexible_grasp_supervisor/tests/fixtures/lift_following_20260928.json').read_bytes()).hexdigest())
    if current_hash != expected_hash:
        raise ValueError('CURRENT_MOVEIT_MODEL_DIFFERS_FROM_HISTORICAL_FIXTURE')
    shifted, frozen, derivation = derive_lift_path(nominal, plan, fk, sample, opening)
    report['derivation'] = derivation
    report['constraints_source'] = '/alicia_controller/constraints'
    constraints = rospy.get_param('/alicia_controller/constraints')
    stop = rospy.get_param('/alicia_controller/stop_trajectory_duration')
    rospy.wait_for_service('/check_state_validity', timeout=3.)
    rospy.wait_for_service('/get_planning_scene', timeout=3.)
    check = transport('/check_state_validity', rospy.ServiceProxy('/check_state_validity', GetStateValidity))
    scene = transport('/get_planning_scene', rospy.ServiceProxy('/get_planning_scene', GetPlanningScene))
    adapter = ClosedLiftMoveItAdapter(current_model, check, scene)
    req = GetPlanningSceneRequest()
    req.components.components = PlanningSceneComponents.ROBOT_STATE | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
    current_state = scene(req).scene.robot_state
    report['live_state_snapshot'] = yaml.safe_load(str(current_state))
    report['live_state_snapshot_sha256'] = wire_digest(current_state)
    # Use the native current full variable vector/attachments. _request replaces
    # arm+finger positions with explicitly historical hypothetical values.
    phase = 'complete_adapter_historical_nominal_and_shifted'
    try:
        report['adapter_result'] = adapter(nominal=nominal, shifted=shifted, frozen=frozen,
            start_state=current_state, sdk_hold=point(sample['sdk_positions_rad']),
            bodies=(), gripper=None, constraints=constraints, stop_duration=stop)
    except Exception as exc:
        report['adapter_rejection'] = str(exc)
    # These probes cannot rescue a rejected full check. They only ensure BOTH
    # native occupancy message shapes reach MoveIt when the full check stops.
    report['diagnostic_probes_are_not_retry_or_motion_authority'] = True
    for label, q in [('historical_measured_start', frozen.measured),
                     ('historical_shifted_terminal', np.asarray(shifted.joint_trajectory.points[-1].positions))]:
        for hypothesis in ('world', 'hypothetical_carried'):
            phase = 'native_probe/' + label + '/' + hypothesis
            check(adapter._request(q, current_state, frozen, hypothesis))
    phase = 'after_scene'
    report['scene_sha256_after_diagnostic'] = adapter.scene_digest()
    report['completed_native_interface_diagnostic'] = True
except Exception as exc:
    report['error'] = str(exc)
    report['traceback'] = traceback.format_exc()
finally:
    report['elapsed_seconds'] = time.monotonic() - started
    report['calls'] = calls
    report['source_sha256'] = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in [Path(__file__).resolve(), ROOT/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/lift_moveit_adapter.py', ROOT/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/lift_path_continuity.py']}
    report['service_counts'] = {name: sum(c['service']==name for c in calls)
                               for name in ['/check_state_validity','/get_planning_scene']}
    report['service_elapsed_seconds'] = {name: sum(c['elapsed_seconds'] for c in calls if c['service']==name)
                                        for name in report['service_counts']}
    (OUT/'native_moveit_report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps({k:report[k] for k in ['scope','motion_authorized','service_counts','service_elapsed_seconds','elapsed_seconds']}, indent=2))
    print('RESULT', report.get('adapter_rejection', report.get('error', 'completed')))
    print('ARCHIVE', OUT)
