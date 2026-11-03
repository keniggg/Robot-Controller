#!/usr/bin/env python3
"""Frozen gen28 FK/IK/plan diagnosis. No action, execution, or parameter writes.

Default only reconstructs recorded inputs. --compute calls the four allowlisted
MoveIt computation services. Collision-ignored IK is diagnostic only. Even a
returned plan is NOT validated by the original visibility/following/stop gates.
The collision scene is LIVE; historical target geometry remains frozen.
"""
import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
import rosbag
from tf.transformations import quaternion_matrix, quaternion_from_matrix

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
AUDIT = ROOT / 'ros_log/user_grasp_20261008T090751Z/source_snapshot_review/gen28_observation_frozen_audit.json'
BAG = ROOT / 'grasp_records/20261008T092818.564023Z_decf934e904a4ca1916b775e621829ce/keyframes.bag'
AUDIT_SHA = '554fffc15e60d2cbda26b11c4d1cdc8319e3e91441f4180cebb196e5542acb5f'
NAMES = ['Joint%d' % i for i in range(1, 7)]
SERVICES = {'/compute_fk', '/compute_ik', '/check_state_validity', '/plan_kinematic_path'}


def sha(data):
    return hashlib.sha256(data).hexdigest()


def transform(position, quaternion):
    result = quaternion_matrix(quaternion)
    result[:3, 3] = position
    return result


def turn(axis, radians):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    return quaternion_matrix(np.r_[axis * math.sin(radians / 2), math.cos(radians / 2)])[:3, :3]


def frozen_inputs(audit_path, bag_path):
    payload = audit_path.read_bytes()
    if sha(payload) != AUDIT_SHA:
        raise ValueError('audit changed: expected sealed gen28 request41 content')
    audit = json.loads(payload)
    evaluation = audit['stable_evaluations'][3]
    search = evaluation['observation_orientation_search']
    hint = search['scheduling']['endpoint_ik_search_hint']
    seed = hint['joint_seed']
    qmap = dict(zip(seed['name'], seed['position']))
    stamp = audit['snapshot_stamp_ns']
    handeyes, before, after = [], None, None
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, msg, _receipt in bag.read_messages(topics=['/tf_static', '/alicia_d/accepted_joint_states']):
            if topic == '/tf_static':
                for t in msg.transforms:
                    if t.header.frame_id.strip('/') == 'tool0' and t.child_frame_id.strip('/') == 'camera_link':
                        p, r = t.transform.translation, t.transform.rotation
                        handeyes.append(transform([p.x, p.y, p.z], [r.x, r.y, r.z, r.w]))
            else:
                sample = dict(stamp_ns=msg.header.stamp.to_nsec(), name=list(msg.name), position=list(msg.position))
                if sample['stamp_ns'] <= stamp and (before is None or sample['stamp_ns'] > before['stamp_ns']):
                    before = sample
                if sample['stamp_ns'] >= stamp and (after is None or sample['stamp_ns'] < after['stamp_ns']):
                    after = sample
    if not handeyes or any(not np.array_equal(handeyes[0], t) for t in handeyes):
        raise ValueError('missing or changing recorded handeye')
    for bracket in [before, after]:
        if bracket is None or abs(bracket['stamp_ns'] - stamp) > 200_000_000:
            raise ValueError('no close recorded accepted-joint bracket')
        actual = dict(zip(bracket['name'], bracket['position']))
        if any(abs(actual[n] - qmap[n]) > 1e-12 for n in NAMES):
            raise ValueError('audit seed differs from actual accepted joints around frozen exposure')
    handeye = handeyes[0]
    camera = np.asarray(audit['snapshot_transform']['T_base_camera_link'])
    view = evaluation['observation_view']
    metadata = view['observe_return_directed_missing_side']
    center = np.asarray(view['target_base_xyz'])
    axis = metadata['rotation_axis_base']
    endpoints = []
    for i, row in enumerate(search['variant_results']):
        tilt, roll = row['camera_tilt_offset_deg'], row['camera_roll_offset_deg']
        rotation = turn(axis, math.radians(tilt)) @ camera[:3, :3] @ turn([1., 0., 0.], math.radians(roll))
        tool_rotation = rotation @ handeye[:3, :3].T
        optical = rotation[:, 0] / np.linalg.norm(rotation[:, 0])
        matrix = np.eye(4)
        matrix[:3, :3] = tool_rotation
        matrix[:3, 3] = center - optical * row['camera_target_distance_m'] - tool_rotation @ handeye[:3, 3]
        endpoints.append(dict(index=i, recorded_search=row, T_base_tool0=matrix.tolist()))
    if len(endpoints) != 42:
        raise ValueError('unexpected recorded endpoint count')
    # Independent numerical binding against an exact saved production endpoint.
    stage = evaluation['execution_sequence']['stages'][0]
    recorded = transform(stage['position_m'], stage['quaternion_xyzw'])
    compatible = [e for e in endpoints if abs(e['recorded_search']['camera_roll_offset_deg'] - view['camera_roll_offset_deg']) < 1e-10
                  and abs(e['recorded_search']['camera_target_distance_m'] - view['camera_target_distance_m']) < 1e-10]
    if len(compatible) != 1:
        raise ValueError('recorded endpoint does not identify one reconstructed variant')
    residual = float(np.max(np.abs(recorded - np.asarray(compatible[0]['T_base_tool0']))))
    if residual > 1e-8:
        raise ValueError('reconstruction disagrees with saved production endpoint: %r' % residual)
    return dict(audit_sha256=AUDIT_SHA, source_stamp_ns=stamp, request_id=audit['request_id'],
        selection=audit['grasp_mode_selection'], seed=seed, accepted_joint_brackets=[before, after],
        robot_description_sha256=hint['robot_description_sha256'], T_tool0_camera_link=handeye.tolist(),
        T_base_camera_link=camera.tolist(), frozen_tool_reference=(camera @ np.linalg.inv(handeye)).tolist(),
        target_center_base_m=center.tolist(), reconstructed_endpoint_max_abs_error=residual,
        endpoints=endpoints, replay_geometry=audit['replay_geometry'])


def compute(data, emit, args):
    import rospy
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.msg import RobotState, Constraints, JointConstraint
    from moveit_msgs.srv import (GetPositionFK, GetPositionFKRequest, GetPositionIK, GetPositionIKRequest,
                                GetStateValidity, GetStateValidityRequest, GetMotionPlan, GetMotionPlanRequest)
    from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
    rospy.init_node('gen28_frozen_observation_readonly', anonymous=True, disable_signals=True, disable_rosout=True)
    description = rospy.get_param('/robot_description')
    if sha(description.encode()) != data['robot_description_sha256']:
        raise ValueError('live model differs from frozen audit')
    (args.output.parent / (args.output.stem + '.model.urdf')).write_text(description)
    model = SerialUrdfFk(description, NAMES)
    seed = RobotState()
    seed.joint_state.name = data['seed']['name']
    seed.joint_state.position = data['seed']['position']
    seed.is_diff = False
    q = np.array([dict(zip(seed.joint_state.name, seed.joint_state.position))[n] for n in NAMES])
    services = {}
    started = time.monotonic()

    def call(name, cls, request):
        if name not in SERVICES:
            raise ValueError('service outside computation allowlist')
        if time.monotonic() - started > args.seconds:
            raise TimeoutError('overall diagnostic budget exhausted')
        if name not in services:
            rospy.wait_for_service(name, timeout=2.)
            services[name] = rospy.ServiceProxy(name, cls, persistent=False)
        return services[name](request)

    def pose(matrix):
        matrix = np.asarray(matrix)
        p = PoseStamped(); p.header.frame_id = 'base_link'
        # Geometry already frozen in base_link: zero avoids stale TF lookups.
        for n, v in zip('xyz', matrix[:3, 3]): setattr(p.pose.position, n, float(v))
        for n, v in zip('xyzw', quaternion_from_matrix(matrix)): setattr(p.pose.orientation, n, float(v))
        return p

    def fk(state):
        request = GetPositionFKRequest(); request.header.frame_id = 'base_link'
        request.fk_link_names = ['tool0']; request.robot_state = deepcopy(state)
        r = call('/compute_fk', GetPositionFK, request)
        if r.error_code.val != 1 or len(r.pose_stamped) != 1:
            raise ValueError('FK failed: %s' % r.error_code.val)
        p = r.pose_stamped[0].pose
        return transform([p.position.x, p.position.y, p.position.z],
                         [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    def validity(state):
        request = GetStateValidityRequest(); request.group_name = 'alicia'; request.robot_state = deepcopy(state)
        r = call('/check_state_validity', GetStateValidity, request)
        return dict(valid=bool(r.valid), contacts=[dict(body1=c.contact_body_1, body2=c.contact_body_2, depth=c.depth) for c in r.contacts])

    def ik(matrix, avoid, label):
        request = GetPositionIKRequest(); r = request.ik_request
        r.group_name = 'alicia'; r.ik_link_name = 'tool0'; r.robot_state = deepcopy(seed)
        r.pose_stamped = pose(matrix); r.avoid_collisions = avoid; r.timeout = rospy.Duration(args.ik_timeout)
        t = time.monotonic(); response = call('/compute_ik', GetPositionIK, request)
        record = dict(kind='ik', label=label, avoid_collisions=avoid, timeout_sec=args.ik_timeout,
                      error_code=response.error_code.val, elapsed_sec=time.monotonic()-t)
        if response.error_code.val == 1:
            state = response.solution; solved = dict(zip(state.joint_state.name, state.joint_state.position))
            if not all(n in solved and math.isfinite(solved[n]) for n in NAMES):
                raise ValueError('malformed IK solution')
            actual = fk(state); target = np.asarray(matrix)
            record.update(joint_names=list(state.joint_state.name), joint_positions=list(state.joint_state.position),
                joint_max_delta_rad=max(abs(solved[n]-q[i]) for i,n in enumerate(NAMES)),
                fk_position_error_m=float(np.linalg.norm(actual[:3,3]-target[:3,3])),
                fk_rotation_matrix_max_error=float(np.max(np.abs(actual[:3,:3]-target[:3,:3]))),
                state_validity=validity(state))
            emit(record)
            if record['fk_position_error_m'] > 1e-4 or record['fk_rotation_matrix_max_error'] > 1e-3:
                raise ValueError('IK returned success but FK residual violates exact-pose diagnostic bound')
            return state, record
        emit(record)
        return None, record

    start_fk = fk(seed); start_validity = validity(seed)
    # Analytic spatial Jacobian from the same frozen URDF (no perturbed hardware states).
    current = np.eye(4); jacobian = np.zeros((6,6))
    for origin, index, axis in model.chain:
        current = current @ origin
        if index is not None:
            axis_base = current[:3,:3] @ axis
            jacobian[:3,index] = np.cross(axis_base, start_fk[:3,3]-current[:3,3])
            jacobian[3:,index] = axis_base
            current[:3,:3] = current[:3,:3] @ turn(axis, q[index])
    emit(dict(kind='baseline', live_scene=True, kinematics_config=rospy.get_param('/robot_description_kinematics', {}),
        fk_base_tool0=start_fk.tolist(), serial_fk_max_abs_error=float(np.max(np.abs(model(q)-start_fk))),
        frozen_tf_fk_max_abs_error=float(np.max(np.abs(np.asarray(data['frozen_tool_reference'])-start_fk))),
        spatial_jacobian=jacobian.tolist(), singular_values=np.linalg.svd(jacobian, compute_uv=False).tolist(),
        jacobian_rank_at_seed=int(np.linalg.matrix_rank(jacobian, tol=1e-8)), joint_limits=model.limits.tolist(),
        state_validity=start_validity))
    for avoid in [False, True]:
        state, _ = ik(start_fk, avoid, 'known_reachable_fk_seed')
        if state is None and not avoid:
            raise ValueError('FK(seed) fails IK(seed) even ignoring collisions: endpoint search not trustworthy')
    passing = []
    for endpoint in data['endpoints']:
        for avoid in [True, False]:
            solution, row = ik(endpoint['T_base_tool0'], avoid, 'recorded_endpoint_%d' % endpoint['index'])
            if avoid and solution is not None and row['state_validity']['valid']:
                passing.append((endpoint, solution))
    emit(dict(kind='endpoint_summary', collision_aware_solutions=len(passing), endpoint_count=42,
              path_or_motion_authorized=False))
    if args.plan_count and not start_validity['valid']:
        emit(dict(kind='plan_skipped', reason='frozen start collides with live scene')); return
    for endpoint, goal in passing[:args.plan_count]:
        request = GetMotionPlanRequest(); r = request.motion_plan_request
        r.group_name = 'alicia'; r.start_state = deepcopy(seed); r.num_planning_attempts = 1
        r.allowed_planning_time = 2.; r.max_velocity_scaling_factor = .3; r.max_acceleration_scaling_factor = .2
        constraints = Constraints(); positions = dict(zip(goal.joint_state.name, goal.joint_state.position))
        for name in NAMES:
            constraints.joint_constraints.append(JointConstraint(joint_name=name, position=positions[name],
                tolerance_above=1e-4, tolerance_below=1e-4, weight=1.))
        r.goal_constraints = [constraints]
        response = call('/plan_kinematic_path', GetMotionPlan, request).motion_plan_response
        trajectory = response.trajectory.joint_trajectory
        emit(dict(kind='plan', index=endpoint['index'], error_code=response.error_code.val,
            planning_time_sec=response.planning_time, joint_names=list(trajectory.joint_names),
            points=[dict(positions=list(p.positions), velocities=list(p.velocities), accelerations=list(p.accelerations),
                         time_ns=p.time_from_start.to_nsec()) for p in trajectory.points],
            original_visibility_following_stop_gates_checked=False, motion_authorized=False))
    if sha(rospy.get_param('/robot_description').encode()) != data['robot_description_sha256']:
        raise ValueError('model changed during diagnostic; results invalid')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit', type=Path, default=AUDIT); parser.add_argument('--bag', type=Path, default=BAG)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--compute', action='store_true')
    parser.add_argument('--ik-timeout', type=float, default=.02)
    parser.add_argument('--seconds', type=float, default=45.)
    parser.add_argument('--plan-count', type=int, choices=[0,1,2], default=0)
    args = parser.parse_args()
    if not 0 < args.ik_timeout <= .2 or not 0 < args.seconds <= 60:
        parser.error('bounded IK timeout <=.2s and overall start budget <=60s required')
    if not Path('/media/zhuyupei/SanStand').is_mount() or ROOT not in args.output.resolve().parents:
        parser.error('output must be directly on mounted SanStand experiment root')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as output:
        def emit(record):
            output.write(json.dumps(record, ensure_ascii=False, allow_nan=False)+'\n'); output.flush()
        try:
            data = frozen_inputs(args.audit, args.bag)
            emit(dict(kind='inputs', diagnostic_source_sha256=sha(Path(__file__).read_bytes()),
                computation_services=sorted(SERVICES), execution_authorized=False, **data))
            if args.compute: compute(data, emit, args)
            emit(dict(kind='completed', computation_performed=args.compute, execution_authorized=False))
            print(json.dumps(dict(output=str(args.output), endpoints=42, computation_performed=args.compute)))
        except Exception as exc:
            emit(dict(kind='error', error_type=type(exc).__name__, reason=str(exc), execution_authorized=False))
            raise


if __name__ == '__main__':
    main()
