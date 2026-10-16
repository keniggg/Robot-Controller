#!/usr/bin/env python3
"""Replay audited pose sequences through FK/IK/Cartesian computation only.

This deliberately does not construct MoveGroupCommander, action clients,
publishers, or an execution service. The collision scene is the live MoveIt
scene, not a reconstruction of the historical scene. Results never authorize
robot motion, and an audit's remote-side joint sample is only an approximate
historical planner start state.
"""
import argparse
import json
import math
from pathlib import Path
from types import SimpleNamespace

import rospy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from alicia_flexible_grasp.robot.moveit_planner import MoveItPlanner


def replay(evaluation, seconds):
    stages = evaluation['execution_sequence']['stages']
    if [s['stage'] for s in stages] != ['pregrasp', 'approach', 'grasp', 'lift']:
        raise ValueError('expected exact four-stage contact sequence')
    targets = []
    for stage in stages:
        if stage['frame_id'] != 'base_link':
            raise ValueError('replay must not transform or relocate recorded poses')
        pose = PoseStamped()
        pose.header.frame_id = stage['frame_id']
        stamp = int(stage['stamp_ns'])
        pose.header.stamp = rospy.Time(stamp // 10**9, stamp % 10**9)
        for field, values, names in (
            (pose.pose.position, stage['position_m'], 'xyz'),
            (pose.pose.orientation, stage['quaternion_xyzw'], 'xyzw'),
        ):
            if len(values) != len(names) or not all(math.isfinite(float(v)) for v in values):
                raise ValueError('invalid recorded pose')
            for name, value in zip(names, values):
                setattr(field, name, float(value))
        targets.append(pose)
    recorded = evaluation['moveit_input_joint_state']
    if not recorded.get('available'):
        raise ValueError('recorded virtual joint seed unavailable')
    state = RobotState()
    state.joint_state.name = recorded['name']
    state.joint_state.position = recorded['position_rad']
    MoveItPlanner._joint_state_positions(state)
    planner = MoveItPlanner.__new__(MoveItPlanner)
    planner.ready, planner.error = True, None
    planner.robot = SimpleNamespace(get_current_state=lambda: state)
    planner.manipulator = SimpleNamespace(get_end_effector_link=lambda: 'tool0',
                                         clear_pose_targets=lambda: None)
    planner.manipulator_group = 'alicia'
    defaults = {
        'orientation_resolution_step_rad': 0.02,
        'orientation_resolution_max_candidates': 96,
        'orientation_resolution_ik_timeout_sec': 0.05,
        'orientation_resolution_repeatability_tolerance_rad': 1e-6,
        'cartesian_eef_step_m': 0.003,
        'cartesian_jump_threshold': 0.0,
        'cartesian_min_fraction': 0.98,
    }
    configuration = {name: rospy.get_param('/robot/' + name, value)
                     for name, value in defaults.items()}
    for name, value in configuration.items():
        setattr(planner, name, value)
    counts = {'ik': 0, 'fk': 0, 'cartesian': 0}
    for label, method in [('ik', '_inverse_kinematics_from_state'),
                          ('fk', '_forward_kinematics_from_state'),
                          ('cartesian', '_plan_cartesian_from_start_state')]:
        original = getattr(planner, method)

        def counted(*args, _original=original, _label=label, **kwargs):
            counts[_label] += 1
            return _original(*args, **kwargs)

        setattr(planner, method, counted)
    started = rospy.Time.now().to_sec()
    ok, code, stage, poses, metrics, reason = planner.resolve_free_space_orientations(
        targets, [s['stage'] for s in stages], [False, True, True, False],
        [True, False, False, True], deadline_sec=started + seconds)
    return {
        'success': ok, 'code': code, 'failed_stage': stage, 'reason': reason,
        'elapsed_sec': rospy.Time.now().to_sec() - started, 'service_calls': counts,
        'metrics': metrics, 'configuration': configuration,
        'resolved_stage_count': len(poses),
        'execution_authorized': False,
        'historical_start_exact': bool(recorded.get('service_start_state_exact', False)),
        'scene': 'live MoveIt collision scene; recorded base-frame poses and virtual seed',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('audit', type=Path)
    parser.add_argument('--indices', type=int, nargs='+', default=[0])
    parser.add_argument('--seconds', type=float, default=15.0)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 60:
        parser.error('--seconds must be in (0, 60] for each bounded replay')
    data = json.loads(args.audit.read_text())
    evaluations = data['stable_evaluations']
    if any(i < 0 or i >= len(evaluations) for i in args.indices):
        parser.error('index outside stable_evaluations')
    rospy.init_node('grasp_sequence_replay_readonly', anonymous=True, disable_signals=True)
    for index in args.indices:
        result = replay(evaluations[index], args.seconds)
        result.update(index=index, request_id=data.get('request_id'))
        print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)


if __name__ == '__main__':
    main()
