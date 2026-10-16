#!/usr/bin/env python3
"""Replay a closed-bag-derived observation fixture, without ROS control I/O.

The fixture retains exact executed path digests and the frozen model hash.
Candidate checks are counterfactual screening, not executed replacements or
hardware guarantees. This module creates no node, publisher or service client.
"""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import rospy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_flexible_grasp.robot.observation_path_guard import (
    FrozenObservationScene, SerialUrdfFk, ObservationPathError,
    observation_following_support_bound, observation_tracking_error_bounds,
    validate_observation_trajectory, wire_digest,
    FOLLOWING_PATH_MAX_CHECKS, FOLLOWING_PATH_MAX_SECONDS,
)


def replay(fixture, description):
    names = ['Joint%d' % i for i in range(1, 7)]
    fk = SerialUrdfFk(description, names)
    if fk.model_sha256 != fixture['model_sha256']:
        raise ValueError('fixture/model hash mismatch; do not mix calibration epochs')
    gripper = GripperGeometry(.05, .002, np.array([.0434, .0286, .06]),
                              np.array([.1175, .155, .0774]), .003)
    errors = observation_tracking_error_bounds(
        {name: {'trajectory': .12, 'goal': .035} for name in names}, names)
    result = {'read_only': True, 'plan_id': fixture['plan_id'],
              'source_audit_sha256': fixture['source_audit_sha256'],
              'model_sha256': fk.model_sha256, 'candidates': [], 'executed_paths': [],
              'replacement_path_executed': False, 'grasp_success_proven': False}
    for candidate in fixture['candidates']:
        row = dict(candidate)
        try:
            row['following_support'] = observation_following_support_bound(
                fk, row['q'], errors, fixture['normal'], fixture['offset'], gripper,
                .05)  # Same fully open candidate-screening envelope as production.
        except ObservationPathError as exc:
            row['following_support'] = {'ok': False, 'reason': str(exc)}
        result['candidates'].append(row)
    passing = [row for row in result['candidates'] if row['following_support']['ok']]
    result['selected_candidate'] = min(passing, key=lambda row:
        row['execution_duration_lower_bound_sec']) if passing else None
    for executed in fixture['executed']:
        plan = RobotTrajectory()
        plan.joint_trajectory.header.frame_id = 'base_link'
        plan.joint_trajectory.joint_names = executed['names']
        for p in executed['points']:
            point = JointTrajectoryPoint()
            for field in ('positions', 'velocities', 'accelerations'):
                setattr(point, field, p[field])
            secs, nsecs = divmod(p['time_ns'], 10**9)
            point.time_from_start = rospy.Duration(secs, nsecs)
            plan.joint_trajectory.points.append(point)
        proof = executed['proof']
        if wire_digest(plan) != proof['trajectory_sha256']:
            raise ValueError('executed trajectory digest mismatch')
        scene = FrozenObservationScene(fixture['plan_id'], proof['scene_sha256'], 1,
            np.array(fixture['normal']), fixture['offset'], np.array(fixture['center']),
            np.array(fixture['rotation']), np.array(fixture['size']))
        hold = SimpleNamespace(positions=executed['hold'], velocities=[0.]*6,
                               accelerations=[0.]*6)
        args = (plan, scene, fk, gripper, fixture['opening'], hold)
        row = {'trajectory_sha256': wire_digest(plan),
               'nominal': validate_observation_trajectory(*args)}
        try:
            row['following'] = validate_observation_trajectory(
                *args, joint_error_bounds_rad=errors,
                max_checks=FOLLOWING_PATH_MAX_CHECKS, max_seconds=FOLLOWING_PATH_MAX_SECONDS)
            row['following_passed'] = True
        except ObservationPathError as exc:
            row.update(following_passed=False, following_error=str(exc))
        result['executed_paths'].append(row)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture', required=True)
    parser.add_argument('--urdf', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    result = replay(json.loads(Path(args.fixture).read_text()), Path(args.urdf).read_text())
    with open(args.output, 'x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
