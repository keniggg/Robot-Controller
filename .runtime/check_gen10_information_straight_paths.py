#!/usr/bin/env python3
"""Offline diagnostic straight joint paths; no ROS initialization or calls."""
import json
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from genpy import Duration
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_flexible_grasp.robot.observation_path_guard import (
    ObservationPathError, SerialUrdfFk, validate_observation_trajectory,
)
from alicia_flexible_grasp.robot.observation_preview import candidate_scene
from alicia_flexible_grasp.robot.observation_view_guard import validate_observation_view_trajectory

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z/gen10_offline_ik')


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    source = json.loads((ROOT/'frozen_source/source.json').read_text())
    with np.load(str(ROOT/'frozen_source/source.npz')) as data:
        source.update({key: data[key].copy() for key in data.files})
    info = json.loads((ROOT/'information_split_result.json').read_text())
    fk = SerialUrdfFk(Path(source['source_provenance']['model_path']).read_text(),
                      ['Joint%d' % i for i in range(1, 7)])
    assert fk.model_sha256 == info['model_sha256']
    q0 = source['current_joint_positions'].tolist()
    error = info['results'][1]['audit']['joint_error_bounds_rad']
    geometry = source['geometry']
    scene = candidate_scene(SimpleNamespace(
        support_normal_base=geometry['support_normal_base'], support_offset_m=geometry['support_offset_m'],
        center_base=geometry['obb_center_base_m'], axes_base=geometry['R_base_obb'],
        size_xyz_m=geometry['obb_size_xyz_m']), source['binding']['snapshot_stamp_ns'])
    gripper = GripperGeometry(.05, .002, np.array([.0434,.0286,.06]),
                             np.array([.1175,.155,.0774]), .003)
    hold = JointTrajectoryPoint(positions=q0, velocities=[0.]*6, accelerations=[0.]*6)
    report = dict(offline=True, ros_calls=False, execution_authority=False,
        scope='Synthetic straight joint paths between frozen measured q and proposed FK endpoints; original guard functions only.',
        interpolation='positions only: linear Noetic polynomial, stationary desired-to-first-positive waypoint bridge',
        synthetic_duration_sec=5., source_binding=source['binding'], model_sha256=fk.model_sha256,
        joint_error_bounds_rad=error, results=[], limitations=[
            'No actual MoveIt path, whole-arm collision, controller state or runtime freshness validation.',
            'Controller desired hold is assumed equal to frozen measured q for this diagnostic only.',
            'Continuous physical guard covers Link6/7/8; following uncertainty certificate is support-only.',
            'Visibility guard is bounded sampled, not continuous visibility or occlusion proof.',
            'All actual fresh plan, complete path, execution and arrival gates remain necessary.'])
    camera = source['camera']
    for index in range(2):
        candidate = json.loads((ROOT/('information_perception_only_candidate%d.json' % index)).read_text())
        assert candidate['binding'] == source['binding']
        assert candidate['model_sha256'] == fk.model_sha256
        plan = RobotTrajectory()
        plan.joint_trajectory.joint_names = list(fk.names)
        plan.joint_trajectory.points = [
            JointTrajectoryPoint(positions=q0, time_from_start=Duration(0)),
            JointTrajectoryPoint(positions=candidate['q'], time_from_start=Duration(5))]
        row = dict(index=index, initial_q=q0, endpoint_q=candidate['q'])
        for label, check in (
            ('physical_path', lambda: validate_observation_trajectory(
                plan,scene,fk,gripper,.05,hold,joint_error_bounds_rad=error)),
            ('view_path', lambda: validate_observation_view_trajectory(
                plan,fk,source['tool_from_camera'],source['corners_base'],source['center_base'],
                source['points_base'],[camera[k] for k in ('fx','fy','cx','cy')],
                camera['width'],camera['height'],hold,**source['view_limits']['effective_visibility_limits']))):
            started = time.monotonic()
            try:
                row[label] = dict(ok=True, audit=check())
            except ObservationPathError as exc:
                row[label] = dict(ok=False, reason=str(exc), audit=getattr(exc,'audit',{}))
            row[label]['wall_sec'] = time.monotonic()-started
        report['results'].append(row)
        print(json.dumps(dict(index=index, checks={key: dict(ok=row[key]['ok'], reason=row[key].get('reason',''), wall_sec=row[key]['wall_sec']) for key in ('physical_path','view_path')})), flush=True)
    with (ROOT/'information_straight_path_review.json').open('x') as output:
        json.dump(report,output,indent=2,allow_nan=False)


if __name__ == '__main__':
    main()
