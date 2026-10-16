from copy import deepcopy
from dataclasses import dataclass

import numpy as np
from tf.transformations import quaternion_matrix


@dataclass
class Grasp6DPlan:
    pregrasp: object
    approach: object
    grasp: object
    lift: object


def make_grasp_sequence_from_grasp_pose(
    grasp_pose,
    pregrasp_distance_m=0.08,
    approach_offset_m=0.015,
    lift_height_m=0.05,
    tool_approach_axis='x',
):
    """Create a motion sequence from a 6D gripper pose in base frame."""
    pregrasp = _offset_along_approach_axis(
        grasp_pose,
        -abs(float(pregrasp_distance_m)),
        tool_approach_axis,
    )
    approach = _offset_along_approach_axis(
        grasp_pose,
        -abs(float(approach_offset_m)),
        tool_approach_axis,
    )
    grasp = deepcopy(grasp_pose)
    lift = deepcopy(grasp_pose)
    lift.pose.position.z += float(lift_height_m)
    return Grasp6DPlan(pregrasp=pregrasp, approach=approach, grasp=grasp, lift=lift)


def _offset_along_approach_axis(pose_stamped, distance_m, tool_approach_axis='x'):
    pose = deepcopy(pose_stamped)
    axis = _approach_axis_xyz(pose_stamped, tool_approach_axis)
    pose.pose.position.x += axis[0] * float(distance_m)
    pose.pose.position.y += axis[1] * float(distance_m)
    pose.pose.position.z += axis[2] * float(distance_m)
    return pose


def _approach_axis_xyz(pose_stamped, tool_approach_axis='x'):
    q = pose_stamped.pose.orientation
    mat = quaternion_matrix([float(q.x), float(q.y), float(q.z), float(q.w)])
    name = str(tool_approach_axis or 'x').strip().lower()
    sign = -1.0 if name.startswith('-') else 1.0
    name = name.lstrip('+-')
    if name not in ('x', 'y', 'z'):
        raise ValueError('tool approach axis must be x, y, z, -x, -y, or -z')
    axis = sign * np.asarray(mat[:3, {'x': 0, 'y': 1, 'z': 2}[name]], dtype=float)
    norm = float(np.linalg.norm(axis))
    if norm <= 1e-9:
        return np.array([0.0, 0.0, 1.0], dtype=float)
    return axis / norm
