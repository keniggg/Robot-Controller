"""Request-local immutable terms of the original endpoint support bound.

This accelerates proposal constraints only. The original complete endpoint
gate must still recompute its independent bound before admitting a proposal.
No sample, margin, first-order term or Taylor remainder is removed.
"""
from copy import deepcopy
import math

import numpy as np

from alicia_flexible_grasp.grasp.gripper_geometry import (
    GripperGeometry, _box_corners, _stage_boxes)
from alicia_flexible_grasp.robot.observation_path_guard import (
    ObservationPathError, SerialUrdfFk, _array, _axis_rotation)


class PreparedEndpointSupportBound:
    """One request's fixed model, CAD, opening, plane and error-box snapshot."""

    def __init__(self, fk, errors, normal, offset, gripper, opening):
        if not isinstance(fk, SerialUrdfFk) or not isinstance(gripper, GripperGeometry):
            raise ObservationPathError('endpoint bound requires the exact URDF/CAD model')
        epsilon = _array(errors, len(fk.names), 'endpoint error bounds')
        n = _array(normal, 3, 'endpoint support normal')
        d = float(offset)
        if (np.any(epsilon < 0.) or np.any(epsilon > math.pi)
                or not math.isfinite(d) or abs(np.linalg.norm(n)-1.) > 1e-6
                or not math.isfinite(opening) or not 0. <= opening <= gripper.max_inner_gap_m
                or gripper.support_clearance_m < .003):
            raise ObservationPathError('invalid endpoint error box or physical support contract')
        length = float(np.linalg.norm(n))
        n, d = n/length, d/length
        # Never retain mutable caller arrays or reuse this snapshot across
        # requests. A later source/model/contract requires a fresh instance.
        self._fk = deepcopy(fk)
        self._gripper = deepcopy(gripper)
        identity = np.eye(4)
        local = np.concatenate([
            _box_corners(center, identity[:3, :3], size)
            for _, center, size in _stage_boxes(identity, self._gripper, opening, 'y', 'z')
        ])
        radii = self._fk.point_motion_radii(float(np.max(np.linalg.norm(local, axis=1))))
        self._remainder = .5 * float(epsilon @ np.minimum.outer(radii, radii) @ epsilon)
        self._epsilon = np.array(epsilon, dtype=float, copy=True)
        self._normal = np.array(n, dtype=float, copy=True)
        self._offset = d
        self._local = local
        for value in (self._epsilon, self._normal, self._local):
            value.setflags(write=False)

    def __call__(self, goal):
        fk, epsilon, n = self._fk, self._epsilon, self._normal
        q = _array(goal, len(fk.names), 'endpoint goal')
        transform = fk(q)  # The original finite/shape and URDF-limit checks.
        chain_pose = np.eye(4)
        origins, axes = np.zeros((len(q), 3)), np.zeros((len(q), 3))
        for origin, index, axis in fk.chain:
            chain_pose = chain_pose @ origin
            if index is not None:
                origins[index] = chain_pose[:3, 3]
                axes[index] = chain_pose[:3, :3] @ axis
                chain_pose[:3, :3] = chain_pose[:3, :3] @ _axis_rotation(axis, q[index])
        points = self._local @ transform[:3, :3].T + transform[:3, 3]
        gradients = np.cross(axes[None, :, :], points[:, None, :]-origins[None, :, :]) @ n
        nominal = points @ n + self._offset
        linear = np.abs(gradients) @ epsilon
        lower = nominal-linear-self._remainder-1e-12
        return {
            'kind': 'conditional_endpoint_joint_error_box_support_bound',
            'robot_description_sha256': fk.model_sha256,
            'joint_names': list(fk.names),
            'goal_joint_positions_rad': q.tolist(),
            'joint_error_bounds_rad': epsilon.tolist(),
            'nominal_minimum_support_clearance_m': float(np.min(nominal)),
            'maximum_first_order_allowance_m': float(np.max(linear)),
            'second_order_allowance_m': self._remainder,
            'minimum_support_clearance_lower_bound_m': float(np.min(lower)),
            'required_support_clearance_m': float(self._gripper.support_clearance_m),
            'ok': bool(np.min(lower) >= self._gripper.support_clearance_m),
            'certifies_transient_tracking_or_calibration': False,
        }
