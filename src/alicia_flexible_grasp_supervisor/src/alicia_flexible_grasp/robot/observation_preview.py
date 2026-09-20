"""Bounded, read-only candidate path evidence; never execution authority.

The existing strict-pose RPC carries one space-free evidence field so its ROS
wire contract stays compatible. The final executor still replans/revalidates
against its committed scene, current reference and actual timed trajectory.
"""
import base64
import io
import json
import math
from types import SimpleNamespace

import numpy as np
from moveit_msgs.msg import RobotTrajectory

from .observation_path_guard import (
    FrozenObservationScene, ObservationPathError, wire_digest,
    observation_following_support_bound, validate_observation_trajectory,
    FOLLOWING_PATH_MAX_CHECKS, FOLLOWING_PATH_MAX_SECONDS,
)

MAX_EVIDENCE_BYTES = 2 * 1024 * 1024
SCOPE = 'candidate_path_only_not_execution_authority'


def encode_path_evidence(target, plan, hold, opening, mode, epoch, model_sha):
    stream = io.BytesIO()
    plan.serialize(stream)
    data = dict(scope=SCOPE, target_sha256=wire_digest(target),
                trajectory_sha256=wire_digest(plan),
                trajectory=base64.b64encode(stream.getvalue()).decode('ascii'),
                hold=list(hold.positions), opening=float(opening),
                reference_mode=str(mode), reference_epoch_ns=int(epoch),
                model_sha256=str(model_sha))
    payload = json.dumps(data, separators=(',', ':'), allow_nan=False).encode('utf-8')
    if len(payload) > MAX_EVIDENCE_BYTES:
        raise ObservationPathError('candidate path evidence exceeds size limit')
    return base64.b64encode(payload).decode('ascii')


def decode_path_evidence(token, target, fk):
    if not isinstance(token, str) or len(token) > 4 * MAX_EVIDENCE_BYTES // 3 + 4:
        raise ObservationPathError('candidate path evidence exceeds size limit')
    try:
        payload = base64.b64decode(token, validate=True)
        if len(payload) > MAX_EVIDENCE_BYTES:
            raise ValueError('oversized payload')
        data = json.loads(payload)
        if (data['scope'] != SCOPE or data['target_sha256'] != wire_digest(target)
                or data['model_sha256'] != fk.model_sha256
                or type(data['reference_epoch_ns']) is not int
                or data['reference_epoch_ns'] <= 0
                or data['reference_mode'] not in (
                    'preserved', 'stationary_sdk_reacquisition',
                    'frozen_sdk_handoff', 'initial_positive_enable')):
            raise ValueError('target/model/reference binding mismatch')
        plan = RobotTrajectory()
        plan.deserialize(base64.b64decode(data['trajectory'], validate=True))
        trajectory = plan.joint_trajectory
        hold = np.asarray(data['hold'], dtype=float)
        if (wire_digest(plan) != data['trajectory_sha256']
                or tuple(trajectory.joint_names) != fk.names
                or not 2 <= len(trajectory.points) <= 4096
                or hold.shape != (len(fk.names),) or not np.all(np.isfinite(hold))
                or not math.isfinite(data['opening']) or not 0. <= data['opening'] <= .05):
            raise ValueError('invalid trajectory or stationary reference')
        return data, plan, SimpleNamespace(positions=hold.tolist(),
                                          velocities=[0.] * len(hold),
                                          accelerations=[0.] * len(hold))
    except Exception as exc:
        raise ObservationPathError('invalid candidate path evidence: %s' % exc)


def qualify_candidate_path(plan, hold, scene, fk, gripper, opening, errors):
    """Distinguish an impossible start from an endpoint or intermediate failure.

    Start and goal checks are necessary but not sufficient. Only the complete
    timed-curve and stationary controller-bridge proof can return ok=True.
    No measured residual is substituted for the configured tracking allowance.
    """
    report = dict(ok=False, scope=SCOPE, trajectory_sha256=wire_digest(plan),
                  failure_location='start', certifies_hardware_tracking=False)
    for location, q in (
            ('start', hold.positions),
            ('goal', plan.joint_trajectory.points[-1].positions)):
        report['failure_location'] = location
        try:
            bound = observation_following_support_bound(
                fk, q, errors, scene.normal, scene.offset, gripper, opening)
        except ObservationPathError as exc:
            report['reason'] = str(exc)
            return report
        report[location + '_support'] = bound
        if not bound['ok']:
            report['reason'] = bound['reason']
            return report
    report['failure_location'] = 'path'
    try:
        report['continuous_path'] = validate_observation_trajectory(
            plan, scene, fk, gripper, opening, hold,
            joint_error_bounds_rad=errors, max_checks=FOLLOWING_PATH_MAX_CHECKS,
            max_seconds=FOLLOWING_PATH_MAX_SECONDS)
    except ObservationPathError as exc:
        report['reason'] = str(exc)
        return report
    report.update(ok=True, failure_location='', reason='')
    return report


def candidate_scene(geometry, source_ns):
    """Match ObjectGeometry: Vector3 is float64, support offset is float32."""
    normal = np.asarray(geometry.support_normal_base, dtype=float)
    offset = float(np.float32(geometry.support_offset_m))
    norm = float(np.linalg.norm(normal))
    if not math.isfinite(norm) or abs(norm-1.) > 1e-6:
        raise ObservationPathError('candidate support normal is not unit length')
    return FrozenObservationScene('candidate-only', 'not-committed', int(source_ns),
        normal/norm, offset/norm, np.array(geometry.center_base, dtype=float),
        np.array(geometry.axes_base, dtype=float), np.array(geometry.size_xyz_m, dtype=float))
