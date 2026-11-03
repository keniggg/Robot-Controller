"""A bounded planning proposal from one failed compensated endpoint.

Balance the nominal and command tool origins in the support tangent plane.
The target, orientation and contact height stay fixed. This heuristic does
not certify the new endpoint, an IK branch, a path or future servo response.
Every proposed family must pass the caller's original complete checks.
"""
import math

import numpy as np

from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.robot.pregrasp_compensation import pose_matrix
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES


def _vector(value, size):
    if (not isinstance(value, (list, tuple, np.ndarray)) or len(value) != size
            or any(isinstance(v, (bool, np.bool_)) or not isinstance(v, (int, float, np.number))
                   or not math.isfinite(float(v)) for v in value)):
        raise ValueError('finite numeric vector required')
    return np.asarray(value, dtype=float)


def contact_lateral_placement(*, failure, plan_id, source_stamp_ns, goal_pose,
                              support_normal_base, model_xml, maximum_translation_m):
    """Return at most one proposal, with no reuse of a motion verdict.

Only an exact, same-plan terminal counterexample from the compensated grasp
curve can seed this heuristic. Historical following is explicitly conditional;
the caller must acquire fresh following and replan the complete sequence.
Missing or inconsistent evidence leaves the existing search unchanged.
"""
    audit = dict(policy='balance_failed_contact_tool_origins_in_support_plane',
                 proposed=False, target_relocated=False, motion_authorized=False,
                 path_verified=False, future_following_verified=False)
    try:
        if (not isinstance(failure, dict) or failure.get('success') is not False
                or failure.get('plan_id') != plan_id
                or failure.get('scope') != 'read_only_candidate_constant_bias_prediction'
                or failure.get('execution_authorized') is not False
                or failure.get('failed_stage') != 'grasp'
                or failure.get('failure_code') != 'CONTACT_SEQUENCE_PRECHECK_FAILED'
                or not str(failure.get('reason', '')).startswith(
                    'predicted_command continuous geometry: target OBB collision at ')):
            raise ValueError('bound compensated endpoint failure unavailable')
        endpoint = failure.get('contact_terminal_failure', {})
        if (not isinstance(endpoint, dict) or type(endpoint.get('schema_version')) is not int
                or endpoint.get('schema_version') != 1
                or endpoint.get('plan_id') != plan_id
                or type(source_stamp_ns) is not int or source_stamp_ns <= 0
                or endpoint.get('source_stamp_ns') != source_stamp_ns
                or endpoint.get('curve') != 'predicted_command'
                or endpoint.get('stage') != 'grasp'
                or endpoint.get('endpoint_screen_only') is not True
                or endpoint.get('execution_authorized') is not False):
            raise ValueError('same-source terminal diagnostic unavailable')
        if (isinstance(maximum_translation_m, (bool, np.bool_))
                or not isinstance(maximum_translation_m, (int, float, np.number))
                or not math.isfinite(float(maximum_translation_m))
                or maximum_translation_m <= 0.):
            raise ValueError('original remaining translation bound unavailable')
        fk = SerialUrdfFk(model_xml, ARM_NAMES)
        sample = failure.get('source_sample', {})
        if (endpoint.get('model_sha256') != fk.model_sha256
                or failure.get('model_sha256') != fk.model_sha256
                or sample.get('model_sha256') != fk.model_sha256
                or tuple(sample.get('joint_names', ())) != ARM_NAMES
                or type(failure.get('epoch_ns')) is not int or failure['epoch_ns'] <= 0
                or sample.get('epoch_ns') != failure['epoch_ns']):
            raise ValueError('model or measured-bias context changed')
        bias = (_vector(sample.get('sdk_positions_rad'), 6)
                - _vector(sample.get('accepted_positions_rad'), 6))
        if np.max(np.abs(bias)) > .035:
            raise ValueError('following outside original contact contract')
        command_q = _vector(endpoint.get('command_endpoint_rad'), 6)
        command, nominal = fk(command_q), fk(command_q-bias)
        values = _vector(goal_pose, 7)
        if abs(float(np.linalg.norm(values[3:]))-1.) > 1e-6:
            raise ValueError('goal quaternion is not unit length')
        goal = pose_matrix(values)
        angle = math.acos(float(np.clip((np.trace(goal[:3,:3].T@nominal[:3,:3])-1.)/2., -1., 1.)))
        if np.linalg.norm(goal[:3,3]-nominal[:3,3]) > .0005 or angle > .005:
            raise ValueError('nominal endpoint does not describe this fixed goal')
        normal = _vector(support_normal_base, 3)
        if abs(float(np.linalg.norm(normal))-1.) > 1e-6:
            raise ValueError('support normal is not unit length')
        normal = normal/np.linalg.norm(normal)
        residual = command[:3,3]-nominal[:3,3]
        displacement = -.5*(residual-normal*float(normal@residual))
        distance = float(np.linalg.norm(displacement))
        if not 1e-9 < distance <= float(maximum_translation_m):
            raise ValueError('balanced placement is zero or outside original translation bound')
        proposed = goal.copy()
        proposed[:3,3] += displacement
        audit.update(proposed=True, plan_id=str(plan_id), source_stamp_ns=source_stamp_ns,
            model_sha256=fk.model_sha256, epoch_ns=failure['epoch_ns'],
            normal_translation_m=0., translation_base_m=displacement.tolist(),
            translation_distance_m=distance, maximum_translation_m=float(maximum_translation_m),
            command_minus_nominal_position_m=residual.tolist(), sdk_minus_encoder_rad=bias.tolist(),
            command_endpoint_rad=command_q.tolist(), nominal_endpoint_rad=(command_q-bias).tolist(),
            reason='proposal only; fresh geometry, IK, complete paths and execution checks required')
        return proposed, audit
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError) as exc:
        audit['reason'] = str(exc)
        return None, audit
