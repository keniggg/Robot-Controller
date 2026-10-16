"""Shared conditional endpoint arithmetic. No feedback or motion authority.

The inputs may be virtual candidate states. They carry no source timestamps,
epochs or claims of real feedback. The execution caller must validate those
independently before and after the unchanged complete path checks.
"""
import math

import numpy as np

from .endpoint_correction import sdk_counts
from .pregrasp_compensation import PregraspCompensation, pose_matrix, remaining_budget_guide
from .stationary_following import ARM_NAMES, SDK_QUANTUM_RAD as Q, SerialUrdfFk


class HeldAxesPredictionError(ValueError):
    def __init__(self, code, evidence):
        super().__init__(code)
        self.evidence = evidence


def predict_held_axes_endpoint(fk, sdk_positions, predicted_positions, goal_values, *,
                               position_tolerance_m, orientation_tolerance_rad,
                               allow_already_converged=False, guide_provider=None):
    """Apply the execution guide and gates to a constant-bias prediction.

Failure means this bounded guide did not qualify; it is not a global proof
that the fixed goal is unreachable. No alternate joints or budgets are tried.
"""
    if not isinstance(fk, SerialUrdfFk) or tuple(fk.names) != ARM_NAMES:
        raise ValueError('PREGRASP_CANONICAL_FK_REQUIRED')
    if (not 0 < position_tolerance_m <= .010
            or not 0 < orientation_tolerance_rad <= math.radians(5)):
        raise ValueError('PREGRASP_TOLERANCE_CANNOT_BE_RELAXED')
    sdk, actual = np.asarray(sdk_positions, dtype=float), np.asarray(predicted_positions, dtype=float)
    if any(a.shape != (6,) or not np.all(np.isfinite(a)) for a in (sdk, actual)):
        raise ValueError('PREGRASP_HELD_AXES_INPUT_INVALID')
    goal = pose_matrix(goal_values)
    def residual(q):
        transform = fk(q)
        return (float(np.linalg.norm(goal[:3, 3]-transform[:3, 3])),
                math.acos(float(np.clip((np.trace(goal[:3, :3].T @ transform[:3, :3])-1)/2, -1., 1.))))
    counts = sdk_counts(sdk)
    position, angle = residual(actual)
    offset = sdk-actual
    evidence = dict(scope='conditional_held_axes_endpoint_prediction',
        fixed_goal_pose=list(goal_values), position_error_m=position,
        orientation_error_rad=angle, position_tolerance_m=float(position_tolerance_m),
        orientation_tolerance_rad=float(orientation_tolerance_rad),
        held_joints=['Joint2', 'Joint5'], maximum_total_counts=PregraspCompensation.MAX_TOTAL_COUNTS,
        sdk_minus_encoder_rad=offset.tolist(), initial_sdk_counts=counts.tolist(),
        virtual_start_positions_rad=actual.tolist(), future_following_verified=False,
        hardware_response_verified=False, execution_authorized=False, globally_optimal=False)
    def fail(code):
        evidence.update(success=False, code=code)
        raise HeldAxesPredictionError(code, evidence)
    if max(abs(offset)) > .035:
        fail('PREGRASP_FOLLOWING_OUTSIDE_CONTRACT')
    if allow_already_converged and position <= position_tolerance_m and angle <= orientation_tolerance_rad:
        evidence.update(success=True, code='PREGRASP_CONDITIONALLY_CONVERGED',
            proposed_delta_counts=[0]*6, predicted_position_error_m=position,
            predicted_orientation_error_rad=angle, predicted_positions_rad=actual.tolist(),
            predicted_command_positions_rad=sdk.tolist(), steps_predicted=0)
        return counts, evidence
    if position > .025 or angle > orientation_tolerance_rad:
        fail('PREGRASP_OUTSIDE_LOCAL_CAPTURE_RANGE')
    if guide_provider is not None:
        guide, _ = guide_provider(actual)
    else:
        guide, _ = remaining_budget_guide(fk, residual, actual, counts, counts,
            maximum_steps=PregraspCompensation.MAX_STEPS, steps=0,
            maximum_step_counts=PregraspCompensation.MAX_STEP_COUNTS,
            maximum_total_counts=PregraspCompensation.MAX_TOTAL_COUNTS,
            held_axes=PregraspCompensation.HELD_AXES, response_gains=np.ones(6),
            tolerances=(position_tolerance_m, orientation_tolerance_rad))
    target = counts+guide
    evidence['proposed_delta_counts'] = guide.tolist()
    if (not np.any(guide) or max(abs(guide)) > PregraspCompensation.MAX_TOTAL_COUNTS
            or any(guide[axis] != 0 for axis in PregraspCompensation.HELD_AXES)
            or np.any(target < 0) or np.any(target > 4095)):
        fail('PREGRASP_HELD_AXES_GUIDE_INVALID')
    command = (target-2048)*Q
    predicted = command-offset
    fk(command)
    p, a = residual(predicted)
    evidence.update(predicted_position_error_m=p, predicted_orientation_error_rad=a,
                    predicted_positions_rad=predicted.tolist(),
                    predicted_command_positions_rad=command.tolist(), steps_predicted=1)
    if (max(abs(command-predicted)) > .035 or p > position_tolerance_m
            or a > orientation_tolerance_rad or p >= position-1e-6):
        fail('PREGRASP_HELD_AXES_GUIDE_DOES_NOT_REACH_FIXED_GOAL')
    evidence.update(success=True, code='PREGRASP_HELD_AXES_GUIDE_QUALIFIED')
    return target, evidence
