"""Prefer arrival accuracy among already feasible nominal pregrasp endpoints.

The original selector establishes feasibility of both frozen poses. This one
bounded refinement minimizes normalized pregrasp position/orientation error
while retaining the original held-axis approach. It does not estimate future
following bias, authorize a trajectory, or enlarge a correction episode.
"""
import math
import time

import numpy as np
from scipy.optimize import minimize

from .endpoint_correction import sdk_counts
from .held_axes_prediction import predict_held_axes_endpoint, HeldAxesPredictionError
from .pregrasp_compensation import pose_matrix, PregraspCompensation
from .stationary_following import ARM_NAMES, SerialUrdfFk, SDK_QUANTUM_RAD as Q
from .following_model import (
    following_offset, following_evidence, validate_source_bias_model, FollowingModelError)


def refine_held_pregrasp_margin(fk, sample, pregrasp_goal, approach_goal, selected, *,
        position_tolerance_m, orientation_tolerance_rad, deadline_check=None,
        following_model=None):
    started = time.monotonic()
    if (not isinstance(fk, SerialUrdfFk) or tuple(fk.names) != ARM_NAMES
            or sample.get('model_sha256') != fk.model_sha256
            or tuple(sample.get('joint_names', ())) != ARM_NAMES):
        raise ValueError('HELD_PREGRASP_MODEL_MISMATCH')
    if (not 0 < position_tolerance_m <= .020
            or not 0 < orientation_tolerance_rad <= math.radians(10)):
        raise ValueError('PREGRASP_TOLERANCE_CANNOT_BE_RELAXED')
    sdk, actual, selected = [np.asarray(v, dtype=float) for v in
        (sample['sdk_positions_rad'], sample['accepted_positions_rad'], selected)]
    if any(v.shape != (6,) or not np.all(np.isfinite(v)) for v in (sdk, actual, selected)):
        raise ValueError('HELD_PREGRASP_FOLLOWING_INVALID')
    if following_model is not None:
        validate_source_bias_model(following_model, sample)
    try:
        offset = following_offset(sdk, actual, following_model=following_model)
    except FollowingModelError as exc:
        if str(exc) == 'FOLLOWING_OUTSIDE_CONTRACT':
            raise ValueError('PREGRASP_FOLLOWING_OUTSIDE_CONTRACT') from exc
        raise
    mapped = {'following_model': following_model} if following_model is not None else {}
    selected = (sdk_counts(selected)-2048)*Q
    fk(selected)
    pg, ag = pose_matrix(pregrasp_goal), pose_matrix(approach_goal)
    approach_tolerance = min(position_tolerance_m, .012)
    free = [i for i in range(6) if i not in PregraspCompensation.HELD_AXES]
    def check():
        if deadline_check is not None:
            deadline_check()
    def verify(command):
        check()
        _, pre = predict_held_axes_endpoint(fk, command, command-offset, pregrasp_goal,
            position_tolerance_m=position_tolerance_m,
            orientation_tolerance_rad=orientation_tolerance_rad, allow_already_converged=True,
            **mapped)
        _, approach = predict_held_axes_endpoint(fk, pre['predicted_command_positions_rad'],
            pre['predicted_positions_rad'], approach_goal,
            position_tolerance_m=approach_tolerance,
            orientation_tolerance_rad=orientation_tolerance_rad, allow_already_converged=True,
            **mapped)
        check()
        return dict(predicted_pregrasp=pre, predicted_approach=approach)
    original = verify(selected)
    cache = {}
    def residual(q, goal):
        if np.any(q < fk.limits[:, 0]) or np.any(q > fk.limits[:, 1]):
            return 1.e3, math.pi
        key = q.tobytes()
        if key not in cache:
            if len(cache) >= 64:
                cache.clear()
            cache[key] = fk(q)
        matrix = cache[key]
        return (float(np.linalg.norm(matrix[:3, 3]-goal[:3, 3])),
            math.acos(float(np.clip((np.trace(matrix[:3, :3].T@goal[:3, :3])-1)/2, -1., 1.))))
    def cost(q):
        p, a = residual(q, pg)
        return (p/position_tolerance_m)**2+(a/orientation_tolerance_rad)**2
    initial_cost = cost(selected-offset)
    evidence = dict(scope='conditional_pregrasp_arrival_margin', refined=False,
        original_command_rad=selected.tolist(), original_normalized_cost=initial_cost,
        fixed_pregrasp_goal=list(pregrasp_goal), fixed_approach_goal=list(approach_goal),
        source_sdk_stamp_ns=sample.get('sdk_stamp_ns'),
        source_accepted_stamp_ns=sample.get('accepted_stamp_ns'), epoch_ns=sample.get('epoch_ns'),
        model_sha256=fk.model_sha256, sdk_minus_encoder_rad=(sdk-actual).tolist(),
        future_following_verified=False, hardware_response_verified=False,
        trajectory_checked=False, execution_authorized=False, globally_optimal=False)
    if following_model is not None:
        evidence['following_reference'] = following_evidence(
            sdk, actual, following_model=following_model)
    def finish(command, proof, reason):
        check()
        evidence.update(proof, reason=reason, selected_command_rad=command.tolist(),
            selected_normalized_cost=cost(command-offset), elapsed_sec=time.monotonic()-started)
        return command, evidence
    # This is only a preference. A feasible endpoint never becomes invalid
    # merely because the optimizer cannot improve it or achieve this margin.
    if initial_cost <= .6**2:
        return finish(selected, original, 'already_inside_preferred_margin')
    centre = selected-offset
    low = np.maximum.reduce((centre-.25, fk.limits[:, 0], fk.limits[:, 0]-offset,
                             np.full(6, -math.pi)-offset))
    high = np.minimum.reduce((centre+.25, fk.limits[:, 1], fk.limits[:, 1]-offset,
                              np.full(6, (4095-2048)*Q)-offset))
    if np.any(low >= high) or np.any(centre < low) or np.any(centre > high):
        return finish(selected, original, 'no_refinement_branch')
    def after(x):
        q = x[:6].copy(); q[free] += x[6:]
        return q
    def objective(x):
        check()
        return cost(x[:6])
    def constraints(x):
        check()
        ap, aa = residual(after(x), ag)
        _, pa = residual(x[:6], pg)
        # Soft reserve for quantization. The shared predictors, not these
        # continuous optimizer constraints, decide final qualification.
        return [.9-ap/approach_tolerance, .9-aa/orientation_tolerance_rad,
                .9-pa/orientation_tolerance_rad]
    cap = PregraspCompensation.MAX_TOTAL_COUNTS*Q
    result = minimize(objective, np.r_[centre, np.zeros(len(free))], method='SLSQP',
        bounds=list(zip(low, high))+[(-cap, cap)]*len(free),
        constraints=[dict(type='ineq', fun=constraints)],
        options=dict(maxiter=150, ftol=1e-12))
    check()
    evidence['optimizer'] = dict(success=bool(result.success), iterations=int(result.nit),
        message=str(result.message))
    if (np.asarray(result.x).shape != (6+len(free),)
            or not np.all(np.isfinite(result.x))):
        return finish(selected, original, 'invalid_refinement')
    command = (sdk_counts(result.x[:6]+offset)-2048)*Q
    if (np.any(command < fk.limits[:, 0]) or np.any(command > fk.limits[:, 1])
            or np.any(command-offset < low) or np.any(command-offset > high)
            or cost(command-offset) >= initial_cost-1e-6):
        return finish(selected, original, 'no_quantized_improvement')
    try:
        proof = verify(command)
    except HeldAxesPredictionError as exc:
        if str(exc) not in ('PREGRASP_HELD_AXES_GUIDE_DOES_NOT_REACH_FIXED_GOAL',
                            'PREGRASP_OUTSIDE_LOCAL_CAPTURE_RANGE'):
            raise
        evidence['candidate_rejection'] = dict(code=str(exc), evidence=exc.evidence)
        return finish(selected, original, 'refinement_not_qualified')
    evidence['refined'] = True
    return finish(command, proof, 'smaller_normalized_arrival_error')
