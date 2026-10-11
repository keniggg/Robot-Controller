"""Select a pregrasp endpoint jointly with the unchanged held-axis approach.

This is conditional arithmetic, not a trajectory or an execution permission.
Both callers must obtain their own fresh following sample, plan the complete
command path, audit its geometry and revalidate the frozen task before motion.
The pose targets remain fixed. The nominal pregrasp move may use all six axes;
the subsequent local approach still holds Joint2/Joint5 within its old budget.
"""
from copy import deepcopy
import math
import time

import numpy as np
from scipy.optimize import minimize

from .endpoint_correction import sdk_counts
from .held_axes_prediction import predict_held_axes_endpoint, HeldAxesPredictionError
from .pregrasp_compensation import pose_matrix, PregraspCompensation
from .stationary_following import ARM_NAMES, SerialUrdfFk, SDK_QUANTUM_RAD as Q


def select_held_pregrasp_endpoint(fk, sample, pregrasp_goal_values,
        approach_goal_values, nominal_pregrasp_command_rad, *,
        position_tolerance_m, orientation_tolerance_rad, deadline_check=None):
    """Return a quantized command and evidence, never mutate either goal.

    Keep an already qualified original endpoint. Otherwise try two finite,
    deterministic local starts before a bounded normalized fallback. A solver
    success flag grants nothing: every
    quantized result must pass the actual shared pregrasp/approach predictor.
    The original 90%/80% soft targets leave room for wire quantization. If both
    fail, one normalized 99%/99% try uses more of the same admissible region;
    the original 10 mm / 5 degree and 6 mm / 5 degree gates still decide.
    """
    started = time.monotonic()
    if (not isinstance(fk, SerialUrdfFk) or tuple(fk.names) != ARM_NAMES
            or sample.get('model_sha256') != fk.model_sha256
            or tuple(sample.get('joint_names', ())) != ARM_NAMES):
        raise ValueError('HELD_PREGRASP_MODEL_MISMATCH')
    if (not 0 < position_tolerance_m <= .010
            or not 0 < orientation_tolerance_rad <= math.radians(5)):
        raise ValueError('PREGRASP_TOLERANCE_CANNOT_BE_RELAXED')
    sdk = np.asarray(sample['sdk_positions_rad'], dtype=float)
    actual = np.asarray(sample['accepted_positions_rad'], dtype=float)
    nominal = np.asarray(nominal_pregrasp_command_rad, dtype=float)
    if any(q.shape != (6,) or not np.all(np.isfinite(q)) for q in (sdk, actual, nominal)):
        raise ValueError('HELD_PREGRASP_FOLLOWING_INVALID')
    offset = sdk-actual
    if np.max(np.abs(offset)) > .035:
        raise ValueError('PREGRASP_FOLLOWING_OUTSIDE_CONTRACT')
    nominal = (sdk_counts(nominal)-2048)*Q
    fk(nominal)
    pg, ag = pose_matrix(pregrasp_goal_values), pose_matrix(approach_goal_values)
    def check():
        if deadline_check is not None:
            deadline_check()
    def verify(command):
        check()
        _, pre = predict_held_axes_endpoint(fk, command, command-offset,
            pregrasp_goal_values, position_tolerance_m=position_tolerance_m,
            orientation_tolerance_rad=orientation_tolerance_rad,
            allow_already_converged=True)
        _, approach = predict_held_axes_endpoint(fk,
            pre['predicted_command_positions_rad'], pre['predicted_positions_rad'],
            approach_goal_values, position_tolerance_m=min(position_tolerance_m, .006),
            orientation_tolerance_rad=orientation_tolerance_rad,
            allow_already_converged=True)
        check()
        return dict(predicted_pregrasp=pre, predicted_approach=approach)
    evidence = dict(scope='conditional_held_pregrasp_endpoint_selection',
        model_sha256=fk.model_sha256, fixed_pregrasp_goal=list(pregrasp_goal_values),
        fixed_approach_goal=list(approach_goal_values),
        source_sdk_stamp_ns=sample.get('sdk_stamp_ns'),
        source_accepted_stamp_ns=sample.get('accepted_stamp_ns'),
        epoch_ns=sample.get('epoch_ns'), sdk_minus_encoder_rad=offset.tolist(),
        original_nominal_command_rad=nominal.tolist(), future_following_verified=False,
        hardware_response_verified=False, execution_authorized=False,
        trajectory_checked=False, globally_optimal=False)
    try:
        verified = verify(nominal)
    except HeldAxesPredictionError as exc:
        original_error = exc
        # The final failure attaches this selection report to the exception.
        # Freeze its original evidence now to avoid a back-reference cycle.
        evidence['original_failure'] = dict(code=str(exc), evidence=deepcopy(exc.evidence))
        # Only the known finite-guide rejection may select a new nominal
        # waypoint. Missing provenance, large bias and capture failures retain
        # their original meaning and cannot turn into an optimization retry.
        if str(exc) != 'PREGRASP_HELD_AXES_GUIDE_DOES_NOT_REACH_FIXED_GOAL':
            raise
    else:
        evidence.update(verified, reselected=False, success=True,
            selected_command_rad=nominal.tolist(), elapsed_sec=time.monotonic()-started)
        return nominal, evidence

    free = np.array([axis for axis in range(6) if axis not in PregraspCompensation.HELD_AXES])
    cap = PregraspCompensation.MAX_TOTAL_COUNTS
    def residual(q, target):
        # Finite optimizers may probe an infeasible approach joint while
        # differentiating. Treat that trial as infeasible, without clipping a
        # joint or terminating the whole candidate search. Final verification
        # still calls the unchanged FK with the actual quantized command.
        if np.any(q < fk.limits[:, 0]) or np.any(q > fk.limits[:, 1]):
            return 1.e3, math.pi
        transform = fk(q)
        return (float(np.linalg.norm(transform[:3, 3]-target[:3, 3])),
            math.acos(float(np.clip((np.trace(transform[:3, :3].T@target[:3, :3])-1)/2, -1., 1.))))
    def after(x):
        q = x[:6].copy()
        q[free] += x[6:]*Q
        return q
    def objective(x):
        check()
        return residual(after(x), ag)[0]**2
    position_margin, angle_margin = .9, .8
    def constraints(x):
        check()
        pe, pa = residual(x[:6], pg)
        _, aa = residual(after(x), ag)
        return np.array([position_margin*position_tolerance_m-pe,
            angle_margin*orientation_tolerance_rad-pa,
            angle_margin*orientation_tolerance_rad-residual(x[:6], ag)[1],
            angle_margin*orientation_tolerance_rad-aa])
    # This is a local branch search, not a larger local correction episode.
    # The future command must still be inside both URDF and SDK limits after
    # adding the currently observed bias. A path checker will audit the move.
    low = np.maximum.reduce((nominal-.25, fk.limits[:, 0], fk.limits[:, 0]-offset,
                             np.full(6, -math.pi)-offset))
    high = np.minimum.reduce((nominal+.25, fk.limits[:, 1], fk.limits[:, 1]-offset,
                              np.full(6, (4095-2048)*Q)-offset))
    if np.any(low >= high):
        raise original_error
    seed_q = np.minimum(high, np.maximum(low, nominal))
    guide = np.asarray(original_error.evidence.get('proposed_delta_counts', [0]*6))[free]
    starts = (np.r_[seed_q, guide], np.r_[seed_q, np.zeros(len(free))])
    trials, accepted = [], []
    for normalized in (False, True):
        if accepted:
            break
        position_margin, angle_margin = ((.99, .99) if normalized else (.9, .8))
        for seed in (starts[:1] if normalized else starts):
            check()
            bounds = list(zip(low, high))+[(-cap, cap)]*len(free)
            if normalized:
                # The first two tries and their successful commands remain
                # unchanged. Only after both fail, use consistent numerical
                # units and the rest of the original hard acceptance region.
                # Quantization and the shared predictor still certify every
                # result; these optimizer targets grant no extra tolerance.
                scale = np.r_[np.ones(6), np.full(len(free), Q)]
                constraint_scale = np.array([position_tolerance_m]+[orientation_tolerance_rad]*3)
                result = minimize(lambda y: objective(y/scale)/min(position_tolerance_m, .006)**2,
                    seed*scale, method='SLSQP',
                    bounds=[(lo*unit, hi*unit) for (lo, hi), unit in zip(bounds, scale)],
                    constraints=[dict(type='ineq', fun=lambda y: constraints(y/scale)/constraint_scale)],
                    options=dict(maxiter=150, ftol=1e-13))
                result.x = result.x/scale
            else:
                result = minimize(objective, seed, method='SLSQP', bounds=bounds,
                    constraints=[dict(type='ineq', fun=constraints)],
                    options=dict(maxiter=150, ftol=1e-13))
            trial = dict(solver_success=bool(result.success), iterations=int(result.nit),
                solver_message=str(result.message), solver_endpoint=result.x.tolist(),
                normalized_fallback=normalized, optimizer_position_margin=position_margin,
                optimizer_orientation_margin=angle_margin)
            trials.append(trial)
            if not np.all(np.isfinite(result.x)):
                trial['rejection'] = 'NONFINITE_SOLVER_RESULT'
                continue
            command = (sdk_counts(result.x[:6]+offset)-2048)*Q
            trial['quantized_command_rad'] = command.tolist()
            if np.any(command < fk.limits[:, 0]) or np.any(command > fk.limits[:, 1]):
                trial['rejection'] = 'QUANTIZED_COMMAND_OUTSIDE_URDF'
                continue
            try:
                verified = verify(command)
            except HeldAxesPredictionError as exc:
                trial['rejection'] = str(exc)
                trial['rejection_evidence'] = deepcopy(exc.evidence)
                continue
            trial['qualified'] = True
            a = verified['predicted_approach']
            margin_cost = max(a['predicted_position_error_m']/min(position_tolerance_m, .006),
                              a['predicted_orientation_error_rad']/orientation_tolerance_rad)
            accepted.append((margin_cost, command, verified))
            if normalized:
                break  # First qualified fallback; do not spend another solve.
    evidence.update(trials=trials, elapsed_sec=time.monotonic()-started)
    if not accepted:
        original_error.evidence['pregrasp_endpoint_selection'] = evidence
        raise original_error
    _, command, verified = min(accepted, key=lambda item: item[0])
    evidence.update(verified, reselected=True, success=True, selected_command_rad=command.tolist())
    return command, evidence
