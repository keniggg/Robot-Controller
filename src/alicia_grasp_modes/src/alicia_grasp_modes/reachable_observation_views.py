"""Bounded joint-space proposals for measured missing-side observations.

No ROS I/O or motion authority. A proposal preserves joint limits and passes
the existing endpoint gates; the caller must still run the original complete
scene, trajectory, source, visibility, following-error and stop checks.
SciPy is loaded only when this auxiliary proposal path is explicitly called.
"""
from alicia_flexible_grasp.robot.observation_view_limits import MINIMUM_OBSERVATION_IMAGE_MARGIN_PX
from copy import deepcopy
from collections.abc import Mapping
from dataclasses import asdict
import math
import time

import numpy as np

from alicia_flexible_grasp.grasp.gripper_geometry import (
    GripperGeometry, _box_corners, _stage_boxes,
    evaluate_open_gripper_observation_envelope)
from alicia_flexible_grasp.grasp.observation_working_volume import (
    validate_planned_observation_working_volume)
from alicia_flexible_grasp.robot.observation_path_guard import (
    SerialUrdfFk, observation_endpoint_support_bound, observation_endpoint_joint_reserve,
    observation_following_support_bound)
from alicia_flexible_grasp.robot.stationary_following import SDK_QUANTUM_RAD
from alicia_flexible_grasp.robot.observation_arrival_reserve import arrival_reserve_from_source
from alicia_flexible_grasp.robot.observation_view_guard import (
    validate_observation_support_projection)
from .observe_return_visual import (missing_side_view_gain, missing_side_observation_span,
                                   missing_side_ray_projection)
from .observation_information_uncertainty import (PREDICTED_GAIN_POLICY, _positive_target,
                                                observation_resolvable_gain_target_m)
from .observation_arrival_projection import validate_arrival_projection
from .observation_endpoint_precompute import PreparedEndpointSupportBound


FK_PROPOSAL_BUDGET_SEC = 5.  # User-selected shared request budget, 2026-11-02.


class _BudgetExpired(Exception):
    pass


class _DirectionBudgetExpired(Exception):
    """Yield one numerical direction without extending the shared deadline."""
    pass


class _EndpointQualified(Exception):
    """Stop optimization only after the original complete endpoint audit."""
    def __init__(self, endpoint):
        self.endpoint = endpoint


def _copy_evidence(value):
    """Copy immutable request evidence without changing leaf values or types."""
    if isinstance(value, Mapping):
        return {key: _copy_evidence(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return tuple(_copy_evidence(item) for item in value)
    if isinstance(value, list):
        return [_copy_evidence(item) for item in value]
    return deepcopy(value)


def _array(value, shape, label):
    result = np.array(value, dtype=float, copy=True)
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError(label + ' must have finite shape ' + str(shape))
    return result


def _rigid(value, label):
    value = _array(value, (4, 4), label)
    if (not np.allclose(value[3], [0., 0., 0., 1.], atol=1e-9, rtol=0.)
            or not np.allclose(value[:3, :3].T @ value[:3, :3], np.eye(3), atol=1e-8, rtol=0.)
            or abs(np.linalg.det(value[:3, :3])-1.) > 1e-8):
        raise ValueError(label + ' must be rigid')
    return value


def _backend():
    from scipy.optimize import minimize
    return minimize


def _arrival_center_preference(prism, camera_pose, intrinsics, image_size):
    """Angular-scale soft preference; never a visibility/uncertainty proof.

    Use the same immutable measured prism as arrival, not just the smaller
    current OBB. Its bounding-box centre is steered toward the image centre.
    Both terms in the solver objective are dimensionless squared angular
    displacements: joint motion and pinhole-normalized image displacement.
    No new clearance threshold or contact/arrival authority is introduced.
    """
    points = (prism-camera_pose[:3, 3]) @ camera_pose[:3, :3]
    # Intermediate solver proposals may be outside the camera half-space;
    # their original depth constraints still reject them independently.
    depth = np.maximum(points[:, 0], 1e-6)
    uv = intrinsics[2:] - intrinsics[:2]*points[:, 1:]/depth[:, None]
    centre = .5*(uv.min(axis=0)+uv.max(axis=0))
    desired = .5*(np.asarray(image_size, dtype=float)-1.)
    residual = (centre-desired)/intrinsics[:2]
    return dict(cost=float(residual @ residual), centre_px=centre.tolist(),
        desired_centre_px=desired.tolist(), normalized_residual=residual.tolist(),
        proposal_preference_only=True, certifies_arrival_or_following=False)


def _optical_side_evidence(camera, support, rotation, size, uncertainty):
    """Mirror the caller's original optical-axis side gate, not the target ray."""
    support = _array(support, (3,), 'support normal')
    length = float(np.linalg.norm(support))
    if not math.isfinite(length) or length <= 1e-12:
        raise ValueError('support normal is invalid for observation evidence')
    support /= length
    object_height = float(np.sum(np.abs(rotation.T.dot(support)) * size))
    optical = np.array(camera[:3, 0], dtype=float, copy=True)
    length = float(np.linalg.norm(optical))
    if not math.isfinite(length) or length <= 1e-12:
        raise ValueError('camera optical axis is invalid for observation evidence')
    optical /= length
    incidence = math.acos(max(-1., min(1., float(np.dot(optical, -support)))))
    projected = float(object_height * math.sin(incidence))
    if not math.isfinite(object_height) or object_height <= 0. or not math.isfinite(projected):
        raise ValueError('object height or projected side is invalid')
    deficit = max(0., uncertainty-projected)
    return dict(ok=deficit == 0., observation_incidence_angle_deg=math.degrees(incidence),
        observation_object_height_m=object_height,
        observation_projected_side_evidence_m=projected,
        observation_side_uncertainty_m=uncertainty,
        observation_side_evidence_deficit_m=deficit,
        observation_side_signal_to_uncertainty=None if uncertainty <= 0. else projected/uncertainty,
        observation_side_decision_rule='object_height_m * sin(incidence_angle) >= live_depth_uncertainty_m')


def generate_reachable_observation_views(*, source, current_joint_positions, fk,
        gripper, opening_width_m, gaps, depth_uncertainty_m,
        joint_error_bounds_rad, side_depth_uncertainty_m=None, minimum_view_baseline_m=.015,
        max_directions=2, max_iterations=96, max_seconds=FK_PROPOSAL_BUDGET_SEC,
        proposal_image_reserve_px=4., clock=time.monotonic, endpoint_error_bounds_rad=None,
        predicted_gain_target_m=None, max_search_directions=6):
    """Return bounded exposure tiers using request-local directional searches.

    ``source`` contains binding.snapshot_stamp_ns, corners_base, center_base,
    points_base (original measured support only), tool_from_camera, camera,
    view_limits.effective_visibility_limits, and geometry with support normal,
    offset and target OBB. ``joint_error_bounds_rad`` must be supplied from an
    existing permitted following/stop contract; this function never invents or
    relaxes a contract. The exact caller binding is copied into every proposal.
    ``side_depth_uncertainty_m`` binds the caller's current-source optical-axis
    gate separately from the retained missing-side evidence's uncertainty.

    max_seconds is a cooperative start/completion budget, not preemption of a
    native solver call. Late or unvalidated results are discarded. Previously
    admitted proposals are returned on exhaustion; no failure prunes the
    caller's existing pose families. A feasible solver probe may be admitted
    before objective convergence only after every original endpoint gate passes
    within that same budget. Failed early audits leave the same solver running;
    at most four distinct probes per direction/tier receive this extra audit.
    The final solver result always receives its original complete audit.
    ``max_directions`` bounds the number of admitted directions; at most
    twice that many proposals are returned. ``max_search_directions`` bounds
    angular-ranked search coverage independently, and is not a completeness
    claim. A first pass gives every covered direction at most eight iterations
    and a fair share of the original deadline before refining admitted views.
    A direction receives at most ``max_iterations`` solver iterations
    across its baseline, refinement and any later continuation combined.
    That refinement maximizes directed exposure up to its requested target,
    retaining the baseline exposure and every physical constraint. An infeasible
    higher target must not discard a useful, fully checked intermediate view.
    Early admission of a refinement requires the requested exposure; a final
    solver endpoint can provide a smaller gain after the original complete audit.
    A stronger tier starts from that same direction's qualified endpoint when
    available. This is only a numerical seed: the measured start still defines
    motion cost, baseline and reported joint displacement. Seeds remain local
    to this frozen request and never replace any endpoint or execution check.
    Solver bounds retain the existing joint-error box inside the original
    limits, with a one-count numerical reserve. Only a numerical seed may be
    projected into that range; measured motion origins remain unchanged.
    Unqualified directions receive one deterministic second seed a quarter of
    the way from the measurement toward the reserved URDF-range centre. This
    convex numerical seed is independent of any object, candidate or saved pose.
    A finite, in-limit major iterate or solver endpoint from a rejected or interrupted
    attempt can seed only that same direction's later short baseline continuation in
    this request; it never becomes a proposal without every original endpoint
    gate. These are numerical restarts from the last major iterate, not a
    continuation of SLSQP's Hessian or multiplier state. Measured ``q0`` remains
    the motion origin throughout. Only predicted missing-side exposure can use an explicit
    source contact-band target; optical/physical gates retain their original
    uncertainties. The target is not a measurement error bound. Neither
    exposure tier establishes measured common contact height. The larger of
    the task target and visual uncertainty bounds predicted gain. Sorting is
    a proposal heuristic only.
    """
    started = clock()
    report = dict(policy='bounded_measured_joint_space_missing_side_proposals',
        execution_authority=False, original_candidates_removed=0, directions=[],
        candidate_count=0, status='invalid_input', proves_complete_path=False,
        max_seconds=max_seconds, max_iterations=max_iterations,
        maximum_shared_budget_sec=FK_PROPOSAL_BUDGET_SEC,
        proposal_image_reserve_px=proposal_image_reserve_px,
        backend='scipy_slsqp', seed_policy='measured_start_then_joint_range_centre_then_request_local_same_direction_continuation_or_qualified_refinement',
        joint_range_centre_seed_fraction=.25,
        **({'exposure_target_noise_multipliers': [1., 2.]} if predicted_gain_target_m is None else
           dict(exposure_target_multipliers=[1., 2.], predicted_gain_target_m=predicted_gain_target_m,
                endpoint_acceptance_predicted_gain_target_m=predicted_gain_target_m,
                predicted_gain_policy=PREDICTED_GAIN_POLICY)),
        exposure_tier_order='bounded_direction_coverage_then_qualified_refinement',
        max_qualified_directions=max_directions, max_search_directions=max_search_directions,
        exposure_prediction_only=True, proves_common_contact_height=False,
        endpoint_acceptance_uncertainty_m=depth_uncertainty_m)
    candidates = []
    qualified_seeds = {}
    numerical_seeds = {}
    early_seen_by_tier = {}
    direction_deadline = None

    def remaining():
        now = clock()
        if now-started >= max_seconds:
            raise _BudgetExpired()
        if direction_deadline is not None and now >= direction_deadline:
            raise _DirectionBudgetExpired()

    try:
        if (not isinstance(fk, SerialUrdfFk) or not isinstance(gripper, GripperGeometry)
                or type(max_directions) is not int or not 1 <= max_directions <= 6
                or type(max_search_directions) is not int or not 1 <= max_search_directions <= 6
                or type(max_iterations) is not int or not 1 <= max_iterations <= 160
                or isinstance(max_seconds, bool) or not math.isfinite(max_seconds)
                or not 0. < max_seconds <= FK_PROPOSAL_BUDGET_SEC):
            raise ValueError('invalid FK/CAD or bounded solver configuration')
        if (isinstance(proposal_image_reserve_px, bool)
                or not math.isfinite(float(proposal_image_reserve_px))
                or not 0. <= proposal_image_reserve_px <= 32.):
            raise ValueError('finite bounded proposal image reserve required')
        q0 = _array(current_joint_positions, (len(fk.names),), 'measured joints')
        errors = _array(joint_error_bounds_rad, q0.shape, 'bound contract errors')
        if np.any(errors <= 0.) or np.any(errors > .5):
            raise ValueError('positive existing contract error bounds required')
        if (not math.isfinite(float(depth_uncertainty_m)) or not 0. <= depth_uncertainty_m <= .05
                or not math.isfinite(float(minimum_view_baseline_m)) or minimum_view_baseline_m < .015
                or not math.isfinite(float(opening_width_m))
                or not 0. <= opening_width_m <= gripper.max_inner_gap_m):
            raise ValueError('invalid uncertainty, opening, or original baseline')
        gain_target = observation_resolvable_gain_target_m(depth_uncertainty_m, predicted_gain_target_m)
        report['effective_gain_target_m'] = gain_target
        side_uncertainty = depth_uncertainty_m if side_depth_uncertainty_m is None else side_depth_uncertainty_m
        if (isinstance(side_uncertainty, bool) or not math.isfinite(float(side_uncertainty))
                or not 0. <= side_uncertainty <= .05):
            raise ValueError('finite current-source optical side uncertainty required')
        initial = fk(q0)  # Enforces the actual URDF limits, including measured seed.
        # Reserve the existing following/stop box, plus one wire count as a
        # numerical proposal preference. The final strict IK branch separately
        # checks the original contract reserve; neither step proves correction.
        proposal_limits = fk.limits + np.column_stack((
            errors+SDK_QUANTUM_RAD, -errors-SDK_QUANTUM_RAD))
        if np.any(proposal_limits[:, 0] >= proposal_limits[:, 1]):
            raise ValueError('existing contract leaves no interior joint range')
        report['proposal_joint_limits_rad'] = proposal_limits.tolist()
        report['proposal_joint_numerical_reserve_rad'] = SDK_QUANTUM_RAD
        mount = _rigid(source['tool_from_camera'], 'camera mount')
        camera = dict(source['camera'])
        k = _array([camera[n] for n in ('fx', 'fy', 'cx', 'cy')], (4,), 'intrinsics')
        fx, fy, cx, cy = k
        width, height = camera['width'], camera['height']
        limits = dict(source['view_limits']['effective_visibility_limits'])
        margin, near, far = (float(limits[n]) for n in
                            ('roi_margin_px', 'minimum_depth_m', 'maximum_depth_m'))
        if (fx <= 0. or fy <= 0. or type(width) is not int or type(height) is not int
                or width < 1 or height < 1 or margin < MINIMUM_OBSERVATION_IMAGE_MARGIN_PX or near < .07 or far > .5
                or not 0. < near < far or not all(math.isfinite(v) for v in (margin, near, far))):
            raise ValueError('camera limits must retain original visibility bounds')
        corners = _array(source['corners_base'], (8, 3), 'target corners')
        arrival_projection = source.get('arrival_projection')
        prism = None if arrival_projection is None else np.asarray(arrival_projection['prism_base'], dtype=float)
        if prism is not None and (prism.ndim != 2 or prism.shape[1:] != (3,)
                or not 6 <= len(prism) <= 2048 or not np.all(np.isfinite(prism))):
            raise ValueError('invalid frozen measured arrival prism')
        report['objective'] = ('joint_motion_plus_frozen_prism_image_centre'
            if prism is not None else 'joint_motion_only')
        center = _array(source['center_base'], (3,), 'target center')
        points = np.array(source['points_base'], dtype=float, copy=True)
        if points.ndim != 2 or points.shape[1] != 3 or not np.all(np.isfinite(points)):
            raise ValueError('original finite support points required')
        geometry = source['geometry']
        normal = _array(geometry['support_normal_base'], (3,), 'support normal')
        offset = float(geometry['support_offset_m'])
        rotation = _array(geometry['R_base_obb'], (3, 3), 'target axes')
        size = _array(geometry['obb_size_xyz_m'], (3,), 'target size')
        if abs(np.linalg.norm(normal)-1.) > 1e-6 or not math.isfinite(offset):
            raise ValueError('invalid measured support plane')
        binding = _copy_evidence(source['binding']); stamp = binding['snapshot_stamp_ns']
        if type(stamp) is not int or stamp <= 0:
            raise ValueError('source snapshot binding required')
        gaps = _copy_evidence(tuple(gaps))
        current_camera_origin = (initial @ mount)[:3, 3]
        ordered = []
        for gap_index, gap in enumerate(gaps):
            gc = _array(gap['target_center_base_m'], (3,), 'gap center')
            jaw = _array(gap['jaw_axis_base'], (3,), 'gap axis')
            origin = _array(gap['initial_camera_origin_base_m'], (3,), 'initial gap camera')-gc
            object_height, half_width = float(gap['object_height_m']), .5*float(gap['whole_obb_width_m'])
            if (abs(np.linalg.norm(jaw)-1.) > 1e-6 or np.linalg.norm(origin) <= 1e-9
                    or object_height <= 0. or half_width <= 0.
                    or not math.isfinite(object_height+half_width)):
                raise ValueError('invalid measured gap geometry')
            if (gap['target_epoch'] != binding['target_epoch']
                    or gap['target_track_id'] != binding['target_track_id']):
                raise ValueError('gap does not belong to frozen target')
            view_stamps = gap['source_view_stamps_ns']
            if (not isinstance(view_stamps, (list, tuple)) or not view_stamps
                    or any(type(value) is not int or not 0 < value <= stamp for value in view_stamps)):
                raise ValueError('gap source views must precede the frozen snapshot')
            for sign in gap['missing_side_signs']:
                if type(sign) is not int or sign not in (-1, 1):
                    raise ValueError('invalid missing side sign')
                observation_span, _ = missing_side_observation_span(gap, sign)
                previous = observation_span*max(0., missing_side_ray_projection(origin,jaw,half_width,sign))
                required = (previous+gain_target+.0002)/observation_span
                if required >= 1.:
                    continue
                current_ray = current_camera_origin-gc-sign*half_width*jaw
                steering = max(0., math.acos(float(np.clip(sign*float(jaw@current_ray)/max(1e-12,np.linalg.norm(current_ray)), -1., 1.)))
                               - math.acos(required))
                ordered.append((steering, gap_index, sign, gc, jaw, half_width, required))
        ordered.sort(key=lambda row: (row[0], row[1], row[2]))
        unique = []
        for row in ordered:
            direction = -row[2]*row[4]
            if not any(float(direction @ (-prior[2]*prior[4])) > 1.-1e-10 for prior in unique):
                unique.append(row)
        try:
            remaining(); minimize = _backend(); remaining()
        except (ImportError, AttributeError, RuntimeError) as exc:
            report.update(status='backend_unavailable', reason=str(exc))
            return (), report
        # Immutable proposal terms belong to this request and consume its
        # original wall-clock budget. Complete endpoint admission below still
        # recomputes the original independent support bound without this cache.
        remaining()
        prepared_support_bound = PreparedEndpointSupportBound(
            fk, errors, normal, offset, gripper, opening_width_m)
        arrival_bound = (None if endpoint_error_bounds_rad is None else
            arrival_reserve_from_source(fk,endpoint_error_bounds_rad,source))
        report['arrival_error_box_required'] = arrival_bound is not None
        remaining()
        tiers = (1., 2.) if gain_target > 0. else (1.,)
        directions = unique[:max_search_directions]
        iteration_budget = {(row[1], row[2]): max_iterations for row in directions}
        first_pass_iterations = min(8, max_iterations)
        maximum_baseline_rounds = int(math.ceil(max_iterations/first_pass_iterations))
        report.update(status='complete', available_directions=len(unique),
                      **({'exposure_target_noise_multipliers': list(tiers)} if predicted_gain_target_m is None
                         else dict(exposure_target_multipliers=list(tiers))),
                      selected_direction_count=len(directions),
                      coverage_limit_includes_all_directions=len(directions) == len(unique),
                      maximum_tier_attempt_count=maximum_baseline_rounds*len(directions)+(min(max_directions, len(directions)) if len(tiers) > 1 else 0),
                      maximum_candidate_count=len(tiers)*max_directions,
                      first_pass_iteration_limit=first_pass_iterations,
                      model_sha256=fk.model_sha256,
                      binding=binding, joint_error_bounds_rad=errors.tolist(),
                      optical_side_uncertainty_m=side_uncertainty,
                      direction_ranking_policy='finite_side_target_ray_required_angular_change',
                      optical_side_proposal_reserve_m=.0001)
        def attempts():
            # Angular rank alone cannot prove joint reachability. Do not let
            # the first difficult directions consume the entire budget or
            # exclude a later direction before any solver sees it.
            for index, direction in enumerate(directions):
                if len(qualified_seeds) >= max_directions:
                    break
                yield (1., direction, first_pass_iterations, 'coverage',
                       len(directions)-index + (max_directions if len(tiers) > 1 else 0))
            if len(tiers) > 1:
                admitted = [d for d in directions if (d[1], d[2]) in qualified_seeds]
                for index, direction in enumerate(admitted):
                    key = (direction[1], direction[2])
                    if iteration_budget[key] > 0:
                        yield 2., direction, iteration_budget[key], 'qualified_refinement', len(admitted)-index
            # A failed numerical endpoint carries no authority. It may only
            # continue its own frozen-direction baseline after fair coverage
            # and refinements of already admitted views have had their turn.
            for _round in range(1, maximum_baseline_rounds):
                pending = [d for d in directions if (d[1], d[2]) not in qualified_seeds
                           and iteration_budget[(d[1], d[2])] > 0]
                if not pending:
                    break
                for index, direction in enumerate(pending):
                    if len(qualified_seeds) >= max_directions:
                        return
                    key = (direction[1], direction[2])
                    # Short continuations must also reach the original final
                    # endpoint audit. One long continuation can otherwise hit
                    # its fair time slice before auditing a useful iterate.
                    yield (1., direction, min(first_pass_iterations, iteration_budget[key]),
                           'joint_range_centre_restart' if _round == 1 else 'same_direction_continuation',
                           len(pending)-index)
                    if len(tiers) > 1 and key in qualified_seeds and iteration_budget[key] > 0:
                        yield 2., direction, iteration_budget[key], 'qualified_refinement', max(1, len(pending)-index)

        for tier, direction, iteration_limit, phase, budget_shares in attempts():
            _steering, gap_index, sign, gc, jaw, half_width, baseline_required = direction
            remaining(); direction_started = clock()
            observation_span, span_audit = missing_side_observation_span(gaps[gap_index], sign)
            requested = baseline_required + (tier-1.)*gain_target/observation_span
            required = baseline_required
            initial_ray = np.asarray(gaps[gap_index]['initial_camera_origin_base_m'])-gc
            initial_center_projection = max(0.,sign*float(jaw@initial_ray)/np.linalg.norm(initial_ray))
            required_center_projection = initial_center_projection+(gain_target+.0002)/observation_span
            row_audit = dict(gap_index=gap_index, side_sign=sign, accepted=False, seed_count=1,
                search_phase=phase, allocated_iterations=iteration_limit,
                objective=('bounded_missing_side_exposure' if tier > 1. else report['objective']),
                observation_span=span_audit,
                **({'exposure_target_noise_multiplier': tier} if predicted_gain_target_m is None
                   else dict(exposure_target_multiplier=tier, predicted_gain_target_m=predicted_gain_target_m,
                             endpoint_acceptance_predicted_gain_target_m=gain_target,
                             predicted_gain_policy=PREDICTED_GAIN_POLICY)),
                requested_new_side_gain_m=tier*gain_target+.0002,
                endpoint_acceptance_uncertainty_m=depth_uncertainty_m,
                exposure_prediction_only=True, proves_common_contact_height=False)
            report['directions'].append(row_audit)
            if requested >= 1.:
                row_audit.update(reason='requested exposure exceeds object-height projection', elapsed_sec=0.)
                continue
            seed_key = (gap_index, sign)
            continued_seed = tier > 1. and seed_key in qualified_seeds
            unqualified_seed = phase == 'same_direction_continuation' and seed_key in numerical_seeds
            centre_restart = phase == 'joint_range_centre_restart'
            numerical_seed = (qualified_seeds[seed_key] if continued_seed else
                q0+.25*(np.mean(proposal_limits, axis=1)-q0) if centre_restart else
                numerical_seeds[seed_key] if unqualified_seed else q0).copy()
            seed_before_projection = numerical_seed.copy()
            numerical_seed = np.clip(numerical_seed, proposal_limits[:, 0], proposal_limits[:, 1])
            seed_projected = not np.array_equal(numerical_seed, seed_before_projection)
            row_audit.update(numerical_seed_kind=('same_direction_qualified_endpoint'
                if continued_seed else 'joint_range_centre_interpolation' if centre_restart
                else 'same_direction_unqualified_numerical_endpoint' if unqualified_seed
                else ('joint_reserve_projected_measured_start'
                    if seed_projected else 'current_measured_start')),
                numerical_seed_projected_to_joint_reserve=seed_projected,
                numerical_seed_positions_rad=numerical_seed.tolist(),
                measured_start_unchanged=True)
            time_share = max(0., (max_seconds-(clock()-started))/max(1, budget_shares))
            direction_deadline = min(started+max_seconds, clock()+time_share)
            row_audit['allocated_seconds'] = time_share
            completed_iterations = [0]
            reported_iterations = [None]
            # A request-local last-evaluation cache avoids redundant solver probes.
            last = [None, None]
            early_seen = early_seen_by_tier.setdefault((gap_index, sign, tier), set())
            row_audit['early_endpoint_checks'] = []

            def retain_numerical_seed(q):
                value = np.asarray(q, dtype=float)
                if (value.shape == q0.shape and np.all(np.isfinite(value))
                        and np.all(value >= fk.limits[:, 0]) and np.all(value <= fk.limits[:, 1])):
                    numerical_seeds[seed_key] = value.copy()

            def iteration_completed(q):
                completed_iterations[0] += 1
                # SciPy 1.3 emits a final callback with nit=maxiter+1 before
                # returning iteration-limit status. It must reach the original
                # final endpoint audit; it is not another solver allocation.
                if completed_iterations[0] > iteration_limit+1:
                    raise _DirectionBudgetExpired('direction solver iteration quota exhausted')
                retain_numerical_seed(q)
                remaining()

            def complete_endpoint(q):
                # This is shared by early proposals and the original final
                # solver result. Neither numerical feasibility nor success
                # substitutes for measured support, arrival or CAD checks.
                remaining()
                q = _array(q, q0.shape, 'solver joints')
                if np.any(q < fk.limits[:, 0]-1e-10) or np.any(q > fk.limits[:, 1]+1e-10):
                    raise ValueError('solver returned joints outside original limits')
                q = np.clip(q, fk.limits[:, 0], fk.limits[:, 1])
                joint_reserve = observation_endpoint_joint_reserve(fk, q, errors)
                tool = fk(q); cam = tool @ mount
                gain = missing_side_view_gain(gaps, cam[:3, 3], depth_uncertainty_m,
                    **({} if predicted_gain_target_m is None else dict(predicted_gain_target_m=predicted_gain_target_m)))
                view = validate_planned_observation_working_volume(corners, cam, k, width, height,
                    snapshot_stamp_ns=stamp, **limits)
                envelope = evaluate_open_gripper_observation_envelope(gripper=gripper, T_base_tool0=tool,
                    opening_width_m=opening_width_m, support_normal_base=normal, support_offset_m=offset,
                    obb_center_base=center, R_base_obb=rotation, obb_size_xyz_m=size)
                robust = observation_endpoint_support_bound(fk, q, errors, normal, offset, gripper, opening_width_m)
                if not robust['ok']:
                    # An insufficient coarse Taylor lower bound is not a
                    # collision witness. The existing independent box proof
                    # may certify the same complete error box and clearance.
                    # Its work shares this direction and request's deadlines.
                    remaining()
                    support_deadline = started + max_seconds
                    if direction_deadline is not None:
                        support_deadline = min(support_deadline, direction_deadline)
                    support_seconds = support_deadline - clock()
                    if support_seconds <= 0.:
                        remaining()
                        raise _DirectionBudgetExpired('no remaining support-proof budget')
                    coarse = robust
                    method = 'existing_full_joint_error_box_after_inconclusive_coarse_bound'
                    refinement = dict(coarse_endpoint_bound=deepcopy(coarse),
                        support_bound_method=method, allocated_seconds=support_seconds,
                        proof_returned=False, checked_within_remaining_budget=False)
                    row_audit.setdefault('support_bound_refinements', []).append(refinement)
                    proof = observation_following_support_bound(
                        fk, q, errors, normal, offset, gripper, opening_width_m,
                        max_seconds=support_seconds, clock=clock)
                    refinement.update(proof_returned=True, proof=deepcopy(proof))
                    remaining()
                    refinement['checked_within_remaining_budget'] = True
                    robust = dict(proof, coarse_endpoint_bound=coarse,
                        support_bound_method=method, support_proof_budget_sec=support_seconds)
                projection = validate_observation_support_projection(cam, points, center, k, width, height,
                    target_corners_base=corners, **limits)
                side = _optical_side_evidence(cam, normal, rotation, size, side_uncertainty)
                arrival_view = None if arrival_projection is None else validate_arrival_projection(
                    arrival_projection, cam, k, width, height, **limits)
                arrival_reserve = (None if arrival_bound is None else arrival_bound.audit(q,
                    rgb_valid=None if arrival_projection is None else arrival_projection['rgb_valid']))
                baseline = float(np.linalg.norm(tool[:3, 3]-initial[:3, 3]))
                checks = dict(missing_side_view_gain=gain, working_volume=view,
                    observation_envelope=asdict(envelope), conditional_endpoint_support=robust,
                    measured_support_projection=projection, tool_translation_delta_m=baseline,
                    optical_side_evidence=side, within_original_joint_limits=True,
                    endpoint_joint_reserve=joint_reserve)
                if arrival_view is not None:
                    checks['arrival_projection_endpoint'] = arrival_view
                    checks['arrival_centre_preference'] = _arrival_center_preference(
                        prism, cam, k, (width, height))
                if arrival_reserve is not None:
                    checks['arrival_error_box_visibility'] = arrival_reserve
                accepted = (joint_reserve['ok'] and gain['ok'] and view['ok'] and envelope.ok and robust['ok']
                    and projection['ok'] and side['ok'] and baseline >= minimum_view_baseline_m
                    and (arrival_view is None or arrival_view['ok'])
                    and (arrival_reserve is None or arrival_reserve['ok']))
                remaining()
                return q, tool, cam, checks, bool(accepted)

            def inequalities(q):
                remaining()
                q = np.clip(q, fk.limits[:, 0], fk.limits[:, 1])
                if last[0] is not None and np.array_equal(q, last[0]):
                    return last[1]
                tool = fk(q); cam = tool @ mount
                x, y, z = ((corners-cam[:3, 3]) @ cam[:3, :3]).T
                delta = cam[:3, 3]-gc
                physical_corners = np.concatenate([_box_corners(c, tool[:3, :3], s)
                    for _, c, s in _stage_boxes(tool, gripper, opening_width_m, 'y', 'z')])
                robust = prepared_support_bound(q)
                side = _optical_side_evidence(cam, normal, rotation, size, side_uncertainty)
                values = np.r_[x-near-.0001, far-.0001-x,
                    ((cx-margin-proposal_image_reserve_px)*x-fx*y)/fx, ((width-1.-margin-proposal_image_reserve_px-cx)*x+fx*y)/fx,
                    ((cy-margin-proposal_image_reserve_px)*x-fy*z)/fy, ((height-1.-margin-proposal_image_reserve_px-cy)*x+fy*z)/fy,
                    .1*(missing_side_ray_projection(delta,jaw,half_width,sign)-required),
                    .1*(sign*float(jaw @ delta)/max(1e-12,np.linalg.norm(delta))-required_center_projection),
                    sign*float(jaw @ delta)-half_width-.0001,
                    physical_corners @ normal+offset-gripper.support_clearance_m-.0001,
                    np.linalg.norm(tool[:3, 3]-initial[:3, 3])-minimum_view_baseline_m-.0001,
                    robust['minimum_support_clearance_lower_bound_m']-gripper.support_clearance_m-.0001,
                    side['observation_projected_side_evidence_m']-side_uncertainty-.0001]
                if prism is not None:
                    px, py, pz = ((prism-cam[:3, 3]) @ cam[:3, :3]).T
                    # Preserve the existing proposal reserve for the later
                    # strict IK endpoint; actual acceptance uses the shared image-margin policy.
                    values = np.r_[values, px-near, far-px,
                        ((cx-margin-proposal_image_reserve_px)*px-fx*py)/fx,
                        ((width-1.-margin-proposal_image_reserve_px-cx)*px+fx*py)/fx,
                        ((cy-margin-proposal_image_reserve_px)*px-fy*pz)/fy,
                        ((height-1.-margin-proposal_image_reserve_px-cy)*px+fy*pz)/fy]
                if arrival_bound is not None:
                    values=np.r_[values,arrival_bound.lower_bounds(q,
                        extra_margin_px=proposal_image_reserve_px)]
                last[:] = [q.copy(), values]
                key = q.tobytes()
                if (len(early_seen) < 4 and key not in early_seen
                        and (tier == 1. or (missing_side_ray_projection(delta,jaw,half_width,sign) >= requested
                             and sign*float(jaw@delta)/max(1e-12,np.linalg.norm(delta))
                                 >= required_center_projection+(tier-1.)*gain_target/observation_span))
                        and np.all(np.isfinite(values)) and np.all(values >= 0.)):
                    early_seen.add(key)
                    try:
                        endpoint = complete_endpoint(q)
                    except (ValueError, TypeError, RuntimeError, np.linalg.LinAlgError) as exc:
                        row_audit['early_endpoint_checks'].append(dict(ok=False, reason=str(exc)))
                    else:
                        row_audit['early_endpoint_checks'].append(dict(ok=endpoint[-1]))
                        if endpoint[-1]:
                            raise _EndpointQualified(endpoint)
                return values

            try:
                def objective(q):
                    remaining()
                    if tier > 1.:
                        # A preference, not a weakened exposure/clearance gate:
                        # all original endpoint checks remain below. Capping the
                        # benefit retains the existing bounded exposure target.
                        delta = (fk(np.clip(q, fk.limits[:, 0], fk.limits[:, 1])) @ mount)[:3, 3]-gc
                        return -min(requested,missing_side_ray_projection(delta,jaw,half_width,sign),
                            baseline_required-initial_center_projection
                                +sign*float(jaw@delta)/max(1e-12,np.linalg.norm(delta))
                                -(gain_target+.0002)/observation_span)
                    motion = float(np.sum((q-q0)**2))
                    if prism is None:
                        return motion
                    cam = fk(np.clip(q, fk.limits[:, 0], fk.limits[:, 1])) @ mount
                    return motion + _arrival_center_preference(
                        prism, cam, k, (width, height))['cost']

                try:
                    result = minimize(objective, numerical_seed, method='SLSQP',
                        bounds=proposal_limits.tolist(), constraints=[dict(type='ineq', fun=inequalities)],
                        options=dict(maxiter=iteration_limit, ftol=1e-10), callback=iteration_completed)
                except _EndpointQualified as qualified:
                    endpoint = qualified.endpoint
                    row_audit.update(early_endpoint_qualified=True, solver_success=False,
                        solver_message='original endpoint gates qualified a feasible probe before objective convergence')
                else:
                    result_q = _array(result.x, q0.shape, 'solver joints')
                    if (np.any(result_q < fk.limits[:, 0]) or np.any(result_q > fk.limits[:, 1])
                            or isinstance(result.nit, (bool, np.bool_))
                            or not isinstance(result.nit, (int, np.integer))
                            or not 0 <= result.nit <= iteration_limit+1):
                        numerical_seeds.pop(seed_key, None)
                        raise ValueError('invalid bounded solver result')
                    retain_numerical_seed(result_q)
                    reported_iterations[0] = int(result.nit)
                    row_audit.update(iterations=int(result.nit), solver_success=bool(result.success),
                                     solver_message=str(result.message))
                    endpoint = complete_endpoint(result.x)
                remaining()
                q, tool, cam, checks, accepted = endpoint
                if not accepted:
                    row_audit.update(reason='original endpoint gate rejected', endpoint_checks=checks)
                    continue
                row_audit['accepted'] = True
                if tier == 1.:
                    qualified_seeds[seed_key] = q.copy()
                candidates.append(dict(joint_names=list(fk.names), q=q.tolist(), T_base_tool0=tool.tolist(),
                    T_base_camera_link=cam.tolist(), model_sha256=fk.model_sha256, binding=deepcopy(binding),
                    endpoint_checks=checks, direction_gap_index=gap_index,
                    **({'exposure_target_noise_multiplier': tier} if predicted_gain_target_m is None
                   else dict(exposure_target_multiplier=tier, predicted_gain_target_m=predicted_gain_target_m,
                             endpoint_acceptance_predicted_gain_target_m=gain_target,
                             predicted_gain_policy=PREDICTED_GAIN_POLICY)),
                    requested_new_side_gain_m=tier*gain_target+.0002,
                    exposure_prediction_only=True, proves_common_contact_height=False,
                    joint_max_delta_rad=float(np.max(np.abs(q-q0))), execution_authority=False,
                    whole_arm_collision_checked=False, complete_path_checked=False,
                    actual_arrival_measurement_required=True))
            except _BudgetExpired:
                row_audit['reason'] = 'shared wall-clock budget exhausted'; raise
            except _DirectionBudgetExpired as exc:
                row_audit['reason'] = str(exc) or 'direction wall-clock slice exhausted'
            except (ValueError, TypeError, RuntimeError, np.linalg.LinAlgError) as exc:
                row_audit['reason'] = str(exc)
            finally:
                # SLSQP 1.3 reports/notifies maxiter+1 on iteration-limit exit.
                # Cap that terminal notification at the allocated iterations. A
                # backend that omits callbacks is charged its reported count,
                # or the full allocation if no usable result was returned.
                used = (completed_iterations[0] or reported_iterations[0]
                        or (1 if row_audit.get('early_endpoint_qualified') else iteration_limit))
                charged = min(iteration_limit, max(1, used))
                iteration_budget[seed_key] -= charged
                row_audit.update(solver_callback_notifications=completed_iterations[0],
                    charged_iterations=charged,
                    remaining_direction_iterations=iteration_budget[seed_key])
                direction_deadline = None
                row_audit['elapsed_sec'] = max(0., clock()-direction_started)
        # The caller ranks all returned families by resolved information and
        # checked motion duration. This local order is only a joint-cost hint.
        candidates.sort(key=lambda row: row['joint_max_delta_rad'])
    except _BudgetExpired:
        report['status'] = 'budget_exhausted'
    except (KeyError, TypeError, ValueError, AttributeError, OverflowError) as exc:
        report.update(status='invalid_input', reason=str(exc))
    finally:
        report.update(candidate_count=len(candidates), tier_attempt_count=len(report['directions']),
                      qualified_direction_count=len(qualified_seeds),
                      attempted_direction_count=len({(r['gap_index'], r['side_sign']) for r in report['directions']}),
                      elapsed_sec=max(0., clock()-started))
    return tuple(candidates), report
