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
    SerialUrdfFk, observation_endpoint_support_bound)
from alicia_flexible_grasp.robot.observation_view_guard import (
    validate_observation_support_projection)
from .observe_return_visual import missing_side_view_gain
from .observation_arrival_projection import validate_arrival_projection
from .observation_endpoint_precompute import PreparedEndpointSupportBound


class _BudgetExpired(Exception):
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
        max_directions=2, max_iterations=96, max_seconds=2.,
        proposal_image_reserve_px=4., clock=time.monotonic):
    """Return bounded exposure tiers, with one measured seed per side and tier.

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
    After the first one-noise proposal passes
    all endpoint gates, its two-noise target runs before the other directions.
    Remaining baselines then precede remaining extra targets, all within the
    same budget. Every endpoint gate still uses the original uncertainty. Neither
    exposure tier establishes measured common contact height. Sorting is a
    proposal heuristic only.
    """
    started = clock()
    report = dict(policy='bounded_measured_joint_space_missing_side_proposals',
        execution_authority=False, original_candidates_removed=0, directions=[],
        candidate_count=0, status='invalid_input', proves_complete_path=False,
        max_seconds=max_seconds, max_iterations=max_iterations,
        proposal_image_reserve_px=proposal_image_reserve_px,
        backend='scipy_slsqp', seed_policy='one_current_measured_joint_state_per_direction_per_tier',
        exposure_target_noise_multipliers=[1., 2.],
        exposure_tier_order='first_accepted_baseline_then_same_direction_extra',
        exposure_prediction_only=True, proves_common_contact_height=False,
        endpoint_acceptance_uncertainty_m=depth_uncertainty_m)
    candidates = []

    def remaining():
        if clock()-started >= max_seconds:
            raise _BudgetExpired()

    try:
        if (not isinstance(fk, SerialUrdfFk) or not isinstance(gripper, GripperGeometry)
                or type(max_directions) is not int or not 1 <= max_directions <= 6
                or type(max_iterations) is not int or not 1 <= max_iterations <= 160
                or isinstance(max_seconds, bool) or not math.isfinite(max_seconds)
                or not 0. < max_seconds <= 3.):
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
        side_uncertainty = depth_uncertainty_m if side_depth_uncertainty_m is None else side_depth_uncertainty_m
        if (isinstance(side_uncertainty, bool) or not math.isfinite(float(side_uncertainty))
                or not 0. <= side_uncertainty <= .05):
            raise ValueError('finite current-source optical side uncertainty required')
        initial = fk(q0)  # Enforces the actual URDF limits, including measured seed.
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
        optical = (initial @ mount)[:3, 0]
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
                previous = object_height*max(0., sign*float(jaw @ origin)/np.linalg.norm(origin))
                required = (previous+depth_uncertainty_m+.0002)/object_height
                if required >= 1.:
                    continue
                steering = max(0., math.acos(float(np.clip(optical @ (-sign*jaw), -1., 1.)))
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
        remaining()
        tiers = (1., 2.) if depth_uncertainty_m > 0. else (1.,)
        report.update(status='complete', available_directions=len(unique),
                      exposure_target_noise_multipliers=list(tiers),
                      selected_direction_count=len(unique[:max_directions]),
                      maximum_tier_attempt_count=len(tiers)*len(unique[:max_directions]), model_sha256=fk.model_sha256,
                      binding=binding, joint_error_bounds_rad=errors.tolist(),
                      optical_side_uncertainty_m=side_uncertainty,
                      optical_side_proposal_reserve_m=.0001)
        def attempts():
            # Keep the first useful baseline before spending time on a harder
            # target. A second infeasible baseline must not starve the stronger
            # view of a direction already admitted by the original gates.
            # Solver success alone does not qualify this priority.
            preferred_index = None
            directions = unique[:max_directions]
            for index, direction in enumerate(directions):
                yield 1., direction
                if (len(tiers) > 1 and preferred_index is None
                        and report['directions'][-1]['accepted']):
                    preferred_index = index
                    yield 2., direction
            if len(tiers) > 1:
                for index, direction in enumerate(directions):
                    if index != preferred_index:
                        yield 2., direction

        for tier, (_steering, gap_index, sign, gc, jaw, half_width, baseline_required) in attempts():
            remaining(); direction_started = clock()
            required = baseline_required + (tier-1.)*depth_uncertainty_m/float(gaps[gap_index]['object_height_m'])
            row_audit = dict(gap_index=gap_index, side_sign=sign, accepted=False, seed_count=1,
                exposure_target_noise_multiplier=tier,
                requested_new_side_gain_m=tier*depth_uncertainty_m+.0002,
                endpoint_acceptance_uncertainty_m=depth_uncertainty_m,
                exposure_prediction_only=True, proves_common_contact_height=False)
            report['directions'].append(row_audit)
            if required >= 1.:
                row_audit.update(reason='requested exposure exceeds object-height projection', elapsed_sec=0.)
                continue
            # A request-local last-evaluation cache avoids redundant solver probes.
            last = [None, None]
            early_seen = set()
            row_audit['early_endpoint_checks'] = []

            def complete_endpoint(q):
                # This is shared by early proposals and the original final
                # solver result. Neither numerical feasibility nor success
                # substitutes for measured support, arrival or CAD checks.
                remaining()
                q = _array(q, q0.shape, 'solver joints')
                if np.any(q < fk.limits[:, 0]-1e-10) or np.any(q > fk.limits[:, 1]+1e-10):
                    raise ValueError('solver returned joints outside original limits')
                q = np.clip(q, fk.limits[:, 0], fk.limits[:, 1])
                tool = fk(q); cam = tool @ mount
                gain = missing_side_view_gain(gaps, cam[:3, 3], depth_uncertainty_m)
                view = validate_planned_observation_working_volume(corners, cam, k, width, height,
                    snapshot_stamp_ns=stamp, **limits)
                envelope = evaluate_open_gripper_observation_envelope(gripper=gripper, T_base_tool0=tool,
                    opening_width_m=opening_width_m, support_normal_base=normal, support_offset_m=offset,
                    obb_center_base=center, R_base_obb=rotation, obb_size_xyz_m=size)
                robust = observation_endpoint_support_bound(fk, q, errors, normal, offset, gripper, opening_width_m)
                projection = validate_observation_support_projection(cam, points, center, k, width, height,
                    target_corners_base=corners, **limits)
                side = _optical_side_evidence(cam, normal, rotation, size, side_uncertainty)
                arrival_view = None if arrival_projection is None else validate_arrival_projection(
                    arrival_projection, cam, k, width, height, **limits)
                baseline = float(np.linalg.norm(tool[:3, 3]-initial[:3, 3]))
                checks = dict(missing_side_view_gain=gain, working_volume=view,
                    observation_envelope=asdict(envelope), conditional_endpoint_support=robust,
                    measured_support_projection=projection, tool_translation_delta_m=baseline,
                    optical_side_evidence=side, within_original_joint_limits=True)
                if arrival_view is not None:
                    checks['arrival_projection_endpoint'] = arrival_view
                    checks['arrival_centre_preference'] = _arrival_center_preference(
                        prism, cam, k, (width, height))
                accepted = (gain['ok'] and view['ok'] and envelope.ok and robust['ok']
                    and projection['ok'] and side['ok'] and baseline >= minimum_view_baseline_m
                    and (arrival_view is None or arrival_view['ok']))
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
                    .1*(sign*float(jaw @ delta)/max(1e-12, np.linalg.norm(delta))-required),
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
                last[:] = [q.copy(), values]
                key = q.tobytes()
                if (len(early_seen) < 4 and key not in early_seen
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
                    motion = float(np.sum((q-q0)**2))
                    if prism is None:
                        return motion
                    cam = fk(np.clip(q, fk.limits[:, 0], fk.limits[:, 1])) @ mount
                    return motion + _arrival_center_preference(
                        prism, cam, k, (width, height))['cost']

                try:
                    result = minimize(objective, q0.copy(), method='SLSQP',
                        bounds=fk.limits.tolist(), constraints=[dict(type='ineq', fun=inequalities)],
                        options=dict(maxiter=max_iterations, ftol=1e-10))
                except _EndpointQualified as qualified:
                    endpoint = qualified.endpoint
                    row_audit.update(early_endpoint_qualified=True, solver_success=False,
                        solver_message='original endpoint gates qualified a feasible probe before objective convergence')
                else:
                    row_audit.update(iterations=int(result.nit), solver_success=bool(result.success),
                                     solver_message=str(result.message))
                    endpoint = complete_endpoint(result.x)
                remaining()
                q, tool, cam, checks, accepted = endpoint
                if not accepted:
                    row_audit.update(reason='original endpoint gate rejected', endpoint_checks=checks)
                    continue
                row_audit['accepted'] = True
                candidates.append(dict(joint_names=list(fk.names), q=q.tolist(), T_base_tool0=tool.tolist(),
                    T_base_camera_link=cam.tolist(), model_sha256=fk.model_sha256, binding=deepcopy(binding),
                    endpoint_checks=checks, direction_gap_index=gap_index,
                    exposure_target_noise_multiplier=tier,
                    requested_new_side_gain_m=tier*depth_uncertainty_m+.0002,
                    exposure_prediction_only=True, proves_common_contact_height=False,
                    joint_max_delta_rad=float(np.max(np.abs(q-q0))), execution_authority=False,
                    whole_arm_collision_checked=False, complete_path_checked=False,
                    actual_arrival_measurement_required=True))
            except _BudgetExpired:
                row_audit['reason'] = 'shared wall-clock budget exhausted'; raise
            except (ValueError, TypeError, RuntimeError, np.linalg.LinAlgError) as exc:
                row_audit['reason'] = str(exc)
            finally:
                row_audit['elapsed_sec'] = max(0., clock()-direction_started)
        candidates.sort(key=lambda row: row['joint_max_delta_rad'])
    except _BudgetExpired:
        report['status'] = 'budget_exhausted'
    except (KeyError, TypeError, ValueError, AttributeError, OverflowError) as exc:
        report.update(status='invalid_input', reason=str(exc))
    finally:
        report.update(candidate_count=len(candidates), tier_attempt_count=len(report['directions']),
                      elapsed_sec=max(0., clock()-started))
    return tuple(candidates), report
