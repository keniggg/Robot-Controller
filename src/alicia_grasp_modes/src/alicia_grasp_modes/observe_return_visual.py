"""Request-local missing-view evidence; never a contact or motion certificate."""
from collections.abc import Mapping
import math

import numpy as np


def bounded_bilateral_view_gaps(diagnostics, geometry, config):
    """Require a missing measured side and a whole-OBB open-aperture fit.

    Local contact-band fits can coexist with body collisions at other angles.
    Only the independently recorded missing-surface direction is used here.
    Observation motion and every later contact need their own complete checks.
    """
    try:
        table = diagnostics['tabletop_geometry']
        if (not isinstance(table, Mapping)
                or table.get('plan_phase') != 'CONTACT_EXECUTION_PLAN'
                or table.get('contact_execution_gate_deferred') is not False):
            return ()
        rotation = np.asarray(geometry.axes_base, dtype=float)
        size = np.asarray(geometry.size_xyz_m, dtype=float)
        normal = np.asarray(geometry.support_normal_base, dtype=float)
        limit = float(config.max_inner_gap_m)
        clearance = float(config.opening_fit_clearance_each_side_m)
        if (rotation.shape != (3, 3) or size.shape != (3,) or normal.shape != (3,)
                or not all(np.all(np.isfinite(v)) for v in (rotation, size, normal))
                or np.any(size <= 0.)
                or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8, rtol=0.)
                or abs(np.linalg.det(rotation) - 1.) > 1e-8
                or abs(np.linalg.norm(normal) - 1.) > 1e-8
                or np.dot(rotation[:, 2], normal) < 1. - 1e-5
                or not math.isfinite(limit) or not 0. < limit <= .05
                or not math.isfinite(clearance) or not 0. <= clearance < limit / 2.):
            return ()
        # Same support-plane angular convention as generate_tabletop_proposals.
        reference = np.array([0., 1., 0.] if abs(normal[0]) > .9 else [1., 0., 0.])
        basis_x = reference - normal * np.dot(reference, normal)
        basis_x /= np.linalg.norm(basis_x)
        basis_y = np.cross(normal, basis_x)
        rows = table.get('surface_evidence_rejections', ())
        if not isinstance(rows, (tuple, list)):
            return ()
        result = []
        for row in rows:
            if not isinstance(row, Mapping) or row.get('failure_code') != 'BILATERAL_SURFACE_EVIDENCE_MISSING':
                continue
            keys = ('angle_deg', 'negative_reach_m', 'positive_reach_m', 'maximum_side_reach_m')
            values = [row.get(k) for k in keys]
            if any(isinstance(v, (bool, str)) or not isinstance(v, (int, float))
                   or not math.isfinite(v) for v in values):
                continue
            angle, negative, positive, reach_limit = values
            if not (0. <= angle < 180. and 0. < negative <= reach_limit <= limit / 2.
                    and 0. < positive <= reach_limit):
                continue
            jaw = basis_x * math.cos(math.radians(angle)) + basis_y * math.sin(math.radians(angle))
            width = float(np.abs(rotation.T @ jaw) @ size)
            required = width + 2. * clearance
            if required > limit:
                continue
            result.append(dict(failure_code='BILATERAL_SURFACE_EVIDENCE_MISSING',
                angle_deg=angle, jaw_axis_base=jaw.tolist(), whole_obb_width_m=width,
                required_open_width_m=required, max_inner_gap_m=limit,
                opening_fit_clearance_each_side_m=clearance,
                negative_reach_m=negative, positive_reach_m=positive,
                maximum_side_reach_m=reach_limit, contact_or_motion_authorized=False))
        return tuple(sorted(result, key=lambda row: (row['required_open_width_m'], row['angle_deg'])))
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
        return ()


def measured_bilateral_view_gaps(diagnostics, geometry, config, surface,
                                  finger_geometry, initial_camera_origin):
    """Annotate aperture-fitting directions using only retained measured points.

    Use the contact evaluator's exact masks, including its footprint, robust
    jaw bands and common-height filtering. These are observation proposals,
    never proof that an unseen surface exists or will be graspable.
    """
    from alicia_flexible_grasp.grasp.gripper_geometry import (
        _bilateral_surface_measurement, _MEASURED_SURFACE_TRIM_FRACTION)
    try:
        gaps = bounded_bilateral_view_gaps(diagnostics, geometry, config)
        center = np.asarray(geometry.center_base, dtype=float)
        normal = np.asarray(geometry.support_normal_base, dtype=float)
        camera = np.asarray(initial_camera_origin, dtype=float)
        if (center.shape != (3,) or camera.shape != (3,)
                or not np.all(np.isfinite(center)) or not np.all(np.isfinite(camera))
                or np.linalg.norm(camera - center) <= 1e-9):
            return ()
        minimum = max(12, config.min_contact_band_points)
        points = np.asarray(surface.points_base, dtype=float)
        support_height = (points - center) @ normal
        result = []
        for gap in gaps:
            jaw = np.asarray(gap['jaw_axis_base'])
            counts, lower, upper, masks = _bilateral_surface_measurement(
                surface, center, jaw, -normal, finger_geometry, minimum, normal)
            if lower is not None and upper is not None:
                continue  # The retained measurement no longer has this gap.
            side_counts = (0, 0) if counts is None else counts[:2]
            heights = []
            for mask in (masks if isinstance(masks, tuple) else (None, None)):
                values = support_height[mask] if mask is not None else ()
                quantiles = np.quantile(values, (_MEASURED_SURFACE_TRIM_FRACTION,
                    1. - _MEASURED_SURFACE_TRIM_FRACTION)) if len(values) else (0., 0.)
                heights.append(float(quantiles[1] - quantiles[0]))
            missing = tuple(sign for sign, count, height in zip((-1, 1), side_counts, heights)
                            if count < minimum or height <= 1e-9)
            if not missing:
                # Both sides have points but lack a common height. Improving
                # an already stronger side must not stand in for the weak one.
                weakest = min(heights)
                missing = tuple(sign for sign, height in zip((-1, 1), heights)
                                if height <= weakest + 1e-9)
            result.append(dict(gap, missing_side_signs=list(missing),
                negative_measured_points=int(side_counts[0]), positive_measured_points=int(side_counts[1]),
                negative_measured_height_m=heights[0], positive_measured_height_m=heights[1],
                minimum_points_per_side=int(minimum), target_center_base_m=center.tolist(),
                missing_side_policy='insufficient_samples_or_weakest_common_height',
                object_height_m=float(geometry.size_xyz_m[2]),
                initial_camera_origin_base_m=camera.tolist(),
                source_view_stamps_ns=list(surface.view_stamps_ns),
                target_epoch=int(surface.identity.epoch), target_track_id=surface.identity.track_id))
        return tuple(result)
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
        return ()


def missing_side_view_gain(gaps, camera_origin, depth_uncertainty_m):
    """Screen existing views for predicted new exposure of a measured deficit.

    A radial approach or camera roll has zero directional gain. Prediction is
    bounded by the live depth uncertainty; actual registration/contact checks
    must establish whether any useful surface was really observed.
    """
    failure = dict(ok=False, code='OBSERVE_RETURN_MISSING_SIDE_NOT_EXPOSED',
                   contact_or_motion_authorized=False, measured_registration_required=True)
    try:
        camera = np.asarray(camera_origin, dtype=float)
        uncertainty = float(depth_uncertainty_m)
        if (camera.shape != (3,) or not np.all(np.isfinite(camera))
                or not math.isfinite(uncertainty) or uncertainty < 0.):
            return dict(failure, code='OBSERVE_RETURN_VIEW_GAIN_INPUT_INVALID')
        accepted = []
        for gap in gaps:
            center = np.asarray(gap['target_center_base_m'], dtype=float)
            initial = np.asarray(gap['initial_camera_origin_base_m'], dtype=float) - center
            delta = camera - center
            jaw = np.asarray(gap['jaw_axis_base'], dtype=float)
            height = float(gap['object_height_m'])
            half_width = .5 * float(gap['whole_obb_width_m'])
            if (any(v.shape != (3,) or not np.all(np.isfinite(v)) for v in (center, initial, jaw))
                    or abs(np.linalg.norm(jaw) - 1.) > 1e-8
                    or np.linalg.norm(initial) <= 1e-9 or np.linalg.norm(delta) <= 1e-9
                    or not math.isfinite(height) or height <= 0.
                    or not math.isfinite(half_width) or half_width <= 0.):
                continue
            original_projection = float(jaw @ (initial / np.linalg.norm(initial)))
            proposed_projection = float(jaw @ (delta / np.linalg.norm(delta)))
            for sign in gap['missing_side_signs']:
                if type(sign) is not int or sign not in (-1, 1):
                    continue
                exposed = height * max(0., sign * proposed_projection)
                previous = height * max(0., sign * original_projection)
                gain = exposed - previous
                if (sign * float(jaw @ delta) > half_width + 1e-9
                        and exposed > uncertainty + 1e-9 and gain > uncertainty + 1e-9):
                    accepted.append(dict(angle_deg=gap['angle_deg'], missing_side_sign=sign,
                        projected_missing_side_m=exposed, initial_projected_missing_side_m=previous,
                        predicted_new_side_gain_m=gain, depth_uncertainty_m=uncertainty,
                        target_epoch=gap['target_epoch'], target_track_id=gap['target_track_id'],
                        source_view_stamps_ns=list(gap['source_view_stamps_ns'])))
        if not accepted:
            return failure
        best = max(accepted, key=lambda row: row['predicted_new_side_gain_m'])
        return dict(failure, **best, ok=True, code='OBSERVE_RETURN_PREDICTED_NEW_MISSING_SIDE')
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
        return dict(failure, code='OBSERVE_RETURN_VIEW_GAIN_INPUT_INVALID')


def directed_missing_side_rotations(gaps, camera_rotation, support_normal,
                                    depth_uncertainty_m, minimum_camera_distance_m,
                                    maximum_rotation_deg=45., maximum_directions=2, *,
                                    allow_geometry_directed_expansion=False):
    """Generate a small, category-independent camera-direction proposal family.

    Rotate the current optical axis toward each measured weak side along its
    shortest great-circle path. The first angle is derived from the exact gain
    and whole-body side-plane inequalities, with one micrometre numerical
    reserve; the second is the bounded maximum in the original <=45 degree
    family. An explicit opt-in can add a second tier only when that family is
    empty and the measured gap requires >45 degrees. Its angles are derived
    from one and two depth-uncertainty units of gain, never a blind maximum-
    angle probe. These are unvalidated poses, not motion authorization. The
    old support-normal tilt family and all motion limits are independent.
    """
    try:
        rotation = np.asarray(camera_rotation, dtype=float)
        normal = np.asarray(support_normal, dtype=float)
        uncertainty, distance, maximum = map(float, (
            depth_uncertainty_m, minimum_camera_distance_m, maximum_rotation_deg))
        if (rotation.shape != (3, 3) or normal.shape != (3,)
                or not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(normal))
                or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8, rtol=0.)
                or abs(np.linalg.det(rotation)-1.) > 1e-8
                or abs(np.linalg.norm(normal)-1.) > 1e-8
                or not all(math.isfinite(v) for v in (uncertainty, distance, maximum))
                or uncertainty < 0. or distance <= 0. or not 0. < maximum <= 45.
                or type(maximum_directions) is not int or not 1 <= maximum_directions <= 2
                or type(allow_geometry_directed_expansion) is not bool):
            return ()
        optical = rotation[:, 0]
        directions, expanded_directions = [], []
        for gap in gaps:
            center = np.asarray(gap['target_center_base_m'], dtype=float)
            initial = np.asarray(gap['initial_camera_origin_base_m'], dtype=float) - center
            jaw = np.asarray(gap['jaw_axis_base'], dtype=float)
            height, width = float(gap['object_height_m']), float(gap['whole_obb_width_m'])
            if (any(v.shape != (3,) or not np.all(np.isfinite(v)) for v in (center, initial, jaw))
                    or abs(np.linalg.norm(jaw)-1.) > 1e-8 or np.linalg.norm(initial) <= 1e-9
                    or not math.isfinite(height) or height <= 0.
                    or not math.isfinite(width) or width <= 0.):
                continue
            initial /= np.linalg.norm(initial)
            for sign in gap['missing_side_signs']:
                if type(sign) is not int or sign not in (-1, 1):
                    continue
                wanted_optical = -sign * jaw
                previous = height * max(0., float(sign * jaw @ initial))
                required = max((previous + uncertainty + 1e-6) / height,
                               (.5 * width + 1e-6) / distance)
                if required >= 1.:
                    continue
                angle_to_side = math.acos(float(np.clip(optical @ wanted_optical, -1., 1.)))
                minimum_angle = max(0., angle_to_side - math.acos(required))
                in_original = minimum_angle <= math.radians(maximum) + 1e-12
                in_expansion = (allow_geometry_directed_expansion
                    and minimum_angle > math.radians(45.) + 1e-12
                    and minimum_angle <= math.radians(90.) + 1e-12)
                if not (in_original or in_expansion):
                    continue
                axis = np.cross(optical, wanted_optical)
                length = float(np.linalg.norm(axis))
                if length <= 1e-9:
                    continue
                axis /= length
                row = (minimum_angle, float(gap['required_open_width_m']),
                       gap, sign, axis, angle_to_side, previous, height, width)
                (directions if in_original else expanded_directions).append(row)

        def proposals(rows, expanded):
            result, selected_directions = [], []
            for minimum_angle, _width, gap, sign, axis, angle_to_side, previous, height, width in sorted(
                    rows, key=lambda item: (item[0], item[1], item[2]['angle_deg'], item[3])):
                wanted = -sign * np.asarray(gap['jaw_axis_base'])
                if any(float(wanted @ prior) > 1. - 1e-10 for prior in selected_directions):
                    continue
                if len(selected_directions) >= maximum_directions:
                    break
                # Preserve the original family's direction selection exactly.
                # Expanded directions consume a slot only after a valid proposal.
                if not expanded:
                    selected_directions.append(wanted)
                if expanded:
                    angles = [minimum_angle]
                    stronger_required = max((previous + 2.*uncertainty + 1e-6) / height,
                                            (.5*width + 1e-6) / distance)
                    if stronger_required < 1.:
                        stronger_angle = max(0., angle_to_side - math.acos(stronger_required))
                        if (stronger_angle <= math.radians(90.) + 1e-12
                                and stronger_angle - minimum_angle >= 1e-9):
                            angles.append(stronger_angle)
                else:
                    upper = min(math.radians(maximum), angle_to_side)
                    angles = ((minimum_angle,) if abs(upper-minimum_angle) < 1e-9
                              else (minimum_angle, upper))
                x, y, z = axis
                skew = np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
                direction_result = []
                for angle in angles:
                    if angle <= 1e-9:
                        continue  # Centred zero-turn views already exist in the old library.
                    turn = np.eye(3) + math.sin(angle)*skew + (1.-math.cos(angle))*(skew@skew)
                    proposed = turn @ rotation
                    if float(proposed[:, 0] @ normal) >= -1e-9:
                        continue  # No upward/horizontal camera proposal is invented.
                    camera = np.asarray(gap['target_center_base_m']) - distance*proposed[:, 0]
                    gain = missing_side_view_gain((gap,), camera, uncertainty)
                    if not gain['ok']:
                        continue
                    metadata = dict(gain, orientation_source='measured_missing_side_geodesic',
                        camera_rotation_offset_deg=math.degrees(angle),
                        minimum_required_rotation_deg=math.degrees(minimum_angle),
                        maximum_rotation_deg=90. if expanded else maximum,
                        maximum_directions=maximum_directions,
                        maximum_candidates_per_direction=2, numeric_reserve_m=1e-6,
                        rotation_axis_base=axis.tolist(), jaw_axis_base=list(gap['jaw_axis_base']),
                        minimum_camera_distance_m=distance, observation_only=True,
                        direction_source='measured_bilateral_weak_side',
                        measured_target_geometry_direction_used=True, contact_candidate_yaw_used=False,
                        optimality_scope='minimum_distance_centered_view_information_model_only',
                        global_path_optimality_claimed=False)
                    if expanded:
                        metadata.update(proposal_tier='geometry_directed_expansion',
                            original_maximum_rotation_deg=maximum,
                            expanded_maximum_rotation_deg=90.,
                            expansion_reason='original_family_empty_and_measured_gap_requires_over_45_deg',
                            expansion_opt_in=True, original_family_candidate_count=0,
                            angle_generation_policy='minimum_turn_for_one_or_two_depth_uncertainty_gain',
                            maximum_angle_probe_generated=False)
                    direction_result.append((proposed, metadata))
                if expanded and direction_result:
                    selected_directions.append(wanted)
                    for _, metadata in direction_result:
                        metadata['geometry_derived_angles_deg'] = [
                            row[1]['camera_rotation_offset_deg'] for row in direction_result]
                result.extend(direction_result)
            return tuple(sorted(result, key=lambda item: item[1]['camera_rotation_offset_deg']))

        original = proposals(directions, expanded=False)
        return original or proposals(expanded_directions, expanded=True)
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
        return ()


def bounded_existing_camera_rolls(offsets, maximum=25):
    """Choose dispersed optical-axis rolls already present in the old library."""
    try:
        values = tuple(float(value) for value in offsets)
        if (type(maximum) is not int or not 1 <= maximum <= 25 or not values
                or any(not math.isfinite(value) or abs(value) > 180. for value in values)):
            return ()
        def separation(a, b):
            return abs((a-b+180.) % 360. - 180.)
        chosen = [min(values, key=lambda value: (abs(value), values.index(value)))]
        while len(chosen) < min(maximum, 4):
            remaining = [value for value in values if all(separation(value, prior) > 1e-9 for prior in chosen)]
            if not remaining:
                break
            chosen.append(max(remaining, key=lambda value: (
                min(separation(value, prior) for prior in chosen), -abs(value), -values.index(value))))
        first = sorted(chosen, key=lambda value: (abs(value), values.index(value)))
        rest = sorted((value for value in values if all(separation(value, prior) > 1e-9 for prior in first)),
                      key=lambda value: (abs(value), values.index(value)))
        return tuple((first + rest)[:maximum])
    except (TypeError, ValueError, OverflowError):
        return ()


def diversified_observation_checks(variants, nominal_distance_m):
    """Interleave retained roll/distance families inside the unchanged budget.

    All inputs have already passed endpoint checks. This schedules checks;
    only the existing strict planner/following tests can make them reachable.
    """
    variants = tuple(variants)
    if not variants:
        return ()
    try:
        nominal = float(nominal_distance_m)
        if not math.isfinite(nominal) or nominal <= 0.:
            return variants
        unique_rolls = tuple(dict.fromkeys(float(v['camera_roll_offset_deg']) for v in variants))
        rolls = bounded_existing_camera_rolls(unique_rolls)
        distances = sorted(set(float(v['camera_target_distance_m']) for v in variants))
        if any(not math.isfinite(value) for value in (*rolls, *distances)):
            return variants
        distance_order = []
        for value in (min(distances, key=lambda d: (abs(d-nominal), d)), distances[0], distances[-1], *distances):
            if value not in distance_order:
                distance_order.append(value)
        groups = {roll: [v for v in variants if float(v['camera_roll_offset_deg']) == roll] for roll in rolls}
        result, cycle = [], 0
        while any(groups.values()):
            for index, roll in enumerate(rolls):
                if not groups[roll]:
                    continue
                preferred = distance_order[(cycle+index) % len(distance_order)]
                def key(v):
                    clearance = float(getattr(v.get('observation_envelope'), 'minimum_support_clearance_m', 0.))
                    return (abs(float(v['camera_target_distance_m'])-preferred),
                            float(v['camera_tilt_offset_deg']), -clearance,
                            int(v.get('preference_index',0)))
                selected = min(groups[roll], key=key)
                groups[roll].pop(next(i for i, item in enumerate(groups[roll]) if item is selected))
                result.append(selected)
            cycle += 1
        return tuple(result)
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
        return variants
