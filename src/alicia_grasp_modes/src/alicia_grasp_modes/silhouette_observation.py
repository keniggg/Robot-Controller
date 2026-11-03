"""Bounded view proposals for measured silhouette ambiguity, never contact.

The original geometry remains authoritative for every physical path check.
This predicts potential exposure at the disputed edge; it neither establishes
surface ownership nor removes any point from the measured collision model.
"""
from copy import deepcopy
import math
import re

import numpy as np

from .selection import parse_selection


TRIGGER = 'RGB_DEPTH_SILHOUETTE_AMBIGUOUS'
_INPUTS = ('rgb', 'depth', 'seed', 'hard_foreground', 'hard_background', 'height')


def _vector(value, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError('invalid ' + name)
    return result


def _geometry(evidence):
    data = evidence['view_geometry']
    center = _vector(data['target_center_base_m'], 'target center')
    anchor = _vector(data['ambiguity_anchor_base_m'], 'ambiguity anchor')
    normal = _vector(data['support_normal_base'], 'support normal')
    origin = _vector(data['initial_camera_origin_base_m'], 'initial camera')
    rotation = np.asarray(data['initial_camera_rotation_base'], dtype=float)
    height, uncertainty, offset = map(float, (data['target_height_m'],
        data['depth_uncertainty_m'], data['support_offset_m']))
    if (abs(np.linalg.norm(normal)-1.) > 1e-6 or rotation.shape != (3, 3)
            or not np.all(np.isfinite(rotation))
            or not np.allclose(rotation.T@rotation, np.eye(3), atol=1e-6, rtol=0.)
            or abs(np.linalg.det(rotation)-1.) > 1e-6
            or not all(math.isfinite(v) for v in (height, uncertainty, offset))
            or not 0. < height <= .5 or uncertainty < 0.
            or not isinstance(data.get('source_frame_id'), str) or not data['source_frame_id']):
        raise ValueError('invalid measured view geometry')
    initial = origin-center
    lateral = anchor-center
    lateral -= normal*float(lateral@normal)
    radius = float(np.linalg.norm(lateral))
    if (np.linalg.norm(initial) <= 1e-9 or radius <= 1e-6
            or radius > .6 or float(origin@normal+offset) <= 0.):
        raise ValueError('unresolved ambiguity direction')
    return data, center, anchor, normal, origin, initial/np.linalg.norm(initial), lateral/radius, height, uncertainty


def validate_silhouette_ambiguity(evidence, *, selection, request_id, generation,
                                  target_epoch, snapshot_stamp_ns, target_track_id):
    """Validate immutable request/source evidence; this grants no motion."""
    if not isinstance(evidence, dict):
        raise ValueError('missing silhouette evidence')
    expected = dict(request_id=request_id, generation=generation, target_epoch=target_epoch,
                    snapshot_stamp_ns=snapshot_stamp_ns, target_track_id=target_track_id)
    for key, value in expected.items():
        if type(evidence.get(key)) is not type(value) or evidence.get(key) != value:
            raise ValueError('silhouette binding mismatch: '+key)
    for key in ('generation', 'target_epoch', 'snapshot_stamp_ns'):
        if type(expected[key]) is not int or expected[key] < (1 if key == 'snapshot_stamp_ns' else 0):
            raise ValueError('invalid silhouette '+key)
    selected = parse_selection(selection)
    if (parse_selection(evidence.get('selection')) != selected
            or selected['mode'] != 'unknown' or selected['strategy'] != 'observe_return'
            or not selected['stamp_ns'] < snapshot_stamp_ns
            or evidence.get('source_stamp_ns') != snapshot_stamp_ns
            or type(evidence.get('source_stamp_ns')) is not int
            or evidence.get('baseline_mask_reproduced') is not True):
        raise ValueError('silhouette source or baseline mismatch')
    if (evidence.get('ok') is not True
            or evidence.get('diagnostic_only') is not True
            or evidence.get('execution_authority') is not False
            or evidence.get('observation_authority') is not False
            or evidence.get('code') != 'SILHOUETTE_SOURCE_DIAGNOSTIC_READY'
            or evidence.get('source_kind') != 'original_single_frame_of_fused_snapshot'
            or evidence.get('geometry_used_only_as_request_context') is not True):
        raise ValueError('missing original single-source diagnostic proof')
    stamps = evidence['snapshot_source_stamps_ns']
    if (not isinstance(stamps, list) or not stamps
            or any(type(v) is not int or v <= selected['stamp_ns'] for v in stamps)
            or any(b <= a for a, b in zip(stamps, stamps[1:]))
            or stamps[-1] != snapshot_stamp_ns):
        raise ValueError('invalid fused snapshot source lineage')
    plane = evidence['plane_provenance']
    anchor_stamp = plane.get('anchor_source_stamp_ns')
    if (plane.get('kind') != 'same_source_perception_verified_model'
            or plane.get('verification_stamp_ns') != snapshot_stamp_ns
            or type(anchor_stamp) is not int
            or not selected['stamp_ns'] < anchor_stamp <= snapshot_stamp_ns):
        raise ValueError('missing live source model verification')
    raw = evidence['raw_source_evidence']
    if set(raw) != {'rgb', 'depth', 'published_mask'}:
        raise ValueError('missing original source byte digests')
    for name, item in raw.items():
        if (not re.fullmatch('[0-9a-f]{64}', str(item.get('sha256', '')))
                or item.get('dtype') != ('uint16' if name == 'depth' else 'uint8')
                or not isinstance(item.get('shape'), list)
                or any(type(v) is not int or v <= 0 for v in item['shape'])):
            raise ValueError('invalid original source byte digest')
    shape = raw['depth']['shape']
    crop = evidence['crop_xyxy']
    if (len(shape) != 2 or raw['published_mask']['shape'] != shape
            or raw['rgb']['shape'] != shape+[3]
            or not isinstance(crop, list) or len(crop) != 4
            or any(type(v) is not int for v in crop)
            or not (0 <= crop[0] < crop[2] <= shape[1] and 0 <= crop[1] < crop[3] <= shape[0])):
        raise ValueError('invalid original crop dimensions')
    projection = evidence['projection_evidence']
    intrinsics = np.asarray(projection['intrinsics'], dtype=float)
    scale = float(projection['depth_scale'])
    if (intrinsics.shape != (4,) or not np.all(np.isfinite(intrinsics))
            or np.any(intrinsics[:2] <= 0) or not math.isfinite(scale) or scale <= 0
            or not re.fullmatch('[0-9a-f]{64}', str(projection.get('transform_sha256', '')))):
        raise ValueError('invalid source projection provenance')
    data = evidence['view_geometry']
    if (data.get('source_frame_id') != evidence.get('frame_id')
            or data.get('camera_rotation_convention') != 'ros_camera_link'
            or data.get('target_geometry_source') != 'original_fused_request_collision_geometry'
            or data.get('target_geometry_source_stamps_ns') != stamps
            or data.get('ambiguity_anchor_source_stamp_ns') != snapshot_stamp_ns
            or data.get('ambiguity_anchor_source') != 'mean_of_current_original_valid_depth_disagreement_rays'):
        raise ValueError('view geometry source provenance mismatch')
    measurement = evidence['measurement']
    if (measurement.get('source_stamp_ns') != snapshot_stamp_ns
            or measurement.get('source_stamps_ns') != dict.fromkeys(_INPUTS, snapshot_stamp_ns)
            or measurement.get('diagnostic_only') is not True
            or measurement.get('execution_authority') is not False
            or measurement.get('observation_authority') is not False
            or measurement.get('executable_geometry') is not False
            or measurement.get('physical_contour_certified') is not False
            or measurement.get('preferred_hypothesis') is not None
            or measurement.get('ambiguous') is not True
            or measurement.get('hypotheses_evaluated') != 2
            or measurement.get('conclusion') != 'SILHOUETTE_INITIALIZATION_SENSITIVE'):
        raise ValueError('silhouette hypotheses are not diagnostic uncertainty')
    inputs = measurement['input_evidence']
    if set(inputs) != set(_INPUTS):
        raise ValueError('incomplete silhouette input hashes')
    for item in inputs.values():
        if (not isinstance(item.get('sha256'), str)
                or not re.fullmatch('[0-9a-f]{64}', item['sha256'])
                or not isinstance(item.get('shape'), list) or not item['shape']
                or any(type(v) is not int or v <= 0 for v in item['shape'])
                or not isinstance(item.get('dtype'), str) or not item['dtype']):
            raise ValueError('invalid silhouette input digest')
    difference = measurement['disagreement']
    if (type(difference.get('valid_depth_pixels')) is not int
            or difference['valid_depth_pixels'] < 25
            or not .02 <= float(difference['fraction_of_union']) <= 1.
            or difference.get('hard_labels_touched') is not False
            or difference.get('hard_foreground_pixels') != 0
            or difference.get('hard_background_pixels') != 0):
        raise ValueError('insufficient or conflicting silhouette region')
    for key in ('original', 'counterfactual'):
        hypothesis = measurement[key]
        retained = float(hypothesis['hard_foreground_retained_fraction'])
        if (hypothesis.get('core_component_unambiguous') is not True
                or type(hypothesis.get('hard_background_included_pixels')) is not int
                or hypothesis['hard_background_included_pixels'] != 0
                or not math.isfinite(retained) or not .85 <= retained <= 1.):
            raise ValueError('unresolved silhouette hard labels')
    _geometry(evidence)
    return deepcopy(evidence)


def silhouette_view_gain(evidence, T_base_camera, depth_uncertainty_m):
    """Predict extra edge exposure, with the original path checks still required."""
    failure = dict(ok=False, code='SILHOUETTE_VIEW_GAIN_INSUFFICIENT',
                   contact_or_motion_authorized=False, measured_registration_required=True,
                   diagnostic_observation_only=True)
    try:
        data, center, anchor, normal, origin, initial, outward, height, frozen_noise = _geometry(evidence)
        transform = np.asarray(T_base_camera, dtype=float)
        requested_noise = float(depth_uncertainty_m)
        if not math.isfinite(requested_noise) or requested_noise < 0.:
            raise ValueError('invalid depth uncertainty')
        noise = max(requested_noise, frozen_noise)
        if (transform.shape != (4, 4) or not np.all(np.isfinite(transform))
                or not np.allclose(transform[3], [0., 0., 0., 1.], atol=1e-8, rtol=0.)
                or not np.allclose(transform[:3,:3].T@transform[:3,:3], np.eye(3), atol=1e-6, rtol=0.)
                or abs(np.linalg.det(transform[:3,:3])-1.) > 1e-6
                or not math.isfinite(noise) or noise < 0.):
            raise ValueError('invalid proposed camera')
        camera = transform[:3, 3]
        ray = camera-center
        if np.linalg.norm(ray) <= 1e-9:
            raise ValueError('camera at target')
        ray /= np.linalg.norm(ray)
        previous = height*max(0., float(outward@initial))
        proposed = height*max(0., float(outward@ray))
        gain = proposed-previous
        baseline = float(np.linalg.norm(camera-origin))
        ok = (gain > noise+1e-6 and baseline >= .015
              and float((camera-anchor)@outward) > noise
              and float(camera@normal+data['support_offset_m']) > 0.)
        return dict(failure, ok=bool(ok), code=('SILHOUETTE_VIEW_POTENTIAL_GAIN' if ok else failure['code']),
            predicted_edge_exposure_m=proposed, initial_edge_exposure_m=previous,
            predicted_gain_m=gain, depth_uncertainty_m=noise,
            camera_translation_baseline_m=baseline, ambiguity_outward_axis_base=outward.tolist(),
            source_stamp_ns=evidence['source_stamp_ns'],
            target_epoch=evidence['target_epoch'], target_track_id=evidence['target_track_id'],
            prediction_only=True, physical_contour_certified=False)
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
        return dict(failure, code='SILHOUETTE_VIEW_GAIN_INPUT_INVALID')


def directed_silhouette_rotations(evidence, camera_rotation, support_normal,
                                  depth_uncertainty_m, minimum_camera_distance_m,
                                  maximum_rotation_deg=45., maximum_directions=2):
    """At most two rotations towards the measured disputed edge, no robot I/O."""
    try:
        data, center, anchor, normal, origin, initial, outward, height, frozen_noise = _geometry(evidence)
        rotation = np.asarray(camera_rotation, dtype=float)
        normal_input = _vector(support_normal, 'normal')
        requested_noise = float(depth_uncertainty_m)
        if not math.isfinite(requested_noise) or requested_noise < 0.:
            raise ValueError('invalid depth uncertainty')
        noise = max(requested_noise, frozen_noise)
        distance, maximum = float(minimum_camera_distance_m), float(maximum_rotation_deg)
        if (rotation.shape != (3,3) or not np.all(np.isfinite(rotation))
                or not np.allclose(rotation.T@rotation, np.eye(3), atol=1e-6, rtol=0.)
                or abs(np.linalg.det(rotation)-1.) > 1e-6
                or not np.allclose(normal_input, normal, atol=1e-6, rtol=0.)
                or not all(math.isfinite(v) for v in (noise, distance, maximum))
                or noise < 0. or distance <= 0. or not 0. < maximum <= 45.
                or type(maximum_directions) is not int or not 1 <= maximum_directions <= 2):
            return ()
        optical, wanted = rotation[:,0], -outward
        angle_to_edge = math.acos(float(np.clip(optical@wanted, -1., 1.)))
        required = max(max(0., float(outward@initial))+(noise+2e-6)/height,
                       (float((anchor-center)@outward)+noise+2e-6)/distance)
        if required >= 1.:
            return ()
        minimum = max(0., angle_to_edge-math.acos(required))
        upper = min(math.radians(maximum), angle_to_edge)
        if minimum > upper+1e-12:
            return ()
        axis = np.cross(optical, wanted)
        if np.linalg.norm(axis) <= 1e-9:
            return ()
        axis /= np.linalg.norm(axis)
        x,y,z = axis
        skew = np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
        result = []
        for angle in sorted(set((minimum, upper))):
            if angle <= 1e-9:
                continue
            turn = np.eye(3)+math.sin(angle)*skew+(1.-math.cos(angle))*(skew@skew)
            proposed = turn@rotation
            if float(proposed[:,0]@normal) >= -1e-9:
                continue
            transform = np.eye(4); transform[:3,:3] = proposed
            transform[:3,3] = center-distance*proposed[:,0]
            gain = silhouette_view_gain(evidence, transform, noise)
            if not gain['ok']:
                continue
            result.append((proposed, dict(gain,
                orientation_source='measured_silhouette_ambiguity_geodesic',
                direction_source='same_source_disputed_boundary',
                camera_rotation_offset_deg=math.degrees(angle),
                minimum_required_rotation_deg=math.degrees(minimum),
                maximum_rotation_deg=maximum, maximum_directions=maximum_directions,
                direction_family_count=1,
                maximum_candidates_per_direction=2, minimum_camera_distance_m=distance,
                numeric_reserve_m=2e-6, measured_target_geometry_direction_used=True,
                contact_candidate_yaw_used=False, observation_only=True,
                global_path_optimality_claimed=False)))
        return tuple(result[:maximum_directions])
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
        return ()
