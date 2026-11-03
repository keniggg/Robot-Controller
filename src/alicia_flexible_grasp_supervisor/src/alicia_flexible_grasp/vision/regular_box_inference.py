"""Bounded regular-prism inference from calibrated top and current RGB evidence.

This is an explicitly assumed box model, never a measured side-surface cloud.
Metric depth, the real instance mask and a verified support plane qualify the
assumption.  Uncertainties are empirical budgets, not certified error bounds.
This module neither requests a view nor authorizes motion.
"""
from dataclasses import dataclass, replace
from hashlib import sha256
import json
import math
from numbers import Integral, Real
from types import MappingProxyType, SimpleNamespace

import cv2
import numpy as np

from .object_geometry import GeometryEstimate, deproject_depth, _validated_transform
from .rgbd_snapshot import retained_exact_source
from .target_observation import TargetTrackIdentity, validate_target_identity
from .outer_rgb_rectangle import infer_outer_rgb_rectangle, validate_outer_rgb_interval
from .cuboid_rgb_interval import (infer_cuboid_rgb_interval, validate_cuboid_rgb_interval,
    validate_cuboid_rgb_interval_source)


KIND = 'regular_box_inferred_from_complete_top'
ASSUMPTION = 'orthogonal_vertical_prism_on_verified_support'
_FIELDS = frozenset((
    'schema_version', 'kind', 'assumption', 'center_base', 'axes_base',
    'size_xyz_m', 'empirical_edge_uncertainty_m', 'empirical_height_uncertainty_m',
    'empirical_yaw_uncertainty_rad', 'support_normal_base', 'support_offset_m',
    'source_center_base', 'source_axes_base', 'source_support_normal_base',
    'source_support_offset_m', 'registration_transform', 'source',
    'qualification', 'model_sha256'))
_SOURCE_FIELDS = frozenset(('target_identity', 'stamp_ns', 'depth_sha256', 'color_sha256',
    'mask_sha256', 'intrinsics_sha256', 'T_base_camera_sha256', 'geometry_sha256'))
_QUALIFICATION_FIELDS = frozenset(('top_plane_rms_m', 'top_support_angle_deg',
    'rectangle_fill_ratio', 'corner_coverage', 'edge_coverage',
    'outside_top_continuation_fraction', 'outside_valid_depth_fraction',
    'valid_depth_fraction', 'edge_clearance_px', 'top_point_count',
    'top_occupancy_fraction', 'maximum_internal_hole_fraction',
    'camera_top_cosine', 'metric_pixel_size_m', 'edge_fit_rms_m',
    'maximum_observed_model_excess_m', 'uncertainty_semantics', 'contour_source',
    'unobserved_boundary_extension_m'))


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'),
                      allow_nan=False, ensure_ascii=True)


def _hash(value):
    return sha256(_canonical(value).encode('utf8')).hexdigest()


def _array_hash(value):
    a = np.ascontiguousarray(value)
    return sha256(a.tobytes()).hexdigest()


def _number(value, name, minimum=None, maximum=None):
    if (isinstance(value, (bool, np.bool_)) or not isinstance(value, Real)
            or not math.isfinite(value)):
        raise ValueError(name + ' must be finite numeric')
    x = float(value)
    if (minimum is not None and x < minimum) or (maximum is not None and x > maximum):
        raise ValueError(name + ' outside inference limits')
    return x


def _array(value, shape, name):
    a = np.asarray(value)
    if a.shape != shape or a.dtype.kind not in 'iuf' or not np.all(np.isfinite(a)):
        raise ValueError(name + ' must contain finite numeric values')
    return np.asarray(a, dtype=float)


def _rotation(value, name):
    a = _array(value, (3, 3), name)
    if not np.allclose(a.T @ a, np.eye(3), rtol=0., atol=1e-7) or abs(np.linalg.det(a)-1.) > 1e-7:
        raise ValueError(name + ' must be a right handed rotation')
    return a


def _unit(value, name):
    a = _array(value, (3,), name)
    if not np.isclose(np.linalg.norm(a), 1., rtol=0., atol=1e-7):
        raise ValueError(name + ' must be unit length')
    return a


def validate_box_model_evidence(value):
    """Validate and independently own the canonical, source-bound model JSON.

    A hash authenticates consistency, not measurements. The caller must bind
    this object to its already admitted source frame and signed plan lineage.
    """
    if not isinstance(value, dict) or set(value) != _FIELDS:
        raise ValueError('regular box model field set differs')
    d = json.loads(_canonical(value))
    if type(d['schema_version']) is not int or d['schema_version'] not in (1, 2, 3) or d['kind'] != KIND or d['assumption'] != ASSUMPTION:
        raise ValueError('regular box model version/kind/assumption differs')
    claimed = d['model_sha256']
    body = {k: v for k, v in d.items() if k != 'model_sha256'}
    if not isinstance(claimed, str) or claimed != _hash(body):
        raise ValueError('regular box model digest differs')
    c = _array(d['center_base'], (3,), 'center')
    a = _rotation(d['axes_base'], 'axes')
    size = _array(d['size_xyz_m'], (3,), 'size')
    if np.any(size < .005) or np.any(size > .6) or size[2] > .5:
        raise ValueError('box dimensions outside inference limits')
    n = _unit(d['support_normal_base'], 'support normal')
    offset = _number(d['support_offset_m'], 'support offset')
    if not np.allclose(a[:, 2], n, rtol=0., atol=1e-7) or abs(c @ n + offset - size[2]/2.) > 1e-7:
        raise ValueError('box not resting on its verified support')
    _number(d['empirical_edge_uncertainty_m'], 'edge uncertainty', .001, .01)
    hu = _number(d['empirical_height_uncertainty_m'], 'height uncertainty', .001, .01)
    _number(d['empirical_yaw_uncertainty_rad'], 'yaw uncertainty', math.radians(.25), math.radians(12.))
    if size[2] <= 2.*hu + .002:
        raise ValueError('box has no contact height inside empirical uncertainty')
    sc = _array(d['source_center_base'], (3,), 'source center')
    sa = _rotation(d['source_axes_base'], 'source axes')
    sn = _unit(d['source_support_normal_base'], 'source normal')
    sd = _number(d['source_support_offset_m'], 'source offset')
    T = _validated_transform(d['registration_transform'])
    if (not np.allclose(c, T[:3, :3] @ sc + T[:3, 3], rtol=0., atol=1e-8)
            or not np.allclose(a, T[:3, :3] @ sa, rtol=0., atol=1e-8)
            or not np.allclose(n, T[:3, :3] @ sn, rtol=0., atol=1e-8)
            or abs(offset - (sd - n @ T[:3, 3])) > 1e-8):
        raise ValueError('registered model differs from original source model')
    source = d['source']
    if not isinstance(source, dict) or set(source) != _SOURCE_FIELDS:
        raise ValueError('model source fields differ')
    identity = source['target_identity']
    if not isinstance(identity, dict) or set(identity) != {'epoch', 'track_id'}:
        raise ValueError('model target identity invalid')
    TargetTrackIdentity(**identity)
    if type(source['stamp_ns']) is not int or source['stamp_ns'] <= 0:
        raise ValueError('model source stamp invalid')
    for key in _SOURCE_FIELDS - {'stamp_ns', 'target_identity'}:
        s = source[key]
        if key == 'color_sha256' and s is None:
            continue
        if not isinstance(s, str) or len(s) != 64 or any(c not in '0123456789abcdef' for c in s):
            raise ValueError('model source digest invalid')
    q = d['qualification']
    is_outer = d['schema_version'] == 2
    is_cuboid = d['schema_version'] == 3
    extra_fields = ({'outer_rgb_interval', 'top_plane_vertical_deviation_m'} if is_outer else
        {'cuboid_rgb_interval', 'top_plane_vertical_deviation_m'} if is_cuboid else set())
    if not isinstance(q, dict) or set(q) != _QUALIFICATION_FIELDS | extra_fields:
        raise ValueError('model qualification fields differ')
    visual_contour = q.get('contour_source') in ('rgb_edges_on_measured_top_plane', 'rgb_outer_joint_interval_on_measured_top_plane',
        'cuboid_rgb_joint_interval_on_measured_support')
    if is_outer != (q.get('contour_source') == 'rgb_outer_joint_interval_on_measured_top_plane'):
        raise ValueError('RGB interval schema/qualification differs')
    if is_cuboid != (q.get('contour_source') == 'cuboid_rgb_joint_interval_on_measured_support'):
        raise ValueError('cuboid interval schema/qualification differs')
    if is_cuboid:
        interval = q['cuboid_rgb_interval']
        validate_cuboid_rgb_interval(interval, size,
            d['empirical_edge_uncertainty_m'], d['empirical_yaw_uncertainty_rad'], hu)
        directional = _array(interval['empirical_axis_uncertainty_m'], (2,),
            'source cuboid directional position budget')
        if (np.any(directional < .001) or np.any(directional > .01)
                or abs(float(directional.max())-d['empirical_edge_uncertainty_m']) > 1e-9):
            raise ValueError('cuboid scalar and directional budgets differ')
        # The interval describes the immutable source geometry. Accepted rigid
        # registration changes only the current model, not this image proof.
        ia = _rotation(interval['nominal_axes_base'], 'source cuboid axes')
        lower = _array(interval['nominal_lower'], (2,), 'source cuboid lower')
        upper = _array(interval['nominal_upper'], (2,), 'source cuboid upper')
        inormal = _unit(interval['support_normal_base'], 'source cuboid support')
        ioffset = _number(interval['support_offset_m'], 'source cuboid support offset')
        icenter = ia[:, :2] @ ((lower+upper)/2.) + inormal*(size[2]/2.-ioffset)
        if (not np.allclose(ia, sa, rtol=0., atol=1e-8)
                or not np.allclose(icenter, sc, rtol=0., atol=1e-8)
                or not np.allclose(inormal, sn, rtol=0., atol=1e-8)
                or abs(ioffset-sd) > 1e-8):
            raise ValueError('cuboid interval differs from immutable source geometry')
        for name in ('color', 'depth', 'mask'):
            if interval['source_'+name+'_sha256'] != source[name+'_sha256']:
                raise ValueError('cuboid interval source pixels differ')
        optical = _validated_transform(interval['T_base_optical'])
        intrinsic = _array(interval['intrinsics'], (4,), 'source cuboid intrinsics')
        image_shape = interval['image_shape']
        if (not isinstance(image_shape, list) or len(image_shape) != 2 or
                any(type(v) is not int or v <= 0 for v in image_shape)):
            raise ValueError('cuboid source image dimensions invalid')
        scale = _number(interval['depth_scale_m'], 'source cuboid depth scale', 1e-7, 1.)
        intrinsic_hash = _hash(dict(shape=image_shape, K=intrinsic.tolist(), depth_scale_m=scale))
        if (interval['T_base_camera_sha256'] != source['T_base_camera_sha256']
                or _array_hash(optical) != source['T_base_camera_sha256']
                or interval['intrinsics_sha256'] != source['intrinsics_sha256']
                or intrinsic_hash != source['intrinsics_sha256']):
            raise ValueError('cuboid calibration differs from immutable source')
        if (abs(interval['top_plane_rms_m']-q['top_plane_rms_m']) > 1e-12
                or abs(interval['metric_pixel_size_m']-q['metric_pixel_size_m']) > 1e-12
                or abs(interval['top_plane_vertical_deviation_m']-q['top_plane_vertical_deviation_m']) > 1e-9):
            raise ValueError('cuboid qualification differs from interval evidence')
        top_normal = _unit(interval['measured_top_normal_base'], 'source cuboid top normal')
        if abs(math.degrees(math.acos(float(np.clip(top_normal @ inormal, -1., 1.))))
                -q['top_support_angle_deg']) > 1e-7:
            raise ValueError('cuboid measured top angle differs from qualification')
    if is_outer or is_cuboid:
        interval = q['outer_rgb_interval'] if is_outer else q['cuboid_rgb_interval']
        if is_outer:
            validate_outer_rgb_interval(interval, size[:2], d['empirical_edge_uncertainty_m'], d['empirical_yaw_uncertainty_rad'])
        vertical = _number(q['top_plane_vertical_deviation_m'], 'top plane vertical deviation', 0., .01)
        required_height = vertical+interval['top_plane_slab_m']/math.cos(math.radians(_number(q['top_support_angle_deg'], 'top/support angle', 0., 6.)))
        if hu+1e-12 < max(.001, required_height):
            raise ValueError('observed top tilt and plane slab omitted from height uncertainty')
    for key, lo, hi in (
            ('top_plane_rms_m', 0., .0015), ('top_support_angle_deg', 0., 6.),
            ('rectangle_fill_ratio', .88, 1.000001), ('valid_depth_fraction', .60, 1.),
            ('top_occupancy_fraction', .65 if visual_contour else .88, 1.), ('maximum_internal_hole_fraction', 0., .02),
            ('camera_top_cosine', .35, 1.000001), ('metric_pixel_size_m', 0., .003),
            ('edge_fit_rms_m', 0., .005), ('maximum_observed_model_excess_m', 0., .001),
            ('unobserved_boundary_extension_m', 0., .01)):
        _number(q[key], key, lo, hi)
    for key, lo, hi in (
            ('corner_coverage', 0. if is_outer or is_cuboid else .5, 1.), ('edge_coverage', 0. if is_outer or is_cuboid else .7, 1.),
            ('outside_top_continuation_fraction', 0., 1. if visual_contour else .30),
            ('outside_valid_depth_fraction', 0. if visual_contour else .60, 1.)):
        if not isinstance(q[key], list) or len(q[key]) != 4:
            raise ValueError('four independent top edges/corners required')
        for item in q[key]:
            _number(item, key, lo, hi)
    if type(q['edge_clearance_px']) is not int or q['edge_clearance_px'] < 4 or type(q['top_point_count']) is not int or q['top_point_count'] < 80:
        raise ValueError('complete top image support unavailable')
    if q['uncertainty_semantics'] != 'empirical_budget_not_certified_error_bound':
        raise ValueError('uncertainty provenance missing')
    if q['contour_source'] not in ('measured_depth_top', 'rgb_edges_on_measured_top_plane', 'rgb_outer_joint_interval_on_measured_top_plane',
        'cuboid_rgb_joint_interval_on_measured_support'):
        raise ValueError('top contour source invalid')
    if visual_contour and source['color_sha256'] is None:
        raise ValueError('visual contour missing original image digest')
    if visual_contour and any(
            valid >= .20 and continuation > .30 for valid,continuation in zip(
                q['outside_valid_depth_fraction'],q['outside_top_continuation_fraction'])):
        raise ValueError('visual top continues into valid external measured depth')
    if d['empirical_edge_uncertainty_m'] < q['unobserved_boundary_extension_m'] + .001 - 1e-12:
        raise ValueError('unobserved contour band omitted from empirical uncertainty')
    return d


@dataclass(frozen=True)
class RegularBoxInference:
    """Immutable JSON-backed model; side faces remain explicitly inferred."""
    _serialized: str

    def __post_init__(self):
        checked = validate_box_model_evidence(json.loads(self._serialized))
        serialized = _canonical(checked)
        object.__setattr__(self, '_serialized', serialized)
        # Keep only immutable strings outside the dataclass value/schema. A
        # field read need not parse the complete source/qualification record.
        # Decode the selected field on every read to retain owned containers.
        object.__setattr__(self, '_attribute_json_cache', (serialized, tuple(
            (key, _canonical(value)) for key, value in checked.items())))

    @classmethod
    def from_evidence_dict(cls, value):
        return cls(_canonical(value))

    def to_evidence_dict(self):
        return json.loads(self._serialized)

    def __getattr__(self, key):
        # Older serialized instances can lack the optional cache. Also bind
        # it to the actual immutable backing string, rather than trusting a
        # detached cache after an explicit low-level reconstruction.
        try:
            cache = object.__getattribute__(self, '_attribute_json_cache')
        except AttributeError:
            cache = None
        serialized = object.__getattribute__(self, '_serialized')
        if cache is not None and cache[0] is serialized:
            for field, encoded in cache[1]:
                if field == key:
                    value = json.loads(encoded)
                    break
            else:
                raise AttributeError(key)
        else:
            d = self.to_evidence_dict()
            if key not in d:
                raise AttributeError(key)
            value = d[key]
        if key in ('center_base', 'axes_base', 'size_xyz_m', 'support_normal_base', 'registration_transform'):
            a = np.asarray(value, dtype=float)
            return np.frombuffer(a.tobytes(), dtype=float).reshape(a.shape)
        return value


@dataclass(frozen=True)
class BoxInferenceResult:
    ok: bool
    code: str
    model: object
    audit: object


@dataclass(frozen=True)
class BoxMeasuredConsistency:
    """An owned, read-only check of measured points, never model authority."""
    ok: bool
    code: str
    maximum_observed_model_excess_m: float
    _conflict_json: str

    @property
    def conflict(self):
        return json.loads(self._conflict_json)


def _measured_family_excess(measured, center, axes, size, edge_axes,
                            height_uncertainty, yaw_uncertainty, full_family):
    """The qualifier's original extent, excess and first-witness arithmetic."""
    measured = np.asarray(measured)
    if measured.ndim != 2 or measured.shape[1] != 3 or not len(measured) or not np.all(np.isfinite(measured)):
        raise ValueError('verified measured geometry missing')
    local = (measured-center) @ axes
    measured_half = size/2.
    if full_family:
        # The same complete yaw family is already used for opening and
        # collision. A measured corner inside it is not a contradiction.
        measured_half = measured_half.copy()
        for axis in range(2):
            turn = min(yaw_uncertainty, math.atan2(size[1-axis], size[axis]))
            measured_half[axis] = (size[axis]*math.cos(turn)+size[1-axis]*math.sin(turn))/2.
    point_excess = np.abs(local)-measured_half-np.r_[edge_axes, height_uncertainty]
    excess = float(max(0., np.max(point_excess)))
    conflict = None
    if excess > .001:
        point_index, axis_index = np.unravel_index(np.argmax(point_excess), point_excess.shape)
        conflict = dict(
            measured_points_sha256=_array_hash(measured),
            measured_point_count=len(measured), point_index=int(point_index),
            axis_index=int(axis_index), point_base_m=measured[point_index].tolist(),
            point_local_m=local[point_index].tolist(),
            full_family_half_extent_m=(measured_half+np.r_[edge_axes, height_uncertainty]).tolist(),
            excess_m=excess, permitted_observed_excess_m=.001,
            point_view_provenance='not_available_in_GeometryEstimate',
            execution_authorized=False)
    return excess, conflict


def evaluate_box_measured_consistency(model, points):
    """Check every supplied point against the model's original measured gate.

    The caller supplies a complete measured union in the model's coordinates.
    This does not qualify pixels, prove registration, change measurements, or
    make this source model a final fused planning model.
    """
    if not isinstance(model, RegularBoxInference):
        raise ValueError('qualified RegularBoxInference required')
    evidence = validate_box_model_evidence(model.to_evidence_dict())
    measured = np.asarray(points)
    if measured.dtype.kind not in 'iuf':
        raise ValueError('verified measured geometry missing')
    edge_axes = inferred_box_axis_uncertainty(model)
    excess, conflict = _measured_family_excess(measured,
        np.asarray(evidence['center_base']), np.asarray(evidence['axes_base']),
        np.asarray(evidence['size_xyz_m']), edge_axes,
        evidence['empirical_height_uncertainty_m'],
        evidence['empirical_yaw_uncertainty_rad'], evidence['schema_version'] in (2, 3))
    return BoxMeasuredConsistency(conflict is None,
        'BOX_MEASURED_GEOMETRY_CONSISTENT' if conflict is None else 'BOX_MEASURED_GEOMETRY_CONFLICT',
        excess, _canonical(conflict))


def _box_geometry_source_payload(geometry):
    return dict(center=np.asarray(geometry.center_base).tolist(), axes=np.asarray(geometry.axes_base).tolist(),
        size=np.asarray(geometry.size_xyz_m).tolist(), support_normal=np.asarray(geometry.support_normal_base).tolist(),
        support_offset=float(geometry.support_offset_m), support_inlier_ratio=float(geometry.support_inlier_ratio),
        measured_points_sha256=_array_hash(np.asarray(geometry.object_points_base)))


def regular_box_geometry_sha256(geometry):
    """The unchanged geometry fields in an inferred model's source binding."""
    if not isinstance(geometry, GeometryEstimate) or geometry.ok is not True:
        raise ValueError('valid original GeometryEstimate required')
    return _hash(_box_geometry_source_payload(geometry))


def transform_inferred_box(model, T_reference_source):
    if not isinstance(model, RegularBoxInference):
        raise ValueError('qualified RegularBoxInference required')
    T = _validated_transform(T_reference_source)
    d = model.to_evidence_dict()
    for name in ('center_base',):
        d[name] = (T[:3, :3] @ np.asarray(d[name]) + T[:3, 3]).tolist()
    d['axes_base'] = (T[:3, :3] @ np.asarray(d['axes_base'])).tolist()
    normal = T[:3, :3] @ np.asarray(d['support_normal_base'])
    d['support_normal_base'] = normal.tolist()
    d['support_offset_m'] -= float(normal @ T[:3, 3])
    d['registration_transform'] = (T @ np.asarray(d['registration_transform'])).tolist()
    d['model_sha256'] = _hash({k: v for k, v in d.items() if k != 'model_sha256'})
    return RegularBoxInference.from_evidence_dict(d)


def enclose_inferred_box_geometry(geometry, model):
    """Enclose a source-bound complete box family and every measured point.

    This qualified planning representation keeps the original geometry and
    measured point order as source evidence. Its old nominal OBB corners are
    not measurements or an additional unseen-occupancy budget. Generic
    measured-only fusion retains its separate outward-only OBB contract.
    All-point containment is verified here at the producer; the present wire
    consumer independently checks the frozen planning OBB and complete family,
    but does not receive the complete source points.
    """
    if not isinstance(geometry, GeometryEstimate) or geometry.ok is not True or not isinstance(model, RegularBoxInference):
        raise ValueError('valid original geometry and qualified inferred box required')
    evidence = validate_box_model_evidence(model.to_evidence_dict())
    if not np.array_equal(np.asarray(evidence['registration_transform']), np.eye(4)):
        raise ValueError('planning enclosure requires the original source-frame model')
    source_digest = regular_box_geometry_sha256(geometry)
    if source_digest != evidence['source']['geometry_sha256']:
        raise ValueError('planning enclosure original source geometry digest differs')
    if geometry.source_mode != 'instance_mask' or geometry.failure_code or geometry.failure_reason:
        raise ValueError('planning enclosure requires admitted original geometry')
    n = model.support_normal_base
    if not np.array_equal(geometry.support_normal_base, n) or geometry.support_offset_m != model.support_offset_m:
        raise ValueError('collision enclosure and inferred support differ')
    old_c = _array(geometry.center_base, (3,), 'original center')
    old_A = _rotation(geometry.axes_base, 'original axes')
    old_s = _array(geometry.size_xyz_m, (3,), 'original size')
    if np.any(old_s <= 0.) or np.any(old_s > .6) or old_s[2] > .5:
        raise ValueError('original geometry dimensions outside physical limits')
    measured = np.asarray(geometry.object_points_base)
    measured_digest = _array_hash(measured)
    consistency = evaluate_box_measured_consistency(model, measured)
    if not consistency.ok:
        raise ValueError('planning enclosure measured geometry conflicts with qualified family')
    A = model.axes_base
    inferred = inferred_box_enclosure_vertices(model)
    enclosed = np.vstack((measured, inferred))
    # Use all exact extrema, with no percentile fitting, trimming or padding
    # reduction. A nearby origin avoids unnecessary base-coordinate error.
    origin = model.center_base
    local = (enclosed-origin) @ A
    lo, hi = local.min(axis=0), local.max(axis=0)
    center = origin+A @ ((lo+hi)/2.)
    size = hi-lo
    if (not np.all(np.isfinite(center)) or not np.all(np.isfinite(size))
            or np.any(size <= 0.) or np.any(size > .6) or size[2] > .5):
        raise ValueError('planning enclosure dimensions outside physical limits')
    residual = np.abs((enclosed-center) @ A)
    if not np.all(np.isfinite(residual)) or np.any(residual > size/2.+1e-12):
        raise ValueError('planning enclosure point or family containment lost')
    updated = replace(geometry, center_base=center, axes_base=A, size_xyz_m=size)
    if (not np.array_equal(updated.object_points_base, measured)
            or _array_hash(updated.object_points_base) != measured_digest
            or regular_box_geometry_sha256(geometry) != source_digest):
        raise ValueError('planning enclosure must not change measured points')
    return updated, dict(schema_version=1,
        policy='source_bound_measured_points_and_inferred_box_uncertainty',
        model_sha256=model.model_sha256, source_geometry_sha256=source_digest,
        source_stamp_ns=evidence['source']['stamp_ns'],
        source_target_identity=dict(evidence['source']['target_identity']),
        coordinate_contract='qualified_original_source_frame_registration_identity',
        measured_points_sha256=measured_digest, measured_points_count=len(measured),
        measured_points_unchanged=True, measured_point_order_unchanged=True,
        producer_verified_all_points=True, all_measured_points_contained=True,
        full_family_contained=True, full_family_vertex_count=len(inferred),
        maximum_observed_model_excess_m=consistency.maximum_observed_model_excess_m,
        permitted_observed_excess_m=.001, original_obb_used_as_occupancy=False,
        consumer_independent_measured_point_verification=False,
        measured_side_surfaces=False, synthetic_measured_points_added=0,
        contact_or_motion_authorized=False, original_center_base=old_c.tolist(),
        original_axes_base=old_A.tolist(), original_size_xyz_m=old_s.tolist(),
        planning_center_base=updated.center_base.tolist(), planning_axes_base=updated.axes_base.tolist(),
        planning_size_xyz_m=updated.size_xyz_m.tolist(),
        uncertainty_semantics='empirical_budget_not_certified_error_bound')


def inferred_box_enclosure_vertices(model):
    """Eight conservative collision vertices, including empirical budgets."""
    from itertools import product
    if not isinstance(model, RegularBoxInference):
        raise ValueError('qualified RegularBoxInference required')
    A=model.axes_base;size=model.size_xyz_m;n=model.support_normal_base
    yaw=model.empirical_yaw_uncertainty_rad
    # Include stationary projection extrema, not just interval endpoints.
    spread=[]
    edge_axes = inferred_box_axis_uncertainty(model)
    for index,(x,y) in enumerate(((size[0],size[1]),(size[1],size[0]))):
        angle=min(float(yaw),math.atan2(float(y),float(x)))
        spread.append(float(x*math.cos(angle)+y*math.sin(angle)+2.*edge_axes[index]))
    half=np.asarray([spread[0]/2.,spread[1]/2.,(size[2]+model.empirical_height_uncertainty_m)/2.])
    expanded_center=model.center_base+n*model.empirical_height_uncertainty_m/2.
    vertices=np.asarray([expanded_center+A@(np.asarray(sign)*half) for sign in product((-1.,1.),repeat=3)])
    return np.frombuffer(vertices.tobytes(),dtype=float).reshape((8,3))


def inferred_box_axis_uncertainty(model):
    """The same source-frame position family for opening and collision.

    V1/V2 retain their original common scalar allowance. V3 stores its two
    independently derived fixed-axis bounds in the validated interval; their
    maximum remains the legacy scalar for conservative generic diagnostics.
    Accepted rigid registration rotates axes with the model and preserves
    these two bounds, so they are never reinterpreted as base-frame errors.
    """
    if not isinstance(model, RegularBoxInference):
        raise ValueError('qualified RegularBoxInference required')
    edge = float(model.empirical_edge_uncertainty_m)
    if model.schema_version != 3:
        return np.array([edge, edge])
    values = _array(model.qualification['cuboid_rgb_interval'][
        'empirical_axis_uncertainty_m'], (2,), 'cuboid fixed-axis uncertainty')
    if np.any(values < .001) or np.any(values > .01) or abs(float(values.max())-edge) > 1e-9:
        raise ValueError('cuboid directional and common position budgets differ')
    return values


def _failure(code, reason, audit):
    return BoxInferenceResult(False, code, None,
        MappingProxyType(dict(audit, reason=reason, measured_side_surfaces=False)))


def _basis(normal):
    reference = np.eye(3)[int(np.argmin(np.abs(normal)))]
    u = np.cross(reference, normal)
    u /= np.linalg.norm(u)
    return np.column_stack((u, np.cross(normal, u)))


def _project(points, T, K):
    pc = (np.asarray(points)-T[:3, 3]) @ T[:3, :3]
    if np.any(pc[:, 2] <= 0.):
        raise ValueError('box top behind camera')
    fx, fy, cx, cy = K
    return np.column_stack((pc[:, 0]*fx/pc[:, 2]+cx, pc[:, 1]*fy/pc[:, 2]+cy))


def _visual_rectangle(color, mask, depth, scale, T, K, origin, tn,
                      axes, lower, upper, pixel_size):
    """Bounded RGB edge intersections supported by the measured top plane.

    An object silhouette can include a visible side. Each proposed boundary
    therefore needs top-plane depth immediately inside and non-top depth
    outside. No filled silhouette or fitted vertices become measured points.
    """
    from itertools import product
    if color is None:
        return None
    if not isinstance(color, np.ndarray) or color.dtype != np.uint8 or color.shape != depth.shape+(3,):
        raise ValueError('same-frame BGR image must match aligned depth')
    edges = cv2.Canny(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY), 50., 100.)
    edges[cv2.dilate(mask.astype(np.uint8), np.ones((5,5), np.uint8)) == 0] = 0
    ys, xs = np.nonzero(mask)
    min_length = max(12, int(.35*min(np.ptp(xs), np.ptp(ys))))
    found = cv2.HoughLinesP(edges, 1., np.pi/180., max(12, min_length//2),
                            minLineLength=min_length, maxLineGap=4)
    if found is None:
        return None
    lines = sorted(found[:,0].tolist(), key=lambda row: (-math.hypot(row[2]-row[0],row[3]-row[1]), tuple(row)))[:48]
    camera_center = T[:3,3]
    def on_plane(pixels):
        ray = np.column_stack(((pixels[:,0]-K[2])/K[0], (pixels[:,1]-K[3])/K[1], np.ones(len(pixels)))) @ T[:3,:3].T
        denominator = ray @ tn
        distance = float((origin-camera_center) @ tn) / denominator
        if np.any(~np.isfinite(distance)) or np.any(distance <= 0.):
            raise ValueError('visual edge does not intersect visible measured top')
        return camera_center + ray*distance[:,None]
    # Four rectangles edges: lower-y, upper-x, upper-y, lower-x.
    limits = ((1,lower[1]), (0,upper[0]), (1,upper[1]), (0,lower[0]))
    choices = [[] for _ in range(4)]
    for line in lines:
        pixels = np.asarray(line,float).reshape(2,2)
        metric = on_plane(pixels) @ axes[:,:2]
        delta = metric[1]-metric[0]
        for index,(across,limit) in enumerate(limits):
            along = 1-across
            if abs(delta[along]) < .35*(upper[along]-lower[along]):
                continue
            if abs(delta[across]) > math.tan(math.radians(8.))*abs(delta[along]):
                continue
            offset = float(np.mean(metric[:,across]))
            # Candidate-search radius only; each proposed edge still needs
            # independent inward/outward measured depth support below.
            if abs(offset-limit) > max(.012,8.*pixel_size):
                continue
            overlap = min(float(np.max(metric[:,along])),upper[along])-max(float(np.min(metric[:,along])),lower[along])
            if overlap < .35*(upper[along]-lower[along]):
                continue
            # Keep distinct actual lines, not repeated Hough fragments of the
            # same edge. Larger overlap wins within one pixel-scale cluster.
            row=(offset,float(overlap),pixels,abs(float(delta[across]/delta[along])))
            old=next((j for j,v in enumerate(choices[index]) if abs(v[0]-offset)<pixel_size),None)
            if old is None:
                choices[index].append(row)
            elif overlap > choices[index][old][1]:
                choices[index][old]=row
    if any(not values for values in choices):
        return None
    # These are rays through actual mask pixels, intersected with a measured
    # plane, not new depth points. Their envelope bounds any unresolved strip
    # beyond an RGB line (for example a printed inner line in a depth hole).
    silhouette=on_plane(np.column_stack((xs,ys)).astype(float))@axes[:,:2]
    silhouette_lo,silhouette_hi=silhouette.min(axis=0),silhouette.max(axis=0)
    edge_pixels=cv2.dilate(edges,np.ones((5,5),np.uint8))>0
    for index in range(4):
        choices[index].sort(key=lambda row:(abs(row[0]-limits[index][1]),-row[1]))
        choices[index]=choices[index][:3]
    candidates=[]
    for selected in product(*choices):
        lo=np.array([selected[3][0],selected[0][0]])
        hi=np.array([selected[1][0],selected[2][0]])
        if np.any(hi-lo < .005):
            continue
        corners2=np.array([[lo[0],lo[1]],[hi[0],lo[1]],[hi[0],hi[1]],[lo[0],hi[1]]])
        tangent=corners2 @ axes[:,:2].T
        base=tangent+np.outer(((origin-tangent)@tn)/(tn@axes[:,2]),axes[:,2])
        projected=_project(base,T,K)
        center=np.mean(projected,axis=0)
        coverage=[]; corner_cov=[]; good_candidate=True; mask_boundary_count=0;unobserved_extension=0.
        for corner in projected:
            x,y=np.rint(corner).astype(int)
            if min(x,y,depth.shape[1]-1-x,depth.shape[0]-1-y)<4:
                good_candidate=False;break
            patch=mask[y-2:y+3,x-2:x+3]
            # A true convex rectangle corner occupies one quadrant of its
            # centered patch; expecting half/full occupancy rejects the very
            # corners this visual branch is intended to measure.
            corner_cov.append(float(min(1.,np.mean(patch)/.25)))
        if not good_candidate or min(corner_cov)<.5:
            continue
        for i in range(4):
            a,b=projected[i],projected[(i+1)%4];v=b-a
            outward=np.array([v[1],-v[0]])/np.linalg.norm(v)
            if outward@((a+b)/2.-center)<0.:outward*=-1.
            values=[]
            edge_samples=np.rint(a+np.outer(np.linspace(.08,.92,31),v)).astype(int)
            visual_support=float(np.mean(edge_pixels[edge_samples[:,1],edge_samples[:,0]]))
            outside_pixels=np.rint(a+np.outer(np.linspace(.12,.88,21),v)+3.*outward).astype(int)
            mask_border_fraction=float(np.mean(~mask[outside_pixels[:,1],outside_pixels[:,0]]))
            is_mask_boundary=mask_border_fraction>=.70
            mask_boundary_count+=int(is_mask_boundary)
            for direction in (-1.,1.):
                sample=np.rint(a+np.outer(np.linspace(.12,.88,21),v)+direction*3.*outward).astype(int)
                zz=depth[sample[:,1],sample[:,0]].astype(float)*scale
                valid=np.isfinite(zz)&(zz>0.)
                xyz=np.column_stack(((sample[:,0]-K[2])*zz/K[0],(sample[:,1]-K[3])*zz/K[1],zz))@T[:3,:3].T+T[:3,3]
                plane=(xyz-origin)@tn
                values.append((valid,np.abs(plane)<.0015,plane))
            inside,outside=values
            supported=float(np.mean(inside[0]&inside[1]))
            valid_inside=float(np.mean(inside[0]))
            valid_outside=float(np.mean(outside[0]))
            continued=float(np.count_nonzero(outside[0]&outside[1]))/max(1,int(np.count_nonzero(outside[0])))
            # Missing depth on a visually complete edge is not contradictory
            # depth. Positive same-plane extension and measured non-top inner
            # strips remain contradictions; RGB never overwrites either one.
            if (visual_support < .70 or
                    (valid_inside>=.60 and supported/max(valid_inside,1e-12)<.70) or
                    (valid_outside>=.20 and continued>.30) or
                    np.mean(outside[0]&(outside[2]>.0025))>.30):
                good_candidate=False;break
            if valid_outside < .60 or valid_inside < .60:
                # Inconclusive boundary depth cannot localize a hidden edge.
                # Include the entire measured-mask ray extent beyond the
                # selected line in the budget, not an arbitrary 1 mm credit.
                extension=(lo[1]-silhouette_lo[1],silhouette_hi[0]-hi[0],
                           silhouette_hi[1]-hi[1],lo[0]-silhouette_lo[0])[i]
                unobserved_extension=max(unobserved_extension,float(max(0.,extension)))
            coverage.append(visual_support)
        if good_candidate and mask_boundary_count>=3:
            candidates.append((float(np.prod(hi-lo)),lo,hi,corner_cov,coverage,
                               max(row[3] for row in selected),unobserved_extension))
    if not candidates:
        return None
    # The largest fully evidenced top avoids choosing a printed inner border.
    return max(candidates,key=lambda row:row[0])


def _retained_box_pixels(snapshot, source_identity, source_stamp_ns):
    """One unchanged raw-source contract for qualification and view candidates."""
    if getattr(snapshot, 'ok', None) is not True or snapshot.source_mode != 'instance_mask':
        raise ValueError('admitted instance-mask snapshot required')
    original = retained_exact_source(snapshot)
    if original is None:
        return None
    validate_target_identity(source_identity)
    if (isinstance(source_stamp_ns, (bool, np.bool_))
            or not isinstance(source_stamp_ns, Integral) or source_stamp_ns <= 0
            or original.stamp_ns != source_stamp_ns
            or original.target_identity != source_identity
            or original.target_epoch != source_identity.epoch):
        raise ValueError('retained original differs from the current target observation')
    if (not isinstance(original.frame_id, str) or not original.frame_id
            or original.frame_id != original.frame_id.strip()):
        raise ValueError('retained original camera frame is invalid')
    color, depth, mask = (np.asarray(original.color_bgr),
                         np.asarray(original.depth_raw), np.asarray(original.object_mask))
    if (depth.ndim != 2 or depth.dtype.kind not in 'iuf'
            or mask.shape != depth.shape or mask.dtype != np.uint8
            or not np.all(np.isin(mask, (0, 255))) or not np.any(mask)
            or color.dtype != np.uint8 or color.shape != depth.shape + (3,)):
        raise ValueError('retained original RGB/depth/binary mask arrays are invalid')
    mask_hashes = getattr(snapshot, 'sample_mask_sha256', ())
    if (not isinstance(mask_hashes, tuple) or not mask_hashes
            or len(mask_hashes) != len(snapshot.sample_stamp_ns)
            or mask_hashes[-1] != _array_hash(np.asarray(mask > 0, dtype=np.uint8))):
        raise ValueError('retained original mask does not match the final admitted sample hash')
    return original, color, depth, mask, mask_hashes


def qualify_regular_box_snapshot(snapshot, camera_intrinsics, depth_scale_m,
        T_base_camera, geometry, source_identity, source_stamp_ns):
    """Use one retained raw frame for pixels, retaining fused measured geometry.

    The temporal depth mask is deliberately unsuitable as an RGB silhouette.
    A missing or mismatched original never falls back to fused/newer pixels.
    Geometry and calibration remain the unchanged admitted request inputs;
    the existing model separately binds their hashes and all raw pixel hashes.
    """
    audit = dict(policy=KIND, assumption=ASSUMPTION,
        pixel_source='retained_exact_original_sample',
        geometry_source='unchanged_request_fused_measured_geometry',
        source_binding_valid=False)
    try:
        retained = _retained_box_pixels(snapshot, source_identity, source_stamp_ns)
        if retained is None:
            return _failure('BOX_INFERENCE_SOURCE_UNAVAILABLE',
                'retained original RGB/depth/mask sample is required', audit)
        original, color, depth, mask, mask_hashes = retained
        audit.update(source_binding_valid=True, source_stamp_ns=int(original.stamp_ns),
            source_frame_id=original.frame_id, source_target_identity=dict(
                epoch=original.target_identity.epoch, track_id=original.target_identity.track_id),
            retained_pixel_sha256=dict(color_sha256=_array_hash(color),
                depth_sha256=_array_hash(depth), mask_sha256=_array_hash(mask)),
            final_sample_binary_mask_sha256=mask_hashes[-1])
    except (ValueError, TypeError, AttributeError) as exc:
        return _failure('BOX_INFERENCE_SOURCE_MISMATCH', str(exc), audit)
    result = qualify_regular_box_top(depth, mask, camera_intrinsics, depth_scale_m,
        T_base_camera, geometry, original.target_identity, original.stamp_ns,
        snapshot.source_mode, color_bgr=color)
    return replace(result, audit=MappingProxyType(dict(result.audit, **audit)))


def _qualify_regular_box_top(depth_raw, object_mask, camera_intrinsics,
        depth_scale_m, T_base_camera, geometry, source_identity,
        source_stamp_ns, source_mode='instance_mask', color_bgr=None, *, outer_fallback=False,
        cuboid_fallback=False):
    """Infer a regular box from calibrated optical depth and a complete top.

    ``T_base_camera`` maps OpenCV optical xyz into base, including any ROS
    camera-link conversion.  Source stamp/identity must come from the exact
    snapshot. Geometry must be its verified, unchanged support/target result.
    """
    audit = {'policy': KIND, 'assumption': ASSUMPTION,
             'uncertainty_semantics': 'empirical_budget_not_certified_error_bound'}
    try:
        if source_mode != 'instance_mask' or not isinstance(geometry, GeometryEstimate) or geometry.ok is not True or geometry.source_mode != source_mode:
            raise ValueError('an admitted instance mask and valid same-source geometry are required')
        validate_target_identity(source_identity)
        if isinstance(source_stamp_ns, (bool, np.bool_)) or not isinstance(source_stamp_ns, Integral) or source_stamp_ns <= 0:
            raise ValueError('source stamp must be exact positive integer nanoseconds')
        depth, mask_input = np.asarray(depth_raw), np.asarray(object_mask)
        if depth.ndim != 2 or mask_input.shape != depth.shape or depth.dtype.kind not in 'iuf' or mask_input.dtype.kind not in 'biu':
            raise ValueError('aligned metric-depth source and real pixel mask required')
        if hasattr(camera_intrinsics, 'fx'):
            K = np.array([getattr(camera_intrinsics, k) for k in ('fx', 'fy', 'cx', 'cy')], dtype=float)
            if (camera_intrinsics.height, camera_intrinsics.width) != depth.shape:
                raise ValueError('camera dimensions differ from depth')
        else:
            K = _array(camera_intrinsics, (4,), 'camera intrinsics')
        if min(K[:2]) <= 0. or not np.all(np.isfinite(K)):
            raise ValueError('camera focal length invalid')
        scale = _number(depth_scale_m, 'depth scale', 1e-7, 1.)
        T = _validated_transform(T_base_camera)
        n = _unit(geometry.support_normal_base, 'support normal')
        d = _number(geometry.support_offset_m, 'support offset')
        if geometry.support_inlier_ratio < .35:
            return _failure('BOX_SUPPORT_INCONSISTENT', 'verified support inlier fraction too low', audit)
        mask = mask_input > 0
        ys, xs = np.nonzero(mask)
        if len(xs) < 100:
            return _failure('BOX_TOP_CONTOUR_INCOMPLETE', 'too few actual mask pixels', audit)
        margin = int(min(xs.min(), ys.min(), depth.shape[1]-1-xs.max(), depth.shape[0]-1-ys.max()))
        audit['edge_clearance_px'] = margin
        if margin < 4:
            return _failure('BOX_TOP_CONTOUR_INCOMPLETE', 'target contour approaches image boundary', audit)
        valid = np.isfinite(depth) & (depth > 0.)
        valid_fraction = float(np.count_nonzero(valid & mask))/len(xs)
        # The real object mask also contains visible sides. Missing side depth
        # is not a prerequisite for top-based inference; completeness of the
        # actual top interior and all its edges is checked separately below.
        if valid_fraction < .60:
            return _failure('BOX_TOP_CONTOUR_INCOMPLETE', 'insufficient target depth to identify a complete top', audit)
        intrinsics = SimpleNamespace(width=depth.shape[1], height=depth.shape[0], fx=K[0], fy=K[1], cx=K[2], cy=K[3])
        pc, uv = deproject_depth(depth, mask, intrinsics, scale)
        points = pc @ T[:3, :3].T + T[:3, 3]
        heights = points @ n + d
        top_height_seed = float(np.quantile(heights, .80))
        seed = np.abs(heights-top_height_seed) <= .0025
        if np.count_nonzero(seed) < 80:
            return _failure('BOX_TOP_PLANE_INVALID', 'no substantial upper plane', audit)
        top = points[seed]
        for _ in range(2):
            origin = np.mean(top, axis=0)
            _u, _s, vt = np.linalg.svd(top-origin, full_matrices=False)
            tn = vt[-1] * (1. if vt[-1] @ n >= 0. else -1.)
            residual = (points-origin) @ tn
            keep = (np.abs(residual) <= .0015) & (heights >= .5*top_height_seed)
            top = points[keep]
            if len(top) < 80:
                return _failure('BOX_TOP_PLANE_INVALID', 'top plane does not have enough measured depth', audit)
        rms = float(np.sqrt(np.mean(((top-origin) @ tn)**2)))
        angle = math.degrees(math.acos(np.clip(tn @ n, -1., 1.)))
        top_height = float(np.median(top @ n + d))
        cosine = float((T[:3, 3]-np.mean(top, axis=0)) @ tn / np.linalg.norm(T[:3, 3]-np.mean(top, axis=0)))
        audit.update(top_plane_rms_m=rms, top_support_angle_deg=angle, top_height_m=top_height, camera_top_cosine=cosine)
        if rms > .0015 or angle > 6. or cosine < .35 or not .005 <= top_height <= .5:
            return _failure('BOX_TOP_PLANE_INVALID', 'top must be resolved, approximately parallel to support and viewed above it', audit)
        pixel_size = float(np.median(pc[keep, 2])) / min(K[:2]) / max(cosine, .35)
        if pixel_size > .003:
            return _failure('BOX_TOP_CONTOUR_INCOMPLETE', 'metric pixel footprint too large for bounded box inference', audit)
        B = _basis(n)
        xy = top @ B
        rect = cv2.minAreaRect(xy.astype(np.float32))
        vertices = cv2.boxPoints(rect).astype(float)
        edge_a, edge_b = vertices[1]-vertices[0], vertices[2]-vertices[1]
        if np.linalg.norm(edge_b) > np.linalg.norm(edge_a):
            edge_a = edge_b
        ax0 = B @ (edge_a/np.linalg.norm(edge_a))
        if ax0[int(np.argmax(np.abs(ax0)))] < 0.:
            ax0 = -ax0
        ax1 = np.cross(n, ax0)
        axes = np.column_stack((ax0, ax1, n))
        coord = top @ axes[:, :2]
        lower, upper = coord.min(axis=0), coord.max(axis=0)
        visual = _visual_rectangle(color_bgr, mask, depth, scale, T, K,
                                   origin, tn, axes, lower, upper, pixel_size)
        outer = None
        cuboid = None
        if cuboid_fallback:
            cuboid = infer_cuboid_rgb_interval(color_bgr, mask_input, depth, scale, T, K,
                origin, tn, axes, pixel_size, rms, d, top_height)
            audit['cuboid_rgb_interval'] = cuboid['audit']
            if not cuboid['ok']:
                return _failure(cuboid.get('code', 'BOX_TOP_NOT_RECTANGULAR'), cuboid['reason'], audit)
            # JSON/digest checks establish consistency; current retained pixels
            # establish actual Canny membership and complete unknown coverage.
            validate_cuboid_rgb_interval_source(cuboid['audit'], color_bgr,
                mask_input, depth, scale, T, K)
            axes, lower, upper = cuboid['axes'], cuboid['lower'], cuboid['upper']
            top_height = cuboid['nominal_height_m']
            coord = top @ axes[:, :2]
            visual = None
        elif outer_fallback:
            outer = infer_outer_rgb_rectangle(color_bgr, mask, depth, scale, T, K,
                origin, tn, axes, pixel_size, rms)
            audit['outer_rgb_interval'] = outer['audit']
            if not outer['ok']:
                return _failure(outer.get('code', 'BOX_TOP_NOT_RECTANGULAR'), outer['reason'], audit)
            axes, lower, upper = outer['axes'], outer['lower'], outer['upper']
            coord = top @ axes[:, :2]
            visual = None
        elif visual is not None:
            lower,upper=visual[1],visual[2]
        size = np.r_[upper-lower, top_height]
        if np.any(size < .005) or np.any(size > .6):
            return _failure('BOX_TOP_NOT_RECTANGULAR', 'metric rectangle has invalid extent', audit)
        midpoint = (upper+lower)/2.
        center = axes[:, :2] @ midpoint + n*(top_height/2.-d)
        hull = cv2.convexHull(coord.astype(np.float32))
        fill = float(min(1.,cv2.contourArea(hull)/(size[0]*size[1])))
        measured_depth_hull_fill=fill
        tolerance = max(2.5*pixel_size, .04*min(size[:2]))
        corner_coverage, edge_coverage, edge_residuals, corner_distances = [], [], [], []
        signs = ((-1,-1), (1,-1), (1,1), (-1,1))
        corners2 = np.array([midpoint+np.asarray(s)*size[:2]/2. for s in signs])
        for corner in corners2:
            distance = float(np.min(np.linalg.norm(coord-corner, axis=1)))
            corner_distances.append(distance)
            corner_coverage.append(float(max(0., 1.-distance/(2.*tolerance))))
        for i in range(4):
            a, b = corners2[i], corners2[(i+1)%4]
            v = b-a
            along = (coord-a) @ v / float(v@v)
            distance = np.abs(np.cross(v, coord-a))/np.linalg.norm(v)
            near = (distance <= tolerance) & (along >= 0.) & (along <= 1.)
            bins = set(np.clip((along[near]*10).astype(int), 0, 9).tolist())
            edge_coverage.append(len(bins)/10.)
            # Actual edge fit residual uses the nearest occupied edge in each
            # bin; interior samples are not mistaken for edge uncertainty.
            for j in bins:
                group = (along >= j/10.) & (along <= (j+1)/10.)
                edge_residuals.append(float(np.min(distance[group])))
        if visual is not None:
            corner_coverage,edge_coverage=visual[3],visual[4]
        audit.update(rectangle_fill_ratio=fill, corner_coverage=corner_coverage, edge_coverage=edge_coverage,
                     inferred_size_xyz_m=size.tolist(), corner_distances_m=corner_distances, metric_pixel_size_m=pixel_size)
        if outer is None and cuboid is None and ((visual is None and fill < .88) or min(corner_coverage) < .5 or min(edge_coverage) < .7):
            return _failure('BOX_TOP_NOT_RECTANGULAR', 'all four metric sides and corners must be supported', audit)
        top_mask = np.zeros(depth.shape, np.uint8)
        top_mask[uv[keep, 1], uv[keep, 0]] = 1
        # Image polygon is obtained from the fitted metric top, so oblique
        # perspective is respected rather than demanding a 2-D rectangle.
        corners_base = corners2 @ axes[:, :2].T + n*(top_height-d)
        # Fit the actual top plane at each tangent-plane corner.
        corners_base += np.outer(((origin-corners_base) @ tn)/(tn@n), n)
        image_corners = _project(corners_base, T, K)
        polygon = np.zeros(depth.shape, np.uint8)
        cv2.fillConvexPoly(polygon, np.rint(image_corners).astype(np.int32), 1)
        interior = cv2.erode(polygon, np.ones((3,3), np.uint8)) > 0
        if visual is not None or outer is not None or cuboid is not None:
            # Complete visual contour support is separate from the measured
            # depth occupancy. This does not fill the missing depth pixels.
            fill=float(np.count_nonzero(interior & mask))/max(1,int(np.count_nonzero(interior)))
            audit['rectangle_fill_ratio']=fill
            audit['measured_depth_hull_fill_ratio']=measured_depth_hull_fill
            if fill<.88:
                return _failure('BOX_TOP_CONTOUR_INCOMPLETE','visual top rectangle is not covered by the actual object mask',audit)
        occupancy = float(np.count_nonzero(interior & (top_mask > 0)))/max(1, int(np.count_nonzero(interior)))
        missing = (interior & (top_mask == 0)).astype(np.uint8)
        count, labels, stats, _centroids = cv2.connectedComponentsWithStats(missing, 8)
        largest_hole = 0
        boundary = (polygon > 0) & ~interior
        dilated_boundary = cv2.dilate(boundary.astype(np.uint8), np.ones((3,3),np.uint8)) > 0
        for index in range(1, count):
            component = labels == index
            if not np.any(component & dilated_boundary):
                largest_hole = max(largest_hole, int(stats[index, cv2.CC_STAT_AREA]))
        hole_fraction = float(largest_hole)/max(1, int(np.count_nonzero(interior)))
        audit.update(top_occupancy_fraction=occupancy, maximum_internal_hole_fraction=hole_fraction)
        if occupancy < (.65 if visual is not None or outer is not None or cuboid is not None else .88) or (hole_fraction > .02 and outer is None):
            return _failure('BOX_TOP_CONTOUR_INCOMPLETE', 'missing or holed top occupancy', audit)
        if outer is not None or cuboid is not None:
            contour = outer if outer is not None else cuboid
            continuation, outside_valid = contour['continuation'], contour['outside_valid']
            audit.update(outside_top_continuation_fraction=continuation, outside_valid_depth_fraction=outside_valid)
        else:
            continuation, outside_valid = [], []
            image_center = np.mean(image_corners, axis=0)
            for i in range(4):
                a, b = image_corners[i], image_corners[(i+1)%4]
                v = b-a
                outward = np.array([v[1], -v[0]]) / np.linalg.norm(v)
                if outward @ ((a+b)/2.-image_center) < 0.:
                    outward *= -1.
                sample = a + np.outer(np.linspace(.1,.9,21), v) + 3.*outward
                pixels = np.rint(sample).astype(int)
                inside = ((pixels[:,0] >= 0) & (pixels[:,0] < depth.shape[1]) &
                          (pixels[:,1] >= 0) & (pixels[:,1] < depth.shape[0]))
                if not np.all(inside):
                    return _failure('BOX_TOP_CONTOUR_INCOMPLETE', 'outside contour cannot be observed', audit)
                outside_z = depth[pixels[:,1], pixels[:,0]].astype(float)*scale
                good = np.isfinite(outside_z) & (outside_z > 0.)
                outside_valid.append(float(np.mean(good)))
                xyz = np.column_stack(((pixels[:,0]-K[2])*outside_z/K[0], (pixels[:,1]-K[3])*outside_z/K[1], outside_z))
                outside = xyz @ T[:3,:3].T + T[:3,3]
                plane_distance = (outside-origin) @ tn
                continuation.append(float(np.count_nonzero(good & (np.abs(plane_distance) < .0015)))/max(1,int(np.count_nonzero(good))))
                if np.count_nonzero(good & (plane_distance > .0025)) > .30*len(good):
                    return _failure('BOX_TOP_CONTOUR_INCOMPLETE', 'an occluder can truncate the top boundary', audit)
            audit.update(outside_top_continuation_fraction=continuation, outside_valid_depth_fraction=outside_valid)
            if ((visual is None and (max(continuation)>.30 or min(outside_valid)<.60)) or
                    (visual is not None and any(v>=.20 and c>.30 for v,c in zip(outside_valid,continuation)))):
                return _failure('BOX_TOP_CONTOUR_INCOMPLETE', 'top continues outside mask or outside depth unavailable', audit)
        edge_rms = float(np.sqrt(np.mean(np.square(edge_residuals))))
        if visual is not None:
            # Depth-hull truncation is not an RGB line-fit residual. Use its
            # measured angular discrepancy plus pixel localization instead.
            edge_rms=max(.5*pixel_size,.5*min(size[:2])*visual[5])
        unobserved_extension=float(visual[6]) if visual is not None else 0.
        edge_uncertainty = max(.001, 2.*rms+scale) + .5*pixel_size + unobserved_extension
        height_uncertainty = max(.001, 2.*rms+scale, .5*max(size[:2])*math.sin(math.radians(angle)))
        yaw_uncertainty = max(math.radians(.25), math.atan2(2.*max(edge_rms, .5*pixel_size), max(size[:2])),
                              math.atan(visual[5]) if visual is not None else 0.)
        if outer is not None:
            edge_uncertainty, yaw_uncertainty = outer['edge_uncertainty'], outer['yaw_uncertainty']
            edge_rms = outer['edge_rms']
            top_vertical_deviation = float(np.max(np.abs(corners_base@n+d-top_height)))
            height_uncertainty = max(.001, top_vertical_deviation+outer['audit']['top_plane_slab_m']/float(tn@n))
        if cuboid is not None:
            edge_uncertainty, yaw_uncertainty = cuboid['edge_uncertainty'], cuboid['yaw_uncertainty']
            edge_rms = cuboid['edge_rms']
            top_vertical_deviation = cuboid['top_plane_vertical_deviation_m']
            height_uncertainty = cuboid['height_uncertainty']
        measured = np.asarray(geometry.object_points_base)
        if measured.ndim != 2 or measured.shape[1] != 3 or not len(measured) or not np.all(np.isfinite(measured)):
            raise ValueError('verified measured geometry missing')
        edge_axes = (np.asarray(cuboid['audit']['empirical_axis_uncertainty_m'], dtype=float)
                     if cuboid is not None else np.array([edge_uncertainty, edge_uncertainty]))
        excess, conflict = _measured_family_excess(measured, center, axes, size,
            edge_axes, height_uncertainty, yaw_uncertainty, outer is not None or cuboid is not None)
        audit['maximum_observed_model_excess_m'] = excess
        if excess > .001:
            audit['measured_geometry_conflict'] = conflict
            return _failure('BOX_MEASURED_GEOMETRY_CONFLICT', 'measured target extends outside inferred regular prism', audit)
        if size[2] <= 2.*height_uncertainty+.002:
            return _failure('BOX_INFERENCE_PRECISION_VIEW_REQUIRED', 'height uncertainty leaves no robust contact interval', audit)
        if edge_uncertainty>.01 or yaw_uncertainty>math.radians(12.):
            return _failure('BOX_INFERENCE_PRECISION_VIEW_REQUIRED','unresolved contour extent exceeds the bounded inference uncertainty',dict(audit,empirical_edge_uncertainty_m=edge_uncertainty,empirical_yaw_uncertainty_rad=yaw_uncertainty))
        if outer is not None and hole_fraction > .02:
            regions = []
            for index in range(1, count):
                component = labels == index
                if np.any(component & dilated_boundary):
                    continue
                yy, xx = np.where(component)
                z = depth[yy, xx].astype(float)*scale
                observed = np.isfinite(z)&(z > 0.)
                measured_region = np.column_stack(((xx-K[2])*z/K[0], (yy-K[3])*z/K[1], z))@T[:3, :3].T+T[:3, 3]
                distance = (measured_region-origin)@tn
                compatible = observed & (np.abs(distance) <= height_uncertainty)
                contradictory = observed & ~compatible
                regions.append(dict(pixels_yx=np.column_stack((yy, xx)).tolist(),
                    raw_unknown_pixels_yx=np.column_stack((yy[~observed], xx[~observed])).tolist(),
                    measured_compatible_pixels_yx=np.column_stack((yy[compatible], xx[compatible])).tolist(),
                    measured_recess_or_occluder_pixels_yx=np.column_stack((yy[contradictory], xx[contradictory])).tolist(),
                    measured_plane_distance_m=distance[observed].tolist()))
            audit['interior_depth_regions'] = regions
            if any(region['measured_recess_or_occluder_pixels_yx'] for region in regions):
                return _failure('BOX_TOP_CONTOUR_INCOMPLETE',
                    'observed interior depth contradicts the bounded top layer', audit)
            source = _box_source_evidence(depth, mask_input, color_bgr, K, scale, T,
                geometry, source_identity, source_stamp_ns)
            narrow = int(np.argmin(size[:2]))
            missing_pixels = sorted(pixel for region in regions for pixel in region['raw_unknown_pixels_yx'])
            candidate = dict(schema_version=1, kind='regular_box_precision_candidate',
                qualification='observation_only_not_grasp_qualified', motion_authorized=False,
                measured_side_surfaces=False, source=source, image_shape=list(depth.shape),
                center_base=center.tolist(), axes_base=axes.tolist(), size_xyz_m=size.tolist(),
                empirical_edge_uncertainty_m=edge_uncertainty,
                empirical_height_uncertainty_m=height_uncertainty,
                empirical_yaw_uncertainty_rad=yaw_uncertainty,
                support_normal_base=n.tolist(), support_offset_m=d,
                outer_rgb_interval=outer['audit'], interior_regions=regions,
                top_occupancy_fraction=occupancy, maximum_internal_unknown_fraction=hole_fraction,
                maximum_observed_model_excess_m=excess,
                targeted_view=dict(purpose='resolve_narrow_edge_localization_and_interior_depth',
                    jaw_axis_base=axes[:, narrow].tolist(), target_edges_or_corners_base=corners_base.tolist(),
                    source_stamp_ns=int(source_stamp_ns), missing_region_pixels_yx=missing_pixels),
                uncertainty_semantics='empirical_budget_not_certified_error_bound')
            candidate['candidate_sha256'] = _hash(candidate)
            candidate = validate_box_precision_candidate(candidate)
            return _failure('BOX_INFERENCE_PRECISION_VIEW_REQUIRED',
                'bounded RGB top hypothesis requires localized depth in its unknown interior',
                dict(audit, precision_view_candidate=candidate,
                    unknown_interior_is_not_measured_hole=True, unknown_depth_filled=False))
        qualification = dict(
            top_plane_rms_m=rms, top_support_angle_deg=angle, rectangle_fill_ratio=fill,
            corner_coverage=corner_coverage, edge_coverage=edge_coverage,
            outside_top_continuation_fraction=continuation, outside_valid_depth_fraction=outside_valid,
            valid_depth_fraction=valid_fraction, edge_clearance_px=margin, top_point_count=len(top),
            top_occupancy_fraction=occupancy, maximum_internal_hole_fraction=hole_fraction,
            camera_top_cosine=cosine, metric_pixel_size_m=pixel_size, edge_fit_rms_m=edge_rms,
            maximum_observed_model_excess_m=excess,
            uncertainty_semantics='empirical_budget_not_certified_error_bound',
            unobserved_boundary_extension_m=unobserved_extension,
            contour_source='rgb_edges_on_measured_top_plane' if visual is not None else 'measured_depth_top')
        if outer is not None:
            qualification['contour_source'] = 'rgb_outer_joint_interval_on_measured_top_plane'
            qualification['outer_rgb_interval'] = outer['audit']
            qualification['top_plane_vertical_deviation_m'] = top_vertical_deviation
        if cuboid is not None:
            qualification['contour_source'] = 'cuboid_rgb_joint_interval_on_measured_support'
            qualification['cuboid_rgb_interval'] = cuboid['audit']
            qualification['top_plane_vertical_deviation_m'] = top_vertical_deviation
        geom_payload = _box_geometry_source_payload(geometry)
        source = dict(target_identity=dict(epoch=source_identity.epoch, track_id=source_identity.track_id),
            stamp_ns=int(source_stamp_ns), depth_sha256=_array_hash(depth), mask_sha256=_array_hash(mask_input),
            color_sha256=_array_hash(color_bgr) if color_bgr is not None else None,
            intrinsics_sha256=_hash(dict(shape=list(depth.shape), K=K.tolist(), depth_scale_m=scale)),
            T_base_camera_sha256=_array_hash(T), geometry_sha256=_hash(geom_payload))
        model = dict(schema_version=3 if cuboid is not None else 2 if outer is not None else 1, kind=KIND, assumption=ASSUMPTION,
            center_base=center.tolist(), axes_base=axes.tolist(), size_xyz_m=size.tolist(),
            empirical_edge_uncertainty_m=edge_uncertainty, empirical_height_uncertainty_m=height_uncertainty,
            empirical_yaw_uncertainty_rad=yaw_uncertainty, support_normal_base=n.tolist(), support_offset_m=d,
            source_center_base=center.tolist(), source_axes_base=axes.tolist(), source_support_normal_base=n.tolist(),
            source_support_offset_m=d, registration_transform=np.eye(4).tolist(), source=source, qualification=qualification)
        model['model_sha256'] = _hash(model)
        return BoxInferenceResult(True, 'REGULAR_BOX_INFERENCE_QUALIFIED',
            RegularBoxInference.from_evidence_dict(model), MappingProxyType(dict(audit,
                qualification=qualification, model_sha256=model['model_sha256'], measured_side_surfaces=False)))
    except (ValueError, TypeError, AttributeError, np.linalg.LinAlgError, cv2.error) as exc:
        return _failure('BOX_INFERENCE_INPUT_INVALID', str(exc), audit)


def qualify_regular_box_top(depth_raw, object_mask, camera_intrinsics,
        depth_scale_m, T_base_camera, geometry, source_identity,
        source_stamp_ns, source_mode='instance_mask', color_bgr=None):
    """Preserve qualified v1 models; retry incomplete tops with current RGB.

    The fallback supplies independent outer-edge qualification and a common
    model uncertainty budget. Missing depth coverage remains visible in the
    audit. It never alters raw depth, measured geometry or source identity.
    """
    args = (depth_raw, object_mask, camera_intrinsics, depth_scale_m,
            T_base_camera, geometry, source_identity, source_stamp_ns, source_mode)
    result = _qualify_regular_box_top(*args, color_bgr=color_bgr)
    if result.ok or color_bgr is None or result.code not in (
            'BOX_TOP_NOT_RECTANGULAR', 'BOX_TOP_CONTOUR_INCOMPLETE'):
        return result
    fallback = _qualify_regular_box_top(*args, color_bgr=color_bgr, outer_fallback=True)
    if not fallback.ok and fallback.code in ('BOX_TOP_NOT_RECTANGULAR',
            'BOX_TOP_CONTOUR_INCOMPLETE', 'BOX_INFERENCE_PRECISION_VIEW_REQUIRED'):
        cuboid = _qualify_regular_box_top(*args, color_bgr=color_bgr, cuboid_fallback=True)
        interval = cuboid.audit.get('cuboid_rgb_interval')
        if (cuboid.ok or (isinstance(interval, dict)
                         and interval.get('complete_raster_responses_used') is True)):
            # A completed silhouette fit can fail a later occupancy or
            # measured-geometry check. Preserve that first unresolved handoff,
            # rather than relabeling it as the earlier top-projection failure.
            # This remains a failed result with no executable model.
            return replace(cuboid, audit=MappingProxyType(dict(cuboid.audit,
                original_qualification_code=result.code,
                original_qualification_reason=result.audit.get('reason'),
                top_projection_qualification_code=fallback.code,
                top_projection_qualification_reason=fallback.audit.get('reason'))))
        fallback = replace(fallback, audit=MappingProxyType(dict(fallback.audit,
            cuboid_qualification_code=cuboid.code,
            cuboid_qualification_reason=cuboid.audit.get('reason'),
            cuboid_rgb_interval=cuboid.audit.get('cuboid_rgb_interval'))))
    return replace(fallback, audit=MappingProxyType(dict(fallback.audit,
        original_qualification_code=result.code, original_qualification_reason=result.audit.get('reason'))))


def _box_source_evidence(depth, mask, color, K, scale, T, geometry, identity, stamp):
    payload = _box_geometry_source_payload(geometry)
    return dict(target_identity=dict(epoch=identity.epoch, track_id=identity.track_id), stamp_ns=int(stamp),
        depth_sha256=_array_hash(depth), mask_sha256=_array_hash(mask),
        color_sha256=_array_hash(color) if color is not None else None,
        intrinsics_sha256=_hash(dict(shape=list(depth.shape), K=K.tolist(), depth_scale_m=scale)),
        T_base_camera_sha256=_array_hash(T), geometry_sha256=_hash(payload))


_PRECISION_CANDIDATE_FIELDS = frozenset(('schema_version', 'kind', 'qualification', 'motion_authorized',
    'measured_side_surfaces', 'source', 'image_shape', 'center_base', 'axes_base', 'size_xyz_m',
    'empirical_edge_uncertainty_m', 'empirical_height_uncertainty_m', 'empirical_yaw_uncertainty_rad',
    'support_normal_base', 'support_offset_m', 'outer_rgb_interval', 'interior_regions',
    'top_occupancy_fraction', 'maximum_internal_unknown_fraction', 'maximum_observed_model_excess_m',
    'targeted_view', 'uncertainty_semantics', 'candidate_sha256'))


def validate_box_precision_candidate(value):
    """Own an observation-only, exact-source hypothesis, never a grasp model.

    The remote must independently bind these hashes to its retained snapshot,
    and use its actual measured collision geometry for any observation motion.
    """
    if not isinstance(value, dict) or set(value) != _PRECISION_CANDIDATE_FIELDS:
        raise ValueError('box precision candidate fields differ')
    d = json.loads(_canonical(value))
    if (type(d['schema_version']) is not int or d['schema_version'] != 1
            or d['kind'] != 'regular_box_precision_candidate'
            or d['qualification'] != 'observation_only_not_grasp_qualified'
            or d['motion_authorized'] is not False or d['measured_side_surfaces'] is not False
            or d['uncertainty_semantics'] != 'empirical_budget_not_certified_error_bound'):
        raise ValueError('box precision candidate cannot authorize grasping')
    if d['candidate_sha256'] != _hash({k:v for k,v in d.items() if k != 'candidate_sha256'}):
        raise ValueError('box precision candidate digest differs')
    center = _array(d['center_base'], (3,), 'precision center')
    axes = _rotation(d['axes_base'], 'precision axes')
    size = _array(d['size_xyz_m'], (3,), 'precision size')
    if np.any(size < .005) or np.any(size > .6) or size[2] > .5:
        raise ValueError('precision candidate dimensions invalid')
    normal = _unit(d['support_normal_base'], 'precision support normal')
    offset = _number(d['support_offset_m'], 'precision support offset')
    if not np.allclose(axes[:, 2], normal, rtol=0., atol=1e-7) or abs(center@normal+offset-size[2]/2.)>1e-7:
        raise ValueError('precision candidate support differs')
    edge = _number(d['empirical_edge_uncertainty_m'], 'precision edge', .001, .01)
    height = _number(d['empirical_height_uncertainty_m'], 'precision height', .001, .01)
    yaw = _number(d['empirical_yaw_uncertainty_rad'], 'precision yaw', math.radians(.25), math.radians(12.))
    if size[2] <= 2.*height+.002:
        raise ValueError('precision candidate height unresolved')
    validate_outer_rgb_interval(d['outer_rgb_interval'], size[:2], edge, yaw)
    _number(d['top_occupancy_fraction'], 'precision top occupancy', .65, 1.)
    _number(d['maximum_internal_unknown_fraction'], 'precision internal unknown', .02, .35)
    _number(d['maximum_observed_model_excess_m'], 'precision measured excess', 0., .001)
    source = d['source']
    if not isinstance(source, dict) or set(source) != _SOURCE_FIELDS:
        raise ValueError('precision source fields differ')
    identity = source['target_identity']
    if not isinstance(identity, dict) or set(identity) != {'epoch', 'track_id'}:
        raise ValueError('precision target identity invalid')
    TargetTrackIdentity(**identity)
    if type(source['stamp_ns']) is not int or source['stamp_ns'] <= 0:
        raise ValueError('precision source stamp invalid')
    for key in _SOURCE_FIELDS-{'target_identity', 'stamp_ns'}:
        digest = source[key]
        if not isinstance(digest, str) or len(digest) != 64 or any(c not in '0123456789abcdef' for c in digest):
            raise ValueError('precision source hash invalid')
    shape = d['image_shape']
    if not isinstance(shape, list) or len(shape) != 2 or any(type(v) is not int or v <= 0 for v in shape):
        raise ValueError('precision source image shape invalid')
    def pixels(rows):
        if not isinstance(rows, list) or any(not isinstance(p, list) or len(p) != 2 or any(type(v) is not int for v in p)
                or not 0 <= p[0] < shape[0] or not 0 <= p[1] < shape[1] for p in rows):
            raise ValueError('precision interior pixel invalid')
        result = {tuple(row) for row in rows}
        if len(result) != len(rows) or rows != [list(p) for p in sorted(result)]:
            raise ValueError('precision interior pixels must be unique row-major')
        return result
    regions = d['interior_regions']
    if not isinstance(regions, list) or not regions:
        raise ValueError('precision unknown interior evidence absent')
    all_pixels, all_unknown = set(), set()
    for region in regions:
        if not isinstance(region, dict) or set(region) != {'pixels_yx', 'raw_unknown_pixels_yx', 'measured_compatible_pixels_yx', 'measured_recess_or_occluder_pixels_yx', 'measured_plane_distance_m'}:
            raise ValueError('precision interior partition fields differ')
        whole = pixels(region['pixels_yx'])
        unknown = pixels(region['raw_unknown_pixels_yx'])
        compatible = pixels(region['measured_compatible_pixels_yx'])
        contradictory = pixels(region['measured_recess_or_occluder_pixels_yx'])
        if not whole or whole&all_pixels or unknown&compatible or contradictory or whole != unknown|compatible:
            raise ValueError('precision interior is unpartitioned or contradictory')
        distances = region['measured_plane_distance_m']
        if not isinstance(distances, list) or len(distances) != len(compatible):
            raise ValueError('precision measured interior distances absent')
        for value in distances:
            _number(value, 'precision interior plane distance', -height, height)
        all_pixels |= whole
        all_unknown |= unknown
    target = d['targeted_view']
    if not isinstance(target, dict) or set(target) != {'purpose', 'jaw_axis_base', 'target_edges_or_corners_base', 'source_stamp_ns', 'missing_region_pixels_yx'}:
        raise ValueError('precision view target fields differ')
    jaw = _unit(target['jaw_axis_base'], 'precision jaw axis')
    corners = _array(target['target_edges_or_corners_base'], (4, 3), 'precision top corners')
    if (target['purpose'] != 'resolve_narrow_edge_localization_and_interior_depth'
            or target['source_stamp_ns'] != source['stamp_ns']
            or pixels(target['missing_region_pixels_yx']) != all_unknown
            or not np.allclose(jaw, axes[:, int(np.argmin(size[:2]))], rtol=0., atol=1e-7)):
        raise ValueError('precision view target differs from candidate')
    local = (corners-center)@axes
    expected = np.asarray([[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]])*size[:2]/2.
    if not np.allclose(local[:, :2], expected, rtol=0., atol=1e-7) or np.max(np.abs(local[:, 2]-size[2]/2.)) > height+1e-7:
        raise ValueError('precision view corners differ from bounded top')
    return d


def validate_box_precision_candidate_source(value, snapshot, camera_intrinsics,
        depth_scale_m, T_base_camera, geometry, source_identity, source_stamp_ns):
    """Bind a view-only candidate to the retained current request, without fitting.

    This verifies pixel/calibration/geometry identity. Request freshness, target
    lifecycle and observation-motion collision checks remain the caller's job.
    No grasp model or inferred collision geometry is produced.
    """
    d = validate_box_precision_candidate(value)
    retained = _retained_box_pixels(snapshot, source_identity, source_stamp_ns)
    if retained is None:
        raise ValueError('retained original RGB/depth/mask sample is required')
    original, color, depth, mask, _ = retained
    if (not isinstance(geometry, GeometryEstimate) or geometry.ok is not True
            or geometry.source_mode != snapshot.source_mode):
        raise ValueError('precision candidate requires admitted measured geometry')
    if hasattr(camera_intrinsics, 'fx'):
        K = np.array([getattr(camera_intrinsics, k) for k in ('fx', 'fy', 'cx', 'cy')], dtype=float)
        if (camera_intrinsics.height, camera_intrinsics.width) != depth.shape:
            raise ValueError('camera dimensions differ from retained depth')
    else:
        K = _array(camera_intrinsics, (4,), 'camera intrinsics')
    if not np.all(np.isfinite(K)) or min(K[:2]) <= 0.:
        raise ValueError('camera focal length invalid')
    scale = _number(depth_scale_m, 'depth scale', 1e-7, 1.)
    T = _validated_transform(T_base_camera)
    source = _box_source_evidence(depth, mask, color, K, scale, T, geometry,
        original.target_identity, original.stamp_ns)
    if d['image_shape'] != list(depth.shape) or _canonical(d['source']) != _canonical(source):
        raise ValueError('precision candidate differs from retained source evidence')
    return d
