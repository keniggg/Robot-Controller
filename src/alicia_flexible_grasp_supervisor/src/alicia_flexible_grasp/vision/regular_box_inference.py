"""Bounded regular-prism inference from a single complete measured box top.

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
    if type(d['schema_version']) is not int or d['schema_version'] not in (1, 2) or d['kind'] != KIND or d['assumption'] != ASSUMPTION:
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
    if not isinstance(q, dict) or set(q) != (_QUALIFICATION_FIELDS | {'outer_rgb_interval'} if is_outer else _QUALIFICATION_FIELDS):
        raise ValueError('model qualification fields differ')
    visual_contour = q.get('contour_source') in ('rgb_edges_on_measured_top_plane', 'rgb_outer_joint_interval_on_measured_top_plane')
    if is_outer != (q.get('contour_source') == 'rgb_outer_joint_interval_on_measured_top_plane'):
        raise ValueError('RGB interval schema/qualification differs')
    if is_outer:
        validate_outer_rgb_interval(q['outer_rgb_interval'], size[:2], d['empirical_edge_uncertainty_m'], d['empirical_yaw_uncertainty_rad'])
    for key, lo, hi in (
            ('top_plane_rms_m', 0., .0015), ('top_support_angle_deg', 0., 6.),
            ('rectangle_fill_ratio', .88, 1.000001), ('valid_depth_fraction', .60, 1.),
            ('top_occupancy_fraction', .65 if visual_contour else .88, 1.), ('maximum_internal_hole_fraction', 0., .02),
            ('camera_top_cosine', .35, 1.000001), ('metric_pixel_size_m', 0., .003),
            ('edge_fit_rms_m', 0., .005), ('maximum_observed_model_excess_m', 0., .001),
            ('unobserved_boundary_extension_m', 0., .01)):
        _number(q[key], key, lo, hi)
    for key, lo, hi in (
            ('corner_coverage', 0. if is_outer else .5, 1.), ('edge_coverage', 0. if is_outer else .7, 1.),
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
    if q['contour_source'] not in ('measured_depth_top', 'rgb_edges_on_measured_top_plane', 'rgb_outer_joint_interval_on_measured_top_plane'):
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
        object.__setattr__(self, '_serialized', _canonical(checked))

    @classmethod
    def from_evidence_dict(cls, value):
        return cls(_canonical(value))

    def to_evidence_dict(self):
        return json.loads(self._serialized)

    def __getattr__(self, key):
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
    """Conservative planning enclosure; original measured points stay intact.

    The returned OBB is collision geometry and explicitly includes an inferred
    prism plus empirical uncertainty. It is not evidence of measured sides.
    """
    from itertools import product
    if not isinstance(geometry, GeometryEstimate) or not geometry.ok or not isinstance(model, RegularBoxInference):
        raise ValueError('valid original geometry and qualified inferred box required')
    n=model.support_normal_base
    if not np.allclose(geometry.support_normal_base,n,rtol=0.,atol=1e-6) or abs(geometry.support_offset_m-model.support_offset_m)>1e-6:
        raise ValueError('collision enclosure and inferred support differ')
    A=model.axes_base; size=model.size_xyz_m
    old_c=np.asarray(geometry.center_base); old_A=np.asarray(geometry.axes_base); old_s=np.asarray(geometry.size_xyz_m)
    old_vertices=np.asarray([old_c+old_A@(np.asarray(sign)*old_s/2.) for sign in product((-1.,1.),repeat=3)])
    inferred=inferred_box_enclosure_vertices(model)
    all_vertices=np.vstack((old_vertices,inferred))
    local=all_vertices@A
    lo,hi=local.min(axis=0),local.max(axis=0)
    updated=replace(geometry,center_base=A@((lo+hi)/2.),axes_base=A,size_xyz_m=hi-lo)
    if not np.array_equal(updated.object_points_base,geometry.object_points_base):
        raise ValueError('planning enclosure must not change measured points')
    return updated,dict(policy='union_measured_enclosure_and_inferred_box_uncertainty',
        model_sha256=model.model_sha256, measured_points_unchanged=True,
        measured_side_surfaces=False, original_center_base=old_c.tolist(),
        original_axes_base=old_A.tolist(),original_size_xyz_m=old_s.tolist(),
        planning_center_base=updated.center_base.tolist(),planning_axes_base=updated.axes_base.tolist(),
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
    for x,y in ((size[0],size[1]),(size[1],size[0])):
        angle=min(float(yaw),math.atan2(float(y),float(x)))
        spread.append(float(x*math.cos(angle)+y*math.sin(angle)+2.*model.empirical_edge_uncertainty_m))
    half=np.asarray([spread[0]/2.,spread[1]/2.,(size[2]+model.empirical_height_uncertainty_m)/2.])
    expanded_center=model.center_base+n*model.empirical_height_uncertainty_m/2.
    vertices=np.asarray([expanded_center+A@(np.asarray(sign)*half) for sign in product((-1.,1.),repeat=3)])
    return np.frombuffer(vertices.tobytes(),dtype=float).reshape((8,3))


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
        if getattr(snapshot, 'ok', None) is not True or snapshot.source_mode != 'instance_mask':
            raise ValueError('admitted instance-mask snapshot required')
        original = retained_exact_source(snapshot)
        if original is None:
            return _failure('BOX_INFERENCE_SOURCE_UNAVAILABLE',
                'retained original RGB/depth/mask sample is required', audit)
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
        source_stamp_ns, source_mode='instance_mask', color_bgr=None, *, outer_fallback=False):
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
        if outer_fallback:
            outer = infer_outer_rgb_rectangle(color_bgr, mask, depth, scale, T, K,
                origin, tn, axes, pixel_size, rms)
            audit['outer_rgb_interval'] = outer['audit']
            if not outer['ok']:
                return _failure('BOX_TOP_NOT_RECTANGULAR', outer['reason'], audit)
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
        if outer is None and ((visual is None and fill < .88) or min(corner_coverage) < .5 or min(edge_coverage) < .7):
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
        if visual is not None or outer is not None:
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
        if occupancy < (.65 if visual is not None or outer is not None else .88) or hole_fraction > .02:
            return _failure('BOX_TOP_CONTOUR_INCOMPLETE', 'missing or holed top occupancy', audit)
        if outer is not None:
            continuation, outside_valid = outer['continuation'], outer['outside_valid']
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
        measured = np.asarray(geometry.object_points_base)
        if measured.ndim != 2 or measured.shape[1] != 3 or not len(measured) or not np.all(np.isfinite(measured)):
            raise ValueError('verified measured geometry missing')
        local = (measured-center) @ axes
        excess = float(max(0., np.max(np.abs(local)-size/2.-np.array([edge_uncertainty, edge_uncertainty, height_uncertainty]))))
        audit['maximum_observed_model_excess_m'] = excess
        if excess > .001:
            return _failure('BOX_MEASURED_GEOMETRY_CONFLICT', 'measured target extends outside inferred regular prism', audit)
        if size[2] <= 2.*height_uncertainty+.002:
            return _failure('BOX_INFERENCE_PRECISION_VIEW_REQUIRED', 'height uncertainty leaves no robust contact interval', audit)
        if edge_uncertainty>.01 or yaw_uncertainty>math.radians(12.):
            return _failure('BOX_INFERENCE_PRECISION_VIEW_REQUIRED','unresolved contour extent exceeds the bounded inference uncertainty',dict(audit,empirical_edge_uncertainty_m=edge_uncertainty,empirical_yaw_uncertainty_rad=yaw_uncertainty))
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
        geom_payload = dict(center=np.asarray(geometry.center_base).tolist(), axes=np.asarray(geometry.axes_base).tolist(),
            size=np.asarray(geometry.size_xyz_m).tolist(), support_normal=n.tolist(), support_offset=d,
            support_inlier_ratio=float(geometry.support_inlier_ratio), measured_points_sha256=_array_hash(measured))
        source = dict(target_identity=dict(epoch=source_identity.epoch, track_id=source_identity.track_id),
            stamp_ns=int(source_stamp_ns), depth_sha256=_array_hash(depth), mask_sha256=_array_hash(mask_input),
            color_sha256=_array_hash(color_bgr) if color_bgr is not None else None,
            intrinsics_sha256=_hash(dict(shape=list(depth.shape), K=K.tolist(), depth_scale_m=scale)),
            T_base_camera_sha256=_array_hash(T), geometry_sha256=_hash(geom_payload))
        model = dict(schema_version=2 if outer is not None else 1, kind=KIND, assumption=ASSUMPTION,
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
    return replace(fallback, audit=MappingProxyType(dict(fallback.audit,
        original_qualification_code=result.code, original_qualification_reason=result.audit.get('reason'))))
