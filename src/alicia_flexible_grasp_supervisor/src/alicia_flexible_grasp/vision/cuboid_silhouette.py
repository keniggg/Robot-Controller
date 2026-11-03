"""Source-bound diagnostics for one cuboid's complete visible silhouette.

This module does not qualify a grasp model or authorize motion.  A projected
side is a model explanation, never a measured top-to-side transition.  The
original pixels and admitted measured geometry remain the caller's evidence.
"""
import hashlib
from itertools import product
import json
import math
from numbers import Integral, Real

import cv2
import numpy as np


_SIGNS = np.asarray(tuple(product((-1., 1.), repeat=3)))
_KINDS = ('top', 'support', 'vertical', 'ambiguous')
_RASTER_RADIUS = math.sqrt(8.)


def _array(value, shape, name):
    value = np.asarray(value)
    if value.shape != shape or value.dtype.kind not in 'iuf' or not np.all(np.isfinite(value)):
        raise ValueError(name + ' must have finite numeric values and the expected shape')
    return np.asarray(value, dtype=float)


def _rotation(value, name):
    value = _array(value, (3, 3), name)
    if (not np.allclose(value.T @ value, np.eye(3), rtol=0., atol=1e-7)
            or abs(float(np.linalg.det(value))-1.) > 1e-7):
        raise ValueError(name + ' must be a right-handed rotation')
    return value


def _intrinsics(value):
    if hasattr(value, 'fx'):
        value = [getattr(value, k) for k in ('fx', 'fy', 'cx', 'cy')]
    elif isinstance(value, dict):
        value = [value[k] for k in ('fx', 'fy', 'cx', 'cy')]
    value = _array(value, (4,), 'intrinsics')
    if np.any(value[:2] <= 0.):
        raise ValueError('camera focal lengths must be positive')
    return value


def _transform(value):
    value = _array(value, (4, 4), 'T_base_optical')
    _rotation(value[:3, :3], 'camera rotation')
    if not np.allclose(value[3], [0., 0., 0., 1.], rtol=0., atol=1e-10):
        raise ValueError('camera transform must be homogeneous')
    return value


def _candidate(center, axes, size):
    center = _array(center, (3,), 'center')
    axes = _rotation(axes, 'axes')
    size = _array(size, (3,), 'size')
    if np.any(size <= 0.):
        raise ValueError('cuboid dimensions must be positive')
    return center, axes, size


def _edge_label(start, end):
    changed = np.flatnonzero(_SIGNS[start] != _SIGNS[end])
    if len(changed) != 1:
        return 'ambiguous', None
    axis = int(changed[0])
    if axis == 2:
        return 'vertical', axis
    return ('top' if _SIGNS[start, 2] > 0. else 'support'), axis


def project_cuboid_silhouette(center_base, axes_base, size_xyz_m,
                             T_base_optical, intrinsics):
    """Project all eight *shared* corners and label the actual external hull.

    Corner order is ``product((-1, 1), repeat=3)``.  Image-space collinearity
    can remove intermediate corners; an unresolved composite edge is explicitly
    ``ambiguous`` rather than being renamed a top/support edge.  The arrays in
    this geometric helper are independent copies; diagnostics serialize them.
    """
    center, axes, size = _candidate(center_base, axes_base, size_xyz_m)
    T, K = _transform(T_base_optical), _intrinsics(intrinsics)
    vertices = center + (_SIGNS*(size/2.)) @ axes.T
    optical = (vertices-T[:3, 3]) @ T[:3, :3]
    if np.any(optical[:, 2] <= 0.):
        raise ValueError('all cuboid corners must be in front of the camera')
    pixels = np.column_stack((optical[:, 0]/optical[:, 2]*K[0]+K[2],
                              optical[:, 1]/optical[:, 2]*K[1]+K[3]))
    hull = cv2.convexHull(pixels.astype(np.float32), returnPoints=False)
    if hull is None or len(hull) < 3:
        raise ValueError('cuboid has a degenerate image silhouette')
    indices = [int(i) for i in hull[:, 0]]
    edges = []
    for index, start in enumerate(indices):
        end = indices[(index+1) % len(indices)]
        kind, axis = _edge_label(start, end)
        edges.append(dict(start_vertex=start, end_vertex=end,
            start_pixel=pixels[start].tolist(), end_pixel=pixels[end].tolist(),
            kind=kind, axis=axis))
    return dict(vertices_base=vertices.copy(), vertices_pixel=pixels.copy(),
        optical_depth_m=optical[:, 2].copy(), hull_vertex_indices=indices,
        hull_pixels=pixels[indices].copy(), edges=edges)


def _number(value, name, *, positive=False):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real):
        raise ValueError(name + ' must be numeric')
    value = float(value)
    if not math.isfinite(value) or value < 0. or (positive and value == 0.):
        raise ValueError(name + ' must be finite and nonnegative')
    return value


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'),
                      ensure_ascii=True, allow_nan=False)


def _digest(value):
    return hashlib.sha256(_canonical(value).encode('utf8')).hexdigest()


def _pixel_hash(value):
    return hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()


def _nearest_distances(points, edge_pixels):
    """Exact Euclidean distances to actual raster responses, in small chunks."""
    if not len(edge_pixels):
        return None
    result = []
    for start in range(0, len(points), 16):
        delta = points[start:start+16, None, :]-edge_pixels[None, :, :]
        result.extend(np.sqrt(np.min(np.sum(delta*delta, axis=2), axis=1)).tolist())
    return np.asarray(result)


def _maximum(value):
    return None if value is None or not len(value) else float(np.max(value))


def _supported(value):
    return value is not None and len(value) > 0 and bool(np.max(value) <= _RASTER_RADIUS+1e-10)


def _ray_hits(pixels, K, T, center, axes, size):
    """First-hit faces for pixel-center rays (optical z parameterization)."""
    pixels = np.asarray(pixels, float).reshape((-1, 2))
    optical = np.column_stack(((pixels[:, 0]-K[2])/K[0],
                               (pixels[:, 1]-K[3])/K[1], np.ones(len(pixels))))
    rays = optical @ T[:3, :3].T
    directions = rays @ axes
    camera = (T[:3, 3]-center) @ axes
    half = size/2.
    parallel = np.abs(directions) <= 1e-15
    lower = np.zeros_like(directions)
    upper = np.zeros_like(directions)
    np.divide(-half-camera, directions, out=lower, where=~parallel)
    np.divide(half-camera, directions, out=upper, where=~parallel)
    near, far = np.minimum(lower, upper), np.maximum(lower, upper)
    near[parallel], far[parallel] = -np.inf, np.inf
    impossible = np.any(parallel & (np.abs(camera) > half), axis=1)
    entry, exit_ = np.max(near, axis=1), np.min(far, axis=1)
    hit = ~impossible & np.isfinite(entry) & (entry > 0.) & (entry <= exit_+1e-12)
    axis = np.argmax(near, axis=1)
    sign = np.where(directions[np.arange(len(axis)), axis] < 0., 1, -1)
    face = 2*axis+(sign > 0).astype(int)
    face[~hit] = -1
    point = np.full((len(pixels), 3), np.nan)
    point[hit] = camera+directions[hit]*entry[hit, None]
    return dict(hit=hit, face=face, entry=entry, local_point=point, rays_base=rays)


def _unknown_report(pixels, K, T, center, axes, size, tolerance):
    if not len(pixels):
        return dict(pixels_xy=[], classification=[], nominal_face=[],
                    counts=dict(top=0, side=0, support=0, ambiguous=0, outside=0),
                    ray_policy='center_and_four_half_pixel_corners_with_inherited_metric_face_tolerance')
    samples = [_ray_hits(pixels+offset, K, T, center, axes, size)
               for offset in ((0., 0.), (-.5, -.5), (-.5, .5), (.5, -.5), (.5, .5))]
    hits = np.column_stack([row['hit'] for row in samples])
    faces = np.column_stack([row['face'] for row in samples])
    same = np.all(hits, axis=1) & np.all(faces == faces[:, :1], axis=1)
    # A ray close to two physical faces remains ambiguous even when the five
    # raster rays happen to select the same nominal face.
    for row in samples:
        near = np.abs(np.abs(row['local_point'])-size/2.) <= tolerance
        same &= np.count_nonzero(near, axis=1) == 1
    labels = []
    for index, face in enumerate(faces[:, 0]):
        if not np.any(hits[index]):
            label = 'outside'
        elif not same[index]:
            label = 'ambiguous'
        elif face == 5:
            label = 'top'
        elif face == 4:
            label = 'support'
        else:
            label = 'side'
        labels.append(label)
    return dict(pixels_xy=pixels.astype(int).tolist(), classification=labels,
        nominal_face=faces[:, 0].tolist(),
        counts={key:labels.count(key) for key in ('top', 'side', 'support', 'ambiguous', 'outside')},
        ray_policy='center_and_four_half_pixel_corners_with_inherited_metric_face_tolerance')


def _depth_report(pixels, raw, scale, K, T, center, axes, size, tolerance):
    if not len(pixels):
        return dict(valid_mask_pixels=0, contradiction_count=0, contradictions=[],
                    maximum_first_face_normal_residual_m=None)
    hit = _ray_hits(pixels, K, T, center, axes, size)
    z = raw[pixels[:, 1], pixels[:, 0]].astype(float)*scale
    measured = (T[:3, 3]-center) @ axes+(hit['rays_base'] @ axes)*z[:, None]
    face = hit['face']
    axis = np.maximum(face, 0)//2
    sign = np.where(face % 2, 1., -1.)
    residual = np.abs(measured[np.arange(len(face)), axis]-sign*size[axis]/2.)
    excess = np.max(np.maximum(0., np.abs(measured)-size/2.), axis=1)
    contradiction = ~hit['hit'] | (residual > tolerance) | (excess > tolerance)
    rows = []
    for i in np.flatnonzero(contradiction):
        rows.append(dict(pixel_xy=pixels[i].astype(int).tolist(), raw_depth_m=float(z[i]),
            nominal_first_face=int(face[i]),
            reason='ray_outside_candidate' if not hit['hit'][i] else
                   'measured_point_outside_candidate' if excess[i] > tolerance else
                   'measured_depth_disagrees_with_nominal_first_face',
            first_face_normal_residual_m=None if not hit['hit'][i] else float(residual[i]),
            outside_volume_excess_m=float(excess[i])))
    return dict(valid_mask_pixels=len(pixels), contradiction_count=len(rows), contradictions=rows,
        maximum_first_face_normal_residual_m=_maximum(residual[hit['hit']]),
        policy='nominal_first_face_normal_and_volume_residual_not_a_complete_uncertainty_family')


def diagnose_cuboid_silhouette(color_bgr, object_mask, depth_raw, intrinsics,
        depth_scale_m, T_base_optical, center_base, axes_base, size_xyz_m,
        support_normal_base, support_offset_m, *, source_stamp_ns,
        target_identity, plane_slab_m, pixel_footprint_m):
    """Return diagnostic evidence without altering or qualifying any source.

    The full-image Canny test concerns the *cuboid's* projected outer edges.
    Mask support, including its original top-interior 88% calculation, is
    reported separately.  A segmentation boundary is not required to equal
    the cuboid boundary: real visible barcode sides may extend past the mask.
    Reverse/full-mask 88% figures are reports, not inherited admission rules.
    """
    color, mask_input, depth = map(np.asarray, (color_bgr, object_mask, depth_raw))
    if (depth.ndim != 2 or depth.dtype.kind not in 'iuf' or mask_input.shape != depth.shape
            or mask_input.dtype.kind not in 'biu' or color.dtype != np.uint8
            or color.shape != depth.shape+(3,)):
        raise ValueError('same-frame color, raw depth and real mask must align')
    mask = mask_input > 0
    if not np.any(mask):
        raise ValueError('source mask must contain actual pixels')
    if isinstance(source_stamp_ns, (bool, np.bool_)) or not isinstance(source_stamp_ns, Integral) or source_stamp_ns <= 0:
        raise ValueError('source stamp must be exact positive integer nanoseconds')
    if not isinstance(target_identity, dict):
        target_identity = {key:getattr(target_identity, key) for key in ('epoch', 'track_id')}
    if (set(target_identity) != {'epoch', 'track_id'} or isinstance(target_identity['epoch'], (bool, np.bool_))
            or not isinstance(target_identity['epoch'], Integral) or target_identity['epoch'] < 0
            or not isinstance(target_identity['track_id'], str) or not target_identity['track_id'].strip()):
        raise ValueError('exact target epoch and track are required')
    identity = dict(epoch=int(target_identity['epoch']), track_id=target_identity['track_id'])
    scale = _number(depth_scale_m, 'depth scale', positive=True)
    slab = _number(plane_slab_m, 'plane slab', positive=True)
    footprint = _number(pixel_footprint_m, 'pixel footprint', positive=True)
    normal = _array(support_normal_base, (3,), 'support normal')
    if not np.isclose(np.linalg.norm(normal), 1., rtol=0., atol=1e-7):
        raise ValueError('support normal must be unit')
    if isinstance(support_offset_m, (bool, np.bool_)) or not isinstance(support_offset_m, Real) or not math.isfinite(support_offset_m):
        raise ValueError('support offset must be finite numeric')
    offset = float(support_offset_m)
    center, axes, size = _candidate(center_base, axes_base, size_xyz_m)
    if (not np.allclose(axes[:, 2], normal, rtol=0., atol=1e-7)
            or abs(float(center @ normal)+offset-size[2]/2.) > 1e-7):
        raise ValueError('candidate must rest on its unchanged support plane')
    T, K = _transform(T_base_optical), _intrinsics(intrinsics)
    projected = project_cuboid_silhouette(center, axes, size, T, K)
    canny = cv2.Canny(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY), 50., 100.)
    yy, xx = np.where(canny)
    edge_pixels = np.column_stack((xx, yy))
    edge_rows = []
    for edge in projected['edges']:
        a, b = np.asarray(edge['start_pixel']), np.asarray(edge['end_pixel'])
        full = a+np.outer(np.linspace(0., 1., 101), b-a)
        middle = a+np.outer(np.linspace(.08, .92, 101), b-a)
        full_dist, middle_dist = _nearest_distances(full, edge_pixels), _nearest_distances(middle, edge_pixels)
        edge_rows.append(dict(edge, complete_max_distance_px=_maximum(full_dist),
            central_max_distance_px=_maximum(middle_dist),
            start_arm_max_distance_px=_maximum(None if full_dist is None else full_dist[:13]),
            end_arm_max_distance_px=_maximum(None if full_dist is None else full_dist[-13:]),
            complete_supported=_supported(full_dist), central_supported=_supported(middle_dist)))
    image_height, image_width = mask.shape
    polygon = np.zeros(mask.shape, np.uint8)
    cv2.fillConvexPoly(polygon, np.rint(projected['hull_pixels']).astype(np.int32), 1)
    top = np.zeros(mask.shape, np.uint8)
    top_pixels = projected['vertices_pixel'][_SIGNS[:, 2] > 0.]
    top_hull = cv2.convexHull(top_pixels.astype(np.float32))[:, 0]
    cv2.fillConvexPoly(top, np.rint(top_hull).astype(np.int32), 1)
    top_interior = cv2.erode(top, np.ones((3, 3), np.uint8)) > 0
    intersection = int(np.count_nonzero((polygon > 0) & mask))
    model_pixels, mask_pixels = int(np.count_nonzero(polygon)), int(np.count_nonzero(mask))
    top_coverage = float(np.count_nonzero(top_interior & mask))/max(1, int(top_interior.sum()))
    model_coverage = intersection/max(1, model_pixels)
    explained = intersection/mask_pixels
    contours = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
    boundary = np.concatenate([contour[:, 0] for contour in contours]).astype(float)
    boundary_dist = _nearest_distances(boundary, edge_pixels)
    valid = np.isfinite(depth) & (depth > 0.)
    uy, ux = np.where(mask & ~valid)
    vy, vx = np.where(mask & valid)
    tolerance = slab+footprint
    unknown = _unknown_report(np.column_stack((ux, uy)), K, T, center, axes, size, tolerance)
    measured = _depth_report(np.column_stack((vx, vy)), depth, scale, K, T, center, axes, size, tolerance)
    pixels = projected['hull_pixels']
    inside = bool(np.all(pixels >= 4.) and np.all(pixels <= [image_width-5., image_height-5.]))
    source = dict(stamp_ns=int(source_stamp_ns), target_identity=identity,
        image_shape=list(mask.shape), color_sha256=_pixel_hash(color),
        mask_sha256=_pixel_hash(mask_input), depth_sha256=_pixel_hash(depth),
        intrinsics_sha256=_digest(dict(shape=list(mask.shape), K=K.tolist(), depth_scale_m=scale)),
        T_base_optical_sha256=_pixel_hash(T))
    candidate = dict(center_base=center.tolist(), axes_base=axes.tolist(), size_xyz_m=size.tolist(),
        support_normal_base=normal.tolist(), support_offset_m=offset)
    result = dict(schema_version=1, kind='cuboid_silhouette_diagnostic', diagnostic_only=True,
        qualified_model=None, motion_authorized=False, measured_side_surfaces=False,
        source=source, candidate=candidate, candidate_sha256=_digest(candidate),
        projection=dict(vertices_base=projected['vertices_base'].tolist(),
            vertices_pixel=projected['vertices_pixel'].tolist(), hull_vertex_indices=projected['hull_vertex_indices'],
            hull_pixels=projected['hull_pixels'].tolist(), edges=edge_rows),
        raster=dict(policy='original_full_image_canny_50_100_with_sqrt8_neighbourhood',
            maximum_distance_px=_RASTER_RADIUS, projection_inside_original_4px_margin=inside,
            complete_outer_edges_supported=all(row['complete_supported'] for row in edge_rows),
            physical_edge_assignment_complete=all(row['kind'] != 'ambiguous' for row in edge_rows),
            source_mask_boundary_canny_max_distance_px=_maximum(boundary_dist),
            source_mask_boundary_canny_supported=_supported(boundary_dist)),
        coverage=dict(original_top_interior_mask_coverage_fraction=top_coverage,
            original_top_interior_88pct_supported=top_coverage >= .88,
            model_silhouette_mask_coverage_fraction=model_coverage,
            source_mask_explained_by_model_fraction=explained,
            model_coverage_88pct_report_only=model_coverage >= .88,
            source_explained_88pct_report_only=explained >= .88,
            model_pixels=model_pixels, source_mask_pixels=mask_pixels, intersection_pixels=intersection,
            missing_mask_pixels_inside_model=model_pixels-intersection,
            source_mask_pixels_outside_model=mask_pixels-intersection,
            strict_boundary_equality_required=False),
        inherited_budget=dict(plane_slab_m=slab, pixel_footprint_m=footprint,
            face_diagnostic_tolerance_m=tolerance,
            semantics='inherited_empirical_budget_not_a_certified_error_bound'),
        unknown_pixels=unknown, measured_depth=measured,
        limitations=[
            'One candidate only; no exhaustive common parameter/yaw/height family or aperture U is established.',
            'Full Canny may support an internal print line; complete mask explanation and physical edge assignment remain separate evidence.',
            'Mask coverage reports do not introduce a strict mask-to-cuboid boundary equality admission rule.',
            'Predicted top/side/ambiguous labels never become measured resolved_sides or measured contact points.',
            'Nominal depth residuals do not replace original geometry conflict, registered union, contact or path checks.',
            'Source hashes establish consistency only; the caller must bind this source to its admitted task/target/frame lineage.',
            'No calibration certification, new qualified model, free-space proof or motion authorization is issued.'])
    result['proof_sha256'] = _digest(result)
    return result


def validate_cuboid_silhouette_diagnostic(value, expected_source=None):
    """Check diagnostic digest/source identity, without upgrading its scope."""
    if (not isinstance(value, dict) or value.get('schema_version') != 1
            or value.get('kind') != 'cuboid_silhouette_diagnostic'
            or value.get('diagnostic_only') is not True
            or value.get('qualified_model', 'missing') is not None
            or value.get('motion_authorized') is not False
            or value.get('measured_side_surfaces') is not False):
        raise ValueError('diagnostic scope differs')
    body = dict(value)
    claimed = body.pop('proof_sha256', None)
    if not isinstance(claimed, str) or claimed != _digest(body):
        raise ValueError('diagnostic proof digest differs')
    if expected_source is not None and value.get('source') != expected_source:
        raise ValueError('diagnostic source differs from admitted source')
    return json.loads(_canonical(value))
