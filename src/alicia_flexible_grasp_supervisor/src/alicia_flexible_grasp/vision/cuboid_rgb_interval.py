"""Empirical mixed-plane RGB cuboid intervals; this module grants no motion.

Visible horizontal silhouette edges are back-projected onto their actual top
or verified support plane. Every raster response in each original two-pixel
strip is retained. Vertical edges remain independent complete-hull checks.
"""
import math
import hashlib
import json
import cv2
import numpy as np

from .cuboid_silhouette import project_cuboid_silhouette
from .outer_rgb_rectangle import _trig_range, RGB_CONTOUR_MAX_DISTANCE_PX

POLICY = 'current_cuboid_rgb_joint_interval'


def _array_hash(value):
    return hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()


def _hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def _plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {k: _plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(v) for v in value]
    return value


def _rays(pixels, T, K):
    return np.column_stack(((pixels[:, 0]-K[2])/K[0],
        (pixels[:, 1]-K[3])/K[1], np.ones(len(pixels)))) @ T[:3, :3].T


def _plane(pixels, height, n, d, T, K):
    rays = _rays(np.asarray(pixels, float), T, K)
    distance = (height-d-float(T[:3, 3] @ n))/(rays @ n)
    if np.any(~np.isfinite(distance)) or np.any(distance <= 0.):
        raise ValueError('silhouette rays do not meet the assigned visible plane')
    return T[:3, 3]+rays*distance[:, None]


def _project(points, T, K):
    p = (points-T[:3, 3]) @ T[:3, :3]
    if np.any(p[:, 2] <= 0.):
        raise ValueError('uncertainty envelope behind camera')
    return np.column_stack((p[:, 0]/p[:, 2]*K[0]+K[2], p[:, 1]/p[:, 2]*K[1]+K[3]))


def _model(A, lo, hi, h, d, T, K):
    center = A[:, :2] @ ((lo+hi)/2.)+A[:, 2]*(h/2.-d)
    return project_cuboid_silhouette(center, A, np.r_[hi-lo, h], T, K)


_TOP_MASK_COVERAGE_POLICY = 'measured_top_mask_with_complete_cuboid_rgb_v1'
_COVERAGE_COUNTS = (
    ('silhouette_mask_intersection_pixels', 'silhouette_mask_eroded_polygon_pixels',
     'silhouette_mask_coverage', 'whole'),
    ('top_mask_intersection_pixels', 'top_mask_eroded_polygon_pixels',
     'top_mask_coverage', 'top'),
)


def _mask_coverage_regions(model, shape):
    """Keep full-body diagnostics separate from the measured top predicate."""
    whole = np.zeros(tuple(shape), np.uint8)
    cv2.fillConvexPoly(whole, np.rint(model['hull_pixels']).astype(np.int32), 1)
    # project_cuboid_silhouette uses product((-1, 1), repeat=3) corner order.
    # These are the four shared positive-z corners, not a separately fit top.
    top_pixels = np.asarray(model['vertices_pixel'])[[1, 3, 5, 7]]
    top = np.zeros(tuple(shape), np.uint8)
    hull = cv2.convexHull(top_pixels.astype(np.float32))[:, 0]
    cv2.fillConvexPoly(top, np.rint(hull).astype(np.int32), 1)
    kernel = np.ones((3, 3), np.uint8)
    return dict(whole=cv2.erode(whole, kernel) > 0,
                top=cv2.erode(top, kernel) > 0)


def _mask_coverage_evidence(model, mask):
    regions = _mask_coverage_regions(model, mask.shape)
    value = dict(coverage_policy=_TOP_MASK_COVERAGE_POLICY)
    for numerator, denominator, ratio, region in _COVERAGE_COUNTS:
        count = int(np.count_nonzero(regions[region]))
        intersection = int(np.count_nonzero(regions[region] & mask))
        value.update({numerator: intersection, denominator: count,
                      ratio: intersection/max(1, count)})
    return value


def _validate_mask_coverage(value, model, shape):
    if 'coverage_policy' not in value:
        # Existing models retain their original whole-hull requirement.
        new_fields = {key for row in _COVERAGE_COUNTS for key in row[:3]
                      if key != 'silhouette_mask_coverage'}
        if new_fields & set(value):
            raise ValueError('cuboid top coverage fields require an explicit policy')
        coverage = value['silhouette_mask_coverage']
        if (isinstance(coverage, (bool, np.bool_))
                or not isinstance(coverage, (int, float, np.integer, np.floating))
                or not math.isfinite(float(coverage)) or not 0. <= coverage <= 1.):
            raise ValueError('legacy cuboid silhouette coverage is invalid')
        if coverage < .88:
            raise ValueError('cuboid original qualification budget violated')
        return
    if value['coverage_policy'] != _TOP_MASK_COVERAGE_POLICY:
        raise ValueError('unknown cuboid mask coverage policy')
    regions = _mask_coverage_regions(model, shape)
    for numerator, denominator, ratio, region in _COVERAGE_COUNTS:
        if any(key not in value for key in (numerator, denominator, ratio)):
            raise ValueError('cuboid mask coverage evidence is incomplete')
        count, intersection, coverage = value[denominator], value[numerator], value[ratio]
        if (type(count) is not int or type(intersection) is not int or count <= 0
                or not 0 <= intersection <= count
                or count != int(np.count_nonzero(regions[region]))
                or isinstance(coverage, (bool, np.bool_))
                or not isinstance(coverage, (int, float, np.integer, np.floating))
                or not math.isfinite(float(coverage))
                or abs(float(coverage)-intersection/count) > 1e-12):
            raise ValueError('cuboid mask coverage counts or ratio differ')
    if value['top_mask_coverage'] < .88:
        raise ValueError('complete measured top is not covered by current mask')


def _complete_rgb(model, edge_pixels):
    # Edges outside this original raster neighbourhood cannot satisfy the
    # gate. Cropping only saves pairwise-distance memory, never adds support.
    lower = np.min(model['hull_pixels'], axis=0)-4.
    upper = np.max(model['hull_pixels'], axis=0)+4.
    edge_pixels = edge_pixels[np.all((edge_pixels >= lower) & (edge_pixels <= upper), axis=1)]
    if not len(edge_pixels):
        return [1e6], [1e6]
    distances, corners = [], []
    for row in model['edges']:
        a, b = np.asarray(row['start_pixel']), np.asarray(row['end_pixel'])
        uv = a+np.outer(np.linspace(0., 1., 101), b-a)
        nearest = np.min(np.linalg.norm(uv[:, None, :]-edge_pixels[None, :, :], axis=2), axis=1)
        distances.append(float(nearest.max()))
        corners.append(float(nearest[0]))
    return distances, corners


def _raster_evidence(model, edge_pixels):
    lower = np.min(model['hull_pixels'], axis=0)-4.; upper = np.max(model['hull_pixels'], axis=0)+4.
    points = edge_pixels[np.all((edge_pixels >= lower) & (edge_pixels <= upper), axis=1)]
    if not len(points):
        raise ValueError('complete silhouette source raster evidence unavailable')
    evidence = []
    for index, row in enumerate(model['edges']):
        a, b = np.asarray(row['start_pixel']), np.asarray(row['end_pixel'])
        uv = a+np.outer(np.linspace(0., 1., 101), b-a)
        nearest = np.argmin(np.linalg.norm(uv[:, None, :]-points[None, :, :], axis=2), axis=1)
        evidence.append(dict(edge_index=index, start_vertex=row['start_vertex'], end_vertex=row['end_vertex'],
            plane_kind=row['kind'], axis=row['axis'], corresponding_canny_pixels=points[nearest]))
    return evidence


def _refit_cuboid(lo, hi, height, hu, A, d, T, K, edges, edge_pixels):
    """One bounded nominal refit; raster-family budgets are computed later.

    The optimizer only supplies a proposal. It cannot reduce the analytic
    height/edge/yaw family or replace any final raw-evidence check.
    """
    field = cv2.distanceTransform((edges == 0).astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
    unit = np.asarray([.001, .001, .001, .001, math.radians(1.), .001])
    bound = np.asarray([.006, .006, .012, .012, math.radians(8.), hu])/unit
    # Keep the measured height as the common nominal reference. Its entire
    # continuous uncertainty domain is retained by the analytic strip family.
    bound[5] = 0.
    mid, size = (lo+hi)/2., hi-lo
    def model(x):
        p = x*unit; angle = p[4]
        R = np.asarray([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
        B = np.column_stack((A[:, :2] @ R, A[:, 2]))
        center = A[:, :2] @ (mid+p[:2])
        tangent_mid = center @ B[:, :2]; dims = size+p[2:4]
        return _model(B, tangent_mid-dims/2., tangent_mid+dims/2., height+p[5], d, T, K), B, tangent_mid-dims/2., tangent_mid+dims/2.
    def residual(x):
        m, _, _, _ = model(x); samples = []
        for row in m['edges']:
            a, b = np.asarray(row['start_pixel']), np.asarray(row['end_pixel'])
            samples.append(a+np.outer(np.linspace(0., 1., 31), b-a))
        uv = np.concatenate(samples); lower = np.floor(uv).astype(int); frac = uv-lower
        if np.any(lower < 0) or np.any(lower >= np.asarray([field.shape[1]-1, field.shape[0]-1])):
            raise ValueError('bounded refit leaves current image')
        x0, y0 = lower[:, 0], lower[:, 1]; dx, dy = frac[:, 0], frac[:, 1]
        value = ((1-dx)*(1-dy)*field[y0, x0]+dx*(1-dy)*field[y0, x0+1]+
                 (1-dx)*dy*field[y0+1, x0]+dx*dy*field[y0+1, x0+1])
        return np.r_[value, .015*x]
    x = np.zeros(6); r = residual(x); candidates = [x.copy()]
    for _ in range(30):
        columns = []
        for j in range(6):
            step = np.zeros(6); step[j] = .01
            rp, rm = residual(np.clip(x+step, -bound, bound)), residual(np.clip(x-step, -bound, bound))
            if len(rp) != len(r) or len(rm) != len(r):
                raise ValueError('silhouette visibility changed ambiguously during bounded refit')
            columns.append((rp-rm)/.02)
        J = np.column_stack(columns)
        delta = np.linalg.lstsq(np.r_[J, .01*np.eye(6)], np.r_[-r, np.zeros(6)], rcond=None)[0]
        delta /= max(1., float(np.max(np.abs(delta))))
        improved = False
        for fraction in (1., .5, .25, .125, .0625):
            other = np.clip(x+fraction*delta, -bound, bound); rr = residual(other)
            if len(rr) == len(r) and float(rr @ rr) < float(r @ r)-1e-8:
                x, r, improved = other, rr, True; candidates.append(x.copy()); break
        if not improved:
            break
    rows = []
    for candidate in candidates:
        m, B, lower, upper = model(candidate); distances, corners = _complete_rgb(m, edge_pixels)
        rows.append(dict(model=m, axes=B, lower=lower, upper=upper,
            height=height+candidate[5]*unit[5], score=max(max(distances), max(corners)), parameters=candidate*unit))
    best = min(rows, key=lambda row: row['score'])
    best['height_budget'] = hu+abs(best['height']-height)
    return best


def _height_budget(A, lo, hi, h, d, origin, tn, slab, edge=0., yaw=0.):
    size = hi-lo
    half = []
    for j in range(2):
        turn = min(yaw, math.atan2(size[1-j], size[j]))
        half.append((size[j]*math.cos(turn)+size[1-j]*math.sin(turn))/2.+edge)
    mid = (lo+hi)/2.
    xy = mid+np.asarray([[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]])*half
    nominal = xy @ A[:, :2].T+A[:, 2]*(h-d)
    vertical = float(np.max(np.abs((nominal-origin) @ tn)))/float(tn @ A[:, 2])
    return max(.001, vertical+slab/float(tn @ A[:, 2])), vertical


def _groups(model, edge_pixels, T, K, A, n, d, h, hu, retained_groups=None):
    groups = [None]*4
    center_xy = np.mean(model['vertices_base'], axis=0) @ A[:, :2]
    for edge_index, row in enumerate(model['edges']):
        if row['kind'] == 'vertical':
            continue
        if row['kind'] not in ('top', 'support') or row['axis'] not in (0, 1):
            raise ValueError('ambiguous silhouette plane correspondence remains unresolved')
        a, b = np.asarray(row['start_pixel']), np.asarray(row['end_pixel'])
        v = b-a
        along = (edge_pixels-a) @ v/float(v @ v)
        across_px = np.abs(np.cross(v, edge_pixels-a))/np.linalg.norm(v)
        pixels = edge_pixels[(along >= .08) & (along <= .92) & (across_px <= 2.)]
        if len(pixels) < 12:
            raise ValueError('too few distinct complete outer-edge raster responses')
        height = h if row['kind'] == 'top' else 0.
        metric = _plane(pixels, height, n, d, T, K) @ A[:, :2]
        ac = 1-row['axis']
        upper = float(metric[:, ac].mean()) > center_xy[ac]
        side = (1 if upper else 3) if ac == 0 else (2 if upper else 0)
        if groups[side] is not None:
            raise ValueError('multiple competing silhouette plane assignments require a family')
        proposal = dict(axes_base=A, lower=np.min(model['vertices_base'] @ A[:, :2], axis=0),
            upper=np.max(model['vertices_base'] @ A[:, :2], axis=0), nominal_height_m=h,
            hull_edge_index=edge_index, start_vertex=row['start_vertex'], end_vertex=row['end_vertex'],
            plane=row['kind'], parallel_axis=row['axis'])
        proposals = [proposal]
        if retained_groups is not None:
            retained = retained_groups[side]
            if retained['plane'] != row['kind']:
                raise ValueError('seed and final proposals have unresolved top/support correspondence')
            proposals.extend(retained['raster_proposals'])
            pixels = np.unique(np.concatenate((pixels, np.asarray(retained['line_pixels'], int))), axis=0)
            pixels = pixels[np.lexsort((pixels[:, 0], pixels[:, 1]))]
            metric = _plane(pixels, height, n, d, T, K) @ A[:, :2]
        centered = metric-metric.mean(0)
        _, _, vt = np.linalg.svd(centered, full_matrices=False)
        normal = np.asarray([-vt[0, 1], vt[0, 0]])
        residual = float(np.max(np.abs(centered @ normal)))
        sampling = 0.
        for dx, dy in ((-.5, -.5), (-.5, .5), (.5, -.5), (.5, .5)):
            other = _plane(pixels+[dx, dy], height, n, d, T, K) @ A[:, :2]
            sampling = max(sampling, float(np.max(np.abs(other-metric))))
        sampling *= math.cos(math.radians(8.))+math.sin(math.radians(8.))
        height_shift = 0.
        if row['kind'] == 'top':
            for dh in (-hu, hu):
                other = _plane(pixels, height+dh, n, d, T, K) @ A[:, :2]
                displacement = other-metric
                for dx, dy in displacement:
                    low, high = (_trig_range(dx, dy, -math.radians(8.), math.radians(8.)) if ac == 0 else
                                 _trig_range(dy, -dx, -math.radians(8.), math.radians(8.)))
                    height_shift = max(height_shift, abs(low), abs(high))
        groups[side] = dict(side=side, plane=row['kind'], plane_height_m=height,
            source_hull_edge_index=edge_index, source_start_vertex=row['start_vertex'], source_end_vertex=row['end_vertex'],
            source_parallel_axis=row['axis'],
            raster_proposals=proposals,
            line_pixels=pixels, metric_points=metric, maximum_line_residual_m=residual,
            sampling_bound_m=sampling, height_projection_bound_m=height_shift,
            strip_radius_m=residual+sampling+height_shift)
    if any(row is None for row in groups):
        raise ValueError('all four footprint boundaries need an unambiguous top/support edge')
    return groups


def _positive_rotation_support(x, y, angle):
    turn = min(angle, math.atan2(y, x))
    return x*math.cos(turn)+y*math.sin(turn)


def _reference_joint(groups, lo, hi):
    result = _joint(groups, fixed_theta=0.)
    errors = np.asarray([max(result['position_intervals'][i]['error_m'] for i in (1, 3)),
                         max(result['position_intervals'][i]['error_m'] for i in (0, 2))])
    shift = np.abs((result['lower']+result['upper']-lo-hi)/2.)
    size = np.abs(result['upper']-result['lower']-(hi-lo))/2.
    yaw = result['yaw_uncertainty']
    axis = np.asarray([max(.001, _positive_rotation_support(errors[i], errors[1-i], yaw)+shift[i]+
        _positive_rotation_support(size[i], size[1-i], yaw)) for i in (0, 1)])
    result.update(axis_uncertainty=axis, edge_uncertainty=float(axis.max()),
        axis_position_error_m=errors, nominal_reference_center_offset_m=shift,
        nominal_reference_half_size_offset_m=size)
    return result


def _assignment_stability(A, lo, hi, h, d, T, axis, yaw, hu):
    """Require one physical top/support visibility assignment for the family."""
    mid, size = (lo+hi)/2., hi-lo
    camera = T[:3, 3] @ A[:, :2]-mid
    if float(T[:3, 3] @ A[:, 2])+d <= h+hu:
        raise ValueError('common cuboid height family does not remain below the camera')
    states = []
    for j in (0, 1):
        low, high = (_trig_range(camera[0], camera[1], -yaw, yaw) if j == 0 else
                     _trig_range(camera[1], -camera[0], -yaw, yaw))
        uncertainty = _positive_rotation_support(axis[j], axis[1-j], yaw)
        if low > size[j]/2.+uncertainty:
            states.append('outside_positive')
        elif high < -size[j]/2.-uncertainty:
            states.append('outside_negative')
        elif low > -size[j]/2.+uncertainty and high < size[j]/2.-uncertainty:
            states.append('inside')
        else:
            raise ValueError('common cuboid family has unresolved top/support silhouette visibility')
    return states


_VISIBILITY_FAMILY_POLICY = 'complete_rgb_nominal_anchor_and_all_plane_assignment_enclosures_v1'


def _line_yaw_interval(points, radius, side):
    """Necessary common-yaw interval from every ordered raw-response pair.

    Horizontal line normals are unoriented, with periodic alternatives pi
    apart. A pair admitting two pieces of the original +/-8 degree domain
    retains their convex enclosure. No least-squares angle or fixed-zero fit
    is used to declare a branch empty.
    """
    lower, upper = -math.radians(8.), math.radians(8.)
    for start in range(0, len(points), 256):
        for other in range(0, len(points), 256):
            diff = points[start:start+256, None, :]-points[None, other:other+256, :]
            length = np.linalg.norm(diff, axis=2)
            active = length > 2.*radius+1e-15
            diff, length = diff[active], length[active]
            if not len(length):
                continue
            angle = np.arctan2(diff[:, 1], diff[:, 0])-(math.pi/2. if side % 2 else 0.)
            angle = (angle+math.pi/2.) % math.pi-math.pi/2.
            arc = np.arcsin(2.*radius/length)
            # A pair near the +/-pi/2 wrap can admit two disjoint pieces of
            # the bounded domain. Do not discard either piece: its convex
            # enclosure is the full original domain, a conservative bound.
            wrapped = ((angle-arc <= -math.pi+math.radians(8.)+1e-12) |
                       (angle+arc >= math.pi-math.radians(8.)-1e-12))
            if np.any(wrapped):
                angle, arc = angle[~wrapped], arc[~wrapped]
            if len(angle):
                lower = max(lower, float(np.max(angle-arc)))
                upper = min(upper, float(np.min(angle+arc)))
    # A weak group may contribute no angular restriction. The other groups
    # still constrain the common box; lack of information is not emptiness.
    return [lower, upper]


def _strip_projection_interval(points, radius, side, lower, upper):
    extrema = [_trig_range(p[0], p[1], lower, upper) if side % 2 else
               _trig_range(p[1], -p[0], lower, upper) for p in points]
    return [max(p[0] for p in extrema)-radius,
            min(p[1] for p in extrema)+radius]


def _visibility_family(groups, A, lo, hi, h, d, T, K, hu, minimum_axis, minimum_yaw):
    """Enclose every physical top/support interpretation of the same strips.

    The already checked complete-RGB cuboid is the nominal anchor, not a claim
    that this anchor satisfies every response band. Each alternative is a
    necessary-condition enclosure, not an additional accepted pose. Keeping
    infeasible alternatives is conservative; choosing one best branch is not.
    """
    if float(T[:3, 3] @ A[:, 2])+d <= h+hu:
        raise ValueError('common cuboid height family does not remain below the camera')
    n, mid, size = A[:, 2], (lo+hi)/2., hi-lo
    camera = T[:3, 3] @ A[:, :2]
    # The original empirical position/yaw enclosure includes its 1 mm and
    # 0.25 degree floors. Raw strip signs cannot erase that prior envelope.
    # Preserve it as a union, not a second additive downstream allowance.
    prior_signs = []
    delta_camera = mid-camera
    for side in range(4):
        j = 0 if side % 2 else 1
        bounds = (_trig_range(delta_camera[0], delta_camera[1], -minimum_yaw, minimum_yaw)
                  if j == 0 else _trig_range(delta_camera[1], -delta_camera[0], -minimum_yaw, minimum_yaw))
        nominal = (-1. if side in (0, 3) else 1.)*size[j]/2.
        uncertainty = _positive_rotation_support(minimum_axis[j], minimum_axis[1-j], minimum_yaw)
        prior_signs.append([bounds[0]+nominal-uncertainty, bounds[1]+nominal+uncertainty])
    projected = []
    for side, source in enumerate(groups):
        pixels = np.asarray(source['line_pixels'], float)
        choices = {}
        for plane in ('top', 'support'):
            height = h if plane == 'top' else 0.
            points = _plane(pixels, height, n, d, T, K) @ A[:, :2]
            centered = points-points.mean(0)
            _, _, vt = np.linalg.svd(centered, full_matrices=False)
            residual = float(np.max(np.abs(centered @ [-vt[0, 1], vt[0, 0]])))
            sampling = 0.
            for dx, dy in ((-.5, -.5), (-.5, .5), (.5, -.5), (.5, .5)):
                other = _plane(pixels+[dx, dy], height, n, d, T, K) @ A[:, :2]
                sampling = max(sampling, float(np.max(np.abs(other-points))))
            sampling *= math.cos(math.radians(8.))+math.sin(math.radians(8.))
            shift = 0.
            if plane == 'top':
                for dh in (-hu, hu):
                    other = _plane(pixels, height+dh, n, d, T, K) @ A[:, :2]
                    for delta in other-points:
                        bounds = (_trig_range(delta[0], delta[1], -math.radians(8.), math.radians(8.))
                                  if side % 2 else _trig_range(delta[1], -delta[0], -math.radians(8.), math.radians(8.)))
                        shift = max(shift, abs(bounds[0]), abs(bounds[1]))
            radius = residual+sampling+shift
            choices[plane] = dict(plane=plane, metric_points=points,
                maximum_line_residual_m=residual, sampling_bound_m=sampling,
                height_projection_bound_m=shift, strip_radius_m=radius,
                common_yaw_interval_rad=_line_yaw_interval(points, radius, side))
        projected.append(choices)
    branches, retained = [], []
    states = ('inside', 'outside_negative', 'outside_positive')
    for xstate in states:
        for ystate in states:
            planes = ['top']*4
            for state, lower_side, upper_side in ((xstate, 3, 1), (ystate, 0, 2)):
                if state == 'outside_negative':
                    planes[lower_side] = 'support'
                elif state == 'outside_positive':
                    planes[upper_side] = 'support'
            rows = [projected[i][plane] for i, plane in enumerate(planes)]
            lower = max(row['common_yaw_interval_rad'][0] for row in rows)
            upper = min(row['common_yaw_interval_rad'][1] for row in rows)
            branch = dict(states=[xstate, ystate], planes=planes,
                raw_common_yaw_interval_rad=[lower, upper], retained=False)
            branches.append(branch)
            if lower > upper+1e-12:
                branch['rejection'] = 'all_pair_yaw_intersection_empty'
                continue
            # Keep the original empirical angular floor in every alternative.
            lower, upper = min(lower, -math.radians(.25)), max(upper, math.radians(.25))
            raw_relative = [_strip_projection_interval(row['metric_points']-camera,
                row['strip_radius_m'], i, lower, upper) for i, row in enumerate(rows)]
            relative = [[min(raw[0], prior[0]), max(raw[1], prior[1])]
                        for raw, prior in zip(raw_relative, prior_signs)]
            branch.update(enclosed_yaw_interval_rad=[lower, upper],
                raw_camera_relative_face_intervals_m=raw_relative,
                camera_relative_face_intervals_m=relative)
            impossible = []
            for i, (plane, bounds) in enumerate(zip(planes, relative)):
                # Closed inequalities retain both adjacent interpretations at
                # a tangent view. Only a whole-interval sign contradiction
                # proves an alternative impossible.
                positive = (plane == 'support') if i in (0, 3) else (plane == 'top')
                if (positive and bounds[1] < -1e-12) or (not positive and bounds[0] > 1e-12):
                    impossible.append(i)
            if impossible:
                branch.update(rejection='camera_side_sign_contradiction', contradicted_sides=impossible)
                continue
            intervals = [_strip_projection_interval(row['metric_points']-mid,
                row['strip_radius_m'], i, lower, upper) for i, row in enumerate(rows)]
            if any(bounds[0] > bounds[1]+1e-12 for bounds in intervals):
                branch.update(rejection='face_position_intersection_empty')
                continue
            error = np.zeros(2)
            for i, bounds in enumerate(intervals):
                axis = 0 if i % 2 else 1
                nominal = (-1. if i in (0, 3) else 1.)*size[axis]/2.
                error[axis] = max(error[axis], abs(bounds[0]-nominal), abs(bounds[1]-nominal))
            yaw = max(-lower, upper)
            bound = np.asarray([max(.001, _positive_rotation_support(error[i], error[1-i], yaw)) for i in (0, 1)])
            branch.update(retained=True, face_intervals_about_nominal_center_m=intervals,
                axis_face_error_m=error, axis_uncertainty=bound, yaw_uncertainty=yaw)
            retained.append(branch)
    if not retained:
        raise ValueError('no possible common cuboid plane-assignment enclosure')
    axis = np.maximum(np.asarray(minimum_axis), np.max([row['axis_uncertainty'] for row in retained], axis=0))
    yaw = max(float(minimum_yaw), max(row['yaw_uncertainty'] for row in retained))
    return dict(policy=_VISIBILITY_FAMILY_POLICY, nominal_anchor_membership_claimed=False,
        interpretation='all_retained_necessary_condition_enclosures_not_individually_certified_poses',
        prior_empirical_camera_relative_face_intervals_m=prior_signs,
        projection_groups=projected, branches=branches, retained_branch_count=len(retained),
        axis_uncertainty=axis, yaw_uncertainty=yaw)


def _same_family_evidence(actual, expected):
    """Compare independently rebuilt arithmetic with only roundoff allowance."""
    if isinstance(expected, dict):
        return isinstance(actual, dict) and set(actual) == set(expected) and all(
            _same_family_evidence(actual[key], item) for key, item in expected.items())
    if isinstance(expected, list):
        return isinstance(actual, list) and len(actual) == len(expected) and all(
            _same_family_evidence(a, b) for a, b in zip(actual, expected))
    if isinstance(expected, float):
        return (isinstance(actual, (int, float)) and not isinstance(actual, bool)
                and math.isfinite(actual) and abs(actual-expected) <= 1e-9)
    return type(actual) is type(expected) and actual == expected


def _proposal_window(model, seed_A, seed_lo, seed_hi, pixel):
    """Check the original line-search radius and angle in the seed frame."""
    maximum_offset, maximum_angle = 0., 0.
    for row in model['edges']:
        if row['kind'] == 'vertical':
            continue
        if row['kind'] not in ('top', 'support') or row['axis'] not in (0, 1):
            raise ValueError('bounded proposal has unresolved physical outer edge')
        ac, al = 1-row['axis'], row['axis']
        p = model['vertices_base'][[row['start_vertex'], row['end_vertex']]] @ seed_A[:, :2]
        delta = p[1]-p[0]; angle = math.atan2(abs(delta[ac]), abs(delta[al]))
        midpoint = float(p[:, ac].mean()); reference = float((seed_lo[ac]+seed_hi[ac])/2.)
        limit = seed_hi[ac] if midpoint > reference else seed_lo[ac]
        offset = abs(midpoint-limit)
        maximum_angle, maximum_offset = max(maximum_angle, angle), max(maximum_offset, offset)
        if angle > math.radians(8.)+1e-12 or offset > max(.012, 8.*pixel)+1e-12:
            raise ValueError('cuboid proposal exceeds original angle or position search radius')
    return dict(maximum_angle_rad=maximum_angle, maximum_face_offset_m=maximum_offset)


def _joint(groups, fixed_theta=None):
    cov = []
    for row in groups:
        p = np.asarray(row['metric_points']); centered = p-p.mean(0)
        cov.append(centered.T @ centered/len(p))
    J = np.asarray([[0., -1.], [1., 0.]])
    _, eig = np.linalg.eigh(cov[1]+cov[3]+J.T @ (cov[0]+cov[2]) @ J)
    u = eig[:, 0]
    if u[0] < 0.:
        u = -u
    theta = math.atan2(u[1], u[0]) if fixed_theta is None else float(fixed_theta)
    rotation = np.asarray([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    lower_angle, upper_angle = -math.radians(8.), math.radians(8.)
    for i, row in enumerate(groups):
        p = np.asarray(row['metric_points']); delta = row['strip_radius_m']
        constrained = False
        # Keep every ordered pair and the original interval arithmetic, while
        # bounding intermediate allocation independently of response count.
        # This is a memory bound, not a sampling or physical evidence limit.
        for start in range(0, len(p), 256):
            for other in range(0, len(p), 256):
                differences = p[start:start+256, None, :]-p[None, other:other+256, :]
                length = np.linalg.norm(differences, axis=2)
                active = length > 2.*delta+1e-15
                dif, lengths = differences[active], length[active]
                if not len(lengths):
                    continue
                constrained = True
                angle = np.arctan2(dif[:, 1], dif[:, 0])-(math.pi/2. if i % 2 else 0.)
                angle = (angle-theta+math.pi/2.) % math.pi-math.pi/2.+theta
                arc = np.arcsin(2.*delta/lengths)
                lower_angle = max(lower_angle, float(np.max(angle-arc)))
                upper_angle = min(upper_angle, float(np.min(angle+arc)))
        if not constrained:
            raise ValueError('outer response family cannot constrain a common orientation')
    if not lower_angle <= theta <= upper_angle:
        raise ValueError('four complete mixed-plane lines have no common orthogonal interval')
    offsets = []
    for i, row in enumerate(groups):
        p = np.asarray(row['metric_points']) @ rotation[:, 1 if i % 2 == 0 else 0]
        offsets.append(float((p.max()+p.min())/2.))
    lo, hi = np.asarray([offsets[3], offsets[0]]), np.asarray([offsets[1], offsets[2]])
    if np.any(hi-lo < .005) or np.any(hi-lo > .6):
        raise ValueError('mixed-plane rectangle extent invalid')
    mid = (lo+hi)/2.; center = rotation @ mid
    intervals, error = [], 0.
    for i, row in enumerate(groups):
        p = np.asarray(row['metric_points'])-center; ac = 1 if i % 2 == 0 else 0
        extrema = [_trig_range(v[0], v[1], lower_angle, upper_angle) if ac == 0 else
                   _trig_range(v[1], -v[0], lower_angle, upper_angle) for v in p]
        low = max(v[0] for v in extrema)-row['strip_radius_m']
        high = min(v[1] for v in extrema)+row['strip_radius_m']
        nominal = offsets[i]-mid[ac]; err = max(abs(low-nominal), abs(high-nominal))
        if low > high+1e-12:
            raise ValueError('current edge response strips have empty position intersection')
        error = max(error, err)
        intervals.append(dict(minimum_m=low, maximum_m=high, nominal_m=nominal, error_m=err))
    yaw = max(math.radians(.25), theta-lower_angle, upper_angle-theta)
    edge = max(.001, error*(math.cos(yaw)+math.sin(yaw)))
    return dict(rotation=rotation, lower=lo, upper=hi, theta=theta,
        common_yaw_interval_rad=[lower_angle, upper_angle], position_intervals=intervals,
        position_family_error_m=error, edge_uncertainty=edge, yaw_uncertainty=yaw)


def _exterior(depth, scale, T, K, origin, tn, A, lo, hi, edge, yaw, pixel):
    size, mid = hi-lo, (hi+lo)/2.; half = []; edge = np.broadcast_to(np.asarray(edge), (2,))
    for j in range(2):
        turn = min(yaw, math.atan2(size[1-j], size[j]))
        half.append((size[j]*math.cos(turn)+size[1-j]*math.sin(turn))/2.+edge[j])
    half = np.asarray(half); margin = max(.003, 3.*pixel)
    xy = mid+np.asarray([[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]])*(half+margin)
    p = xy @ A[:, :2].T
    p += np.outer(((origin-p) @ tn)/(tn @ A[:, 2]), A[:, 2])
    pixels = _project(p, T, K)
    if np.any(pixels < 1.) or np.any(pixels > np.asarray([depth.shape[1]-2, depth.shape[0]-2])):
        raise ValueError('complete cuboid uncertainty exterior is not visible')
    x0, y0 = np.floor(pixels.min(0)).astype(int); x1, y1 = np.ceil(pixels.max(0)).astype(int)+1
    gy, gx = np.mgrid[y0:y1, x0:x1]; grid = np.column_stack((gx.ravel(), gy.ravel()))
    ray = _rays(grid, T, K); distance = float((origin-T[:3, 3]) @ tn)/(ray @ tn)
    on_plane = T[:3, 3]+ray*distance[:, None]; projected = on_plane @ A[:, :2]-mid
    zz = depth[grid[:, 1], grid[:, 0]].astype(float)*scale; valid = np.isfinite(zz) & (zz > 0.)
    measured = T[:3, 3]+ray*zz[:, None]; residual = (measured-origin) @ tn
    rows = []
    for i in range(4):
        ac = 1 if i % 2 == 0 else 0; al = 1-ac; sign = -1. if i in (0, 3) else 1.
        outward = sign*projected[:, ac]-half[ac]
        strip = (outward > 0.) & (outward <= margin) & (np.abs(projected[:, al]) <= size[al]/2.)
        count = int(strip.sum()); known = int(np.count_nonzero(strip & valid))
        if not count:
            raise ValueError('complete cuboid exterior strip is empty')
        fraction = known/count
        continued = float(np.count_nonzero(strip & valid & (np.abs(residual) < .0015)))/max(1, known)
        occluding = float(np.count_nonzero(strip & valid & (residual > .0025)))/count
        rows.append(dict(pixels=count, valid=known, valid_fraction=fraction, continuation=continued, occluding=occluding))
        if fraction >= .20 and continued > .30:
            raise ValueError('measured top continues outside complete cuboid uncertainty family')
        if occluding > .30:
            raise ValueError('external occluder truncates complete cuboid uncertainty family')
    return rows


def _unknown_mask_rays(pixels_yx, T, K, A, lo, hi, h, d, edge, yaw, hu):
    """Possible occupancy only; unknown rays never become measured faces."""
    pixels = np.asarray(pixels_yx, int).reshape(-1, 2)[:, ::-1]
    size, mid = hi-lo, (hi+lo)/2.; half = []; edge = np.broadcast_to(np.asarray(edge), (2,))
    for j in range(2):
        turn = min(yaw, math.atan2(size[1-j], size[j]))
        half.append((size[j]*math.cos(turn)+size[1-j]*math.sin(turn))/2.+edge[j])
    # The physical family rests on the verified support: uncertainty extends
    # its upper cap only, never invents occupied volume below that plane.
    center = A[:, :2] @ mid+A[:, 2]*((h+hu)/2.-d)
    half = np.r_[half, (h+hu)/2.]
    camera = (T[:3, 3]-center) @ A
    compatible = np.ones(len(pixels), bool)
    for shift in ((0., 0.), (-.5, -.5), (-.5, .5), (.5, -.5), (.5, .5)):
        rays = _rays(pixels+shift, T, K) @ A
        with np.errstate(divide='ignore', invalid='ignore'):
            t0, t1 = (-half-camera)/rays, (half-camera)/rays
        near = np.maximum(np.minimum(t0, t1).max(1), 0.)
        far = np.maximum(t0, t1).min(1)
        compatible &= np.isfinite(near) & (far >= near)
    return compatible


def infer_cuboid_rgb_interval(color, mask, depth, scale, T, K, origin, tn,
                              axes, pixel_size, rms, support_offset, top_height):
    """Return a source-image empirical interval; never an execution approval."""
    audit = dict(policy=POLICY, certified_error_bound=False,
        uncertainty_semantics='empirical_budget_not_certified_error_bound')
    def reject(reason):
        return dict(ok=False, code='BOX_INFERENCE_PRECISION_VIEW_REQUIRED', reason=reason, audit=_plain(audit))
    try:
        color, depth, mask_input = np.asarray(color), np.asarray(depth), np.asarray(mask)
        mask = mask_input > 0
        if color.dtype != np.uint8 or color.shape != depth.shape+(3,) or mask.shape != depth.shape:
            raise ValueError('exact same-source BGR/depth/mask required')
        K = np.asarray([getattr(K, name) for name in ('fx', 'fy', 'cx', 'cy')] if hasattr(K, 'fx') else K, float)
        T, A, n = np.asarray(T, float), np.asarray(axes, float), np.asarray(axes, float)[:, 2]
        origin, tn = np.asarray(origin, float), np.asarray(tn, float)
        if K.shape != (4,) or not np.all(np.isfinite(K)) or min(K[:2]) <= 0. or not 0. < scale <= 1.:
            raise ValueError('calibration invalid')
        if (T.shape != (4, 4) or A.shape != (3, 3) or origin.shape != (3,) or tn.shape != (3,) or
                not np.all(np.isfinite(T)) or not np.all(np.isfinite(A)) or not np.all(np.isfinite(origin)) or
                not np.all(np.isfinite(tn)) or not np.allclose(T[3], [0., 0., 0., 1.], rtol=0., atol=1e-10) or
                not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), rtol=0., atol=1e-7) or
                abs(float(np.linalg.det(T[:3, :3]))-1.) > 1e-7 or not np.allclose(A.T @ A, np.eye(3), rtol=0., atol=1e-7) or
                abs(float(np.linalg.det(A))-1.) > 1e-7 or abs(float(tn @ tn)-1.) > 1e-7 or
                float(tn @ n) < math.cos(math.radians(6.)) or not math.isfinite(float(rms)) or rms < 0. or rms > .0015):
            raise ValueError('verified rigid geometry or measured top plane invalid')
        yy, xx = np.where(mask & np.isfinite(depth) & (depth > 0.))
        z = depth[yy, xx].astype(float)*scale
        points = T[:3, 3]+_rays(np.column_stack((xx, yy)), T, K)*z[:, None]
        top = points[(np.abs((points-origin) @ tn) <= .0015) & (points @ n+support_offset >= .5*top_height)]
        if len(top) < 80:
            raise ValueError('too few directly measured top points')
        coord = top @ A[:, :2]; lo, hi = coord.min(0), coord.max(0)
        slab = max(.001, 2.*float(rms)+float(scale)); hu, vertical = _height_budget(A, lo, hi, top_height, support_offset, origin, tn, slab)
        edges = cv2.Canny(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY), 50., 100.)
        ey, ex = np.where(edges); edge_pixels = np.column_stack((ex, ey))
        if len(edge_pixels) < 12:
            raise ValueError('complete current RGB silhouette unavailable')
        seed = _model(A, lo, hi, top_height, support_offset, T, K)
        groups = _groups(seed, edge_pixels, T, K, A, n, support_offset, top_height, hu)
        seed_groups = groups
        audit.update(seed_axes_base=A, seed_lower=lo, seed_upper=hi, seed_nominal_height_m=top_height)
        # Check the four-group common interval before attempting a refit;
        # optimization may not rescue inconsistent competing raster evidence.
        _joint(groups)
        fitted = _refit_cuboid(lo, hi, top_height, hu, A, support_offset, T, K, edges, edge_pixels)
        nominal_A = fitted['axes']; nominal_lo, nominal_hi = fitted['lower'], fitted['upper']
        original_height = float(top_height); original_hu = hu
        top_height = fitted['height']; hu = fitted['height_budget']
        # Exactly one bounded refit from the original four response groups.
        if abs(fitted['parameters'][4]) > math.radians(8.) or np.max(np.abs(nominal_hi-nominal_lo-(hi-lo))) > .024:
            raise ValueError('refitted cuboid exceeds original bounded line-search window')
        model = fitted['model']
        proposal_window = _proposal_window(model, A, lo, hi, pixel_size)
        full_distances, corner_distances = _complete_rgb(model, edge_pixels)
        audit.update(full_silhouette_rgb_distances_px=full_distances, corner_rgb_distances_px=corner_distances,
            refit_count=1, nominal_height_m=top_height, top_plane_slab_m=slab,
            bounded_refit_parameters=fitted['parameters'],
            original_measured_top_height_m=original_height,
            original_common_height_interval_m=[original_height-original_hu, original_height+original_hu])
        audit['bounded_proposal_window'] = proposal_window
        if max(full_distances) > RGB_CONTOUR_MAX_DISTANCE_PX or max(corner_distances) > RGB_CONTOUR_MAX_DISTANCE_PX:
            raise ValueError('complete cuboid silhouette or corner arms exceed original raster gate')
        audit.update(_mask_coverage_evidence(model, mask))
        if audit['top_mask_coverage'] < .88:
            raise ValueError('complete measured top is not covered by current mask')
        # Monotone height/position enclosure. Each recomputation retains all
        # original raster responses; this is not another pose fit.
        result = None
        for _ in range(16):
            groups = _groups(model, edge_pixels, T, K, nominal_A, n, support_offset, top_height, hu, retained_groups=seed_groups)
            result = _reference_joint(groups, nominal_lo, nominal_hi)
            new_hu, vertical = _height_budget(nominal_A, nominal_lo, nominal_hi, top_height,
                support_offset, origin, tn, slab, result['edge_uncertainty'], result['yaw_uncertainty'])
            if new_hu <= hu+1e-13:
                # Round outward before publishing the model. The unchanged
                # raster groups tolerate this sub-picometre closure increment;
                # no caller receives a height budget below its required slab.
                hu = max(hu, new_hu)+1e-13
                break
            hu = new_hu
        else:
            raise ValueError('common cuboid height/position family did not close conservatively')
        edge, yaw, axis = result['edge_uncertainty'], result['yaw_uncertainty'], result['axis_uncertainty']
        if edge > .01 or yaw > math.radians(12.) or top_height <= 2.*hu+.002:
            raise ValueError('complete cuboid family exceeds existing precision budget')
        visibility_family = None
        try:
            stability = _assignment_stability(nominal_A, nominal_lo, nominal_hi, top_height,
                support_offset, T, axis, yaw, hu)
        except ValueError as exc:
            if str(exc) != 'common cuboid family has unresolved top/support silhouette visibility':
                raise
            # A near-tangent silhouette can have two physical plane labels
            # while both explanations fit a small metric uncertainty family.
            # Preserve every possible assignment and use one common enclosure
            # for subsequent opening, contact and collision checks.
            minimum_axis, minimum_yaw = axis.copy(), yaw
            for _ in range(16):
                groups = _groups(model, edge_pixels, T, K, nominal_A, n, support_offset,
                    top_height, hu, retained_groups=seed_groups)
                result = _reference_joint(groups, nominal_lo, nominal_hi)
                minimum_axis = np.maximum(minimum_axis, result['axis_uncertainty'])
                minimum_yaw = max(minimum_yaw, result['yaw_uncertainty'])
                visibility_family = _visibility_family(groups, nominal_A, nominal_lo, nominal_hi,
                    top_height, support_offset, T, K, hu,
                    result['axis_uncertainty'], result['yaw_uncertainty'])
                axis = np.maximum(minimum_axis, visibility_family['axis_uncertainty'])
                yaw = max(minimum_yaw, visibility_family['yaw_uncertainty'])
                edge = float(axis.max())
                new_hu, vertical = _height_budget(nominal_A, nominal_lo, nominal_hi,
                    top_height, support_offset, origin, tn, slab, edge, yaw)
                if new_hu <= hu+1e-13:
                    hu = max(hu, new_hu)+1e-13
                    break
                minimum_axis, minimum_yaw = axis, yaw
                hu = new_hu
            else:
                raise ValueError('alternative cuboid height/position family did not close conservatively')
            if edge > .01 or yaw > math.radians(12.) or top_height <= 2.*hu+.002:
                raise ValueError('complete cuboid alternative family exceeds existing precision budget')
            stability = dict(policy=_VISIBILITY_FAMILY_POLICY,
                retained_assignments=[row['states'] for row in visibility_family['branches'] if row['retained']])
        unknown = np.column_stack(np.where(mask & (~np.isfinite(depth) | (depth <= 0.))))
        unknown_compatible = _unknown_mask_rays(unknown, T, K, nominal_A, nominal_lo, nominal_hi,
            top_height, support_offset, axis, yaw, hu)
        audit.update(unknown_mask_pixels_yx=unknown, unknown_mask_pixel_count=len(unknown),
            unknown_mask_pixels_sha256=_array_hash(unknown),
            unexplained_unknown_mask_pixel_count=int(np.count_nonzero(~unknown_compatible)),
            unknown_mask_policy='center_and_four_cell_corners_hit_same_complete_family_enclosure_possible_occupancy_only')
        if not np.all(unknown_compatible):
            raise ValueError('unknown mask band lies outside complete cuboid uncertainty family')
        exterior = _exterior(depth, scale, T, K, origin, tn, nominal_A, nominal_lo, nominal_hi, axis, yaw, pixel_size)
        audit.update(groups=groups, interval=result, empirical_edge_uncertainty_m=edge,
            empirical_axis_uncertainty_m=axis, raster_response_policy='complete_seed_and_final_fixed_strip_union',
            silhouette_plane_assignment_stability=stability,
            empirical_yaw_uncertainty_rad=yaw, empirical_height_uncertainty_m=hu,
            top_plane_vertical_deviation_m=vertical, nominal_size_xyz_m=np.r_[nominal_hi-nominal_lo, top_height],
            nominal_axes_base=nominal_A, nominal_lower=nominal_lo, nominal_upper=nominal_hi,
            support_normal_base=n, support_offset_m=support_offset, measured_top_origin_base=origin,
            measured_top_normal_base=tn, T_base_optical=T, intrinsics=K, exterior=exterior,
            common_height_interval_m=[top_height-hu, top_height+hu], complete_raster_responses_used=True,
            top_support_assignment_policy='actual_shared_eight_corner_convex_hull_no_ambiguous_edges')
        if visibility_family is not None:
            audit.update(visibility_family=visibility_family,
                top_support_assignment_policy=_VISIBILITY_FAMILY_POLICY)
        audit.update(source_color_sha256=_array_hash(color), source_depth_sha256=_array_hash(depth),
            source_mask_sha256=_array_hash(mask_input), image_shape=list(depth.shape), depth_scale_m=scale,
            metric_pixel_size_m=pixel_size,
            top_plane_rms_m=rms, intrinsics_sha256=_hash(dict(shape=list(depth.shape), K=K.tolist(), depth_scale_m=scale)),
            T_base_camera_sha256=_array_hash(T), silhouette_raster_evidence=_raster_evidence(model, edge_pixels))
        audit = _plain(audit)
        validate_cuboid_rgb_interval_source(audit, color, mask_input, depth, scale, T, K)
        return dict(ok=True, axes=nominal_A, lower=nominal_lo, upper=nominal_hi,
            nominal_height_m=top_height, height_uncertainty=hu, top_plane_vertical_deviation_m=vertical,
            edge_uncertainty=edge, yaw_uncertainty=yaw,
            axis_uncertainty=axis,
            edge_rms=max(row['maximum_line_residual_m'] for row in groups),
            continuation=[row['continuation'] for row in exterior],
            outside_valid=[row['valid_fraction'] for row in exterior], audit=audit)
    except (ValueError, TypeError, IndexError, cv2.error, np.linalg.LinAlgError) as exc:
        return reject(str(exc))


def validate_cuboid_rgb_interval(value, size_xyz, edge_budget, yaw_budget, height_budget):
    """Independently recompute stored mixed-plane strips and common budgets."""
    if not isinstance(value, dict) or value.get('policy') != POLICY or value.get('certified_error_bound') is not False:
        raise ValueError('cuboid interval policy differs')
    if value.get('uncertainty_semantics') != 'empirical_budget_not_certified_error_bound' or value.get('complete_raster_responses_used') is not True:
        raise ValueError('cuboid interval provenance differs')
    size = np.asarray(size_xyz, float)
    if size.shape != (3,) or np.any(~np.isfinite(size)) or np.any(size < .005):
        raise ValueError('cuboid interval size invalid')
    A = np.asarray(value['nominal_axes_base'], float); n = np.asarray(value['support_normal_base'], float)
    if A.shape != (3, 3) or n.shape != (3,) or not np.all(np.isfinite(A)) or not np.allclose(A.T @ A, np.eye(3), rtol=0., atol=1e-7) or abs(float(np.linalg.det(A))-1.) > 1e-7 or not np.allclose(A[:, 2], n, rtol=0., atol=1e-8):
        raise ValueError('cuboid axes differ from verified support')
    lo, hi = np.asarray(value['nominal_lower']), np.asarray(value['nominal_upper'])
    if not np.allclose(np.r_[hi-lo, value['nominal_height_m']], size, rtol=0., atol=1e-9):
        raise ValueError('cuboid interval dimensions differ')
    groups = value['groups']
    if not isinstance(groups, list) or len(groups) != 4:
        raise ValueError('four independent mixed-plane edge groups required')
    if value.get('raster_response_policy') != 'complete_seed_and_final_fixed_strip_union':
        raise ValueError('complete seed/final response-family policy differs')
    T, K = np.asarray(value['T_base_optical']), np.asarray(value['intrinsics'])
    tn = np.asarray(value['measured_top_normal_base'], float)
    if (T.shape != (4, 4) or K.shape != (4,) or not np.all(np.isfinite(T)) or not np.all(np.isfinite(K)) or min(K[:2]) <= 0. or
            not np.allclose(T[3], [0., 0., 0., 1.], rtol=0., atol=1e-10) or
            not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), rtol=0., atol=1e-7) or abs(float(np.linalg.det(T[:3, :3]))-1.) > 1e-7 or
            tn.shape != (3,) or not np.all(np.isfinite(tn)) or abs(float(tn @ tn)-1.) > 1e-7 or float(tn @ n) < math.cos(math.radians(6.))):
        raise ValueError('cuboid calibration or measured top normal invalid')
    shape = np.asarray(value['image_shape'])
    if shape.shape != (2,) or shape.dtype.kind not in 'iu' or np.any(shape <= 0):
        raise ValueError('cuboid source image shape invalid')
    scale, rms = value['depth_scale_m'], value['top_plane_rms_m']
    if not math.isfinite(float(scale)) or not 1e-7 <= scale <= 1. or not math.isfinite(float(rms)) or not 0. <= rms <= .0015 or value['top_plane_slab_m']+1e-12 < max(.001, 2.*rms+scale):
        raise ValueError('cuboid measured plane slab was reduced')
    for key in ('source_color_sha256', 'source_depth_sha256', 'source_mask_sha256', 'intrinsics_sha256', 'T_base_camera_sha256'):
        if not isinstance(value[key], str) or len(value[key]) != 64 or any(c not in '0123456789abcdef' for c in value[key]):
            raise ValueError('cuboid source digest invalid')
    if value['T_base_camera_sha256'] != _array_hash(T) or value['intrinsics_sha256'] != _hash(dict(shape=shape.tolist(), K=K.tolist(), depth_scale_m=scale)):
        raise ValueError('cuboid calibration digest differs')
    projected = _model(A, lo, hi, size[2], value['support_offset_m'], T, K)
    _validate_mask_coverage(value, projected, shape)
    expected_window = _proposal_window(projected, np.asarray(value['seed_axes_base']),
        np.asarray(value['seed_lower']), np.asarray(value['seed_upper']), value['metric_pixel_size_m'])
    if any(abs(value['bounded_proposal_window'][key]-expected_window[key]) > 1e-10 for key in expected_window):
        raise ValueError('cuboid bounded proposal window differs')
    evidence = value['silhouette_raster_evidence']
    if not isinstance(evidence, list) or len(evidence) != len(projected['edges']):
        raise ValueError('complete cuboid raster edge count differs')
    recomputed_distances, recomputed_corners = [], []
    for index, (record, edge) in enumerate(zip(evidence, projected['edges'])):
        if (record['edge_index'] != index or record['start_vertex'] != edge['start_vertex'] or record['end_vertex'] != edge['end_vertex'] or record['plane_kind'] != edge['kind'] or record['axis'] != edge['axis']):
            raise ValueError('cuboid raster edge topology or plane differs')
        pixels = np.asarray(record['corresponding_canny_pixels'])
        if pixels.shape != (101, 2) or pixels.dtype.kind not in 'iu' or np.any(pixels < 0) or np.any(pixels >= shape[::-1]):
            raise ValueError('complete source Canny correspondence invalid')
        start, end = np.asarray(edge['start_pixel']), np.asarray(edge['end_pixel'])
        samples = start+np.outer(np.linspace(0., 1., 101), end-start)
        distance = np.linalg.norm(samples-pixels, axis=1)
        recomputed_distances.append(float(distance.max())); recomputed_corners.append(float(distance[0]))
    if (np.shape(value['full_silhouette_rgb_distances_px']) != (len(evidence),) or np.shape(value['corner_rgb_distances_px']) != (len(evidence),) or
            not np.allclose(value['full_silhouette_rgb_distances_px'], recomputed_distances, rtol=0., atol=1e-9) or
            not np.allclose(value['corner_rgb_distances_px'], recomputed_corners, rtol=0., atol=1e-9)):
        raise ValueError('cuboid complete raster distance declaration differs from source correspondences')
    hu = float(value['empirical_height_uncertainty_m'])
    for i, row in enumerate(groups):
        pixels, p = np.asarray(row['line_pixels']), np.asarray(row['metric_points'])
        if row['side'] != i or row['plane'] not in ('top', 'support') or pixels.ndim != 2 or len(pixels) < 12 or pixels.shape != p.shape or pixels.shape[1] != 2 or not np.all(np.isfinite(p)):
            raise ValueError('mixed-plane raw group invalid')
        unique = np.unique(pixels, axis=0)
        if (pixels.dtype.kind not in 'iu' or
                not np.array_equal(pixels, unique[np.lexsort((unique[:, 0], unique[:, 1]))])):
            raise ValueError('mixed-plane raster responses must be unique in source order')
        expected_h = value['nominal_height_m'] if row['plane'] == 'top' else 0.
        if abs(row['plane_height_m']-expected_h) > 1e-10:
            raise ValueError('stored silhouette plane height differs')
        edge_index = row['source_hull_edge_index']
        if not isinstance(edge_index, int) or not 0 <= edge_index < len(projected['edges']):
            raise ValueError('mixed-plane source hull edge index invalid')
        edge = projected['edges'][edge_index]
        if edge['kind'] != row['plane'] or edge['axis'] != row['source_parallel_axis'] or edge['start_vertex'] != row['source_start_vertex'] or edge['end_vertex'] != row['source_end_vertex']:
            raise ValueError('mixed-plane group assigned to the wrong physical silhouette face')
        proposals = row['raster_proposals']
        if not isinstance(proposals, list) or len(proposals) != 2:
            raise ValueError('seed and final physical raster proposals must both be retained')
        admitted = np.zeros(len(pixels), bool)
        for index, proposal in enumerate(proposals):
            pA = np.asarray(proposal['axes_base']); plo, phi = np.asarray(proposal['lower']), np.asarray(proposal['upper'])
            expected_A = A if index == 0 else np.asarray(value['seed_axes_base'])
            expected_lo = lo if index == 0 else np.asarray(value['seed_lower'])
            expected_hi = hi if index == 0 else np.asarray(value['seed_upper'])
            expected_height = size[2] if index == 0 else value['seed_nominal_height_m']
            if (not np.allclose(pA, expected_A, rtol=0., atol=1e-10) or not np.allclose(plo, expected_lo, rtol=0., atol=1e-10) or
                    not np.allclose(phi, expected_hi, rtol=0., atol=1e-10) or abs(proposal['nominal_height_m']-expected_height) > 1e-10):
                raise ValueError('seed/final raster proposal geometry was changed')
            pmodel = _model(pA, plo, phi, expected_height, value['support_offset_m'], T, K)
            pindex = proposal['hull_edge_index']
            if not isinstance(pindex, int) or not 0 <= pindex < len(pmodel['edges']):
                raise ValueError('seed/final source edge index invalid')
            pedge = pmodel['edges'][pindex]
            if (pedge['kind'] != row['plane'] or pedge['kind'] != proposal['plane'] or pedge['axis'] != proposal['parallel_axis'] or
                    pedge['start_vertex'] != proposal['start_vertex'] or pedge['end_vertex'] != proposal['end_vertex']):
                raise ValueError('seed/final source edge plane assignment differs')
            start, end = np.asarray(pedge['start_pixel']), np.asarray(pedge['end_pixel']); direction = end-start
            along = (pixels-start) @ direction/float(direction @ direction)
            across = np.abs(np.cross(direction, pixels-start))/np.linalg.norm(direction)
            admitted |= (along >= .08-1e-10) & (along <= .92+1e-10) & (across <= 2.+1e-10)
        if pixels.dtype.kind not in 'iu' or np.any(pixels < 0) or np.any(pixels >= shape[::-1]) or not np.all(admitted):
            raise ValueError('mixed-plane response lies outside original fixed raster strip')
        expected = _plane(pixels, expected_h, n, value['support_offset_m'], T, K) @ A[:, :2]
        if not np.allclose(p, expected, rtol=0., atol=1e-10):
            raise ValueError('mixed-plane response differs from raw pixel rays')
        centered = p-p.mean(0); _, _, vt = np.linalg.svd(centered, full_matrices=False)
        residual = float(np.max(np.abs(centered @ [-vt[0, 1], vt[0, 0]])))
        sampling = 0.
        for dx, dy in ((-.5, -.5), (-.5, .5), (.5, -.5), (.5, .5)):
            other = _plane(pixels+[dx, dy], expected_h, n, value['support_offset_m'], T, K) @ A[:, :2]
            sampling = max(sampling, float(np.max(np.abs(other-p))))
        sampling *= math.cos(math.radians(8.))+math.sin(math.radians(8.))
        shift = 0.
        if row['plane'] == 'top':
            for dh in (-hu, hu):
                other = _plane(pixels, expected_h+dh, n, value['support_offset_m'], T, K) @ A[:, :2]
                for dx, dy in other-p:
                    low, high = (_trig_range(dx, dy, -math.radians(8.), math.radians(8.)) if i % 2 else
                                 _trig_range(dy, -dx, -math.radians(8.), math.radians(8.)))
                    shift = max(shift, abs(low), abs(high))
        if (abs(row['maximum_line_residual_m']-residual) > 1e-9 or abs(row['height_projection_bound_m']-shift) > 1e-9 or
                abs(row['sampling_bound_m']-sampling) > 1e-9 or abs(row['strip_radius_m']-(residual+sampling+shift)) > 1e-9):
            raise ValueError('cuboid strip omits raw response or shared height uncertainty')
    joint = _reference_joint(groups, lo, hi)
    expected_axis, expected_yaw = joint['axis_uncertainty'], joint['yaw_uncertainty']
    family = None
    if 'visibility_family' in value:
        if value.get('top_support_assignment_policy') != _VISIBILITY_FAMILY_POLICY:
            raise ValueError('cuboid alternative visibility policy differs')
        try:
            _assignment_stability(A, lo, hi, size[2], value['support_offset_m'], T,
                expected_axis, expected_yaw, hu)
        except ValueError as exc:
            if str(exc) != 'common cuboid family has unresolved top/support silhouette visibility':
                raise
        else:
            raise ValueError('cuboid alternative family requires an ambiguous source assignment')
        family = _visibility_family(groups, A, lo, hi, size[2], value['support_offset_m'],
            T, K, hu, expected_axis, expected_yaw)
        if not _same_family_evidence(value['visibility_family'], _plain(family)):
            raise ValueError('cuboid alternative family differs from complete source strips')
        expected_axis, expected_yaw = family['axis_uncertainty'], family['yaw_uncertainty']
    elif value.get('top_support_assignment_policy') == _VISIBILITY_FAMILY_POLICY:
        raise ValueError('cuboid alternative family evidence missing')
    axis_budget = np.asarray(value['empirical_axis_uncertainty_m'], float)
    if axis_budget.shape != (2,) or not np.all(np.isfinite(axis_budget)) or not np.allclose(axis_budget, expected_axis, rtol=0., atol=1e-9) or abs(float(axis_budget.max())-edge_budget) > 1e-9:
        raise ValueError('cuboid directional family budget differs from its common strip interval')
    required_hu, deviation = _height_budget(A, lo, hi, size[2], value['support_offset_m'],
        np.asarray(value['measured_top_origin_base']), np.asarray(value['measured_top_normal_base']),
        value['top_plane_slab_m'], float(expected_axis.max()), expected_yaw)
    for name, actual in (('empirical_edge_uncertainty_m', edge_budget), ('empirical_yaw_uncertainty_rad', yaw_budget), ('empirical_height_uncertainty_m', height_budget)):
        if not math.isfinite(float(actual)) or abs(value[name]-float(actual)) > 1e-9:
            raise ValueError('cuboid declared uncertainty differs')
    if abs(edge_budget-float(expected_axis.max())) > 1e-9 or abs(yaw_budget-expected_yaw) > 1e-9 or height_budget+1e-9 < required_hu:
        raise ValueError('cuboid family uncertainty differs from the complete raster and exterior proof')
    if abs(value['top_plane_vertical_deviation_m']-deviation) > 1e-8:
        raise ValueError('cuboid top-plane deviation differs')
    original_height = np.asarray(value['original_common_height_interval_m'], float)
    common_height = np.asarray(value['common_height_interval_m'], float)
    if (original_height.shape != (2,) or common_height.shape != (2,) or
            not np.allclose(common_height, [size[2]-height_budget, size[2]+height_budget], rtol=0., atol=1e-9) or
            common_height[0] > original_height[0]+1e-9 or common_height[1] < original_height[1]-1e-9):
        raise ValueError('original common height family was truncated')
    unknown = np.asarray(value['unknown_mask_pixels_yx'])
    if unknown.size == 0:
        unknown = np.empty((0, 2), dtype=np.int64)
    if (unknown.ndim != 2 or unknown.shape[1] != 2 or unknown.dtype.kind not in 'iu' or np.any(unknown < 0) or np.any(unknown >= shape) or
            len(unknown) != value['unknown_mask_pixel_count'] or value['unknown_mask_pixels_sha256'] != _array_hash(unknown) or
            value['unknown_mask_policy'] != 'center_and_four_cell_corners_hit_same_complete_family_enclosure_possible_occupancy_only'):
        raise ValueError('unknown source mask evidence invalid')
    compatible = _unknown_mask_rays(unknown, T, K, A, lo, hi, size[2], value['support_offset_m'], axis_budget, yaw_budget, height_budget)
    if not np.all(compatible) or value['unexplained_unknown_mask_pixel_count'] != 0:
        raise ValueError('unknown source mask strip is not explained by complete cuboid family')
    expected_stability = (_assignment_stability(A, lo, hi, size[2], value['support_offset_m'],
        T, axis_budget, yaw_budget, height_budget) if family is None else
        dict(policy=_VISIBILITY_FAMILY_POLICY,
            retained_assignments=[row['states'] for row in family['branches'] if row['retained']]))
    if value['silhouette_plane_assignment_stability'] != expected_stability:
        raise ValueError('common cuboid plane-visibility assignment differs')
    if (edge_budget > .01 or yaw_budget > math.radians(12.) or size[2] <= 2.*height_budget+.002 or
            max(value['full_silhouette_rgb_distances_px']) > RGB_CONTOUR_MAX_DISTANCE_PX or
            max(value['corner_rgb_distances_px']) > RGB_CONTOUR_MAX_DISTANCE_PX):
        raise ValueError('cuboid original qualification budget violated')
    exterior = value['exterior']
    if len(exterior) != 4 or any(row['pixels'] <= 0 or (row['valid_fraction'] >= .20 and row['continuation'] > .30) or row['occluding'] > .30 for row in exterior):
        raise ValueError('cuboid exterior evidence inconsistent')
    return value


def validate_cuboid_rgb_interval_source(value, color, mask_input, depth, scale, T, K):
    """Bind numeric interval evidence to all exact retained source pixels.

    This is required at the source boundary. The JSON-only validator proves
    arithmetic consistency, never authenticity of Canny or unknown-pixel lists.
    """
    color, mask_input, depth = np.asarray(color), np.asarray(mask_input), np.asarray(depth)
    K = np.asarray([getattr(K, name) for name in ('fx', 'fy', 'cx', 'cy')] if hasattr(K, 'fx') else K, float)
    T = np.asarray(T, float)
    for name, array in (('source_color_sha256', color), ('source_mask_sha256', mask_input), ('source_depth_sha256', depth)):
        if value[name] != _array_hash(array):
            raise ValueError('cuboid interval exact retained pixel source differs')
    if (list(depth.shape) != value['image_shape'] or float(scale) != value['depth_scale_m'] or
            _array_hash(T) != value['T_base_camera_sha256'] or
            _hash(dict(shape=list(depth.shape), K=K.tolist(), depth_scale_m=scale)) != value['intrinsics_sha256']):
        raise ValueError('cuboid interval exact source calibration differs')
    unknown = np.column_stack(np.where((mask_input > 0) & (~np.isfinite(depth) | (depth <= 0.))))
    recorded_unknown = np.asarray(value['unknown_mask_pixels_yx'], int).reshape(-1, 2)
    if not np.array_equal(unknown, recorded_unknown):
        raise ValueError('cuboid interval omitted or changed unknown source mask pixels')
    A = np.asarray(value['nominal_axes_base']); lo, hi = np.asarray(value['nominal_lower']), np.asarray(value['nominal_upper'])
    h, d = value['nominal_height_m'], value['support_offset_m']; hu = value['empirical_height_uncertainty_m']
    edge, yaw = value['empirical_edge_uncertainty_m'], value['empirical_yaw_uncertainty_rad']
    validate_cuboid_rgb_interval(value, np.r_[hi-lo, h], edge, yaw, hu)
    model = _model(A, lo, hi, h, d, T, K)
    actual_edges = cv2.Canny(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY), 50., 100.)
    ey, ex = np.where(actual_edges); actual_pixels = np.column_stack((ex, ey))
    seed_A, seed_lo, seed_hi = np.asarray(value['seed_axes_base']), np.asarray(value['seed_lower']), np.asarray(value['seed_upper'])
    seed_h = value['seed_nominal_height_m']; seed_model = _model(seed_A, seed_lo, seed_hi, seed_h, d, T, K)
    yy, xx = np.where((mask_input > 0) & np.isfinite(depth) & (depth > 0.))
    z = depth[yy, xx].astype(float)*scale
    raw_points = T[:3, 3]+_rays(np.column_stack((xx, yy)), T, K)*z[:, None]
    top = raw_points[(np.abs((raw_points-np.asarray(value['measured_top_origin_base'])) @ np.asarray(value['measured_top_normal_base'])) <= .0015) &
                     (raw_points @ A[:, 2]+d >= .5*value['original_measured_top_height_m'])]
    if len(top) < 80 or not np.allclose((top @ seed_A[:, :2]).min(0), seed_lo, rtol=0., atol=1e-10) or not np.allclose((top @ seed_A[:, :2]).max(0), seed_hi, rtol=0., atol=1e-10):
        raise ValueError('seed footprint differs from unchanged measured source top')
    seed_groups = _groups(seed_model, actual_pixels, T, K, seed_A, A[:, 2], d, seed_h, hu)
    actual_groups = _groups(model, actual_pixels, T, K, A, A[:, 2], d, h, hu, retained_groups=seed_groups)
    for stored, actual in zip(value['groups'], actual_groups):
        if not np.array_equal(np.asarray(stored['line_pixels']), actual['line_pixels']):
            raise ValueError('cuboid interval discarded competing source raster responses')
        for key in ('maximum_line_residual_m', 'sampling_bound_m', 'height_projection_bound_m', 'strip_radius_m'):
            if abs(stored[key]-actual[key]) > 1e-9:
                raise ValueError('cuboid interval raster statistics differ from actual source responses')
    for row in value['silhouette_raster_evidence']:
        pixels = np.asarray(row['corresponding_canny_pixels'], int)
        if not np.all(actual_edges[pixels[:, 1], pixels[:, 0]] > 0):
            raise ValueError('cuboid silhouette correspondence is not an actual source Canny pixel')
    actual_coverage = _mask_coverage_evidence(model, mask_input > 0)
    if 'coverage_policy' in value:
        differs = any(value[key] != actual_coverage[key]
            for row in _COVERAGE_COUNTS for key in row[:2])
        differs = differs or any(abs(value[row[2]]-actual_coverage[row[2]]) > 1e-12
            for row in _COVERAGE_COUNTS)
    else:
        differs = abs(value['silhouette_mask_coverage']-actual_coverage['silhouette_mask_coverage']) > 1e-12
    if differs:
        raise ValueError('cuboid mask coverage differs from actual source mask')
    exterior = _exterior(depth, scale, T, K, np.asarray(value['measured_top_origin_base']),
        np.asarray(value['measured_top_normal_base']), A, lo, hi, np.asarray(value['empirical_axis_uncertainty_m']), yaw,
        value['metric_pixel_size_m'])
    if _plain(exterior) != value['exterior']:
        raise ValueError('cuboid complete-family exterior differs from raw source depth')
    return value
