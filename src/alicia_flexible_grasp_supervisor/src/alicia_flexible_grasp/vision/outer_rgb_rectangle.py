"""Current RGB outer-edge inference with a common empirical rectangle budget.

No reconstructed point is presented as measured depth.  The uncertainty family
is conditional on a box hypothesis and current raster/plane evidence, and is
not a certified camera calibration error bound.
"""
import math
import cv2
import numpy as np


def _trig_range(x, y, lower, upper):
    angles = [lower, upper]
    principal = math.atan2(y, x)
    for k in range(-2, 3):
        angle = principal + k * math.pi
        if lower <= angle <= upper:
            angles.append(angle)
    values = [x * math.cos(a) + y * math.sin(a) for a in angles]
    return min(values), max(values)


def infer_outer_rgb_rectangle(color, mask, depth, scale, T, K, origin, tn,
                              axes, pixel_size, rms):
    audit = {'policy': 'current_outer_rgb_joint_interval', 'certified_error_bound': False}
    def reject(reason):
        return {'ok': False, 'reason': reason, 'audit': audit}
    if color is None:
        return reject('original RGB unavailable')
    mask = np.asarray(mask, bool)
    if color.dtype != np.uint8 or color.shape != mask.shape + (3,):
        raise ValueError('same-frame BGR image must match depth')
    camera = T[:3, 3]
    def on_plane(pixels, shift=0.):
        ray = np.column_stack(((pixels[:, 0]-K[2])/K[0],
            (pixels[:, 1]-K[3])/K[1], np.ones(len(pixels)))) @ T[:3, :3].T
        distance = (float((origin-camera) @ tn)+shift)/(ray @ tn)
        if np.any(~np.isfinite(distance)) or np.any(distance <= 0.):
            raise ValueError('RGB rays do not intersect a visible measured top')
        return camera + ray * distance[:, None]
    def project(xy):
        points = xy @ axes[:, :2].T
        points += np.outer(((origin-points) @ tn)/(tn @ axes[:, 2]), axes[:, 2])
        pc = (points-camera) @ T[:3, :3]
        return np.column_stack((pc[:, 0]/pc[:, 2]*K[0]+K[2], pc[:, 1]/pc[:, 2]*K[1]+K[3]))
    yy, xx = np.where(mask)
    silhouette = on_plane(np.column_stack((xx, yy))) @ axes[:, :2]
    lo, hi = silhouette.min(0), silhouette.max(0)
    corners = np.array([[lo[0], lo[1]], [hi[0], lo[1]], [hi[0], hi[1]], [lo[0], hi[1]]])
    pixels = project(corners)
    if np.any(pixels < 4) or np.any(pixels > np.array([mask.shape[1]-5, mask.shape[0]-5])):
        return reject('outer contour approaches image boundary')
    edges = cv2.Canny(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY), 50., 100.)
    full_ey, full_ex = np.where(edges)
    full_edge_pixels = np.column_stack((full_ex, full_ey))
    edges[cv2.dilate(mask.astype(np.uint8), np.ones((5, 5), np.uint8)) == 0] = 0
    ey, ex = np.where(edges)
    edge_pixels = np.column_stack((ex, ey))
    if len(edge_pixels) < 12:
        return reject('outer RGB edge evidence missing')
    sample_pixels = np.concatenate([pixels[i] + np.outer(np.linspace(0., 1., 101), pixels[(i+1)%4]-pixels[i]) for i in range(4)])
    sample_points = on_plane(sample_pixels)
    sampling = 0.
    for dx, dy in ((-.5, -.5), (-.5, .5), (.5, -.5), (.5, .5)):
        sampling = max(sampling, float(np.max(np.abs((on_plane(sample_pixels+[dx, dy])-sample_points) @ axes[:, :2]))))
    # The fitted line normal can rotate within the existing eight-degree
    # search window. Convert raster-axis pixel bounds before using strips.
    sampling *= math.cos(math.radians(8.))+math.sin(math.radians(8.))
    plane_slab = max(.001, 2.*rms+scale)
    plane_translation = max(float(np.max(np.abs((on_plane(sample_pixels, shift)-sample_points) @ axes[:, :2]))) for shift in (-plane_slab, plane_slab))
    anchor = (lo+hi)/2.
    groups, evidence, corner_distances = [], [], []
    for i in range(4):
        a, b = pixels[i], pixels[(i+1)%4]
        v = b-a
        uv = a+np.outer(np.linspace(.08, .92, 101), v)
        distance = np.linalg.norm(uv[:, None, :]-edge_pixels[None, :, :], axis=2)
        nearest = np.min(distance, axis=1)
        # This is the pre-existing 5x5 current-raster edge neighbourhood.
        # Missing long edges or corners cannot be replaced by straight lines.
        if np.max(nearest) > math.sqrt(8.):
            audit['unsupported_edge'] = i
            audit['maximum_edge_distance_px'] = float(np.max(nearest))
            return reject('RGB outer line or its corner arms are incomplete')
        # Retain every competing edge response in this fixed neighbourhood;
        # never choose only the most favourable parallel printed/shadow line.
        along = (edge_pixels-a) @ v / float(v @ v)
        normal_distance = np.abs(np.cross(v, edge_pixels-a))/np.linalg.norm(v)
        take = (along >= .08) & (along <= .92) & (normal_distance <= 2.)
        ep = on_plane(edge_pixels[take]) @ axes[:, :2]-anchor
        if len(ep) < 12:
            return reject('outer line has too few distinct RGB pixels')
        centered = ep-ep.mean(0)
        _, _, vt = np.linalg.svd(centered, full_matrices=False)
        across = np.array([-vt[0, 1], vt[0, 0]])
        residual = float(np.max(np.abs(centered @ across)))
        groups.append(ep)
        evidence.append({'strip_radius_m': residual+sampling, 'maximum_line_residual_m': residual,
            'edge_pixel_count': len(ep), 'maximum_rgb_distance_px': float(np.max(nearest))})
    covariance = [(p-p.mean(0)).T @ (p-p.mean(0))/len(p) for p in groups]
    J = np.array([[0., -1.], [1., 0.]])
    _, eig = np.linalg.eigh(covariance[1]+covariance[3]+J.T@(covariance[0]+covariance[2])@J)
    u = eig[:, 0]
    if u[0] < 0.:
        u = -u
    theta = math.atan2(u[1], u[0])
    rotation = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    lower_angle, upper_angle = -math.radians(8.), math.radians(8.)
    for i, (p, e) in enumerate(zip(groups, evidence)):
        delta = e['strip_radius_m']
        difference = p[:, None, :]-p[None, :, :]
        length = np.linalg.norm(difference, axis=2)
        active = length > 2.*delta+1e-15
        dif, lengths = difference[active], length[active]
        if not len(lengths):
            return reject('RGB line cannot constrain orientation')
        angle = np.arctan2(dif[:, 1], dif[:, 0])-(math.pi/2. if i%2 else 0.)
        angle = (angle-theta+math.pi/2.)%math.pi-math.pi/2.+theta
        arc = np.arcsin(2.*delta/lengths)
        lower_angle = max(lower_angle, float(np.max(angle-arc)))
        upper_angle = min(upper_angle, float(np.min(angle+arc)))
    if not lower_angle <= theta <= upper_angle:
        return reject('four observed lines have no common orthogonal uncertainty interval')
    offsets = [float((np.max(p @ rotation[:, 1 if i%2 == 0 else 0])+np.min(p @ rotation[:, 1 if i%2 == 0 else 0]))/2.) for i, p in enumerate(groups)]
    lower, upper = np.array([offsets[3], offsets[0]]), np.array([offsets[1], offsets[2]])
    size = upper-lower
    if np.any(size < .005) or np.any(size > .6):
        return reject('RGB outer rectangle dimensions invalid')
    mid = (lower+upper)/2.
    center = anchor+rotation@mid
    position_error, bounds = 0., []
    for i, (p, e) in enumerate(zip(groups, evidence)):
        points = p+anchor-center
        j = 1 if i%2 == 0 else 0
        extrema = [_trig_range(v[0], v[1], lower_angle, upper_angle) if j == 0 else
                   _trig_range(v[1], -v[0], lower_angle, upper_angle) for v in points]
        low = max(v[0] for v in extrema)-e['strip_radius_m']
        high = min(v[1] for v in extrema)+e['strip_radius_m']
        nominal = offsets[i]-mid[j]
        err = max(abs(low-nominal), abs(high-nominal))
        position_error = max(position_error, err)
        bounds.append({'minimum_m': low, 'maximum_m': high, 'nominal_m': nominal, 'error_m': err})
    local_silhouette = (silhouette-center)@rotation
    mask_extension = float(np.max(np.maximum(np.abs(local_silhouette)-size/2., 0.)))
    yaw_budget = max(math.radians(.25), theta-lower_angle, upper_angle-theta)
    fixed_axis_position_error = position_error*(math.cos(yaw_budget)+math.sin(yaw_budget))
    new_axes = np.column_stack((axes[:, :2]@rotation, axes[:, 2]))
    # Shifting the shared parallel top plane induces one homothety about the
    # camera, not independent endpoint rotations. Bound it on the complete
    # rectangle family in final fixed axes, including its offset uncertainty.
    half_yaw_extent = []
    for j in range(2):
        turn = min(yaw_budget, math.atan2(size[1-j], size[j]))
        half_yaw_extent.append((size[j]*math.cos(turn)+size[1-j]*math.sin(turn))/2.)
    family_half = np.asarray(half_yaw_extent)+max(fixed_axis_position_error, mask_extension)
    tangent_center = axes[:, :2]@center
    plane_translation = plane_slab/abs(float((origin-camera)@tn))*float(np.max(
        np.abs((tangent_center-camera)@new_axes[:, :2])+family_half))
    edge_budget = max(.001, max(fixed_axis_position_error, mask_extension)+plane_translation)
    global_midpoint = new_axes[:, :2].T @ (axes[:, :2]@center)
    new_lower, new_upper = global_midpoint-size/2., global_midpoint+size/2.
    new_corners = np.array([[new_lower[0], new_lower[1]], [new_upper[0], new_lower[1]], [new_upper[0], new_upper[1]], [new_lower[0], new_upper[1]]])
    tangent = new_corners@new_axes[:, :2].T
    base = tangent+np.outer(((origin-tangent)@tn)/(tn@axes[:, 2]), axes[:, 2])
    pc = (base-camera)@T[:3, :3]
    corner_pixels = np.column_stack((pc[:, 0]/pc[:, 2]*K[0]+K[2], pc[:, 1]/pc[:, 2]*K[1]+K[3]))
    for uv in corner_pixels:
        near = int(np.argmin(np.linalg.norm(full_edge_pixels-uv, axis=1)))
        corner_distances.append(float(np.linalg.norm((on_plane(full_edge_pixels[near:near+1])-on_plane(uv[None]))@new_axes[:, :2])))
    if max(corner_distances) > max(position_error, mask_extension)+sampling:
        audit['corner_rgb_distances_m'] = corner_distances
        return reject('RGB corner truncation exceeds observed edge intersection budget')
    final_edge_distances = []
    for i in range(4):
        uv = corner_pixels[i]+np.outer(np.linspace(0., 1., 101), corner_pixels[(i+1)%4]-corner_pixels[i])
        nearest = np.argmin(np.sum((uv[:, None, :]-full_edge_pixels[None, :, :])**2, axis=2), axis=1)
        distances = np.linalg.norm((on_plane(uv)-on_plane(full_edge_pixels[nearest]))@new_axes[:, :2], axis=1)
        final_edge_distances.append(float(np.max(distances)))
    if max(final_edge_distances) > max(position_error, mask_extension)+sampling:
        audit['full_edge_rgb_distances_m'] = final_edge_distances
        return reject('complete RGB edge arms exceed observed intersection budget')
    # The complete current raw exterior is checked outside the entire finite
    # model family. Points inside its uncertainty band are not contradictions.
    half_extent = np.asarray(half_yaw_extent)+edge_budget
    outer_half = half_extent+max(.003, 3.*pixel_size)
    outer_xy = global_midpoint+np.asarray([[-1.,-1.],[1.,-1.],[1.,1.],[-1.,1.]])*outer_half
    outer_points = outer_xy@new_axes[:, :2].T
    outer_points += np.outer(((origin-outer_points)@tn)/(tn@axes[:, 2]), axes[:, 2])
    outer_pc = (outer_points-camera)@T[:3, :3]
    outer_pixels = np.column_stack((outer_pc[:, 0]/outer_pc[:, 2]*K[0]+K[2], outer_pc[:, 1]/outer_pc[:, 2]*K[1]+K[3]))
    if np.any(outer_pixels < 1.) or np.any(outer_pixels > np.array([mask.shape[1]-2, mask.shape[0]-2])):
        return reject('complete outer uncertainty envelope reaches image boundary')
    x0, y0 = np.floor(outer_pixels.min(0)).astype(int)
    x1, y1 = np.ceil(outer_pixels.max(0)).astype(int)+1
    gy, gx = np.mgrid[y0:y1, x0:x1]
    grid = np.column_stack((gx.ravel(), gy.ravel()))
    projected = on_plane(grid) @ new_axes[:, :2]-global_midpoint
    zz = depth[grid[:, 1], grid[:, 0]].astype(float)*scale
    valid = np.isfinite(zz)&(zz > 0.)
    xyz = np.column_stack(((grid[:, 0]-K[2])*zz/K[0], (grid[:, 1]-K[3])*zz/K[1], zz))@T[:3, :3].T+camera
    plane_distance = (xyz-origin)@tn
    continuation, outside_valid, exterior_counts = [], [], []
    for i in range(4):
        across = 1 if i%2 == 0 else 0
        along = 1-across
        sign = -1. if i in (0, 3) else 1.
        distance = sign*projected[:, across]-half_extent[across]
        strip = (distance > 0.) & (distance <= max(.003, 3.*pixel_size)) & (np.abs(projected[:, along]) <= size[along]/2.)
        count, measured = int(np.count_nonzero(strip)), int(np.count_nonzero(strip&valid))
        if not count:
            return reject('external full uncertainty envelope is not visible')
        vfrac = measured/count
        continued = float(np.count_nonzero(strip&valid&(np.abs(plane_distance)<.0015)))/max(1, measured)
        occluding = float(np.count_nonzero(strip&valid&(plane_distance>.0025)))/count
        continuation.append(continued); outside_valid.append(vfrac)
        exterior_counts.append({'pixels': count, 'valid': measured, 'continuation': continued, 'occluding': occluding})
        if vfrac >= .20 and continued > .30:
            audit['exterior'] = exterior_counts
            return reject('measured top continues outside the complete RGB uncertainty envelope')
        if occluding > .30:
            return reject('an external occluder can truncate the inferred RGB top')
    audit.update(edge_evidence=evidence, corner_rgb_distances_m=corner_distances, full_edge_rgb_distances_m=final_edge_distances,
        position_intervals=bounds, common_yaw_interval_rad=[lower_angle, upper_angle],
        nominal_yaw_rad=theta, current_raster_half_pixel_m=sampling,
        top_plane_slab_m=plane_slab, common_plane_translation_m=plane_translation,
        mask_ray_extension_m=mask_extension, position_family_error_m=position_error, fixed_axis_position_error_m=fixed_axis_position_error,
        empirical_edge_uncertainty_m=edge_budget, empirical_yaw_uncertainty_rad=yaw_budget,
        exterior=exterior_counts, complete_mask_used=True,
        uncertainty_semantics='empirical_budget_not_certified_error_bound')
    return {'ok': True, 'axes': new_axes, 'lower': new_lower, 'upper': new_upper,
        'edge_uncertainty': edge_budget, 'yaw_uncertainty': yaw_budget,
        'continuation': continuation, 'outside_valid': outside_valid,
        'edge_rms': max(e['maximum_line_residual_m'] for e in evidence), 'audit': audit}


_INTERVAL_FIELDS = frozenset(('policy', 'certified_error_bound', 'edge_evidence',
    'corner_rgb_distances_m', 'full_edge_rgb_distances_m', 'position_intervals',
    'common_yaw_interval_rad', 'nominal_yaw_rad', 'current_raster_half_pixel_m',
    'top_plane_slab_m', 'common_plane_translation_m', 'mask_ray_extension_m',
    'position_family_error_m', 'fixed_axis_position_error_m',
    'empirical_edge_uncertainty_m', 'empirical_yaw_uncertainty_rad', 'exterior',
    'complete_mask_used', 'uncertainty_semantics'))


def validate_outer_rgb_interval(value, size_xy, edge_budget, yaw_budget):
    """Check the self-contained numerical family budget used by all consumers.

    Source hashes are checked by the containing RegularBoxInference contract;
    this checks arithmetic/qualification, not authenticity of image evidence.
    """
    def number(v, low=0., high=math.inf):
        if isinstance(v, (bool, np.bool_)) or not isinstance(v, (int, float, np.integer, np.floating)):
            raise ValueError('RGB interval numeric evidence invalid')
        v = float(v)
        if not math.isfinite(v) or not low <= v <= high:
            raise ValueError('RGB interval numeric evidence outside limits')
        return v
    def same(a, b):
        if not math.isclose(float(a), float(b), rel_tol=0., abs_tol=1e-10):
            raise ValueError('RGB interval budget arithmetic differs')
    if not isinstance(value, dict) or set(value) != _INTERVAL_FIELDS:
        raise ValueError('RGB interval evidence fields differ')
    e = value
    if (e['policy'] != 'current_outer_rgb_joint_interval' or e['certified_error_bound'] is not False
            or e['complete_mask_used'] is not True
            or e['uncertainty_semantics'] != 'empirical_budget_not_certified_error_bound'):
        raise ValueError('RGB interval qualification semantics differ')
    for key, length in (('edge_evidence', 4), ('position_intervals', 4), ('exterior', 4),
                        ('corner_rgb_distances_m', 4), ('full_edge_rgb_distances_m', 4),
                        ('common_yaw_interval_rad', 2)):
        if not isinstance(e[key], list) or len(e[key]) != length:
            raise ValueError('RGB interval requires all four independent edges')
    sampling = number(e['current_raster_half_pixel_m'], 1e-9, .005)
    number(e['top_plane_slab_m'], .001, .01)
    plane = number(e['common_plane_translation_m'], 0., .01)
    extension = number(e['mask_ray_extension_m'], 0., .01)
    lower, upper = [number(x, -math.radians(8.), math.radians(8.)) for x in e['common_yaw_interval_rad']]
    theta = number(e['nominal_yaw_rad'], lower, upper)
    yaw = max(math.radians(.25), theta-lower, upper-theta)
    same(yaw, e['empirical_yaw_uncertainty_rad'])
    same(yaw, yaw_budget)
    position = 0.
    for i, row in enumerate(e['position_intervals']):
        if not isinstance(row, dict) or set(row) != {'minimum_m', 'maximum_m', 'nominal_m', 'error_m'}:
            raise ValueError('RGB interval face fields differ')
        low = number(row['minimum_m'], -.6, .6)
        high = number(row['maximum_m'], low, .6)
        nominal = number(row['nominal_m'], low, high)
        axis = 1 if i%2 == 0 else 0
        same(nominal, float(size_xy[axis])/2.*(-1. if i in (0, 3) else 1.))
        err = max(abs(low-nominal), abs(high-nominal))
        same(err, number(row['error_m'], 0., .01))
        position = max(position, err)
    same(position, e['position_family_error_m'])
    fixed = position*(math.cos(yaw)+math.sin(yaw))
    same(fixed, e['fixed_axis_position_error_m'])
    required = max(.001, max(fixed, extension)+plane)
    same(required, e['empirical_edge_uncertainty_m'])
    same(required, edge_budget)
    for row in e['edge_evidence']:
        if not isinstance(row, dict) or set(row) != {'strip_radius_m', 'maximum_line_residual_m', 'edge_pixel_count', 'maximum_rgb_distance_px'}:
            raise ValueError('RGB interval line fields differ')
        residual = number(row['maximum_line_residual_m'], 0., .005)
        same(number(row['strip_radius_m'], sampling, .01), residual+sampling)
        number(row['maximum_rgb_distance_px'], 0., math.sqrt(8.)+1e-10)
        if type(row['edge_pixel_count']) is not int or row['edge_pixel_count'] < 12:
            raise ValueError('RGB interval line evidence too small')
    for key in ('corner_rgb_distances_m', 'full_edge_rgb_distances_m'):
        for distance in e[key]:
            number(distance, 0., max(position, extension)+sampling+1e-10)
    for row in e['exterior']:
        if not isinstance(row, dict) or set(row) != {'pixels', 'valid', 'continuation', 'occluding'}:
            raise ValueError('RGB interval exterior fields differ')
        if type(row['pixels']) is not int or row['pixels'] <= 0 or type(row['valid']) is not int or not 0 <= row['valid'] <= row['pixels']:
            raise ValueError('RGB interval exterior counts invalid')
        continuation = number(row['continuation'], 0., 1.)
        number(row['occluding'], 0., .30)
        if row['valid']/row['pixels'] >= .20 and continuation > .30:
            raise ValueError('measured top continues outside RGB uncertainty family')
    return value
