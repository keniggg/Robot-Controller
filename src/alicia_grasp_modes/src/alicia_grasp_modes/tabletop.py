"""Measured-depth tabletop instances and a generation-scoped target lock.

No ROS imports, motion interfaces, semantic classifier, hole filling, or synthetic
surface completion. Positions use ROS camera_link coordinates [z, -x, -y].
"""
from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class Config:
    depth_min_m: float = 0.03
    depth_max_m: float = 2.0
    support_threshold_m: float = 0.002
    foreground_min_m: float = 0.004
    foreground_max_m: float = 0.12
    support_min_points: int = 120
    support_max_samples: int = 1800
    support_min_ratio: float = 0.70
    support_max_normal_change_deg: float = 4.0
    support_max_local_shift_m: float = 0.007
    support_local_shift_warning_m: float = 0.004
    min_object_points: int = 60
    center_radius_fraction: float = 0.125
    center_ambiguity_fraction: float = 0.045
    depth_edge_m: float = 0.008
    association_distance_m: float = 0.025
    max_surface_extent_m: float = 0.15


@dataclass
class Candidate:
    mask: np.ndarray
    center_uv: np.ndarray
    bbox: tuple
    position_camera: np.ndarray
    position_base: np.ndarray
    extent_m: np.ndarray
    depth_m: float
    quality: float
    metrics: dict = field(default_factory=dict)
    support_footprint_base: np.ndarray = None


@dataclass
class Segmentation:
    candidates: list
    plane_base: np.ndarray
    metrics: dict


def _project(depth, intrinsics):
    v, u = np.indices(depth.shape)
    return _project_pixels(depth, u, v, intrinsics)


def _project_pixels(depth, u, v, intrinsics):
    fx, fy, cx, cy = [float(v) for v in intrinsics]
    if not all(np.isfinite(intrinsics)) or fx <= 0 or fy <= 0:
        raise ValueError('invalid_camera_intrinsics')
    # This workspace publishes aligned images in camera_link, not optical TF.
    return np.stack((depth, -(u - cx) * depth / fx,
                     -(v - cy) * depth / fy), axis=-1)


def project_base_to_pixel(position, transform, intrinsics):
    camera = np.linalg.solve(transform, np.r_[position, 1.0])[:3]
    if camera[0] <= 0.01:
        raise ValueError('locked_target_behind_camera')
    fx, fy, cx, cy = intrinsics
    return np.array([cx - fx * camera[1] / camera[0],
                     cy - fy * camera[2] / camera[0]])


def _fit_plane(points, config):
    """Fixed-cost RANSAC with consensus-preserving measured SVD refinement."""
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1:] != (3,) or not np.all(np.isfinite(points)):
        raise ValueError('invalid_support_plane_points')
    if len(points) < config.support_min_points:
        raise ValueError('insufficient_support_depth')
    indices = np.linspace(0, len(points) - 1,
                          min(len(points), config.support_max_samples), dtype=int)
    points = points[indices]
    rng = np.random.RandomState(17)
    best = np.zeros(len(points), dtype=bool)
    best_plane = None
    for _ in range(128):
        indices = rng.randint(len(points), size=3)
        if len(set(indices.tolist())) < 3:
            continue
        a, b, c = points[indices]
        normal = np.cross(b - a, c - a)
        length = np.linalg.norm(normal)
        if length < 1e-9:
            continue
        normal /= length
        # einsum avoids multithreaded BLAS startup for these small vectors.
        inliers = np.abs(np.einsum('ij,j->i', points - a, normal)) <= config.support_threshold_m
        if np.count_nonzero(inliers) > np.count_nonzero(best):
            best = inliers
            best_plane = np.r_[normal, -float(normal @ a)]
    if np.count_nonzero(best) < config.support_min_points or best_plane is None:
        raise ValueError('support_plane_not_found')

    def evaluate(plane):
        distances = np.einsum('ij,j->i', points, plane[:3]) + plane[3]
        inliers = np.abs(distances) <= config.support_threshold_m
        count = int(np.count_nonzero(inliers))
        residual = float(np.sqrt(np.mean(distances[inliers] ** 2))) if count else float('inf')
        return inliers, count, residual

    # Least squares minimizes squared error, not the number of points inside
    # the original distance gate. In asymmetric noise it can discard an
    # already qualified consensus. Compare every model on the same unmodified
    # samples, retaining the RANSAC model if either SVD iteration is worse.
    best, best_count, best_residual = evaluate(best_plane)
    refining = best
    for _ in range(2):
        if np.count_nonzero(refining) < 3:
            break
        center = np.mean(points[refining], axis=0)
        _, _, vh = np.linalg.svd(points[refining] - center, full_matrices=False)
        normal = vh[-1]
        plane = np.r_[normal, -float(normal @ center)]
        refining, count, residual = evaluate(plane)
        if count > best_count or (count == best_count and residual < best_residual):
            best, best_count, best_residual, best_plane = refining, count, residual, plane
    if best_count < config.support_min_points:
        raise ValueError('support_plane_not_found')
    ratio = float(best_count / len(points))
    if ratio < config.support_min_ratio:
        raise ValueError('support_plane_inlier_ratio_low')
    if best_plane[3] < 0:
        best_plane = -best_plane
    return best_plane, ratio, best_residual, len(points)


def _normalized_plane(plane):
    result = np.asarray(plane, dtype=float)
    if result.shape != (4,) or not np.all(np.isfinite(result)):
        raise ValueError('invalid_support_plane_coefficients')
    norm = float(np.linalg.norm(result[:3]))
    if norm <= 1e-12:
        raise ValueError('invalid_support_plane_normal')
    return result / norm


def _plane_axes(normal):
    seed = np.array([1., 0., 0.]) if abs(normal[0]) < .9 else np.array([0., 1., 0.])
    first = np.cross(normal, seed)
    first /= np.linalg.norm(first)
    return first, np.cross(normal, first)


def _measured_support_footprint(points_base, plane_base):
    """Convex hull of ALL measured instance pixels projected onto its support.

    This hull is only a plane-consistency footprint. It is never a target mask
    and never creates surface/depth pixels for a grasp.
    """
    plane = _normalized_plane(plane_base)
    first, second = _plane_axes(plane[:3])
    origin = -plane[3] * plane[:3]
    coordinates = np.column_stack(((points_base-origin) @ first, (points_base-origin) @ second))
    hull = cv2.convexHull(coordinates.astype(np.float32)).reshape(-1, 2)
    if len(hull) < 3:
        raise ValueError('degenerate_measured_support_footprint')
    footprint = origin + hull[:, :1] * first + hull[:, 1:] * second
    footprint.setflags(write=False)
    return footprint


def _support_offset_diagnostics(maximum, config):
    """Record measured disagreement without correcting geometry or authorizing it."""
    warning = float(config.support_local_shift_warning_m)
    if not np.isfinite(warning) or warning < 0:
        raise ValueError('invalid_support_plane_warning_config')
    # A caller may retain a stricter hard limit below the warning threshold.
    return dict(support_reference_distance_warning=bool(maximum > warning + 1e-12),
                support_reference_warning_distance_m=warning,
                support_reference_hard_distance_m=float(config.support_max_local_shift_m))


def _validate_support_plane_update(reference_plane_base, current_plane_base,
                                   target_position_base, config,
                                   target_footprint_base):
    reference = _normalized_plane(reference_plane_base)
    current = _normalized_plane(current_plane_base)
    anchor = np.asarray(target_position_base, dtype=float)
    footprint = np.asarray(target_footprint_base, dtype=float)
    max_angle = float(config.support_max_normal_change_deg)
    max_shift = float(config.support_max_local_shift_m)
    if (not np.isfinite(max_angle) or not 0 < max_angle < 90
            or not np.isfinite(max_shift) or max_shift <= 0):
        raise ValueError('invalid_support_plane_consistency_config')
    if anchor.shape != (3,) or not np.all(np.isfinite(anchor)):
        raise ValueError('invalid_support_reference_anchor')
    if (footprint.ndim != 2 or footprint.shape[1:] != (3,)
            or len(footprint) < 3 or not np.all(np.isfinite(footprint))):
        raise ValueError('invalid_measured_support_footprint')
    if np.linalg.matrix_rank(footprint-footprint[0], tol=1e-8) < 2:
        raise ValueError('degenerate_measured_support_footprint')
    if np.max(np.abs(footprint @ reference[:3] + reference[3])) > 1e-5:
        raise ValueError('support_footprint_not_on_reference_plane')
    if float(reference[:3] @ current[:3]) < 0:
        current = -current
    angle = float(np.rad2deg(np.arccos(np.clip(reference[:3] @ current[:3], -1., 1.))))
    if angle > max_angle + 1e-9:
        raise ValueError('support_plane_normal_inconsistent: %.6f > %.6f deg' % (angle, max_angle))
    delta = current - reference
    anchor_shift = abs(float(delta[:3] @ anchor + delta[3]))
    footprint_shift = float(np.max(np.abs(footprint @ delta[:3] + delta[3])))
    maximum = max(anchor_shift, footprint_shift)
    diagnostics = _support_offset_diagnostics(maximum, config)
    if maximum > max_shift + 1e-12:
        raise ValueError('support_plane_local_distance_inconsistent: %.6f > %.6f m' % (maximum, max_shift))
    return current, dict(support_reference_angle_deg=angle,
                         support_reference_anchor_distance_m=anchor_shift,
                         support_reference_max_distance_m=maximum,
                         support_reference_footprint_vertices=int(len(footprint)),
                         **diagnostics)


def _support_quadrants(supporting, u, v, center):
    return sum(bool(np.count_nonzero(
        supporting & (u < center[0] if left else u >= center[0])
        & (v < center[1] if top else v >= center[1])) >= 20)
        for left in (True, False) for top in (True, False))


def _fit_support_ring_sequence(depth, intrinsics, valid, radius, scale, config,
                               points_camera=None):
    """Use the same bounded measured rings before and after target locking.

    Only an insufficient inlier ratio admits a smaller ring. A fit that meets
    the original quality contract is final: spatial coverage and immutable
    reference consistency are checked by the caller, never bypassed by trying
    another footprint after positive disagreement.
    """
    for outer in (170, 140, 110):
        ring = valid & (radius >= 70 * scale) & (radius <= outer * scale)
        if points_camera is None:
            ring_v, ring_u = np.nonzero(ring)
            points = _project_pixels(depth[ring], ring_u, ring_v, intrinsics)
        else:
            points = points_camera[ring]
        try:
            plane, ratio, residual, count = _fit_plane(points, config)
        except ValueError as exc:
            if str(exc) != 'support_plane_inlier_ratio_low' or outer == 110:
                raise
        else:
            return plane, ratio, residual, count, ring, points, float(outer * scale)


def _audit_current_support(depth_m, intrinsics, transform_base_camera,
                           config, anchor, reference_plane_base):
    """Look for positive current-frame contradiction before reference filtering.

    Missing/occluded support is not a contradiction. A fit is compared with the
    immutable reference only after the original ratio, residual and spatial
    coverage gates pass. This audit neither segments nor changes target identity.
    """
    depth = np.asarray(depth_m, dtype=np.float32)
    transform = np.asarray(transform_base_camera, dtype=float)
    if depth.ndim != 2 or not depth.size:
        raise ValueError('invalid_depth_shape')
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError('invalid_exact_transform')
    valid = np.isfinite(depth) & (depth >= config.depth_min_m) & (depth <= config.depth_max_m)
    center = project_base_to_pixel(anchor.position_base, transform, intrinsics)
    v, u = np.indices(depth.shape)
    scale = min(depth.shape[1] / 640.0, depth.shape[0] / 480.0)
    radius = np.hypot(u - center[0], v - center[1])
    evidence = dict(support_current_audit_available=False)
    try:
        # Preserve row-major samples and avoid a second full-frame cloud.
        plane, ratio, residual, count, ring, points, outer = _fit_support_ring_sequence(
            depth, intrinsics, valid, radius, scale, config)
    except ValueError as exc:
        if str(exc) not in ('insufficient_support_depth', 'support_plane_not_found',
                            'support_plane_inlier_ratio_low'):
            raise
        evidence['support_current_audit_reason'] = str(exc)
        return evidence
    ring_u, ring_v = u[ring], v[ring]
    supporting = (np.abs(np.einsum('ij,j->i', points, plane[:3])
                         + plane[3]) <= config.support_threshold_m)
    covered = _support_quadrants(supporting, ring_u, ring_v, center)
    evidence.update(support_current_audit_inlier_ratio=ratio,
                    support_current_audit_residual_m=residual,
                    support_current_audit_sample_count=int(count),
                    support_current_audit_quadrants=covered,
                    support_current_audit_ring_outer_radius_px=outer)
    if not np.isfinite(residual) or residual > config.support_threshold_m:
        evidence['support_current_audit_reason'] = 'support_plane_residual_too_large'
        return evidence
    if covered < 3:
        evidence['support_current_audit_reason'] = 'insufficient_support_spatial_coverage'
        return evidence
    current = _normalized_plane(np.linalg.solve(transform.T, plane))
    _, agreement = _validate_support_plane_update(
        reference_plane_base, current, anchor.position_base, config,
        anchor.support_footprint_base)
    evidence.update(agreement)
    evidence['support_current_audit_available'] = True
    return evidence


def _depth_components(foreground, depth, minimum, edge,
                      points_camera=None, target_camera=None, association_distance=None, *, connectivity=4):
    """Measured neighbors; no filled pixels or relaxed depth edge."""
    if type(connectivity) is not int or connectivity not in (4, 8):
        raise ValueError('invalid_depth_component_connectivity')
    # First discard speckles in C++; floodFill then splits depth-discontinuous
    # touching regions without bridging missing depth or inventing mask pixels.
    count, labels, stats, _ = cv2.connectedComponentsWithStats(foreground, connectivity=connectivity)
    eligible = [index for index in range(1, count)
                if stats[index, cv2.CC_STAT_AREA] >= minimum]
    if target_camera is not None:
        target = np.asarray(target_camera, dtype=float)
        radius = float(association_distance)
        if (target.shape != (3,) or not np.all(np.isfinite(target))
                or not np.isfinite(radius) or radius <= 0
                or points_camera is None or points_camera.shape != depth.shape + (3,)):
            raise ValueError('invalid_locked_component_region')
        nearby = []
        for index in eligible:
            left, top, width, height = [int(v) for v in stats[index, :4]]
            region = np.s_[top:top+height, left:left+width]
            xyz = points_camera[region][labels[region] == index]
            # Every split child's coordinate-wise median is inside its parent
            # component's measured camera-space AABB. A rigid transform keeps
            # distances, so a box farther than the original association radius
            # cannot contain any eligible target. Keep intersecting parents
            # whole: cropping or testing their centroid could hide a valid
            # child or an ambiguous second instance.
            lower, upper = np.min(xyz, axis=0), np.max(xyz, axis=0)
            distance = np.linalg.norm(target - np.clip(target, lower, upper))
            if distance <= radius + 1e-9:
                nearby.append(index)
        eligible = nearby
    if len(eligible) > 32:
        raise ValueError('too_many_foreground_instances')
    components = []
    for index in eligible:
        left, top, width, height = [int(value) for value in stats[index, :4]]
        region = np.s_[top:top+height, left:left+width]
        component = labels[region] == index
        patch = np.ascontiguousarray(depth[region])
        blocked = np.ones((height + 2, width + 2), np.uint8)
        blocked[1:-1, 1:-1][component] = 0
        remaining = component.copy()
        # Bounded work even for highly corrupted surfaces.
        for _ in range(64):
            locations = np.argwhere(remaining)
            if len(locations) < minimum:
                break
            y, x = locations[len(locations) // 2]
            _, _, blocked, rect = cv2.floodFill(
                patch, blocked, (int(x), int(y)), 0, edge, edge,
                flags=connectivity | cv2.FLOODFILL_MASK_ONLY | (2 << 8))
            selected = blocked[1:-1, 1:-1] == 2
            if np.count_nonzero(selected) >= minimum:
                full_mask = np.zeros(depth.shape, bool)
                full_mask[region] = selected
                components.append(full_mask)
                if len(components) > 32:
                    raise ValueError('too_many_depth_components')
            remaining[selected] = False
            blocked[1:-1, 1:-1][selected] = 1
    return components


def segment(depth_m, intrinsics, transform_base_camera, config=None,
            target_position_base=None, plane_base=None,
            target_footprint_base=None, fit_current_support=True):
    config = config or Config()
    depth = np.asarray(depth_m, dtype=np.float32)
    transform = np.asarray(transform_base_camera, dtype=float)
    if depth.ndim != 2 or not depth.size:
        raise ValueError('invalid_depth_shape')
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError('invalid_exact_transform')
    valid = np.isfinite(depth) & (depth >= config.depth_min_m) & (depth <= config.depth_max_m)
    if np.count_nonzero(valid) < config.support_min_points:
        raise ValueError('insufficient_valid_depth')
    points = _project(depth, intrinsics)
    height, width = depth.shape
    v, u = np.indices(depth.shape)
    center = np.array([width / 2.0, height / 2.0])
    if target_position_base is not None:
        center = project_base_to_pixel(target_position_base, transform, intrinsics)
    scale = min(width / 640.0, height / 480.0)
    radius = np.hypot(u - center[0], v - center[1])
    ring = valid & (radius >= 70 * scale) & (radius <= 170 * scale)
    # Segment against this frame's measured table. A transported initial fit
    # alone can absorb a table patch into a thin object as the camera rolls.
    # The initial plane remains an immutable consistency reference, not an
    # assumed exact surface model for every subsequent view.
    agreement = {}
    if not fit_current_support:
        if plane_base is None:
            raise ValueError('missing_reference_support_plane')
        # Keep the supplied reference coefficients as exact provenance. Even
        # an already normalized fitted plane can change by one ULP when
        # normalized again; that is not a new support measurement. Candidate
        # metrics and calculations still require the normalized metric plane.
        reference_plane_base = np.array(plane_base, dtype=float, copy=True)
        current_plane_base = _normalized_plane(reference_plane_base)
        plane_camera = transform.T @ current_plane_base
        distances = np.einsum('ijk,k->ij', points, plane_camera[:3]) + plane_camera[3]
        supporting = valid & (np.abs(distances) <= config.support_threshold_m)
        if np.count_nonzero(supporting) < config.support_min_points:
            raise ValueError('locked_support_plane_no_longer_visible')
        residual = float(np.sqrt(np.mean(distances[supporting] ** 2)))
        ratio = float(np.mean(supporting[ring])) if np.any(ring) else 0.
        sample_count = min(np.count_nonzero(supporting), config.support_max_samples)
        plane_source = 'locked_base_plane'
    else:
        # The same bounded ring selection serves acquisition, current support
        # audit and locked refitting. It never replaces the immutable target
        # footprint used by the consistency checks below.
        plane_camera, ratio, residual, sample_count, ring, _, outer = _fit_support_ring_sequence(
            depth, intrinsics, valid, radius, scale, config, points_camera=points)
        agreement['support_ring_outer_radius_px'] = outer
        current_plane_base = _normalized_plane(np.linalg.solve(transform.T, plane_camera))
        plane_source = 'current_ring_fit'
        supporting = ring & (np.abs(np.einsum('ijk,k->ij', points, plane_camera[:3]) + plane_camera[3]) <= config.support_threshold_m)
    if not np.isfinite(residual) or residual > config.support_threshold_m:
        raise ValueError('support_plane_residual_too_large')
    # A narrow strip or isolated patch must not establish the support surface.
    covered = _support_quadrants(supporting, u, v, center)
    if covered < 3:
        raise ValueError('insufficient_support_spatial_coverage')
    if fit_current_support and plane_base is not None:
        current_plane_base, consistency = _validate_support_plane_update(
            plane_base, current_plane_base, target_position_base, config,
            target_footprint_base)
        agreement.update(consistency)
        plane_camera = transform.T @ current_plane_base
    above = np.einsum('ijk,k->ij', points, plane_camera[:3]) + plane_camera[3]
    # Once locked, search the full frame; selection is based on base coordinates.
    roi = radius <= 155 * scale if target_position_base is None else np.ones(depth.shape, bool)
    foreground = (valid & roi & (above >= config.foreground_min_m)
                  & (above <= config.foreground_max_m)).astype(np.uint8)
    foreground = cv2.morphologyEx(foreground, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    # Morphology must never restore a depth hole or non-foreground pixel.
    foreground &= (valid & (above >= config.foreground_min_m)).astype(np.uint8)
    candidates = []
    target_camera = (None if target_position_base is None else
                     np.linalg.solve(transform, np.r_[target_position_base, 1.])[:3])
    for binary in _depth_components(
            foreground, depth, config.min_object_points, config.depth_edge_m,
            points_camera=points, target_camera=target_camera,
            association_distance=config.association_distance_m):
        xyz = points[binary].astype(float)
        points_base = xyz @ transform[:3, :3].T + transform[:3, 3]
        p_camera = np.median(xyz, axis=0)
        p_base = (transform @ np.r_[p_camera, 1.0])[:3]
        _, _, vh = np.linalg.svd(xyz - np.mean(xyz, axis=0), full_matrices=False)
        oriented = (xyz - np.mean(xyz, axis=0)) @ vh.T
        extent = np.sort(np.percentile(oriented, 98, axis=0) - np.percentile(oriented, 2, axis=0))[::-1]
        if extent[0] > config.max_surface_extent_m:
            continue
        ys, xs = np.nonzero(binary)
        bbox = (int(xs.min()), int(ys.min()), int(xs.max() - xs.min() + 1), int(ys.max() - ys.min() + 1))
        count = int(len(xs))
        support_quality = max(0.0, 1.0 - residual / (2 * config.support_threshold_m))
        point_quality = min(1.0, count / 400.0)
        # This is explicitly a measured-geometry score, NOT semantic confidence.
        quality = float(0.65 * support_quality + 0.35 * point_quality)
        metrics = dict(valid_points=count, median_height_m=float(np.median(above[binary])),
                       observed_surface_extent_m=extent.tolist(), support_residual_m=residual,
                       support_inlier_ratio=ratio, support_source=plane_source,
                       support_sample_count=int(sample_count), support_quadrants=covered,
                       support_plane_base=current_plane_base.tolist(), **agreement)
        candidates.append(Candidate(binary.astype(np.uint8) * 255,
                                    np.array([np.mean(xs), np.mean(ys)]), bbox,
                                    p_camera, p_base, extent, float(p_camera[0]), quality, metrics,
                                    _measured_support_footprint(points_base, current_plane_base)))
    result_plane_base = current_plane_base if fit_current_support else reference_plane_base
    return Segmentation(candidates, result_plane_base,
                        dict(support_source=plane_source, support_inlier_ratio=ratio,
                             support_residual_m=residual, support_sample_count=int(sample_count),
                             **agreement))


def _matching_candidates(candidates, anchor, last, config):
    """Read-only form of the original identity gates, before a single commit."""
    matching = []
    for candidate in candidates:
        distance = np.linalg.norm(candidate.position_base - anchor.position_base)
        step = np.linalg.norm(candidate.position_base - last.position_base)
        ratios = candidate.extent_m[:2] / np.maximum(anchor.extent_m[:2], 0.003)
        if (distance <= config.association_distance_m
                and step <= config.association_distance_m
                and np.all((ratios >= 0.5) & (ratios <= 2.0))):
            matching.append((float(distance), candidate))
    matching.sort(key=lambda item: item[0])
    return matching


def segment_with_support_fallback(depth_m, intrinsics, transform_base_camera,
                                  config=None, anchor=None, last=None,
                                  reference_plane_base=None, *, prefer_current_support=False):
    """Try bounded same-frame re-segmentation before mutating target identity.

    Independent current-support evidence is checked even when the reference
    path yields an eligible measured instance. A contaminated component can
    trigger one current-table re-segmentation; that fit must
    agree with the immutable original plane over its actual measured object
    footprint. Neither path commits tracking state or unsets a loss latch.
    """
    if type(prefer_current_support) is not bool:
        raise ValueError('invalid_current_support_preference')
    config = config or Config()
    if anchor is None:
        return segment(depth_m, intrinsics, transform_base_camera, config)
    if last is None or reference_plane_base is None:
        raise ValueError('missing_locked_target_reference')
    arguments = dict(target_position_base=anchor.position_base,
                     plane_base=reference_plane_base,
                     target_footprint_base=anchor.support_footprint_base)
    support_audit = _audit_current_support(
        depth_m, intrinsics, transform_base_camera, config, anchor,
        reference_plane_base)
    reference_error = ''
    try:
        reference = segment(depth_m, intrinsics, transform_base_camera, config,
                            fit_current_support=False, **arguments)
        matching = _matching_candidates(reference.candidates, anchor, last, config)
    except ValueError as exc:
        # An old transported plane can turn table texture into many separate
        # foreground patches while the camera moves. Previously that bounded
        # work rejection bypassed the same-frame support-refit path entirely.
        # The original 2 mm band can also be empty or cover only a narrow strip
        # while a qualified current plane agrees within the original local-
        # distance tolerance. Admit those old-plane visibility failures only
        # with this frame's independent support audit; missing/contradictory
        # current support is not recovery evidence.
        capped = str(exc) in ('too_many_foreground_instances', 'too_many_depth_components')
        qualified_visibility_recovery = (
            str(exc) in ('locked_support_plane_no_longer_visible',
                         'insufficient_support_spatial_coverage')
            and support_audit.get('support_current_audit_available') is True)
        if not (capped or qualified_visibility_recovery):
            raise
        reference_error = str(exc)
        matching = []
    # An already ambiguous pair must remain ambiguous. Re-fitting a plane is
    # not independent evidence for discarding one of two eligible identities.
    # At a qualified stationary measurement, even a plausible single old-
    # plane component can include table texture. Use the independently audited
    # current fit for its measured foreground. The immutable plane still gates
    # agreement, and no refit may discard an already ambiguous second identity.
    observation_refit = (prefer_current_support and len(matching) == 1
                         and support_audit.get('support_current_audit_available') is True)
    if len(matching) >= 1 and not observation_refit:
        reference.metrics.update(support_audit)
        for candidate in reference.candidates:
            candidate.metrics.update(support_audit)
        return reference
    current = segment(depth_m, intrinsics, transform_base_camera, config,
                      fit_current_support=True, **arguments)
    evidence = dict(support_audit, support_refit_attempted=True,
                    support_reference_eligible_candidates=len(matching))
    if observation_refit:
        evidence['support_refit_reason'] = 'qualified_current_support_measurement'
    if reference_error:
        evidence['support_reference_segmentation_error'] = reference_error
    current.metrics.update(evidence)
    for candidate in current.candidates:
        candidate.metrics.update(evidence)
    return current


class TargetTracker:
    """A lost target can only be replaced by an explicit new selection epoch."""
    def __init__(self, config=None):
        self.config = config or Config()
        self.reset()

    def reset(self):
        self.anchor = None
        self.last = None
        self.plane_base = None
        self.lost = False

    def unavailable(self):
        if self.anchor is not None:
            self.lost = True

    def choose(self, result, shape):
        if self.lost:
            return None, 'target_lost_requires_new_generation'
        candidates = result.candidates
        if self.anchor is None:
            center = np.array([shape[1] / 2.0, shape[0] / 2.0])
            scored = sorted([(float(np.linalg.norm(c.center_uv - center)), index, c)
                             for index, c in enumerate(candidates)], key=lambda item: item[0])
            nearby = [item for item in scored if item[0] <= min(shape) * self.config.center_radius_fraction]
            if not nearby:
                return None, 'no_center_instance'
            if len(nearby) > 1 and nearby[1][0] - nearby[0][0] <= min(shape) * self.config.center_ambiguity_fraction:
                return None, 'ambiguous_center_instances'
            chosen = nearby[0][2]
            self.anchor = chosen
            self.plane_base = result.plane_base.copy()
        else:
            matching = _matching_candidates(candidates, self.anchor, self.last, self.config)
            if len(matching) != 1:
                self.lost = True
                return None, ('ambiguous_locked_instances' if matching else 'target_lost_requires_new_generation')
            chosen = matching[0][1]
        self.last = chosen
        return chosen, 'tracked' if self.last is not self.anchor else 'acquired'
