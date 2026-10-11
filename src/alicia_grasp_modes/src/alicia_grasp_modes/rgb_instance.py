"""Current-image instance silhouettes seeded by measured foreground depth.

An instance mask describes pixel ownership, not depth validity. Depth remains
untouched: downstream geometry must still reject missing and support points.
No previous image/mask, semantic class, or completed surface is used here.
"""
from dataclasses import replace
import threading
import hashlib

import cv2
import numpy as np

from alicia_grasp_modes.tabletop import _project, _measured_support_footprint, project_base_to_pixel
from alicia_grasp_modes.measured_mask_seeds import confidence_labels, _confidence_labels, local_surface_normals


_grabcut_lock = threading.Lock()


def freeze_observation_spatial_reference(anchor, plane):
    """Copy measured identity geometry; it supplies neither pixels nor depth."""
    footprint = np.asarray(anchor.support_footprint_base, dtype=float)
    plane = np.asarray(plane, dtype=float)
    height = anchor.metrics.get('median_height_m')
    if (footprint.ndim != 2 or footprint.shape[1:] != (3,) or len(footprint) < 3
            or not np.all(np.isfinite(footprint)) or plane.shape != (4,)
            or not np.all(np.isfinite(plane)) or np.linalg.norm(plane[:3]) < 1e-9
            or isinstance(height, (bool, np.bool_)) or not isinstance(height, (int, float))
            or not np.isfinite(height) or height <= 0.):
        raise ValueError('invalid_observation_spatial_reference')
    def immutable(value):
        value = np.ascontiguousarray(value, dtype=float)
        return np.frombuffer(value.tobytes(), dtype=float).reshape(value.shape)
    return dict(footprint_base=immutable(footprint), support_plane_base=immutable(plane),
                height_m=float(height))


def _observation_spatial_support(points_camera, transform_base_camera, intrinsics, reference, config):
    """A zero-expanded measured hull bounds CURRENT support and a crop proposal."""
    footprint = np.asarray(reference['footprint_base'], dtype=float)
    plane = np.asarray(reference['support_plane_base'], dtype=float)
    height = reference['height_m']
    transform = np.asarray(transform_base_camera, dtype=float)
    if (footprint.ndim != 2 or footprint.shape[1:] != (3,) or len(footprint) < 3
            or not np.all(np.isfinite(footprint)) or plane.shape != (4,)
            or not np.all(np.isfinite(plane)) or np.linalg.norm(plane[:3]) < 1e-9
            or isinstance(height, (bool, np.bool_)) or not isinstance(height, (int, float))
            or not config.foreground_min_m <= height <= config.foreground_max_m
            or transform.shape != (4, 4) or not np.all(np.isfinite(transform))):
        raise ValueError('invalid_observation_spatial_reference')
    normal = plane[:3] / np.linalg.norm(plane[:3])
    if np.max(np.abs(footprint @ normal + plane[3] / np.linalg.norm(plane[:3]))) > 1e-6:
        raise ValueError('observation_spatial_footprint_off_plane')
    axis = np.eye(3)[np.argmin(np.abs(normal))]
    u = np.cross(normal, axis); u /= np.linalg.norm(u)
    basis = np.stack((u, np.cross(normal, u)), axis=1)
    hull = cv2.convexHull(np.asarray(footprint @ basis, dtype=np.float32)).reshape(-1, 2)
    if cv2.contourArea(hull) <= 1e-8 or np.max(np.ptp(footprint @ basis, axis=0)) > config.max_surface_extent_m:
        raise ValueError('invalid_observation_spatial_footprint')
    points_base = points_camera @ transform[:3, :3].T + transform[:3, 3]
    projected = points_base @ basis
    domain = np.all(np.isfinite(points_base), axis=-1)
    for start, end in zip(hull, np.roll(hull, -1, axis=0)):
        edge = end - start
        domain &= edge[0]*(projected[..., 1]-start[1])-edge[1]*(projected[..., 0]-start[0]) >= -1e-9
    prism = np.vstack((footprint, footprint + normal * height))
    pixels = np.array([project_base_to_pixel(p, transform, intrinsics) for p in prism])
    if not np.all(np.isfinite(pixels)):
        raise ValueError('invalid_observation_spatial_projection')
    return dict(domain_mask=domain,
                search_box_xyxy=[*np.floor(pixels.min(axis=0)).astype(int),
                                 *(np.ceil(pixels.max(axis=0)).astype(int)+1)],
                provenance=dict(policy='current_depth_in_frozen_measured_footprint',
                    footprint_sha256=hashlib.sha256(np.ascontiguousarray(footprint).tobytes()).hexdigest(),
                    support_plane_sha256=hashlib.sha256(np.ascontiguousarray(plane).tobytes()).hexdigest(),
                    measured_height_m=float(height), footprint_expansion_m=0.,
                    historical_pixels_counted=0))


def _observed_spatial_search_bounds(rgb_valid, desired_bounds, first_bounds, proposal_bounds):
    """Trim optional background only, maximizing observed rectangular area.

    The original trusted-core crop and the complete measured prism remain
    inside the rectangle. Four pixels outside the prism preserve the original
    hard-background border and its adjacent boundary-detection line. The
    caller bounds enumeration by its original background padding (at most
    twice the scaled 16-pixel margin per edge), including a boundary retry.
    """
    valid = np.asarray(rgb_valid)
    boxes = [np.asarray(value) for value in (desired_bounds, first_bounds, proposal_bounds)]
    if (valid.ndim != 2 or valid.dtype != np.bool_
            or any(box.shape != (4,) or box.dtype.kind not in 'iu' for box in boxes)):
        raise ValueError('invalid_rgb_validity_search_bounds')
    dx0, dy0, dx1, dy1 = map(int, boxes[0])
    fx0, fy0, fx1, fy1 = map(int, boxes[1])
    px0, py0, px1, py1 = map(int, boxes[2])
    rx0, ry0 = min(fx0, px0-4), min(fy0, py0-4)
    rx1, ry1 = max(fx1, px1+4), max(fy1, py1+4)
    if (not 0 <= dx0 <= rx0 < rx1 <= dx1 <= valid.shape[1]
            or not 0 <= dy0 <= ry0 < ry1 <= dy1 <= valid.shape[0]
            or not np.all(valid[ry0:ry1, rx0:rx1])):
        raise ValueError('rgb_required_spatial_region_unobserved')
    bad = ~valid[dy0:dy1, dx0:dx1]
    prefix = np.vstack((np.zeros((1, bad.shape[1]), np.int32),
                        np.cumsum(bad, axis=0, dtype=np.int32)))
    best, best_score = None, None
    for top in range(dy0, ry0+1):
        for bottom in range(ry1, dy1+1):
            columns = prefix[bottom-dy0] - prefix[top-dy0] > 0
            if np.any(columns[rx0-dx0:rx1-dx0]):
                continue
            left_bad = np.flatnonzero(columns[:rx0-dx0])
            right_bad = np.flatnonzero(columns[rx1-dx0:])
            left = dx0 if not len(left_bad) else dx0+int(left_bad[-1])+1
            right = dx1 if not len(right_bad) else rx1+int(right_bad[0])
            # Largest area first, then a deterministic coordinate ordering.
            score = (-(right-left)*(bottom-top), left, top, right, bottom)
            if best_score is None or score < best_score:
                best_score, best = score, (left, top, right, bottom)
    if best is None or not np.all(valid[best[1]:best[3], best[0]:best[2]]):
        raise ValueError('rgb_required_spatial_region_unobserved')
    return best


def instance_mask_from_seed(bgr, depth_seed, *, geometry=None, rgb_valid=None,
                            observation_arrival_retry=False, observation_spatial_support=None,
                            contact_spatial_support=None):
    return _instance_mask_from_seed(bgr, depth_seed, geometry=geometry,
        rgb_valid=rgb_valid, observation_arrival_retry=observation_arrival_retry,
        observation_spatial_support=observation_spatial_support,
        contact_spatial_support=contact_spatial_support,
        confidence_labeler=confidence_labels)


def _instance_mask_from_seed(bgr, depth_seed, *, geometry, rgb_valid,
                            observation_arrival_retry, observation_spatial_support,
                            confidence_labeler, contact_spatial_support=None):
    if type(observation_arrival_retry) is not bool:
        raise ValueError('invalid_observation_arrival_retry')
    if observation_spatial_support is not None and (not observation_arrival_retry or geometry is None):
        raise ValueError('observation_spatial_support_requires_arrival_geometry')
    if contact_spatial_support is not None and (geometry is None or observation_arrival_retry
                                               or observation_spatial_support is not None):
        raise ValueError('contact_spatial_support_requires_independent_current_geometry')
    color = np.asarray(bgr)
    seed = np.asarray(depth_seed) > 0
    if color.dtype != np.uint8 or color.ndim != 3 or color.shape[2] != 3 or color.shape[:2] != seed.shape:
        raise ValueError('rgb_seed_shape_or_encoding_invalid')
    if rgb_valid is not None:
        rgb_valid = np.asarray(rgb_valid)
        if rgb_valid.shape != seed.shape or rgb_valid.dtype != np.bool_:
            raise ValueError('invalid_rgb_validity_mask')
        if np.any(seed & ~rgb_valid):
            raise ValueError('rgb_seed_contains_unobserved_rgb')
    ys, xs = np.nonzero(seed)
    if len(xs) < 60:
        raise ValueError('rgb_seed_has_insufficient_measured_foreground')
    margin = max(8, int(round(16 * min(seed.shape[1] / 640., seed.shape[0] / 480.))))
    x0, x1 = max(0, int(xs.min())-margin), min(seed.shape[1], int(xs.max())+margin+1)
    y0, y1 = max(0, int(ys.min())-margin), min(seed.shape[0], int(ys.max())+margin+1)
    if rgb_valid is not None and not np.all(rgb_valid[y0:y1, x0:x1]):
        raise ValueError('rgb_search_contains_unobserved_rgb')
    patch = np.ascontiguousarray(color[y0:y1, x0:x1])
    measured = seed[y0:y1, x0:x1].astype(np.uint8)
    support = np.zeros(measured.shape, dtype=bool)
    seed_evidence = dict(seed_policy='legacy_depth_component')
    if geometry is None:
        core = cv2.erode(measured, np.ones((3, 3), np.uint8)) > 0
    else:
        points, depth, heights, normal, config = geometry
        if (points.shape != seed.shape + (3,) or depth.shape != seed.shape
                or heights.shape != seed.shape):
            raise ValueError('rgb_depth_geometry_shape_mismatch')
        sl = np.s_[y0:y1, x0:x1]
        core, support, seed_evidence = confidence_labeler(
            measured > 0, points[sl], depth[sl], heights[sl], normal, config, bgr=patch)
    if np.count_nonzero(core) < 25:
        raise ValueError('rgb_seed_core_too_small')
    # Freeze the measured trusted core from the first same-frame search. A
    # larger crop may supply more current background, never a new identity or
    # a different hard-foreground seed.
    first_bounds = (x0, y0, x1, y1)
    first_core, first_support = core.copy(), support.copy()
    arrival_geometry = observation_arrival_retry and geometry is not None
    spatial = observation_spatial_support if contact_spatial_support is None else contact_spatial_support
    if spatial is not None:
        domain = np.asarray(spatial['domain_mask'])
        box = np.asarray(spatial['search_box_xyxy'])
        if (domain.shape != seed.shape or domain.dtype != np.bool_ or box.shape != (4,)
                or box.dtype.kind not in 'iu' or box[0] >= box[2] or box[1] >= box[3]):
            raise ValueError('invalid_observation_spatial_support')
    boundary_sides = ()
    side_names = ('left', 'top', 'right', 'bottom')

    def attempt(search_margin, *, side_margins=None):
        nonlocal boundary_sides
        margins = (search_margin,) * 4 if side_margins is None else side_margins
        left, top, right, bottom = margins
        ax0 = max(0, int(xs.min()) - left)
        ax1 = min(seed.shape[1], int(xs.max()) + right + 1)
        ay0 = max(0, int(ys.min()) - top)
        ay1 = min(seed.shape[0], int(ys.max()) + bottom + 1)
        if spatial is not None:
            # The measured prism is only a crop proposal. Include the frozen
            # first core and real background without importing historical RGB.
            extra = 0 if side_margins is None else margin
            ax0 = max(0, min(x0, int(box[0])-margin-(extra if 'left' in boundary_sides else 0)))
            ay0 = max(0, min(y0, int(box[1])-margin-(extra if 'top' in boundary_sides else 0)))
            ax1 = min(seed.shape[1], max(x1, int(box[2])+margin+(extra if 'right' in boundary_sides else 0)))
            ay1 = min(seed.shape[0], max(y1, int(box[3])+margin+(extra if 'bottom' in boundary_sides else 0)))
        desired_bounds = (ax0, ay0, ax1, ay1)
        if (spatial is not None and rgb_valid is not None
                and not np.all(rgb_valid[ay0:ay1, ax0:ax1])):
            ax0, ay0, ax1, ay1 = _observed_spatial_search_bounds(
                rgb_valid, desired_bounds, first_bounds, box)
        # Unobserved black padding must not train a foreground/background
        # colour model. Refuse this RGB estimate without editing real depth.
        if rgb_valid is not None and not np.all(rgb_valid[ay0:ay1, ax0:ax1]):
            raise ValueError('rgb_search_contains_unobserved_rgb')
        current = np.ascontiguousarray(color[ay0:ay1, ax0:ax1])
        measured_now = seed[ay0:ay1, ax0:ax1].astype(np.uint8)
        core_now = np.zeros(measured_now.shape, dtype=bool)
        core_now[y0-ay0:y1-ay0, x0-ax0:x1-ax0] = first_core
        if (ax0, ay0, ax1, ay1) == first_bounds:
            support_now = first_support
        elif geometry is None:
            support_now = np.zeros(measured_now.shape, dtype=bool)
        else:
            # This is the same measured-support label rule as confidence_labels,
            # extended to the larger CURRENT crop with the frozen seed height.
            sl = np.s_[ay0:ay1, ax0:ax1]
            valid = (np.isfinite(depth[sl]) & (depth[sl] >= config.depth_min_m)
                     & (depth[sl] <= config.depth_max_m)
                     & np.all(np.isfinite(points[sl]), axis=-1))
            support_now = (valid & (np.abs(heights[sl]) <= config.support_threshold_m)
                           & seed_evidence['support_background_height_separated'])
        if np.any(core_now & support_now):
            raise ValueError('rgb_depth_foreground_support_conflict')
        near = cv2.dilate(measured_now, np.ones((15, 15), np.uint8)) > 0
        if arrival_geometry:
            # Other current depth fragments are only probable foreground. They
            # do not change the frozen hard core or claim another identity.
            sl = np.s_[ay0:ay1, ax0:ax1]
            near |= (np.isfinite(depth[sl])
                     & (depth[sl] >= config.depth_min_m)
                     & (depth[sl] <= config.depth_max_m)
                     & np.all(np.isfinite(points[sl]), axis=-1)
                     & (heights[sl] >= config.foreground_min_m)
                     & (heights[sl] <= config.foreground_max_m))
        labels = np.full(measured_now.shape, cv2.GC_PR_BGD, np.uint8)
        labels[near] = cv2.GC_PR_FGD
        labels[core_now] = cv2.GC_FGD
        labels[support_now] = cv2.GC_BGD
        labels[:3] = labels[-3:] = cv2.GC_BGD
        labels[:, :3] = labels[:, -3:] = cv2.GC_BGD
        # OpenCV has process-global RNG state. Neither attempt persists colour
        # models or masks from another image, target, or this first attempt.
        with _grabcut_lock:
            cv2.setRNGSeed(23)
            cv2.grabCut(current, labels, None, np.zeros((1, 65)), np.zeros((1, 65)),
                        3, cv2.GC_INIT_WITH_MASK)
        foreground = np.isin(labels, (cv2.GC_FGD, cv2.GC_PR_FGD)).astype(np.uint8)
        count, components = cv2.connectedComponents(foreground, connectivity=8)
        matching = [index for index in range(1, count)
                    if np.count_nonzero((components == index) & core_now)
                    >= .85 * np.count_nonzero(core_now)]
        if len(matching) != 1:
            raise ValueError('rgb_seed_component_ambiguous_or_fragmented')
        chosen = components == matching[0]
        boundary_sides = tuple(name for name, touches in zip(side_names, (
            np.any(chosen[:, 3]), np.any(chosen[3]),
            np.any(chosen[:, -4]), np.any(chosen[-4]))) if touches)
        if boundary_sides:
            raise ValueError('rgb_instance_reaches_search_boundary')
        retained = float(np.count_nonzero(chosen & (measured_now > 0))) / float(np.count_nonzero(measured_now))
        expansion = float(np.count_nonzero(chosen)) / float(np.count_nonzero(measured_now))
        trusted_retained = float(np.count_nonzero(chosen & core_now)) / float(np.count_nonzero(core_now))
        support_expansion = expansion
        support_audit = {}
        if spatial is not None:
            sl = np.s_[ay0:ay1, ax0:ax1]
            actual = (np.isfinite(depth[sl]) & (depth[sl] >= config.depth_min_m)
                      & (depth[sl] <= config.depth_max_m)
                      & np.all(np.isfinite(points[sl]), axis=-1)
                      & (heights[sl] >= config.foreground_min_m)
                      & (heights[sl] <= config.foreground_max_m))
            support_count = int(np.count_nonzero(chosen & actual & domain[sl]))
            if support_count < config.min_object_points:
                raise ValueError('rgb_instance_has_insufficient_current_spatial_support')
            support_expansion = float(np.count_nonzero(chosen)) / support_count
            support_audit = dict(rgb_to_current_measured_support_area_ratio=support_expansion,
                current_measured_support_pixels=support_count,
                current_measured_foreground_outside_domain_pixels=int(np.count_nonzero(chosen & actual & ~domain[sl])),
                current_measured_support_region=dict(spatial['provenance']))
        if (support_expansion > 4. or trusted_retained < .85
                or (geometry is None and (retained < .85 or expansion < .85))):
            raise ValueError('rgb_instance_not_supported_by_current_target_depth' if spatial is not None
                             else 'rgb_instance_not_supported_by_depth_seed')
        result = np.zeros(seed.shape, np.uint8)
        result[ay0:ay1, ax0:ax1] = chosen.astype(np.uint8) * 255
        evidence = dict(seed_evidence)
        evidence.update(support_audit)
        if geometry is not None:
            evidence['measured_support_background_pixels'] = int(np.count_nonzero(support_now))
        evidence.update(mask_kind='current_rgb_instance', depth_seed_retained_fraction=retained,
                        rgb_to_depth_seed_area_ratio=expansion,
                        rgb_instance_pixels=int(np.count_nonzero(result)),
                        rgb_seed_pixels=int(np.count_nonzero(seed)),
                        trusted_core_retained_fraction=trusted_retained,
                        rgb_search_margin_px=int(search_margin),
                        rgb_search_margins_ltrb_px=[int(value) for value in margins],
                        rgb_search_box_xyxy=[ax0, ay0, ax1, ay1],
                        rgb_trusted_core_source='first_same_frame_search')
        if spatial is not None:
            clipped = (ax0, ay0, ax1, ay1) != desired_bounds
            evidence.update(rgb_search_policy='measured_geometry_proposal',
                rgb_proposal_background_margin_px=int(margin),
                rgb_search_desired_box_xyxy=list(desired_bounds),
                rgb_search_observed_background_clipped=clipped,
                rgb_search_clipping_required_box_xyxy=(
                    [min(x0, int(box[0])-4), min(y0, int(box[1])-4),
                     max(x1, int(box[2])+4), max(y1, int(box[3])+4)] if clipped else None),
                rgb_search_preserved_prism_guard_px=4 if clipped else 0,
                rgb_search_background_invalid_pixels_removed=(int(np.count_nonzero(
                    ~rgb_valid[desired_bounds[1]:desired_bounds[3], desired_bounds[0]:desired_bounds[2]]))
                    if clipped else 0),
                rgb_search_margins_ltrb_px=[int(xs.min()-ax0), int(ys.min()-ay0),
                                          int(ax1-xs.max()-1), int(ay1-ys.max()-1)])
        return result, evidence

    retry_reason = ''
    expanded_boundary_sides = ()
    if arrival_geometry or contact_spatial_support is not None:
        # A transported target can yield only one depth fragment. Its small
        # crop border may lie inside the visible object and train its colour
        # as hard background without the selected component reaching the edge.
        # Keep the original small-crop core, but give current RGB a bounded
        # background context from the outset, including at the same station.
        try:
            result, evidence = attempt((3 if arrival_geometry else 1) * margin)
            attempts = 1
        except ValueError as exc:
            if str(exc) != 'rgb_instance_reaches_search_boundary':
                raise
            retry_reason = str(exc)
            expanded_boundary_sides = boundary_sides
            side_margins = tuple((4 if side in boundary_sides else 3) * margin
                                 for side in side_names)
            result, evidence = attempt(4 * margin, side_margins=side_margins)
            attempts = 2
    else:
        # Ordinary acquisition and grasp perception retain exactly one retry.
        try:
            result, evidence = attempt(margin)
            attempts = 1
        except ValueError as exc:
            if str(exc) != 'rgb_instance_reaches_search_boundary':
                raise
            retry_reason = str(exc)
            try:
                result, evidence = attempt(2 * margin)
                attempts = 2
            except ValueError as exc:
                if (not observation_arrival_retry
                        or str(exc) != 'rgb_instance_reaches_search_boundary'):
                    raise
                expanded_boundary_sides = boundary_sides
                side_margins = tuple((4 if side in boundary_sides else 3) * margin
                                     for side in side_names)
                result, evidence = attempt(4 * margin, side_margins=side_margins)
                attempts = 3
    evidence.update(rgb_contact_spatial_proposal=contact_spatial_support is not None,
                    rgb_search_attempts=attempts,
                    rgb_search_initial_margin_px=int((3 if arrival_geometry and spatial is None else 1) * margin),
                    rgb_trusted_core_search_margin_px=int(margin),
                    rgb_search_expanded_boundary_sides=list(expanded_boundary_sides),
                    rgb_search_retry_reason=retry_reason)
    return result, evidence


def refine_candidate(bgr, depth_m, intrinsics, transform_base_camera, candidate, config, *,
                     rgb_valid=None, observation_arrival_retry=False, observation_spatial_reference=None,
                     contact_spatial_reference=None):
    depth = np.asarray(depth_m)
    transform = np.asarray(transform_base_camera)
    if depth.shape != candidate.mask.shape:
        raise ValueError('rgb_depth_shape_mismatch')
    plane = np.asarray(candidate.metrics['support_plane_base'], dtype=float)
    points = _project(depth, intrinsics)
    plane_camera = transform.T @ plane
    heights = np.einsum('ijk,k->ij', points, plane_camera[:3]) + plane_camera[3]
    spatial_kwargs = {}
    if contact_spatial_reference is not None:
        if observation_spatial_reference is not None or observation_arrival_retry:
            raise ValueError('contact_spatial_reference_is_not_arrival_authority')
        spatial_kwargs['contact_spatial_support'] = _observation_spatial_support(
            points, transform, intrinsics, contact_spatial_reference, config)
    if observation_spatial_reference is not None:
        if observation_arrival_retry is not True:
            raise ValueError('observation_spatial_support_requires_arrival_geometry')
        spatial_kwargs['observation_spatial_support'] = _observation_spatial_support(
            points, transform, intrinsics, observation_spatial_reference, config)
    return _refine_candidate_measured(bgr, depth, transform, candidate, config,
        points, plane, plane_camera, heights, spatial_kwargs,
        rgb_valid=rgb_valid, observation_arrival_retry=observation_arrival_retry,
        mask_builder=instance_mask_from_seed)


def _refine_candidate_measured(bgr, depth, transform, candidate, config,
        points, plane, plane_camera, heights, spatial_kwargs, *, rgb_valid,
        observation_arrival_retry, mask_builder):
    mask, evidence = mask_builder(
        bgr, candidate.mask, geometry=(points, depth, heights, plane_camera[:3], config),
        observation_arrival_retry=observation_arrival_retry,
        **spatial_kwargs,
        **({} if rgb_valid is None else {'rgb_valid': rgb_valid}))
    measured = ((mask > 0) & np.isfinite(depth)
                & (depth >= config.depth_min_m) & (depth <= config.depth_max_m)
                & (heights >= config.foreground_min_m) & (heights <= config.foreground_max_m))
    if np.count_nonzero(measured) < config.min_object_points:
        raise ValueError('rgb_instance_has_insufficient_measured_depth')
    xyz = points[measured].astype(float)
    p_camera = np.median(xyz, axis=0)
    p_base = (transform @ np.r_[p_camera, 1.])[:3]
    if np.linalg.norm(p_base-candidate.position_base) > config.association_distance_m:
        raise ValueError('rgb_instance_shifted_from_measured_target')
    centered = xyz-np.mean(xyz, axis=0)
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    extent = np.sort(np.percentile(centered @ vh.T, 98, axis=0)
                     - np.percentile(centered @ vh.T, 2, axis=0))[::-1]
    if extent[0] > config.max_surface_extent_m:
        raise ValueError('rgb_instance_measured_extent_too_large')
    ys, xs = np.nonzero(mask)
    bbox = (int(xs.min()), int(ys.min()), int(xs.max()-xs.min()+1), int(ys.max()-ys.min()+1))
    evidence.update(candidate.metrics)
    evidence.update(depth_seed_extent_m=candidate.extent_m.tolist(),
                    observed_surface_extent_m=extent.tolist(),
                    valid_points=int(np.count_nonzero(measured)),
                    rgb_mask_pixels_without_valid_depth=int(np.count_nonzero(
                        (mask > 0) & ~(np.isfinite(depth) & (depth >= config.depth_min_m)
                                      & (depth <= config.depth_max_m)))))
    points_base = xyz @ transform[:3, :3].T + transform[:3, 3]
    # Return a separate presentation/geometry observation. The original depth
    # tracker retains its immutable anchor, association limits and loss latch.
    return replace(candidate, mask=mask, center_uv=np.array([np.mean(xs), np.mean(ys)]),
                   bbox=bbox, position_camera=p_camera, position_base=p_base,
                   extent_m=extent, depth_m=float(p_camera[0]), metrics=evidence,
                   support_footprint_base=_measured_support_footprint(points_base, plane))


class _FrameRefinement:
    """Pure computed data owned by one fragment-recovery call, never a receipt.

    No caller can supply a precomputed cache to the public refinement API.
    Cached arrays are read-only and local fits are reused only for identical
    point/depth/validity values. Heights and all label decisions stay separate.
    """
    def __init__(self, depth, intrinsics, transform, reference, config):
        self.depth = np.asarray(depth)
        self.transform = np.asarray(transform)
        self.config = config
        self.points = _project(self.depth, intrinsics)
        self.points.setflags(write=False)
        self.spatial = _observation_spatial_support(
            self.points, self.transform, intrinsics, reference, config)
        self.spatial['domain_mask'].setflags(write=False)
        self._fit = None

    def _local_fit(self, points, depth, valid):
        previous = self._fit
        if previous is not None and all(np.array_equal(a, b)
                for a, b in zip((points, depth, valid), previous[:3])):
            return previous[3]
        result = local_surface_normals(points, depth, valid)
        copies = tuple(np.array(value, copy=True) for value in (points, depth, valid))
        for value in copies + result:
            value.setflags(write=False)
        self._fit = copies + (result,)
        return result

    def confidence_labels(self, seed, points, depth, heights, plane_normal, config, *, bgr=None):
        return _confidence_labels(seed, points, depth, heights, plane_normal, config,
                                  bgr=bgr, local_fit=self._local_fit)

    def _mask(self, bgr, depth_seed, *, geometry, observation_arrival_retry,
              observation_spatial_support, rgb_valid=None):
        return _instance_mask_from_seed(bgr, depth_seed, geometry=geometry,
            rgb_valid=rgb_valid, observation_arrival_retry=observation_arrival_retry,
            observation_spatial_support=observation_spatial_support,
            confidence_labeler=self.confidence_labels)

    def refine(self, bgr, candidate, *, rgb_valid=None):
        if self.depth.shape != candidate.mask.shape:
            raise ValueError('rgb_depth_shape_mismatch')
        plane = np.asarray(candidate.metrics['support_plane_base'], dtype=float)
        plane_camera = self.transform.T @ plane
        # Preserve the original refinement arithmetic. The fragment proposal
        # separately uses matmul; threshold-boundary rounding is not unified.
        heights = np.einsum('ijk,k->ij', self.points, plane_camera[:3]) + plane_camera[3]
        return _refine_candidate_measured(bgr, self.depth, self.transform,
            candidate, self.config, self.points, plane, plane_camera, heights,
            {'observation_spatial_support': self.spatial}, rgb_valid=rgb_valid,
            observation_arrival_retry=True, mask_builder=self._mask)
