"""Current-image instance silhouettes seeded by measured foreground depth.

An instance mask describes pixel ownership, not depth validity. Depth remains
untouched: downstream geometry must still reject missing and support points.
No previous image/mask, semantic class, or completed surface is used here.
"""
from dataclasses import replace
import threading

import cv2
import numpy as np

from alicia_grasp_modes.tabletop import _project, _measured_support_footprint
from alicia_grasp_modes.measured_mask_seeds import confidence_labels


_grabcut_lock = threading.Lock()


def instance_mask_from_seed(bgr, depth_seed, *, geometry=None, rgb_valid=None):
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
        core, support, seed_evidence = confidence_labels(
            measured > 0, points[sl], depth[sl], heights[sl], normal, config, bgr=patch)
    if np.count_nonzero(core) < 25:
        raise ValueError('rgb_seed_core_too_small')
    # Freeze the measured trusted core from the first same-frame search. A
    # larger crop may supply more current background, never a new identity or
    # a different hard-foreground seed.
    first_bounds = (x0, y0, x1, y1)
    first_core, first_support = core.copy(), support.copy()

    def attempt(search_margin):
        ax0 = max(0, int(xs.min()) - search_margin)
        ax1 = min(seed.shape[1], int(xs.max()) + search_margin + 1)
        ay0 = max(0, int(ys.min()) - search_margin)
        ay1 = min(seed.shape[0], int(ys.max()) + search_margin + 1)
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
        if (np.any(chosen[3]) or np.any(chosen[-4])
                or np.any(chosen[:, 3]) or np.any(chosen[:, -4])):
            raise ValueError('rgb_instance_reaches_search_boundary')
        retained = float(np.count_nonzero(chosen & (measured_now > 0))) / float(np.count_nonzero(measured_now))
        expansion = float(np.count_nonzero(chosen)) / float(np.count_nonzero(measured_now))
        trusted_retained = float(np.count_nonzero(chosen & core_now)) / float(np.count_nonzero(core_now))
        if (expansion > 4. or trusted_retained < .85
                or (geometry is None and (retained < .85 or expansion < .85))):
            raise ValueError('rgb_instance_not_supported_by_depth_seed')
        result = np.zeros(seed.shape, np.uint8)
        result[ay0:ay1, ax0:ax1] = chosen.astype(np.uint8) * 255
        evidence = dict(seed_evidence)
        if geometry is not None:
            evidence['measured_support_background_pixels'] = int(np.count_nonzero(support_now))
        evidence.update(mask_kind='current_rgb_instance', depth_seed_retained_fraction=retained,
                        rgb_to_depth_seed_area_ratio=expansion,
                        rgb_instance_pixels=int(np.count_nonzero(result)),
                        rgb_seed_pixels=int(np.count_nonzero(seed)),
                        trusted_core_retained_fraction=trusted_retained,
                        rgb_search_margin_px=int(search_margin),
                        rgb_search_box_xyxy=[ax0, ay0, ax1, ay1],
                        rgb_trusted_core_source='first_same_frame_search')
        return result, evidence

    retry_reason = ''
    try:
        result, evidence = attempt(margin)
        attempts = 1
    except ValueError as exc:
        if str(exc) != 'rgb_instance_reaches_search_boundary':
            raise
        retry_reason = str(exc)
        # Exactly one same-source retry, not a search for a preferred mask.
        # A second boundary or any other original gate failure still rejects.
        result, evidence = attempt(2 * margin)
        attempts = 2
    evidence.update(rgb_search_attempts=attempts,
                    rgb_search_initial_margin_px=int(margin),
                    rgb_search_retry_reason=retry_reason)
    return result, evidence


def refine_candidate(bgr, depth_m, intrinsics, transform_base_camera, candidate, config, *, rgb_valid=None):
    depth = np.asarray(depth_m)
    transform = np.asarray(transform_base_camera)
    if depth.shape != candidate.mask.shape:
        raise ValueError('rgb_depth_shape_mismatch')
    plane = np.asarray(candidate.metrics['support_plane_base'], dtype=float)
    points = _project(depth, intrinsics)
    plane_camera = transform.T @ plane
    heights = np.einsum('ijk,k->ij', points, plane_camera[:3]) + plane_camera[3]
    mask, evidence = instance_mask_from_seed(
        bgr, candidate.mask, geometry=(points, depth, heights, plane_camera[:3], config),
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
