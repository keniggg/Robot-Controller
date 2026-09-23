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


def instance_mask_from_seed(bgr, depth_seed, *, geometry=None):
    color = np.asarray(bgr)
    seed = np.asarray(depth_seed) > 0
    if color.dtype != np.uint8 or color.ndim != 3 or color.shape[2] != 3 or color.shape[:2] != seed.shape:
        raise ValueError('rgb_seed_shape_or_encoding_invalid')
    ys, xs = np.nonzero(seed)
    if len(xs) < 60:
        raise ValueError('rgb_seed_has_insufficient_measured_foreground')
    margin = max(8, int(round(16 * min(seed.shape[1] / 640., seed.shape[0] / 480.))))
    x0, x1 = max(0, int(xs.min())-margin), min(seed.shape[1], int(xs.max())+margin+1)
    y0, y1 = max(0, int(ys.min())-margin), min(seed.shape[0], int(ys.max())+margin+1)
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
            measured > 0, points[sl], depth[sl], heights[sl], normal, config)
    if np.count_nonzero(core) < 25:
        raise ValueError('rgb_seed_core_too_small')
    near = cv2.dilate(measured, np.ones((15, 15), np.uint8)) > 0
    labels = np.full(measured.shape, cv2.GC_PR_BGD, np.uint8)
    labels[near] = cv2.GC_PR_FGD
    labels[core] = cv2.GC_FGD
    labels[support] = cv2.GC_BGD
    labels[:3] = labels[-3:] = cv2.GC_BGD
    labels[:, :3] = labels[:, -3:] = cv2.GC_BGD
    # OpenCV has process-global RNG state. Keep each measured-frame result
    # deterministic without persisting colour models from an earlier target.
    with _grabcut_lock:
        cv2.setRNGSeed(23)
        cv2.grabCut(patch, labels, None, np.zeros((1, 65)), np.zeros((1, 65)),
                    3, cv2.GC_INIT_WITH_MASK)
    foreground = np.isin(labels, (cv2.GC_FGD, cv2.GC_PR_FGD)).astype(np.uint8)
    count, components = cv2.connectedComponents(foreground, connectivity=8)
    # Keep only the single colour component containing the measured seed.
    # Independent neighbouring components never get merged into the target.
    matching = [index for index in range(1, count)
                if np.count_nonzero((components == index) & core) >= .85 * np.count_nonzero(core)]
    if len(matching) != 1:
        raise ValueError('rgb_seed_component_ambiguous_or_fragmented')
    chosen = components == matching[0]
    if (np.any(chosen[3]) or np.any(chosen[-4])
            or np.any(chosen[:, 3]) or np.any(chosen[:, -4])):
        raise ValueError('rgb_instance_reaches_search_boundary')
    retained = float(np.count_nonzero(chosen & (measured > 0))) / float(np.count_nonzero(measured))
    expansion = float(np.count_nonzero(chosen)) / float(np.count_nonzero(measured))
    trusted_retained = float(np.count_nonzero(chosen & core)) / float(np.count_nonzero(core))
    # The noisy full component remains probable foreground. Requiring 85% of
    # it would force shadows back into otherwise consistent RGB silhouettes.
    # Preserve the 85% component/core gate, expansion cap and downstream exact
    # measured-depth, three-frame IoU, contact and collision requirements.
    if (expansion > 4. or trusted_retained < .85
            or (geometry is None and (retained < .85 or expansion < .85))):
        raise ValueError('rgb_instance_not_supported_by_depth_seed')
    result = np.zeros(seed.shape, np.uint8)
    result[y0:y1, x0:x1] = chosen.astype(np.uint8) * 255
    return result, dict(mask_kind='current_rgb_instance', depth_seed_retained_fraction=retained,
                        rgb_to_depth_seed_area_ratio=expansion,
                        rgb_instance_pixels=int(np.count_nonzero(result)),
                        rgb_seed_pixels=int(np.count_nonzero(seed)),
                        trusted_core_retained_fraction=trusted_retained, **seed_evidence)


def refine_candidate(bgr, depth_m, intrinsics, transform_base_camera, candidate, config):
    depth = np.asarray(depth_m)
    transform = np.asarray(transform_base_camera)
    if depth.shape != candidate.mask.shape:
        raise ValueError('rgb_depth_shape_mismatch')
    plane = np.asarray(candidate.metrics['support_plane_base'], dtype=float)
    points = _project(depth, intrinsics)
    plane_camera = transform.T @ plane
    heights = np.einsum('ijk,k->ij', points, plane_camera[:3]) + plane_camera[3]
    mask, evidence = instance_mask_from_seed(
        bgr, candidate.mask, geometry=(points, depth, heights, plane_camera[:3], config))
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
