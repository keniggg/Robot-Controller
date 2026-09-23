"""Confidence labels for current measured RGB-D, without temporal depth filling.

The full depth component is a proposal, not an irreversible foreground label:
stereo edge noise may lift a support/shadow patch above the height threshold.
Only its upper interior and locally planar side surfaces seed hard foreground.
"""
import cv2
import numpy as np


def local_surface_normals(points, depth, valid):
    weight = valid.astype(np.float64)
    xyz = np.where(valid[..., None], points, 0.)
    count = cv2.boxFilter(weight, -1, (5, 5), normalize=False, borderType=cv2.BORDER_CONSTANT)
    denominator = np.maximum(count, 1.)
    mean = np.stack([cv2.boxFilter(xyz[..., i], -1, (5, 5), normalize=False,
        borderType=cv2.BORDER_CONSTANT) / denominator for i in range(3)], axis=-1)
    covariance = np.empty(points.shape[:2] + (3, 3))
    for i in range(3):
        for j in range(3):
            covariance[..., i, j] = cv2.boxFilter(xyz[..., i] * xyz[..., j], -1,
                (5, 5), normalize=False, borderType=cv2.BORDER_CONSTANT) / denominator - mean[..., i] * mean[..., j]
    values, vectors = np.linalg.eigh(covariance)
    variance = np.maximum(values[..., 0], 0.)
    local_max = cv2.dilate(np.where(valid, depth, 0.).astype(np.float32), np.ones((5, 5), np.uint8))
    local_min = cv2.erode(np.where(valid, depth, 1e4).astype(np.float32), np.ones((5, 5), np.uint8))
    reliable = ((count >= 21) & (variance <= .0005 ** 2)
                & (variance / np.maximum(values.sum(-1), 1e-15) <= .04)
                & (local_max - local_min <= .008) & (values[..., 1] >= 1e-8))
    return vectors[..., 0], reliable


def confidence_labels(seed, points, depth, heights, plane_normal, config):
    valid = (np.isfinite(depth) & (depth >= config.depth_min_m)
             & (depth <= config.depth_max_m) & np.all(np.isfinite(points), axis=-1))
    measured = (seed & valid & (heights >= config.foreground_min_m)
                & (heights <= config.foreground_max_m))
    if np.count_nonzero(measured) < config.min_object_points:
        raise ValueError('rgb_seed_has_insufficient_measured_foreground')
    core = cv2.erode(measured.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
    normals, reliable = local_surface_normals(points, depth, valid)
    normal = np.asarray(plane_normal, dtype=float)
    normal = normal / np.linalg.norm(normal)
    cosine = np.abs(np.einsum('ijk,k->ij', normals, normal))
    # At 45 degrees the measured surface is closer to perpendicular than to
    # parallel with the support; shallow support ripples do not seed sides.
    side = core & reliable & (cosine <= np.sqrt(.5))
    upper_height = float(np.percentile(heights[measured], 90.))
    top = core & (heights >= upper_height)
    foreground = top | side
    # A low-profile object can have real side pixels inside the support
    # uncertainty band. Only make support depth an irreversible background
    # label when the upper surface clears the foreground threshold by both
    # sides of that band. Otherwise RGB must decide those pixels; their raw
    # depth still faces the same downstream geometry/contact requirements.
    separated = upper_height >= (config.foreground_min_m + 2 * config.support_threshold_m)
    background = valid & (np.abs(heights) <= config.support_threshold_m) & separated
    if np.any(foreground & background):
        raise ValueError('rgb_depth_foreground_support_conflict')
    if np.count_nonzero(foreground) < 25:
        raise ValueError('rgb_seed_core_too_small')
    return foreground, background, dict(
        seed_policy='measured_upper_interior_and_planar_sides',
        upper_seed_height_m=upper_height,
        support_background_height_separated=bool(separated),
        trusted_top_pixels=int(np.count_nonzero(top)),
        trusted_side_pixels=int(np.count_nonzero(side)),
        trusted_core_pixels=int(np.count_nonzero(foreground)),
        measured_support_background_pixels=int(np.count_nonzero(background)))
