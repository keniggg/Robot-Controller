"""Confidence labels for current measured RGB-D, without temporal depth filling.

The full depth component is a proposal, not an irreversible foreground label:
stereo edge noise may lift a support/shadow patch above the height threshold.
Only its upper interior and locally planar side surfaces seed hard foreground.
"""
import cv2
import numpy as np


# Existing local plane-fit residual limit. Its use below as a narrow seed-band
# half-width is a conservative labeling scale, NOT a confidence interval for
# height above the independently measured support plane.
LOCAL_SURFACE_MAX_RMS_M = .0005


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
    reliable = ((count >= 21) & (variance <= LOCAL_SURFACE_MAX_RMS_M ** 2)
                & (variance / np.maximum(values.sum(-1), 1e-15) <= .04)
                & (local_max - local_min <= .008) & (values[..., 1] >= 1e-8))
    return vectors[..., 0], reliable


def _side_seed_colour_evidence(bgr, core, support, heights, upper_height, side_cosine):
    """Demote support-coloured depth ramps to probable, not hard, foreground.

    Smooth stereo edge errors can satisfy the local planar-side test. Compare
    their chromaticity with measured support and the measured upper interior
    in this same image. Keep upper seeds and dark/ambiguous pixels unchanged;
    this does not label a surface as background or change any depth value.
    """
    top = core & (heights >= upper_height)
    intensity = bgr.astype(float).sum(axis=-1)
    chroma = bgr.astype(float) / np.maximum(intensity, 1.)[..., None]
    lit = intensity > 60.
    background = (cv2.erode(support.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0) & lit
    report = dict(support_colour_demoted_side_pixels=0,
                  support_colour_candidate_demotions=0,
                  support_colour_demotion_minimum_pixels=25,
                  support_colour_qualified_regions=0,
                  support_colour_demotion_evidence='none',
                  support_colour_reference_pixels=int(np.count_nonzero(background)))
    if np.count_nonzero(background) < 25 or np.count_nonzero(top & lit) < 25:
        return core, report

    def distance(reference, query):
        # Quantization bounds the reference set independently of image size.
        colours = np.unique(np.round(chroma[reference] * 32.), axis=0) / 32.
        return np.concatenate([np.sqrt(np.min(np.sum(
            (chunk[:, None, :] - colours[None, :, :]) ** 2, axis=2), axis=1))
            for chunk in np.array_split(query, max(1, int(np.ceil(len(query) / 256.))))])

    # Weak chroma is readily changed by reflected table light on a dark
    # physical face. Preserve it and sides steeper than 60 degrees to the
    # support. Only shallow, strongly coloured ramps are ambiguous here.
    side = (core & ~top & lit & (np.ptp(chroma, axis=-1) > .5)
            & (side_cosine >= .5))
    if not np.any(side):
        return core, report
    query = chroma[side]
    support_distance = distance(background, query)
    top_distance = distance(top & lit, query)
    ambiguous_side = ((support_distance < .08)
                      & (top_distance > support_distance + .04))
    # Preserve the established strong relative-evidence case. Below that
    # fraction, a substantial CURRENT connected region remains contradictory
    # even when unrelated upper-face area grows. Scattered weak evidence must
    # not be summed to activate this extra case; never join or enlarge it.
    # Demotion supplies no background label or physical contour.
    report['support_colour_candidate_demotions'] = int(np.count_nonzero(ambiguous_side))
    if np.count_nonzero(ambiguous_side) < 25:
        return core, report
    contradictory = np.zeros_like(core, dtype=np.uint8)
    contradictory[side] = ambiguous_side
    count, components, stats, _ = cv2.connectedComponentsWithStats(contradictory, connectivity=8)
    qualified = [index for index in range(1, count) if stats[index, cv2.CC_STAT_AREA] >= 25]
    if np.count_nonzero(ambiguous_side) >= .1 * np.count_nonzero(core):
        demoted = contradictory.astype(bool)
        report['support_colour_demotion_evidence'] = 'established_relative_evidence'
    else:
        if not qualified:
            return core, report
        demoted = np.isin(components, qualified)
        report['support_colour_demotion_evidence'] = 'current_connected_region'
    result = core.copy()
    result[demoted] = False
    report['support_colour_qualified_regions'] = len(qualified)
    report['support_colour_demoted_side_pixels'] = int(np.count_nonzero(demoted))
    return result, report


def confidence_labels(seed, points, depth, heights, plane_normal, config, *, bgr=None):
    return _confidence_labels(seed, points, depth, heights, plane_normal, config,
                              bgr=bgr, local_fit=local_surface_normals)


def _confidence_labels(seed, points, depth, heights, plane_normal, config, *, bgr, local_fit):
    """Same label rules; a call-local owner may reuse identical local fits."""
    valid = (np.isfinite(depth) & (depth >= config.depth_min_m)
             & (depth <= config.depth_max_m) & np.all(np.isfinite(points), axis=-1))
    measured = (seed & valid & (heights >= config.foreground_min_m)
                & (heights <= config.foreground_max_m))
    if np.count_nonzero(measured) < config.min_object_points:
        raise ValueError('rgb_seed_has_insufficient_measured_foreground')
    core = cv2.erode(measured.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
    normals, reliable = local_fit(points, depth, valid)
    normal = np.asarray(plane_normal, dtype=float)
    normal = normal / np.linalg.norm(normal)
    cosine = np.abs(np.einsum('ijk,k->ij', normals, normal))
    # At 45 degrees the measured surface is closer to perpendicular than to
    # parallel with the support; shallow support ripples do not seed sides.
    side = core & reliable & (cosine <= np.sqrt(.5))
    upper_height = float(np.percentile(heights[measured], 90.))
    top = core & (heights >= upper_height)
    original_top_pixels = int(np.count_nonzero(top))
    upper_surface_band_used = False
    foreground = top | side
    # A low-profile object can have real side pixels inside the support
    # uncertainty band. Only make support depth an irreversible background
    # label when the upper surface clears the foreground threshold by both
    # sides of that band. Otherwise RGB must decide those pixels; their raw
    # depth still faces the same downstream geometry/contact requirements.
    separated = upper_height >= (config.foreground_min_m + 2 * config.support_threshold_m)
    background = valid & (np.abs(heights) <= config.support_threshold_m) & separated
    colour_evidence = {}
    if bgr is not None:
        foreground, colour_evidence = _side_seed_colour_evidence(
            bgr, foreground, background, heights, upper_height, cosine)
    if np.count_nonzero(foreground) < 25 and original_top_pixels < 25:
        # On a small noisy planar face, the highest decile can contain fewer
        # than 25 pixels even when its measured interior is reliable. Reuse
        # the SAME local fits; never expand to the full depth component or
        # admit a pixel outside its eroded, >=foreground_min_m interior.
        # Normal residual and support-relative height are different measures:
        # this narrow absolute band is not a statistical depth correction.
        top = (core & reliable & (cosine >= np.sqrt(.5))
               & (heights >= upper_height - LOCAL_SURFACE_MAX_RMS_M)
               & (heights <= upper_height + LOCAL_SURFACE_MAX_RMS_M))
        foreground = top | side
        if bgr is not None:
            foreground, colour_evidence = _side_seed_colour_evidence(
                bgr, foreground, background, heights, upper_height, cosine)
        upper_surface_band_used = True
    if np.any(foreground & background):
        raise ValueError('rgb_depth_foreground_support_conflict')
    if np.count_nonzero(foreground) < 25:
        raise ValueError('rgb_seed_core_too_small')
    return foreground, background, dict(
        seed_policy='measured_upper_interior_and_planar_sides',
        upper_seed_height_m=upper_height,
        original_top_pixels=original_top_pixels,
        upper_surface_band_used=upper_surface_band_used,
        upper_surface_band_half_width_m=(LOCAL_SURFACE_MAX_RMS_M
                                        if upper_surface_band_used else None),
        support_background_height_separated=bool(separated),
        trusted_top_pixels=int(np.count_nonzero(top)),
        trusted_side_pixels=int(np.count_nonzero(side)),
        trusted_core_pixels=int(np.count_nonzero(foreground)),
        measured_support_background_pixels=int(np.count_nonzero(background)), **colour_evidence)
