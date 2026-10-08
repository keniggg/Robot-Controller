"""Two current-frame ownership hypotheses, for diagnosis only.

Neither hypothesis is a replacement instance mask. In particular, a smaller
colour-derived mask can erase a real support-coloured face. This module only
reports disagreement; it cannot produce geometry or authorize an observation,
contact, or motion. Callers supply one current search crop and explicit source
stamps for every input. No images, masks, or colour models persist across calls.
"""
import hashlib

import cv2
import numpy as np

from alicia_grasp_modes.rgb_instance import _grabcut_lock


_INPUT_NAMES = ('rgb', 'depth', 'seed', 'hard_foreground', 'hard_background', 'height')


def _mask_summary(mask, hard_foreground, hard_background, valid_depth):
    ys, xs = np.nonzero(mask)
    core_count = int(np.count_nonzero(hard_foreground))
    retained = int(np.count_nonzero(mask & hard_foreground))
    return dict(
        pixel_count=int(len(xs)),
        bbox_xywh=(None if not len(xs) else [int(xs.min()), int(ys.min()),
            int(xs.max()-xs.min()+1), int(ys.max()-ys.min()+1)]),
        sha256=hashlib.sha256(mask.astype(np.uint8).tobytes()).hexdigest(),
        valid_depth_pixels=int(np.count_nonzero(mask & valid_depth)),
        missing_or_invalid_depth_pixels=int(np.count_nonzero(mask & ~valid_depth)),
        hard_foreground_retained_pixels=retained,
        hard_foreground_retained_fraction=float(retained/core_count),
        hard_background_included_pixels=int(np.count_nonzero(mask & hard_background)))


def _counterfactual_prior(bgr, core, hard_background, near, heights, valid,
                          upper_height, height_separated, support_band_m):
    """Soft alternative only; support-like colour does not establish ownership."""
    prior = np.zeros(core.shape, dtype=bool)
    evidence = dict(probable_background_pixels=0, support_reference_pixels=0,
                    upper_reference_pixels=0,
                    source='same_crop_measured_support_chroma_counterfactual')
    if not height_separated:
        return prior, evidence
    intensity = bgr.astype(float).sum(axis=-1)
    chroma = bgr.astype(float)/np.maximum(intensity, 1.)[..., None]
    lit = intensity > 60.
    # Exclude artificial crop-border background from the colour reference
    # unless its independent current depth also measures support height.
    measured_support = hard_background & valid & (np.abs(heights) <= support_band_m)
    background = (cv2.erode(measured_support.astype(np.uint8),
                           np.ones((3, 3), np.uint8)) > 0) & lit
    top = core & valid & (heights >= upper_height) & lit
    evidence.update(support_reference_pixels=int(np.count_nonzero(background)),
                    upper_reference_pixels=int(np.count_nonzero(top)))
    if np.count_nonzero(background) < 25 or np.count_nonzero(top) < 25:
        return prior, evidence
    candidate = near & ~core & ~hard_background & lit & (np.ptp(chroma, axis=-1) > .5)
    if not np.any(candidate):
        return prior, evidence

    def distance(reference):
        colours = np.unique(np.round(chroma[reference]*32.), axis=0)/32.
        query = chroma[candidate]
        return np.concatenate([np.sqrt(np.min(np.sum(
            (chunk[:, None, :]-colours[None, :, :])**2, axis=2), axis=1))
            for chunk in np.array_split(query, max(1, int(np.ceil(len(query)/256.))))])

    support_distance, top_distance = distance(background), distance(top)
    prior[candidate] = ((support_distance < .08)
                        & (top_distance > support_distance+.04))
    evidence['probable_background_pixels'] = int(np.count_nonzero(prior))
    return prior, evidence


def evaluate_silhouette_ambiguity(
        bgr, depth_m, depth_seed, hard_foreground, hard_background, heights_m,
        *, source_stamps_ns, upper_seed_height_m, support_height_separated,
        depth_min_m=.03, depth_max_m=2., support_band_m=.002):
    """Return JSON-only summaries of exactly two same-crop hypotheses.

    ``source_stamps_ns`` must bind rgb/depth/seed/hard_foreground/
    hard_background/height to the same positive integer source timestamp.
    Arrays use the caller's existing crop and hard labels (including any crop
    border). This function neither chooses a crop nor retries with another.
    ``original`` is the original probable-label hypothesis recomputed from
    these inputs, not an assertion that a recorded production mask is correct.
    No mask arrays, points, executable geometry, or preferred hypothesis leave
    this function. Even no disagreement does not certify a physical contour.
    """
    if not isinstance(source_stamps_ns, dict) or set(source_stamps_ns) != set(_INPUT_NAMES):
        raise ValueError('missing_or_extra_source_stamp_binding')
    stamps = [source_stamps_ns[name] for name in _INPUT_NAMES]
    if any(type(stamp) is not int or stamp <= 0 for stamp in stamps) or len(set(stamps)) != 1:
        raise ValueError('inputs_must_share_one_positive_source_stamp')
    color = np.asarray(bgr)
    depth, heights = np.asarray(depth_m), np.asarray(heights_m)
    masks = [np.asarray(m) for m in (depth_seed, hard_foreground, hard_background)]
    if (color.dtype != np.uint8 or color.ndim != 3 or color.shape[-1] != 3
            or min(color.shape[:2]) < 7 or depth.shape != color.shape[:2]
            or heights.shape != depth.shape or any(m.shape != depth.shape for m in masks)
            or any(m.dtype not in (np.dtype(bool), np.dtype(np.uint8)) for m in masks)):
        raise ValueError('invalid_same_crop_input_shape_or_type')
    parameters = (upper_seed_height_m, depth_min_m, depth_max_m, support_band_m)
    if (not all(np.isfinite(v) for v in parameters) or not 0 < depth_min_m < depth_max_m
            or support_band_m <= 0 or type(support_height_separated) is not bool):
        raise ValueError('invalid_current_measurement_parameters')
    seed, core, background = [m > 0 for m in masks]
    valid = (np.isfinite(depth) & (depth >= depth_min_m) & (depth <= depth_max_m))
    if np.any(valid & ~np.isfinite(heights)):
        raise ValueError('valid_depth_requires_current_height')
    if np.any(core & background) or np.any(core & ~seed) or np.any(core & ~valid):
        raise ValueError('contradictory_or_unmeasured_hard_foreground')
    if np.count_nonzero(core) < 25 or not np.any(background):
        raise ValueError('insufficient_original_hard_labels')
    near = cv2.dilate(seed.astype(np.uint8), np.ones((15, 15), np.uint8)) > 0
    initial = np.full(seed.shape, cv2.GC_PR_BGD, np.uint8)
    initial[near] = cv2.GC_PR_FGD
    initial[core] = cv2.GC_FGD
    initial[background] = cv2.GC_BGD
    prior, prior_evidence = _counterfactual_prior(
        color, core, background, near, heights, valid, float(upper_seed_height_m),
        support_height_separated, float(support_band_m))
    alternative = initial.copy()
    alternative[prior & np.isin(alternative, (cv2.GC_PR_BGD, cv2.GC_PR_FGD))] = cv2.GC_PR_BGD
    outputs = []
    component_evidence = []
    for labels in (initial, alternative):
        # Share the production lock because OpenCV's RNG is process-global.
        # Each hypothesis has fresh labels/models and the same fixed RNG seed.
        with _grabcut_lock:
            cv2.setRNGSeed(23)
            cv2.grabCut(np.ascontiguousarray(color), labels, None,
                        np.zeros((1, 65)), np.zeros((1, 65)), 3, cv2.GC_INIT_WITH_MASK)
        foreground = np.isin(labels, (cv2.GC_FGD, cv2.GC_PR_FGD))
        component_count, components = cv2.connectedComponents(
            foreground.astype(np.uint8), connectivity=8)
        # Mirror the original current-crop ownership rule for comparability;
        # an inconclusive component does not become a selected smaller mask.
        matching = [index for index in range(1, component_count)
                    if np.count_nonzero((components == index) & core)
                    >= .85*np.count_nonzero(core)]
        accepted = len(matching) == 1
        outputs.append(components == matching[0] if accepted else foreground)
        component_evidence.append(dict(
            foreground_components=int(component_count-1),
            matching_core_components=len(matching),
            core_component_unambiguous=accepted))
    original, counterfactual = outputs
    difference = original ^ counterfactual
    count = int(np.count_nonzero(difference))
    union_count = int(np.count_nonzero(original | counterfactual))
    hard_touched = int(np.count_nonzero(difference & (core | background)))
    components_resolved = all(e['core_component_unambiguous'] for e in component_evidence)
    difference_y, difference_x = np.nonzero(difference)
    valid_y, valid_x = np.nonzero(difference & valid)
    z = depth[valid_y, valid_x].astype(float)
    ray_moments = dict(
        count=int(len(z)),
        mean_z_m=(None if not len(z) else float(np.mean(z))),
        mean_u_z_m_px=(None if not len(z) else float(np.mean(valid_x*z))),
        mean_v_z_m_px=(None if not len(z) else float(np.mean(valid_y*z))))
    source_arrays = dict(zip(_INPUT_NAMES, (color, depth, masks[0], masks[1], masks[2], heights)))
    input_evidence = {name: dict(
        sha256=hashlib.sha256(array.tobytes()).hexdigest(),
        shape=list(array.shape), dtype=str(array.dtype))
        for name, array in source_arrays.items()}
    return dict(
        diagnostic_only=True, executable_geometry=False, execution_authority=False,
        observation_authority=False, source_stamp_ns=stamps[0],
        source_stamps_ns={name: source_stamps_ns[name] for name in _INPUT_NAMES},
        input_evidence=input_evidence,
        hypotheses_evaluated=2, grabcut_iterations_per_hypothesis=3,
        preferred_hypothesis=None, physical_contour_certified=False,
        ambiguous=bool(count or not components_resolved),
        conclusion=('SILHOUETTE_HYPOTHESIS_INCONCLUSIVE' if not components_resolved else
                    'SILHOUETTE_INITIALIZATION_SENSITIVE' if count else
                    'NO_DISAGREEMENT_IN_TWO_HYPOTHESES'),
        original=dict(_mask_summary(original, core, background, valid), **component_evidence[0]),
        counterfactual=dict(_mask_summary(counterfactual, core, background, valid), **component_evidence[1]),
        counterfactual_initialization=prior_evidence,
        disagreement=dict(
            pixel_count=count, fraction_of_union=float(count/max(union_count, 1)),
            crop_centroid_uv=(None if not count else
                              [float(np.mean(difference_x)), float(np.mean(difference_y))]),
            valid_depth_ray_moments=ray_moments,
            valid_depth_pixels=int(np.count_nonzero(difference & valid)),
            missing_or_invalid_depth_pixels=int(np.count_nonzero(difference & ~valid)),
            original_only_pixels=int(np.count_nonzero(original & ~counterfactual)),
            counterfactual_only_pixels=int(np.count_nonzero(counterfactual & ~original)),
            hard_foreground_pixels=int(np.count_nonzero(difference & core)),
            hard_background_pixels=int(np.count_nonzero(difference & background)),
            hard_labels_touched=bool(hard_touched)),
        interpretation='Disagreement is ownership uncertainty, not permission to choose a smaller mask. '
                       'No disagreement cannot rule out a shared error in both hypotheses.')
