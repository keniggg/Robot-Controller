"""Bounded RGB identity evidence for ambiguous measured components.

This does not select, merge or authorize metric geometry. Every original metric
match must complete the unchanged RGB refinement. A single compatible complete
outline and explicit non-ownership for every alternative can justify withholding
measurement briefly; only a later unique metric frame may restore measurement.
"""
import cv2
import numpy as np

from alicia_grasp_modes.tabletop import Candidate
from alicia_grasp_modes.rgb_observation_handoff import HandoffPolicy, _appearance, _mask


def metric_seeded_identity_evidence(rgb, candidates, reference_rgb, reference_mask,
                                    *, refine, minimum_quality, rgb_valid=None):
    policy = HandoffPolicy()
    if not isinstance(candidates, (list, tuple)) or not 1 <= len(candidates) <= policy.maximum_candidates:
        raise ValueError('metric_ambiguity_candidate_budget')
    if (rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3
            or reference_rgb.shape != rgb.shape or reference_rgb.dtype != np.uint8):
        raise ValueError('metric_ambiguity_rgb_shape_invalid')
    shape = rgb.shape[:2]
    reference = _mask(reference_mask, shape) != 0
    if rgb_valid is not None:
        if rgb_valid.shape != shape or np.any(reference & ~rgb_valid):
            raise ValueError('metric_ambiguity_rgb_domain_invalid')
    histogram, contour = _appearance(reference_rgb, reference.astype(np.uint8))
    refined, rows, native_masks, masks = [], [], [], []
    occupied = np.zeros(shape, dtype=bool)
    for index, candidate in enumerate(candidates):
        if (not isinstance(candidate, Candidate) or not np.isfinite(candidate.quality)
                or not minimum_quality <= candidate.quality <= 1.):
            raise ValueError('metric_ambiguity_candidate_quality_invalid')
        native = _mask(candidate.mask, shape) != 0
        if np.any(occupied & native):
            raise ValueError('metric_ambiguity_native_components_overlap')
        occupied |= native
        current = refine(candidate)
        if (not isinstance(current, Candidate) or not np.isfinite(current.quality)
                or not minimum_quality <= current.quality <= 1.):
            raise ValueError('metric_ambiguity_refinement_invalid')
        mask = _mask(current.mask, shape) != 0
        if rgb_valid is not None and np.any(mask & ~rgb_valid):
            raise ValueError('metric_ambiguity_rgb_domain_invalid')
        hist, outline = _appearance(rgb, mask.astype(np.uint8))
        colour = float(np.sqrt(max(0., 1.-np.sum(np.sqrt(hist*histogram)))))
        contour_distance = float(cv2.matchShapes(contour, outline, cv2.CONTOURS_MATCH_I1, 0.))
        overlap = int(np.count_nonzero(reference & mask))
        iou = float(overlap / np.count_nonzero(reference | mask))
        compatible = (iou >= .85 and colour <= policy.max_colour_distance
                      and contour_distance <= policy.max_contour_distance)
        rows.append(dict(candidate_index=index, mask_iou=iou, colour_distance=colour,
            contour_distance=contour_distance, native_reference_overlap=int(np.count_nonzero(native & reference)),
            refined_reference_overlap=overlap, identity_compatible=bool(compatible)))
        refined.append(current); native_masks.append(native); masks.append(mask)
    winners = [i for i, row in enumerate(rows) if row['identity_compatible']]
    if len(winners) != 1:
        raise ValueError('metric_ambiguity_no_unique_complete_rgb_identity')
    winner = winners[0]
    for index, row in enumerate(rows):
        if index == winner:
            continue
        explicit_nonownership = (row['native_reference_overlap'] == 0
            and row['refined_reference_overlap'] == 0
            and not np.any(native_masks[index] & masks[winner])
            and not np.any(masks[index] & masks[winner])
            and row['colour_distance'] > policy.max_colour_distance)
        row['explicit_nonownership'] = bool(explicit_nonownership)
        if not explicit_nonownership:
            raise ValueError('metric_ambiguity_alternative_ownership_unresolved')
    return refined, dict(evidence_type='metric_seeded_full_outline_rgb_identity',
        candidate_count=len(candidates), candidates=rows, full_outline_iou_minimum=.85,
        max_colour_distance=policy.max_colour_distance,
        max_contour_distance=policy.max_contour_distance,
        all_metric_candidates_refined=True, metric_geometry_authority=False,
        motion_authorized=False, candidate_selected=False, geometry_merged=False,
        independent_mser_contract_claimed=False)
