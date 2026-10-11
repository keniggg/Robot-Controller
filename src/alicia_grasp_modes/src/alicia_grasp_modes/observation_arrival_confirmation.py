"""Measured arrival confirmation; no target reset, ROS, or execution authority."""
from copy import deepcopy

import cv2
import numpy as np

from .acquisition import InitialAcquisition
from .rgb_instance import refine_candidate, freeze_observation_spatial_reference
from .rgb_observation_handoff import HandoffPolicy, _appearance, _mask
from .tabletop import TargetTracker, _matching_candidates
from .arrival_fragment_group import (recover_arrival_fragment_group,
                                     recover_arrival_matching_fragments)
from .observation_appearance import match_observation_appearance


class ObservationArrivalConfirmation:
    """Three current frames qualify against the unchanged pre-motion identity.

    Independent support fitting and reference consistency are supplied by the
    original segmentation call. This class refuses missing positive evidence;
    it cannot replace a failed fit with the transported reference plane.
    """
    def __init__(self, config, anchor, last, plane, histogram, contour, minimum_quality,
                 *, appearance_reference=None):
        self.config = config
        self.anchor = anchor
        self.last = last
        self.plane = np.array(plane, dtype=float, copy=True)
        self.spatial_reference = freeze_observation_spatial_reference(anchor, self.plane)
        self.histogram = np.array(histogram, copy=True)
        self.contour = np.array(contour, copy=True)
        self.minimum_quality = float(minimum_quality)
        self.confirmation = InitialAcquisition(config, frames=3)
        self.policy = HandoffPolicy()
        self.last_evidence = None
        self.appearance_reference = appearance_reference
        self.last_appearance_evidence = []

    def clear(self, *, preserve_appearance=False):
        self.confirmation.reset()
        self.last_evidence = None
        if not preserve_appearance:
            self.last_appearance_evidence = []

    def _record_appearance(self, evidence):
        # A frame may contain original fragments plus their consensus. These
        # diagnostics are never identity, geometry, or motion receipts.
        self.last_appearance_evidence.append(deepcopy(evidence))
        self.last_appearance_evidence = self.last_appearance_evidence[-20:]

    @property
    def source_stamps_ns(self):
        return [row[2] for row in self.confirmation.samples]

    def observe(self, result, rgb, depth, intrinsics, transform, source_ns, *,
                appearance_deadline_monotonic=None, appearance_check_current=None, **rgb_kwargs):
        self.last_appearance_evidence = []
        appearance_context = dict(reference=self.appearance_reference, source_ns=source_ns,
            deadline_monotonic=appearance_deadline_monotonic, check_current=appearance_check_current,
            record=self._record_appearance)
        try:
            metrics = result.metrics
            if metrics.get('support_current_audit_available') is not True:
                raise ValueError('arrival_independent_current_support_required')
            for key, limit in (
                    ('support_current_audit_residual_m', self.config.support_threshold_m),
                    ('support_reference_angle_deg', self.config.support_max_normal_change_deg),
                    ('support_reference_max_distance_m', self.config.support_max_local_shift_m)):
                value = metrics.get(key)
                if (isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float))
                        or not np.isfinite(value) or not 0. <= value <= limit + 1e-12):
                    raise ValueError('arrival_support_audit_invalid: ' + key)
            ratio = metrics.get('support_current_audit_inlier_ratio')
            if (isinstance(ratio, (bool, np.bool_)) or not isinstance(ratio, (int, float))
                    or not np.isfinite(ratio) or not self.config.support_min_ratio <= ratio <= 1.
                    or metrics.get('support_current_audit_quadrants') not in (3, 4)):
                raise ValueError('arrival_support_quality_invalid')
            matching = _matching_candidates(result.candidates, self.anchor, self.last, self.config)
            recovered = None
            if not matching:
                result, recovered = recover_arrival_fragment_group(result, rgb, depth, intrinsics,
                    transform, self.config, self.anchor, self.last, self.spatial_reference,
                    self.histogram, self.contour, self.minimum_quality,
                    appearance_context=appearance_context, **rgb_kwargs)
                matching = _matching_candidates(result.candidates, self.anchor, self.last, self.config)
            elif len(matching) > 1:
                # Competing measured fragments may only become one instance
                # after every original match independently agrees in current
                # RGB. The dedicated helper cannot choose or discard a rival.
                result, recovered = recover_arrival_matching_fragments(result, rgb, depth,
                    intrinsics, transform, self.config, self.anchor, self.last,
                    self.spatial_reference, self.histogram, self.contour,
                    self.minimum_quality, appearance_context=appearance_context, **rgb_kwargs)
                matching = _matching_candidates(result.candidates, self.anchor, self.last, self.config)
            if len(matching) != 1:
                raise ValueError('arrival_unique_original_identity_required')
            raw = matching[0][1]
            if not np.isfinite(raw.quality) or raw.quality < self.minimum_quality:
                raise ValueError('arrival_geometry_quality_insufficient')
            candidate = (recovered[1] if recovered is not None and raw is recovered[0] else
                         refine_candidate(rgb, depth, intrinsics, transform, raw, self.config,
                                         observation_arrival_retry=True,
                                         observation_spatial_reference=self.spatial_reference, **rgb_kwargs))
            if not np.isfinite(candidate.quality) or candidate.quality < self.minimum_quality:
                raise ValueError('arrival_refined_quality_insufficient')
            appearance = match_observation_appearance(rgb, candidate.mask, self.histogram, self.contour,
                rgb_valid=rgb_kwargs.get('rgb_valid'), appearance_context=appearance_context)
            colour, shape = appearance['colour_distance'], appearance['contour_distance']
            if not appearance['appearance_match']:
                raise ValueError('arrival_original_rgb_identity_not_matched')
            confirmed, reason = self.confirmation.observe(raw, candidate, source_ns)
            self.last_evidence = dict(qualified=bool(confirmed), reason=reason,
                source_stamps_ns=self.source_stamps_ns, colour_distance=colour, contour_distance=shape,
                measured_points=int(candidate.metrics.get('valid_points',0)),
                original_anchor_unchanged=True, original_plane_unchanged=True,
                independent_support=deepcopy(metrics), metric_geometry_authority=False,
                motion_authorized=False, appearance_identity=deepcopy(self.last_appearance_evidence))
            staged = TargetTracker(self.config)
            staged.anchor, staged.last = self.anchor, raw
            staged.plane_base = self.plane.copy()
            return bool(confirmed), candidate, staged, deepcopy(self.last_evidence)
        except Exception:
            self.clear(preserve_appearance=True)
            raise
