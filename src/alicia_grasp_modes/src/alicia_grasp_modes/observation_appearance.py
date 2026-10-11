"""Bound observation-only RGB corroboration; no geometry or motion authority.

Only the executing receipt may supply the immutable measured reference. The
normal histogram/shape decision stays first and does not extract texture.
"""
import hashlib
import math
import time
from dataclasses import asdict, is_dataclass

import cv2
import numpy as np

from .rgb_observation_handoff import HandoffPolicy, _appearance, _mask


def _immutable(value):
    array = np.ascontiguousarray(value)
    return np.frombuffer(array.tobytes(), dtype=array.dtype).reshape(array.shape)


class FrozenObservationAppearance:
    """Cheap source snapshot with a lazy, once-per-reference feature cache."""
    def __setattr__(self, name, value):
        if name in self.__dict__:
            raise AttributeError('observation appearance reference is immutable')
        object.__setattr__(self, name, value)

    def __init__(self, bgr, mask, rgb_valid, source_ns, histogram, contour):
        image = np.asarray(bgr)
        mask = (_mask(mask, image.shape[:2]) > 0).astype(np.uint8)
        valid = np.ones(mask.shape, bool) if rgb_valid is None else np.asarray(rgb_valid)
        if (image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3
                or valid.shape != mask.shape or valid.dtype != np.bool_
                or np.any((mask > 0) & ~valid)
                or isinstance(source_ns, (bool, np.bool_))
                or not isinstance(source_ns, (int, np.integer)) or source_ns <= 0):
            raise ValueError('observation_appearance_reference_invalid')
        self.source_ns = int(source_ns)
        self.bgr, self.mask, self.valid = map(_immutable, (image, mask, valid))
        self.histogram, self.contour = map(_immutable, (histogram, contour))
        self.mask_sha256 = hashlib.sha256(self.mask.tobytes()).hexdigest()
        self._features = None

    def features(self, check):
        check()
        if self._features is None:
            from .arrival_dense_identity import freeze_reference
            reference = freeze_reference(self.bgr, self.mask, self.valid, self.source_ns)
            # Feature creation consumes the already fixed frame/phase budget.
            check()
            if (not np.array_equal(reference.histogram, self.histogram)
                    or not np.array_equal(reference.contour, self.contour)):
                raise ValueError('observation_appearance_reference_changed')
            object.__setattr__(self, '_features', reference)
        check()
        return self._features


def match_observation_appearance(rgb, mask, histogram, contour, *, rgb_valid=None,
                                 appearance_context=None):
    """Use dense evidence only after the existing color-only rejection.

    The context is call-local and carries the caller's fixed absolute deadline
    and current owner/source recheck. Neither is stored in the feature cache.
    """
    policy = HandoffPolicy()
    instance = _mask(mask, rgb.shape[:2])
    current_histogram, current_contour = _appearance(rgb, instance)
    colour = float(np.sqrt(max(0., 1.-np.sum(np.sqrt(current_histogram*histogram)))))
    shape = float(cv2.matchShapes(contour, current_contour, cv2.CONTOURS_MATCH_I1, 0.))
    chroma = float(np.sqrt(max(0., 1.-np.sum(np.sqrt(
        current_histogram.sum(axis=0)*np.asarray(histogram).sum(axis=0))))))
    context = {} if appearance_context is None else appearance_context
    reference = context.get('reference')
    audit = dict(appearance_match=False, fallback_attempted=False,
        colour_distance=colour, contour_distance=shape, chroma_distance=chroma,
        max_colour_distance=policy.max_colour_distance, max_contour_distance=policy.max_contour_distance,
        current_source_stamp_ns=context.get('source_ns'),
        reference_source_stamp_ns=getattr(reference, 'source_ns', None),
        reference_mask_sha256=getattr(reference, 'mask_sha256', None),
        current_mask_sha256=hashlib.sha256(np.ascontiguousarray(instance).tobytes()).hexdigest(),
        metric_geometry_authority=False, motion_authorized=False, reference_refreshed=False)
    def finish(ok, code):
        audit.update(appearance_match=bool(ok), code=code)
        record = context.get('record')
        if record is not None:
            record(dict(audit))
        return audit
    if (not all(math.isfinite(v) for v in (colour, shape, chroma))
            or not np.all(np.isfinite(current_histogram)) or not np.all(np.isfinite(histogram))):
        return finish(False, 'NONFINITE_APPEARANCE')
    if shape > policy.max_contour_distance:
        return finish(False, 'ORIGINAL_SHAPE_MISMATCH')
    if colour <= policy.max_colour_distance:
        return finish(True, 'ORIGINAL_HISTOGRAM_PASS')
    if not isinstance(reference, FrozenObservationAppearance):
        return finish(False, 'BOUND_OBSERVATION_REFERENCE_UNAVAILABLE')
    if chroma > policy.max_colour_distance:
        return finish(False, 'CHROMA_MISMATCH')
    if (not np.array_equal(histogram, reference.histogram)
            or not np.array_equal(contour, reference.contour)):
        return finish(False, 'REFERENCE_APPEARANCE_BINDING_MISMATCH')
    deadline = context.get('deadline_monotonic')
    check_current = context.get('check_current')
    clock = context.get('clock', time.monotonic)
    if (isinstance(deadline, bool) or not isinstance(deadline, (int, float))
            or not math.isfinite(deadline) or not callable(check_current)):
        return finish(False, 'OBSERVATION_APPEARANCE_BUDGET_REQUIRED')
    def check():
        # Preserve the owner's typed source/identity expiry so the existing
        # station-only discard/reacquisition contract can classify it. The
        # appearance layer must not relabel a true stale source as RGB loss.
        check_current()
        if clock() >= deadline:
            raise ValueError('observation_appearance_deadline_exceeded')
    audit['fallback_attempted'] = True
    started = clock()
    try:
        frozen = reference.features(check)
        audit['reference_digest'] = frozen.digest
        from .arrival_orb_proposal import propose_orb_homography
        from .arrival_dense_identity import evaluate_appearance
        valid = np.ones(instance.shape, bool) if rgb_valid is None else rgb_valid
        proposal, proposal_evidence = propose_orb_homography(frozen, rgb, instance, valid,
            context.get('source_ns'), deadline_monotonic=deadline, clock=clock)
        check()
        audit['proposal'] = asdict(proposal_evidence) if is_dataclass(proposal_evidence) else proposal_evidence
        evidence = evaluate_appearance(frozen, rgb, instance, valid,
            context.get('source_ns'), proposal, deadline_monotonic=deadline, clock=clock, policy=policy)
        check()
        # Per-cell residuals remain in offline replay reports, never in the
        # per-frame ROS diagnostic repeated for each fragment/consensus.
        audit['dense_metrics'] = {key:value for key,value in evidence.metrics.items()
            if value is None or isinstance(value, (bool, int, float, str))}
        audit['elapsed_sec'] = clock()-started
        return finish(evidence.appearance_match, evidence.code)
    except ValueError as exc:
        audit['elapsed_sec'] = clock()-started
        finish(False, str(exc))
        raise
