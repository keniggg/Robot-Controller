"""Bound observation-only RGB corroboration; no geometry or motion authority.

Only the executing receipt may supply the immutable measured reference. The
normal histogram/shape decision stays first and does not extract texture.
"""
import hashlib
import json
from copy import deepcopy
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

    def __init__(self, bgr, mask, rgb_valid, source_ns, histogram, contour, *, measured_face=None):
        original_mask = np.asarray(mask)
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
        self.rgb_sha256 = hashlib.sha256(self.bgr.tobytes()).hexdigest()
        self.mask_sha256 = hashlib.sha256(self.mask.tobytes()).hexdigest()
        if measured_face is not None:
            from .observation_measured_face import FrozenMeasuredFace
            if (not isinstance(measured_face, FrozenMeasuredFace)
                    or measured_face.source.source_ns != int(source_ns)
                    or measured_face.source.rgb_sha256 != self.rgb_sha256
                    or measured_face.source.whole_mask_sha256 != hashlib.sha256(
                        np.ascontiguousarray(original_mask).tobytes()).hexdigest()):
                raise ValueError('observation_measured_face_binding_invalid')
        self.measured_face = measured_face
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
    if 'same_station_mask_iou_minimum' in context:
        # This is appearance corroboration only. Every current depth/support,
        # native-core ownership and final metric gate still runs separately.
        # A fixed full outline prevents chained colour/shape drift at a station.
        check_current = context.get('check_current')
        if (context['same_station_mask_iou_minimum'] != .85
                or not isinstance(reference, FrozenObservationAppearance)
                or not callable(check_current)
                or context.get('reference_role') != 'qualified_station_identity_template'
                or type(context.get('source_ns')) is not int
                or not reference.source_ns < context['source_ns']
                or not np.array_equal(histogram, reference.histogram)
                or not np.array_equal(contour, reference.contour)):
            raise ValueError('observation_station_appearance_context_invalid')
        check_current()
        previous, current = reference.mask > 0, instance > 0
        iou = float(np.count_nonzero(previous & current)/np.count_nonzero(previous | current))
        audit.update(reference_role=context['reference_role'], same_station_mask_iou=iou,
            same_station_mask_iou_minimum=.85)
        if not iou >= .85:
            return finish(False, 'SAME_STATION_COMPLETE_OUTLINE_MISMATCH')
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
        # The premotion measured face is fixed before motion. This branch only
        # corroborates appearance; all full current geometry/native ownership
        # checks remain in the caller. It never labels the old whole mask valid.
        if reference.measured_face is not None and 'measured_face_frame' in context:
            from .observation_measured_face import corroborate_measured_face, MeasuredFaceUnavailable
            try:
                valid = np.ones(instance.shape, bool) if rgb_valid is None else rgb_valid
                frame = context['measured_face_frame']
                check()
                # Fragment corroboration and final confirmation can encounter
                # exactly the same measured frame/mask. Reuse only pure evidence
                # within this one observe call, with every input byte bound.
                def array_key(value):
                    array = np.ascontiguousarray(value)
                    return (array.dtype.str, array.shape,
                            hashlib.sha256(array.tobytes()).hexdigest())
                key = (id(reference.measured_face), reference.measured_face.source.binding_sha256,
                    reference.measured_face.reference.digest, context.get('source_ns'), deadline,
                    array_key(rgb), array_key(instance), array_key(valid), array_key(frame['depth']),
                    array_key(frame['intrinsics']), array_key(frame['transform']), array_key(frame['plane']),
                    json.dumps(asdict(frame['config']), sort_keys=True, allow_nan=False))
                cache = context.get('measured_face_cache')
                cached = None if cache is None else cache.get(key)
                check()
                if cached is None:
                    face_evidence, face_audit = corroborate_measured_face(reference.measured_face,
                        rgb, instance, valid, frame, source_ns=context.get('source_ns'),
                        deadline_monotonic=deadline, check=check, clock=clock)
                    check()
                    if cache is not None:
                        cache[key] = deepcopy((face_evidence, face_audit))
                    face_audit['same_frame_evidence_reused'] = False
                else:
                    face_evidence, face_audit = deepcopy(cached)
                    face_audit['same_frame_evidence_reused'] = True
                check()
                # Keep per-cell residuals in the offline pure-module evidence,
                # not repeated fragment/final ROS status payloads.
                face_audit['dense_metrics'] = {key: value for key, value in
                    face_audit.get('dense_metrics', {}).items()
                    if value is None or isinstance(value, (bool, int, float, str))}
                audit['common_measured_face'] = face_audit
                check()
                if face_evidence.appearance_match:
                    audit.update(elapsed_sec=clock()-started,
                        whole_object_histogram_pass=False, identity_scope='fixed_common_measured_face')
                    return finish(True, 'COMMON_MEASURED_FACE_DENSE_CORROBORATED')
            except MeasuredFaceUnavailable as exc:
                # Only absence of additional face evidence can use the old
                # path. Typed owner/source errors must reach the outer router.
                audit['common_measured_face'] = dict(appearance_match=False, code=str(exc))
                check()
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
