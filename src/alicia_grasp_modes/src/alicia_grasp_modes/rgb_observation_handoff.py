"""Conservative RGB-interest association across measured camera poses.

No frame in this module is aligned, warped into a new metric frame, or given
synthetic depth. The original independent support fit remains the reference;
its raw disagreement is reported before appearance association. A colour patch
can still be a table marking: RGB association only preserves observation
interest. Only the existing InitialAcquisition can accept measured geometry.

The caller supplies exact-source TF and accepted feedback records. Matching
their timestamps here does not establish their authenticity; the ROS adapter
must obtain them from the existing exact-TF / accepted-feedback path. source_id
must hash the frozen camera, hand-eye, robot-model, epoch and selection context,
excluding the changing pose and image timestamp. This module performs no I/O.
"""
from dataclasses import dataclass, field
import hashlib

import cv2
import numpy as np

from alicia_grasp_modes.acquisition import InitialAcquisition
from alicia_grasp_modes.rgb_observation_region import RgbObservationRegion, RgbObservationRegions
from alicia_grasp_modes.rgb_observation_source import bracket_feedback, require_fresh_source
from alicia_grasp_modes.rgb_observation_volume import (
    RGBObservationVolume, SupportPlaneProvenance, project_rgb_observation_volume,
    _intrinsics, _transform)
from alicia_grasp_modes.tabletop import (
    Candidate, Config, _measured_support_footprint, _plane_axes,
    _validate_support_plane_update, _support_offset_diagnostics)


def _array(value, shape=None, dtype=float):
    array = np.array(value, dtype=dtype, copy=True)
    if (shape is not None and array.shape != shape) or not np.all(np.isfinite(array)):
        raise ValueError('handoff_invalid_array')
    array.setflags(write=False)
    return array


def _fingerprint(value):
    array = np.ascontiguousarray(value)
    return hashlib.sha256(str((array.shape, array.dtype)).encode() + array.tobytes()).hexdigest()


def _mask(value, shape):
    raw = np.asarray(value)
    if raw.shape != shape or not np.all(np.isfinite(raw)):
        raise ValueError('handoff_mask_source_shape_invalid')
    result = np.asarray(raw != 0, dtype=np.uint8)
    if np.count_nonzero(result) < 12:
        raise ValueError('handoff_region_too_small')
    if np.any(result[[0, -1]]) or np.any(result[:, [0, -1]]):
        raise ValueError('handoff_region_touches_image_boundary')
    count, _ = cv2.connectedComponents(result, connectivity=8)
    if count != 2:
        raise ValueError('handoff_region_fragmented')
    return result


@dataclass(frozen=True)
class RgbHandoffFrame:
    """Frozen one-frame evidence; depth is measured metres with holes unchanged."""
    bgr: np.ndarray = field(repr=False, compare=False)
    depth_m: np.ndarray = field(repr=False, compare=False)
    camera_matrix: np.ndarray = field(repr=False, compare=False)
    T_base_camera: np.ndarray = field(repr=False, compare=False)
    support_provenance: SupportPlaneProvenance
    source_id: str
    generation: int
    epoch_ns: int
    selection_stamp_ns: int
    source_stamp_ns: int
    tf_stamp_ns: int
    now_ns: int
    feedback_records: tuple = field(repr=False, compare=False)
    support_ring_outer_radius_px: object = None
    joints_rad: tuple = field(init=False)
    feedback_bracket_ns: tuple = field(init=False)
    image_fingerprint: str = field(init=False)
    depth_fingerprint: str = field(init=False)

    def __post_init__(self):
        if (type(self.source_id) is not str or not self.source_id.strip()
                or type(self.generation) is not int or self.generation < 0
                or type(self.epoch_ns) is not int or type(self.selection_stamp_ns) is not int
                or not 0 < self.epoch_ns < self.source_stamp_ns
                or not 0 < self.selection_stamp_ns < self.source_stamp_ns
                or type(self.tf_stamp_ns) is not int or self.tf_stamp_ns != self.source_stamp_ns):
            raise ValueError('handoff_source_binding_invalid')
        require_fresh_source(self.source_stamp_ns, self.now_ns)
        if (type(self.support_provenance) is not SupportPlaneProvenance
                or self.support_provenance.source_stamp_ns != self.source_stamp_ns):
            raise ValueError('handoff_support_source_mismatch')
        bgr, depth = np.asarray(self.bgr), np.asarray(self.depth_m)
        if (bgr.dtype != np.uint8 or bgr.ndim != 3 or bgr.shape[2] != 3
                or bgr.shape[0]*bgr.shape[1] > 1048576 or min(bgr.shape[:2]) < 12
                or depth.shape != bgr.shape[:2] or depth.dtype.kind != 'f'
                or np.any(np.isinf(depth)) or np.any(depth[np.isfinite(depth)] < 0)):
            raise ValueError('handoff_rgbd_source_invalid')
        if self.support_ring_outer_radius_px is not None:
            _support_ring_radius(self.support_ring_outer_radius_px, bgr.shape[:2])
        # Invalid depth stays invalid; NaN is an allowed sensor hole.
        frozen_depth = np.array(depth, dtype=np.float32, copy=True)
        frozen_depth.setflags(write=False)
        K = _array(_intrinsics(self.camera_matrix))
        T = _array(_transform(self.T_base_camera))
        records = tuple((int(ns), tuple(float(v) for v in q))
                        for ns, q in self.feedback_records)
        if len(records) > 128:
            raise ValueError('handoff_feedback_budget_exceeded')
        q, lo, hi = bracket_feedback(self.feedback_records, self.source_stamp_ns, self.epoch_ns)
        for name, value in (('bgr', _array(bgr, dtype=np.uint8)), ('depth_m', frozen_depth),
                            ('camera_matrix', K), ('T_base_camera', T),
                            ('feedback_records', records), ('joints_rad', tuple(q)),
                            ('feedback_bracket_ns', (lo[0], hi[0])),
                            ('image_fingerprint', _fingerprint(bgr)),
                            ('depth_fingerprint', _fingerprint(frozen_depth))):
            object.__setattr__(self, name, value)


@dataclass(frozen=True)
class HandoffPolicy:
    """Generic conservative association limits, never object-specific priors."""
    min_projected_fraction: float = .80
    max_colour_distance: float = .40
    max_contour_distance: float = .40
    ambiguity_margin: float = .12
    minimum_measured_mask_iou: float = .50
    maximum_candidates: int = 16
    maximum_observations: int = 3

    def __post_init__(self):
        values = (self.min_projected_fraction, self.max_colour_distance,
                  self.max_contour_distance, self.ambiguity_margin,
                  self.minimum_measured_mask_iou)
        if (any(isinstance(v, (bool, np.bool_)) for v in values)
                or not np.all(np.isfinite(values))
                or not .5 <= values[0] <= 1 or not 0 < values[1] <= .5
                or not 0 < values[2] <= 1 or not 0 <= values[3] < 1
                or not .5 <= values[4] <= 1
                or type(self.maximum_candidates) is not int
                or not 2 <= self.maximum_candidates <= 16
                or type(self.maximum_observations) is not int
                or not 1 <= self.maximum_observations <= 3):
            raise ValueError('handoff_policy_invalid')


def _appearance(bgr, mask):
    """Actual masked colour histogram and contour, not depth-hole appearance."""
    x, y, w, h = cv2.boundingRect(mask)
    crop = np.ascontiguousarray(bgr[y:y+h, x:x+w])
    local = np.ascontiguousarray(mask[y:y+h, x:x+w])
    lab = cv2.cvtColor(crop, cv2.COLOR_BGR2LAB)
    hist = cv2.calcHist([lab], [0, 1, 2], local, [8, 8, 8], [0, 256]*3).astype(float)
    # Mild bin smoothing tolerates quantization boundaries; preserve colour
    # contrasts (including light/dark) instead of normalizing them away.
    for axis in range(3):
        padding = [(0, 0)]*3; padding[axis] = (1, 1)
        expanded = np.pad(hist, padding, mode='edge')
        parts = []
        for offset in range(3):
            slices = [slice(None)]*3; slices[axis] = slice(offset, offset+8)
            parts.append(expanded[tuple(slices)])
        hist = .125*parts[0] + .75*parts[1] + .125*parts[2]
    hist /= np.sum(hist)
    contours = cv2.findContours(local, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) <= 0:
        raise ValueError('handoff_degenerate_contour')
    return _array(hist), _array(contour, dtype=np.int32)


def _support_ring_radius(outer, shape):
    if (isinstance(outer, (bool, np.bool_)) or not isinstance(outer, (int, float, np.number))
            or not np.isrealobj(outer) or not np.isfinite(outer)
            or not 70*min(shape[1]/640., shape[0]/480.) < outer
            <= np.hypot(shape[1]/2., shape[0]/2.)):
        raise ValueError('handoff_invalid_support_ring')
    return float(outer)


def _support_footprint(frame, outer):
    outer = _support_ring_radius(outer, frame.depth_m.shape)
    depth, K, T = frame.depth_m, frame.camera_matrix, frame.T_base_camera
    h, w = depth.shape
    yy, xx = np.ogrid[:h, :w]
    r = np.hypot(xx-w/2., yy-h/2.)
    # Match existing independent support ring, retaining actual inlier pixels.
    plane = np.asarray(frame.support_provenance.support_plane_base)
    equation = T.T @ plane
    optical_y = (yy-K[1, 2])/K[1, 1]
    optical_x = (xx-K[0, 2]-K[0, 1]*optical_y)/K[0, 0]
    distance = depth*(equation[0]-optical_x*equation[1]-optical_y*equation[2])+equation[3]
    with np.errstate(invalid='ignore'):
        ring = (np.isfinite(depth) & (depth > 0) & (r >= 70*min(w/640., h/480.))
                & (r <= outer) & (np.abs(distance) <= .002))
    quarters = sum(np.count_nonzero(ring & (xx < w/2. if left else xx >= w/2.)
                     & (yy < h/2. if top else yy >= h/2.)) >= 20
                   for left in (True, False) for top in (True, False))
    v, u = np.nonzero(ring)
    if len(u) < 120 or quarters < 3:
        raise ValueError('handoff_actual_support_coverage_insufficient')
    # A sparse deterministic subset is sufficient for an audit footprint, and
    # bounds CPU independently of camera resolution. It is not object geometry.
    step = max(1, int(np.ceil(len(u)/1800)))
    u, v = u[::step], v[::step]
    d = depth[v, u]
    oy = (v-K[1, 2])/K[1, 1]; ox = (u-K[0, 2]-K[0, 1]*oy)/K[0, 0]
    camera = np.c_[d, -ox*d, -oy*d]
    points = camera @ T[:3, :3].T + T[:3, 3]
    return _array(_measured_support_footprint(points, plane))


def _consistency_domain(volume):
    """All original uncertainty vertices projected onto the original plane.

    This constructed domain specifies where two independent support fits must
    agree. It is not measured support, a target surface, or collision geometry.
    """
    vertices = _array(volume.vertices_base_m, (8, 3))
    plane = np.asarray(volume.support_plane_base)
    return _array(vertices-(vertices @ plane[:3]+plane[3])[:, None]*plane[:3])


def _covered_domain_vertices(domain, measured_footprint, reference_plane):
    # Compare spatial coverage in the fixed original plane coordinates. Current
    # inliers retain their measured fit; no plane is aligned to the reference.
    # Convex-hull coverage permits holes and never asserts vertex measurements.
    axes = np.asarray(_plane_axes(np.asarray(reference_plane)[:3]))
    hull = cv2.convexHull((np.asarray(measured_footprint) @ axes.T).astype(np.float32))
    if len(hull) < 3 or cv2.contourArea(hull) <= 0:
        raise ValueError('handoff_actual_support_coverage_insufficient')
    return sum(cv2.pointPolygonTest(hull, tuple(map(float, p)), True) >= -1e-7
               for p in np.asarray(domain) @ axes.T)


@dataclass(frozen=True)
class RgbInterestReference:
    volume: RGBObservationVolume
    source_id: str
    generation: int
    epoch_ns: int
    selection_stamp_ns: int
    source_stamp_ns: int
    mask_fingerprint: str
    source_image_fingerprint: str
    source_depth_fingerprint: str
    histogram: np.ndarray = field(repr=False, compare=False)
    contour: np.ndarray = field(repr=False, compare=False)
    measured_support_footprint_base: np.ndarray = field(repr=False, compare=False)
    support_ring_outer_radius_px: float
    support_consistency_domain_base: np.ndarray = field(init=False, repr=False, compare=False)
    observation_only: bool = field(default=True, init=False)
    metric_geometry_authority: bool = field(default=False, init=False)
    motion_authorized: bool = field(default=False, init=False)

    def __post_init__(self):
        plane = self.volume.support_plane_base
        footprint = _array(self.measured_support_footprint_base)
        _validate_support_plane_update(plane, plane, footprint.mean(axis=0), Config(), footprint)
        domain = _consistency_domain(self.volume)
        _validate_support_plane_update(plane, plane, domain.mean(axis=0), Config(), domain)
        if _covered_domain_vertices(domain, footprint, plane) != len(domain):
            raise ValueError('handoff_original_support_domain_not_covered')
        object.__setattr__(self, 'measured_support_footprint_base', footprint)
        object.__setattr__(self, 'support_consistency_domain_base', domain)
        object.__setattr__(self, 'support_ring_outer_radius_px',
            _support_ring_radius(self.support_ring_outer_radius_px, self.volume.image_shape))


def freeze_rgb_interest(frame, region, volume, *, support_ring_outer_radius_px):
    """Retain original RGB evidence plus a footprint made from measured support."""
    if type(frame) is not RgbHandoffFrame or type(region) is not RgbObservationRegion:
        raise ValueError('handoff_reference_type_invalid')
    if (type(volume) is not RGBObservationVolume
            or volume.source_stamp_ns != frame.source_stamp_ns
            or volume.image_shape != frame.bgr.shape[:2]
            or tuple(volume.bbox_xywh) != tuple(region.bbox_xywh)
            or volume.support_provenance != frame.support_provenance
            or not np.array_equal(volume.camera_matrix, frame.camera_matrix)
            or not np.array_equal(volume.T_base_camera, frame.T_base_camera)):
        raise ValueError('handoff_reference_volume_source_mismatch')
    mask = _mask(region.mask, frame.bgr.shape[:2])
    if cv2.boundingRect(mask) != tuple(region.bbox_xywh):
        raise ValueError('handoff_reference_bbox_mask_mismatch')
    hist, contour = _appearance(frame.bgr, mask)
    if (frame.support_ring_outer_radius_px is not None
            and frame.support_ring_outer_radius_px != support_ring_outer_radius_px):
        raise ValueError('handoff_reference_support_ring_mismatch')
    footprint = _support_footprint(frame, support_ring_outer_radius_px)
    return RgbInterestReference(volume, frame.source_id, frame.generation,
        frame.epoch_ns, frame.selection_stamp_ns, frame.source_stamp_ns,
        _fingerprint(mask), frame.image_fingerprint, frame.depth_fingerprint,
        hist, contour, footprint, support_ring_outer_radius_px)


def _candidate_fingerprint(candidate):
    if not isinstance(candidate, Candidate):
        raise ValueError('handoff_measured_candidate_type_invalid')
    data = '|'.join(_fingerprint(value) for value in (
        candidate.mask, candidate.position_camera, candidate.position_base,
        candidate.extent_m, candidate.support_footprint_base, candidate.bbox,
        candidate.center_uv, candidate.depth_m, candidate.quality,
        candidate.metrics.get('support_plane_base'),
        candidate.metrics.get('valid_points', 0)))
    return hashlib.sha256(data.encode()).hexdigest()


@dataclass(frozen=True)
class RgbInterestAssociation:
    reason: str
    frame: RgbHandoffFrame = field(repr=False, compare=False)
    reference: RgbInterestReference = field(repr=False, compare=False)
    observation_index: int
    rgb_region_index: object = None
    measured_candidate_index: object = None
    region: object = field(default=None, repr=False, compare=False)
    measured_candidate: object = field(default=None, repr=False, compare=False)
    candidate_fingerprint: str = ''
    score: object = None
    alternatives: tuple = ()
    support_audit: tuple = ()
    projected_volume_pixels: tuple = ()
    observation_only: bool = field(default=True, init=False)
    metric_geometry_authority: bool = field(default=False, init=False)
    motion_authorized: bool = field(default=False, init=False)
    synthetic_measured_points_added: int = field(default=0, init=False)

    @property
    def associated(self):
        return self.rgb_region_index is not None


def _raw_support_audit(reference, frame):
    original = np.asarray(reference.volume.support_plane_base)
    current = np.asarray(frame.support_provenance.support_plane_base)
    if original[:3] @ current[:3] < 0:
        current = -current
    footprint = reference.measured_support_footprint_base
    domain = _consistency_domain(reference.volume)
    angle = float(np.rad2deg(np.arccos(np.clip(original[:3] @ current[:3], -1., 1.))))
    delta = current-original
    shift = float(np.max(np.abs(domain @ delta[:3]+delta[3])))
    ring_shift = float(np.max(np.abs(footprint @ delta[:3]+delta[3])))
    audit = dict(support_reference_angle_deg=angle, support_reference_max_distance_m=shift,
        support_reference_source_stamp_ns=reference.source_stamp_ns,
        current_support_source_stamp_ns=frame.source_stamp_ns,
        support_reference_fixed=True, support_alignment_applied=False,
        support_original_ring_max_distance_m=ring_shift,
        support_original_ring_outer_radius_px=reference.support_ring_outer_radius_px,
        current_support_ring_outer_radius_px=frame.support_ring_outer_radius_px,
        support_consistency_domain_vertices_base_m=tuple(map(tuple, domain)),
        support_consistency_domain_derived=True,
        support_coverage_is_vertex_measurement=False)
    audit.update(_support_offset_diagnostics(shift, Config()))
    try:
        _validate_support_plane_update(original, current, footprint.mean(axis=0), Config(), footprint)
        audit['support_original_ring_rejection'] = ''
    except ValueError as exc:
        audit['support_original_ring_rejection'] = str(exc)
    audit['support_original_ring_consistent'] = not audit['support_original_ring_rejection']
    try:
        if not np.array_equal(domain, reference.support_consistency_domain_base):
            raise ValueError('handoff_fixed_support_domain_changed')
        _validate_support_plane_update(original, current, domain.mean(axis=0), Config(), domain)
        original_covered = _covered_domain_vertices(domain, footprint, original)
        audit['support_original_domain_covered_vertices'] = original_covered
        if original_covered != len(domain):
            raise ValueError('handoff_original_support_domain_not_covered')
        if frame.support_ring_outer_radius_px is None:
            raise ValueError('handoff_current_support_ring_required')
        current_footprint = _support_footprint(frame, frame.support_ring_outer_radius_px)
        audit['current_measured_support_footprint_base'] = tuple(map(tuple, current_footprint))
        covered = _covered_domain_vertices(domain, current_footprint, original)
        audit['support_current_domain_covered_vertices'] = covered
        if covered != len(domain):
            raise ValueError('handoff_current_support_domain_not_covered')
    except ValueError as exc:
        return audit, str(exc)
    return audit, ''


def associate_rgb_interest(reference, frame, rgb_regions, measured_candidates=(), *,
                           observation_index, policy=HandoffPolicy()):
    """Associate to the original interest, ignoring current image-centre rank.

    Pass *all* RGB alternatives, even if the per-frame centre selector was
    ambiguous. A unique cross-view appearance/volume match can resolve that
    ambiguity. An ambiguity here cannot be resolved by nearest centre or by a
    depth candidate. observation_index is a caller-maintained 1..3 view budget;
    every view compares to this same original reference, never the preceding
    best patch. Association alone does not confirm a measured target.
    """
    if type(reference) is not RgbInterestReference or type(frame) is not RgbHandoffFrame:
        raise ValueError('handoff_association_type_invalid')
    audit = {}
    projected = ()
    def report(reason, **kwargs):
        return RgbInterestAssociation(reason, frame, reference, observation_index,
            support_audit=tuple(audit.items()), projected_volume_pixels=projected, **kwargs)
    if type(observation_index) is not int or not 1 <= observation_index <= policy.maximum_observations:
        return report('observation_budget_exhausted')
    if (frame.source_id != reference.source_id or frame.generation != reference.generation
            or frame.epoch_ns != reference.epoch_ns
            or frame.selection_stamp_ns != reference.selection_stamp_ns
            or frame.source_stamp_ns <= reference.source_stamp_ns
            or not np.array_equal(frame.camera_matrix, reference.volume.camera_matrix)
            or frame.bgr.shape[:2] != reference.volume.image_shape):
        return report('source_context_changed')
    audit, support_error = _raw_support_audit(reference, frame)
    if support_error:
        return report('support_inconsistent: '+support_error)
    if isinstance(rgb_regions, RgbObservationRegions):
        if rgb_regions.image_shape != frame.bgr.shape[:2]:
            return report('rgb_region_source_shape_mismatch')
        if 'budget' in rgb_regions.reason:
            return report('rgb_candidate_budget_exhausted')
        rgb_regions = rgb_regions.candidates
    rgb_regions, measured_candidates = tuple(rgb_regions), tuple(measured_candidates)
    if max(len(rgb_regions), len(measured_candidates)) > policy.maximum_candidates:
        return report('association_candidate_budget_exhausted')
    try:
        projected = project_rgb_observation_volume(reference.volume,
            T_base_camera=frame.T_base_camera, camera_matrix=frame.camera_matrix,
            image_shape=frame.bgr.shape[:2], working_depth_range_m=reference.volume.working_depth_range_m)
    except ValueError as exc:
        return report('projection_rejected: '+str(exc))
    projected_mask = np.zeros(frame.bgr.shape[:2], np.uint8)
    hull = cv2.convexHull(np.asarray(projected, dtype=np.float32)).reshape(-1, 2)
    cv2.fillConvexPoly(projected_mask, np.rint(hull).astype(np.int32), 1)
    matches = []
    for index, region in enumerate(rgb_regions):
        if type(region) is not RgbObservationRegion:
            return report('rgb_region_type_invalid')
        try:
            mask = _mask(region.mask, frame.bgr.shape[:2])
            fraction = np.count_nonzero(mask & projected_mask)/np.count_nonzero(mask)
            if fraction < policy.min_projected_fraction:
                continue
            hist, contour = _appearance(frame.bgr, mask)
            colour = float(np.sqrt(max(0., 1.-np.sum(np.sqrt(hist*reference.histogram)))))
            shape = float(cv2.matchShapes(reference.contour, contour, cv2.CONTOURS_MATCH_I1, 0.))
        except (ValueError, cv2.error):
            continue
        if colour <= policy.max_colour_distance and shape <= policy.max_contour_distance:
            score = .65*colour/policy.max_colour_distance + .35*shape/policy.max_contour_distance
            matches.append((score, index, colour, shape, fraction))
    matches.sort()
    alternatives = tuple(dict(index=i, score=s, colour_distance=c, contour_distance=h,
                              projected_fraction=f) for s, i, c, h, f in matches)
    if not matches:
        return report('no_cross_view_rgb_match', alternatives=alternatives)
    if len(matches) > 1 and matches[1][0]-matches[0][0] <= policy.ambiguity_margin:
        return report('cross_view_rgb_ambiguous', alternatives=alternatives)
    score, index = matches[0][:2]
    region = rgb_regions[index]
    rgbmask = _mask(region.mask, frame.bgr.shape[:2]) != 0
    measured_matches = []
    for ci, candidate in enumerate(measured_candidates):
        if not isinstance(candidate, Candidate):
            return report('measured_candidate_type_invalid')
        try:
            candidate_mask = _mask(candidate.mask, rgbmask.shape) != 0
            iou = np.count_nonzero(candidate_mask & rgbmask)/np.count_nonzero(candidate_mask | rgbmask)
            if iou >= policy.minimum_measured_mask_iou:
                if not np.allclose(candidate.metrics.get('support_plane_base'),
                    frame.support_provenance.support_plane_base, atol=1e-8, rtol=0.):
                    return report('measured_candidate_support_source_mismatch')
                measured_matches.append(ci)
        except (TypeError, ValueError):
            continue
    if len(measured_matches) > 1:
        return report('measured_candidate_ambiguous', alternatives=alternatives)
    ci = measured_matches[0] if measured_matches else None
    measured = measured_candidates[ci] if ci is not None else None
    return report('measured_candidate_associated' if measured is not None else 'rgb_only_associated',
        rgb_region_index=index, measured_candidate_index=ci, region=region,
        measured_candidate=measured,
        candidate_fingerprint=_candidate_fingerprint(measured) if measured is not None else '',
        score=float(score), alternatives=alternatives)


@dataclass(frozen=True)
class MeasuredHandoffResult:
    ready: bool
    reason: str
    candidate: object = field(default=None, repr=False, compare=False)
    source_stamp_ns: object = None
    confirming_stamps_ns: tuple = ()
    motion_authorized: bool = field(default=False, init=False)
    synthetic_measured_points_added: int = field(default=0, init=False)


class RgbMeasuredHandoff:
    """Use the unchanged InitialAcquisition on the associated real candidates.

    raw must be the exact object returned by association; refined is the output
    of the caller's existing RGB-D refine_candidate on that same frame. This
    adapter neither runs refinement nor manufactures Candidate objects. The
    caller must retain the returned instance when committing TargetTracker; a
    subsequent centre-based choose would undo the association contract.
    """
    def __init__(self, config=None):
        self.acquisition = InitialAcquisition(config or Config())
        self.reset()

    def reset(self):
        self.acquisition.reset()
        self._context = None
        self._first_q = None
        self._last_stamp = 0

    def observe(self, association, raw=None, refined=None):
        def reject(reason):
            self.reset()
            return MeasuredHandoffResult(False, reason)
        if not isinstance(association, RgbInterestAssociation) or not association.associated:
            return reject('handoff_association_not_unique')
        frame = association.frame
        if (raw is None or raw is not association.measured_candidate
                or not isinstance(refined, Candidate)):
            return reject('handoff_requires_associated_measured_candidate')
        if _candidate_fingerprint(raw) != association.candidate_fingerprint:
            return reject('handoff_measured_candidate_mutated')
        context = (id(association.reference), frame.source_id, frame.generation, association.observation_index)
        if context != self._context:
            self.reset(); self._context = context
        if frame.source_stamp_ns <= self._last_stamp:
            return reject('handoff_nonmonotonic_source')
        if self._last_stamp and frame.source_stamp_ns-self._last_stamp > 1_000_000_000:
            self.reset(); self._context = context
        q = np.asarray(frame.joints_rad)
        if self._first_q is not None and np.max(np.abs(q-self._first_q)) > .004:
            self.reset(); self._context = context
        self._first_q = q.copy() if self._first_q is None else self._first_q
        try:
            measured = _mask(refined.mask, frame.depth_m.shape) != 0
            region = _mask(association.region.mask, frame.depth_m.shape) != 0
            if np.count_nonzero(measured & region)/np.count_nonzero(measured | region) < .50:
                return reject('handoff_refined_region_not_associated')
            with np.errstate(invalid='ignore'):
                count = int(np.count_nonzero(measured & np.isfinite(frame.depth_m) & (frame.depth_m > 0)))
            if count < self.acquisition.minimum_points:
                return reject('handoff_insufficient_actual_depth_pixels')
            if not np.allclose(refined.metrics.get('support_plane_base'),
                               frame.support_provenance.support_plane_base, atol=1e-8, rtol=0.):
                return reject('handoff_refined_support_source_mismatch')
            confirmed, reason = self.acquisition.observe(raw, refined, frame.source_stamp_ns)
        except (ValueError, TypeError, KeyError):
            return reject('handoff_measured_evidence_invalid')
        self._last_stamp = frame.source_stamp_ns
        # InitialAcquisition is intentionally unchanged; bound its holding
        # history here after confirmation for a long-running ROS process.
        if len(self.acquisition.samples) > 3:
            self.acquisition.samples = [self.acquisition.samples[0]] + self.acquisition.samples[-2:]
        stamps = tuple(item[2] for item in self.acquisition.samples)
        return MeasuredHandoffResult(bool(confirmed), 'measured_handoff_ready' if confirmed else reason,
            refined if confirmed else None, frame.source_stamp_ns, stamps)
