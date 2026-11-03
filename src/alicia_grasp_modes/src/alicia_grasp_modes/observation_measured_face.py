"""Fixed common measured-face corroboration; never whole-object geometry.

Each face is selected using only its own source RGB-D. Missing depth stays
unknown; GrabCut labels RGB pixels and never creates a measured surface. The
caller must independently establish full current geometry and native ownership.
"""
from dataclasses import asdict
import hashlib
import json
import math
import time

import cv2
import numpy as np

from .tabletop import Config, _project_pixels
from .measured_mask_seeds import local_surface_normals
from .rgb_instance import _grabcut_lock
from .arrival_dense_identity import freeze_reference, propose_homography, evaluate_appearance
from .arrival_train_only_proposal import (refine_train_only_proposal,
    TrainingProposalUnavailable, FrozenTrainingSource)


class MeasuredFaceUnavailable(ValueError):
    """No additional face proof; the unchanged whole-object path may still run."""


def _sha(value):
    return hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()


def _immutable(value):
    array = np.ascontiguousarray(value)
    return np.frombuffer(array.tobytes(), dtype=array.dtype).reshape(array.shape)


class _Frozen:
    def __setattr__(self, name, value):
        if name in self.__dict__:
            raise AttributeError('measured face source is immutable')
        object.__setattr__(self, name, value)


def _geometry(intrinsics, transform, plane):
    k, t, p = (np.asarray(value, dtype=float) for value in (intrinsics, transform, plane))
    if (k.shape != (4,) or not np.all(np.isfinite(k)) or np.any(k[:2] <= 0)
            or t.shape != (4, 4) or not np.all(np.isfinite(t))
            or not np.allclose(t[3], [0., 0., 0., 1.], rtol=0., atol=1e-9)
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), rtol=0., atol=1e-6)
            or not np.isclose(np.linalg.det(t[:3, :3]), 1., rtol=0., atol=1e-6)
            or p.shape != (4,) or not np.all(np.isfinite(p))
            or not np.isclose(np.linalg.norm(p[:3]), 1., rtol=0., atol=1e-6)):
        raise MeasuredFaceUnavailable('measured_face_geometry_source_invalid')
    return k, t, p


class FrozenMeasuredFaceSource(_Frozen):
    """Actual same-frame depth/TF/calibration, captured before any camera motion."""
    def __init__(self, raw_depth, encoding, depth_scale, intrinsics, base_from_camera,
                 support_plane, config, source_ns, source_id, rgb_sha256, whole_mask_sha256):
        raw = np.asarray(raw_depth)
        if (raw.ndim != 2 or raw.size > 2_000_000 or type(source_ns) is not int or source_ns <= 0
                or not isinstance(config, Config) or not isinstance(source_id, str) or not source_id
                or any(not isinstance(x, str) or len(x) != 64 for x in (rgb_sha256, whole_mask_sha256))
                or isinstance(depth_scale, (bool, np.bool_)) or not np.isfinite(depth_scale) or depth_scale <= 0
                or (encoding in ('16UC1', 'mono16') and raw.dtype != np.uint16)
                or (encoding == '32FC1' and (raw.dtype != np.float32 or depth_scale != 1.))
                or encoding not in ('16UC1', 'mono16', '32FC1')):
            raise MeasuredFaceUnavailable('measured_face_raw_source_invalid')
        k, t, p = _geometry(intrinsics, base_from_camera, support_plane)
        self.raw_depth = _immutable(raw)
        self.encoding, self.depth_scale = encoding, float(depth_scale)
        self.intrinsics, self.transform, self.plane = map(_immutable, (k, t, p))
        self.config_json = json.dumps(asdict(config), sort_keys=True, allow_nan=False)
        self.source_ns, self.source_id = source_ns, source_id
        self.rgb_sha256, self.whole_mask_sha256 = rgb_sha256, whole_mask_sha256
        self.depth_sha256 = _sha(raw)
        self.binding_sha256 = hashlib.sha256(json.dumps(dict(source_ns=source_ns, source_id=source_id,
            rgb_sha256=rgb_sha256, whole_mask_sha256=whole_mask_sha256,
            depth_sha256=self.depth_sha256, encoding=encoding, depth_scale=self.depth_scale,
            intrinsics=k.tolist(), transform=t.tolist(), plane=p.tolist(), config=self.config_json),
            sort_keys=True).encode()).hexdigest()

    def metric_depth(self):
        depth = np.asarray(self.raw_depth, dtype=np.float32)
        return depth * np.float32(self.depth_scale) if self.encoding != '32FC1' else depth


class FrozenMeasuredFace(_Frozen):
    def __init__(self, source, dense_reference, measured_points, normal, centre, audit,
                 affine_reference=None, training_reference=None):
        self.source = source
        self.reference = dense_reference
        self.points, self.normal, self.centre = map(_immutable, (measured_points, normal, centre))
        self.audit_json = json.dumps(audit, sort_keys=True, allow_nan=False)
        self.affine_reference = affine_reference
        self.training_reference = training_reference


def _fit(points, minimum):
    if len(points) < minimum or not np.all(np.isfinite(points)):
        raise MeasuredFaceUnavailable('measured_face_planar_core_insufficient')
    centre = points.mean(axis=0)
    values, axes = np.linalg.eigh((points-centre).T @ (points-centre) / len(points))
    if values[1] < 1e-8 or not np.all(np.isfinite(values)):
        raise MeasuredFaceUnavailable('measured_face_plane_degenerate')
    return centre, axes[:, 0]


def extract_measured_face(bgr, whole_mask, rgb_valid, depth, intrinsics, transform, plane, config, check):
    """One source-local planar RGB face. No reference image or identity score."""
    check()
    image, whole, depth = np.asarray(bgr), np.asarray(whole_mask), np.asarray(depth)
    valid_rgb = np.asarray(rgb_valid)
    k, transform, plane = _geometry(intrinsics, transform, plane)
    if (not isinstance(config, Config) or image.dtype != np.uint8 or image.ndim != 3
            or image.shape[2] != 3 or whole.shape != image.shape[:2] or depth.shape != whole.shape
            or valid_rgb.shape != whole.shape or valid_rgb.dtype != np.bool_
            or np.any((whole > 0) & ~valid_rgb) or not np.any(whole)
            or not callable(check)):
        raise MeasuredFaceUnavailable('measured_face_source_inputs_invalid')
    x, y, w, h = cv2.boundingRect((whole > 0).astype(np.uint8))
    x0, y0 = max(0, x-6), max(0, y-6)
    sl = np.s_[y0:min(whole.shape[0], y+h+6), x0:min(whole.shape[1], x+w+6)]
    d, mask = depth[sl], (whole[sl] > 0) & valid_rgb[sl]
    yy, xx = np.indices(d.shape)
    camera_points = _project_pixels(d, xx+x0, yy+y0, k)
    base = camera_points @ transform[:3, :3].T + transform[:3, 3]
    heights = base @ plane[:3] + plane[3]
    valid = (np.isfinite(d) & (d >= config.depth_min_m) & (d <= config.depth_max_m)
             & np.all(np.isfinite(base), axis=-1) & valid_rgb[sl])
    foreground = mask & valid & (heights >= config.foreground_min_m) & (heights <= config.foreground_max_m)
    normals, reliable = local_surface_normals(camera_points, d, valid)
    check()
    core = cv2.erode(foreground.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
    # Surface normal is closer to the measured support normal than to a side.
    seed = core & reliable & (np.abs(normals @ (transform[:3, :3].T @ plane[:3])) >= math.sqrt(.5))
    points = np.asarray(base[seed], dtype=float)
    minimum, count = config.min_object_points, len(points)
    fit = points
    for _ in range(3):
        centre, normal = _fit(fit, minimum)
        retained = np.abs((fit-centre) @ normal) <= config.support_threshold_m
        if np.all(retained):
            break
        fit = fit[retained]
    # Refit after the final removal, rather than using the pre-removal plane.
    centre, normal = _fit(fit, minimum)
    if len(fit) < .85*count:
        raise MeasuredFaceUnavailable('measured_face_plane_not_unique')
    if np.dot(normal, plane[:3]) < 0:
        normal = -normal
    angle = math.degrees(math.acos(float(np.clip(np.dot(normal, plane[:3]), -1., 1.))))
    if angle > config.support_max_normal_change_deg:
        raise MeasuredFaceUnavailable('measured_face_support_normal_inconsistent')
    distance = np.abs((base-centre) @ normal)
    top = foreground & (distance <= config.support_threshold_m)
    hardtop = seed & top
    hardother = mask & valid & ~top
    if np.count_nonzero(hardtop) < minimum or np.count_nonzero(hardtop) < .85*count:
        raise MeasuredFaceUnavailable('measured_face_complete_core_required')
    check()
    labels = np.full(d.shape, cv2.GC_BGD, np.uint8)
    labels[mask] = cv2.GC_PR_FGD
    labels[hardtop], labels[hardother] = cv2.GC_FGD, cv2.GC_BGD
    with _grabcut_lock:
        check()
        cv2.setRNGSeed(23)
        cv2.grabCut(np.ascontiguousarray(image[sl]), labels, None, np.zeros((1, 65)),
                    np.zeros((1, 65)), 3, cv2.GC_INIT_WITH_MASK)
    check()
    fg = np.isin(labels, [cv2.GC_FGD, cv2.GC_PR_FGD]).astype(np.uint8)
    number, components = cv2.connectedComponents(fg, connectivity=8)
    # A second independently measured face cannot disappear into an aggregate
    # 15-percent allowance. The original minimum measured object size applies
    # separately to every RGB-connected owner carrying a planar hard core.
    complete_owners = [i for i in range(1, number)
                       if np.count_nonzero((components == i) & hardtop) >= minimum]
    if len(complete_owners) != 1:
        raise MeasuredFaceUnavailable('measured_face_multiple_complete_owners')
    owners = [i for i in complete_owners
              if np.count_nonzero((components == i) & hardtop) >= .85*np.count_nonzero(hardtop)]
    if len(owners) != 1:
        raise MeasuredFaceUnavailable('measured_face_unique_connected_core_required')
    chosen = components == owners[0]
    if np.any(chosen & hardother) or np.any(chosen & ~mask):
        raise MeasuredFaceUnavailable('measured_face_current_owner_conflict')
    measured_points = base[chosen & top]
    final_centre, final_normal = _fit(measured_points, minimum)
    rms = float(np.sqrt(np.mean(((measured_points-final_centre) @ final_normal)**2)))
    if rms > config.support_threshold_m:
        raise MeasuredFaceUnavailable('measured_face_final_plane_residual')
    result = np.zeros(whole.shape, np.uint8)
    result[sl] = chosen
    audit = dict(source_local_selection=True, raw_depth_unchanged=True, raw_missing_depth_filled=False,
        rgb_face_includes_inferred_pixels=True, whole_mask_sha256=_sha((whole > 0).astype(np.uint8)),
        face_mask_sha256=_sha(result), face_pixels=int(np.count_nonzero(result)),
        measured_planar_pixels=int(len(measured_points)), hard_core_pixels=int(np.count_nonzero(hardtop)),
        measured_non_face_pixels=int(np.count_nonzero(hardother)),
        unknown_depth_rgb_pixels=int(np.count_nonzero(chosen & ~valid)),
        plane_seed_points=count, plane_retained_points=int(len(fit)),
        plane_support_angle_deg=angle, plane_rms_m=rms, tolerance_m=config.support_threshold_m,
        geometry_authority=False, motion_authorized=False)
    check()
    return result, measured_points, normal, centre, audit


def freeze_measured_face(source, bgr, whole_mask, rgb_valid, *, check_current,
                         deadline_monotonic, clock=time.monotonic):
    if not isinstance(source, FrozenMeasuredFaceSource):
        raise MeasuredFaceUnavailable('measured_face_source_unavailable')
    def check():
        check_current()
        if not math.isfinite(deadline_monotonic) or clock() >= deadline_monotonic:
            raise ValueError('observation_appearance_deadline_exceeded')
    check()
    if _sha(bgr) != source.rgb_sha256 or _sha(whole_mask) != source.whole_mask_sha256:
        raise MeasuredFaceUnavailable('measured_face_source_rgb_mask_changed')
    config = Config(**json.loads(source.config_json))
    mask, points, normal, centre, audit = extract_measured_face(bgr, whole_mask, rgb_valid,
        source.metric_depth(), source.intrinsics, source.transform, source.plane, config, check)
    frozen = freeze_reference(bgr, mask, rgb_valid, source.source_ns)
    check()
    affine_reference = FrozenAffineSource(frozen)
    check()
    try:
        training_reference = FrozenTrainingSource(frozen)
    except TrainingProposalUnavailable:
        # Preserve the original optional-path behavior for textureless sources.
        training_reference = None
    check()
    audit.update(source_stamp_ns=source.source_ns, source_id=source.source_id,
        source_binding_sha256=source.binding_sha256, source_depth_sha256=source.depth_sha256,
        immutable_source_features_precomputed=True,
        source_training_samples_precomputed=(training_reference is not None))
    return FrozenMeasuredFace(source, frozen, points, normal, centre, audit,
                              affine_reference, training_reference)


def _affine_features(image, region):
    orb = cv2.ORB_create(nfeatures=1000, scaleFactor=1.2, nlevels=8, edgeThreshold=15,
        firstLevel=0, WTA_K=2, scoreType=cv2.ORB_HARRIS_SCORE, patchSize=31, fastThreshold=20)
    inner = cv2.erode((region > 0).astype(np.uint8)*255, np.ones((7, 7), np.uint8))
    keypoints, descriptors = orb.detectAndCompute(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), inner)
    return np.asarray([p.pt for p in keypoints], np.float32).reshape(-1, 2), descriptors


class FrozenAffineSource(_Frozen):
    def __init__(self, reference):
        self.reference = reference
        self.reference_digest = reference.digest
        points, descriptors = _affine_features(reference.bgr, reference.mask)
        self.points = _immutable(points)
        self.descriptors = None if descriptors is None else _immutable(descriptors)


def _affine_proposal(reference, bgr, mask, source_ns, check, *, frozen_affine=None):
    # One estimator fixed before dense holdout. Matches never authorize identity.
    check()
    if frozen_affine is None:
        kr, dr = _affine_features(reference.bgr, reference.mask)
    else:
        if (not isinstance(frozen_affine, FrozenAffineSource)
                or frozen_affine.reference is not reference
                or frozen_affine.reference_digest != reference.digest):
            raise ValueError('measured_face_affine_source_binding_invalid')
        kr, dr = frozen_affine.points, frozen_affine.descriptors
    check()
    kc, dc = _affine_features(bgr, mask)
    check()
    if dr is None or dc is None or min(len(dr), len(dc)) < 2:
        raise MeasuredFaceUnavailable('measured_face_descriptors_unavailable')
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    forward = {p[0].queryIdx: p[0] for p in matcher.knnMatch(dr, dc, k=2)
               if len(p) == 2 and p[0].distance < .75*p[1].distance}
    reverse = {p[0].queryIdx: p[0] for p in matcher.knnMatch(dc, dr, k=2)
               if len(p) == 2 and p[0].distance < .75*p[1].distance}
    pairs = [v for i, v in forward.items() if v.trainIdx in reverse and reverse[v.trainIdx].trainIdx == i]
    check()
    if len(pairs) < 4:
        raise MeasuredFaceUnavailable('measured_face_insufficient_affine_matches')
    a = np.asarray([kr[p.queryIdx] for p in pairs], np.float32)
    b = np.asarray([kc[p.trainIdx] for p in pairs], np.float32)
    cv2.setRNGSeed(1729)
    affine, _ = cv2.estimateAffine2D(a, b, method=cv2.RANSAC, ransacReprojThreshold=3.,
                                  maxIters=2000, confidence=.995, refineIters=10)
    check()
    if affine is None or not np.all(np.isfinite(affine)):
        raise MeasuredFaceUnavailable('measured_face_affine_unavailable')
    return propose_homography(reference, source_ns, np.vstack([affine, [0., 0., 1.]]))


def corroborate_measured_face(frozen, bgr, whole_mask, rgb_valid, frame, *, source_ns,
                              deadline_monotonic, check, clock=time.monotonic):
    if not isinstance(frozen, FrozenMeasuredFace) or source_ns <= frozen.source.source_ns:
        raise MeasuredFaceUnavailable('measured_face_reference_binding_invalid')
    if json.dumps(asdict(frame['config']), sort_keys=True, allow_nan=False) != frozen.source.config_json:
        raise MeasuredFaceUnavailable('measured_face_configuration_changed')
    check()
    mask, points, normal, centre, audit = extract_measured_face(bgr, whole_mask, rgb_valid,
        frame['depth'], frame['intrinsics'], frame['transform'], frame['plane'], frame['config'], check)
    config = frame['config']
    angle = math.degrees(math.acos(float(np.clip(abs(normal @ frozen.normal), -1., 1.))))
    separation = max(float(np.max(np.abs((points-frozen.centre) @ frozen.normal))),
                     float(np.max(np.abs((frozen.points-centre) @ normal))))
    if angle > config.support_max_normal_change_deg or separation > config.support_max_local_shift_m:
        raise MeasuredFaceUnavailable('measured_face_cross_view_plane_inconsistent')
    proposal = _affine_proposal(frozen.reference, bgr, mask, source_ns, check,
                               frozen_affine=frozen.affine_reference)
    try:
        proposal, training = refine_train_only_proposal(frozen.reference, bgr, mask,
            rgb_valid, source_ns, proposal, check=check,
            frozen_training=frozen.training_reference)
    except TrainingProposalUnavailable as exc:
        # This additional proof is unavailable. Ownership/deadline failures
        # are not caught here and must reach the original caller unchanged.
        raise MeasuredFaceUnavailable(str(exc)) from exc
    evidence = evaluate_appearance(frozen.reference, bgr, mask, rgb_valid, source_ns, proposal,
        deadline_monotonic=deadline_monotonic, clock=clock, require_dense=True)
    check()
    audit.update(reference=json.loads(frozen.audit_json), cross_view_plane_angle_deg=angle,
        cross_view_plane_max_separation_m=separation, required_dense=True,
        dense_code=evidence.code, dense_metrics=evidence.metrics,
        train_only_registration=training)
    return evidence, audit
