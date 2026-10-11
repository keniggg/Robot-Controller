"""Bounded arrival appearance corroboration, never motion authority.

The existing histogram/shape path stays first. A supplied homography is only a
proposal: all intensity samples below come from original pixels inside BOTH
observed valid masks. No ROS, I/O, source refresh, image normalization or recovery
of metric geometry occurs here. Thresholds are experimental device-calibrated
choices, not a generally validated object-identity policy.
"""
from dataclasses import dataclass
import hashlib
import math
import time
from typing import Any, Dict, Tuple

import cv2
import numpy as np
from alicia_grasp_modes.rgb_observation_handoff import HandoffPolicy, _appearance


@dataclass(frozen=True)
class FrozenReference:
    source_stamp_ns: int
    digest: str
    bgr: np.ndarray
    mask: np.ndarray
    valid: np.ndarray
    gray: np.ndarray
    histogram: np.ndarray
    contour: np.ndarray
    bounds: Tuple[int, int, int, int]
    texture_cells: Tuple[Tuple[int, int], ...]


@dataclass(frozen=True)
class HomographyProposal:
    reference_digest: str
    current_source_stamp_ns: int
    matrix: np.ndarray


@dataclass(frozen=True)
class AppearanceEvidence:
    appearance_match: bool
    fallback_attempted: bool
    code: str
    reference_digest: str
    reference_source_stamp_ns: int
    current_source_stamp_ns: int
    metrics: Dict[str, Any]
    motion_authorized: bool = False
    metric_geometry_authority: bool = False
    reference_refreshed: bool = False


class _BudgetExpired(Exception):
    pass


def _frozen(array):
    value = np.ascontiguousarray(array)
    # Immutable bytes backing: callers cannot re-enable WRITEABLE on this view.
    return np.frombuffer(value.tobytes(), dtype=value.dtype).reshape(value.shape)


def _source(value):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)) or value <= 0:
        raise ValueError('INVALID_SOURCE_STAMP')
    return int(value)


def _inputs(bgr, mask, valid):
    bgr = np.asarray(bgr)
    mask = np.asarray(mask)
    valid = np.asarray(valid)
    if (bgr.dtype != np.uint8 or bgr.ndim != 3 or bgr.shape[2] != 3
            or bgr.shape[:2] != mask.shape or mask.shape != valid.shape
            or mask.dtype not in (np.dtype(bool), np.dtype(np.uint8))
            or valid.dtype not in (np.dtype(bool), np.dtype(np.uint8))
            or bgr.shape[0] * bgr.shape[1] > 2_000_000):
        raise ValueError('INVALID_IMAGE_OR_MASK')
    mask = (mask > 0).astype(np.uint8)
    valid = valid > 0
    if not np.any(mask):
        raise ValueError('EMPTY_MASK')
    if np.any((mask > 0) & ~valid):
        raise ValueError('MASK_CONSUMES_INVALID_RGB')
    return bgr, mask, valid


def freeze_reference(bgr, mask, rgb_valid, source_stamp_ns):
    """Freeze the actual admitted reference before looking at a current frame."""
    source_stamp_ns = _source(source_stamp_ns)
    bgr, mask, valid = _inputs(bgr, mask, rgb_valid)
    histogram, contour = _appearance(bgr, mask)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY).astype(float)
    x, y, w, h = cv2.boundingRect(mask)
    inner = (cv2.erode(mask, np.ones((7, 7), np.uint8)) > 0) & valid
    yy, xx = np.nonzero(inner)
    cells = np.c_[(xx - x) // 8, (yy - y) // 8]
    selected = []
    for key in sorted(set(map(tuple, cells))):
        take = np.all(cells == key, axis=1)
        values = gray[yy[take], xx[take]]
        if len(values) >= 25 and float(np.std(values)) >= 10.:
            selected.append(tuple(map(int, key)))
    digest = hashlib.sha256()
    digest.update(str(source_stamp_ns).encode('ascii'))
    for array in (bgr, mask, valid):
        digest.update(str((array.shape, array.dtype.str)).encode('ascii'))
        digest.update(np.ascontiguousarray(array).tobytes())
    return FrozenReference(source_stamp_ns, digest.hexdigest(),
                           *map(_frozen, (bgr, mask, valid, gray, histogram, contour)),
                           (x, y, w, h), tuple(selected))


def propose_homography(reference, current_source_stamp_ns, matrix):
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError('INVALID_HOMOGRAPHY')
    return HomographyProposal(reference.digest, _source(current_source_stamp_ns), _frozen(matrix))


def _ncc(x, y):
    if len(x) < 3 or min(float(np.std(x)), float(np.std(y))) < 1e-9:
        return None
    return float(np.corrcoef(x, y)[0, 1])


def _distance(a, b):
    return float(np.sqrt(max(0., 1. - np.sum(np.sqrt(a * b)))))


def _unique_test_pixels(test, cells, current_flat_indices):
    """Keep the first source-order correspondence per actual current test pixel.

    The choice depends only on coordinates, never brightness or residuals.
    Reference test-cell denominators remain frozen even when samples disappear.
    """
    indices = np.flatnonzero(test)
    values = current_flat_indices[indices]
    unique, first, counts = np.unique(values, return_index=True, return_counts=True)
    crossing = 0
    for value in unique[counts > 1]:
        if len(set(map(tuple, cells[indices[values == value]]))) > 1:
            crossing += 1
    kept = np.zeros_like(test)
    kept[indices[first]] = True
    return kept, dict(unique_current_test_pixels=int(len(unique)),
                     test_duplicate_pairs_removed=int(len(indices)-len(unique)),
                     current_pixels_shared_by_test_cells_before_dedup=int(crossing),
                     current_pixels_shared_by_test_cells_after_dedup=0)


def _mask_geometry(reference_mask, current_mask, H):
    inv = np.linalg.inv(H)
    if not np.all(np.isfinite(inv)):
        raise ValueError('INVALID_HOMOGRAPHY')
    for mask, transform in ((reference_mask, H), (current_mask, inv)):
        x, y, w, h = cv2.boundingRect(mask)
        corners = np.array([[x, y], [x+w, y], [x+w, y+h], [x, y+h]], float)
        homogeneous = np.c_[corners, np.ones(4)] @ transform.T
        denominator = homogeneous[:, 2]
        if (np.any(abs(denominator) <= 1e-12)
                or not (np.all(denominator > 0) or np.all(denominator < 0))):
            raise ValueError('HOMOGRAPHY_HORIZON_OR_FOLD')
        quad = homogeneous[:, :2] / denominator[:, None]
        signed_area = .5 * np.sum(quad[:, 0] * np.roll(quad[:, 1], -1)
                                 - quad[:, 1] * np.roll(quad[:, 0], -1))
        # det(H)/denominator^3 is the local projective Jacobian determinant.
        if (not np.all(np.linalg.det(transform) / denominator**3 > 0)
                or signed_area <= 0 or not cv2.isContourConvex(quad.astype(np.float32))):
            raise ValueError('HOMOGRAPHY_HORIZON_OR_FOLD')
    forward = cv2.warpPerspective(reference_mask, H, current_mask.shape[::-1],
                                  flags=cv2.INTER_NEAREST) > 0
    backward = cv2.warpPerspective(current_mask, inv, reference_mask.shape[::-1],
                                   flags=cv2.INTER_NEAREST) > 0
    ref, cur = reference_mask > 0, current_mask > 0
    return dict(forward_mask_iou=float(np.count_nonzero(forward & cur) / max(1, np.count_nonzero(forward | cur))),
                backward_mask_iou=float(np.count_nonzero(backward & ref) / max(1, np.count_nonzero(backward | ref))),
                current_retained_fraction=float(np.count_nonzero(forward & cur) / np.count_nonzero(cur)),
                reference_retained_fraction=float(np.count_nonzero(backward & ref) / np.count_nonzero(ref)),
                orientation_preserved=True, projective_horizon_excluded=True)


def evaluate_appearance(reference, current_bgr, current_mask, current_rgb_valid,
                        current_source_stamp_ns, proposal, *, deadline_monotonic,
                        clock=time.monotonic, policy=None):
    """Return typed appearance-only evidence under an absolute caller deadline.

    This function neither establishes the caller's source/TF/owner authenticity
    nor changes the existing three-consecutive-frame rule. The caller must still
    enforce those contracts. There is no warm-start cache or renewed deadline.
    """
    metrics = {}
    fallback = False
    source = 0

    def check_budget():
        now = clock()
        if (isinstance(deadline_monotonic, (bool, np.bool_))
                or not isinstance(deadline_monotonic, (int, float, np.number))
                or not np.isfinite(deadline_monotonic) or not np.isfinite(now)
                or now >= deadline_monotonic):
            raise _BudgetExpired()

    def finish(ok, code):
        try:
            check_budget()
        except _BudgetExpired:
            ok, code = False, 'DEADLINE_EXCEEDED'
        return AppearanceEvidence(ok, fallback, code, reference.digest,
                                  reference.source_stamp_ns, source, metrics)

    try:
        check_budget()
        policy = HandoffPolicy() if policy is None else policy
        if not isinstance(policy, HandoffPolicy):
            return finish(False, 'INVALID_HANDOFF_POLICY')
        source = _source(current_source_stamp_ns)
        if source <= reference.source_stamp_ns:
            return finish(False, 'CURRENT_SOURCE_NOT_AFTER_REFERENCE')
        image, mask, valid = _inputs(current_bgr, current_mask, current_rgb_valid)
        if mask.shape != reference.mask.shape:
            return finish(False, 'CAMERA_SHAPE_CHANGED')
        histogram, contour = _appearance(image, mask)
        colour = _distance(histogram, reference.histogram)
        shape = float(cv2.matchShapes(reference.contour, contour, cv2.CONTOURS_MATCH_I1, 0.))
        chroma = _distance(histogram.sum(axis=0), reference.histogram.sum(axis=0))
        metrics.update(colour_distance=colour, shape_distance=shape, chroma_distance=chroma)
        if not all(np.isfinite(v) for v in (colour, shape, chroma)):
            return finish(False, 'NONFINITE_APPEARANCE')
        if shape > policy.max_contour_distance:
            return finish(False, 'ORIGINAL_SHAPE_MISMATCH')
        if colour <= policy.max_colour_distance:
            return finish(True, 'ORIGINAL_HISTOGRAM_PASS')
        fallback = True
        if chroma > policy.max_colour_distance:
            return finish(False, 'CHROMA_MISMATCH')
        if (not isinstance(proposal, HomographyProposal)
                or proposal.reference_digest != reference.digest
                or proposal.current_source_stamp_ns != source):
            return finish(False, 'HOMOGRAPHY_SOURCE_BINDING_MISMATCH')
        H = np.asarray(proposal.matrix, dtype=float)
        if H.shape != (3, 3) or not np.all(np.isfinite(H)) or np.max(abs(H)) == 0:
            return finish(False, 'INVALID_HOMOGRAPHY')
        H = H / np.max(abs(H))
        geometry = _mask_geometry(reference.mask, mask, H)
        metrics.update(geometry)
        if (min(geometry['forward_mask_iou'], geometry['backward_mask_iou']) < .8
                or min(geometry['current_retained_fraction'], geometry['reference_retained_fraction']) < .85):
            return finish(False, 'WHOLE_MASK_INCONSISTENT')
        check_budget()
        ref_inner = (cv2.erode(reference.mask, np.ones((7, 7), np.uint8)) > 0) & reference.valid
        yy, xx = np.nonzero(ref_inner)
        transformed = np.c_[xx, yy, np.ones(len(xx))] @ H.T
        good = np.isfinite(transformed).all(axis=1) & (abs(transformed[:, 2]) > 1e-12)
        ij = np.zeros((len(xx), 2), dtype=np.int64)
        uv = transformed[good, :2] / transformed[good, 2, None]
        # Reject coordinates outside the image before integer conversion.
        bounded = (uv[:, 0] >= -.5) & (uv[:, 0] < mask.shape[1]-.5) & (uv[:, 1] >= -.5) & (uv[:, 1] < mask.shape[0]-.5)
        selected_indices = np.flatnonzero(good)
        good[selected_indices[~bounded]] = False
        ij[selected_indices[bounded]] = np.rint(uv[bounded]).astype(np.int64)
        ix, iy = ij.T
        cur_inner = (cv2.erode(mask, np.ones((7, 7), np.uint8)) > 0) & valid
        good &= cur_inner[iy.clip(0, mask.shape[0]-1), ix.clip(0, mask.shape[1]-1)]
        xx, yy, ix, iy = xx[good], yy[good], ix[good], iy[good]
        x0, y0, width, height = reference.bounds
        cells = np.c_[(xx-x0)//8, (yy-y0)//8]
        selected_cells = set(reference.texture_cells)
        selected = np.array([tuple(cell) in selected_cells for cell in cells], bool)
        train = selected & (cells.sum(axis=1) % 2 == 0)
        test = selected & (cells.sum(axis=1) % 2 == 1)
        # Nearest-pixel mapping can collapse source samples when shrinking.
        # Never let the same current pixel influence fit and held-out evidence.
        current_flat_indices = iy * mask.shape[1] + ix
        overlaps = np.intersect1d(current_flat_indices[train], current_flat_indices[test])
        train &= ~np.isin(current_flat_indices, overlaps)
        test, uniqueness = _unique_test_pixels(test, cells, current_flat_indices)
        metrics.update(uniqueness)
        current_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).astype(float)
        x, y = reference.gray[yy, xx], current_gray[iy, ix]
        reference_test_cells = sum(sum(cell) % 2 == 1 for cell in reference.texture_cells)
        metrics.update(real_pixel_pairs=len(x), training_pairs=int(train.sum()), test_pairs=int(test.sum()),
                       reference_preselected_test_cells=reference_test_cells,
                       source_train_test_overlap_pixels=0,
                       current_train_test_overlap_pixels=0,
                       current_train_pixels_removed_for_overlap=int(len(overlaps)))
        if train.sum() < 100 or test.sum() < 100 or reference_test_cells < 16:
            return finish(False, 'INSUFFICIENT_REFERENCE_TEXTURE')
        A = np.c_[x[train], np.ones(train.sum())]
        target = y[train]
        coef = np.linalg.lstsq(A, target, rcond=None)[0]
        for _ in range(3):
            check_budget()
            residual = target - A @ coef
            weights = np.minimum(1., 10. / np.maximum(abs(residual), 1e-12))
            sw = np.sqrt(weights)
            coef = np.linalg.lstsq(A*sw[:, None], target*sw, rcond=None)[0]
        # Never clip gain into acceptance. Held-out pixels did not fit coef.
        gain, intercept = map(float, coef)
        errors = y[test] - (gain*x[test]+intercept)
        std = float(np.std(y[test]))
        ncc = _ncc(x[test], y[test])
        rmse = float(np.sqrt(np.mean(errors**2)))
        nrmse = rmse / std if std > 1e-9 else None
        rows = []
        for key in sorted(set(map(tuple, cells[test]))):
            check_budget()
            take = test & np.all(cells == key, axis=1)
            if take.sum() < 25:
                continue
            rows.append(dict(cell_xy=list(map(int, key)), pairs=int(take.sum()),
                             ncc=_ncc(x[take], y[take]),
                             quadrant=[int(np.mean(xx[take]) >= x0+width/2), int(np.mean(yy[take]) >= y0+height/2)]))
        good_cells = [row for row in rows if row['ncc'] is not None and row['ncc'] >= .7]
        quadrants = len(set(tuple(row['quadrant']) for row in rows))
        good_quadrants = len(set(tuple(row['quadrant']) for row in good_cells))
        metrics.update(affine_gain=gain, affine_intercept=intercept, holdout_ncc=ncc,
                       holdout_rmse=rmse, holdout_nrmse=nrmse, observed_test_cells=len(rows),
                       observed_test_quadrants=quadrants, good_test_cells=len(good_cells),
                       good_test_quadrants=good_quadrants,
                       required_good_test_cells=math.ceil(reference_test_cells/2), test_cell_metrics=rows)
        if (not all(np.isfinite(value) for value in (gain, intercept, rmse))
                or ncc is None or nrmse is None
                or not np.isfinite(ncc) or not np.isfinite(nrmse)
                or any(row['ncc'] is not None and not np.isfinite(row['ncc']) for row in rows)):
            return finish(False, 'NONFINITE_OR_DEGENERATE_DENSE_EVIDENCE')
        if not .5 <= gain <= 2.:
            return finish(False, 'AFFINE_GAIN_OUT_OF_BOUNDS')
        if len(rows) < 16 or good_quadrants < 3 or len(good_cells) < math.ceil(reference_test_cells/2):
            return finish(False, 'DISTRIBUTED_TEXTURE_NOT_CONFIRMED')
        if ncc < .65 or nrmse > .8:
            return finish(False, 'HOLDOUT_TEXTURE_MISMATCH')
        return finish(True, 'DENSE_TEXTURE_CORROBORATED')
    except _BudgetExpired:
        return AppearanceEvidence(False, fallback, 'DEADLINE_EXCEEDED', reference.digest,
                                  reference.source_stamp_ns, source, metrics)
    except (ValueError, np.linalg.LinAlgError, cv2.error, OverflowError) as error:
        return finish(False, str(error) if isinstance(error, ValueError) else 'INVALID_NUMERICAL_INPUT')
