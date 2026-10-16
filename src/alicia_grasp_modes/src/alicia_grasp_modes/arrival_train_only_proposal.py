"""One bounded registration refinement using fixed PRE training pixels only.

The independent dense validator sees every pixel consumed by every optimizer
trial and finite-difference probe. No holdout score chooses a model or stop.
"""
import hashlib
from dataclasses import dataclass
import cv2
import numpy as np
from scipy.optimize import least_squares

from .arrival_dense_identity import (HomographyProposal, _inputs, _source, _frozen, _array_sha,
                                     propose_trained_homography)


class TrainingProposalUnavailable(ValueError):
    pass


def _samples(reference):
    x0, y0, width, height = reference.bounds
    inner = (cv2.erode(reference.mask, np.ones((7, 7), np.uint8)) > 0) & reference.valid
    yy, xx = np.nonzero(inner)
    cells = np.c_[(xx-x0)//8, (yy-y0)//8]
    selected = set(reference.texture_cells)
    take = np.array([tuple(cell) in selected for cell in cells], bool)
    take &= cells.sum(axis=1) % 2 == 0
    # A fixed interior subset of source training cells, frozen independently
    # of current images. The source holdout cells never fit the transform.
    take &= ((xx-x0) % 8 >= 2) & ((xx-x0) % 8 < 6)
    take &= ((yy-y0) % 8 >= 2) & ((yy-y0) % 8 < 6)
    xx, yy = xx[take], yy[take]
    if len(xx) < 100:
        raise TrainingProposalUnavailable('insufficient_fixed_train_samples')
    normalizer = np.array([[1./width, 0., -(x0+width/2)/width],
                           [0., 1./height, -(y0+height/2)/height], [0., 0., 1.]])
    source = np.c_[xx, yy, np.ones(len(xx))]
    return source, source @ normalizer.T, normalizer, reference.gray[yy, xx]


@dataclass(frozen=True, init=False)
class FrozenTrainingSource:
    reference: object
    reference_digest: str
    samples: tuple

    def __init__(self, reference):
        object.__setattr__(self, 'reference', reference)
        object.__setattr__(self, 'reference_digest', reference.digest)
        object.__setattr__(self, 'samples', tuple(_frozen(x) for x in _samples(reference)))


class _BilinearTraining:
    """Exact residual/Jacobian; derivatives read the same four actual pixels."""
    def __init__(self, gray, domain, predicted, normalized, reference_values, check):
        self.gray, self.domain = gray, domain
        self.predicted, self.normalized, self.reference_values = predicted, normalized, reference_values
        self.check = check
        self.consumed = np.zeros(gray.shape, bool)
        self.residual_calls = self.jacobian_calls = 0
        self._last = None

    def _evaluate(self, parameters):
        self.check()
        if self._last is not None and np.array_equal(parameters, self._last[0]):
            self.check()
            return self._last[1], self._last[2]
        p = np.asarray(parameters)
        if p.shape != (8,) or not np.all(np.isfinite(p)):
            raise TrainingProposalUnavailable('train_parameters_nonfinite')
        with np.errstate(over='ignore', invalid='ignore'):
            uv = self.predicted + self.normalized @ p[:6].reshape(2, 3).T
        if not np.all(np.isfinite(uv)):
            raise TrainingProposalUnavailable('train_warp_nonfinite')
        # Bound floating coordinates before any integer conversion.
        if (np.any(uv[:, 0] < 0.) or np.any(uv[:, 1] < 0.)
                or np.any(uv[:, 0] >= self.gray.shape[1]-1)
                or np.any(uv[:, 1] >= self.gray.shape[0]-1)):
            raise TrainingProposalUnavailable('train_warp_left_image')
        bx, by = np.floor(uv).astype(np.int64).T
        for dy in (0, 1):
            for dx in (0, 1):
                if not np.all(self.domain[by+dy, bx+dx]):
                    raise TrainingProposalUnavailable('train_warp_left_current_face')
        # Record rejected trial steps too; no later holdout may read this union.
        for dy in (0, 1):
            for dx in (0, 1):
                self.consumed[by+dy, bx+dx] = True
        v00, v10 = self.gray[by, bx], self.gray[by, bx+1]
        v01, v11 = self.gray[by+1, bx], self.gray[by+1, bx+1]
        fx, fy = (uv-np.c_[bx, by]).T
        values = ((1-fx)*(1-fy)*v00 + fx*(1-fy)*v10
                  + (1-fx)*fy*v01 + fx*fy*v11)
        du = (1-fy)*(v10-v00) + fy*(v11-v01)
        dv = (1-fx)*(v01-v00) + fx*(v11-v10)
        residual = values - (p[6]*self.reference_values+p[7])
        jacobian = np.c_[du[:, None]*self.normalized, dv[:, None]*self.normalized,
                         -self.reference_values, -np.ones(len(values))]
        self._last = (p.copy(), residual, jacobian)
        self.check()
        # scipy scales residual/J arrays in-place for robust loss. Return
        # independent arrays so a jacobian lookup cannot reuse altered values.
        return residual, jacobian

    def residual(self, p):
        self.residual_calls += 1
        return self._evaluate(p)[0].copy()

    def jacobian(self, p):
        self.jacobian_calls += 1
        return self._evaluate(p)[1].copy()


def _bounded_domain_samples(domain, predicted, normalized):
    """Geometry-only domain for every possible fixed-bound optimizer trial.

    Each affine row has three coefficients bounded by +/-3. Its complete
    reachable interval is therefore +/-3*sum(abs(normalized)). Retain a PRE
    training point only if the rectangle containing ALL corresponding bilinear
    supports is valid current face. No RGB value or holdout enters this choice.
    """
    with np.errstate(over='ignore', invalid='ignore'):
        radius = 3.*np.sum(np.abs(normalized), axis=1)
        low, high = predicted-radius[:, None], predicted+radius[:, None]
    safe = np.all(np.isfinite(low) & np.isfinite(high), axis=1)
    finite = np.flatnonzero(safe)
    safe[finite] &= ((low[finite, 0] >= 0.) & (low[finite, 1] >= 0.)
        & (high[finite, 0] < domain.shape[1]-1) & (high[finite, 1] < domain.shape[0]-1))
    indices = np.flatnonzero(safe)
    if len(indices):
        x0, y0 = np.floor(low[indices]).astype(np.int64).T
        # Exclusive upper end includes floor(high)+1 for bilinear support.
        x1, y1 = (np.floor(high[indices]).astype(np.int64)+2).T
        integral = cv2.integral((~domain).astype(np.uint8), sdepth=cv2.CV_32S)
        invalid = integral[y1, x1]-integral[y0, x1]-integral[y1, x0]+integral[y0, x0]
        safe[indices] &= invalid == 0
    return safe


def refine_train_only_proposal(reference, image, mask, valid, source_ns, initial,
                               *, check, jacobian='analytic', frozen_training=None):
    """Fixed 12-evaluation refinement, with caller ownership/time on every call.

    The numerical-Jacobian option is for explicit offline equivalence tests.
    Production always uses the same analytic implementation, never score choice.
    """
    check()
    source_ns = _source(source_ns)
    image, mask, valid = _inputs(image, mask, valid)
    original_image = image
    # Gray fitting and the final evidence binding must describe the same
    # current RGB snapshot even if the caller owns a writable input array.
    image = _frozen(image)
    image_sha256 = _array_sha(image)
    check()
    if (not isinstance(initial, HomographyProposal)
            or initial.reference_digest != reference.digest
            or initial.current_source_stamp_ns != source_ns
            or source_ns <= reference.source_stamp_ns
            or mask.shape != reference.mask.shape
            or jacobian not in ('analytic', '2-point')):
        raise ValueError('train_proposal_source_binding_invalid')
    H = np.asarray(initial.matrix, dtype=float)
    if H.shape != (3, 3) or not np.all(np.isfinite(H)) or not np.allclose(H[2], [0., 0., 1.], rtol=0., atol=1e-12):
        raise TrainingProposalUnavailable('train_affine_initializer_required')
    if frozen_training is None:
        source, normalized, normalizer, x = _samples(reference)
    else:
        if (not isinstance(frozen_training, FrozenTrainingSource)
                or frozen_training.reference is not reference
                or frozen_training.reference_digest != reference.digest):
            raise ValueError('train_frozen_source_binding_invalid')
        source, normalized, normalizer, x = frozen_training.samples
    check()
    with np.errstate(over='ignore', invalid='ignore', divide='ignore'):
        projected = source @ H.T
        predicted = projected[:, :2] / projected[:, 2, None]
    if not np.all(np.isfinite(predicted)):
        raise TrainingProposalUnavailable('train_warp_nonfinite')
    domain = (mask > 0) & valid
    check()
    selected = _bounded_domain_samples(domain, predicted, normalized)
    check()
    if np.count_nonzero(selected) < 100:
        raise TrainingProposalUnavailable('insufficient_fixed_train_domain')
    source_sample_count = len(source)
    source, predicted, normalized, x = (a[selected] for a in (source, predicted, normalized, x))
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).astype(float)
    problem = _BilinearTraining(gray, domain, predicted, normalized, x, check)
    check()
    result = least_squares(problem.residual, np.array([0., 0., 0., 0., 0., 0., 1., 0.]),
        method='trf', jac=problem.jacobian if jacobian == 'analytic' else '2-point',
        bounds=([-3.]*6+[-np.inf]*2, [3.]*6+[np.inf]*2), loss='soft_l1', f_scale=10.,
        max_nfev=12, ftol=1e-5, xtol=1e-5, gtol=1e-5)
    check()
    if not np.all(np.isfinite(result.x)) or not np.isfinite(result.cost):
        raise TrainingProposalUnavailable('train_result_nonfinite')
    refined = H.copy()
    refined[:2] += result.x[:6].reshape(2, 3) @ normalizer
    if _array_sha(original_image) != image_sha256:
        raise ValueError('train_current_source_rgb_changed')
    proposal = propose_trained_homography(reference, source_ns, refined, problem.consumed,
                                          image, mask, valid)
    check()
    audit = dict(training_samples=len(source), source_preselected_training_samples=source_sample_count,
        source_samples_removed_by_bounded_current_domain=source_sample_count-len(source),
        current_train_domain_uses_only_mask_and_valid=True,
        fixed_affine_increment_coefficient_bound_px=3., fixed_optimizer_max_nfev=12,
        residual_calls=problem.residual_calls,
        jacobian_calls=problem.jacobian_calls, optimizer_nfev=result.nfev,
        optimizer_status=int(result.status), optimizer_parameters=result.x.tolist(),
        optimizer_final_cost=float(result.cost), analytic_jacobian=jacobian == 'analytic',
        all_current_interpolation_support_pixels=int(np.count_nonzero(problem.consumed)),
        current_support_sha256=hashlib.sha256(problem.consumed.tobytes()).hexdigest(),
        preselected_training_only=True, final_holdout_used_for_search_or_selection=False,
        geometry_authority=False, motion_authorized=False)
    return proposal, audit
