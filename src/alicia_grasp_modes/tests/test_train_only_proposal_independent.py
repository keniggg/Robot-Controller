"""Independent optimizer/holdout separation, not recorded acceptance evidence."""
from dataclasses import replace
from types import SimpleNamespace
import warnings

import cv2
import numpy as np
import pytest

from alicia_grasp_modes import arrival_dense_identity as dense
from alicia_grasp_modes import arrival_train_only_proposal as train
from test_measured_face_independent import flat_scene


def fixture():
    scene = flat_scene()
    reference = dense.freeze_reference(scene['rgb'], scene['whole'], scene['valid'], 100)
    initial = dense.propose_homography(reference, 101, np.eye(3))
    return scene, reference, initial


def refine(scene, reference, initial, *, check=lambda: None, **kwargs):
    return train.refine_train_only_proposal(reference, scene['rgb'], scene['whole'],
        scene['valid'], 101, initial, check=check, **kwargs)


def evaluate(scene, reference, proposal):
    return dense.evaluate_appearance(reference, scene['rgb'], scene['whole'],
        scene['valid'], 101, proposal, deadline_monotonic=1., clock=lambda: 0.,
        require_dense=True)


def small_problem(*, check=lambda: None, predicted=None, domain=None):
    rng = np.random.RandomState(159)
    gray = rng.uniform(0., 255., (12, 14))
    normalized = np.array([[-.3, -.2, 1.], [.2, -.1, 1.], [.1, .4, 1.]])
    return train._BilinearTraining(gray, np.ones(gray.shape, bool) if domain is None else domain,
        np.array([[3.25, 4.3], [7.15, 5.45], [8.65, 8.25]]) if predicted is None else predicted,
        normalized, np.array([55., 81., 110.]), check)


def test_analytic_jacobian_agrees_with_independent_symmetric_finite_difference():
    problem = small_problem()
    p = np.array([.03, -.02, .01, -.02, .015, .025, .81, 12.])
    actual = problem.jacobian(p)
    expected = np.empty_like(actual)
    for column in range(8):
        step = np.zeros(8); step[column] = 1e-7
        expected[:, column] = (problem.residual(p+step)-problem.residual(p-step))/(2e-7)
    np.testing.assert_allclose(actual, expected, rtol=1e-6, atol=2e-6)
    assert problem.consumed.sum() == 12


def test_scipy_mutating_returned_residual_and_jacobian_cannot_pollute_cache():
    problem = small_problem()
    p = np.r_[np.zeros(6), 1., 0.]
    residual, jacobian = problem.residual(p), problem.jacobian(p)
    before = residual.copy(), jacobian.copy()
    residual[:] = -999.; jacobian[:] = -999.
    np.testing.assert_array_equal(problem.residual(p), before[0])
    np.testing.assert_array_equal(problem.jacobian(p), before[1])


def test_real_fixed_optimizer_returns_immutable_bound_support_and_original_dense_pass():
    scene, reference, initial = fixture()
    proposal, audit = refine(scene, reference, initial)
    assert isinstance(proposal, dense.TrainedHomographyProposal)
    assert audit['optimizer_nfev'] <= 12
    assert audit['preselected_training_only'] and not audit['final_holdout_used_for_search_or_selection']
    assert not audit['geometry_authority'] and not audit['motion_authorized']
    result = evaluate(scene, reference, proposal)
    assert result.appearance_match and result.code == 'DENSE_TEXTURE_CORROBORATED'
    assert result.metrics['reference_preselected_test_cells'] == sum(sum(x) % 2 for x in reference.texture_cells)
    for array in (proposal.matrix, proposal.optimizer_current_support):
        with pytest.raises(ValueError):
            array.setflags(write=True)
    with pytest.raises(AttributeError):
        proposal.current_rgb_sha256 = '0'*64


def test_rejected_trial_and_jacobian_support_remain_excluded_from_final_holdout(monkeypatch):
    scene, reference, initial = fixture()
    problems = []
    original_problem = train._BilinearTraining
    def capture_problem(*args, **kwargs):
        problem = original_problem(*args, **kwargs)
        problems.append(problem)
        return problem
    monkeypatch.setattr(train, '_BilinearTraining', capture_problem)
    p0 = np.r_[np.zeros(6), 1., 0.]
    trial = p0.copy(); trial[2] = 3.
    calls = []
    def fixed_experiment(residual, initial_values, **kwargs):
        assert kwargs['max_nfev'] == 12 and kwargs['loss'] == 'soft_l1'
        residual(p0)
        residual(trial)  # Deliberately rejected trial: final transform is p0.
        kwargs['jac'](trial)
        calls.append(True)
        return SimpleNamespace(x=p0, cost=0., nfev=2, status=1)
    monkeypatch.setattr(train, 'least_squares', fixed_experiment)
    proposal, _ = refine(scene, reference, initial)
    assert len(problems) == 1
    expected = np.zeros(scene['whole'].shape, bool)
    for p in (p0, trial):
        uv = problems[0].predicted + problems[0].normalized @ p[:6].reshape(2, 3).T
        bx, by = np.floor(uv).astype(int).T
        for dy in (0, 1):
            for dx in (0, 1):
                expected[by+dy, bx+dx] = True
    assert calls == [True]
    np.testing.assert_array_equal(proposal.optimizer_current_support, expected)
    np.testing.assert_array_equal(proposal.matrix, np.eye(3))
    result = evaluate(scene, reference, proposal)
    assert result.metrics['optimizer_contaminated_holdout_pairs_removed'] > 0
    assert result.metrics['reference_preselected_test_cells'] == sum(sum(x) % 2 for x in reference.texture_cells)
    assert result.metrics['current_train_test_overlap_pixels'] == 0


@pytest.mark.parametrize('hole', [(5, 5), (17, 17), (5, 17), (17, 5), (10, 10), (18, 18)])
def test_reachable_domain_includes_interior_holes_and_closed_endpoint_neighbour(hole):
    # radius=3*(.5+.25+1)=5.25, predicted=10.75: every possible
    # interpolation support is in inclusive integer rectangle [5,17]^2.
    domain = np.ones((30, 30), bool)
    domain[hole[1], hole[0]] = False
    selected = train._bounded_domain_samples(domain, np.array([[10.75, 10.75]]),
        np.array([[.5, -.25, 1.]]))
    assert selected.tolist() == [hole == (18, 18)]


@pytest.mark.parametrize('predicted', [[1e100, 10.], [-1e100, 10.], [np.inf, 10.],
    [np.nan, 10.], [3., 3.], [26., 26.]])
def test_reachable_domain_rejects_float_bounds_before_integer_conversion(predicted):
    with warnings.catch_warnings():
        warnings.simplefilter('error')
        selected = train._bounded_domain_samples(np.ones((30, 30), bool),
            np.array([predicted]), np.array([[.5, -.25, 1.]]))
    assert selected.tolist() == [False]


def test_geometry_domain_filter_is_not_allowed_to_waive_original_minimum(monkeypatch):
    scene, reference, initial = fixture()
    # Real observed face is now only a 12x12 current island. The unchanged
    # PRE pool cannot supply >=100 points whose entire trial domain fits it.
    scene['whole'][:] = 0; scene['whole'][105:117, 155:167] = 1
    def forbidden(*args, **kwargs):
        pytest.fail('insufficient fixed training pool reached optimizer')
    monkeypatch.setattr(train, 'least_squares', forbidden)
    with pytest.raises(train.TrainingProposalUnavailable, match='insufficient.*train'):
        refine(scene, reference, initial)


@pytest.mark.parametrize('scale', [1., .5, .25])
def test_entire_holdout_consumed_cannot_reduce_denominator_to_pass(scale):
    scene, reference, _ = fixture()
    H = np.diag([scale, scale, 1.])
    shape = scene['whole'].shape[::-1]
    scene['rgb'] = cv2.warpPerspective(scene['rgb'], H, shape, flags=cv2.INTER_NEAREST)
    scene['whole'] = cv2.warpPerspective(scene['whole'], H, shape, flags=cv2.INTER_NEAREST)
    proposal = dense.propose_trained_homography(reference, 101, H,
        np.ones(scene['whole'].shape, bool), scene['rgb'], scene['whole'], scene['valid'])
    result = evaluate(scene, reference, proposal)
    assert not result.appearance_match and result.code == 'INSUFFICIENT_REFERENCE_TEXTURE', result
    assert result.metrics['test_pairs'] == 0
    assert result.metrics['optimizer_contaminated_holdout_pairs_removed'] > 0
    assert result.metrics['reference_preselected_test_cells'] == sum(sum(x) % 2 for x in reference.texture_cells)


@pytest.mark.parametrize('which', ['rgb', 'mask', 'valid', 'writable_history',
    'empty_history', 'changed_history', 'source', 'reference'])
def test_optimizer_history_is_bound_to_exact_current_inputs(which):
    scene, reference, initial = fixture()
    proposal, _ = refine(scene, reference, initial)
    if which == 'rgb': scene['rgb'][0, 0, 0] ^= 1
    elif which == 'mask': scene['whole'][0, 0] = 1
    elif which == 'valid': scene['valid'][0, 0] = False
    elif which == 'writable_history': proposal = replace(proposal, optimizer_current_support=proposal.optimizer_current_support.copy())
    elif which == 'empty_history': proposal = replace(proposal,
        optimizer_current_support=dense._frozen(np.zeros(scene['whole'].shape, bool)))
    elif which == 'changed_history':
        changed = proposal.optimizer_current_support.copy(); changed[0, 0] ^= True
        proposal = replace(proposal, optimizer_current_support=dense._frozen(changed))
    elif which == 'source': proposal = replace(proposal, current_source_stamp_ns=102)
    else: proposal = replace(proposal, reference_digest='0'*64)
    result = evaluate(scene, reference, proposal)
    assert not result.appearance_match
    assert result.code in ('OPTIMIZER_CURRENT_SOURCE_BINDING_MISMATCH', 'HOMOGRAPHY_SOURCE_BINDING_MISMATCH')


@pytest.mark.parametrize('matrix', [np.diag([1e308, 1., 1.]),
    np.array([[1., 0., np.inf], [0., 1., 0.], [0., 0., 1.]]),
    np.array([[1., 0., 0.], [0., 1., 0.], [.01, 0., 1.]])])
def test_initializer_overflow_nonfinite_or_nonaffine_never_reaches_optimizer(monkeypatch, matrix):
    scene, reference, initial = fixture()
    initial = replace(initial, matrix=matrix)
    def forbidden(*args, **kwargs):
        pytest.fail('invalid initial warp reached optimizer')
    monkeypatch.setattr(train, 'least_squares', forbidden)
    with warnings.catch_warnings():
        warnings.simplefilter('error')
        with pytest.raises(train.TrainingProposalUnavailable):
            refine(scene, reference, initial)


@pytest.mark.parametrize('uv', [[1e100, 3.], [-1e100, 3.], [3., 1e100], [np.nan, 3.], [np.inf, 3.], [-.001, 3.], [13., 3.], [3., 11.]])
def test_nonfinite_or_extreme_warp_rejected_before_integer_conversion(uv):
    problem = small_problem(predicted=np.repeat([uv], 3, axis=0))
    with warnings.catch_warnings():
        warnings.simplefilter('error')
        with pytest.raises(train.TrainingProposalUnavailable):
            problem.residual(np.r_[np.zeros(6), 1., 0.])
    assert not np.any(problem.consumed)


@pytest.mark.parametrize('corner', [(0, 0), (1, 0), (0, 1), (1, 1)])
def test_every_bilinear_support_pixel_must_be_current_face_and_valid(corner):
    domain = np.ones((12, 14), bool)
    dx, dy = corner
    domain[4+dy, 3+dx] = False
    problem = small_problem(domain=domain)
    with pytest.raises(train.TrainingProposalUnavailable, match='train_warp_left_current_face'):
        problem.residual(np.r_[np.zeros(6), 1., 0.])
    assert not np.any(problem.consumed)


@pytest.mark.parametrize('when', [1, 2, 3, 4])
def test_owner_check_before_after_work_and_cached_jacobian_preserves_exact_exception(when):
    class OwnerRevoked(ValueError):
        pass
    failure = OwnerRevoked('manual epoch source or original absolute deadline')
    calls = []
    def check():
        calls.append(True)
        if len(calls) == when:
            raise failure
    problem = small_problem(check=check)
    p = np.r_[np.zeros(6), 1., 0.]
    with pytest.raises(OwnerRevoked) as caught:
        problem.residual(p)
        problem.jacobian(p)  # Same parameters exercise the cached path.
    assert caught.value is failure and len(calls) == when


@pytest.mark.parametrize('boundary', ['domain', 'optimizer', 'proposal'])
def test_real_refinement_cannot_return_after_owner_revoked_at_final_boundaries(monkeypatch, boundary):
    scene, reference, initial = fixture()
    class OwnerRevoked(ValueError):
        pass
    failure = OwnerRevoked('original owner/deadline revoked')
    active = [True]
    def check():
        if not active[0]:
            raise failure
    name = {'domain': '_bounded_domain_samples', 'optimizer': 'least_squares',
            'proposal': 'propose_trained_homography'}[boundary]
    original = getattr(train, name)
    def revoke_after_real_work(*args, **kwargs):
        result = original(*args, **kwargs)
        active[0] = False
        return result
    monkeypatch.setattr(train, name, revoke_after_real_work)
    with pytest.raises(OwnerRevoked) as caught:
        refine(scene, reference, initial, check=check)
    assert caught.value is failure


def test_changed_current_rgb_during_real_optimizer_cannot_bind_old_training_to_new_pixels(monkeypatch):
    scene, reference, initial = fixture()
    original = train.least_squares
    def mutate_after_actual_fit(*args, **kwargs):
        result = original(*args, **kwargs)
        scene['rgb'][100, 100, 0] ^= 1
        return result
    monkeypatch.setattr(train, 'least_squares', mutate_after_actual_fit)
    with pytest.raises(ValueError, match='source|binding|changed'):
        refine(scene, reference, initial)


def test_changed_only_unseen_holdout_cannot_change_training_proposal_but_fails_validation():
    scene, reference, initial = fixture()
    good, _ = refine(scene, reference, initial)
    support = good.optimizer_current_support
    changed = dict(scene, rgb=scene['rgb'].copy())
    # Preserve every current pixel read in the positive optimization. Change
    # all other observed object pixels independently, including odd holdout.
    outside = (scene['whole'] > 0) & ~support
    changed['rgb'][outside] = np.random.RandomState(2025).randint(0, 105,
        (np.count_nonzero(outside), 1)).astype(np.uint8)
    bad, _ = refine(changed, reference, initial)
    np.testing.assert_array_equal(good.matrix, bad.matrix)
    np.testing.assert_array_equal(good.optimizer_current_support, bad.optimizer_current_support)
    result = evaluate(changed, reference, bad)
    assert not result.appearance_match
    assert result.code in ('DISTRIBUTED_TEXTURE_NOT_CONFIRMED', 'HOLDOUT_TEXTURE_MISMATCH', 'AFFINE_GAIN_OUT_OF_BOUNDS')
