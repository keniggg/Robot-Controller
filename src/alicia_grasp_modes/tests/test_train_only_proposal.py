"""The registration Jacobian and training/holdout independence are contracts."""
import numpy as np
import pytest
from alicia_grasp_modes.arrival_train_only_proposal import (
    _BilinearTraining, _bounded_domain_samples, refine_train_only_proposal, TrainingProposalUnavailable)
from alicia_grasp_modes import arrival_dense_identity as dense
from test_measured_face_independent import flat_scene


def problem(check=lambda: None):
    rng = np.random.RandomState(401)
    gray = rng.uniform(0., 255., (40, 50))
    normalized = np.c_[rng.uniform(-.45, .45, (140, 2)), np.ones(140)]
    predicted = np.c_[rng.randint(4, 43, 140)+.32, rng.randint(4, 33, 140)+.61]
    return _BilinearTraining(gray, np.ones(gray.shape, bool), predicted,
        normalized, rng.uniform(0., 255., 140), check)


def test_exact_bilinear_jacobian_matches_independent_central_differences():
    p = np.array([.017, -.033, .025, -.035, .018, -.019, .83, 27.])
    item = problem()
    analytic = item.jacobian(p)
    columns = []
    epsilon = 1e-6
    for i in range(8):
        step = np.zeros(8); step[i] = epsilon
        columns.append((item.residual(p+step)-item.residual(p-step))/(2.*epsilon))
    np.testing.assert_allclose(analytic, np.column_stack(columns), rtol=1e-6, atol=1e-6)


def test_robust_solver_mutation_cannot_poison_repeated_residual_or_jacobian():
    item = problem(); p = np.array([0.]*6+[1., 0.])
    expected, jacobian = item.residual(p), item.jacobian(p)
    item.residual(p)[:] = 0.
    item.jacobian(p)[:] = 0.
    np.testing.assert_array_equal(item.residual(p), expected)
    np.testing.assert_array_equal(item.jacobian(p), jacobian)


@pytest.mark.parametrize('method', ['residual', 'jacobian'])
def test_owner_checks_cover_cached_and_new_solver_calls(method):
    marker = ValueError('owner retired during native solve')
    active = [True]
    def check():
        if not active[0]: raise marker
    item = problem(check); p = np.array([0.]*6+[1., 0.]);getattr(item,method)(p)
    active[0] = False
    with pytest.raises(ValueError) as caught:getattr(item,method)(p)
    assert caught.value is marker


@pytest.mark.parametrize('value', [float('nan'), float('inf'), 1e100, -1e100])
def test_invalid_or_extreme_warp_never_indexes_current_pixels(value):
    item = problem(); item.predicted[0, 0] = value
    with pytest.raises(TrainingProposalUnavailable):item.residual(np.array([0.]*6+[1., 0.]))
    assert not item.consumed.any()


@pytest.mark.parametrize('neighbour', [(0,0),(0,1),(1,0),(1,1)])
def test_each_bilinear_neighbour_must_be_in_actual_current_face(neighbour):
    item = problem(); x,y=np.floor(item.predicted[0]).astype(int)
    dy,dx=neighbour;item.domain[y+dy,x+dx]=False
    with pytest.raises(TrainingProposalUnavailable,match='left_current_face'):
        item.residual(np.array([0.]*6+[1.,0.]))
    assert not item.consumed.any()


def test_all_trial_supports_are_accumulated_including_a_rejected_trial():
    item=problem();p=np.array([0.]*6+[1.,0.]);item.residual(p);before=item.consumed.copy()
    trial=p.copy();trial[2]=1.1;item.residual(trial);after=item.consumed.copy()
    assert np.any(after & ~before)
    item.residual(p)
    np.testing.assert_array_equal(item.consumed,after)


def test_fixed_bounded_training_domain_covers_all_affine_parameter_corners():
    item=problem(); item.domain[8:10,8:15]=False
    selected=_bounded_domain_samples(item.domain,item.predicted,item.normalized)
    assert np.count_nonzero(selected)>30
    good=_BilinearTraining(item.gray,item.domain,item.predicted[selected],
        item.normalized[selected],item.reference_values[selected],lambda:None)
    import itertools
    for corner in itertools.product((-3.,3.),repeat=6):
        good.residual(np.array([*corner,1.,0.]))
    assert not np.any(good.consumed & ~item.domain)


def test_trained_proposal_binds_current_pixels_and_excludes_history_without_changing_denominator():
    scene=flat_scene();ref=dense.freeze_reference(scene['rgb'],scene['mask'],scene['valid'],100)
    support=np.ones(scene['mask'].shape,bool)
    proposal=dense.propose_trained_homography(ref,101,np.eye(3),support,scene['rgb'],scene['mask'],scene['valid'])
    support[:]=False
    with pytest.raises(ValueError):proposal.optimizer_current_support.setflags(write=True)
    result=dense.evaluate_appearance(ref,scene['rgb'],scene['mask'],scene['valid'],101,proposal,
        deadline_monotonic=1.,clock=lambda:0.,require_dense=True)
    assert not result.appearance_match and result.code=='INSUFFICIENT_REFERENCE_TEXTURE'
    assert result.metrics['test_pairs']==0
    assert result.metrics['reference_preselected_test_cells']==sum(sum(c)%2==1 for c in ref.texture_cells)
    current=scene['rgb'].copy();current[0,0,0]^=1
    changed=dense.evaluate_appearance(ref,current,scene['mask'],scene['valid'],101,proposal,
        deadline_monotonic=1.,clock=lambda:0.,require_dense=True)
    assert not changed.appearance_match and changed.code=='OPTIMIZER_CURRENT_SOURCE_BINDING_MISMATCH'


def test_same_texture_proposal_still_requires_independent_dense_not_optimizer_cost():
    scene=flat_scene();ref=dense.freeze_reference(scene['rgb'],scene['mask'],scene['valid'],100)
    initial=dense.propose_homography(ref,101,np.eye(3))
    proposal,audit=refine_train_only_proposal(ref,scene['rgb'],scene['mask'],scene['valid'],101,
        initial,check=lambda:None)
    result=dense.evaluate_appearance(ref,scene['rgb'],scene['mask'],scene['valid'],101,proposal,
        deadline_monotonic=1.,clock=lambda:0.,require_dense=True)
    assert result.appearance_match and result.code=='DENSE_TEXTURE_CORROBORATED'
    assert audit['optimizer_nfev']<=12
    assert audit['analytic_jacobian'] and not audit['final_holdout_used_for_search_or_selection']
    assert not result.metric_geometry_authority and not result.motion_authorized
