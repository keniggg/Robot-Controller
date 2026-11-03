"""Full-box proof dispatch and real shared-clock fault injection, without ROS.

The existing synthetic scene isolates time and control flow. Independent
recorded-input tests exercise the actual geometric rejection/acceptance gates.
"""
from copy import deepcopy

import numpy as np
import pytest

from alicia_grasp_modes import reachable_observation_views as module
from alicia_flexible_grasp.robot.observation_path_guard import ObservationPathError
from test_reachable_observation_exposure_tiers import synthetic


def one_endpoint(synthetic):
    arguments, observed, solver = synthetic
    arguments['gaps'] = arguments['gaps'][:1]
    arguments['max_directions'] = 1
    arguments['max_iterations'] = 1
    return arguments, observed, solver


def coarse_failure(*arguments):
    return dict(ok=False, minimum_support_clearance_lower_bound_m=.002,
                goal_joint_positions_rad=arguments[1].tolist(),
                joint_error_bounds_rad=arguments[2].tolist(),
                required_support_clearance_m=.003)


def test_qualified_coarse_bound_preserves_report_and_never_starts_refinement(synthetic, monkeypatch):
    args, _, _ = one_endpoint(synthetic)
    original = dict(ok=True, minimum_support_clearance_lower_bound_m=.01, original_receipt=True)
    monkeypatch.setattr(module, 'observation_endpoint_support_bound', lambda *_a: deepcopy(original))
    monkeypatch.setattr(module, 'observation_following_support_bound',
                        lambda *_a, **_k: pytest.fail('qualified coarse bound started a new proof'))
    rows, report = module.generate_reachable_observation_views(**args)
    assert len(rows) == 1
    assert rows[0]['endpoint_checks']['conditional_endpoint_support'] == original
    assert 'support_bound_refinements' not in report['directions'][0]


@pytest.mark.parametrize('shared_seconds', [.8, 2., 5.])
def test_proof_uses_direction_remaining_time_and_exact_existing_physical_contract(
        synthetic, monkeypatch, shared_seconds):
    args, _, solver = one_endpoint(synthetic)
    args['max_seconds'] = shared_seconds
    now = [0.]
    clock = lambda: now[0]
    calls = []
    real_proof = module.observation_following_support_bound

    def delayed_solver(*a, **kw):
        result = solver(*a, **kw)
        now[0] = .1
        return result

    def independent_proof(*a, **kw):
        calls.append((a, dict(kw)))
        assert kw['clock'] is clock
        assert kw['max_seconds'] == pytest.approx(shared_seconds / 2. - .1)
        assert kw['max_seconds'] < shared_seconds - now[0]
        assert a[0] is args['fk'] and a[5] is args['gripper']
        np.testing.assert_array_equal(a[1], [.3])
        np.testing.assert_array_equal(a[2], args['joint_error_bounds_rad'])
        np.testing.assert_array_equal(a[3], args['source']['geometry']['support_normal_base'])
        assert a[4] == args['source']['geometry']['support_offset_m']
        assert a[6] == args['opening_width_m'] == .05
        assert a[5].support_clearance_m == .003
        return real_proof(*a, **kw)

    monkeypatch.setattr(module, '_backend', lambda: delayed_solver)
    monkeypatch.setattr(module, 'observation_endpoint_support_bound', coarse_failure)
    monkeypatch.setattr(module, 'observation_following_support_bound', independent_proof)
    before = deepcopy(args)
    rows, report = module.generate_reachable_observation_views(**args, clock=clock)
    assert len(rows) == len(calls) == 1
    final = rows[0]['endpoint_checks']['conditional_endpoint_support']
    assert final['ok'] and final['box_checks'] >= 1
    assert final['coarse_endpoint_bound']['ok'] is False
    assert final['coarse_endpoint_bound']['minimum_support_clearance_lower_bound_m'] == .002
    assert final['support_proof_budget_sec'] == pytest.approx(shared_seconds / 2. - .1)
    assert final['support_bound_method'] == 'existing_full_joint_error_box_after_inconclusive_coarse_bound'
    receipt = report['directions'][0]['support_bound_refinements'][0]
    assert receipt['proof_returned'] and receipt['checked_within_remaining_budget']
    assert receipt['proof']['ok']
    assert not rows[0]['execution_authority'] and not rows[0]['complete_path_checked']
    assert args['current_joint_positions'] == before['current_joint_positions']
    assert args['gaps'] == before['gaps']
    np.testing.assert_array_equal(args['source']['points_base'], before['source']['points_base'])


def test_refinement_after_first_admission_uses_only_shared_request_remainder(synthetic, monkeypatch):
    args, _, solver = synthetic
    args['gaps'] = args['gaps'][:1]
    args['max_directions'] = 1
    args['max_seconds'] = 1.
    now, solver_calls, proof_calls = [0.], [], []
    real_proof = module.observation_following_support_bound

    def delayed(*a, **kw):
        solver_calls.append(True)
        result = solver(*a, **kw)
        now[0] += .1
        return result

    def coarse(*a):
        return ({'ok': True, 'minimum_support_clearance_lower_bound_m': .01}
                if len(solver_calls) == 1 else coarse_failure(*a))

    def proof(*a, **kw):
        proof_calls.append(kw['max_seconds'])
        return real_proof(*a, **kw)

    monkeypatch.setattr(module, '_backend', lambda: delayed)
    monkeypatch.setattr(module, 'observation_endpoint_support_bound', coarse)
    monkeypatch.setattr(module, 'observation_following_support_bound', proof)
    rows, report = module.generate_reachable_observation_views(**args, clock=lambda: now[0])
    assert len(rows) == len(solver_calls) == 2
    assert proof_calls == pytest.approx([.8])
    assert report['elapsed_sec'] == pytest.approx(.2)


@pytest.mark.parametrize('boundary', ['direction_exact', 'direction_after', 'shared_exact', 'shared_after'])
def test_a_successful_proof_returning_at_or_after_either_deadline_is_discarded(
        synthetic, monkeypatch, boundary):
    args, _, solver = one_endpoint(synthetic)
    args['max_seconds'] = 1.
    now = [0.]
    real_proof = module.observation_following_support_bound

    def delayed(*a, **kw):
        result = solver(*a, **kw)
        now[0] = .1
        return result

    def late(*a, **kw):
        result = real_proof(*a, **kw)
        assert result['ok']
        now[0] = (.5 if boundary.startswith('direction') else 1.) + (
            1e-6 if boundary.endswith('after') else 0.)
        return result

    monkeypatch.setattr(module, '_backend', lambda: delayed)
    monkeypatch.setattr(module, 'observation_endpoint_support_bound', coarse_failure)
    monkeypatch.setattr(module, 'observation_following_support_bound', late)
    rows, report = module.generate_reachable_observation_views(**args, clock=lambda: now[0])
    assert not rows and report['candidate_count'] == 0
    receipt = report['directions'][0]['support_bound_refinements'][0]
    assert receipt['proof_returned'] and receipt['proof']['ok']
    assert not receipt['checked_within_remaining_budget']
    assert report['directions'][0]['accepted'] is False
    assert report['directions'][0]['reason'] == (
        'direction wall-clock slice exhausted' if boundary.startswith('direction')
        else 'shared wall-clock budget exhausted')


@pytest.mark.parametrize('deadline', [.5, 1.])
def test_no_remaining_time_after_coarse_check_never_starts_full_proof(synthetic, monkeypatch, deadline):
    args, _, _ = one_endpoint(synthetic)
    args['max_seconds'] = 1.
    now = [0.]

    def coarse(*a):
        now[0] = deadline
        return coarse_failure(*a)

    monkeypatch.setattr(module, 'observation_endpoint_support_bound', coarse)
    monkeypatch.setattr(module, 'observation_following_support_bound',
        lambda *_a, **_k: pytest.fail('expired proof started'))
    rows, report = module.generate_reachable_observation_views(**args, clock=lambda: now[0])
    assert not rows
    assert 'support_bound_refinements' not in report['directions'][0]


@pytest.mark.parametrize('reason', ['following support box proof budget exhausted',
    'following support box unresolved at precision limit', 'invalid support box physical contract'])
def test_unresolved_or_invalid_proof_never_turns_coarse_failure_into_a_candidate(synthetic, monkeypatch, reason):
    args, _, _ = one_endpoint(synthetic)

    def reject(*_a, **_kw):
        raise ObservationPathError(reason)

    monkeypatch.setattr(module, 'observation_endpoint_support_bound', coarse_failure)
    monkeypatch.setattr(module, 'observation_following_support_bound', reject)
    rows, report = module.generate_reachable_observation_views(**args)
    assert not rows and report['directions'][0]['reason'] == reason
    receipt = report['directions'][0]['support_bound_refinements'][0]
    assert not receipt['proof_returned'] and not receipt['checked_within_remaining_budget']
    assert receipt['coarse_endpoint_bound']['ok'] is False


def test_box_counterexample_remains_an_explicit_rejection(synthetic, monkeypatch):
    args, _, _ = one_endpoint(synthetic)
    rejection = dict(ok=False, reason='joint error box contains insufficient support clearance',
        counterexample_joint_positions_rad=[.3], counterexample_model_clearance_m=.001, box_checks=1)
    monkeypatch.setattr(module, 'observation_endpoint_support_bound', coarse_failure)
    monkeypatch.setattr(module, 'observation_following_support_bound', lambda *_a, **_k: deepcopy(rejection))
    rows, report = module.generate_reachable_observation_views(**args)
    assert not rows
    final = report['directions'][0]['endpoint_checks']['conditional_endpoint_support']
    assert not final['ok'] and final['counterexample_model_clearance_m'] == .001
    assert not final['coarse_endpoint_bound']['ok']


def test_late_proof_preserves_an_already_admitted_endpoint_without_new_budget(synthetic, monkeypatch):
    args, _, solver = synthetic
    now, calls = [0.], []
    real_proof = module.observation_following_support_bound

    def delayed(*a, **kw):
        result = solver(*a, **kw)
        calls.append(True); now[0] += .1
        return result

    def coarse(*a):
        return {'ok': True, 'minimum_support_clearance_lower_bound_m': .01} if len(calls)==1 else coarse_failure(*a)

    def late(*a, **kw):
        result = real_proof(*a, **kw)
        now[0] = args['max_seconds']
        return result

    monkeypatch.setattr(module, '_backend', lambda: delayed)
    monkeypatch.setattr(module, 'observation_endpoint_support_bound', coarse)
    monkeypatch.setattr(module, 'observation_following_support_bound', late)
    rows, report = module.generate_reachable_observation_views(**args, clock=lambda: now[0])
    assert len(rows) == 1 and len(calls) == 2
    assert report['status'] == 'budget_exhausted'
    assert report['directions'][0]['accepted'] and not report['directions'][1]['accepted']
    assert 'support_bound_refinements' not in report['directions'][0]
    assert not report['directions'][1]['support_bound_refinements'][0]['checked_within_remaining_budget']


def test_unexpected_caller_cancellation_propagates_unchanged(synthetic, monkeypatch):
    args, _, _ = one_endpoint(synthetic)
    class CancelRequest(Exception):
        pass
    cancellation = CancelRequest('request owner changed')
    def cancel(*_a, **_kw):
        raise cancellation
    monkeypatch.setattr(module, 'observation_endpoint_support_bound', coarse_failure)
    monkeypatch.setattr(module, 'observation_following_support_bound', cancel)
    with pytest.raises(CancelRequest) as caught:
        module.generate_reachable_observation_views(**args)
    assert caught.value is cancellation
