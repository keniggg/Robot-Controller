"""Actual ModeAware proposal ingress; solver transport only is controlled."""
from copy import deepcopy
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_remote_modes import configured, module, base
from test_reachable_observation_views import recorded_inputs
from test_reachable_observation_integration import context, recorded_proposal, proposals
from alicia_flexible_grasp.robot.stationary_following import SDK_QUANTUM_RAD as COUNT


@pytest.fixture
def seed_case(configured, monkeypatch, recorded_inputs, recorded_proposal):
    node, params, source, prepared, reference = context(configured, monkeypatch, recorded_inputs)
    names = ['Joint%d' % i for i in range(1, 7)]
    params['/alicia_controller/constraints'] = {n: {'trajectory': .2, 'goal': .035} for n in names}
    params['/robot/observation_tracking_contract_enabled'] = True
    params['/alicia_controller/stop_trajectory_duration'] = .5
    params['/gui/joint_direct_mode'] = False
    node._measurement_remote_epoch_ns = prepared.stamp.to_nsec()-1
    node.latest_joint_state = NS(name=names, position=list(recorded_inputs['current_joint_positions']),
        header=NS(stamp=prepared.stamp))
    now = [prepared.stamp]
    monkeypatch.setattr(module.rospy.Time, 'now', lambda: now[0])
    state = dict(calls=[], elapsed=2., mutate=lambda: None)
    def generate(**kwargs):
        state['calls'].append(deepcopy(kwargs['current_joint_positions']))
        state['mutate']()
        return (deepcopy(recorded_proposal),), dict(elapsed_sec=state['elapsed'],
            status='budget_exhausted', candidate_count=1, execution_authority=False)
    monkeypatch.setattr(proposals, 'generate_reachable_observation_views', generate)
    return NS(node=node, params=params, source=source, prepared=prepared, state=state,
        now=now, seed=list(node.latest_joint_state.position), proposal=recorded_proposal,
        reference=reference)


def move(c, count):
    c.node.latest_joint_state.position[0] = c.seed[0] + count*COUNT


@pytest.mark.parametrize('counts', [0., 1., -1.])
def test_generation_one_quantum_retains_endpoint_and_original_seed(seed_case, counts):
    c = seed_case
    c.state['mutate'] = lambda: move(c, counts)
    rows, report = c.node._joint_space_observation_candidates(c.prepared)
    assert rows, report
    assert report['current_joint_positions'] == c.seed
    assert report['joint_seed_stamp_ns'] == c.prepared.stamp.to_nsec()
    assert report['latest_joint_positions'] == c.node.latest_joint_state.position
    assert report['seed_handoff']['limit_rad'] == COUNT
    assert not report['seed_handoff']['execution_authority']
    assert rows[0]['q'] == c.proposal['q']
    assert rows[0]['T_base_tool0'] == c.proposal['T_base_tool0']
    assert rows[0]['endpoint_checks'] == c.proposal['endpoint_checks']
    rows2, report2 = c.node._joint_space_observation_candidates(c.prepared)
    assert rows2 and report2['cache_hit'] and len(c.state['calls']) == 1
    assert c.node._joint_space_observation_cache['search_seed'] == c.seed
    assert c.node._joint_space_observation_cache['used_sec'] == 2.


@pytest.mark.parametrize('counts', [1.00000001, -1.00000001, 2., -2., .004/COUNT])
def test_above_quantization_is_rejected_and_reason_persisted(seed_case, counts):
    c = seed_case
    c.state['mutate'] = lambda: move(c, counts)
    rows, report = c.node._joint_space_observation_candidates(c.prepared)
    assert not rows and report['status'] == 'inputs_changed'
    assert report['reason'] == 'joint seed changed beyond one SDK count'
    cache = c.node._joint_space_observation_cache
    assert cache['audit'] == report and not cache['candidates'] and cache['key'] is None
    assert cache['used_sec'] == 2. and cache['search_seed'] == c.seed
    assert report['generated_candidate_count'] == 1 and report['candidate_count'] == 0


def test_cache_anchor_does_not_ratchet_across_one_count_steps_or_revive(seed_case):
    c = seed_case
    assert c.node._joint_space_observation_candidates(c.prepared)[0]
    for count in (1., 0., -1., 0., 1.):
        move(c, count)
        rows, report = c.node._joint_space_observation_candidates(c.prepared)
        assert rows and report['cache_hit']
        assert report['current_joint_positions'] == c.seed
    move(c, 2.)
    rows, report = c.node._joint_space_observation_candidates(c.prepared)
    assert not rows and report['status'] == 'budget_exhausted'
    assert c.node._joint_space_observation_cache['audit'] == report
    move(c, 0.)
    assert not c.node._joint_space_observation_candidates(c.prepared)[0]
    assert len(c.state['calls']) == 1


@pytest.mark.parametrize('at', ['generation', 'cache'])
@pytest.mark.parametrize('fault', ['stale', 'future', 'nan', 'duplicate', 'missing'])
def test_feedback_invalidity_cannot_reuse_proposals(seed_case, at, fault):
    c = seed_case
    def corrupt():
        q = c.node.latest_joint_state
        if fault == 'stale': c.now[0] = q.header.stamp+base.rospy.Duration(.500001)
        elif fault == 'future': c.now[0] = q.header.stamp-base.rospy.Duration(.000001)
        elif fault == 'nan': q.position[0] = float('nan')
        elif fault == 'duplicate': q.name[1] = q.name[0]
        else: q.name[0] = 'other'
    if at == 'cache':
        assert c.node._joint_space_observation_candidates(c.prepared)[0]
        corrupt()
    else: c.state['mutate'] = corrupt
    rows, report = c.node._joint_space_observation_candidates(c.prepared)
    assert not rows and report['reason'] == 'fresh finite measured joint seed required'
    assert c.node._joint_space_observation_cache['audit'] == report
    assert not c.node._joint_space_observation_cache['candidates']


@pytest.mark.parametrize('fault', ['model', 'constraints', 'contract', 'stop', 'opening', 'gaps', 'uncertainty'])
def test_generation_configuration_recheck_cannot_keep_old_candidate(seed_case, fault):
    c = seed_case
    def mutate():
        if fault == 'model': c.params['/robot_description'] += '\n'
        elif fault == 'constraints': c.params['/alicia_controller/constraints']['Joint1']['goal'] = .04
        elif fault == 'contract': c.params['/robot/observation_tracking_contract_enabled'] = False
        elif fault == 'stop': c.params['/alicia_controller/stop_trajectory_duration'] = .7
        elif fault == 'opening': c.node.gripper_physical_open_width_m = .049
        elif fault == 'gaps': c.node._observe_return_view_gaps = ()
        else: c.node._observe_return_view_uncertainty_m = .004
    c.state['mutate'] = mutate
    rows, report = c.node._joint_space_observation_candidates(c.prepared)
    assert not rows and report['status'] == 'inputs_changed'
    assert c.node._joint_space_observation_cache['audit'] == report
    assert c.node._joint_space_observation_cache['used_sec'] == 2.


@pytest.mark.parametrize('at', ['generation', 'cache'])
@pytest.mark.parametrize('fault', ['epoch', 'manual', 'selection', 'session', 'supplement'])
def test_owner_change_remains_rejected_on_second_call_with_remaining_budget(seed_case, at, fault):
    c = seed_case; c.state['elapsed'] = .1
    def mutate():
        if fault == 'epoch': c.node._measurement_remote_epoch_ns += 1
        elif fault == 'manual': c.params['/gui/joint_direct_mode'] = True
        elif fault == 'selection': c.node._mode_selection['generation'] += 1
        elif fault == 'session': c.node._observe_return_session = {'session_id': 'changed'}
        else: c.node._observe_return_pending_supplement = {'view_index': 2}
    if at == 'cache':
        assert c.node._joint_space_observation_candidates(c.prepared)[0]
        mutate()
    else: c.state['mutate'] = mutate
    rows, report = c.node._joint_space_observation_candidates(c.prepared)
    assert not rows and report['status'] == 'inputs_changed'
    c.params['/gui/joint_direct_mode'] = False
    assert not c.node._joint_space_observation_candidates(c.prepared)[0]
    assert len(c.state['calls']) == 1
    assert c.node._joint_space_observation_cache['invalidated']


@pytest.mark.parametrize('fault', ['new_source', 'source_guard'])
def test_source_invalidation_stays_bound_to_prepared(seed_case, monkeypatch, fault):
    c = seed_case; c.state['elapsed'] = .1
    assert c.node._joint_space_observation_candidates(c.prepared)[0]
    def invalid(p):
        if fault == 'source_guard':
            raise base.CandidateContractError('UNKNOWN_OBSERVATION_SOURCE_INVALID', 'recorded source expired')
        return deepcopy(c.source)
    monkeypatch.setattr(c.node, '_require_diagnostic_observation_view_source', invalid)
    rows, report = c.node._joint_space_observation_candidates(c.prepared)
    assert not rows and c.node._joint_space_observation_cache['audit'] == report
    monkeypatch.setattr(c.node, '_require_diagnostic_observation_view_source', lambda p: c.source)
    assert not c.node._joint_space_observation_candidates(c.prepared)[0]
    assert len(c.state['calls']) == 1


@pytest.mark.parametrize('fault', ['epoch', 'two_count', 'source'])
def test_cache_hit_second_source_check_catches_interleaving(seed_case, monkeypatch, fault):
    c = seed_case; assert c.node._joint_space_observation_candidates(c.prepared)[0]
    calls = []
    def source(p):
        calls.append(True)
        if len(calls) == 2:
            if fault == 'epoch': c.node._measurement_remote_epoch_ns += 1
            elif fault == 'two_count': move(c, 2.)
            else: return deepcopy(c.source)
        return c.source
    monkeypatch.setattr(c.node, '_require_diagnostic_observation_view_source', source)
    rows, report = c.node._joint_space_observation_candidates(c.prepared)
    assert len(calls) == 2 and not rows and report['status'] == 'inputs_changed'
    assert c.node._joint_space_observation_cache['audit'] == report
    assert len(c.state['calls']) == 1


def test_changed_model_after_budget_cannot_hide_invalidation_or_regenerate(seed_case):
    c = seed_case
    assert c.node._joint_space_observation_candidates(c.prepared)[0]
    c.params['/robot_description'] += '\n'
    rows, report = c.node._joint_space_observation_candidates(c.prepared)
    assert not rows and report['status'] == 'budget_exhausted'
    assert report['changed_input_fields'] == ['model_sha256']
    assert c.node._joint_space_observation_cache['audit'] == report
    assert len(c.state['calls']) == 1


def test_quantization_bound_applies_to_each_joint_not_only_first(seed_case):
    c = seed_case
    def all_joints():
        c.node.latest_joint_state.position = [q+COUNT for q in c.seed]
    c.state['mutate'] = all_joints
    rows, report = c.node._joint_space_observation_candidates(c.prepared)
    assert rows and max(report['seed_handoff']['delta_rad']) == pytest.approx(COUNT)
    c.node.latest_joint_state.position[-1] += COUNT
    rows, report = c.node._joint_space_observation_candidates(c.prepared)
    assert not rows and c.node._joint_space_observation_cache['search_seed'] == c.seed
