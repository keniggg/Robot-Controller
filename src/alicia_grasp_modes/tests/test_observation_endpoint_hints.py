"""Budgeted IK hints may change work order, never strict motion authority."""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace as NS

import numpy as np
import pytest
from sensor_msgs.msg import JointState

from test_remote_modes import configured, base, module
from test_silhouette_observation_planning import context
from test_silhouette_observation_order import transform
from alicia_grasp_modes.silhouette_observation_order import transform_fingerprint


def setup_hints(configured, monkeypatch, tmp_path):
    node, prepared, strict_calls, variant = context(configured, monkeypatch, tmp_path)
    configured[1]['/robot_description'] = 'frozen-test-model'
    monkeypatch.setattr(base.rospy.Time, 'now', staticmethod(lambda: base.rospy.Time(10)))
    state = JointState(name=['Joint%d' % i for i in range(1, 7)], position=[0.] * 6)
    state.header.stamp = base.rospy.Time(9, 900000000)
    node.latest_joint_state = state
    clock = [100.]
    node._observation_endpoint_hint_clock = lambda: clock[0]
    calls = []
    def service(request):
        calls.append(deepcopy(request))
        clock[0] += .02
        solved = deepcopy(state); solved.position = [.1] * 6
        return NS(error_code=NS(val=1), solution=NS(joint_state=solved))
    def proxy(name, _kind):
        assert name == '/compute_ik'
        return service
    monkeypatch.setattr(base.rospy, 'wait_for_service', lambda name, timeout: None)
    monkeypatch.setattr(base.rospy, 'ServiceProxy', proxy)
    return node, prepared, variant, calls, clock, strict_calls


def variants_of(variant, count):
    variants = []
    for i in range(count):
        v = deepcopy(variant)
        v['sequence'].pregrasp.pose.position.x += i * .001
        variants.append(v)
    return variants


def test_only_readonly_collision_ik_and_same_request_exact_pose_cache(configured, monkeypatch, tmp_path):
    node, prepared, variant, calls, _, strict = setup_hints(configured, monkeypatch, tmp_path)
    hint, report = node._observation_endpoint_search_hints((variant,), prepared)
    assert len(calls) == 1 and not strict
    request = calls[0].ik_request
    assert request.group_name == 'alicia' and request.ik_link_name == 'tool0'
    assert request.avoid_collisions and request.timeout.to_sec() == pytest.approx(.02)
    assert request.pose_stamped == variant['sequence'].pregrasp
    assert next(iter(hint.values()))['status'] == 'endpoint_found'
    assert not report['execution_authority'] and report['candidates_removed'] == 0
    node._observation_endpoint_search_hints((deepcopy(variant),), prepared)
    assert len(calls) == 1
    assert 'prepared' not in node._unknown_observation_endpoint_hint_cache
    # A new immutable request cannot consume the previous request's hints.
    fresh = replace(prepared, ticket=replace(prepared.ticket, request_id=prepared.ticket.request_id + 1))
    node._observation_endpoint_search_hints((variant,), fresh)
    assert len(calls) == 2


@pytest.mark.parametrize('fault', ['no_solution', 'nan', 'missing_joint', 'duplicate_joint', 'missing_solution', 'invalid_status'])
def test_bad_response_is_unknown_not_unreachable(configured, monkeypatch, tmp_path, fault):
    node, prepared, variant, _, _, _ = setup_hints(configured, monkeypatch, tmp_path)
    solved = deepcopy(node.latest_joint_state)
    response = NS(error_code=NS(val=1), solution=NS(joint_state=solved))
    if fault == 'no_solution': response.error_code.val = -31
    elif fault == 'nan': solved.position[2] = np.nan
    elif fault == 'missing_joint': solved.name[2] = 'another_joint'
    elif fault == 'duplicate_joint': solved.name[2] = solved.name[1]
    elif fault == 'missing_solution': response.solution = None
    else: response.error_code = None
    monkeypatch.setattr(base.rospy, 'ServiceProxy', lambda *args: lambda request: response)
    hints, report = node._observation_endpoint_search_hints((variant,), prepared)
    assert next(iter(hints.values()))['status'] == 'unknown'
    assert report['hinted_count'] == 0 and report['candidates_removed'] == 0


@pytest.mark.parametrize('limit', ['count', 'elapsed'])
def test_budget_shared_across_candidates_and_no_retry_after_exhaustion(configured, monkeypatch, tmp_path, limit):
    node, prepared, variant, calls, clock, _ = setup_hints(configured, monkeypatch, tmp_path)
    def service(request):
        calls.append(request)
        clock[0] += 1.1 if limit == 'elapsed' else .001
        return NS(error_code=NS(val=-31))
    monkeypatch.setattr(base.rospy, 'ServiceProxy', lambda *args: service)
    variants = variants_of(variant, 110)
    _, report = node._observation_endpoint_search_hints(variants, prepared)
    expected = 3 if limit == 'elapsed' else 96
    assert len(calls) == expected and report['status'] == 'budget_exhausted'
    node._observation_endpoint_search_hints(tuple(reversed(variants)), prepared)
    assert len(calls) == expected


def test_transport_failure_does_not_repeat_per_pose_or_candidate(configured, monkeypatch, tmp_path):
    node, prepared, variant, calls, _, _ = setup_hints(configured, monkeypatch, tmp_path)
    def service(request):
        calls.append(request)
        raise RuntimeError('IK unavailable')
    monkeypatch.setattr(base.rospy, 'ServiceProxy', lambda *args: service)
    hints, report = node._observation_endpoint_search_hints(variants_of(variant, 5), prepared)
    node._observation_endpoint_search_hints((variant,), prepared)
    assert len(calls) == 1 and report['status'] == 'unavailable'
    assert report['hinted_count'] == 0


@pytest.mark.parametrize('fault', ['stale', 'future', 'no_model', 'duplicate_seed'])
def test_bad_seed_never_calls_ik(configured, monkeypatch, tmp_path, fault):
    node, prepared, variant, calls, _, _ = setup_hints(configured, monkeypatch, tmp_path)
    if fault == 'stale': node.latest_joint_state.header.stamp = base.rospy.Time(9)
    elif fault == 'future': node.latest_joint_state.header.stamp = base.rospy.Time(11)
    elif fault == 'no_model': configured[1]['/robot_description'] = ''
    else: node.latest_joint_state.name[1] = node.latest_joint_state.name[0]
    hints, report = node._observation_endpoint_search_hints((variant,), prepared)
    assert hints == {} and calls == [] and report['status'] == 'unavailable'


@pytest.mark.parametrize('fault', ['context', 'model'])
def test_invalidation_during_ik_cannot_return_a_hint(configured, monkeypatch, tmp_path, fault):
    node, prepared, variant, _, _, _ = setup_hints(configured, monkeypatch, tmp_path)
    def service(request):
        if fault == 'model': configured[1]['/robot_description'] = 'changed'
        else: node._observe_return_paused = True
        return NS(error_code=NS(val=1), solution=NS(joint_state=node.latest_joint_state))
    monkeypatch.setattr(base.rospy, 'ServiceProxy', lambda *args: service)
    with pytest.raises((ValueError, base.CandidateContractError, base.StreamResultCancelled)):
        node._observation_endpoint_search_hints((variant,), prepared)


def test_hint_prioritizes_useful_candidate_but_strict_failure_still_rejects(configured, monkeypatch, tmp_path):
    node, prepared, variant, _, _, strict = setup_hints(configured, monkeypatch, tmp_path)
    variants = variants_of(variant, 5)
    reference = base.pose_matrix(variant['sequence'].pregrasp)
    for i, v in enumerate(variants):
        matrix = reference @ transform(.1 * i, (.01 * i, 0., 0.))
        v['sequence'].pregrasp = base.make_pose_stamped('base_link', matrix[:3, 3],
            base.quaternion_from_matrix(matrix), stamp=prepared.stamp)
    monkeypatch.setattr(node, '_far_field_observation_variants', lambda *_: (tuple(variants), ()))
    fingerprint = transform_fingerprint(base.pose_matrix(variants[-1]['sequence'].pregrasp))
    monkeypatch.setattr(node, '_observation_endpoint_search_hints', lambda *args: (
        {fingerprint: dict(status='endpoint_found', joint_max_delta_rad=.1, joint_l1_delta_rad=.2)},
        dict(execution_authority=False, candidates_removed=0)))
    checked = []
    def reject(target, observation=False):
        checked.append(target)
        return base.MoveItResult(False, 0., 0., 'collision', failure_code='MOVEIT_COLLISION'), {}, {}
    monkeypatch.setattr(node, '_strict_moveit_evaluation', reject)
    result = node._accept_prediction(prepared)
    assert result['status'] == 'SILHOUETTE_OBSERVATION_NO_SAFE_VIEW'
    assert checked[0] is variants[-1]['sequence'].pregrasp
    assert {id(p) for p in checked} == {id(v['sequence'].pregrasp) for v in variants}
    assert not getattr(node, 'latest_rich_plan', None)
