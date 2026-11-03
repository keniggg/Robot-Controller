"""Post-first-path comparison uses the real source-guarded strict loop.

Transport/path responses are fixtures, not physical feasibility evidence.
"""
from dataclasses import replace

import pytest

from test_observation_fk_first_strict import loop_context, configured, base


@pytest.mark.parametrize('later_better', [True, False])
def test_first_success_keeps_comparing_ranked_views_without_reordering_or_hints(
        configured, monkeypatch, tmp_path, later_better):
    node, _, runtime, rows, candidate, checker, events, hints, _ = loop_context(
        configured, monkeypatch, tmp_path)
    # Freeze an initial order whose first same-stratum FK is index1. Adjust
    # fixture utility only at the original strict response: this isolates
    # checked-path selection from endpoint pre-ordering.
    strict = base.RemoteGrasp6DNode._strict_moveit_evaluation
    order = node._order_unknown_observation_checks
    order_calls = []

    def once(variants, **kwargs):
        order_calls.append((tuple(v['preference_index'] for v in variants), kwargs))
        return order(variants, **kwargs)

    def evaluated(self, target, observation=False):
        index = next(i for i, v in enumerate(rows) if v['sequence'].pregrasp is target)
        result, evidence, extra = strict(self, target, observation=observation)
        utility = .8 if (index == 2 and later_better) or (index == 1 and not later_better) else .2
        rows[index]['observation_view']['information_utility']['utility'] = utility
        duration = 3. if index == 2 else 1.
        result = replace(result, reason=result.reason.replace(
            'observation_screened_duration_sec=2.0',
            'observation_screened_duration_sec=%s' % duration))
        return result, evidence, extra

    monkeypatch.setattr(node, '_order_unknown_observation_checks', once)
    monkeypatch.setattr(base.RemoteGrasp6DNode, '_strict_moveit_evaluation', evaluated)
    assert checker(candidate).reachable
    assert [index for event, index in events if event == 'strict'] == [1, 0, 2]
    assert order_calls == [((0, 1, 2), {'defer_endpoint_hints': True})]
    assert not hints
    assert runtime['observation_sequence'] is rows[2 if later_better else 1]['sequence']
    assert runtime['observation_orientation_search']['checked_variant_count'] == 3


@pytest.mark.parametrize('call_cost, expected_comparisons', [(.1, 6), (4., 2)])
def test_first_success_uses_existing_count_and_start_deadline_without_reset(
        configured, monkeypatch, tmp_path, call_cost, expected_comparisons):
    node, prepared, runtime, _, candidate, checker, events, hints, clock = loop_context(
        configured, monkeypatch, tmp_path, count=20)
    following = base.RemoteGrasp6DNode._observation_following_support_evaluation

    def timed(self, current, result, target=None):
        clock[0] += call_cost - .1
        return following(self, current, result, target)

    monkeypatch.setattr(base.RemoteGrasp6DNode, '_observation_following_support_evaluation', timed)
    assert checker(candidate).reachable
    checked = [i for kind, i in events if kind == 'strict']
    assert checked[0] == 1 and checked.count(1) == 1
    assert len(checked) == len(set(checked)) == 1 + expected_comparisons
    assert not hints
    budget = node._observation_tilt_search_budget
    assert budget['discovery_checked_count'] == budget['comparison_start_count'] == 1
    assert budget['checked_count'] - budget['comparison_start_count'] == expected_comparisons
    assert budget['comparison_start_used_sec'] == pytest.approx(call_cost)
    assert budget['used_sec'] == pytest.approx(call_cost * len(checked))
    assert node._observation_information_comparison['deadline'] == pytest.approx(call_cost + 8.)
    assert node._observation_information_comparison['ticket'] is prepared.ticket
    audit = runtime['observation_orientation_search']
    assert audit['comparison_budget_reached'] and not audit['search_complete']
    assert audit['unchecked_variant_count'] == 20 - len(checked)
    assert base.OBSERVATION_TILT_MAX_STRICT_CHECKS == 6
    assert base.OBSERVATION_COMPARISON_AFTER_INCUMBENT_SEC == 8.


@pytest.mark.parametrize('failure', ['strict', 'view', 'following', 'arrival_reserve'])
def test_useful_later_view_cannot_replace_incumbent_without_every_original_gate(
        configured, monkeypatch, tmp_path, failure):
    node, _, runtime, rows, candidate, checker, events, hints, _ = loop_context(
        configured, monkeypatch, tmp_path)
    strict = base.RemoteGrasp6DNode._strict_moveit_evaluation
    following = base.RemoteGrasp6DNode._observation_following_support_evaluation

    def evaluated(self, target, observation=False):
        index = next(i for i, v in enumerate(rows) if v['sequence'].pregrasp is target)
        result, evidence, extra = strict(self, target, observation=observation)
        rows[index]['observation_view']['information_utility']['utility'] = .9 if index == 2 else (.5 if index == 1 else .1)
        if index == 2 and failure == 'strict':
            result = replace(result, reachable=False, failure_code='MOVEIT_COLLISION')
        if index == 2 and failure == 'view':
            result = replace(result, reason='missing original full-path proof')
        return result, evidence, extra

    def followed(self, current, result, target=None):
        result, evidence = following(self, current, result, target)
        if target is rows[2]['sequence'].pregrasp and failure == 'following':
            return replace(result, reachable=False,
                failure_code='OBSERVATION_FOLLOWING_SUPPORT_INVALID'), {'ok': False}
        return result, evidence

    monkeypatch.setattr(base.RemoteGrasp6DNode, '_strict_moveit_evaluation', evaluated)
    monkeypatch.setattr(base.RemoteGrasp6DNode, '_observation_following_support_evaluation', followed)
    if failure == 'arrival_reserve':
        monkeypatch.setattr(node,'_observation_arrival_reserve_evaluation',
            lambda *a:dict(ok=events[-1] != ('following',2),scope='scheduling_test_fixture'))
    assert checker(candidate).reachable
    assert runtime['observation_sequence'] is rows[1]['sequence']
    assert [i for kind, i in events if kind == 'strict'] == [1, 0, 2]
    assert not hints
    assert runtime['observation_orientation_search']['reachable_variant_count'] == 2


@pytest.mark.parametrize('invalidation', ['source_expired', 'ticket_revoked'])
def test_first_success_does_not_start_comparison_or_promote_after_invalidation(
        configured, monkeypatch, tmp_path, invalidation):
    node, _, runtime, _, candidate, checker, events, hints, _ = loop_context(
        configured, monkeypatch, tmp_path)
    current = node._observation_search_prepared_current

    def guarded(prepared):
        # The base sets the incumbent only after commit and every full-path
        # gate. Invalidate at its next source check before a second dispatch.
        if runtime.get('_observation_qualified_incumbent_ticket') is prepared.ticket:
            if invalidation == 'source_expired':
                return False
            node._advance_target_instance_epoch('TEST_COMPARISON_REVOKED', preserve_identity=False)
        return current(prepared)

    monkeypatch.setattr(node, '_observation_search_prepared_current', guarded)
    if invalidation == 'ticket_revoked':
        with pytest.raises(base.StreamResultCancelled):
            checker(candidate)
    else:
        result = checker(candidate)
        assert not result.reachable
        assert runtime['_observation_qualified_incumbent_ticket'] is None
        assert runtime['observation_orientation_search']['stop_reason'] == 'OBSERVATION_SOURCE_RESERVE_REACHED'
    assert [i for kind, i in events if kind == 'strict'] == [1]
    assert not hints
    assert not any(message.valid for message in node.rich_plan_pub.messages)
