"""Stage diagnostics cannot renew hint budgets or bypass request/source guards."""
from copy import deepcopy
from dataclasses import replace
import json
import math
from types import SimpleNamespace as NS

import pytest
from sensor_msgs.msg import JointState

from test_remote_modes import configured, base
from test_observation_endpoint_hints import setup_hints, variants_of
from test_observation_information_scheduler import scheduler_context
from alicia_grasp_modes import silhouette_observation_planning as planning


def timing_runtime(node, prepared):
    runtime = dict(prepared=prepared)
    node._active_unknown_observation_check_runtime = runtime
    return runtime


def assert_finite_timing(timing):
    json.dumps(timing, allow_nan=False)
    assert timing['execution_authority'] is False
    assert timing['active_stage'] is None
    for name in ('wall_sec', 'thread_cpu_sec'):
        assert timing[name] >= 0. and math.isfinite(timing[name])
        assert sum(row[name] for row in timing['stages'].values()) == pytest.approx(timing[name])
    assert all(row['entries'] >= 1 for row in timing['stages'].values())


def test_ordinary_hint_times_complete_prequery_path_and_retains_original_query_contract(
        configured, monkeypatch, tmp_path):
    node, prepared, variants, runtime = scheduler_context(configured, monkeypatch, tmp_path,
        [(0.01, .1), (.2, .8), (.3, .7)])
    monkeypatch.setattr(base.rospy.Time, 'now', staticmethod(lambda: base.rospy.Time(10)))
    state = JointState(name=['Joint%d' % i for i in range(1, 7)], position=[0.] * 6)
    state.header.stamp = base.rospy.Time(9, 900000000)
    node.latest_joint_state = state
    calls = []
    def service(request):
        calls.append(deepcopy(request))
        return NS(error_code=NS(val=-31))
    monkeypatch.setattr(base.rospy, 'wait_for_service', lambda *_args, **_kwargs: None)
    monkeypatch.setattr(base.rospy, 'ServiceProxy', lambda *_: service)
    hints, report = node._observation_endpoint_search_hints(tuple(variants), prepared)
    assert len(calls) == 3
    assert all(request.ik_request.avoid_collisions for request in calls)
    assert all(request.ik_request.group_name == 'alicia' and request.ik_request.ik_link_name == 'tool0'
        and request.ik_request.timeout.to_sec() == pytest.approx(.02) for request in calls)
    assert all(hint['status'] == 'unknown' for hint in hints.values())
    assert report['maximum_start_budget_sec'] == 3. and report['maximum_queries'] == 96
    timing = report['stage_timing']
    assert timing == runtime['observation_endpoint_hint_timing']
    assert timing['binding']['request_id'] == prepared.ticket.request_id
    assert timing['binding']['snapshot_stamp_ns'] == prepared.snapshot.stamp_ns
    assert timing['outcome'] == 'returned' and timing['query_start_count'] == 3
    assert timing['first_query_start_offset_sec'] >= 0.
    assert set(('entry_validation', 'initial_model_read_and_cache', 'joint_seed_validation',
        'shared_budget_setup', 'message_imports', 'service_discovery',
        'roll_distance_diversification', 'preloop_source_validation',
        'information_metrics_and_support_ordering', 'distance_direction_coverage',
        'station_family_ordering', 'pre_query_authority_validation',
        'query_construction_and_pose_cache', 'ik_service_call',
        'post_query_authority_validation', 'ik_response_validation',
        'shared_budget_accounting', 'final_authority_validation',
        'final_model_validation')).issubset(timing['stages'])
    assert_finite_timing(timing)
    assert not any(message.valid for message in node.rich_plan_pub.messages)


def test_preparation_exhaustion_still_spends_original_three_seconds_and_starts_no_query(
        configured, monkeypatch, tmp_path):
    node, prepared, variant, calls, clock, _ = setup_hints(configured, monkeypatch, tmp_path)
    runtime = timing_runtime(node, prepared)
    original = planning.diversified_observation_checks
    def delayed(variants, nominal):
        result = original(variants, nominal)
        clock[0] += 3.7
        return result
    monkeypatch.setattr(planning, 'diversified_observation_checks', delayed)
    hints, report = node._observation_endpoint_search_hints(variants_of(variant, 10), prepared)
    assert hints == {} and calls == []
    assert report['status'] == 'budget_exhausted' and report['checked_count'] == 0
    assert report['used_sec'] == pytest.approx(3.7)
    assert report['stage_timing']['query_start_count'] == 0
    assert report['stage_timing']['first_query_start_offset_sec'] is None
    assert 'roll_distance_diversification' in report['stage_timing']['stages']
    _, repeated = node._observation_endpoint_search_hints((variant,), prepared)
    assert calls == [] and repeated['used_sec'] == pytest.approx(3.7)
    assert runtime['observation_endpoint_hint_timing']['query_start_count'] == 0


@pytest.mark.parametrize('fault', ['context_after_query', 'model_after_query', 'source_before_query'])
def test_original_guard_exception_keeps_partial_stage_audit_and_never_returns_a_hint(
        configured, monkeypatch, tmp_path, fault):
    if fault == 'source_before_query':
        node, prepared, variants, runtime = scheduler_context(configured, monkeypatch, tmp_path,
            [(.01, .8)])
        monkeypatch.setattr(base.rospy.Time, 'now', staticmethod(lambda: base.rospy.Time(10)))
        state = JointState(name=['Joint%d' % i for i in range(1, 7)], position=[0.] * 6)
        state.header.stamp = base.rospy.Time(9, 900000000)
        node.latest_joint_state = state
        monkeypatch.setattr(node, '_require_diagnostic_observation_view_source',
            lambda *_: (_ for _ in ()).throw(ValueError('source changed')))
        monkeypatch.setattr(base.rospy, 'wait_for_service', lambda *_args, **_kwargs: None)
        # Constructing a proxy is permitted; only invoking it would cross the guard.
        monkeypatch.setattr(base.rospy, 'ServiceProxy', lambda *_: lambda _request: pytest.fail('query before source guard'))
        expected_stage = 'preloop_source_validation'
    else:
        node, prepared, variant, _, clock, _ = setup_hints(configured, monkeypatch, tmp_path)
        runtime = timing_runtime(node, prepared)
        variants = (variant,)
        def service(request):
            clock[0] += .02
            if fault == 'model_after_query':
                configured[1]['/robot_description'] = 'changed-model'
            else:
                node._observe_return_paused = True
            return NS(error_code=NS(val=1), solution=NS(joint_state=node.latest_joint_state))
        monkeypatch.setattr(base.rospy, 'ServiceProxy', lambda *_: service)
        expected_stage = 'final_model_validation' if fault == 'model_after_query' else 'post_query_authority_validation'
    with pytest.raises((ValueError, base.CandidateContractError, base.StreamResultCancelled)):
        node._observation_endpoint_search_hints(tuple(variants), prepared)
    timing = runtime['observation_endpoint_hint_timing']
    assert timing['outcome'] == 'raised' and timing['exception_type']
    assert expected_stage in timing['stages']
    assert_finite_timing(timing)
    assert not any(message.valid for message in node.rich_plan_pub.messages)


@pytest.mark.parametrize('bad_stage', ['entry_validation', 'service_discovery',
    'ik_service_call', 'shared_budget_accounting', 'final_model_validation', None])
def test_diagnostic_clock_failure_cannot_be_swallowed_as_an_ik_transport_result(
        configured, monkeypatch, tmp_path, bad_stage):
    node, prepared, variant, calls, _, _ = setup_hints(configured, monkeypatch, tmp_path)
    runtime = timing_runtime(node, prepared)
    original = planning._ObservationHintTiming.switch
    def fault(self, stage):
        if stage == bad_stage:
            monkeypatch.setattr(planning.time, 'thread_time', lambda: float('nan'))
        return original(self, stage)
    monkeypatch.setattr(planning._ObservationHintTiming, 'switch', fault)
    with pytest.raises(ValueError, match='diagnostic clock'):
        node._observation_endpoint_search_hints((variant,), prepared)
    timing = runtime['observation_endpoint_hint_timing']
    assert 'timing_error' in timing
    assert timing['outcome'] == 'raised'
    json.dumps(timing, allow_nan=False)
    if bad_stage in ('entry_validation', 'service_discovery', 'ik_service_call'):
        assert calls == []
    else:
        assert len(calls) == 1
        assert node._unknown_observation_endpoint_hint_cache['used_sec'] == pytest.approx(.02)


def test_timing_report_does_not_attach_to_another_prepared_owner(configured, monkeypatch, tmp_path):
    node, prepared, variant, _, _, _ = setup_hints(configured, monkeypatch, tmp_path)
    runtime = timing_runtime(node, replace(prepared))
    _, report = node._observation_endpoint_search_hints((variant,), prepared)
    assert report['stage_timing']['query_start_count'] == 1
    assert 'observation_endpoint_hint_timing' not in runtime


def test_wall_and_thread_cpu_are_distinct_without_inventing_a_wait_cause(monkeypatch):
    wall, cpu = [10.], [1.]
    monkeypatch.setattr(planning.time, 'monotonic', lambda: wall[0])
    monkeypatch.setattr(planning.time, 'thread_time', lambda: cpu[0])
    timing = planning._ObservationHintTiming(dict(request_id=1))
    timing.switch('message_imports')
    wall[0] += 3.; cpu[0] += .2
    timing.switch(None)
    assert timing.report['stages']['message_imports']['wall_sec'] == 3.
    assert timing.report['stages']['message_imports']['thread_cpu_sec'] == pytest.approx(.2)
    assert timing.report['wall_minus_cpu_is_not_a_specific_wait_cause'] is True
    assert_finite_timing(timing.report)
