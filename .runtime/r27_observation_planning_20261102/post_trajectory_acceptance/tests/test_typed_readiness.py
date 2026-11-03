"""Pure archived-message and fake-clock checks; no ROS connection or QUERY."""
from copy import deepcopy
from dataclasses import asdict
import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS
import sys

import pytest

W = Path(__file__).resolve().parents[4]
B = Path('/home/zhuyupei/ssd_data/实验记录/Robot-Controller')
M = B/'maintenance/r27_observation_planning_20261102'
E = M/'new_near_field_failure_20261102/typed_attempt1_terminal_review_r1'


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


CHECK = load('post_trajectory_typed_readiness_test', Path(__file__).resolve().parents[1]/'typed_readiness.py')
STARTUP = load('unchanged_typed_startup_for_post_trajectory_test', W/'tools/diagnostics/verify_physical_reference_startup.py')


def read(path, digest):
    raw = Path(path).read_bytes()
    assert hashlib.sha256(raw).hexdigest() == digest
    return json.loads(raw)


@pytest.fixture
def actual():
    history = read(E/'recorded_typed_transitions.json',
        '28195d9f9e7fc438e249a77d5f40d2770289f9f3adfbb85c9b0c81c9620a8702')
    actions = read(E/'recorded_action_goal_terminal_r1.json',
        'd33c07c03533bef2285a17fa3f7b7c866bb86a99a4e58dd901c373cc99d23806')
    snapshot = read(B/'ros_log/full_latest_r26_manual_20261102_r1/r27_minimum_placement_20261103_r2/post_attempt_typed_readonly.json',
        '3fe198949bd645f389c4b2469e301260860957ae0b01873c85b3dec8d23e6dfa')
    revision = read(M/'remote_only_prepare_r5_final/revision.json',
        'b18f5f04a17b3d7bf8f3ecbe897fa67dddfe52638595ca9db14b51b5a2c620ec')
    assets = read(revision['typed_assets']['path'], revision['typed_assets']['sha256'])
    binding = CHECK.validate_transport_history(history, actions)
    return NS(history=history, actions=actions, snapshot=snapshot, assets=assets, binding=binding)


def validate(f):
    return CHECK.validate_recorded_snapshot(f.snapshot, f.binding, startup=STARTUP, assets=f.assets)


def test_actual_history_and_manual_snapshot_reproduce_only_startup_sequence_mismatch(actual):
    f = actual
    untouched = deepcopy(f.snapshot)
    ready = validate(f)
    assert ready['historical_diagnostic_only'] and not ready['current_readiness_claimed']
    assert not ready['initial_startup_claimed']
    assert asdict(f.binding)['action_transport_sequence'] == 1
    assert f.binding.connection_callerids_available is False
    assert f.binding.record_epoch_ns < f.snapshot['state']['epoch_ns']
    assert ready['states']['driver']['physical_reference_rad'] != ready['states']['driver']['wire_positions_rad']
    assert ready['states']['driver']['physical_reference_rad'] != ready['accepted_positions_rad']
    assert f.snapshot == untouched
    rows = CHECK.recorded_rows(f.snapshot)
    rospy = NS(Time=NS(now=lambda:CHECK._Stamp(f.snapshot['finished_ns'])))
    with pytest.raises(RuntimeError, match='initial typed binding is not empty LEGACY'):
        STARTUP.observed_ready(NS(snapshot=lambda:rows), f.snapshot['state']['epoch_ns'], rospy, f.assets)


@pytest.mark.parametrize('sequence', [23, 0xffffffff])
def test_transport_sequence_is_bound_from_records_not_a_hardcoded_one_or_count(actual, sequence):
    f = actual
    for row in f.actions['records']:
        if row['topic'] == CHECK.GOAL_TOPIC:
            row['message']['header']['seq'] = sequence
    for topic in (CHECK.DRIVER_TOPIC, CHECK.HW_TOPIC):
        for transition in f.history['typed_transitions'][topic]:
            for edge in ('first', 'last'):
                if transition[edge]['message']['action_transport_sequence']:
                    transition[edge]['message']['action_transport_sequence'] = sequence
        f.history['last_messages'][topic]['message']['action_transport_sequence'] = sequence
    f.binding = CHECK.validate_transport_history(f.history, f.actions)
    for row in f.snapshot['typed'].values():
        row['message']['action_transport_sequence'] = sequence
    assert validate(f)['transport_history']['action_transport_sequence'] == sequence
    assert f.binding.recorded_goal_count == 1


@pytest.mark.parametrize('fault', ['goal_sequence', 'result_id', 'result_stamp', 'result_status',
    'result_error', 'missing_terminal', 'wrong_bag', 'wrong_md5', 'foreign_caller',
    'endpoint_sequence', 'typed_binding', 'history_epoch', 'unclosed_source'])
def test_unproven_or_unsuccessful_action_history_is_rejected(actual, fault):
    f = actual
    goal = next(r for r in f.actions['records'] if r['topic'] == CHECK.GOAL_TOPIC)
    result = next(r for r in f.actions['records'] if r['topic'] == CHECK.RESULT_TOPIC)
    if fault == 'goal_sequence': goal['message']['header']['seq'] += 1
    elif fault == 'result_id': result['message']['status']['goal_id']['id'] += '-foreign'
    elif fault == 'result_stamp': result['message']['status']['goal_id']['stamp'] += 1
    elif fault == 'result_status': result['message']['status']['status'] = 4
    elif fault == 'result_error': result['message']['result']['error_code'] = -1
    elif fault == 'missing_terminal': f.actions['records'].remove(result)
    elif fault == 'wrong_bag': f.actions['bag_sha256'] = '0'*64
    elif fault == 'wrong_md5': goal['connection']['md5sum'] = '0'*32
    elif fault == 'foreign_caller': goal['connection']['callerid'] = '/foreign'
    elif fault == 'endpoint_sequence': f.history['last_messages'][CHECK.HW_TOPIC]['message']['action_transport_sequence'] += 1
    elif fault == 'typed_binding': f.history['last_messages'][CHECK.HW_TOPIC]['message']['binding_id'] = 'lease'
    elif fault == 'history_epoch': f.history['last_messages'][CHECK.HW_TOPIC]['message']['epoch_ns'] += 1
    else: f.actions['source_record_unchanged'] = False
    with pytest.raises(RuntimeError): CHECK.validate_transport_history(f.history, f.actions)


@pytest.mark.parametrize('field', CHECK.EMPTY_FIELDS)
def test_every_original_empty_binding_field_still_rejects(actual, field):
    actual.snapshot['typed']['driver']['message'][field] = 'not-empty'
    with pytest.raises(RuntimeError): validate(actual)


@pytest.mark.parametrize('field', [f for f in CHECK.ZERO_FIELDS if f != 'action_transport_sequence'])
def test_every_other_original_zero_field_still_rejects(actual, field):
    actual.snapshot['typed']['hardware']['message'][field] = 1
    with pytest.raises(RuntimeError): validate(actual)


@pytest.mark.parametrize('fault', ['sequence', 'boolean_sequence', 'phase', 'code', 'bias',
    'goal_observed', 'goal_terminal', 'output_gated', 'evidence_valid', 'pending_complete',
    'stationary', 'manual_reference', 'owner', 'latch', 'md5', 'state_epoch',
    'old_current_epoch', 'old_now', 'wire_decode', 'sdk_counts', 'physical_reference'])
def test_current_typed_state_keeps_all_original_gates(actual, fault):
    f = actual; row = f.snapshot['typed']['driver']; s = row['message']
    if fault == 'sequence': s['action_transport_sequence'] += 1
    elif fault == 'boolean_sequence': s['action_transport_sequence'] = True
    elif fault == 'phase': s['phase'] = 2
    elif fault == 'code': s['code'] = 'REFERENCE_RELEASED_SAME_WIRE'
    elif fault == 'bias': s['bias_rad'][0] = .001
    elif fault in ('goal_observed', 'goal_terminal', 'output_gated'): s[fault] = True
    elif fault in ('evidence_valid', 'pending_complete', 'stationary', 'manual_reference'): s[fault] = False
    elif fault == 'owner': row['transport']['callerid'] = '/foreign'
    elif fault == 'latch': row['transport']['latching'] = '1'
    elif fault == 'md5': row['transport']['md5sum'] = '0'*32
    elif fault == 'state_epoch': s['epoch_ns'] += 1
    elif fault == 'old_current_epoch': f.snapshot['state']['epoch_ns'] = f.binding.record_epoch_ns-1
    elif fault == 'old_now': f.snapshot['finished_ns'] += 500_000_001
    elif fault == 'wire_decode': s['wire_positions_rad'][0] += .001
    elif fault == 'sdk_counts': s['sdk_counts'][0] += 1
    else: s['physical_reference_rad'][0] += .001
    with pytest.raises(RuntimeError): validate(f)


@pytest.mark.parametrize('field', ['sdk_stamp_ns', 'encoder_stamp_ns', 'controller_stamp_ns'])
def test_each_exact_typed_raw_source_must_be_in_current_window(actual, field):
    actual.snapshot['typed']['hardware']['message'][field] -= 1
    with pytest.raises(RuntimeError, match='unseen'): validate(actual)


@pytest.mark.parametrize('fault', ['sdk_owner', 'encoder_frame', 'reference_mode',
    'reference_latch', 'reference_stamp', 'sdk_window', 'encoder_window', 'controller_velocity',
    'controller_acceleration', 'controller_joint_layout'])
def test_actual_raw_feedback_and_jtc_guards_remain_required(actual, fault):
    f = actual; raw = f.snapshot['raw']
    if fault == 'sdk_owner': raw['sdk'][-1]['transport']['callerid'] = '/foreign'
    elif fault == 'encoder_frame': raw['encoder'][-1]['message']['header']['frame_id'] = 'synthetic'
    elif fault == 'reference_mode': raw['reference'][-1]['message']['header']['frame_id'] = 'sdk_automatic'
    elif fault == 'reference_latch': raw['reference'][-1]['transport']['latching'] = '1'
    elif fault == 'reference_stamp': raw['reference'][-1]['message']['header']['stamp']['nsecs'] -= 1
    elif fault == 'sdk_window': raw['sdk'] = raw['sdk'][-2:]
    elif fault == 'encoder_window': raw['encoder'] = raw['encoder'][-2:]
    else:
        for row in raw['controller']:
            if fault == 'controller_velocity': row['message']['desired']['velocities'][0] = .001
            elif fault == 'controller_acceleration': row['message']['desired']['accelerations'][0] = .001
            else: row['message']['joint_names'][0] = 'foreign'
    with pytest.raises(RuntimeError): validate(f)


class Clock:
    def __init__(self, now):
        self.now, self.mono, self.sleeps, self.mode = now, 0., [], 'normal'
    def monotonic(self): return self.mono
    def sleep(self, seconds):
        assert 0 < seconds <= .01
        self.sleeps.append(seconds); self.mono += seconds
        if self.mode == 'normal': self.now += round(seconds*1e9)
        elif self.mode == 'backward': self.now -= 1
        elif self.mode == 'forward': self.now += 1_000_000_000


def fixed_wait(f, monkeypatch, *, delay=.02, fault=None):
    rows = CHECK.recorded_rows(f.snapshot); query = {k:deepcopy(rows[k][-1]) for k in ('driver', 'hardware')}
    clock = Clock(f.snapshot['finished_ns']); clock.mode = fault if fault in ('backward', 'forward', 'stalled_ros') else 'normal'
    monkeypatch.setattr(CHECK, 'time', clock)
    original = query['driver'].sdk_stamp_ns
    identities = {key:id(value) for key,value in query.items()}
    def snapshot():
        if fault == 'receipt_changed' and clock.mono:
            query['driver'].sdk_stamp_ns += 1
        available = delay is not None and clock.mono >= delay
        if available: return rows
        return dict(rows, sdk=[r for r in rows['sdk'] if r.header.stamp.to_nsec() != original])
    rospy = NS(Time=NS(now=lambda:CHECK._Stamp(clock.now)))
    audit = {}
    answer = CHECK.wait_for_fixed_query_sources(NS(snapshot=snapshot), f.snapshot['state']['epoch_ns'],
        rospy, f.assets, f.binding, query, audit, startup=STARTUP)
    assert {key:id(value) for key,value in query.items()} == identities
    return answer, audit, clock, query


@pytest.mark.parametrize('delay', [0., .02])
def test_fixed_query_sources_use_same_receipts_and_original_remaining_500ms_window(actual, monkeypatch, delay):
    answer, audit, clock, query = fixed_wait(actual, monkeypatch, delay=delay)
    stamps = [v for row in audit['fixed_query_stamps'].values() for v in row.values()]
    assert audit['expires_ros_ns'] == min(stamps)+500_000_000
    assert audit['completed'] and not audit['query_reissued'] and not audit['receipt_refreshed']
    assert bool(clock.sleeps) == bool(delay)
    assert answer['query_states']['driver']['action_transport_sequence'] == actual.binding.action_transport_sequence
    assert query['driver'].sdk_stamp_ns == audit['fixed_query_stamps']['driver']['sdk_stamp_ns']


@pytest.mark.parametrize('fault', ['missing', 'backward', 'forward', 'stalled_ros', 'receipt_changed'])
def test_fixed_query_never_renews_receipt_or_wait_budget(actual, monkeypatch, fault):
    with pytest.raises(RuntimeError):
        fixed_wait(actual, monkeypatch, delay=None, fault=fault)


def test_fixed_query_requires_both_original_endpoints(actual, monkeypatch):
    rows = CHECK.recorded_rows(actual.snapshot)
    query = {'driver':rows['driver'][-1]}
    rospy = NS(Time=NS(now=lambda:CHECK._Stamp(actual.snapshot['finished_ns'])))
    with pytest.raises(RuntimeError, match='both fixed QUERY'):
        CHECK.wait_for_fixed_query_sources(NS(snapshot=lambda:rows), actual.snapshot['state']['epoch_ns'],
            rospy, actual.assets, actual.binding, query, {}, startup=STARTUP)
