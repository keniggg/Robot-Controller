"""Non-authorizing diagnostics around the original station cancellation path.

The real GEN41 remote callbacks/producer fixture supplies ordinary source joins
and an unpublished ticket. Fault inputs below are explicit test providers, not a
claim to reproduce G45's unrecorded subscriber scheduling.
"""
from copy import deepcopy
import json
import threading
from types import SimpleNamespace as NS

import pytest

from test_gen41_station_private_pending import work
from test_observation_measurement_remote import context
from test_station_compute_wait_remote import station, row_at, clock, feedback
from test_station_compute_work_handoff import next_row
from test_unknown_pending_identity import deliver
from alicia_grasp_modes import station_compute_wait_remote as remote
from alicia_grasp_modes import station_handoff_diagnostics as diagnostic


def _logged_retirement(monkeypatch, node):
    events = []
    reports = []
    retire = node._station_compute_retire

    def original_retire(reason):
        events.append(('retire_start', reason))
        result = retire(reason)
        events.append(('retire_done', reason))
        return result

    def log(message, *args):
        if message == 'Station handoff diagnostic: %s':
            events.append(('diagnostic_log', None))
            reports.append(json.loads(args[0]))

    monkeypatch.setattr(node, '_station_compute_retire', original_retire)
    monkeypatch.setattr(remote.rospy, 'logwarn', log)
    return events, reports


def _assert_original_cancel(node, ticket, events, reports):
    assert node._station_compute_retired
    assert node._station_compute_reference is None
    assert node._station_compute_work == {}
    assert not node.latest_object_geometry.valid
    # Original observe-return retirement clears the local preview cache. Its
    # already-published preview tombstone remains non-executable; the original
    # execution channel also publishes the new TARGET_LOST invalidation.
    assert node.latest_preview_rich_plan is None
    assert node.preview_rich_plan_pub.messages[-1].valid is False
    assert node.preview_plan_pub.messages[-1].poses == []
    assert node.rich_plan_pub.messages[-1].valid is False
    assert node.rich_plan_pub.messages[-1].diagnostic == (
        'TARGET_LOST: station computation wait: station private pending: '
        'station_compute_wait_expired_or_future')
    assert node._stream_ticket_cancellation_code_locked(ticket) == 'TARGET_LOST'
    assert events[0] == ('retire_start',
        'station private pending: station_compute_wait_expired_or_future')
    assert events[1][0] == 'retire_done'
    assert events[2][0] == 'diagnostic_log'
    assert len(reports) == 1
    assert reports[0]['retired_after'] is True
    assert reports[0]['current_reference_after'] is None
    assert reports[0]['diagnostic_only'] is True
    assert reports[0]['execution_authorized'] is False


def test_original_top_expiry_cancels_ticket_then_reports_exact_contract(work, monkeypatch):
    c, ticket, _plan, parent, _generation = work
    node = c.node
    events, reports = _logged_retirement(monkeypatch, node)
    clock(c, parent['identity_deadline_ns'] + 1_000_000)
    # Direct original fan-in avoids a different outer callback expiry branch.
    assert node._station_private_pending_input('object', parent['source_ns'],
        deepcopy(node._station_compute_reference['object'])) is True
    _assert_original_cancel(node, ticket, events, reports)
    report = reports[0]
    leaf = report['validator_frames'][-1]
    assert leaf['function'] == 'validate_contract'
    assert leaf['path'].endswith('/station_compute_reacquisition.py')
    assert leaf['contracts']['value']['source_ns'] == parent['source_ns']
    assert leaf['clocks']['now_ns'] == node._ros_source_clock_ns()
    assert report['before_retire']['current_reference']['source_ns'] == parent['source_ns']
    assert report['input_kind'] == 'object'
    assert report['input_source_ns'] == parent['source_ns']


def test_nested_expired_parent_is_distinct_from_newer_current_reference(work, monkeypatch):
    c, ticket, _plan, old, _generation = work
    node = c.node
    # Advance through the original full fresh join while old identity is alive.
    newer = next_row(c, old['source_ns'] + 4_500_000_000, old)
    current = deepcopy(node._station_compute_reference['contract'])
    assert current['source_ns'] > old['source_ns']
    now = old['identity_deadline_ns'] + 10_000_000
    clock(c, now)
    source = now - 20_000_000
    feedback(c, source - 10_000_000)
    feedback(c, source + 10_000_000)
    row = row_at(c, source, recovery=current)
    # Explicit invalid historical recovery provider: the current owner remains
    # healthy but this offered nested parent has already expired. No validator
    # is replaced, so the actual throwing pure-validator frame is observable.
    row['status']['station_compute_recovery'] = deepcopy(old)
    row['status']['station_compute_recovery_chain'] = [deepcopy(old)]
    row['status']['station_feedback_gap_recovery'] = {}
    events, reports = _logged_retirement(monkeypatch, node)
    deliver(node, row, ('status',))
    _assert_original_cancel(node, ticket, events, reports)
    report = reports[0]
    assert report['before_retire']['current_reference']['source_ns'] == current['source_ns']
    assert report['input_source_ns'] == source
    leaf = report['validator_frames'][-1]
    assert leaf['function'] == 'validate_contract'
    assert leaf['contracts']['value']['source_ns'] == old['source_ns']
    assert leaf['clocks']['now_ns'] == now
    assert report['callback']['kind'] == 'status'
    assert report['callback']['source_ns'] == source
    assert report['callback']['locks'][0]['name'] == 'mode_stream_geometry'
    assert report['callback']['locks'][0]['wait_ns'] >= 0
    assert report['callback']['locks'][0]['thread_cpu_ns'] >= 0
    assert node._station_compute_reference is None  # No observer resurrection.


@pytest.mark.parametrize('failed_observer', ['failure_before_retire', 'log_failure_after_retire'])
def test_observer_exception_cannot_block_original_retirement(work, monkeypatch, failed_observer):
    c, ticket, _plan, parent, _generation = work
    node = c.node
    original = node._station_compute_retire
    reasons = []

    def retire(reason):
        reasons.append(reason)
        return original(reason)

    def broken(*_args):
        raise RuntimeError('diagnostic provider unavailable')

    monkeypatch.setattr(node, '_station_compute_retire', retire)
    monkeypatch.setattr(remote, failed_observer, broken)
    clock(c, parent['identity_deadline_ns'] + 1_000_000)
    assert node._station_private_pending_input('object', parent['source_ns'],
        deepcopy(node._station_compute_reference['object'])) is True
    assert reasons == ['station private pending: station_compute_wait_expired_or_future']
    assert node._station_compute_retired and not node._station_compute_work
    assert node._stream_ticket_cancellation_code_locked(ticket) == 'TARGET_LOST'


def test_original_retire_exception_is_not_swallowed_or_replaced(work, monkeypatch):
    c, _ticket, _plan, parent, _generation = work
    sentinel = RuntimeError('original retirement failure')
    logged = []

    def retire(_reason):
        raise sentinel

    monkeypatch.setattr(c.node, '_station_compute_retire', retire)
    monkeypatch.setattr(remote, 'log_failure_after_retire', lambda *_args: logged.append(True))
    clock(c, parent['identity_deadline_ns'] + 1_000_000)
    with pytest.raises(RuntimeError) as caught:
        c.node._station_private_pending_input('object', parent['source_ns'],
            deepcopy(c.node._station_compute_reference['object']))
    assert caught.value is sentinel
    assert logged == []


class _Message:
    def __init__(self, source):
        self.header = NS(stamp=NS(to_nsec=lambda: source))


class _Node:
    pass


@pytest.mark.parametrize('raises', [False, True])
def test_nested_callbacks_preserve_return_exception_and_restore_context(raises):
    node = _Node()
    returned = object()
    sentinel = ValueError('original callback error')
    observed = []

    @diagnostic.observe_callback('inner')
    def inner(_node, _message):
        observed.append(diagnostic.callback_snapshot()['source_ns'])
        if raises:
            raise sentinel
        return returned

    @diagnostic.observe_callback('outer')
    def outer(_node, _message):
        try:
            return inner(node, _Message(202))
        finally:
            observed.append(diagnostic.callback_snapshot()['source_ns'])

    assert diagnostic.callback_snapshot() is None
    if raises:
        with pytest.raises(ValueError) as caught:
            outer(node, _Message(101))
        assert caught.value is sentinel
    else:
        assert outer(node, _Message(101)) is returned
    assert observed == [202, 101]
    assert diagnostic.callback_snapshot() is None
    assert [row['source_ns'] for row in diagnostic.recent_callbacks(node)] == [202, 101]


def test_concurrent_callbacks_have_separate_contexts_and_bounded_lock_waits():
    node = _Node()
    entered = threading.Barrier(3)
    release = threading.Event()
    samples = []
    errors = []
    sample_lock = threading.Lock()
    original_calls = []

    @diagnostic.observe_callback('object')
    def callback(_node, message):
        diagnostic.lock_wait_start('controlled_test_wait')
        entered.wait(timeout=3)
        assert release.wait(timeout=3)
        diagnostic.lock_acquired('controlled_test_wait')
        report = diagnostic.callback_snapshot()
        with sample_lock:
            original_calls.append(message.header.stamp.to_nsec())
            samples.append(report)

    def run(source):
        try:
            callback(node, _Message(source))
        except BaseException as error:
            errors.append(error)

    threads = [threading.Thread(target=run, args=(source,)) for source in (11, 22)]
    for thread in threads:
        thread.start()
    try:
        entered.wait(timeout=3)
        assert diagnostic.callback_snapshot() is None
    finally:
        release.set()
        for thread in threads:
            thread.join(timeout=3)
    assert not any(thread.is_alive() for thread in threads)
    assert errors == []
    assert sorted(original_calls) == [11, 22]
    assert {sample['source_ns'] for sample in samples} == {11, 22}
    assert len({sample['thread_id'] for sample in samples}) == 2
    for sample in samples:
        assert len(sample['locks']) == 1
        assert sample['locks'][0]['wait_ns'] >= 0
        assert sample['locks'][0]['thread_cpu_ns'] >= 0
        assert sample['elapsed_ns'] >= sample['locks'][0]['wait_ns']
    assert {row['source_ns'] for row in diagnostic.recent_callbacks(node)} == {11, 22}


def test_completed_history_is_bounded_and_does_not_keep_source_payload():
    node = _Node()

    @diagnostic.observe_callback('status')
    def callback(_node, message):
        diagnostic.source_parsed(message.source)
        return message

    for source in range(40):
        message = NS(source=source, data=b'large-opaque-pixel-payload')
        assert callback(node, message) is message
    history = diagnostic.recent_callbacks(node)
    assert len(history) == 32
    assert [row['source_ns'] for row in history] == list(range(8, 40))
    assert all('data' not in row and 'message' not in row for row in history)
    assert diagnostic.callback_snapshot() is None


def test_traceback_summary_never_serializes_unrelated_locals_or_payloads(monkeypatch):
    class Opaque:
        def __repr__(self):
            raise AssertionError('payload was inspected')

    node = _Node()
    node._station_compute_reference = {'contract': dict(source_ns=200,
        identity_deadline_ns=400, binding={'selection': {'generation': 5}})}
    node._station_private_pending_sources = {1: {'kind': 'collecting', 'payload': Opaque()}}
    node._station_compute_objects = {1: Opaque()}
    node._station_compute_masks = {1: Opaque()}
    node._station_compute_status = {1: Opaque()}
    node._station_compute_work = {15: Opaque()}
    node._station_compute_retired = True
    reports = []
    monkeypatch.setattr(remote.rospy, 'logwarn',
        lambda _message, encoded: reports.append(json.loads(encoded)))

    def validate_contract(value, *, now_ns):
        # The names mirror original validator locals, but their payloads must
        # remain opaque; only the explicit scalar contract fields may escape.
        config = Opaque()
        pixels = Opaque()
        raise ValueError('station_compute_wait_expired_or_future')

    old = dict(source_ns=100, identity_deadline_ns=150,
        binding={'selection': {'generation': 5}}, unrelated=Opaque())
    before = diagnostic.failure_before_retire(node)
    try:
        validate_contract(old, now_ns=300)
    except ValueError as error:
        diagnostic.log_failure_after_retire(node, error, 'status', 250, before)
    assert len(reports) == 1
    report = reports[0]
    assert report['validator_frames'][-1]['contracts']['value']['source_ns'] == 100
    assert report['before_retire']['current_reference']['source_ns'] == 200
    assert report['validator_frames'][-1]['clocks']['now_ns'] == 300
    encoded = json.dumps(report)
    assert 'unrelated' not in encoded and 'pixels' not in encoded and 'config' not in encoded
    assert 'payload' not in encoded and 'temperature' not in encoded
