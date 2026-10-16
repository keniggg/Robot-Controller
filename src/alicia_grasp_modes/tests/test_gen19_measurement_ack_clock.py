"""Measurement ACK snapshot races with real task validation and inert transport."""
from copy import deepcopy
import json
from types import SimpleNamespace as NS

import pytest

from alicia_grasp_modes import observe_return_task as task
from alicia_grasp_modes.observe_return_measurement import PARTICIPANTS, measurement_ack
from test_supplement_measurement_preparation import prepared_task


class ConditionBarrier:
    """Schedule one real callback immediately before condition acquisition."""
    def __init__(self, original, event):
        self.original = original
        self.event = event
        self.entered = False

    def __enter__(self):
        if not self.entered:
            self.entered = True
            self.event()
        return self.original.__enter__()

    def __exit__(self, *args):
        return self.original.__exit__(*args)

    def wait(self, timeout):
        return self.original.wait(timeout)

    def notify_all(self):
        return self.original.notify_all()


@pytest.fixture
def capture(prepared_task, monkeypatch):
    c = prepared_task
    initial = c.node._observation_measurement_begin(c.plan)
    c.now[0] = 10_000_000_000
    c.mono = [20.]
    monkeypatch.setattr(task.time, 'monotonic', lambda: c.mono[0])
    c.request = dict(initial, state='requalifying', request_id='capture',
        stamp_ns=c.now[0], arrival_stamp_ns=c.now[0], capture_deadline_ns=15_000_000_000,
        settle=dict(epoch_ns=initial['control_epoch_ns'],
            stationary_window_start_ns=9_000_000_000,
            stationary_window_end_ns=9_900_000_000))
    c.original_request = deepcopy(c.request)
    c.sources = [10_100_000_000, 10_200_000_000, 10_300_000_000]
    c.sent = []
    c.node._observation_measurement_pub = NS(publish=lambda msg: c.sent.append(json.loads(msg.data)))
    return c


def deliver(c, *, qualified=True, accepted=True):
    for participant in PARTICIPANTS:
        ack = measurement_ack(c.request, participant, accepted=accepted,
            qualified=qualified, source_stamps_ns=c.sources if qualified else [])
        c.node._observation_measurement_ack_cb(NS(data=json.dumps(ack)))


def schedule(c, event):
    c.node._observe_return_condition = ConditionBarrier(c.node._observe_return_condition, event)


def publish(c):
    return c.node._observation_measurement_publish(c.request, qualified=True)


def test_qualified_sources_arriving_while_waiting_for_condition_use_snapshot_clock(capture):
    c = capture

    def incoming():
        c.now[0] = c.sources[-1] + 1
        c.mono[0] += .3
        deliver(c)

    schedule(c, incoming)
    result = publish(c)
    assert len(result) == 2 and all(row['qualified'] for row in result)
    assert [row['source_stamps_ns'] for row in result] == [c.sources, c.sources]
    assert c.sent == [c.original_request] and c.request == c.original_request


@pytest.mark.parametrize('fault', ['future', 'capture_deadline', 'request_deadline', 'monotonic_deadline'])
def test_snapshot_clock_does_not_accept_real_future_or_expired_ack(capture, fault):
    c = capture

    def incoming():
        c.now[0] = c.sources[-1] + 1
        if fault == 'future':
            c.sources[-1] = c.now[0] + 1
        elif fault == 'capture_deadline':
            c.now[0] = c.request['capture_deadline_ns']
        elif fault == 'request_deadline':
            c.now[0] = c.request['deadline_ns']
        else:
            c.mono[0] += 5.
        deliver(c)

    schedule(c, incoming)
    with pytest.raises(ValueError):
        publish(c)
    assert c.sent == [c.original_request] and c.request == c.original_request


@pytest.mark.parametrize('fault', [
    'capture_deadline', 'request_deadline', 'monotonic_deadline', 'epoch',
    'session', 'manual', 'stopped', 'selection', 'revoked', 'request_replaced',
    'request_mutated', 'ack_rejected', 'ack_future', 'ack_mismatch',
])
def test_ack_return_rechecks_original_owner_budget_and_sources_after_logging(capture, monkeypatch, fault):
    c = capture
    c.now[0] = c.sources[-1] + 1
    deliver(c)
    fired = []

    def logging(message, *args):
        if 'measurement acknowledged:' not in message or fired:
            return
        fired.append(True)
        if fault == 'capture_deadline':
            c.now[0] = c.request['capture_deadline_ns']
        elif fault == 'request_deadline':
            c.now[0] = c.request['deadline_ns']
        elif fault == 'monotonic_deadline':
            c.mono[0] += 5.
        elif fault == 'epoch':
            c.node._following_epoch_ns += 1
        elif fault == 'session':
            c.node._observe_return_session['session_id'] = 'other'
        elif fault == 'manual':
            c.params['/gui/joint_direct_mode'] = True
        elif fault == 'stopped':
            c.node.active = False
        elif fault == 'selection':
            c.node._mode_bound_selection = dict(c.node._mode_bound_selection, generation=99)
        elif fault == 'revoked':
            c.node._execution_authority_revoked = True
        elif fault == 'request_replaced':
            c.node._observation_measurement_request = deepcopy(c.request)
        elif fault == 'request_mutated':
            c.request['deadline_ns'] += 1_000_000_000
        elif fault == 'ack_rejected':
            deliver(c, accepted=False)
        elif fault == 'ack_future':
            c.sources[-1] = c.now[0] + 1
            deliver(c)
        elif fault == 'ack_mismatch':
            ack = measurement_ack(c.request, PARTICIPANTS[1], accepted=True,
                qualified=True, source_stamps_ns=[10_100_000_001, *c.sources[1:]])
            c.node._observation_measurement_ack_cb(NS(data=json.dumps(ack)))

    monkeypatch.setattr(task.rospy, 'loginfo', logging)
    with pytest.raises(ValueError):
        publish(c)
    assert fired == [True] and c.sent == [c.original_request]


@pytest.mark.parametrize('update', ['identical', 'qualified_successor'])
def test_same_request_ack_refresh_reuses_original_wait_without_republication(capture, monkeypatch, update):
    c = capture
    c.now[0] = c.sources[-1] + 1
    deliver(c)
    fired = []

    def logging(message, *args):
        if 'measurement acknowledged:' in message and not fired:
            fired.append(True)
            if update == 'qualified_successor':
                c.sources = [ns + 20_000_000 for ns in c.sources]
                c.now[0] += 20_000_000
                c.mono[0] += .02
            deliver(c)

    monkeypatch.setattr(task.rospy, 'loginfo', logging)
    result = publish(c)
    assert [row['source_stamps_ns'] for row in result] == [c.sources, c.sources]
    assert c.sent == [c.original_request] and c.request == c.original_request
