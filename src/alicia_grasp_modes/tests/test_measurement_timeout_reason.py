"""Distinguish transport failure from exhausted arrival evidence, without new time."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from alicia_grasp_modes import observe_return_measurement as contract
from test_observe_return_measurement import request, arrived

SOURCE = Path(__file__).resolve().parents[1] / 'src/alicia_grasp_modes/observe_return_task.py'
SPEC = importlib.util.spec_from_file_location('measurement_timeout_under_test', SOURCE)
task = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(task)


@pytest.fixture
def exchange(monkeypatch):
    now = [10.4]
    n = task.ObserveReturnTask.__new__(task.ObserveReturnTask)
    n._observation_measurement_acks = {}
    live = []
    n._observation_measurement_live = lambda req: live.append(deepcopy(req))
    sent = []
    n._observation_measurement_pub = NS(publish=lambda msg: sent.append(json.loads(msg.data)))
    req = arrived(request())
    req['capture_deadline_ns'] = 13_000_000_000
    callback = [lambda: None]

    class Condition:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def wait(self, seconds):
            now[0] = round(now[0] + seconds, 9)
            callback[0]()
        def notify_all(self): pass

    n._observe_return_condition = Condition()
    monkeypatch.setattr(task, 'time', NS(monotonic=lambda: now[0]))
    monkeypatch.setattr(task.rospy.Time, 'now', staticmethod(
        lambda: NS(to_nsec=lambda: round(now[0] * 1e9))))
    monkeypatch.setattr(task.rospy, 'loginfo', lambda *a, **kw: None)

    def ack(participant, qualified=False, mutate=None):
        value = contract.measurement_ack(req, participant, accepted=True,
            qualified=qualified,
            source_stamps_ns=[10_100_000_000, 10_200_000_000, 10_300_000_000] if qualified else [])
        if mutate: mutate(value)
        n._observation_measurement_ack_cb(NS(data=json.dumps(value)))
        return value

    return NS(node=n, req=req, now=now, sent=sent, live=live, callback=callback, ack=ack)


@pytest.mark.parametrize('ready,pending', [([], 'unknown,remote'), (['unknown'], 'remote'), (['remote'], 'unknown')])
def test_accepted_transport_without_three_frame_completion_reports_evidence_timeout(exchange, ready, pending):
    c = exchange
    for participant in contract.PARTICIPANTS:
        c.ack(participant, participant in ready)
    original = deepcopy(c.req)
    with pytest.raises(ValueError, match='^OBSERVATION_MEASUREMENT_QUALIFICATION_TIMEOUT: pending=' + pending + '$'):
        c.node._observation_measurement_publish(c.req, qualified=True)
    assert 13.0 <= c.now[0] < 13.021
    assert c.req == original and c.sent == [original]


@pytest.mark.parametrize('received', [[], ['unknown'], ['remote']])
def test_missing_transport_ack_keeps_original_transport_timeout(exchange, received):
    c = exchange
    for participant in received: c.ack(participant)
    with pytest.raises(ValueError, match='^OBSERVATION_MEASUREMENT_ACK_TIMEOUT$'):
        c.node._observation_measurement_publish(c.req, qualified=True)
    assert 13.0 <= c.now[0] < 13.021


def test_later_qualified_pair_completes_with_identical_original_sources(exchange):
    c = exchange
    for participant in contract.PARTICIPANTS: c.ack(participant)
    c.callback[0] = lambda: [c.ack(p, True) for p in contract.PARTICIPANTS]
    result = c.node._observation_measurement_publish(c.req, qualified=True)
    assert c.now[0] < 10.5 and all(a['qualified'] for a in result)
    assert result[0]['source_stamps_ns'] == result[1]['source_stamps_ns']
    assert not any(a['motion_authorized'] or a['metric_geometry_authority'] for a in result)


@pytest.mark.parametrize('field,value', [('request_sha256', 'bad'), ('accepted', False), ('lease_id', 'other'), ('motion_authorized', True)])
def test_invalid_ack_is_immediate_rejection_not_evidence_timeout(exchange, field, value):
    c = exchange
    c.ack('unknown', mutate=lambda a: a.update({field: value}))
    c.ack('remote')
    with pytest.raises(ValueError, match='OBSERVATION_MEASUREMENT_ACK_REJECTED'):
        c.node._observation_measurement_publish(c.req, qualified=True)
    assert c.now[0] == 10.4


def test_transport_only_call_returns_on_acceptance_without_claiming_qualification(exchange):
    c = exchange
    for participant in contract.PARTICIPANTS: c.ack(participant)
    result = c.node._observation_measurement_publish(c.req, qualified=False)
    assert c.now[0] == 10.4 and all(not a['qualified'] for a in result)


def test_bad_update_after_accepted_pair_still_rejects_immediately(exchange):
    c = exchange
    for participant in contract.PARTICIPANTS: c.ack(participant)
    c.callback[0] = lambda: c.ack('remote', mutate=lambda a: a.update(accepted=False, reason='owner_changed'))
    with pytest.raises(ValueError, match='ACK_REJECTED: owner_changed'):
        c.node._observation_measurement_publish(c.req, qualified=True)
    assert c.now[0] < 10.5
