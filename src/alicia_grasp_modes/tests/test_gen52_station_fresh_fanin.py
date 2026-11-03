"""Expired cached joins cannot spend the next RGB-D source's admission window.

Clock cost below explicitly models live context/ROS latency. It is not a claim
that the historical recorder captured the task subscriber's scheduling.
"""
from copy import deepcopy
from itertools import permutations

import pytest

from test_task_modes import configured
from test_station_compute_wait_task import station, frame, feedback, deliver, advance


@pytest.mark.parametrize('order', list(permutations(('object', 'mask', 'status'))))
def test_expired_backlog_cannot_starve_fresh_three_topic_join(station, monkeypatch, order):
    c = station
    deliver(c, frame(c))
    parent = deepcopy(c.node._station_task_record['contract'])
    advance(c, 102_600_000_000)
    # All delayed sources arrive through original callbacks, remain recorded,
    # and fail the unchanged source-age gate. No accepted result is injected.
    for i in range(12):
        source = 100_100_000_000 + i * 100_000_000
        feedback(c, source - 20_000_000)
        feedback(c, source + 20_000_000)
        deliver(c, frame(c, source, parent))
    assert c.node._station_task_record['contract'] == parent
    assert c.node._station_task_record['pending']
    feedback(c, 101_980_000_000)
    feedback(c, 102_020_000_000)
    feedback(c, 102_600_000_000)
    original = c.node._station_task_context
    calls = []

    def context_with_latency(contract, **kwargs):
        # A bounded 30 ms delay for each *admission* context operation.
        # Ordinary identity checks and transport callbacks remain original.
        if kwargs.get('source_ns') is not None:
            calls.append(kwargs['source_ns'])
            c.now[0] += 30_000_000
            c.mono[0] += .03
        return original(contract, **kwargs)

    monkeypatch.setattr(c.node, '_station_task_context', context_with_latency)
    fresh = frame(c, 102_000_000_000, parent)
    deliver(c, fresh, order)
    assert calls == [102_000_000_000]
    assert c.node._station_task_record['contract'] == fresh['status']['station_compute_wait']
    assert not c.node._station_task_record['pending']
    assert not c.node._station_task_retired
    assert c.node.latest_obj.header.stamp.to_nsec() == 102_000_000_000
    # Successful measurement fan-in alone is never a contact/motion commit.
    assert c.node._station_contact_admitted is None


def test_final_age_recheck_still_rejects_delay_inside_live_context(station, monkeypatch):
    c = station
    deliver(c, frame(c))
    parent = deepcopy(c.node._station_task_record['contract'])
    advance(c, 101_600_000_000)
    feedback(c, 100_980_000_000)
    feedback(c, 101_020_000_000)
    feedback(c, 101_600_000_000)
    original = c.node._station_task_context

    def late_context(contract, **kwargs):
        result = original(contract, **kwargs)
        if kwargs.get('source_ns') is not None:
            c.now[0] += 250_000_000
            c.mono[0] += .25
        return result

    monkeypatch.setattr(c.node, '_station_task_context', late_context)
    deliver(c, frame(c, 101_000_000_000, parent))
    assert c.node._station_task_record['contract'] == parent
    assert c.node._station_task_record['pending']
    assert c.node._station_contact_admitted is None
