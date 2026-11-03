"""GEN6 measured timings; synthetic producer pixels, unchanged consumer code."""
from copy import deepcopy

import pytest

from test_locked_rgb_hold import locked
from test_prestream_acquisition_identity import inactive_consumer
from test_unknown_pending_identity import publication, deliver


ACQUIRED_NS = 1791800395830269098
LATEST_NS = 1791800396363036632
RETIRE_NS = 1791800396726848840


def test_recorded_late_start_rejects_old_template_despite_fresh_latest_geometry(monkeypatch):
    p, clock, feed, _, params = locked(monkeypatch, initial_station=True)
    n = inactive_consumer(p, clock, params)
    acquired = publication(p)
    deliver(n, acquired)
    source = acquired['status']['source_stamp_ns']
    # The producer fixture stamps feed() at current clock minus .1 seconds.
    clock[0] = source / 1e9 + (LATEST_NS - ACQUIRED_NS) / 1e9 + .1
    feed(target_depth=.35)
    current = publication(p)
    assert current['status']['detail'] == 'tracked'
    deliver(n, current)
    clock[0] = source / 1e9 + (RETIRE_NS - ACQUIRED_NS) / 1e9
    assert n.start_streaming()
    assert n._unknown_identity_retired_selection == n._mode_selection
    assert not n.latest_object.detected
    assert any('initial template does not match acquired source' in msg.failure_reason
               for msg in n.geometry_pub.messages)
    assert clock[0] - source / 1e9 == pytest.approx(.896579742)
    assert clock[0] - current['status']['source_stamp_ns'] / 1e9 == pytest.approx(.363812208)
    # The same fresh true publication gets replaced by false only after the
    # local irreversible retirement, reproducing the live router/remote split.
    assert current['object'].detected
    n.object_cb(deepcopy(current['object']))
    assert not n.latest_object.detected


def test_prewarmed_order_uses_same_original_source_deadline_and_identity_guards(monkeypatch):
    p, clock, _, _, params = locked(monkeypatch, initial_station=True)
    n = inactive_consumer(p, clock, params)
    # Preload before selecting/acquiring; start still goes through the normal
    # callback chain. Nothing changes in the consumer or its .8-second rule.
    assert n.start_streaming()
    acquired = publication(p)
    deliver(n, acquired)
    source = acquired['status']['source_stamp_ns']
    assert n._unknown_identity_retired_selection is None
    assert n._unknown_identity_template['provenance']['source_stamp_ns'] == source
    assert n._unknown_identity_reference['deadline_ns'] == source + 800_000_000
    assert not n.rich_plan_pub.messages[-1].valid
