"""Task collector joins the same timely notice as the actual remote consumer."""
from copy import deepcopy
import itertools
import json
from types import SimpleNamespace as NS

import pytest

import test_gen27_feedback_gap_completion_independent as independent
from test_initial_preflight_task import prepared, feed, notice
from alicia_grasp_modes.feedback_recovery_completion import FIELD, validate_feedback_completion


def task_case(monkeypatch, *, forward=True):
    state = {}
    original_scene = independent.initial_scene.__wrapped__
    original_deliver = independent.deliver
    def scene(m):
        state['scene'] = original_scene(m)
        return state['scene']
    def gap_deliver(remote, gap):
        original_deliver(remote, gap)
        s = state['scene']; s.gap = gap
        c = prepared.__wrapped__(s, monkeypatch); state['task'] = c
        c.node.target_geometry_cb(notice(c)); feed(c, gap)
        old = remote._silhouette_source_status_cb
        def receive(msg):
            old(msg)
            if forward:
                c.node._unknown_support_status_cb(deepcopy(msg))
        remote._silhouette_source_status_cb = receive
    monkeypatch.setattr(independent, 'initial_scene', NS(__wrapped__=scene))
    monkeypatch.setattr(independent, 'deliver', gap_deliver)
    s = independent.recovery_case(monkeypatch)
    return s, state['task']


@pytest.mark.parametrize('order', list(itertools.permutations(('object', 'mask', 'status'))))
def test_real_task_timely_notice_keeps_quarantine_until_complete_new_triple(monkeypatch, order):
    s, c = task_case(monkeypatch)
    assert s.recovery['object'].detected
    c.node._initial_preflight_refresh()
    assert c.node._initial_preflight_pending
    assert c.node.latest_obj is None
    assert not c.node._execution_authority_revoked
    feed(c, s.recovery, order)
    assert not c.node._initial_preflight_collector.fatal
    assert not c.node._initial_preflight_pending
    assert not c.node._execution_authority_revoked
    assert c.node.latest_obj.header.stamp.to_nsec() == s.source_ns
    assert not c.node._initial_preflight_attempt['physical_action_submitted']


def test_task_missing_notice_cannot_adopt_late_completion(monkeypatch):
    s, c = task_case(monkeypatch, forward=False)
    c.node._initial_preflight_refresh()
    assert c.node._execution_authority_revoked
    for kind, message in s.intermediate:
        if kind == 'status': c.node._unknown_support_status_cb(message)
    feed(c, s.recovery)
    assert c.node._execution_authority_revoked


@pytest.mark.parametrize('mutation', ('source', 'entry', 'deadline', 'bracket', 'station', 'authority'))
def test_completion_contract_rejects_mutated_bound_evidence(monkeypatch, mutation):
    s = independent.recovery_case(monkeypatch)
    value = deepcopy(s.recovery['status']['source_qualification'][FIELD])
    args = (value['reference_source_stamp_ns'], value['maximum_frame_age_seconds'],
            deepcopy(value['identity_template_source']), deepcopy(value['station']))
    if mutation == 'source': value['source_stamp_ns'] += 1
    elif mutation == 'entry': value['qualified_entry_ns'] = value['geometry_deadline_ns']
    elif mutation == 'deadline': value['identity_completion_deadline_ns'] += 1
    elif mutation == 'bracket': value['requalified_accepted_feedback'][1][0] += 300000000
    elif mutation == 'station': value['station']['q_station_rad'][0] += .01
    else: value['motion_authorized'] = True
    with pytest.raises(ValueError):
        validate_feedback_completion(value, *args, round(s.clock[0]*1e9), s.source_ns)
