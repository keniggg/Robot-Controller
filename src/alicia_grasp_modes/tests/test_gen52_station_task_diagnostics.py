"""Real task callback observations without changing authority or expiry."""
from copy import deepcopy
import json

import pytest

from test_station_compute_wait_task import station, frame, deliver, advance, feedback
from test_task_modes import configured
from alicia_grasp_modes import station_handoff_diagnostics as diagnostic
from alicia_grasp_modes import station_compute_wait_task as task


def test_late_fan_in_records_first_rejection_and_exact_expiring_owner(station, monkeypatch):
    c = station
    first = frame(c)
    deliver(c, first)
    old = deepcopy(c.node._station_task_record['contract'])
    advance(c, 101_050_000_000)
    feedback(c, 100_980_000_000)
    feedback(c, 101_020_000_000)
    later = frame(c, 101_000_000_000, old)
    deliver(c, later, ('object', 'status'))
    advance(c, 102_050_000_000)
    deliver(c, later, ('mask',))
    c.node._station_compute_task_tick()
    saved = diagnostic.task_station_snapshot(c.node)
    rejected = [row for row in saved['recent_join_events']
                if row['kind'] == 'join_rejected']
    assert rejected[-1]['source_ns'] == 101_000_000_000
    assert rejected[-1]['reason'] == 'station_compute_wait_registration_too_late'
    assert rejected[-1]['occurrences'] >= 2
    assert saved['current_reference']['source_ns'] == old['source_ns']
    assert c.node._station_task_record['pending']
    assert c.node.latest_obj is None
    reports = []
    monkeypatch.setattr(task.rospy, 'logwarn', lambda message, *args:
        reports.append(json.loads(args[0])) if message == 'Station task diagnostic: %s' else None)
    advance(c, old['identity_deadline_ns'] + 1_000_000)
    c.node._station_compute_task_tick()
    assert c.node._station_task_retired
    assert c.node._execution_authority_revoked
    assert c.node._station_task_record is None
    assert reports[-1]['reason'] == 'station_compute_wait_expired_or_future'
    assert reports[-1]['retired_after'] is True
    assert reports[-1]['current_reference_after'] is None
    before = reports[-1]['before_retire']
    assert before['current_reference']['source_ns'] == old['source_ns']
    failure = before['recent_join_events'][-1]
    assert failure['kind'] == 'check_rejected'
    assert failure['validator_frames'][-1]['contracts']['value']['source_ns'] == old['source_ns']


def test_recovery_and_duplicate_do_not_gain_authority_from_diagnostics(station):
    c = station
    deliver(c, frame(c))
    old = deepcopy(c.node._station_task_record['contract'])
    advance(c, 101_050_000_000)
    feedback(c, 100_980_000_000)
    feedback(c, 101_020_000_000)
    new = frame(c, 101_000_000_000, old)
    deliver(c, new, ('status', 'mask', 'object'))
    state = deepcopy(c.node._station_task_record)
    deliver(c, new)
    assert c.node._station_task_record == state
    events = diagnostic.task_station_snapshot(c.node)['recent_join_events']
    assert [row['source_ns'] for row in events if row['kind'] == 'join_accepted'] == [
        old['source_ns'], 101_000_000_000]
    assert not c.node._station_task_retired
    assert c.node._station_contact_admitted is None


@pytest.mark.parametrize('failure', ['history', 'logger'])
def test_diagnostic_failure_cannot_prevent_retirement(station, monkeypatch, failure):
    c = station
    deliver(c, frame(c))
    if failure == 'history':
        class Broken:
            def get(self, *args):
                raise RuntimeError('diagnostic storage unavailable')
        monkeypatch.setattr(diagnostic, '_TASK_HISTORY', Broken())
    else:
        def broken(*args):
            raise RuntimeError('diagnostic logger unavailable')
        monkeypatch.setattr(task.rospy, 'logwarn', broken)
    advance(c, c.node._station_task_record['contract']['identity_deadline_ns']+1_000_000)
    c.node._station_compute_task_tick()
    assert c.node._station_task_retired
    assert c.node._execution_authority_revoked
    assert c.node._station_task_record is None


def test_distinct_diagnostic_events_are_bounded(station):
    c = station
    deliver(c, frame(c))
    saved = deepcopy(c.node._station_task_record)
    for i in range(64):
        diagnostic.task_station_event(c.node, 'join_rejected', 100_000_000_000 + i,
            ValueError('example rejected input'))
    report = diagnostic.task_station_snapshot(c.node)
    assert len(report['recent_join_events']) == 16
    assert c.node._station_task_record == saved
