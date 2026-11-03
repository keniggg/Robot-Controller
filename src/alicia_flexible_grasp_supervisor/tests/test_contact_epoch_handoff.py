"""SDK reference replacement while idle preserves task facts, not authority."""
from copy import deepcopy
import threading
from types import SimpleNamespace as NS

import pytest
import rospy

from test_contact_manual_handoff import manual_round_trip
from test_gen26_direct_task_authority_independent import direct_scenario, run


def epoch(c, stamp):
    c.node._command_reference_lock = threading.RLock()
    seconds, nanoseconds = divmod(round(stamp*1e9), 1_000_000_000)
    c.node._control_reference_epoch_cb(NS(
        stamp=rospy.Time(seconds, nanoseconds), frame_id='sdk_reference_epoch'))


@pytest.mark.parametrize('strategy', ['direct', 'observe_return'])
def test_idle_epoch_change_then_new_publication_and_task_use_new_epoch(monkeypatch, strategy):
    def before_start(c):
        manual_round_trip(c)
        c.params['/grasp_mode/selection']['strategy'] = strategy
        epoch(c, 9.1)
        assert not c.node._direct_contact_authority.has_budget
        assert not c.node._direct_contact_publications
        # No new inactive task message and no repeated plan publication.
    c = direct_scenario(monkeypatch, receipt=False, prestart_receipt=True,
                        before_start=before_start)
    response, report = run(c)
    assert response.success, report
    assert c.state['moves'] == 1
    lease = c.contexts[0]['direct_contact_task']
    assert lease['bindings']['epoch_ns'] == 9_100_000_000
    assert lease['start_stamp_ns'] == 9_500_000_000
    assert lease['ros_deadline_ns'] == 609_500_000_000


@pytest.mark.parametrize('fault', ['missing', 'invalid', 'expired', 'config', 'track'])
def test_epoch_idle_preservation_keeps_exact_publication_checks(monkeypatch, fault):
    c = direct_scenario(monkeypatch, receipt=False, prestart_receipt=fault!='missing',
        prestart_fault=fault, before_start=lambda c: epoch(c, 9.1))
    response, report = run(c)
    assert not response.success, report
    assert not c.calls and c.state['moves']==0


def test_epoch_after_active_revokes_budget_and_heartbeat_cannot_recreate_it(monkeypatch):
    c = direct_scenario(monkeypatch)
    previous = deepcopy(c.node._direct_contact_authority._lease)
    epoch(c, 9.6)
    c.task_callback(True, 9.7)
    assert not c.node._direct_contact_authority.has_budget
    assert c.node._direct_contact_authority.budget_required
    assert not c.node._direct_contact_publications
    assert not c.node._direct_contact_plan_receipts
    response, report = run(c)
    assert not response.success, report
    assert not c.calls and c.state['moves']==0 and previous is not None


def test_epoch_without_observed_inactive_task_cannot_create_start(monkeypatch):
    c = direct_scenario(monkeypatch, inactive=False, receipt=False,
        prestart_receipt=True, before_start=lambda c: epoch(c, 9.1))
    assert not c.node._direct_contact_authority.has_budget
    response, report = run(c)
    assert not response.success, report
    assert not c.calls and c.state['moves']==0


def test_old_epoch_publication_is_cleared_even_while_task_is_idle(monkeypatch):
    def before(c):
        c.node._committed_observation_plan_cb(deepcopy(c.plan))
        assert c.node._direct_contact_publications
        epoch(c, 9.1)
        assert not c.node._direct_contact_publications
    c = direct_scenario(monkeypatch, receipt=False, before_start=before)
    response, report = run(c)
    assert not response.success, report
    assert not c.calls and c.state['moves']==0
