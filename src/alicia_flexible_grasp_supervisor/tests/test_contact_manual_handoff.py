"""Manual ownership must revoke motion without erasing an observed idle task.

Callbacks, plan admission and service validation are real; geometry/controller
doubles are inherited explicitly. No robot or ROS transport is used.
"""
from copy import deepcopy
from types import SimpleNamespace as NS

import pytest

from alicia_flexible_grasp.robot.direct_contact_authority import DirectContactTaskAuthority
from test_gen26_direct_task_authority_independent import bindings, event, lease_case, direct_scenario, run


def manual_round_trip(c):
    c.node._observation_task_sub = NS(get_num_connections=lambda: 1)
    c.node._require_live_observation_task = (
        type(c.node)._require_live_observation_task.__get__(c.node))
    c.node._manual_control_active = lambda: c.node._gui_direct_mode or c.state.get('manual', False)
    # Fixture planner has no real MoveIt execution to cancel.
    c.planner.ready = False
    for mode in (True, True, False, False):
        c.node._gui_mode_cb(NS(data=mode))


@pytest.mark.parametrize('strategy', ['direct', 'observe_return'])
def test_prestart_published_plan_survives_manual_idle_round_trip_without_new_idle_or_republication(monkeypatch, strategy):
    def before_start(c):
        manual_round_trip(c)
        c.params['/grasp_mode/selection']['strategy'] = strategy
    c = direct_scenario(monkeypatch, receipt=False, prestart_receipt=True, before_start=before_start)
    plan = deepcopy(c.plan)
    response, report = run(c)
    assert response.success, report
    assert c.state['moves'] == 1 and c.plan == plan
    assert c.contexts[0]['contact_deadlines']['source'] == 'captured_initial_contact_task'
    assert c.contexts[0]['direct_contact_task']['ros_deadline_ns'] == 609_500_000_000
    assert report['real_grasp_success'] is False


@pytest.mark.parametrize('fault', ['missing', 'invalid', 'expired', 'config', 'track'])
def test_manual_idle_preservation_still_requires_exact_fresh_published_plan(monkeypatch, fault):
    c = direct_scenario(monkeypatch, receipt=False, prestart_receipt=fault != 'missing',
                        prestart_fault=fault, before_start=manual_round_trip)
    response, report = run(c)
    assert not response.success, report
    assert c.state['moves'] == 0 and not c.calls


def test_manual_takeover_after_start_cannot_restore_old_task_or_publication(monkeypatch):
    c = direct_scenario(monkeypatch)
    old = deepcopy(c.node._direct_contact_authority._lease)
    manual_round_trip(c)
    c.task_callback(True, 9.75)
    assert not c.node._direct_contact_authority.has_budget
    assert c.node._direct_contact_authority.budget_required
    assert not c.node._direct_contact_publications
    assert not c.node._direct_contact_plan_receipts
    response, report = run(c)
    assert not response.success, report
    assert c.state['moves'] == 0 and not c.calls
    assert old is not None


@pytest.mark.parametrize('reason', ['task revoked', 'SDK reference epoch changed', 'clock rollback'])
def test_nonmanual_revocation_does_not_preserve_idle(reason):
    owner = DirectContactTaskAuthority()
    event(owner, active=False, stamp=2.)
    owner.revoke(reason)
    owner.revoke('manual takeover', preserve_inactive=True)
    assert event(owner, active=True, stamp=3., mono=101.) is None


@pytest.mark.parametrize('state', ['unobserved', 'active', 'missed', 'invalid', 'expired'])
def test_manual_preservation_cannot_create_or_renew_start_edge(state):
    owner = DirectContactTaskAuthority()
    if state in ('active', 'expired'):
        owner, lease = lease_case()
        if state == 'expired':
            with pytest.raises(ValueError):
                owner.validate(lease, bindings=bindings(), now_ns=604_000_000_000, monotonic_now=701.)
    elif state == 'missed':
        event(owner, active=True, stamp=3.)
    elif state == 'invalid':
        event(owner, active=False, stamp=2.)
        bad = bindings(); bad['selection']['stamp_ns'] = 4_000_000_000
        with pytest.raises(ValueError):
            event(owner, active=True, stamp=3., mono=101., binding=bad)
    owner.revoke('manual takeover', preserve_inactive=True)
    assert event(owner, active=True, stamp=605., mono=702.) is None
    assert not owner.has_budget


def test_repeated_manual_idle_preservation_does_not_change_source_order_or_deadline():
    owner = DirectContactTaskAuthority()
    event(owner, active=False, stamp=2.)
    for _ in range(3):
        owner.revoke('manual takeover', preserve_inactive=True)
    # A late duplicate idle cannot rewrite the existing observed lifecycle.
    event(owner, active=False, stamp=1.9, now=5., mono=103.)
    lease = event(owner, active=True, stamp=4., now=5., mono=103.)
    assert lease['start_stamp_ns'] == 4_000_000_000
    assert lease['ros_deadline_ns'] == 604_000_000_000
    assert lease['monotonic_deadline'] == pytest.approx(702.)
