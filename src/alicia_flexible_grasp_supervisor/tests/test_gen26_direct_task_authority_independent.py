"""Independent task lifecycle and real gateway entry checks; no ROS motion.

The controller double advances through the recorded 135.853 s path. Authority,
task callbacks, published-plan binding and service methods are production code.
"""
from copy import deepcopy
from types import SimpleNamespace as NS

import pytest
import rospy

from alicia_flexible_grasp.robot.direct_contact_authority import (
    DirectContactTaskAuthority, authority_bindings)
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
from test_gen25_contact_execution_budget_independent import scenario, run, RECORDED_DURATION


def bindings():
    return authority_bindings(dict(mode='unknown', strategy='direct', generation=4,
        stamp_ns='1500000000'), 1_000_000_000, 2, '<robot/>', {'budget':600.})


def event(owner, *, active, stamp, now=None, mono=100., binding=None, **kw):
    return owner.observe(active=active, terminal=False, stamp_ns=round(stamp*1e9),
        now_ns=round((stamp if now is None else now)*1e9), monotonic_now=mono,
        bindings=bindings() if binding is None else binding, budget_sec=600., **kw)


def lease_case():
    owner = DirectContactTaskAuthority()
    event(owner, active=False, stamp=2., mono=100.)
    lease = event(owner, active=True, stamp=3., now=5., mono=102.)
    return owner, lease


def test_fixed_task_clocks_start_at_original_active_stamp_not_receive_or_plan_time():
    owner, lease = lease_case()
    assert lease['ros_deadline_ns'] == 603_000_000_000
    assert lease['monotonic_deadline'] == pytest.approx(700.)
    heartbeat = event(owner, active=True, stamp=200., mono=297.)
    assert heartbeat == lease
    first = owner.bind_plan(lease, plan_sha256='a'*64, source_ns=2_000_000_000,
        track_id='g4-t1', bindings=bindings(), now_ns=200_000_000_000, monotonic_now=297.)
    second = owner.bind_plan(lease, plan_sha256='b'*64, source_ns=201_000_000_000,
        track_id='g4-t1', bindings=bindings(), now_ns=201_000_000_000, monotonic_now=298.)
    assert first['task_key'] == second['task_key'] == lease['task_key']
    assert owner.capture(bindings=bindings(), now_ns=201_000_000_000,
        monotonic_now=298.) == lease


def test_latched_active_and_later_heartbeat_cannot_create_budget():
    owner = DirectContactTaskAuthority()
    assert event(owner, active=True, stamp=3.) is None
    assert event(owner, active=True, stamp=4., mono=101.) is None
    assert not owner.has_budget


def test_invalid_first_active_consumes_inactive_arm():
    owner = DirectContactTaskAuthority()
    event(owner, active=False, stamp=2.)
    invalid = bindings(); invalid['selection']['stamp_ns'] = 4_000_000_000
    with pytest.raises(ValueError):
        event(owner, active=True, stamp=3., mono=101., binding=invalid)
    assert event(owner, active=True, stamp=5., mono=103.) is None
    assert not owner.has_budget


def test_delayed_inactive_does_not_turn_heartbeat_into_new_task():
    owner, lease = lease_case()
    event(owner, active=False, stamp=2.5, now=100., mono=197.)
    assert event(owner, active=True, stamp=101., mono=198.) == lease


@pytest.mark.parametrize('reason', ['manual takeover', 'terminal', 'connection lost', 'SDK epoch'])
def test_revocation_requires_new_real_inactive_edge(reason):
    owner, lease = lease_case()
    owner.revoke(reason)
    assert event(owner, active=True, stamp=10., mono=107.) is None
    with pytest.raises(ValueError):
        owner.validate(lease, bindings=bindings(), now_ns=10_000_000_000, monotonic_now=107.)


@pytest.mark.parametrize('field', ['selection', 'epoch_ns', 'revocation', 'model_sha256', 'config_sha256'])
def test_changed_binding_irreversibly_revokes_even_if_restored(field):
    owner, lease = lease_case(); changed = deepcopy(bindings())
    if field == 'selection': changed[field]['generation'] += 1
    elif field.endswith('sha256'): changed[field] = 'e'*64
    else: changed[field] += 1
    with pytest.raises(ValueError):
        owner.validate(lease, bindings=changed, now_ns=6_000_000_000, monotonic_now=103.)
    with pytest.raises(ValueError):
        owner.validate(lease, bindings=bindings(), now_ns=6_000_000_000, monotonic_now=103.)


@pytest.mark.parametrize('clock', ['ros', 'mono'])
def test_high_water_clock_rollback_cannot_renew_task(clock):
    owner, lease = lease_case()
    owner.validate(lease, bindings=bindings(), now_ns=50_000_000_000, monotonic_now=147.)
    with pytest.raises(ValueError, match='CLOCK_ROLLBACK'):
        owner.validate(lease, bindings=bindings(), now_ns=49_900_000_000 if clock=='ros' else 50_000_000_000,
            monotonic_now=146.9 if clock=='mono' else 147.)
    assert not owner.has_budget


@pytest.mark.parametrize('clock', ['ros', 'mono'])
def test_either_fixed_deadline_expires_at_equality(clock):
    owner, lease = lease_case()
    with pytest.raises(ValueError):
        owner.validate(lease, bindings=bindings(), now_ns=603_000_000_000 if clock=='ros' else 6_000_000_000,
            monotonic_now=700. if clock=='mono' else 103.)


def test_plan_track_change_cannot_rebind_same_task():
    owner, lease = lease_case()
    kw=dict(plan_sha256='a'*64, source_ns=2_000_000_000, bindings=bindings(),
        now_ns=5_000_000_000, monotonic_now=102.)
    owner.bind_plan(lease, track_id='g4-t1', **kw)
    with pytest.raises(ValueError): owner.bind_plan(lease, track_id='g4-t2', **kw)
    assert not owner.has_budget


def direct_scenario(monkeypatch, *, inactive=True, receipt=True, compute=6., prestart_receipt=False,
                    prestart_fault=None):
    c = scenario(monkeypatch, bound=False, compute=compute)
    from alicia_flexible_grasp.robot import contact_pregrasp_gateway as module
    monkeypatch.setattr(module.rospy,'sleep',lambda seconds:c.advance(seconds))
    c.params['/grasp_mode/selection']['strategy'] = 'direct'
    c.params['/grasp']['unknown_observe_return_session_timeout_sec'] = 600.
    c.node._observation_task_state_stamp_ns = 0
    c.node._observation_task_requires_inactive = False
    c.node._gui_direct_mode = False
    def callback(active, stamp, stage=GraspStages.MOVE_PREGRASP):
        msg = NS(header=NS(stamp=rospy.Time.from_sec(stamp)), active=active,
            stage=stage, state='MOVE_PREGRASP' if active else 'IDLE',
            message='planning 6D pregrasp' if active else 'idle')
        c.node._observation_task_state_cb(msg)
    c.task_callback = callback
    if inactive: callback(False, 9., GraspStages.IDLE)
    if prestart_receipt:
        c.plan.header.stamp=rospy.Time.from_sec(9.2 if prestart_fault!='expired' else 2.)
        c.plan.object_geometry.header.stamp=deepcopy(c.plan.header.stamp)
        c.plan.plan_id=compute_plan_id(c.plan)
        c.node._committed_observation_plan_cb(deepcopy(c.plan))
        if prestart_fault=='invalid':
            invalid=deepcopy(c.plan);invalid.valid=False
            c.node._committed_observation_plan_cb(invalid)
        elif prestart_fault=='config':
            c.params['/grasp']['measured_endpoint_position_tolerance_m']+=.001
        elif prestart_fault=='track':
            c.plan.target_track_id='g20-t99';c.plan.object_geometry.target_track_id='g20-t99'
            c.plan.plan_id=compute_plan_id(c.plan)
    callback(True, 9.5)
    if receipt: c.node._committed_observation_plan_cb(deepcopy(c.plan))
    return c


def test_actual_inactive_active_callback_and_published_plan_allow_136s_without_observation_origin(monkeypatch):
    c = direct_scenario(monkeypatch)
    assert not getattr(c.node, '_observe_return_origin', None)
    response, report = run(c)
    assert response.success, report
    assert c.authority_samples == [(30.,True),(61.,True),(RECORDED_DURATION,True)]
    assert c.state['moves'] == 1
    ctx=c.contexts[0]
    assert ctx['outer_observation_session'] is None
    assert ctx['contact_deadlines']['source'] == 'captured_initial_contact_task'
    assert ctx['direct_contact_task']['ros_deadline_ns'] == 609_500_000_000
    assert c.node._pregrasp_context(deepcopy(c.plan),0)['deadline']-c.clock['mono'] == pytest.approx(60.)
    assert report['real_grasp_success'] is False


def test_actual_latched_active_callback_cannot_grant_long_path(monkeypatch):
    c=direct_scenario(monkeypatch,inactive=False)
    response,report=run(c)
    assert not response.success,report
    assert not c.calls and c.state['moves']==0


def test_actual_direct_task_still_enforces_compute60(monkeypatch):
    c=direct_scenario(monkeypatch,compute=60.001)
    response,report=run(c)
    assert not response.success,report
    assert not c.calls and c.state['moves']==0


@pytest.mark.parametrize('fault',['manual','epoch','receipt','config'])
def test_actual_direct_submission_rechecks_owner_and_exact_published_receipt(monkeypatch,fault):
    c=direct_scenario(monkeypatch)
    def change(phase):
        if phase!='before_submit':return
        if fault=='manual':c.state['manual']=True
        elif fault=='epoch':c.node._control_reference_epoch_ns+=1
        elif fault=='receipt':c.node._direct_contact_plan_receipts.clear()
        else:c.params['/grasp']['unknown_observe_return_session_timeout_sec']=601.
    c.phase_hook=change
    response,report=run(c)
    assert not response.success,report
    assert not c.calls and c.state['moves']==0


def test_actual_callback_ignores_delayed_inactive_without_extending_task(monkeypatch):
    c=direct_scenario(monkeypatch)
    lease=deepcopy(c.node._direct_contact_authority._lease)
    c.advance(20.)
    c.task_callback(False,9.25,GraspStages.IDLE)
    c.task_callback(True,30.)
    assert c.node._direct_contact_authority._lease==lease


def test_actual_published_plan_before_task_start_does_not_depend_on_republication(monkeypatch):
    # The real task freezes an existing published plan before setting active.
    # Subscriber ordering cannot require a later identical plan publication.
    c=direct_scenario(monkeypatch,receipt=False,prestart_receipt=True)
    from alicia_flexible_grasp.robot import contact_pregrasp_gateway as module
    monkeypatch.setattr(module.rospy,'sleep',lambda seconds:c.advance(seconds))
    response,report=run(c)
    assert response.success,report
    assert c.state['moves']==1


@pytest.mark.parametrize('fault',['missing','invalid','expired','config','track'])
def test_prestart_publication_is_exact_fresh_fact_never_created_by_service_request(monkeypatch,fault):
    c=direct_scenario(monkeypatch,receipt=False,prestart_receipt=fault!='missing',prestart_fault=fault)
    from alicia_flexible_grasp.robot import contact_pregrasp_gateway as module
    monkeypatch.setattr(module.rospy,'sleep',lambda seconds:c.advance(seconds))
    if fault=='missing':c.node._direct_contact_publications={}
    response,report=run(c)
    assert not response.success,report
    assert c.state['moves']==0 and not c.calls
