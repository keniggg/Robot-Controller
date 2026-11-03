"""Bound old-producer callbacks never grant geometry or action authority."""
from copy import deepcopy
import json
import threading
from types import SimpleNamespace as NS
import pytest
from test_station_compute_wait_task import station, configured, frame, deliver, advance, feedback, base, module, at_ns


def owner(c):
    return next(k for k in type(c.node).__mro__ if k.__name__ == 'ObserveReturnTask')


@pytest.fixture
def handoff(station, monkeypatch):
    c = station
    c.plan.object_geometry.header = deepcopy(c.plan.header)
    c.plan.plan_id = base.compute_plan_id(c.plan)
    c.node._bound_execution_plan = deepcopy(c.plan)
    c.node._bound_execution_plan_id = c.plan.plan_id
    c.node._bound_execution_plan_digest = c.node._execution_plan_digest(c.plan)
    c.node.latest_grasp6d_plan = deepcopy(c.plan)
    c.node._observe_return_session['view_plan_id'] = c.plan.plan_id
    c.node._observation_measurement_request['view_plan_id'] = c.plan.plan_id
    c.node._bound_target_drift_result = lambda *a: base.PlanValidationResult(True)
    c.f = frame(c); deliver(c, c.f)
    c.next_plan = deepcopy(c.plan)
    c.next_plan.header.stamp = at_ns(c.now[0]-1)
    c.next_plan.object_geometry.header = deepcopy(c.next_plan.header)
    c.next_plan.plan_id = base.compute_plan_id(c.next_plan)
    c.next_session = dict(c.node._observe_return_session, view_index=3, view_plan_id=c.next_plan.plan_id)
    monkeypatch.setattr(owner(c), '_observe_return_publish', lambda *a: None)
    c.node._observe_return_publish(c.next_session)
    assert c.node._station_task_handoff is not None
    assert c.node._station_task_record is None
    return c


def pending(c, ns=100_801_000_000, successor=False):
    parent = deepcopy(c.f['status']['station_compute_wait']); chain = [parent]
    if successor:
        parent = frame(c, 100_300_000_000)['status']['station_compute_wait']; chain.append(parent)
    g = deepcopy(c.plan.object_geometry); g.header.stamp = at_ns(ns); g.valid = False
    g.failure_reason = 'TARGET_PROCESSING_PENDING: '+json.dumps(dict(station_compute_wait=parent, station_compute_recovery_chain=chain))
    o = deepcopy(c.f['object']); o.header.stamp = at_ns(ns); o.detected = False
    s = deepcopy(c.f['status']); s.update(source_stamp_ns=ns, state='processing_deadline_pending',
        detail='processing_deadline_pending', station_compute_wait=parent, station_compute_recovery_chain=chain)
    s.update(color_stamp_ns=ns, depth_stamp_ns=ns, tf_lookup_stamp_ns=ns)
    return g, o, s


def status(c, value):
    c.node._unknown_support_status_cb(NS(data=json.dumps(value)))


def rebind(c):
    def bind():
        frozen = c.node._freeze_execution_plan(c.next_plan)
        c.node._observe_return_session = deepcopy(c.next_session)
        return frozen
    return c.node._invoke_plan_bound_action(c.plan, {}, 'supplemental observation rebind', bind)


def retire_producers(c, monkeypatch):
    monkeypatch.setattr(owner(c), '_observation_measurement_prepare', lambda *a: None)
    c.node._observation_measurement_prepare(c.next_plan)
    assert c.node._station_task_handoff['cutoff_ns'] == c.now[0]


def test_proven_successor_generated_after_publish_is_only_history(handoff):
    c = handoff; g,o,s = pending(c, successor=True); advance(c, 100_900_000_000)
    c.node._station_handoff_live()
    assert c.node._station_handoff_pending('geometry',g), (c.node._station_task_handoff, g)
    c.node.target_geometry_cb(g); assert not c.node._execution_authority_revoked
    c.node.obj_cb(o); assert not c.node._execution_authority_revoked
    status(c,s)
    assert c.node._station_task_record is None and c.node.latest_target_geometry is None
    assert not c.node._execution_authority_revoked
    result, frozen = rebind(c)
    assert result.ok and frozen.plan_id == c.next_plan.plan_id


def test_unpaired_object_blocks_validation_then_other_thread_status_releases_same_rebind(handoff):
    c=handoff; g,o,s=pending(c); advance(c,100_900_000_000); c.node.obj_cb(o)
    assert c.node._validate_bound_plan_locked(c.plan,{}).code == 'TARGET_PROCESSING_PENDING'
    started=threading.Event(); result=[]; errors=[]
    def bind():
        started.set()
        try: result.append(rebind(c))
        except BaseException as exc: errors.append(exc)
    worker=threading.Thread(target=bind); worker.start(); assert started.wait(1.)
    assert worker.is_alive()
    status(c,s); worker.join(2.)
    assert not worker.is_alive() and not errors and result[0][0].ok
    assert not c.node._station_task_handoff['unpaired_objects']


@pytest.mark.parametrize('fault', ['phase_deadline','monotonic','manual','stop','epoch','selection','camera','target','nonvisual_fault'])
def test_original_owner_and_deadline_changes_block_rebind(handoff, fault):
    c=handoff
    if fault=='phase_deadline':c.now[0]=190_000_000_000
    elif fault=='monotonic':c.mono[0]=100.
    elif fault=='manual':c.params['/gui/joint_direct_mode']=True
    elif fault=='stop':c.node.active=False
    elif fault=='epoch':c.node._following_epoch_ns+=1
    elif fault=='selection':c.params[module.SELECTION_PARAM]=dict(c.selection,generation=4)
    elif fault=='camera':c.params['/camera']['fx']+=1.
    elif fault=='target':c.node._observe_return_session['target_track_id']='other'
    else:c.node._execution_authority_revoked=True
    result, frozen=rebind(c)
    assert not result.ok and frozen is None
    assert c.node._bound_execution_plan.plan_id==c.plan.plan_id


@pytest.mark.parametrize('fault', ['no_contract','wrong_owner','wrong_track','future','not_pending','malformed','changed_chain'])
def test_unproven_geometry_never_gets_exception(handoff, fault):
    c=handoff;g,o,s=pending(c,successor=True);advance(c,100_900_000_000)
    if fault=='wrong_track':g.target_track_id='other'
    elif fault=='future':g.header.stamp=at_ns(c.now[0]+1)
    elif fault=='not_pending':g.failure_reason='TARGET_LOST: observed loss'
    elif fault=='malformed':g.failure_reason='TARGET_PROCESSING_PENDING: {}'
    else:
        p=json.loads(g.failure_reason.partition(':')[2])
        if fault=='no_contract':p['station_compute_wait']=None
        elif fault=='wrong_owner':p['station_compute_wait']['binding']['lease_id']='other'
        else:p['station_compute_recovery_chain']=[]
        g.failure_reason='TARGET_PROCESSING_PENDING: '+json.dumps(p)
    c.node.target_geometry_cb(g)
    assert c.node._execution_authority_revoked


@pytest.mark.parametrize('fault',['lost','first_loss','untyped','foreign_contract'])
def test_false_object_needs_exact_benign_receipt(handoff,fault):
    c=handoff;g,o,s=pending(c);advance(c,100_900_000_000);c.node.obj_cb(o)
    if fault=='lost':s['target_lost']=True
    elif fault=='first_loss':s['first_loss']={'reason':'lost'}
    elif fault=='untyped':s['detail']='unknown_failure'
    else:s['station_compute_wait']['binding']['lease_id']='other'
    status(c,s)
    assert c.node._execution_authority_revoked
    assert not rebind(c)[0].ok


def test_pair_overflow_fails_closed_without_evicting_obligation(handoff):
    c=handoff;advance(c,100_900_000_000)
    for i in range(17):c.node.obj_cb(pending(c,100_801_000_000+i)[1])
    assert c.node._execution_authority_revoked


def test_tombstone_preserves_late_old_notification_without_binding_later_contact(handoff,monkeypatch):
    c=handoff;g,o,s=pending(c);advance(c,100_900_000_000)
    status(c,s);assert rebind(c)[0].ok;retire_producers(c,monkeypatch)
    cutoff=c.node._station_task_handoff['cutoff_ns']
    c.node._observe_return_session.update(phase='returned')
    contact=deepcopy(c.next_plan);contact.diagnostic='CONTACT_EXECUTION_PLAN'
    contact.header.stamp=at_ns(101_000_000_000);contact.object_geometry.header=deepcopy(contact.header)
    contact.plan_id=base.compute_plan_id(contact)
    c.node._freeze_execution_plan(contact,allow_target_occlusion=True)
    c.now[0]=191_000_000_000;c.mono[0]=101.
    c.node.target_geometry_cb(g);c.node.obj_cb(o);status(c,s)
    assert not c.node._execution_authority_revoked
    assert c.node._validate_bound_plan_locked(contact,{}).code == 'CONTACT_EVIDENCE_INVALID'
    assert c.node._station_task_handoff['cutoff_ns']==cutoff


def test_after_producer_cutoff_new_invalid_geometry_is_never_old_history(handoff,monkeypatch):
    c=handoff;advance(c,100_900_000_000);assert rebind(c)[0].ok;retire_producers(c,monkeypatch)
    g,_,_=pending(c,101_000_000_000);advance(c,101_000_000_000)
    c.node.target_geometry_cb(g)
    assert c.node._execution_authority_revoked


def test_no_old_physical_action_or_second_rebind_is_authorized(handoff):
    c=handoff;called=[]
    result,_=c.node._invoke_plan_bound_action(c.plan,{},'strict motion',lambda:called.append(True))
    assert not result.ok and not called
    result,_=rebind(c);assert result.ok
    result,_=c.node._invoke_plan_bound_action(c.plan,{},'supplemental observation rebind',lambda:called.append(True))
    assert not result.ok and not called


def test_publish_failure_and_attempt_clear_discard_all_transition_state(station,monkeypatch):
    c=station;deliver(c,frame(c))
    c.node._validate_bound_plan_locked=lambda *a:base.PlanValidationResult(True)
    def failed(*a):raise ValueError('remote timeout')
    monkeypatch.setattr(owner(c),'_observe_return_publish',failed)
    with pytest.raises(ValueError,match='remote timeout'):
        c.node._observe_return_publish(dict(c.node._observe_return_session,view_index=3,view_plan_id='next'))
    assert c.node._station_task_handoff is None
    c.node._clear_observe_return_attempt()
    assert c.node._station_task_handoff_history==[]


@pytest.mark.parametrize('closed', [False, True])
def test_missing_pair_stops_at_exact_original_current_phase_deadline(handoff, monkeypatch, closed):
    c=handoff;g,o,s=pending(c);advance(c,100_900_000_000)
    if closed:
        assert rebind(c)[0].ok;retire_producers(c,monkeypatch)
    c.node.obj_cb(o);entered=threading.Event();results=[];called=[]
    def run():
        entered.set()
        plan=c.next_plan if closed else c.plan
        label='strict action' if closed else 'supplemental observation rebind'
        results.append(c.node._invoke_plan_bound_action(plan,{},label,lambda:called.append(True)))
    worker=threading.Thread(target=run);worker.start();assert entered.wait(1.)
    assert worker.is_alive() and not called
    c.now[0]=190_000_000_000
    c.node._station_task_handoff['changed'].set();worker.join(2.)
    assert not worker.is_alive() and not results[0][0].ok and not called


def test_submitted_action_exception_is_not_retried(handoff, monkeypatch):
    c=handoff;assert rebind(c)[0].ok;retire_producers(c,monkeypatch);calls=[]
    def action():
        calls.append('submitted')
        raise RuntimeError('physical response failed')
    with pytest.raises(RuntimeError,match='physical response failed'):
        c.node._invoke_plan_bound_action(c.next_plan,{},'strict action',action)
    assert calls==['submitted']


def test_exact_old_registered_positive_does_not_refresh_but_changed_stale_positive_revokes(handoff):
    c=handoff;advance(c,101_000_000_000)
    original=c.node.latest_obj
    c.node.obj_cb(deepcopy(c.f['object']))
    assert not c.node._execution_authority_revoked and c.node.latest_obj is original
    changed=deepcopy(c.f['object']);changed.pose_base.pose.position.x+=.001
    c.node.obj_cb(changed)
    assert c.node._execution_authority_revoked


@pytest.mark.parametrize('after_cutoff', [False, True])
def test_status_cannot_bridge_to_future_or_after_retirement_parent(handoff,monkeypatch,after_cutoff):
    c=handoff;g,o,s=pending(c);advance(c,100_900_000_000)
    if after_cutoff:
        assert rebind(c)[0].ok;retire_producers(c,monkeypatch)
        advance(c,101_000_000_000);parent_ns=100_950_000_000
    else:parent_ns=101_000_000_000
    parent=frame(c,parent_ns)['status']['station_compute_wait']
    s['station_compute_wait']=parent
    s['station_compute_recovery_chain']=[deepcopy(c.f['status']['station_compute_wait']),parent]
    c.node.obj_cb(o);status(c,s)
    assert c.node._execution_authority_revoked
