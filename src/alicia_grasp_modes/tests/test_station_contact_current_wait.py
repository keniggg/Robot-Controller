"""An already registered station can wait, but expired geometry cannot commit.

Clock delay is inserted after the real context validator; no provenance,
geometry, owner, plan, or submission validator is replaced with success.
"""
from copy import deepcopy
import threading
import pytest
import rospy

from test_station_contact_admission import station, configured, computed, feedback, frame, deliver
from test_gen11_contact_compute_admission_independent import (
    context as actors, computed as actor_computed, audit, measured_row,
    feed_feedback, retirement, task_base)
from alicia_grasp_modes.station_compute_reacquisition import validate_registration
from alicia_grasp_modes import station_compute_wait_task as wait


def expire_during_context(monkeypatch, c, kind='ros'):
    n=c.node; original=n._station_task_context; entered=[]
    record=n._station_task_record
    def context(contract, *a, **kw):
        result=original(contract,*a,**kw)
        if kw.get('admission') and not entered:
            entered.append(True)
            if kind in ('ros','both','witness'):
                c.now[0]=contract['geometry_deadline_ns']+1
            if kind in ('mono','both'):c.mono[0]=record['geometry_mono']+1e-8
            if kind=='identity_ros':c.now[0]=contract['identity_deadline_ns']
            if kind=='identity_mono':c.mono[0]=record['identity_mono']
            if kind=='witness':record['object'].pose_base.pose.position.x+=.001
            if kind=='future':c.now[0]=contract['source_ns']-1
        return result
    monkeypatch.setattr(n,'_station_task_context',context)
    return entered


@pytest.mark.parametrize('kind',['ros','mono','both'])
def test_already_registered_geometry_crossing_waits_without_renewal(station,monkeypatch,kind):
    c=station;n=c.node;plan,receipt,contract=computed(c)
    n.grasp6d_preview_plan_cb(plan)
    record=n._station_task_record; old=deepcopy(record['contract'])
    deadlines=(record['geometry_mono'],record['identity_mono'])
    entered=expire_during_context(monkeypatch,c,kind)
    result=n._station_contact_provenance(plan)
    assert entered and not result.ok and result.code==wait.PENDING and not result.terminal_for_request
    assert n._station_task_record is record and record['pending']
    assert record['contract']==old and (record['geometry_mono'],record['identity_mono'])==deadlines
    assert n._station_contact_proposal==plan and n._station_contact_admitted is None
    assert n.latest_target_geometry is None and n.latest_grasp6d_preview_plan is None
    assert not n._station_task_retired


@pytest.mark.parametrize('kind',['identity_ros','identity_mono','witness','future'])
def test_real_provenance_or_identity_failure_is_never_pending(station,monkeypatch,kind):
    c=station;plan,_,_=computed(c)
    expire_during_context(monkeypatch,c,kind)
    result=c.node._station_contact_provenance(plan)
    assert not result.ok and result.terminal_for_request
    assert result.code=='STATION_CONTACT_PROVENANCE_INVALID'
    assert c.node._station_contact_admitted is None


def test_initial_late_registration_still_cannot_create_identity(station):
    c=station;plan,_,contract=computed(c)
    with pytest.raises(ValueError,match='registration_too_late'):
        validate_registration(contract,binding=contract['binding'],now_ns=contract['geometry_deadline_ns'])


@pytest.mark.parametrize('boundary',['copy','freeze'])
@pytest.mark.parametrize('outcome',['fresh','manual','epoch','phase','no_new_source'])
def test_actual_contact_caller_rechecks_new_ready_before_single_commit(monkeypatch,tmp_path,boundary,outcome):
    c=actor_computed(actors(monkeypatch));audit(c,tmp_path)
    n=c.task; old_plan=deepcopy(n._bound_execution_plan)
    old_digest=n._bound_execution_plan_digest; old_epoch=n._following_epoch_ns
    old_phase=deepcopy(n._published_near_field_phase);old_session=deepcopy(n._observe_return_session)
    old_contact=deepcopy(c.contact);old_receipt=deepcopy(c.receipt)
    old_record=n._station_task_record;old_contract=deepcopy(old_record['contract'])
    old_mono=(old_record['geometry_mono'],old_record['identity_mono'])
    crossed=threading.Event();pending=threading.Event();done=threading.Event()
    results=[];commits=[]; original_context=n._station_task_context
    original_note=n._station_contact_note; original_freeze=task_base.GraspTaskNode._freeze_execution_plan
    def delayed_context(contract,*a,**kw):
        result=original_context(contract,*a,**kw)
        if (kw.get('admission') and not crossed.is_set()
                and (boundary=='copy' or n._station_contact_commit_entered)):
            # Equivalent to R5's elapsed validation work crossing the source
            # deadline. The next callback must acquire the real task guard.
            c.clock[0]=(contract['geometry_deadline_ns']+7_400_000)/1e9
            c.joint(c.clock[0]);feed_feedback(c)
            crossed.set()
        return result
    def note(plan,result):
        value=original_note(plan,result)
        if crossed.is_set() and result.code==wait.PENDING:pending.set()
        return value
    def freeze_once(self,plan,*a,**kw):
        commits.append(deepcopy(plan));return original_freeze(self,plan,*a,**kw)
    monkeypatch.setattr(n,'_station_task_context',delayed_context)
    monkeypatch.setattr(n,'_station_contact_note',note)
    monkeypatch.setattr(task_base.GraspTaskNode,'_freeze_execution_plan',freeze_once)
    # Inert transport and scheduler wait only; original call/commit gates run.
    n._set_near_field_preview_stream=lambda cfg,enabled:True
    monkeypatch.setattr(rospy,'sleep',lambda seconds:threading.Event().wait(.002))
    def run():
        try:results.append(n._maybe_rebind_near_field_grasp6d_plan(c.gcfg,{},c.observation))
        except BaseException as exc:results.append(exc)
        finally:done.set()
    worker=threading.Thread(target=run,daemon=True);worker.start()
    try:
        assert pending.wait(2),results
        with n._grasp6d_plan_guard():
            assert not commits and n._station_contact_admitted is None
            assert n._bound_execution_plan==old_plan and n._bound_execution_plan_digest==old_digest
            assert n._following_epoch_ns==old_epoch
            assert n._published_near_field_phase==old_phase and n._observe_return_session==old_session
            assert old_record['contract']==old_contract
            assert (old_record['geometry_mono'],old_record['identity_mono'])==old_mono
            assert n._station_contact_proposal==old_contact
        assert not done.wait(.025)
        if outcome=='fresh':
            measured_row(c,step=.2)
        elif outcome=='manual':c.params['/gui/joint_direct_mode']=True
        elif outcome=='epoch':n._following_epoch_ns+=1
        elif outcome=='phase':
            c.clock[0]=old_phase.deadline.to_sec();c.joint(c.clock[0]);feed_feedback(c)
        else:
            c.clock[0]=old_contract['identity_deadline_ns']/1e9;c.joint(c.clock[0]);feed_feedback(c)
        assert done.wait(2),n.states
        worker.join()
        assert len(results)==1 and not isinstance(results[0],BaseException),results
        if outcome=='fresh':
            assert results[0]==old_contact and commits==[old_contact]
            assert len(n._frozen_contact_pub.messages)==1
            assert n._station_contact_admitted['registered_source_ns']>old_contract['source_ns']
            assert n._published_near_field_phase==old_phase
            assert n._observe_return_session['deadline_ns']==old_session['deadline_ns']
            assert c.contact==old_contact and c.receipt==old_receipt
            retirement(c,results[0])
            assert len(c.motion_sentinel)==1
        else:
            assert results[0] is None and not commits
            assert not n._frozen_contact_pub.messages and n._station_contact_admitted is None
    finally:
        if not done.is_set():
            n.active=False;c.clock[0]=old_phase.deadline.to_sec()+1
            done.wait(2);worker.join(timeout=2)
