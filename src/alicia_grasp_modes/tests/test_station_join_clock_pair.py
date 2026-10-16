"""Narrow audit probe; delay only after the real task's final registration check."""
from copy import deepcopy
import json,os
from pathlib import Path
import pytest
from test_gen11_contact_compute_admission_independent import (context,computed,audit,measured_row,copy_candidate,freeze,retirement,feed_feedback)
from alicia_grasp_modes import station_compute_wait_task as wait

@pytest.mark.parametrize('ros_advances,pause_sec,expected_pending',[(True,.75,True),(False,.75,True),(True,0.,False),(False,.01,False)])
def test_final_gates_after_join_clock_pair_delay(monkeypatch,tmp_path,ros_advances,pause_sec,expected_pending):
    c=computed(context(monkeypatch));audit(c,tmp_path);n=c.task
    mono=[c.clock[0]];monkeypatch.setattr(wait.time,'monotonic',lambda:mono[0])
    original=wait.validate_registration;calls=[];delayed=[]
    old_record=n._station_task_record;old_source=old_record['contract']['source_ns']
    def checked(value,*a,**kw):
        result=original(value,*a,**kw)
        if value['source_ns']>old_source:
            calls.append(value['source_ns'])
            if calls.count(value['source_ns'])==2 and not delayed:
                # A scheduler pause between final ROS validation and the
                # later monotonic read; all original validators still run.
                delayed.append(dict(source_ns=value['source_ns'],ros_now_ns=kw['now_ns'],mono_before=mono[0]))
                mono[0]+=pause_sec
                if ros_advances:c.clock[0]+=pause_sec;c.joint(c.clock[0]);feed_feedback(c)
        return result
    monkeypatch.setattr(wait,'validate_registration',checked)
    mono[0]+=.4
    measured_row(c,step=.4)
    assert len(delayed)==1
    record=n._station_task_record
    before=dict(contract=deepcopy(record['contract']),geometry_mono=record['geometry_mono'],identity_mono=record['identity_mono'],mono=mono[0],ros_clock=c.clock[0])
    check,plan=copy_candidate(c)
    outcome=dict(ros_advances=ros_advances,delay_sec=pause_sec,delayed=delayed,joined=before,
        task_copy=dict(ok=check.ok,code=check.code,reason=check.reason),freeze_succeeded=False,
        action_sentinel_count=0,exception=None)
    if check.ok:
        try:
            frozen=freeze(c);outcome['freeze_succeeded']=True
            retirement(c,frozen);outcome['action_sentinel_count']=len(c.motion_sentinel)
        except Exception as exc:
            outcome['exception']=type(exc).__name__+': '+str(exc)
            outcome['action_sentinel_count']=len(getattr(c,'motion_sentinel',[]))
    if expected_pending:
        assert not check.ok and check.code==wait.PENDING
        assert record['pending'] and not n._station_task_retired
        assert not outcome['action_sentinel_count']
    else:
        assert check.ok, outcome
        assert outcome['freeze_succeeded'] and outcome['action_sentinel_count']==1, outcome
        assert outcome['exception'] is None, outcome
    # Wall-clock metadata must be anchored to the last actual clock pair,
    # never to the time the rest of registration finished.
    paired=delayed[0]
    assert record['geometry_mono']==pytest.approx(paired['mono_before']+(record['contract']['geometry_deadline_ns']-paired['ros_now_ns'])/1e9,abs=1e-12)
    assert record['identity_mono']==pytest.approx(paired['mono_before']+(record['contract']['identity_deadline_ns']-paired['ros_now_ns'])/1e9,abs=1e-12)



def test_old_check_cannot_quarantine_callback_new_ready(monkeypatch,tmp_path):
    import sys,threading
    c=computed(context(monkeypatch));audit(c,tmp_path);n=c.task
    c.clock[0]+=.82;c.joint(c.clock[0]);feed_feedback(c);n._station_compute_task_tick()
    old=n._station_task_record;assert old['pending']
    entered=threading.Event();resume=threading.Event();done=threading.Event();callback_done=threading.Event()
    target=[];results=[];callback_results=[];lock_owned=[]
    original=n._station_task_context
    def context_gate(contract,*a,**kw):
        result=original(contract,*a,**kw)
        if (target and threading.get_ident()==target[0] and not entered.is_set()
                and sys._getframe(1).f_code.co_name=='_station_task_check'):
            lock_owned.append(n._grasp6d_plan_guard()._is_owned())
            entered.set();assert resume.wait(2)
        return result
    monkeypatch.setattr(n,'_station_task_context',context_gate)
    def run():
        target.append(threading.get_ident())
        try:results.append(copy_candidate(c))
        except BaseException as exc:results.append(exc)
        finally:done.set()
    def callback():
        try:callback_results.append(measured_row(c,step=.2))
        except BaseException as exc:callback_results.append(exc)
        finally:callback_done.set()
    worker=threading.Thread(target=run,daemon=True);worker.start();incoming=None
    try:
        assert entered.wait(2)
        incoming=threading.Thread(target=callback,daemon=True);incoming.start()
        if lock_owned[0]:
            # The original callback must wait for the same guard. The check
            # finishes against its own record before replacement is possible.
            assert not callback_done.wait(.025)
        else:
            # Deterministically expose the original unlocked interleaving.
            assert callback_done.wait(2)
            assert n._station_task_record is not old and not n._station_task_record['pending']
        resume.set();assert done.wait(2) and callback_done.wait(2)
        worker.join();incoming.join()
        assert not any(isinstance(value,BaseException) for value in results+callback_results)
        fresh=n._station_task_record
        assert fresh is not old and not fresh['pending'], 'old check quarantined a new actual READY'
        assert lock_owned==[True]
        # Original plan/source/deadlines remain bound; a fresh check can now
        # commit the original contact candidate exactly once.
        assert fresh['contract']['binding']==old['contract']['binding']
        assert fresh['contract']['phase_deadline_ns']==old['contract']['phase_deadline_ns']
        plan=freeze(c);retirement(c,plan)
        assert len(n._frozen_contact_pub.messages)==1 and len(c.motion_sentinel)==1
    finally:
        resume.set();done.wait(2);worker.join(timeout=2)
        if incoming is not None:callback_done.wait(2);incoming.join(timeout=2)
