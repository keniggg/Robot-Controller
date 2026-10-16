"""Typed drift notices remain quarantining, now atomic with actual READY joins."""
from copy import deepcopy
import json,sys,threading
import pytest
from std_msgs.msg import String
from test_gen11_contact_compute_admission_independent import context,computed,measured_row,feed_feedback
from alicia_grasp_modes import station_compute_wait_task as wait
from alicia_grasp_modes.observe_return_measurement import STATION_REQUALIFICATION,station_requalification_witness


def notice(c):
    parent=deepcopy(c.task._station_task_record['contract'])
    c.clock[0]+=.1;c.joint(c.clock[0]);feed_feedback(c)
    ns=wait.rospy.Time.now().to_nsec();q=list(parent['binding']['station_q_rad']);q[-1]+=.006135923
    value=deepcopy(c.ready['status']);value.update(source_stamp_ns=ns,state='pending',detail=STATION_REQUALIFICATION,
        target_lost=False,first_loss=None,
        **{STATION_REQUALIFICATION:station_requalification_witness(parent,ns,q,ns)})
    return value


def send(n,value):n._unknown_support_status_cb(String(data=json.dumps(value)))


@pytest.mark.parametrize('boundary',['validated_parent','checked_record'])
def test_notice_and_fresh_join_are_one_record_transaction(monkeypatch,boundary):
    c=computed(context(monkeypatch));n=c.task;value=notice(c);old=n._station_task_record
    entered=threading.Event();resume=threading.Event();done=threading.Event();callback_done=threading.Event()
    target=[];errors=[];callback_results=[];held=[];quarantined=[]
    def pause():
        held.append(n._grasp6d_plan_guard()._is_owned());entered.set();assert resume.wait(2)
    validate=wait.validate_station_requalification;check=n._station_task_check;quarantine=n._station_task_quarantine
    def validated(*a,**kw):
        result=validate(*a,**kw)
        if boundary=='validated_parent' and target and threading.get_ident()==target[0] and not entered.is_set():pause()
        return result
    def checked(*a,**kw):
        result=check(*a,**kw)
        if (boundary=='checked_record' and target and threading.get_ident()==target[0]
                and sys._getframe(1).f_code.co_name=='_station_compute_task_status' and not entered.is_set()):pause()
        return result
    def quarantining():
        if target and threading.get_ident()==target[0]:
            quarantined.append((n._station_task_record['contract']['source_ns'],n._grasp6d_plan_guard()._is_owned()))
        return quarantine()
    monkeypatch.setattr(wait,'validate_station_requalification',validated)
    monkeypatch.setattr(n,'_station_task_check',checked)
    monkeypatch.setattr(n,'_station_task_quarantine',quarantining)
    def worker():
        target.append(threading.get_ident())
        try:send(n,value)
        except BaseException as exc:errors.append(exc)
        finally:done.set()
    def callback():
        try:callback_results.append(measured_row(c,step=.2))
        except BaseException as exc:errors.append(exc)
        finally:callback_done.set()
    t=threading.Thread(target=worker,daemon=True);t.start();incoming=None
    try:
        assert entered.wait(2)
        incoming=threading.Thread(target=callback,daemon=True);incoming.start()
        if held[0]:assert not callback_done.wait(.025)
        else:
            assert callback_done.wait(2)
            assert n._station_task_record is not old and not n._station_task_record['pending']
        resume.set();assert done.wait(2) and callback_done.wait(2)
        t.join();incoming.join();assert not errors,errors
        # The notice still actively quarantines. It must do so for its
        # transaction record before the next actual callback can replace it.
        assert quarantined==[(old['contract']['source_ns'],True)]
        fresh=n._station_task_record
        assert old['pending'] and fresh is not old and not fresh['pending']
        assert fresh['contract']['binding']==old['contract']['binding']
        assert fresh['contract']['phase_deadline_ns']==old['contract']['phase_deadline_ns']
        assert n._station_requalification_task is None
        assert n._station_contact_admitted is None
    finally:
        resume.set();done.wait(2);t.join(timeout=2)
        if incoming is not None:callback_done.wait(2);incoming.join(timeout=2)


def test_valid_notice_still_quarantines_without_enrolling_claimed_drift(monkeypatch):
    c=computed(context(monkeypatch));n=c.task;value=notice(c);old=n._station_task_record
    contract=deepcopy(old['contract']);clocks=(old['geometry_mono'],old['identity_mono'])
    send(n,value)
    assert n._station_task_record is old and old['pending']
    assert old['contract']==contract and (old['geometry_mono'],old['identity_mono'])==clocks
    assert n._station_requalification_task is None and not n._station_task_retired
    assert n.latest_obj is None and n.latest_target_geometry is None


@pytest.mark.parametrize('bad',['parent','witness'])
def test_invalid_notice_does_not_quarantine_or_enroll(monkeypatch,bad):
    c=computed(context(monkeypatch));n=c.task;value=notice(c);old=n._station_task_record
    if bad=='parent':value[STATION_REQUALIFICATION]['parent_contract']['binding']['source_id']='f'*64
    else:value[STATION_REQUALIFICATION]['feedback_q_rad']=list(old['contract']['binding']['station_q_rad'])
    send(n,value)
    assert n._station_task_record is old and not old['pending']
    assert not n._station_task_retired and n._station_requalification_task is None


@pytest.mark.parametrize('bad',['manual','epoch'])
def test_notice_cannot_override_real_owner_change(monkeypatch,bad):
    c=computed(context(monkeypatch));n=c.task;value=notice(c)
    if bad=='manual':c.params['/gui/joint_direct_mode']=True
    else:n._following_epoch_ns+=1
    send(n,value)
    assert n._station_task_retired and n._station_task_record is None
    assert n._station_contact_admitted is None and n._station_requalification_task is None
