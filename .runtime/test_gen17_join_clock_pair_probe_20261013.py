"""Narrow audit probe; delay only after the real task's final registration check."""
from copy import deepcopy
import json,os
from pathlib import Path
import pytest
from test_gen11_contact_compute_admission_independent import (context,computed,audit,measured_row,copy_candidate,freeze,retirement,feed_feedback)
from alicia_grasp_modes import station_compute_wait_task as wait

@pytest.mark.parametrize('ros_advances',[True,False])
def test_final_gates_after_join_clock_pair_delay(monkeypatch,tmp_path,ros_advances):
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
                mono[0]+=.75
                if ros_advances:c.clock[0]+=.75;c.joint(c.clock[0]);feed_feedback(c)
        return result
    monkeypatch.setattr(wait,'validate_registration',checked)
    mono[0]+=.4
    measured_row(c,step=.4)
    assert len(delayed)==1
    record=n._station_task_record
    before=dict(contract=deepcopy(record['contract']),geometry_mono=record['geometry_mono'],identity_mono=record['identity_mono'],mono=mono[0],ros_clock=c.clock[0])
    check,plan=copy_candidate(c)
    outcome=dict(ros_advances=ros_advances,delay_sec=.75,delayed=delayed,joined=before,
        task_copy=dict(ok=check.ok,code=check.code,reason=check.reason),freeze_succeeded=False,
        action_sentinel_count=0,exception=None)
    if check.ok:
        try:
            frozen=freeze(c);outcome['freeze_succeeded']=True
            retirement(c,frozen);outcome['action_sentinel_count']=len(c.motion_sentinel)
        except Exception as exc:
            outcome['exception']=type(exc).__name__+': '+str(exc)
            outcome['action_sentinel_count']=len(getattr(c,'motion_sentinel',[]))
    if ros_advances:
        assert not check.ok and check.code==wait.PENDING
        assert record['pending'] and not n._station_task_retired
        assert not outcome['action_sentinel_count']
    out=Path(os.environ['GEN17_PAIR_OUTPUT']);(out/('advancing.json' if ros_advances else 'frozen.json')).write_text(json.dumps(outcome,indent=2)+'\n')
