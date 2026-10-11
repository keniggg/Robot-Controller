"""Recorded GEN7 control contracts plus real producer gates on synthetic RGBD.

The recording ended before a second executing request. It cannot demonstrate
second-view execution/requalification; that lifecycle has a separate synthetic
sensor test. Recorder receipt times are not callback ingress times.
"""
import base64
from copy import deepcopy
import hashlib
import io
import json
from pathlib import Path
import threading

import numpy as np
import pytest
import roslib.message

from alicia_grasp_modes.station_compute_reacquisition import validate_contract
from test_station_compute_wait_producer import armed
from test_locked_rgb_hold import custom_frame, latest_status

PATH=Path(__file__).parent/'fixtures/gen7_station_handoff_control.json'
EXPECTED_SHA='15bd81ccf69eb4d04fdd88f21fd93f33d090b40f23795d933adf6dce1c63cc14'


@pytest.fixture
def recorded():
    raw=PATH.read_bytes()
    assert hashlib.sha256(raw).hexdigest()==EXPECTED_SHA
    fixture=json.loads(raw)
    events=[]
    for row in fixture['events']:
        raw=base64.b64decode(row['roswire_base64'])
        assert hashlib.sha256(raw).hexdigest()==row['wire_sha256']
        cls=roslib.message.get_message_class(row['ros_type'])
        assert cls is not None and cls._md5sum==row['ros_md5']
        msg=cls();msg.deserialize(raw)
        stream=io.BytesIO();msg.serialize(stream)
        assert stream.getvalue()==raw
        events.append((row,msg))
    return fixture,events


def test_recorded_contract_and_current_source_are_both_fresh_at_actual_failure(recorded):
    f,events=recorded
    contract=f['contract'];at=f['first_loss_ros_ns'];source=f['first_loss_source_ns']
    assert validate_contract(contract,binding=contract['binding'],now_ns=at,source_ns=source)=='ARMED'
    assert at-contract['source_ns']<800_000_000
    assert at-source<800_000_000
    assert f['arrival_station_invalidated_reason']=='station_compute_wait_station_changed'
    active=next(m for row,m in events if row['topic']=='/grasp/near_field_phase' and m.active)
    assert active.reference_target_track_id==contract['binding']['target_track_id']=='g1-t4'
    assert active.phase_id==contract['binding']['phase_id']==1
    assert active.header.stamp.to_nsec()==contract['binding']['phase_started_ns']
    assert active.reference_source_stamp.to_nsec()==contract['binding']['phase_reference_source_ns']
    assert active.deadline.to_nsec()==contract['phase_deadline_ns']
    unchanged=deepcopy(contract)
    assert validate_contract(contract,binding=contract['binding'],now_ns=contract['geometry_deadline_ns'])=='PENDING'
    with pytest.raises(ValueError,match='expired'):
        validate_contract(contract,binding=contract['binding'],now_ns=contract['identity_deadline_ns'])
    assert contract==unchanged


def test_recorded_joint_feedback_and_lease_distinguish_phase_change_from_motion(recorded):
    f,events=recorded;at=f['first_loss_ros_ns']
    feedback=[m for row,m in events if row['topic']=='/alicia_d/accepted_joint_states' and row['recorder_received_ns']<=at]
    assert len(feedback)>=9
    station=np.asarray(f['contract']['binding']['station_q_rad'])
    first=None
    for m in feedback:
        assert m.header.frame_id=='sdk_measured'
        by_name=dict(zip(m.name,m.position))
        q=np.array([by_name['Joint%d'%j] for j in range(1,7)])
        if first is None:first=q
        # Feedback is bit-identical across samples. The contract's Joint2
        # differs by one binary64 ULP (5.551115123125783e-17 rad).
        assert np.array_equal(q,first)
        assert np.all(np.abs(q-station)<=np.spacing(np.abs(station)))
    requests=[json.loads(m.data) for row,m in events if row['topic']=='/grasp_mode/observe_return_measurement_request']
    assert [p['state'] for p in requests]==['executing','requalifying','qualified']
    assert {p['view_index'] for p in requests}=={1}
    assert {p['lease_id'] for p in requests}=={f['contract']['binding']['lease_id']}
    sessions=[json.loads(m.data) for row,m in events if row['topic']=='/grasp_mode/observe_return_session']
    assert [p['view_index'] for p in sessions]==[1,2]
    assert all(p['phase']=='observing' for p in sessions)
    assert f['contract']['motion_authorized'] is False
    assert f['contract']['metric_geometry_authority'] is False


def replay_recorded_state_at_segment(monkeypatch,node,clock,joint,message):
    """Inject an unchanged recorded control message at an explicit barrier.

    RGBD and producer setup are synthetic. This is a race regression with a
    recorded payload, not an assertion about real callback entry timestamps.
    """
    ready=threading.Event();resume=threading.Event();errors=[]
    original=node._frame_timing_stage
    def timing(stage,*args,**kwargs):
        result=original(stage,*args,**kwargs)
        if stage=='post_segment':
            ready.set();assert resume.wait(2.)
        return result
    def run():
        try:custom_frame(node,clock,joint,target_depth=.35)
        except Exception as exc:errors.append(exc)
    clock[0]+=.2
    with monkeypatch.context() as change:
        change.setattr(node,'_frame_timing_stage',timing)
        thread=threading.Thread(target=run);thread.start()
        assert ready.wait(2.)
        try:node._station_compute_state_cb(message)
        finally:resume.set()
        thread.join(2.)
        assert not thread.is_alive() and not errors,errors


@pytest.mark.parametrize('which',['admission','failed_active','failed_inactive'])
def test_recorded_task_callback_cancels_calculation_but_never_reopens_ended_task(monkeypatch,recorded,which):
    f,events=recorded
    states=[m for row,m in events if row['topic']=='/grasp/state']
    message=next(m for m in states if
        (which=='admission' and m.message=='admitting supplemental observation view') or
        (which=='failed_active' and m.state=='FAILED' and m.active) or
        (which=='failed_inactive' and m.state=='FAILED' and not m.active))
    n,clock,joint,params,marker,source=armed(monkeypatch)
    reference=n._locked_rgb_hold_reference;last=n.tracker.last
    source_before=n.last_good_source_stamp_ns
    object_count=n.pubs['object'].publish.call_count
    original_contract=deepcopy(n._station_compute_registration['contract'])
    replay_recorded_state_at_segment(monkeypatch,n,clock,joint,message)
    assert n.last_good_source_stamp_ns==source_before
    if which=='admission':
        assert n.first_loss is None and not n.tracker.lost
        assert n._locked_rgb_hold_reference is reference and n.tracker.last is last
        assert n.pubs['object'].publish.call_count==object_count
        assert n._station_compute_registration is None and n._station_compute_frame is None
        assert reference['observation_arrival_retry'] is marker
        assert marker['session_deadline_ns']==original_contract['session_deadline_ns']
        assert n._station_compute_task_state==message
    else:
        assert n.first_loss is not None and n.tracker.lost
        assert not n.pubs['object'].publish.call_args[0][0].detected


def test_recorded_admission_reproduces_failure_when_old_registration_is_untyped(monkeypatch,recorded):
    _,events=recorded
    message=next(m for row,m in events if row['topic']=='/grasp/state' and m.message=='admitting supplemental observation view')
    n,clock,joint,params,marker,source=armed(monkeypatch)
    monkeypatch.setattr(n,'_station_compute_retire_registration',lambda:setattr(n,'_station_compute_registration',None))
    replay_recorded_state_at_segment(monkeypatch,n,clock,joint,message)
    assert n.first_loss is not None and n.tracker.lost
    assert latest_status(n)['frame_timing']['arrival_station_invalidated_reason']=='station_compute_wait_station_changed'
