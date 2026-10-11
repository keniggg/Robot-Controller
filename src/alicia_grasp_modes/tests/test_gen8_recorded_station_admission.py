"""Original GEN8 messages through actual Mode callbacks and station joins.

Historical qualified context is explicitly seeded from its original receipts;
this is not a reconstruction of omitted sensor frames or a robot execution.
The synthetic worker/fuser integration is tested in its separate module.
"""
import base64
from copy import deepcopy
import hashlib
import io
import itertools
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
import roslib.message
from std_msgs.msg import String

from test_station_compute_wait_remote import station
from test_observation_measurement_remote import at, context
from test_remote_modes import base,module
from alicia_grasp_modes.selection import SELECTION_PARAM
from alicia_grasp_modes.observe_return_session import parse_session

PATH=Path(__file__).parent/'fixtures/gen8_recorded_station_admission.json'
SHA='6a434361e2258001ecb1335284e9c7c3e8e267aa06cd8788231dfb46286ebeb2'
SOURCES=(1791602440062703132,1791602440390553236,1791602440792152404)


@pytest.fixture
def recorded():
    raw=PATH.read_bytes();assert hashlib.sha256(raw).hexdigest()==SHA
    data=json.loads(raw);events=[]
    for row in data['events']:
        raw=base64.b64decode(row['roswire_base64'])
        assert hashlib.sha256(raw).hexdigest()==row['wire_sha256']
        cls=roslib.message.get_message_class(row['ros_type'])
        assert cls is not None and cls._md5sum==row['ros_md5']
        msg=cls();msg.deserialize(raw);b=io.BytesIO();msg.serialize(b)
        assert b.getvalue()==raw
        events.append((row,msg))
    return events


@pytest.fixture
def original_station(station,recorded,monkeypatch):
    c=station;n=c.node
    rows={ns:{} for ns in SOURCES}
    for event,msg in recorded:
        topic=event['topic']
        if topic=='/perception/unknown/detector_status':
            payload=json.loads(msg.data);ns=payload.get('source_stamp_ns')
            if ns in rows:rows[ns]['status']=msg;rows[ns]['payload']=payload
        elif topic in ('/perception/object','/perception/object_mask'):
            ns=msg.header.stamp.to_nsec()
            if ns in rows:rows[ns]['object' if topic.endswith('/object') else 'mask']=msg
        else:continue
        if ns in rows:rows[ns].setdefault('received',{})[topic.rsplit('/',1)[-1]]=event['recorder_received_ns']
    for row in rows.values():assert {'object','mask','status','payload','received'}<=set(row)
    phase=next(msg for event,msg in recorded if event['topic']=='/grasp/near_field_phase' and msg.active)
    requests=[json.loads(msg.data) for event,msg in recorded if event['topic']=='/grasp_mode/observe_return_measurement_request']
    qualified=next(p for p in requests if p['state']=='qualified')
    session=next(json.loads(msg.data) for event,msg in recorded if event['topic']=='/grasp_mode/observe_return_session')
    contract=rows[SOURCES[0]]['payload']['station_compute_wait'];binding=contract['binding']
    n._reset_station_compute_wait();n._initialize_unknown_pending_identity()
    n._unknown_identity_publication_started=True
    n._mode_selection=deepcopy(binding['selection']);c.params[SELECTION_PARAM]=deepcopy(binding['selection'])
    n._target_identity_generation=1;n.target_instance_epoch=4
    assert n._current_stream_target_identity().track_id==binding['target_track_id']=='g1-t4'
    n.inference_coordinator.reset_target_epoch(4)
    n._station_compute_phase=deepcopy(phase);n._station_compute_task_state='PLAN_PREGRASP'
    n._measurement_remote_request=deepcopy(qualified)
    n._measurement_remote_reference['binding']=binding['reference_binding']
    n._measurement_remote_epoch_ns=binding['control_epoch_ns']
    n._observe_return_session=parse_session(session,binding['selection'],SOURCES[0])
    n._observe_return_phase='observing_contact';n._observe_return_paused=False
    n._observe_return_diagnostic_observation_only=False
    n.latest_object=deepcopy(rows[SOURCES[0]]['object']);n.latest_object_time=None
    n.near_field_planning_snapshot_frames=3
    n._configured_near_field_strategy=lambda:'single_snapshot_direct'
    n.last_submitted_stamp_ns=phase.header.stamp.to_nsec()
    n.near_field_planning_active=True;n._near_field_phase_deadline_sec=phase.deadline.to_nsec()/1e9
    n._near_field_phase_started_sec=phase.header.stamp.to_nsec()/1e9
    n._near_field_reference_target_track_id=binding['target_track_id']
    now=[SOURCES[0]]
    monkeypatch.setattr(n,'_ros_source_clock_ns',lambda:now[0])
    monkeypatch.setattr(module.rospy.Time,'now',lambda:at(now[0]))
    n.frames=base.SynchronizedRgbdBuffer(source_clock_ns=lambda:now[0])
    feedback=[(e,m) for e,m in recorded if e['topic']=='/alicia_d/accepted_joint_states']
    seen=set()
    def advance(ns):
        assert ns>=now[0]
        now[0]=ns;c.clock[0]=ns/1e9
        for event,msg in feedback:
            stamp=msg.header.stamp.to_nsec()
            if event['recorder_received_ns']<=ns and stamp not in seen:
                n._station_compute_accepted_cb(deepcopy(msg));seen.add(stamp)
                n.latest_joint_state=deepcopy(msg)
    def emit(row,key):
        if key=='status':n._silhouette_source_status_cb(deepcopy(row['status']))
        else:getattr(n,key+'_cb')(deepcopy(row[key]))
    return NS(c=c,node=n,rows=rows,advance=advance,emit=emit,contract=contract,phase=phase,
              events=recorded,now=now,feedback=feedback)


@pytest.mark.parametrize('order',list(itertools.permutations(('object','mask','status'))))
def test_original_ready_joins_all_orders_keep_only_nonexecuting_history(original_station,order):
    r=original_station;n=r.node
    for index,source in enumerate(SOURCES):
        row=r.rows[source];r.advance(max(row['received'].values()))
        assert n._station_compute_scope()
        n._station_compute_context(row['payload']['station_compute_wait'])
        n._station_compute_check_joints(row['payload']['station_compute_wait']['binding'], source)
        if index==0:n._station_compute_check_joints(row['payload']['station_compute_wait']['binding'], n._measurement_remote_request['source_stamps_ns'][-1])
        # These are exact historical receipt times; only the callback order is
        # deliberately permuted to exercise the asynchronous boundary.
        previous=n._station_compute_reference
        for key in order[:-1]:
            r.emit(row,key)
            if index==0:assert n._station_compute_reference is None
            else:assert n._station_compute_reference is previous
            assert not getattr(n,'latest_object_geometry',NS(valid=False)).valid
        r.emit(row,order[-1])
        assert n._station_compute_reference is not None
        assert n._station_compute_reference['contract']==row['payload']['station_compute_wait']
        assert not n._station_compute_pending and not n._station_compute_retired
        assert n.latest_object.header.stamp.to_nsec()==source
        assert n._current_stream_target_identity().track_id=='g1-t4'
        assert not getattr(n,'latest_object_geometry',NS(valid=False)).valid
    assert set(SOURCES)<=set(n._station_compute_ready_history)
    assert n._station_compute_cutoff_ns<SOURCES[0]
    assert n._stream_worker_ticket is None
    q=[v for ns,v in n._station_compute_feedback if ns>=r.c.node._measurement_remote_request['source_stamps_ns'][-1]]
    assert q and np.max(np.ptp(np.asarray(q),axis=0))==0


def test_exact_recorded_deadline_expires_without_renewal(original_station):
    r=original_station;n=r.node;row=r.rows[SOURCES[0]]
    r.advance(max(row['received'].values()))
    for key in ('object','mask','status'):r.emit(row,key)
    assert n._station_compute_reference is not None
    original=deepcopy(n._station_compute_reference['contract'])
    r.advance(original['identity_deadline_ns'])
    n._station_compute_poll()
    assert n._station_compute_retired
    assert n._station_compute_reference is None
    assert not n.latest_object_geometry.valid
    assert original['identity_deadline_ns']==SOURCES[0]+5_000_000_000


def test_actual_recorded_terminal_false_remains_hard_rejection(original_station):
    r=original_station;n=r.node;row=r.rows[SOURCES[0]]
    r.advance(max(row['received'].values()))
    for key in ('object','mask','status'):r.emit(row,key)
    event,msg=next((e,m) for e,m in r.events if e['topic']=='/perception/object' and not m.detected)
    assert msg.header.stamp.to_nsec()==1791602529733577489
    # This real false arrives after the recorded phase expiry; it is a combined
    # terminal case. Unexpired false isolation belongs to synthetic negatives.
    assert event['recorder_received_ns']>r.phase.deadline.to_nsec()
    r.advance(event['recorder_received_ns']);n.object_cb(deepcopy(msg))
    assert n._station_compute_retired and not n.latest_object_geometry.valid
    assert n._station_compute_reference is None
