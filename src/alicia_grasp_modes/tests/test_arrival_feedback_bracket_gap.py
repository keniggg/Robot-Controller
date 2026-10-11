"""GEN18 timing, synthetic RGB-D, actual arrival enrollment and process chain."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
from sensor_msgs.msg import JointState
from alicia_flexible_grasp_supervisor.msg import GraspState

from alicia_grasp_modes.feedback_bracket_gap import validate_station_feedback_gap
from alicia_grasp_modes.rgb_observation_source import bracket_feedback
from test_station_compute_wait_producer import armed
from test_locked_rgb_hold import module, custom_frame
from test_unknown_pending_identity import publication

RECORD = json.loads((Path(__file__).parent/'fixtures/gen18_arrival_feedback_gap.json').read_text())


def scene(monkeypatch):
    p, clock, joint, params, marker, source = armed(monkeypatch)
    return NS(p=p, clock=clock, joint=joint, params=params, ready=publication(p),
        shift=RECORD['reference_ns']-p.last_good_source_stamp_ns, lookup=p.buffer.lookup_transform)


def frame(s, *, good=False, fault=''):
    p=s.p; row=RECORD['next'] if good else RECORD
    ns,lo,hi=[row[key]-s.shift for key in ('source_ns','before_ns','after_ns')]
    finish=row['finish_ns' if good else 'failure_ns']-s.shift
    if fault=='expired':finish=p._station_compute_registration['contract']['identity_deadline_ns']+1
    if fault=='source_expired':finish=ns+800000001
    s.clock[0]=(ns+17918587)/1e9
    def accepted(stamp,q=None):
        m=JointState();m.header.stamp=module.rospy.Time(stamp//10**9,stamp%10**9)
        m.header.frame_id='sdk_measured';m.name=list(module.JOINT_NAMES);m.position=[0.]*6 if q is None else q
        p._rgb_interest_joint_cb(m)
    if fault.startswith('missing'):p._rgb_interest_feedback.clear()
    if fault not in ('missing_before','missing_both'):accepted(lo)
    fired=[]
    def lookup(*args):
        if args[0]=='base_link' and not fired:
            fired.append(True);s.clock[0]=finish/1e9
            if fault not in ('missing_after','missing_both'):
                accepted(hi,[.005,0.,0.,0.,0.,0.] if fault=='pose' else None)
            if fault=='out_back':accepted(hi-1000,[.006,0.,0.,0.,0.,0.]);accepted(hi)
            if fault=='manual':s.params['/gui/joint_direct_mode']=True
            if fault=='epoch':p._rgb_interest_epoch+=1
            if fault=='camera':s.params['/camera']['fx']+=1
            if fault=='selection':s.params['/grasp_mode/selection']['generation']+=1
            if fault=='phase':p._station_compute_phase.phase_id+=1
            if fault=='state':p._station_compute_state_cb(GraspState(state='MOVE_PREGRASP',active=True))
            if fault=='no_registration':p._station_compute_registration=None
        return s.lookup(*args)
    p.buffer.lookup_transform=lookup
    rgb=np.full((240,320,3),(60,90,190),np.uint8);rgb[106:134,148:173]=20
    depth=np.full((240,320),.4,np.float32);depth[106:134,148:173]=.35
    if fault=='bad_support':depth[:]=0.
    pair=(p.bridge.cv2_to_imgmsg(rgb,encoding='bgr8'),p.bridge.cv2_to_imgmsg(depth,encoding='32FC1'))
    for m in pair:m.header.stamp=module.rospy.Time(ns//10**9,ns%10**9);m.header.frame_id=p.camera_frame
    p.latest=pair;p.process(None)
    return publication(p)


def test_recorded_bad_bracket_still_fails_and_next_actual_bracket_passes():
    r=RECORD;q=r['q']
    with pytest.raises(ValueError,match='accepted_feedback_does_not_bracket_source'):
        bracket_feedback([(r['before_ns'],q),(r['after_ns'],q)],r['source_ns'],r['selection_ns'])
    n=r['next'];_,lo,hi=bracket_feedback([(n['before_ns'],q),(n['after_ns'],q)],n['source_ns'],r['selection_ns'])
    assert hi[0]-lo[0]==99916930


def test_actual_arrival_bad_frame_quarantines_then_full_new_frame_recovers_past_old_geometry(monkeypatch):
    s=scene(monkeypatch);p=s.p;old=p._locked_rgb_hold_reference;last=p.tracker.last
    reg=p._station_compute_registration;contract=deepcopy(reg['contract']);mono=reg['monotonic_deadline']
    bad=frame(s);status=bad['status']
    assert status['state']=='processing_deadline_pending',status
    assert status['detail']=='station_accepted_feedback_bracket_pending_revalidation'
    assert status['station_compute_wait']==contract and not bad['object'].detected
    assert not np.any(p.bridge.imgmsg_to_cv2(bad['mask'],'mono8'))
    proof=status['station_feedback_gap'];assert proof['feedback_gap_evidence']['span_ns']==226932289
    assert validate_station_feedback_gap(proof,contract=contract,now_ns=module.rospy.Time.now().to_nsec())==(bad['object'].header.stamp.to_nsec(),)
    assert p._locked_rgb_hold_reference is old and p.tracker.last is last
    assert p.last_good_source_stamp_ns==old['source_stamp_ns'] and not p.tracker.lost and p.first_loss is None
    assert p._station_compute_registration is reg and reg['monotonic_deadline']==mono
    new=frame(s,good=True);status=new['status']
    assert module.rospy.Time.now().to_nsec()>contract['geometry_deadline_ns']
    assert status['state']=='ready',status
    assert status['station_compute_recovery']==contract and status['station_feedback_gap_recovery']==proof
    assert new['object'].detected and p.last_good_source_stamp_ns==new['object'].header.stamp.to_nsec()


@pytest.mark.parametrize('fault',['missing_before','missing_after','missing_both','pose','out_back',
    'manual','epoch','camera','selection','phase','state','no_registration','expired','source_expired'])
def test_bad_feedback_does_not_create_gap_authority_for_invalid_scope(monkeypatch,fault):
    s=scene(monkeypatch);count=s.p.pubs['object'].publish.call_count;row=frame(s,fault=fault)
    if fault=='state':
        # A genuine active planning->motion retirement drops the in-flight
        # frame without publishing any new measurement or pending authority.
        assert s.p.pubs['object'].publish.call_count==count
        assert s.p._station_compute_registration is None
        return
    assert not row['object'].detected
    assert 'station_feedback_gap' not in row['status']
    assert row['status']['state']!='ready'


@pytest.mark.parametrize('expired_reference',[False,True])
def test_even_timely_reference_requires_complete_fresh_support_after_gap(monkeypatch,expired_reference):
    s=scene(monkeypatch);bad=frame(s);assert 'station_feedback_gap' in bad['status']
    old=s.p._locked_rgb_hold_reference;last=s.p.tracker.last
    if not expired_reference:s.shift+=200000000
    original=module.segment_with_support_fallback
    def unsupported(*args,**kwargs):
        result=original(*args,**kwargs);result.metrics['support_current_audit_inlier_ratio']=.01;return result
    monkeypatch.setattr(module,'segment_with_support_fallback',unsupported)
    row=frame(s,good=True)
    assert not row['object'].detected and row['status']['state']!='ready'
    assert s.p.last_good_source_stamp_ns==old['source_stamp_ns'] and s.p.tracker.last is last


def test_repeated_gap_never_extends_registered_identity_or_monotonic_deadline(monkeypatch):
    s=scene(monkeypatch);reg=s.p._station_compute_registration
    contract=deepcopy(reg['contract']);mono=reg['monotonic_deadline']
    a=frame(s);s.shift-=400000000;b=frame(s)
    assert b['status']['station_compute_wait']==contract
    assert reg['monotonic_deadline']==mono and len(b['status']['station_feedback_gap']['discarded_source_stamps_ns'])==2
    s.clock[0]=contract['identity_deadline_ns']/1e9+.001;s.joint(s.clock[0]-.01)
    s.p.latest=None;s.p.process(None)
    assert s.p.first_loss is not None and s.p._station_compute_registration is None


def test_changed_scope_before_pending_publish_cannot_keep_identity(monkeypatch):
    s=scene(monkeypatch);original=s.p._station_compute_publish_feedback_gap
    def changed(source,error):
        s.params['/gui/joint_direct_mode']=True;return original(source,error)
    monkeypatch.setattr(s.p,'_station_compute_publish_feedback_gap',changed)
    row=frame(s)
    assert not row['object'].detected and 'station_feedback_gap' not in row['status'] and s.p.first_loss is not None


def test_source_expires_during_pending_publication_uses_original_deadline_wait(monkeypatch):
    s=scene(monkeypatch);reg=s.p._station_compute_registration;original=s.p._station_compute_publish_feedback_gap
    def late(source,error):
        s.clock[0]=(source[0].header.stamp.to_nsec()+800000001)/1e9
        s.joint(s.clock[0]-.01)
        return original(source,error)
    monkeypatch.setattr(s.p,'_station_compute_publish_feedback_gap',late)
    row=frame(s)
    assert row['status']['state']=='processing_deadline_pending',row['status']
    assert row['status']['detail']=='processing_deadline_pending'
    assert 'station_feedback_gap' not in row['status'] and not row['object'].detected
    assert s.p._station_compute_registration is reg and s.p.first_loss is None


@pytest.mark.parametrize('gate',['_station_compute_recovery_evidence','_station_compute_commit_check'])
def test_late_typed_bracket_failure_never_partially_commits_measurement(monkeypatch,gate):
    # Fault injection at the real post-refinement and commit boundaries. This
    # tests atomic discard, not a claim that an earlier valid bracket vanished.
    from alicia_grasp_modes.station_compute_wait_producer import StationFeedbackBracketGap
    s=scene(monkeypatch);p=s.p;reg=p._station_compute_registration
    old=p._locked_rgb_hold_reference;last=p.tracker.last
    def reject(source,*args):
        ns=source[0].header.stamp.to_nsec();lo=ns-30000000;hi=lo+226932289
        s.clock[0]=(hi+1000000)/1e9;s.joint(s.clock[0]-.0001)
        evidence=dict(accepted_before=[lo,[0.]*6],accepted_after=[hi,[0.]*6],
            span_ns=226932289,maximum_span_ns=200000000,bracket_qualified=False)
        raise StationFeedbackBracketGap(reg,evidence)
    monkeypatch.setattr(p,gate,reject)
    row=frame(s,good=True)
    assert row['status']['state']=='processing_deadline_pending',row['status']
    assert not row['object'].detected and p.tracker.last is last
    assert p._locked_rgb_hold_reference is old and p.last_good_source_stamp_ns==old['source_stamp_ns']
    assert p.first_loss is None
