"""Real producer pixels/segmentation/refinement and original short support hold."""
from copy import deepcopy
import json
from pathlib import Path
import numpy as np
import pytest
from test_station_compute_wait_producer import armed
from test_support_measurement_hold import support_gap_depth
from test_locked_rgb_hold import custom_frame, latest_status, module


@pytest.mark.parametrize('repeat', [False, True])
def test_support_gap_uses_registered_parent_and_complete_fresh_recovery(monkeypatch, repeat):
    n,clock,joint,params,marker,_=armed(monkeypatch)
    parent=deepcopy(latest_status(n)['station_compute_wait']); registration=n._station_compute_registration
    ref=n._locked_rgb_hold_reference; anchor=n.tracker.anchor; last=n.tracker.last
    clock[0]+=.15
    custom_frame(n,clock,joint,depth_image=support_gap_depth())
    pending=latest_status(n)
    assert pending['state']=='processing_deadline_pending',pending
    assert pending['detail']=='support_measurement_pending_revalidation'
    assert pending['station_compute_wait']==parent
    gap=pending['station_support_gap']
    assert gap['deadline_ns']==parent['geometry_deadline_ns']
    assert n._station_compute_registration is registration
    assert n._locked_rgb_hold_reference is ref and n.tracker.last is last
    assert not n.pubs['object'].publish.call_args[0][0].detected
    assert not np.any(n.bridge.imgmsg_to_cv2(n.pubs['object_mask'].publish.call_args[0][0]))
    assert n.first_loss is None and not n.tracker.lost and n._locked_source_pending is None
    assert pending['source_qualification']['identity_verified'] is False
    mono=registration['support_gap_monotonic']
    if repeat:
        clock[0]+=.1;custom_frame(n,clock,joint,depth_image=support_gap_depth())
        again=latest_status(n)['station_support_gap']
        assert again['first_source_ns']==gap['first_source_ns']
        assert again['deadline_ns']==gap['deadline_ns']
        assert registration['support_gap_monotonic']==mono
        gap=again
    clock[0]+=.15
    custom_frame(n,clock,joint,target_depth=.35)
    ready=latest_status(n)
    assert ready['state']=='ready',ready
    assert ready['station_compute_recovery']==parent
    assert ready['station_support_gap_recovery']==gap
    assert ready['station_compute_wait']['source_ns']<gap['deadline_ns']
    assert n.tracker.anchor is anchor and n.first_loss is None
    assert n._station_compute_registration.get('support_gap') is None


@pytest.mark.parametrize('fault', ['expired','manual','moved','epoch','camera','rgb','support_conflict'])
def test_support_gap_cannot_extend_time_or_bypass_current_recovery(monkeypatch,fault):
    n,clock,joint,params,marker,_=armed(monkeypatch)
    parent=deepcopy(latest_status(n)['station_compute_wait']); previous=n.last_good_source_stamp_ns
    clock[0]+=.15;custom_frame(n,clock,joint,depth_image=support_gap_depth())
    assert latest_status(n)['state']=='processing_deadline_pending'
    clock[0]+=.15
    kwargs=dict(target_depth=.35)
    if fault=='expired':clock[0]=parent['geometry_deadline_ns']/1e9+.001
    elif fault=='manual':params['/gui/joint_direct_mode']=True
    elif fault=='moved':kwargs['q']=[.005,0,0,0,0,0]
    elif fault=='epoch':n._rgb_interest_epoch+=1
    elif fault=='camera':params['/camera']['fx']+=1
    elif fault=='rgb':kwargs['rgb_missing']=True
    else:
        original=module.segment_with_support_fallback
        def conflicting(*a,**k):
            result=original(*a,**k)
            result.metrics['support_reference_max_distance_m']=.1
            result.metrics['support_current_audit_available']=False
            result.metrics['support_current_audit_reason']='insufficient_support_spatial_coverage'
            return result
        monkeypatch.setattr(module,'segment_with_support_fallback',conflicting)
    custom_frame(n,clock,joint,**kwargs)
    assert latest_status(n)['state']!='ready'
    assert n.last_good_source_stamp_ns==previous
    assert not n.pubs['object'].publish.call_args[0][0].detected


def test_recorded_support_pending_was_not_target_loss_and_recovered_before_fixed_cap():
    recorded=json.loads((Path(__file__).parent/'fixtures/gen19_support_gap_messages.json').read_text())
    messages=recorded['messages']
    assert messages
    statuses=[v['payload'] for v in messages.values() if v.get('topic')=='/perception/unknown/detector_status']
    gap=next(s for s in statuses if s['detail']=='support_measurement_pending_revalidation')
    assert gap['target_lost'] is False and gap.get('first_loss') is None
    assert gap['source_qualification']['reason']=='insufficient_support_spatial_coverage'
    assert gap.get('station_compute_wait') is None  # Actual missing bridge, not repaired historical data.
    parent=next(s['station_compute_wait'] for s in statuses if s.get('station_compute_wait'))
    recovered=next(s for s in statuses if (s.get('source_qualification') or {}).get('state')=='recovered_unique_measured_frame')
    assert recovered['source_stamp_ns']<parent['geometry_deadline_ns']
    assert recovered['source_qualification']['identity_verified'] is True


@pytest.mark.parametrize('clock_kind', ['ros', 'monotonic'])
def test_ready_final_registration_cannot_cross_short_support_deadline(monkeypatch, clock_kind):
    from alicia_grasp_modes import station_compute_wait_producer as implementation
    n,clock,joint,params,marker,_=armed(monkeypatch)
    clock[0]+=.15;custom_frame(n,clock,joint,depth_image=support_gap_depth())
    parent=n._station_compute_registration
    assert parent['support_gap']['deadline_ns'] == parent['contract']['geometry_deadline_ns']
    original=n._station_requalification_finalizing_check
    crossed=[]
    def finish_then_expire(binding=None):
        original(binding)
        if binding is None:
            return
        assert n._station_compute_frame['recovery_evidence']
        if clock_kind=='ros':
            clock[0]=parent['support_gap']['deadline_ns']/1e9+.001
        else:
            monkeypatch.setattr(implementation.time,'monotonic',lambda:parent['support_gap_monotonic']+.001)
        crossed.append(True)
    monkeypatch.setattr(n,'_station_requalification_finalizing_check',finish_then_expire)
    clock[0]+=.15;custom_frame(n,clock,joint,target_depth=.35)
    assert crossed
    assert n._station_compute_registration is None
    assert latest_status(n).get('station_compute_wait') is None
