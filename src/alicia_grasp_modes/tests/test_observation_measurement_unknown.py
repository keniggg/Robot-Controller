"""Unknown owner keeps only identity through a bound move; real gates restore geometry."""
from copy import deepcopy
from unittest.mock import Mock
import json
import numpy as np
import pytest
from std_msgs.msg import String,Header
from test_locked_rgb_hold import locked,custom_frame,latest_status,module


def send(n,request):
    n._observation_measurement_request_cb(String(data=json.dumps(request)))


def ack(n):return json.loads(n._observation_measurement_ack_pub.publish.call_args[0][0].data)


def frozen(monkeypatch, camera_profile=False):
    n,clock,feed,joint,params=locked(monkeypatch)
    if camera_profile:
        params['/camera']['runtime_profile']={'stationary_temporal':{'active':False}}
        feed(target_depth=.35)
    n._observation_measurement_ack_pub=Mock()
    request=dict(version=1,state='executing',motion_kind='observation',lease_id='lease1',request_id='request1',session_id='session1',
        selection=deepcopy(params['/grasp_mode/selection']),target_epoch=1,target_track_id='target1',origin_plan_id='plan1',
        origin_snapshot_stamp_ns=n.anchor_source_stamp_ns,view_plan_id='plan2',view_source_stamp_ns=n.last_good_source_stamp_ns,
        view_index=1,plan_sha256='a'*64,control_epoch_ns=n._rgb_interest_epoch,deadline_ns=int((clock[0]+60)*1e9),stamp_ns=module.rospy.Time.now().to_nsec(),
        metric_geometry_authority=False,motion_authorized=False)
    send(n,request)
    assert ack(n)['accepted'],ack(n)
    return n,clock,feed,joint,params,request


def arrival(n,clock,joint,request):
    clock[0]+=2.
    start=clock[0]-.35
    for ns in np.arange(start,clock[0]+.001,.05):joint(float(ns))
    request=deepcopy(request);request.update(state='requalifying',request_id='request2',stamp_ns=module.rospy.Time.now().to_nsec(),
        arrival_stamp_ns=module.rospy.Time.now().to_nsec(),capture_deadline_ns=int((clock[0]+5)*1e9),settle=dict(epoch_ns=n._rgb_interest_epoch,
            stationary_window_start_ns=module.rospy.Time.from_sec(start).to_nsec(),stationary_window_end_ns=module.rospy.Time.now().to_nsec()))
    send(n,request);assert ack(n)['accepted'],ack(n)
    return request


def next_frame(n,clock,joint,**kwargs):
    clock[0]+=.2
    joint(clock[0]-.2);joint(clock[0]-.1)
    return custom_frame(n,clock,joint,target_depth=.35,**kwargs)


def test_executing_suspends_metric_publication_without_losing_original_identity(monkeypatch):
    n,clock,feed,joint,params,request=frozen(monkeypatch)
    anchor,last,plane=n.tracker.anchor,n.tracker.last,n.tracker.plane_base.copy()
    before={key:pub.publish.call_count for key,pub in n.pubs.items() if key!='detector_status'}
    clock[0]+=30.
    feed(target_depth=0.)
    assert n.tracker.anchor is anchor and n.tracker.last is last
    assert np.array_equal(n.tracker.plane_base,plane) and not n.tracker.lost and n.first_loss is None
    assert {key:n.pubs[key].publish.call_count for key in before}==before
    assert latest_status(n)['state']=='pending'
    assert latest_status(n)['observation_measurement']['metric_geometry_authority'] is False
    assert n._locked_rgb_hold_reference is None


def test_three_new_real_frames_publish_only_third_and_complete_same_anchor(monkeypatch):
    n,clock,feed,joint,params,request=frozen(monkeypatch)
    anchor=n.tracker.anchor;plane=n.tracker.plane_base.copy();count=n.pubs['object'].publish.call_count
    request=arrival(n,clock,joint,request);stamps=[]
    for index in range(3):
        pair=next_frame(n,clock,joint);stamps.append(pair[0].header.stamp.to_nsec())
        assert n.pubs['object'].publish.call_count==count+(index==2),latest_status(n)
    assert ack(n)['qualified'] and ack(n)['source_stamps_ns']==stamps
    status=latest_status(n);assert status['state']=='ready' and status['observation_measurement']['state']=='qualified'
    assert status['observation_measurement']['request']==n._observation_measurement['request']
    assert n.pubs['object'].publish.call_args[0][0].header.stamp.to_nsec()==stamps[-1]
    assert n.tracker.anchor is anchor and np.array_equal(n.tracker.plane_base,plane) and not n.tracker.lost
    clock[0]+=.01
    terminal=deepcopy(request);terminal.update(state='qualified',request_id='request3',stamp_ns=module.rospy.Time.now().to_nsec(),source_stamps_ns=stamps)
    send(n,terminal)
    assert n._observation_measurement is None and ack(n)['qualified'] and not n.tracker.lost
    assert latest_status(n)['target_lost'] is False
    clock[0]+=.1;feed(target_depth=.35)
    assert latest_status(n)['detail']=='tracked'


@pytest.mark.parametrize('change',['camera','model','handeye','epoch','manual','selection','config','anchor','lost','deadline'])
def test_frozen_binding_changes_revoke_and_keep_original_loss(monkeypatch,change):
    n,clock,feed,joint,params,request=frozen(monkeypatch)
    if change=='camera':params['/camera']['fx']+=1
    elif change=='model':params['/robot_description']='<robot name="changed"/>'
    elif change=='handeye':params['/handeye']['changed']=True
    elif change=='epoch':n._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time(2),frame_id='sdk_reference_epoch'))
    elif change=='manual':params['/gui/joint_direct_mode']=True
    elif change=='selection':params['/grasp_mode/selection']['generation']+=1
    elif change=='config':n.config.support_threshold_m=.0019
    elif change=='anchor':n.tracker.anchor.position_base[0]+=.001
    elif change=='lost':n.tracker.lost=True
    elif change=='deadline':clock[0]+=61.
    n.process(None)
    assert n._observation_measurement is None and n.tracker.lost
    assert not ack(n)['accepted'] and not n.pubs['object'].publish.call_args[0][0].detected


@pytest.mark.parametrize('failure',['support','ambiguous','rgb','few_points'])
def test_bad_arrival_measurement_resets_three_frame_confirmation_without_replacing_reference(monkeypatch,failure):
    from dataclasses import replace
    import alicia_grasp_modes.observation_arrival_confirmation as helper
    n,clock,feed,joint,params,request=frozen(monkeypatch)
    anchor=n.tracker.anchor;plane=n.tracker.plane_base.copy();count=n.pubs['object'].publish.call_count
    arrival(n,clock,joint,request);next_frame(n,clock,joint)
    assert len(n._observation_measurement['confirmation'].source_stamps_ns)==1
    original=module.segment_with_support_fallback;refine=helper.refine_candidate
    def bad(*args,**kwargs):
        if failure=='support':raise ValueError('support_plane_local_distance_inconsistent: .004410 > .004000')
        result=original(*args,**kwargs)
        if failure=='ambiguous':result.candidates*=2
        return result
    def bad_refine(*args,**kwargs):
        if failure=='rgb':raise ValueError('rgb_seed_component_ambiguous_or_fragmented')
        c=refine(*args,**kwargs)
        return replace(c,metrics=dict(c.metrics,valid_points=66)) if failure=='few_points' else c
    with monkeypatch.context() as change:
        change.setattr(module,'segment_with_support_fallback',bad);change.setattr(helper,'refine_candidate',bad_refine)
        next_frame(n,clock,joint)
    assert n._observation_measurement['confirmation'].source_stamps_ns==[]
    assert n.tracker.anchor is anchor and np.array_equal(n.tracker.plane_base,plane) and not n.tracker.lost
    assert n.pubs['object'].publish.call_count==count
    for _ in range(3):next_frame(n,clock,joint)
    assert ack(n)['qualified'],latest_status(n)


def test_out_and_back_after_arrival_retires_even_when_next_frame_returns_to_same_pose(monkeypatch):
    n,clock,feed,joint,params,request=frozen(monkeypatch)
    arrival(n,clock,joint,request);next_frame(n,clock,joint)
    joint(clock[0]-.01,[.005,0,0,0,0,0]);joint(clock[0],[0.]*6)
    n.process(None)
    assert n.tracker.lost and n._observation_measurement is None
    assert 'movement_after_stationary_arrival' in ack(n)['reason']


def test_dynamic_camera_diagnostics_do_not_revoke_fixed_geometry_contract(monkeypatch):
    n,clock,feed,joint,params,request=frozen(monkeypatch,camera_profile=True)
    params['/camera']['runtime_profile']['stationary_temporal'].update(active=True,filtered_pixel_count=123)
    n.process(None)
    assert n._observation_measurement is not None and not n.tracker.lost


def test_existing_lost_target_cannot_accept_new_measurement_lease(monkeypatch):
    n,clock,feed,joint,params,request=frozen(monkeypatch)
    n._observation_measurement=None;n.tracker.lost=True
    request['lease_id']='different_lease'
    send(n,request)
    assert n.tracker.lost and n._observation_measurement is None and not ack(n)['accepted']


def test_missing_lease_preserves_original_positive_support_contradiction_loss(monkeypatch):
    n,clock,feed,joint,params=locked(monkeypatch)
    def bad(*args,**kwargs):raise ValueError('support_plane_local_distance_inconsistent: .004410 > .004000')
    monkeypatch.setattr(module,'segment_with_support_fallback',bad)
    feed(target_depth=.35)
    assert n.tracker.lost and n.first_loss is not None

@pytest.mark.parametrize('change',['preset','activation','unknown_field'])
def test_fixed_camera_configuration_changes_are_not_hidden_as_telemetry(monkeypatch,change):
    n,clock,feed,joint,params,request=frozen(monkeypatch,camera_profile=True)
    if change=='preset':params['/camera']['runtime_profile']['sdk_visual_preset']='different'
    elif change=='activation':params['/camera']['runtime_profile']['geometry_activation_stamp_ns']=123
    else:params['/camera']['runtime_profile']['stationary_temporal']['new_unknown_configuration']=True
    n.process(None)
    assert n.tracker.lost and n._observation_measurement is None and not ack(n)['accepted']


def test_qualified_source_is_withheld_until_receipt_and_expires_without_it(monkeypatch):
    n,clock,feed,joint,params,request=frozen(monkeypatch)
    arrival(n,clock,joint,request)
    for _ in range(3):next_frame(n,clock,joint)
    count=n.pubs['object'].publish.call_count;stamp=n.last_good_source_stamp_ns
    next_frame(n,clock,joint)
    assert n.pubs['object'].publish.call_count==count and n.last_good_source_stamp_ns==stamp
    clock[0]+=.8;n.process(None)
    assert n.tracker.lost and n._observation_measurement is None and not ack(n)['accepted']


def test_capture_deadline_failure_does_not_reset_or_reacquire_target(monkeypatch):
    n,clock,feed,joint,params,request=frozen(monkeypatch)
    arrival(n,clock,joint,request)
    anchor=n.tracker.anchor;clock[0]+=5.;n.process(None)
    assert n.tracker.anchor is anchor and n.tracker.lost and n._observation_measurement is None
    clock[0]+=.1;feed(target_depth=.35)
    assert n.tracker.lost and not n.pubs['object'].publish.call_args[0][0].detected


def test_first_lease_bad_frozen_context_nacks_without_retiring_valid_target(monkeypatch):
    n,clock,feed,joint,params=locked(monkeypatch)
    # Capture a structurally valid request, then replay against another still-live fixture.
    other,_,_,_,_,request=frozen(monkeypatch)
    n._observation_measurement_ack_pub=Mock()
    params['/robot_description']='<robot name="changed_before_admission"/>'
    monkeypatch.setattr(module.rospy,'get_param_cached',lambda key,default=None:params.get(key,default))
    send(n,request)
    assert not ack(n)['accepted'] and not n.tracker.lost
    assert getattr(n,'_observation_measurement',None) is None


def test_static_feedback_excursion_before_first_arrival_frame_is_not_hidden(monkeypatch):
    n,clock,feed,joint,params,request=frozen(monkeypatch)
    request=arrival(n,clock,joint,request)
    joint(clock[0]+.01,[.005,0,0,0,0,0]);joint(clock[0]+.02,[0.]*6)
    next_frame(n,clock,joint)
    assert n.tracker.lost and n._observation_measurement is None


def test_individually_fresh_identity_history_can_age_while_final_geometry_stays_fresh(monkeypatch):
    n,clock,feed,joint,params,request=frozen(monkeypatch)
    arrival(n,clock,joint,request)
    count=n.pubs['object'].publish.call_count;stamps=[]
    for index in range(3):
        for when in np.arange(clock[0],clock[0]+.401,.05):joint(float(when))
        clock[0]+=.4
        pair=custom_frame(n,clock,joint,target_depth=.35)
        stamps.append(pair[0].header.stamp.to_nsec())
        assert n.pubs['object'].publish.call_count==count+(index==2),latest_status(n)
    now=module.rospy.Time.now().to_nsec()
    assert now-stamps[0]>800_000_000 and now-stamps[-1]<800_000_000
    assert ack(n)['qualified'] and ack(n)['source_stamps_ns']==stamps
    assert n.last_good_source_stamp_ns==stamps[-1]
    assert n.pubs['object'].publish.call_args[0][0].header.stamp.to_nsec()==stamps[-1]
    assert not n.tracker.lost


@pytest.mark.parametrize('delay_at',['audit','hold','commit_lock'])
def test_current_source_expiring_during_work_cannot_commit_any_geometry(monkeypatch,delay_at):
    n,clock,feed,joint,params,request=frozen(monkeypatch)
    arrival(n,clock,joint,request)
    for _ in range(2):next_frame(n,clock,joint)
    pending=n._observation_measurement
    count=n.pubs['object'].publish.call_count;last=n.tracker.last
    old_good=n.last_good_source_stamp_ns
    armed=[False]
    if delay_at=='audit':
        original=pending['confirmation'].observe
        def slow_audit(*args,**kwargs):
            result=original(*args,**kwargs);clock[0]+=.81;return result
        monkeypatch.setattr(pending['confirmation'],'observe',slow_audit)
    else:
        original=n._prepare_locked_rgb_hold_reference
        def slow_hold(*args,**kwargs):
            result=original(*args,**kwargs)
            assert result is not None
            if delay_at=='hold':clock[0]+=.81
            else:armed[0]=True
            return result
        monkeypatch.setattr(n,'_prepare_locked_rgb_hold_reference',slow_hold)
        if delay_at=='commit_lock':
            original_lock=n.input_lock
            class DelayedLock:
                def __enter__(self):
                    original_lock.acquire()
                    if armed[0]:clock[0]+=.81;armed[0]=False
                    return self
                def __exit__(self,*args):original_lock.release()
            n.input_lock=DelayedLock()
    next_frame(n,clock,joint)
    assert pending['confirmation'].source_stamps_ns==[]
    assert not pending.get('qualified_stamps') and not ack(n)['qualified']
    assert n.pubs['object'].publish.call_count==count and n.tracker.last is last
    assert n.last_good_source_stamp_ns==old_good and n._locked_rgb_hold_reference is None
    assert latest_status(n)['observation_measurement']['reason']=='source_exceeds_existing_frame_limit'
