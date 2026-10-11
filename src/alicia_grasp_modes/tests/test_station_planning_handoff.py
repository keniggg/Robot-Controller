"""Real producer gates across first arrival, contact planning and another view."""
from copy import deepcopy
import threading
import numpy as np
import pytest
from alicia_flexible_grasp_supervisor.msg import GraspState
from test_station_compute_wait_producer import armed
from test_locked_rgb_hold import custom_frame, latest_status, module
from test_observation_measurement_unknown import send, ack, arrival, next_frame


def cut_over(monkeypatch, node, clock, joint, phase=False, fault=None):
    ready=threading.Event();released=threading.Event();errors=[];sources=[]
    original=node._frame_timing_stage
    def stage(name,*args,**kwargs):
        result=original(name,*args,**kwargs)
        if name=='post_segment':
            ready.set();assert released.wait(2.)
        return result
    def run():
        try:sources.append(custom_frame(node,clock,joint,target_depth=.35))
        except Exception as exc:errors.append(exc)
    clock[0]+=.2
    with monkeypatch.context() as change:
        change.setattr(node,'_frame_timing_stage',stage)
        thread=threading.Thread(target=run);thread.start();assert ready.wait(2.)
        if phase:
            next_phase=deepcopy(node._station_compute_phase);next_phase.active=False
            node._station_compute_phase_cb(next_phase)
        else:node._station_compute_state_cb(GraspState(state='MOVE_PREGRASP',active=True))
        if fault:fault()
        released.set();thread.join(2.)
        assert not thread.is_alive() and not errors,errors
    return sources[0]


@pytest.mark.parametrize('phase',[False,True])
def test_first_arrival_contact_retired_frame_next_lease_and_second_qualified(monkeypatch,phase):
    n,clock,joint,params,marker,source=armed(monkeypatch)
    anchor=n.tracker.anchor;last=n.tracker.last;plane=n.tracker.plane_base.copy()
    reference=n._locked_rgb_hold_reference;old_good=n.last_good_source_stamp_ns
    count=n.pubs['object'].publish.call_count;deadline=marker['session_deadline_ns']
    cut_over(monkeypatch,n,clock,joint,phase)
    assert n.first_loss is None and not n.tracker.lost
    assert n._locked_rgb_hold_reference is reference and n.tracker.last is last
    assert n.last_good_source_stamp_ns==old_good
    assert n.pubs['object'].publish.call_count==count
    assert n._station_compute_registration is None and n._station_compute_frame is None
    assert reference['observation_arrival_retry'] is marker
    request=deepcopy(n._observation_measurement_status['request'])
    for key in ('source_stamps_ns','settle'):request.pop(key,None)
    request.update(state='executing',lease_id='second_lease',request_id='second_execution',
        view_plan_id='second_plan',view_index=2,view_source_stamp_ns=old_good,
        stamp_ns=module.rospy.Time.now().to_nsec(),arrival_stamp_ns=0,capture_deadline_ns=0)
    send(n,request);assert ack(n)['accepted'],ack(n)
    assert n._locked_rgb_hold_reference is None
    clock[0]+=.1;custom_frame(n,clock,joint,target_depth=0.)
    assert n.pubs['object'].publish.call_count==count and not n.tracker.lost
    request=arrival(n,clock,joint,request)
    for _ in range(3):next_frame(n,clock,joint)
    stamps=ack(n)['source_stamps_ns'];assert len(stamps)==3 and ack(n)['qualified']
    terminal=dict(request,state='qualified',request_id='second_qualified',
        stamp_ns=module.rospy.Time.now().to_nsec(),source_stamps_ns=stamps)
    send(n,terminal);assert ack(n)['qualified'] and ack(n)['accepted'],ack(n)
    newer=n._locked_rgb_hold_reference['observation_arrival_retry']
    assert newer is not marker and newer['measurement_binding']['view_index']==2
    assert newer['session_deadline_ns']==deadline
    assert n.tracker.anchor is anchor and np.array_equal(n.tracker.plane_base,plane)
    assert n.first_loss is None and not n.tracker.lost


@pytest.mark.parametrize('fault',['manual','epoch','motion_out_back','selection','camera','config','anchor','plane','deadline','source_stale','ended','already_lost','model','handeye','terminal_active','contact'])
def test_real_station_fault_during_retirement_retains_original_failure(monkeypatch,fault):
    n,clock,joint,params,marker,source=armed(monkeypatch)
    reference=n._locked_rgb_hold_reference;stamp=n.last_good_source_stamp_ns
    def mutate():
        if fault=='manual':params['/gui/joint_direct_mode']=True
        elif fault=='epoch':n._rgb_interest_epoch+=1
        elif fault=='motion_out_back':joint(clock[0],[.005,0,0,0,0,0]);joint(clock[0]+.001)
        elif fault=='selection':params['/grasp_mode/selection']['generation']+=1
        elif fault=='camera':params['/camera']['fx']+=1
        elif fault=='config':n.config.support_threshold_m=.0019
        elif fault=='anchor':n.tracker.anchor.position_base[0]+=.001
        elif fault=='plane':n.tracker.plane_base[3]+=.001
        elif fault=='deadline':marker['session_deadline_ns']=module.rospy.Time.now().to_nsec()
        elif fault=='source_stale':clock[0]+=.81
        elif fault=='ended':n._station_compute_state_cb(GraspState(state='FAILED',active=False))
        elif fault=='already_lost':n.tracker.lost=True
        elif fault=='model':params['/robot_description']='<robot name="different"/>'
        elif fault=='handeye':params['/handeye']['changed']=True
        elif fault=='terminal_active':n._station_compute_state_cb(GraspState(state='FAILED',active=True))
        elif fault=='contact':n._station_compute_state_cb(GraspState(state='APPROACH',active=True))
    cut_over(monkeypatch,n,clock,joint,fault=mutate)
    assert n.first_loss is not None and n.tracker.lost,latest_status(n)
    assert n.last_good_source_stamp_ns==stamp
    assert n._locked_rgb_hold_reference is None or n._locked_rgb_hold_reference is reference
    assert not n.pubs['object'].publish.call_args[0][0].detected


def test_original_inflight_registration_deletion_reproduces_gen7_failure(monkeypatch):
    n,clock,joint,params,marker,source=armed(monkeypatch)
    monkeypatch.setattr(n,'_station_compute_retire_registration',
        lambda:setattr(n,'_station_compute_registration',None))
    cut_over(monkeypatch,n,clock,joint)
    assert n.first_loss is not None
    assert 'observation_arrival_station_invalidated_during_recovery' in latest_status(n)['detail']
    assert latest_status(n)['frame_timing']['arrival_station_invalidated_reason']=='station_compute_wait_station_changed'


@pytest.mark.parametrize('fault',['manual','motion','epoch'])
def test_change_during_final_retirement_context_check_is_not_swallowed(monkeypatch,fault):
    n,clock,joint,params,marker,source=armed(monkeypatch)
    original=n._rgb_interest_source_context
    def context(stamp):
        result=original(stamp)
        frame=getattr(n,'_station_compute_frame',None)
        if frame is not None and frame['registration'].get('retirement'):
            if fault=='manual':params['/gui/joint_direct_mode']=True
            elif fault=='epoch':n._rgb_interest_epoch+=1
            else:joint(clock[0],[.005,0,0,0,0,0]);joint(clock[0]+.001)
        return result
    monkeypatch.setattr(n,'_rgb_interest_source_context',context)
    cut_over(monkeypatch,n,clock,joint)
    assert n.first_loss is not None and n.tracker.lost


def test_frame_enrollment_and_phase_retirement_share_one_atomic_boundary(monkeypatch):
    n,clock,joint,params,marker,source=armed(monkeypatch)
    clock[0]+=.1;joint(clock[0])
    source=deepcopy(source)
    for message in source:message.header.stamp=module.rospy.Time.now()
    entered=threading.Event();release=threading.Event();retired=threading.Event();errors=[]
    original=n._station_compute_scope
    def scope(*args,**kwargs):
        if threading.current_thread().name=='frame-enrollment':
            entered.set();assert release.wait(2.)
        return original(*args,**kwargs)
    monkeypatch.setattr(n,'_station_compute_scope',scope)
    def begin():
        try:n._station_compute_begin_frame(source)
        except Exception as exc:errors.append(exc)
    def change():
        n._station_compute_state_cb(GraspState(state='MOVE_PREGRASP',active=True));retired.set()
    processing=threading.Thread(target=begin,name='frame-enrollment');processing.start()
    assert entered.wait(2.)
    callback=threading.Thread(target=change);callback.start()
    try:assert not retired.wait(.05),'state callback crossed the unfinished enrollment boundary'
    finally:release.set()
    processing.join(2.);callback.join(2.)
    assert not processing.is_alive() and not callback.is_alive() and not errors,errors
    frame=n._station_compute_frame
    assert frame['registration']['retirement']['frame'] is frame
    from alicia_grasp_modes.station_compute_wait_producer import StationComputeRetired
    with pytest.raises(StationComputeRetired) as retired_frame:n._station_compute_scope(frame['registration'])
    assert n._station_compute_retired_frame(source,retired_frame.value)
    assert n.first_loss is None and not n.tracker.lost


@pytest.mark.parametrize('active',[True,False])
def test_phase_retirement_followed_by_terminal_state_cannot_discard_failure(monkeypatch,active):
    n,clock,joint,params,marker,source=armed(monkeypatch)
    cut_over(monkeypatch,n,clock,joint,phase=True,
        fault=lambda:n._station_compute_state_cb(GraspState(state='FAILED',active=active)))
    assert n.first_loss is not None and n.tracker.lost
    assert not n.pubs['object'].publish.call_args[0][0].detected
