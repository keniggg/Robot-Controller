"""Real task callbacks and commit hooks with inert clock and transport."""
from copy import deepcopy
from collections import deque
from itertools import permutations
import hashlib
import json
import threading
from types import SimpleNamespace as NS

import pytest
from sensor_msgs.msg import Image, JointState
from alicia_grasp_modes.station_compute_reacquisition import build_contract
from alicia_grasp_modes.unknown_pending_identity import reference_binding
from alicia_grasp_modes.rgb_observation_source import digest
from alicia_grasp_modes import station_compute_wait_task as wait
from test_task_modes import configured, module, base
from test_observe_return_task import evidence, at_ns
from test_contact_linear_compensation import recorded


@pytest.fixture
def station(configured, monkeypatch):
    node, params = configured
    selected, _, _ = evidence()
    _, _, plan, _, _ = recorded(2)
    plan.diagnostic = 'FAR_FIELD_OBSERVATION_PLAN'
    plan.header.stamp = at_ns(20_000_000_000)
    params[module.SELECTION_PARAM] = dict(selected, generation=3)
    assert node.reset_mode_cb(None).success
    node._mode_bound_selection = deepcopy(node._mode_selection)
    selected = node._mode_bound_selection
    now = [100_050_000_000]; mono = [10.]
    monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda: at_ns(now[0])))
    monkeypatch.setattr(wait.time, 'monotonic', lambda: mono[0])
    monkeypatch.setattr(module.rospy, 'logwarn_throttle', lambda *a: None)
    monkeypatch.setattr(module.rospy, 'loginfo', lambda *a: None)
    params['/camera'] = dict(fx=500., fy=500., width=3, height=2)
    params['/grasp'] = dict(target_observation_validity_sec=.8)
    node.active = True; node.stage = 2
    node._bound_execution_plan = deepcopy(plan)
    node._bound_execution_plan_id = plan.plan_id
    node._bound_execution_plan_digest = node._execution_plan_digest(plan)
    node.latest_grasp6d_plan = deepcopy(plan)
    node._execution_authority_revoked = False
    node._observe_return_session = dict(version=2, phase='observing', selection=deepcopy(selected),
        session_id='session', view_plan_id=plan.plan_id, view_index=2,
        target_epoch=4, target_track_id=plan.target_track_id, deadline_ns=200_000_000_000)
    node._observation_measurement_request = dict(node._observe_return_session,
        state='qualified', motion_kind='observation', lease_id='lease', control_epoch_ns=30_000_000_000,
        source_stamps_ns=[99_000_000_000,99_100_000_000,99_200_000_000])
    node._published_near_field_phase = NS(active=True, phase_id=2,
        header=NS(stamp=at_ns(99_400_000_000), frame_id='base_link'),
        deadline=at_ns(190_000_000_000), reference_source_stamp=at_ns(99_300_000_000),
        reference_center_valid=True, reference_target_track_id=plan.target_track_id, planning_stage='contact_replan')
    node._near_field_active = True; node._near_field_phase_id = 2
    node._near_field_phase_deadline_sec = 190.
    node._following_evidence_lock = threading.RLock()
    node._following_epoch_ns = 30_000_000_000
    node._following_accepted_history = deque(maxlen=128); node._following_sdk_history = deque(maxlen=128)
    c = NS(node=node, params=params, plan=plan, now=now, mono=mono, selection=selected)
    feedback(c, 99_980_000_000); feedback(c, 100_020_000_000)
    return c


def feedback(c, ns, delta=0.):
    m = JointState(); m.header.stamp = at_ns(ns); m.header.frame_id = 'sdk_measured'
    m.name = ['Joint%d' % i for i in range(1,7)]+['right_finger']
    m.position = [delta]+[0.]*5+[.05]
    c.node.accepted_joint_cb(m)


def frame(c, ns=100_000_000_000, recovery=None):
    obj = base.ObjectPose(); obj.header.stamp = at_ns(ns); obj.header.frame_id = 'base_link'
    obj.detected = True; obj.label = 'unknown_small'; obj.confidence = 1.
    obj.pose_base.header = deepcopy(obj.header); obj.pose_base.pose = deepcopy(c.plan.object_geometry.pose_base)
    mask = Image(); mask.header.stamp = at_ns(ns); mask.header.frame_id = 'camera'
    mask.height=2; mask.width=3; mask.step=3; mask.encoding='mono8'; mask.data=bytes([255]*6)
    status=dict(mode='unknown',generation=c.selection['generation'],selection_stamp_ns=c.selection['stamp_ns'],
        state='ready',source_stamp_ns=ns,target_locked=True,target_lost=False,first_loss=None,
        exact_rgbd=True,projection_frame_convention='ros_camera_link',color_stamp_ns=ns,depth_stamp_ns=ns,
        tf_lookup_stamp_ns=ns,color_frame_id='camera',depth_frame_id='camera',anchor_source_stamp_ns=40_000_000_000,
        anchor_position_base=[0.,0.,.02],anchor_extent_m=[.04,.03,.02],reference_support_plane_base=[0.,0.,1.,0.],
        reference_support_footprint_base=[[0.,0.,0.],[.04,0.,0.],[0.,.03,0.]])
    b=dict(selection=deepcopy(c.selection),lease_id='lease',session_id='session',view_plan_id=c.plan.plan_id,
        view_index=2,target_epoch=4,target_track_id=c.plan.target_track_id,control_epoch_ns=30_000_000_000,
        phase_id=2,phase_started_ns=99_400_000_000,phase_reference_source_ns=99_300_000_000,
        reference_binding=reference_binding(status,c.selection),source_id='a'*64,
        camera_configuration_sha256=digest(c.params['/camera']),config_sha256='b'*64,station_q_rad=[0.]*6)
    status['station_compute_wait']=build_contract(source_ns=ns,maximum_frame_age_sec=.8,
        session_deadline_ns=200_000_000_000,phase_deadline_ns=190_000_000_000,binding=b)
    status['station_compute_mask']=dict(sha256=hashlib.sha256(bytes([1]*6)).hexdigest(),shape=[2,3],frame_id='camera')
    if recovery is not None:status['station_compute_recovery']=deepcopy(recovery)
    return dict(object=obj,mask=mask,status=status)


def deliver(c, values, order=('object','mask','status')):
    for key in order:
        if key=='object':c.node.obj_cb(values[key])
        elif key=='mask':c.node._station_compute_task_mask_cb(values[key])
        else:c.node._unknown_support_status_cb(NS(data=json.dumps(values[key])))


def advance(c, ns):
    c.mono[0]+=(ns-c.now[0])/1e9; c.now[0]=ns
    feedback(c,ns)


@pytest.mark.parametrize('order',list(permutations(('object','mask','status'))))
def test_actual_three_topic_join_pins_original_fresh_source(station,order):
    c=station; f=frame(c); deliver(c,f,order)
    assert c.node._station_task_record['contract']==f['status']['station_compute_wait']
    assert not c.node._station_task_record['pending']
    assert c.node.latest_obj.header.stamp.to_nsec()==100_000_000_000
    frozen=deepcopy(c.node._station_task_record)
    advance(c,100_200_000_000);deliver(c,f,order)
    assert c.node._station_task_record==frozen


@pytest.mark.parametrize('order',list(permutations(('object','mask','status'))))
def test_late_registration_cannot_create_identity_wait(station,order):
    c=station;f=frame(c);advance(c,100_800_000_000);deliver(c,f,order)
    assert c.node._station_task_record is None


@pytest.mark.parametrize('order',list(permutations(('object','mask','status'))))
def test_expiry_quarantines_then_only_exact_fresh_recovery_releases(station,order):
    c=station;f=frame(c);deliver(c,f)
    old=deepcopy(c.node._station_task_record['contract'])
    advance(c,100_900_000_000)
    assert c.node._station_task_record['pending'] and c.node.latest_obj is None
    result,candidate=c.node._copy_near_field_preview_candidate(c.plan,0,{})
    assert not result.ok and result.code==wait.PENDING and not result.terminal_for_request and candidate is None
    assert c.node._before_near_field_terminal_recovery(result,c.plan,{},None,None)=={'waiting':True}
    called=[];r,_=c.node._invoke_plan_bound_action(c.plan,{},'move',lambda:called.append(True))
    assert not r.ok and not called
    c.now[0]=101_050_000_000;c.mono[0]=11.
    feedback(c,100_980_000_000);feedback(c,101_020_000_000)
    new=frame(c,101_000_000_000,old);deliver(c,new,order)
    assert not c.node._station_task_record['pending']
    assert c.node._station_task_cutoff==100_999_999_999
    assert c.node.latest_obj.header.stamp.to_nsec()==101_000_000_000
    stale=deepcopy(c.plan);stale.diagnostic='CONTACT_EXECUTION_PLAN';stale.header.stamp=at_ns(100_500_000_000)
    assert c.node._station_task_plan_failure(stale).code==wait.PENDING
    assert c.node._bound_execution_plan.plan_id==c.plan.plan_id
    assert not c.node._execution_authority_revoked


@pytest.mark.parametrize('first',['object','status'])
def test_typed_pending_false_order_preserves_history_not_geometry(station,first):
    c=station;f=frame(c);deliver(c,f)
    old=deepcopy(c.node._station_task_record['contract'])
    p=frame(c,100_400_000_000);p['object'].detected=False
    p['status'].update(state='processing_deadline_pending',station_compute_wait=old)
    advance(c,100_500_000_000);deliver(c,p,(first, 'status' if first=='object' else 'object'))
    assert c.node._station_task_record['pending'] and c.node.latest_obj is None
    assert not c.node._execution_authority_revoked
    assert c.node.latest_grasp6d_plan.plan_id==c.plan.plan_id


@pytest.mark.parametrize('fault',['manual','epoch','out_back','camera','session','phase','selection','five_seconds','monotonic'])
def test_binding_or_clock_failure_is_permanent(station,fault):
    c=station;f=frame(c);deliver(c,f)
    if fault=='manual':c.params['/gui/joint_direct_mode']=True
    elif fault=='epoch':c.node._following_epoch_ns+=1
    elif fault=='out_back':feedback(c,100_030_000_000,.01);feedback(c,100_040_000_000,0.)
    elif fault=='camera':c.params['/camera']['fx']+=1.
    elif fault=='session':c.node._observe_return_session['session_id']='other'
    elif fault=='phase':c.node._near_field_phase_id+=1
    elif fault=='selection':c.params[module.SELECTION_PARAM]=dict(c.selection,generation=c.selection['generation']+1)
    elif fault=='five_seconds':advance(c,105_000_000_000)
    else:c.mono[0]+=5.
    c.node._station_compute_task_tick()
    assert c.node._station_task_retired and c.node._execution_authority_revoked
    c.params['/gui/joint_direct_mode']=False
    deliver(c,f)
    assert c.node._station_task_record is None


def test_pending_fresh_true_without_old_parent_does_not_refresh(station):
    c=station;deliver(c,frame(c));old=deepcopy(c.node._station_task_record)
    advance(c,100_900_000_000)
    c.now[0]=101_050_000_000;feedback(c,100_980_000_000);feedback(c,101_020_000_000)
    deliver(c,frame(c,101_000_000_000))
    assert c.node._station_task_record['pending']
    assert c.node._station_task_record['contract']==old['contract']
    assert c.node._station_task_record['identity_mono']==old['identity_mono']


def test_foreign_lost_status_cannot_retire_registered_selection(station):
    c=station;f=frame(c);deliver(c,f)
    foreign=deepcopy(f['status']);foreign['generation']+=1;foreign['target_lost']=True
    c.node._unknown_support_status_cb(NS(data=json.dumps(foreign)))
    assert c.node._station_task_record and not c.node._station_task_retired


def test_true_loss_then_new_ready_never_resurrects(station):
    c=station;f=frame(c);deliver(c,f)
    lost=deepcopy(f['status']);lost.update(target_lost=True,state='unavailable')
    c.node._unknown_support_status_cb(NS(data=json.dumps(lost)))
    deliver(c,f)
    assert c.node._station_task_retired and c.node._station_task_record is None


@pytest.mark.parametrize('fault',['mask','pose_source','phase_source','no_qualified','moving_stage','direct'])
def test_incomplete_or_foreign_measurement_does_not_arm(station,fault):
    c=station;f=frame(c)
    if fault=='mask':f['mask'].data=bytes([255,0,255,255,255,255])
    elif fault=='pose_source':f['object'].pose_base.header.stamp=at_ns(99_000_000_000)
    elif fault=='phase_source':c.node._published_near_field_phase.reference_source_stamp=at_ns(99_310_000_000)
    elif fault=='no_qualified':c.node._observation_measurement_request['state']='executing'
    elif fault=='moving_stage':c.node.stage=3
    else:c.node._mode_bound_selection['strategy']='direct'
    deliver(c,f)
    assert c.node._station_task_record is None


def test_old_farfield_metadata_rebind_is_not_old_motion_permission(station,monkeypatch):
    c=station;deliver(c,frame(c));called=[]
    monkeypatch.setattr(c.node,'_validate_bound_plan_locked',lambda *a:base.PlanValidationResult(True))
    r,_=c.node._invoke_plan_bound_action(c.plan,{},'supplemental observation rebind',lambda:called.append('metadata'))
    assert r.ok and called==['metadata']
    r,_=c.node._invoke_plan_bound_action(c.plan,{},'strict motion',lambda:called.append('motion'))
    assert not r.ok and called==['metadata']


def test_pending_blocks_atomic_contact_freeze_and_metadata_rebind(station,monkeypatch):
    c=station;deliver(c,frame(c));advance(c,100_900_000_000)
    _,_,contact,_,_=recorded(2)
    contact.header.stamp=at_ns(100_500_000_000)
    with pytest.raises(ValueError,match=wait.PENDING):
        c.node._freeze_execution_plan(contact,allow_target_occlusion=True)
    called=[]
    monkeypatch.setattr(c.node,'_validate_bound_plan_locked',lambda *a:base.PlanValidationResult(True))
    result,_=c.node._invoke_plan_bound_action(c.plan,{},'supplemental observation rebind',lambda:called.append(True))
    assert not result.ok and not called
    assert c.node._bound_execution_plan.plan_id==c.plan.plan_id


def test_contact_freeze_disarms_wait_before_first_action_and_keeps_original_occlusion(station):
    c=station;deliver(c,frame(c))
    _,_,contact,_,_=recorded(2)
    contact.header.stamp=at_ns(100_000_000_000)
    contact.plan_id=base.compute_plan_id(contact)
    c.node._freeze_execution_plan(contact,allow_target_occlusion=True)
    assert c.node._station_task_record is None
    assert c.node._bound_target_occlusion_allowed
    advance(c,101_000_000_000)
    assert not c.node._station_task_retired


def test_late_pending_preview_cannot_repend_newly_qualified_record(station):
    c=station;deliver(c,frame(c));old=deepcopy(c.node._station_task_record['contract'])
    advance(c,100_900_000_000)
    c.now[0]=101_050_000_000;feedback(c,100_980_000_000);feedback(c,101_020_000_000)
    deliver(c,frame(c,101_000_000_000,old))
    saved=deepcopy(c.node._station_task_record)
    c.node.latest_grasp6d_preview_plan=NS(valid=False,candidate_source='near_field_pending',
        diagnostic='TARGET_PROCESSING_PENDING: old worker',header=NS(stamp=at_ns(100_900_000_000)))
    result,_=c.node._copy_near_field_preview_candidate(c.plan,0,{})
    assert result.code==wait.PENDING and not result.terminal_for_request
    assert c.node._station_task_record==saved and not saved['pending']


def test_case_insensitive_real_joint_names_preserve_fixed_station(station):
    c=station
    for msg in c.node._following_accepted_history:msg.name=[name.lower() for name in msg.name]
    deliver(c,frame(c))
    assert c.node._station_task_record and not c.node._station_task_retired


def test_timer_without_new_input_withholds_at_original_geometry_deadline(station):
    c=station;deliver(c,frame(c));contract=deepcopy(c.node._station_task_record['contract'])
    c.mono[0]+=.76;c.now[0]=100_810_000_000
    # Accepted feedback remains fresh; no new perception message is delivered.
    feedback(c,100_800_000_000)
    c.node._station_compute_task_tick()
    assert c.node._station_task_record['pending'] and c.node.latest_obj is None
    assert c.node._station_task_record['contract']==contract


def test_expiry_during_contact_preflight_cannot_commit_action(station,monkeypatch):
    c=station;deliver(c,frame(c))
    advance(c,100_900_000_000)
    called=[]
    monkeypatch.setattr(c.node,'_validate_bound_plan_locked',lambda *a:base.PlanValidationResult(True))
    _,_,contact,_,_=recorded(2);contact.header.stamp=at_ns(100_500_000_000)
    result,_=c.node._invoke_plan_bound_action(contact,{},'strict pregrasp',lambda:called.append(True))
    assert not result.ok and result.code==wait.PENDING and not called


def test_missed_ready_identity_chain_does_not_import_missing_geometry(station):
    c=station;f=frame(c);deliver(c,f);a=deepcopy(c.node._station_task_record['contract'])
    b=frame(c,100_600_000_000)['status']['station_compute_wait']
    advance(c,100_900_000_000)
    old_mono=c.node._station_task_record['identity_mono']
    pending=deepcopy(f['status']);pending.update(source_stamp_ns=100_000_000_000,
        state='processing_deadline_pending',station_compute_wait=b,station_compute_recovery_chain=[a,b])
    c.node._unknown_support_status_cb(NS(data=json.dumps(pending)))
    assert c.node._station_task_record['contract']==a
    assert c.node._station_task_record['identity_mono']==old_mono and c.node.latest_obj is None
    c.now[0]=101_050_000_000;feedback(c,100_980_000_000);feedback(c,101_020_000_000)
    fresh=frame(c,101_000_000_000,b);fresh['status']['station_compute_recovery_chain']=[a,b]
    deliver(c,fresh)
    assert c.node.latest_obj.header.stamp.to_nsec()==101_000_000_000
    assert not c.node._station_task_record['pending']


def test_missed_ready_chain_cannot_extend_consumer_original_five_seconds(station):
    c=station;f=frame(c);deliver(c,f);a=deepcopy(c.node._station_task_record['contract'])
    b=frame(c,104_000_000_000)['status']['station_compute_wait']
    advance(c,105_100_000_000)
    fresh=frame(c,105_000_000_000,b);fresh['status']['station_compute_recovery_chain']=[a,b]
    feedback(c,104_980_000_000);feedback(c,105_020_000_000);deliver(c,fresh)
    assert c.node._station_task_retired and c.node._station_task_record is None


@pytest.mark.parametrize('key',['source_id','config_sha256','reference_binding'])
def test_positive_changed_producer_binding_permanently_retires(station,key):
    c=station;deliver(c,frame(c));advance(c,100_500_000_000)
    changed=frame(c,100_400_000_000)['status']
    changed['station_compute_wait']['binding'][key]='c'*64
    c.node._unknown_support_status_cb(NS(data=json.dumps(changed)))
    assert c.node._station_task_retired and c.node._execution_authority_revoked


def test_real_epoch_phase_deadline_uses_published_message_without_ns_roundtrip(station):
    c=station;f=frame(c)
    # A double at contemporary Unix time cannot represent arbitrary nanoseconds.
    phase_ns=1791552516268740178
    c.node._published_near_field_phase.deadline=at_ns(phase_ns)
    c.node._near_field_phase_deadline_sec=c.node._published_near_field_phase.deadline.to_sec()
    assert at_ns(phase_ns).to_nsec()!=module.rospy.Time.from_sec(c.node._near_field_phase_deadline_sec).to_nsec()
    b=f['status']['station_compute_wait']['binding']
    c.node._observe_return_session['deadline_ns']=phase_ns+1_000_000_000
    f['status']['station_compute_wait']=build_contract(source_ns=100_000_000_000,
        maximum_frame_age_sec=.8,session_deadline_ns=phase_ns+1_000_000_000,
        phase_deadline_ns=phase_ns,binding=b)
    deliver(c,f)
    assert c.node._station_task_record is not None


@pytest.mark.parametrize('callback',['feedback','status','mask'])
def test_inactive_wait_callbacks_do_not_wait_on_motion_action_lock(station,callback):
    c=station;c.node.stage=3
    f=frame(c);done=threading.Event();errors=[]
    def invoke():
        try:
            if callback=='feedback':feedback(c,100_030_000_000)
            elif callback=='status':c.node._station_compute_task_status(f['status'])
            else:c.node._station_compute_task_mask_cb(f['mask'])
        except Exception as exc:errors.append(exc)
        finally:done.set()
    with c.node._grasp6d_plan_guard():
        worker=threading.Thread(target=invoke);worker.start()
        completed=done.wait(1.)
    worker.join(1.)
    assert completed and not errors


def test_supplement_publish_disarms_before_async_wait_and_cannot_rearm_old_station(station,monkeypatch):
    # Other isolated script tests may replace the namespace module during collection.
    # Patch the actual inherited owner used by this configured node.
    owners = [cls for cls in type(station.node).__mro__
              if cls.__module__ == "alicia_grasp_modes.observe_return_task"
              and cls.__name__ == "ObserveReturnTask"
              and "_observe_return_publish" in cls.__dict__]
    assert len(owners) == 1
    ObserveReturnTask = owners[0]
    c=station
    # The real handoff now checks frozen integrity even when this transport
    # test mocks later validation. Bind the fixture's changed phase/header.
    c.plan.object_geometry.header=deepcopy(c.plan.header)
    c.plan.plan_id=base.compute_plan_id(c.plan)
    c.node._bound_execution_plan=deepcopy(c.plan)
    c.node._bound_execution_plan_id=c.plan.plan_id
    c.node._bound_execution_plan_digest=c.node._execution_plan_digest(c.plan)
    c.node.latest_grasp6d_plan=deepcopy(c.plan)
    c.node._observe_return_session['view_plan_id']=c.plan.plan_id
    c.node._observation_measurement_request['view_plan_id']=c.plan.plan_id
    f=frame(c);deliver(c,f);seen=[]
    monkeypatch.setattr(c.node,'_bound_target_drift_result',lambda *a:base.PlanValidationResult(True))
    assert c.node._validate_bound_plan_locked(c.plan,{}).ok
    def publish(self,message):
        assert self._station_task_record is None
        advance(c,101_000_000_000)
        deliver(c,f)
        self._station_compute_task_tick()
        assert self._station_task_record is None and not self._station_task_retired
        seen.append(message)
    monkeypatch.setattr(ObserveReturnTask,'_observe_return_publish',publish)
    nxt=dict(c.node._observe_return_session,view_index=3,view_plan_id='next')
    c.node._observe_return_publish(nxt)
    assert seen==[nxt]
    r,_=c.node._invoke_plan_bound_action(c.plan,{},'supplemental observation rebind',lambda:'bound')
    assert r.ok


def test_pending_cannot_disarm_itself_by_submitting_next_session(station,monkeypatch):
    # Other isolated script tests may replace the namespace module during collection.
    # Patch the actual inherited owner used by this configured node.
    owners = [cls for cls in type(station.node).__mro__
              if cls.__module__ == "alicia_grasp_modes.observe_return_task"
              and cls.__name__ == "ObserveReturnTask"
              and "_observe_return_publish" in cls.__dict__]
    assert len(owners) == 1
    ObserveReturnTask = owners[0]
    c=station;deliver(c,frame(c));advance(c,100_900_000_000);sent=[]
    monkeypatch.setattr(ObserveReturnTask,'_observe_return_publish',lambda *a:sent.append(True))
    with pytest.raises(ValueError,match=wait.PENDING):
        c.node._observe_return_publish(dict(c.node._observe_return_session,view_index=3))
    assert not sent and c.node._station_task_record['pending']


def test_late_old_pending_and_false_do_not_retire_new_recovery(station):
    c=station;old=frame(c);deliver(c,old);a=deepcopy(c.node._station_task_record['contract'])
    advance(c,100_900_000_000);c.now[0]=101_050_000_000
    feedback(c,100_980_000_000);feedback(c,101_020_000_000)
    deliver(c,frame(c,101_000_000_000,a));saved=deepcopy(c.node._station_task_record)
    old['status'].update(state='processing_deadline_pending',target_lost=False)
    old['object'].detected=False
    deliver(c,old)
    assert c.node._station_task_record==saved and not c.node._station_task_retired


@pytest.mark.parametrize('kind',['object','mask','contract'])
def test_same_source_registered_content_cannot_change(station,kind):
    c=station;f=frame(c);deliver(c,f)
    if kind=='object':f['object'].pose_base.pose.position.x+=.001;deliver(c,f,('object',))
    elif kind=='mask':f['mask'].data=bytes([0,255,255,255,255,255]);deliver(c,f,('mask',))
    else:f['status']['station_compute_wait']['binding']['source_id']='e'*64;deliver(c,f,('status',))
    assert c.node._station_task_retired and c.node._execution_authority_revoked


def test_unproven_new_ready_expiring_in_cache_preserves_original_fixed_wait(station):
    c=station;deliver(c,frame(c));old=deepcopy(c.node._station_task_record)
    advance(c,100_900_000_000);c.now[0]=101_050_000_000
    feedback(c,100_980_000_000);feedback(c,101_020_000_000)
    deliver(c,frame(c,101_000_000_000))
    advance(c,101_900_000_000);c.node._station_compute_task_tick()
    assert c.node._station_task_record['pending'] and not c.node._station_task_retired
    assert c.node._station_task_record['contract']==old['contract']
    assert c.node._station_task_record['identity_mono']==old['identity_mono']
