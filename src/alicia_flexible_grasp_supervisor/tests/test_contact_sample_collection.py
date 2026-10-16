"""Offline bounded collection: actual message objects, original validator, no ROS I/O."""
from collections import deque
from copy import deepcopy
from pathlib import Path
import threading

import pytest
import rospy

from alicia_flexible_grasp.robot import contact_sequence_precheck as module
from test_stationary_following import fixture, message


class Clock:
    def __init__(self):
        self.wall = 100.
        self.ros = 10.02
        self.on_sleep = lambda _: None
        self.sleeps = []

    def sleep(self, duration):
        assert 0. < duration <= .020000001
        self.wall += duration
        self.ros += duration
        self.sleeps.append(duration)
        self.on_sleep(duration)


def setup_case(monkeypatch):
    fk, sdk, accepted = fixture()
    sdk = [message(t, 'sdk_transmitted', sdk[-1].position) for t in (9.4, 10.)]
    accepted = [message(t, 'sdk_measured', accepted[-1].position[:6], True)
                for t in (9.4, 9.5, 9.6, 9.7, 9.8, 9.9, 10.)]
    node = module.ContactSequencePrecheckGateway()
    # A plain Lock proves sleep never holds the source lock; real uses RLock.
    node._endpoint_feedback_lock = threading.Lock()
    node._endpoint_feedback_history = dict(sdk=deque(sdk, maxlen=128),
                                          accepted=deque(accepted, maxlen=128))
    node._control_reference_epoch_ns = 9_000_000_000
    context = dict(manual=False, shutdown=False, selection={'mode':'unknown','generation':5})
    node._manual_control_active = lambda: context['manual']
    clock = Clock()
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock.wall)
    monkeypatch.setattr(module.time, 'sleep', clock.sleep)
    monkeypatch.setattr(module.rospy, 'get_time', lambda: clock.ros)
    monkeypatch.setattr(module.rospy, 'is_shutdown', lambda: context['shutdown'])
    def param(key,default=None):
        return context['manual'] if key=='/gui/joint_direct_mode' else context['selection']
    monkeypatch.setattr(module.rospy, 'get_param', param)
    monkeypatch.setattr(module.rospy, 'get_param_cached', param)
    audit = {}
    deadline = clock.wall+3.
    def remaining():
        value = deadline-clock.wall
        if value <= 0.:
            raise module.ContactPrecheckError('candidate precheck deadline consumed','MOVEIT_TIMEOUT')
        return value
    def collect(remaining_override=remaining):
        return node._contact_precheck_sample(fk, 9_000_000_000,
            remaining=remaining_override, audit=audit,
            selection={'mode':'unknown','generation':5})
    return node,fk,clock,context,audit,collect


def append_at(node, stamp, channels=('sdk','accepted')):
    assert node._endpoint_feedback_lock.acquire(False), 'feedback lock held while waiting'
    try:
        for key in channels:
            old = node._endpoint_feedback_history[key][-1]
            node._endpoint_feedback_history[key].append(message(
                stamp, 'sdk_transmitted' if key=='sdk' else 'sdk_measured',
                old.position[:6], key=='accepted'))
    finally:
        node._endpoint_feedback_lock.release()


def add_real_feeds(node,clock,channels=('sdk','accepted')):
    state = [1]
    def update(_):
        while clock.wall-100. >= state[0]*.1-1e-9:
            append_at(node,10.+state[0]*.1,channels)
            state[0] += 1
    clock.on_sleep = update


def test_collection_module_loaded_from_test_package():
    expected=Path(__file__).resolve().parents[1]/'src/alicia_flexible_grasp/robot/contact_sequence_precheck.py'
    assert Path(module.__file__).resolve()==expected.resolve()


def test_new_two_source_messages_recover_exact_original_validator(monkeypatch):
    node,fk,clock,_,audit,collect=setup_case(monkeypatch)
    add_real_feeds(node,clock)
    sample=collect()
    item=audit['following_sample_acquisition'][0]
    assert item['success'] and item['waited'] and item['recovered']
    assert item['elapsed_sec'] == pytest.approx(.3)
    assert sample['sdk_stamp_ns'] > 10_000_000_000
    assert sample['accepted_stamp_ns'] > 10_000_000_000
    assert sample == module.stationary_following_error(fk,
        node._endpoint_feedback_history['sdk'],node._endpoint_feedback_history['accepted'],
        now_sec=clock.ros,epoch_ns=9_000_000_000)
    assert sample['certifies_command_response_or_calibration'] is False
    assert item['initial_snapshot']['sdk']['history_count']==2
    assert item['final_snapshot']['sdk']['last_source_ns']==sample['sdk_stamp_ns']


def test_no_new_real_messages_never_counts_polling_as_samples(monkeypatch):
    _,_,clock,_,audit,collect=setup_case(monkeypatch)
    with pytest.raises(ValueError,match='missing.*sdk_transmitted'):
        collect()
    item=audit['following_sample_acquisition'][0]
    assert item['attempts']==1 and item['success'] is False
    assert item['elapsed_sec']==pytest.approx(.5)
    assert item['initial_snapshot']==item['final_snapshot']


@pytest.mark.parametrize('channels',[('sdk',),('accepted',)])
def test_one_feed_progress_cannot_substitute_for_both_independent_sources(monkeypatch,channels):
    node,_,clock,_,audit,collect=setup_case(monkeypatch)
    add_real_feeds(node,clock,channels)
    with pytest.raises(ValueError):collect()
    assert audit['following_sample_acquisition'][0]['attempts']==1
    assert clock.wall<=100.500000001


@pytest.mark.parametrize('fault,expected',[
    ('epoch','epoch changed'),('manual','manual control'),
    ('selection','selection changed'),('shutdown','shutdown')])
def test_context_change_during_wait_is_immediate_and_never_retried(monkeypatch,fault,expected):
    node,_,clock,context,audit,collect=setup_case(monkeypatch)
    def change(_):
        if fault=='epoch':node._control_reference_epoch_ns+=1
        elif fault=='selection':context['selection']={'mode':'unknown','generation':6}
        else:context[fault]=True
        append_at(node,10.1)
    clock.on_sleep=change
    with pytest.raises(ValueError,match=expected):collect()
    assert len(clock.sleeps)==1
    assert audit['following_sample_acquisition'][0]['attempts']==1


@pytest.mark.parametrize('kind',['sdk','accepted'])
@pytest.mark.parametrize('fault,expected',[
    ('duplicate','duplicate or reversed'),('reversed','duplicate or reversed'),
    ('frame','incorrect'),('nonfinite','invalid'),('names','ambiguous'),
    ('future','future')])
def test_new_corrupt_source_is_rejected_even_before_other_feed_advances(monkeypatch,kind,fault,expected):
    node,_,clock,_,audit,collect=setup_case(monkeypatch)
    def corrupt(_):
        history=node._endpoint_feedback_history[kind]
        msg=deepcopy(history[-1]);msg.header.stamp=rospy.Time.from_sec(10.01)
        if fault=='duplicate':msg.header.stamp=history[-1].header.stamp
        if fault=='reversed':msg.header.stamp=rospy.Time.from_sec(9.99)
        if fault=='frame':msg.header.frame_id='not_a_source'
        if fault=='nonfinite':msg.position[0]=float('nan')
        if fault=='names':msg.name[1]=msg.name[0]
        if fault=='future':msg.header.stamp=rospy.Time.from_sec(11.)
        history.append(msg)
    clock.on_sleep=corrupt
    with pytest.raises(ValueError,match=expected):collect()
    assert len(clock.sleeps)==1
    assert audit['following_sample_acquisition'][0]['attempts']==1


@pytest.mark.parametrize('fault,expected',[
    ('frame','incorrect'),('duplicate','duplicate'),('nonfinite','invalid'),
    ('stale','stale'),('future','future'),('too_short','insufficient stationary'),
    ('measured_gap','missing independent sdk_measured samples'),
    ('moving_sdk','SDK target changed'),('moving_accepted','not stationary')])
def test_other_initial_errors_never_enter_collection_wait(monkeypatch,fault,expected):
    node,fk,clock,_,audit,collect=setup_case(monkeypatch)
    # Supply an otherwise valid window before injecting one distinct fault.
    _,sdk,accepted=fixture()
    node._endpoint_feedback_history.update(sdk=deque(sdk,maxlen=128),accepted=deque(accepted,maxlen=128))
    s=node._endpoint_feedback_history['sdk'];a=node._endpoint_feedback_history['accepted']
    if fault=='frame':s[-1].header.frame_id='wrong'
    if fault=='duplicate':s.append(deepcopy(s[-1]))
    if fault=='nonfinite':s[-1].position[0]=float('nan')
    if fault=='stale':clock.ros=11.
    if fault=='future':clock.ros=9.8
    if fault=='too_short':node._endpoint_feedback_history['sdk']=deque([s[-1]],maxlen=128)
    if fault=='measured_gap':
        node._endpoint_feedback_history['accepted']=deque([
            message(9.3,'sdk_measured',a[-1].position[:6],True),a[-1]],maxlen=128)
    if fault=='moving_sdk':s[-1].position[0]+=.1
    if fault=='moving_accepted':a[-1].position[0]+=.1
    with pytest.raises(ValueError,match=expected):collect()
    assert not clock.sleeps and audit['following_sample_acquisition'][0]['attempts']==1


def test_candidate_deadline_shorter_than_collection_budget_is_not_extended(monkeypatch):
    _,_,clock,_,audit,collect=setup_case(monkeypatch)
    def remaining():
        value=100.055-clock.wall
        if value<=0.:raise module.ContactPrecheckError('candidate precheck deadline consumed','MOVEIT_TIMEOUT')
        return value
    with pytest.raises(module.ContactPrecheckError,match='deadline consumed') as caught:
        collect(remaining)
    assert caught.value.code=='MOVEIT_TIMEOUT'
    assert clock.wall==pytest.approx(100.055)
    assert audit['following_sample_acquisition'][0]['elapsed_sec']==pytest.approx(.055)


def test_frozen_ros_clock_does_not_extend_monotonic_collection_budget(monkeypatch):
    _,_,clock,_,audit,collect=setup_case(monkeypatch)
    clock.on_sleep=lambda _:setattr(clock,'ros',10.02)
    with pytest.raises(ValueError):collect()
    assert clock.wall==pytest.approx(100.5)
    assert audit['following_sample_acquisition'][0]['attempts']==1


def test_late_callback_cannot_refresh_fixed_wait_budget(monkeypatch):
    node,_,clock,_,audit,collect=setup_case(monkeypatch)
    def stalled(_):
        clock.wall+=.6;clock.ros+=.6
        for stamp in [10.2,10.3,10.4,10.5,10.6]:append_at(node,stamp)
    clock.on_sleep=stalled
    with pytest.raises(ValueError,match='missing independent'):collect()
    assert audit['following_sample_acquisition'][0]['wait_exhausted']
    assert audit['following_sample_acquisition'][0]['attempts']==1


def test_epoch_changed_inside_successful_fk_is_rejected(monkeypatch):
    node,_,clock,_,audit,collect=setup_case(monkeypatch)
    add_real_feeds(node,clock)
    original=module.stationary_following_error
    def validate(*args,**kw):
        sample=original(*args,**kw)
        node._control_reference_epoch_ns+=1
        return sample
    monkeypatch.setattr(module,'stationary_following_error',validate)
    with pytest.raises(ValueError,match='epoch changed'):collect()
    assert audit['following_sample_acquisition'][0]['success'] is False


def test_feedback_expiring_during_fk_cannot_return_success(monkeypatch):
    node,_,clock,_,audit,collect=setup_case(monkeypatch)
    _,sdk,accepted=fixture()
    node._endpoint_feedback_history.update(sdk=deque(sdk),accepted=deque(accepted))
    original=module.stationary_following_error
    def validate(*args,**kw):
        sample=original(*args,**kw)
        clock.ros+=.6
        return sample
    monkeypatch.setattr(module,'stationary_following_error',validate)
    with pytest.raises(ValueError,match='stale'):collect()
    assert not clock.sleeps


def test_valid_initial_evidence_returns_without_any_wait(monkeypatch):
    node,fk,clock,_,audit,collect=setup_case(monkeypatch)
    _,sdk,accepted=fixture()
    node._endpoint_feedback_history.update(sdk=deque(sdk),accepted=deque(accepted))
    sample=collect()
    assert not clock.sleeps
    item=audit['following_sample_acquisition'][0]
    assert item['success'] and not item['waited'] and not item['recovered']
    assert item['attempts']==1
    assert sample==module.stationary_following_error(fk,sdk,accepted,
        now_sec=clock.ros,epoch_ns=9_000_000_000)


def test_service_failure_includes_local_histories_and_elapsed_without_planning(monkeypatch):
    from test_contact_sequence_precheck import gateway_case
    node,planner,req,params,sample=gateway_case(monkeypatch)
    del node._contact_precheck_sample  # Actual staged implementation.
    node._endpoint_feedback_lock=threading.Lock()
    _,sdk,accepted=fixture()
    node._endpoint_feedback_history=dict(
        sdk=deque([message(t,'sdk_transmitted',sdk[-1].position) for t in (99.4,100.)]),
        accepted=deque([message(t,'sdk_measured',accepted[-1].position[:6],True)
                        for t in (99.4,99.5,99.6,99.7,99.8,99.9,100.)]))
    node._manual_control_active=lambda:False
    clock=Clock();clock.ros=100.02
    monkeypatch.setattr(module.time,'monotonic',lambda:clock.wall)
    monkeypatch.setattr(module.time,'sleep',clock.sleep)
    monkeypatch.setattr(module.rospy,'get_time',lambda:clock.ros)
    monkeypatch.setattr(module.rospy,'is_shutdown',lambda:False)
    monkeypatch.setattr(module.rospy,'get_param_cached',module.rospy.get_param)
    result=node.handle_contact_sequence_precheck(req)
    assert not result.success
    assert result.failure_code=='CONTACT_SEQUENCE_PRECHECK_FAILED'
    import json
    audit=json.loads(result.evidence_json)
    assert audit['execution_authorized'] is False
    item=audit['following_sample_acquisition'][0]
    assert item['initial_snapshot']['sdk']['history_count']==2
    assert item['final_snapshot']['accepted']['history_count']==7
    assert item['final_snapshot']['sdk']['last_source_ns']==100_000_000_000
    assert item['elapsed_sec']<=.500000001
    planner.check_pose_sequence.assert_not_called()
    node._ensure_trajectory_controllers_started.assert_not_called()


def test_wait_full_owner_rpc_only_on_entry_and_return(monkeypatch):
    node,_,clock,context,audit,collect=setup_case(monkeypatch)
    calls=[]
    def full(key,default=None):
        calls.append(key)
        return context['manual'] if key=='/gui/joint_direct_mode' else context['selection']
    monkeypatch.setattr(module.rospy,'get_param',full)
    node._manual_control_active=lambda:module.rospy.get_param('/gui/joint_direct_mode',False)
    add_real_feeds(node,clock)
    collect()
    assert calls==['/gui/joint_direct_mode','/grasp_mode/selection']*2
    assert len(clock.sleeps)>=10


def test_valid_manual_preview_preserves_fastpath_without_owner_rpc(monkeypatch):
    node,_,clock,context,audit,collect=setup_case(monkeypatch)
    _,sdk,accepted=fixture()
    node._endpoint_feedback_history.update(sdk=deque(sdk),accepted=deque(accepted))
    context['manual']=True
    def no_owner(*a,**kw):raise AssertionError('healthy read-only sample added owner check')
    monkeypatch.setattr(module.rospy,'get_param',no_owner)
    monkeypatch.setattr(module.rospy,'get_param_cached',no_owner)
    node._manual_control_active=no_owner
    assert collect()['measurement_kind']=='stationary_sdk_following_not_executed_endpoint'
    assert not clock.sleeps


def test_missing_samples_while_manual_does_not_enter_wait(monkeypatch):
    _,_,clock,context,audit,collect=setup_case(monkeypatch)
    context['manual']=True
    with pytest.raises(ValueError,match='manual control'):collect()
    assert not clock.sleeps


def test_return_authoritative_selection_catches_lagging_cache(monkeypatch):
    node,_,clock,context,audit,collect=setup_case(monkeypatch)
    cached=deepcopy(context['selection'])
    monkeypatch.setattr(module.rospy,'get_param_cached',lambda key,default=None:
        False if key=='/gui/joint_direct_mode' else cached)
    add_real_feeds(node,clock)
    update=clock.on_sleep
    def change(dt):
        update(dt)
        context['selection']={'mode':'unknown','generation':6}
    clock.on_sleep=change
    with pytest.raises(ValueError,match='selection changed'):collect()
    assert audit['following_sample_acquisition'][0]['success'] is False


def test_owner_rpc_delay_cannot_extend_wait_budget(monkeypatch):
    node,_,clock,_,audit,collect=setup_case(monkeypatch)
    original=node._manual_control_active
    def delayed():
        clock.wall+=.6;clock.ros+=.6
        return original()
    node._manual_control_active=delayed
    with pytest.raises(ValueError,match='missing independent'):collect()
    assert audit['following_sample_acquisition'][0]['wait_exhausted']
    assert not clock.sleeps
