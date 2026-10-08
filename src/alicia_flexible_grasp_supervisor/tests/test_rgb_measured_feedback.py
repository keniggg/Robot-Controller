"""RGB measured gates consume real SDK sources, never command/heartbeat caches."""
from collections import deque
from copy import deepcopy
from types import SimpleNamespace as NS
import math
import threading

import pytest
from sensor_msgs.msg import JointState
from test_motion_gateway_controller_start import motion_gateway_node as gateway


def fixture(monkeypatch):
    node = gateway.MotionGateway.__new__(gateway.MotionGateway)
    node.joint_names = ['Joint%d' % i for i in range(1, 7)]+['right_finger']
    node._command_reference_lock = threading.RLock()
    node._endpoint_feedback_lock = threading.RLock()
    node._endpoint_feedback_history = {'accepted': deque(maxlen=128), 'sdk': deque(maxlen=128)}
    node._control_reference_epoch_ns = 9_000_000_000
    node._manual_control_active = lambda: False
    node.joint_cmd = NS(last_positions=[.8]*7, last_state_time_sec=10.)
    node.gripper = NS(gripper_index=6)
    clock = [10.]
    monkeypatch.setattr(gateway.rospy, 'get_time', lambda:clock[0])
    monkeypatch.setattr(gateway.rospy.Time, 'now', lambda:gateway.rospy.Time.from_sec(clock[0]))
    monkeypatch.setattr(gateway.rospy, 'get_param', lambda key,default=None:default)
    monkeypatch.setattr(gateway.rospy, 'is_shutdown', lambda:False)
    monkeypatch.setattr(gateway.time, 'monotonic', lambda:clock[0])
    def emit(stamp=None, q=None):
        msg = JointState()
        msg.header.stamp = gateway.rospy.Time.from_sec(clock[0] if stamp is None else stamp)
        msg.header.frame_id = 'sdk_measured'
        msg.name = list(node.joint_names)
        msg.position = list(q or [0.]*6)+[.02]
        node._observation_joint_feedback_cb(msg)
        return msg
    emit(9.99)
    monkeypatch.setattr(gateway.time, 'sleep', lambda seconds:clock.__setitem__(0,clock[0]+seconds))
    return node,clock,emit


def test_snapshot_ignores_command_and_partial_heartbeat_cache(monkeypatch):
    node,_,_ = fixture(monkeypatch)
    assert node._rgb_measured_arm_snapshot() == [0.]*6
    node.joint_cmd.last_positions = [float('nan'),99.]
    assert node._rgb_measured_arm_snapshot() == [0.]*6
    result = node._rgb_measured_arm_snapshot();result[0]=.9
    assert node._rgb_measured_arm_snapshot() == [0.]*6


def test_snapshot_returns_canonical_arm_order_even_when_wire_or_config_order_differs(monkeypatch):
    node,_,emit=fixture(monkeypatch)
    values=[.01,.02,.03,.04,.05,.06]
    message=emit(q=values)
    message.name.reverse();message.position.reverse()
    node.joint_names=list(reversed(node.joint_names[:-1]))+[node.joint_names[-1]]
    assert node._rgb_measured_arm_snapshot()==values


@pytest.mark.parametrize('fault', ['missing','stale','future','old_epoch','wrong_frame',
    'partial','duplicate_name','nonfinite','gripper','arm_domain'])
def test_snapshot_rejects_invalid_sdk_evidence(monkeypatch,fault):
    node,clock,_ = fixture(monkeypatch)
    msg=node._observation_joint_feedback
    if fault=='missing': node._observation_joint_feedback=None
    elif fault=='stale':clock[0]+=.6
    elif fault=='future':msg.header.stamp=gateway.rospy.Time.from_sec(10.1)
    elif fault=='old_epoch':msg.header.stamp=gateway.rospy.Time(9)
    elif fault=='wrong_frame':msg.header.frame_id='sdk_transmitted'
    elif fault=='partial':msg.name.pop();msg.position.pop()
    elif fault=='duplicate_name':msg.name[0]=msg.name[1]
    elif fault=='nonfinite':msg.position[0]=float('nan')
    elif fault=='gripper':msg.position[-1]=.06
    elif fault=='arm_domain':msg.position[0]=math.pi+.01
    with pytest.raises((gateway.ObservationPathError, ValueError)):
        node._rgb_measured_arm_snapshot()


@pytest.mark.parametrize('fault',['epoch','time'])
def test_snapshot_rechecks_after_parameter_lookup(monkeypatch,fault):
    node,clock,_=fixture(monkeypatch)
    calls=[0]
    def get(key,default=None):
        calls[0]+=1
        if calls[0]==2:
            if fault=='epoch':node._control_reference_epoch_ns+=1
            else:clock[0]+=.6
        return default
    monkeypatch.setattr(gateway.rospy,'get_param',get)
    with pytest.raises(gateway.ObservationPathError):node._rgb_measured_arm_snapshot()


def stream(monkeypatch,node,clock,emit,mutation=None):
    ticks=[0]
    def sleep(seconds):
        clock[0]=round(clock[0]+seconds,9);ticks[0]+=1
        if ticks[0]%5==0:emit()
        if mutation:mutation(ticks[0])
    monkeypatch.setattr(gateway.time,'sleep',sleep)


def test_stationary_requires_new_real_source_span(monkeypatch):
    node,clock,emit=fixture(monkeypatch)
    stream(monkeypatch,node,clock,emit)
    assert node._wait_for_stationary_rgb_feedback()[0]
    assert clock[0]>=10.4  # New stamps .1,.2,.3,.4, not the initial cache.


@pytest.mark.parametrize('heartbeat',['cache_only','same_stamp'])
def test_repeated_heartbeat_never_proves_stationarity(monkeypatch,heartbeat):
    node,clock,emit=fixture(monkeypatch)
    def sleep(seconds):
        clock[0]+=seconds
        node.joint_cmd.last_state_time_sec=clock[0]
        if heartbeat=='same_stamp':emit(9.99)
    monkeypatch.setattr(gateway.time,'sleep',sleep)
    assert not node._wait_for_stationary_rgb_feedback()[0]
    assert clock[0]<12.1


@pytest.mark.parametrize('fault',['epoch','manual','regressed','duplicate_changed','malformed'])
def test_stationary_rejects_live_evidence_revocation(monkeypatch,fault):
    node,clock,emit=fixture(monkeypatch)
    def mutate(tick):
        if tick!=11:return
        if fault=='epoch':node._control_reference_epoch_ns+=1
        elif fault=='manual':node._manual_control_active=lambda:True
        elif fault=='regressed':emit(10.15)
        elif fault=='duplicate_changed':emit(10.2,[.01]*6)
        else:
            msg=emit();msg.header.frame_id='sdk_transmitted'
    stream(monkeypatch,node,clock,emit,mutate)
    assert not node._wait_for_stationary_rgb_feedback()[0]


def test_out_and_back_between_polls_restarts_measured_window(monkeypatch):
    node,clock,emit=fixture(monkeypatch)
    def mutate(tick):
        if tick==16:
            emit(10.31,[.01]*6)
            emit(10.32)
    stream(monkeypatch,node,clock,emit,mutate)
    assert node._wait_for_stationary_rgb_feedback()[0]
    assert clock[0]>=10.6  # Returning to the original q does not erase excursion.


def test_static_ros_clock_and_repeated_source_cannot_extend_wall_budget(monkeypatch):
    node,clock,_=fixture(monkeypatch)
    wall=[10.]
    monkeypatch.setattr(gateway.time,'monotonic',lambda:wall[0])
    monkeypatch.setattr(gateway.time,'sleep',lambda seconds:wall.__setitem__(0,wall[0]+seconds))
    assert not node._wait_for_stationary_rgb_feedback()[0]
    assert clock[0]==10. and 12.<=wall[0]<12.1


def test_real_five_count_motion_still_rejects_original_rgb_owner(monkeypatch):
    node,clock,emit=fixture(monkeypatch)
    from alicia_flexible_grasp.robot.rgb_acquisition_gateway import digest
    context=dict(epoch_ns=node._control_reference_epoch_ns)
    node._rgb_context=lambda stamp:context
    node._rgb_revocation=0
    node._observation_context_lock=threading.RLock()
    node._observation_task_connection_available=lambda:True
    node._observation_task_requires_inactive=False
    node._observation_contexts=NS(active=False)
    node._observation_task_state_stamp_ns=1
    node._rgb_require_actuation=lambda episode:None
    monkeypatch.setattr(gateway.rospy,'get_param_cached',lambda key,default=None:
        key=='/robot/observation_tracking_contract_enabled')
    episode=dict(revocation=0,created_ns='9000000000',created_monotonic=9.,
        deadline_ns='20000000000',deadline_monotonic=20.,source_id=digest(context),q_source=[0.]*6)
    node.joint_cmd.last_positions=[0.]*7
    emit(q=[0.,0.,-5*2*math.pi/4096.,0.,0.,0.])
    original=deepcopy(episode)
    with pytest.raises(gateway.ObservationPathError,match='MEASURED_POSE_CHANGED'):
        node._rgb_require_owner(episode)
    assert episode==original
