"""Independent gateway/typed-lease integration, with all transports injected."""
from copy import deepcopy
from types import SimpleNamespace as NS
import pytest
import rospy

from test_motion_gateway_controller_start import MotionGateway,motion_gateway_node as gateway_module
from test_r22_typed_client_independent import case,prepared,expired_manual
from alicia_flexible_grasp.robot.endpoint_correction import sdk_counts


def gateway(c,monkeypatch):
    node=MotionGateway.__new__(MotionGateway)
    names=list(c.contract.wire_plan().joint_trajectory.joint_names)
    wire=list(c.contract.wire_plan().joint_trajectory.points[-1].positions)
    physical=list(c.contract.physical_plan().joint_trajectory.points[-1].positions)
    node.joint_names=names+['right_finger']
    node._physical_reference_lease=c.lease
    node._completed_command_reference=None
    node._control_reference_epoch_ns=c.lease.model.epoch_ns
    node._manual_control_active=lambda:False
    node._fresh_accepted_reference=lambda:list(physical)
    node._command_reference_snapshot=lambda:NS(mode='sdk_tracking',positions=wire,stamp_sec=c.clock.ros*1e-9)
    node._stationary_controller_reference=lambda names,**kw:NS(positions=physical.copy(),
        velocities=[0.]*6,accelerations=[0.]*6,controller_stamp_sec=c.clock.ros*1e-9)
    monkeypatch.setattr(rospy,'get_time',lambda:c.clock.ros*1e-9)
    monkeypatch.setattr(rospy.Time,'now',staticmethod(lambda:rospy.Time(c.clock.ros//10**9,c.clock.ros%10**9)))
    monkeypatch.setattr(gateway_module.time,'monotonic',c.clock.now)
    def advance(dt):c.clock.mono+=dt;c.clock.ros+=round(dt*1e9)
    monkeypatch.setattr(gateway_module.time,'sleep',advance)
    return node


def terminal(c):
    c.lease.arm(c.envelope)
    for peer in (c.driver,c.hardware):
        peer.phase=c.lease.EXECUTING;peer.goal_terminal=True
    return c


def test_r22_successful_completion_quiesces_before_next_proven_controller_reference(monkeypatch):
    c=terminal(prepared(case()));node=gateway(c,monkeypatch)
    old=(c.lease.model.digest,c.lease.local_deadline,c.lease.outer_deadline)
    node._record_completed_controller_reference(c.contract.wire_plan())
    assert c.driver.phase==c.hardware.phase==c.lease.ACTIVE_IDLE
    ref=node._controller_reference_for_execution(node.joint_names[:-1])
    assert tuple(ref.positions)==tuple(c.contract.wire_plan().joint_trajectory.points[-1].positions)
    assert ref.physical_positions==tuple(c.contract.physical_plan().joint_trajectory.points[-1].positions)
    assert (c.lease.model.digest,c.lease.local_deadline,c.lease.outer_deadline)==old
    assert node._completed_command_reference['physical_reference']['release_verified'] is False


@pytest.mark.parametrize('fault',['not_terminal','failed_result','pending','moving'])
def test_r22_unconfirmed_terminal_cannot_create_completed_reference(monkeypatch,fault):
    c=terminal(prepared(case()));node=gateway(c,monkeypatch)
    if fault=='not_terminal':c.hardware.goal_terminal=False
    elif fault=='failed_result':c.hardware.overrides[c.lease.QUERY]={'goal_status':4,'goal_error_code':-5}
    elif fault=='pending':c.hardware.overrides[c.lease.QUERY]={'pending_complete':False}
    else:c.hardware.overrides[c.lease.QUERY]={'stationary':False}
    with pytest.raises(ValueError):node._record_completed_controller_reference(c.contract.wire_plan())
    assert node._completed_command_reference is None
    assert not any(op==c.lease.QUIESCE for _,op,_ in c.events)


def test_r22_manual_cleanup_clears_automatic_completion_without_new_owner(monkeypatch):
    c=expired_manual();node=gateway(c,monkeypatch)
    node._manual_control_active=lambda:True
    node._completed_command_reference={'old_failed_attempt':True}
    node._observation_sync_hold_evidence={'old':True}
    events=[]
    node._physical_reference_transaction_hold=lambda *a,**kw:events.append((a,kw))
    assert node._cleanup_manual_physical_reference_lease(c.lease)
    assert node._physical_reference_lease is None and c.lease.released
    assert node._completed_command_reference is None and node._observation_sync_hold_evidence is None
    assert len(events)==1 and events[0][1]['manual_cleanup'] is True
    assert node._last_physical_manual_cleanup['automatic_owner_restored'] is False
    assert node._last_physical_manual_cleanup['original_execution_result_preserved'] is True
    with pytest.raises(ValueError):node._controller_reference_for_execution(node.joint_names[:-1])


def test_r22_failed_manual_reference_transaction_cannot_clear_or_restore_lease(monkeypatch):
    c=expired_manual();node=gateway(c,monkeypatch);node._manual_control_active=lambda:True
    old={'old_failed_attempt':True};node._completed_command_reference=old
    def failed(*a,**kw):raise ValueError('same-word hold not confirmed')
    node._physical_reference_transaction_hold=failed
    with pytest.raises(ValueError):node._cleanup_manual_physical_reference_lease(c.lease)
    assert node._physical_reference_lease is c.lease and not c.lease.released
    assert node._completed_command_reference is old
    assert not hasattr(node,'_last_physical_manual_cleanup')
