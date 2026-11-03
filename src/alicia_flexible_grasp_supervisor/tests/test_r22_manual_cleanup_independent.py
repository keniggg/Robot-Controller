"""Explicit manual cleanup handler with fake transport and real lease checks."""
import json
import threading
from types import SimpleNamespace as NS
import pytest
import rospy
from std_msgs.msg import Header

from test_r22_typed_client_independent import expired_manual
from test_r22_gateway_reference_independent import gateway


def case(monkeypatch):
    c=expired_manual();node=gateway(c,monkeypatch)
    node._gui_direct_mode=True;node._manual_control_active=lambda:True
    node._observation_context_lock=threading.RLock()
    node._observation_contexts=NS(active=False)
    node._observation_task_requires_inactive=False
    node._observation_task_state_stamp_ns=c.clock.ros-1_000_000
    stamp=node._observation_task_state_stamp_ns
    node._observation_last_task_message=NS(active=False,
        header=Header(stamp=rospy.Time(stamp//10**9,stamp%10**9)))
    node._observation_task_sub=NS(get_num_connections=lambda:1)
    node._revoke_observation_task=lambda:setattr(node,'_observation_task_requires_inactive',True)
    node._observation_sync_hold_evidence={'old':True}
    holds=[]
    node._physical_reference_transaction_hold=lambda *a,**kw:holds.append((a,kw))
    c.node=node;c.holds=holds;c.events.clear()
    return c


def test_r22_explicit_manual_cleanup_requires_idle_and_reports_software_only(monkeypatch):
    c=case(monkeypatch);result=c.node.handle_cleanup_physical_reference_manual(NS())
    assert result.success,result.message
    report=json.loads(result.message)
    assert report['code']=='PHYSICAL_MANUAL_CLEANUP_COMPLETE'
    assert report['motion_submitted'] is False and report['automatic_owner_restored'] is False
    assert report['original_execution_result_preserved'] is True and report['transaction_receipts']
    assert c.node._physical_reference_lease is None and c.lease.released
    assert len(c.holds)==1 and c.holds[0][1]['manual_cleanup'] is True


def test_r22_no_old_lease_reports_already_clear_without_any_transport(monkeypatch):
    c=case(monkeypatch);c.node._physical_reference_lease=None
    result=c.node.handle_cleanup_physical_reference_manual(NS())
    assert result.success and not c.events and not c.holds
    assert json.loads(result.message)['automatic_owner_restored'] is False


@pytest.mark.parametrize('fault',['gui','manual_owner','epoch','task_active','inactive_required',
    'context_active','old_epoch_message','future_message','message_stamp_mismatch','disconnected',
    'goal_not_terminal','sdk_manual_missing'])
def test_r22_cleanup_cannot_infer_manual_idle_from_cleared_flags_or_stale_messages(monkeypatch,fault):
    c=case(monkeypatch);n=c.node;m=n._observation_last_task_message
    if fault=='gui':n._gui_direct_mode=False
    elif fault=='manual_owner':n._manual_control_active=lambda:False
    elif fault=='epoch':n._control_reference_epoch_ns+=1
    elif fault=='task_active':m.active=True
    elif fault=='inactive_required':n._observation_task_requires_inactive=True
    elif fault=='context_active':n._observation_contexts.active=True
    elif fault in ('old_epoch_message','future_message'):
        stamp=c.lease.model.epoch_ns-1 if fault=='old_epoch_message' else c.clock.ros+1
        m.header.stamp=rospy.Time(stamp//10**9,stamp%10**9);n._observation_task_state_stamp_ns=stamp
    elif fault=='message_stamp_mismatch':n._observation_task_state_stamp_ns+=1
    elif fault=='disconnected':n._observation_task_sub.get_num_connections=lambda:0
    elif fault=='goal_not_terminal':c.hardware.overrides[c.lease.QUERY].update(goal_observed=True,goal_terminal=False)
    else:c.hardware.overrides[c.lease.QUERY]['manual_reference']=False
    result=n.handle_cleanup_physical_reference_manual(NS())
    assert not result.success,result.message
    assert n._physical_reference_lease is c.lease and not c.lease.released and not c.holds
    assert not any(op==c.lease.RELEASE_PREPARE for _,op,_ in c.events)


def test_r22_manual_authority_lost_during_hold_prevents_release_commit(monkeypatch):
    c=case(monkeypatch)
    def lose_manual(*args,**kw):c.node._gui_direct_mode=False
    c.node._physical_reference_transaction_hold=lose_manual
    result=c.node.handle_cleanup_physical_reference_manual(NS())
    assert not result.success and not c.lease.released
    assert c.node._physical_reference_lease is c.lease
    assert not any(op==c.lease.RELEASE_COMMIT for _,op,_ in c.events)


def test_r22_manual_cleanup_waiting_for_planner_lock_revalidates_owner_after_acquisition(monkeypatch):
    c=case(monkeypatch);lock=threading.Lock();c.node._planner_lock=lock
    lock.acquire();started=threading.Event();done=[]
    def invoke():started.set();done.append(c.node.handle_cleanup_physical_reference_manual(NS()))
    worker=threading.Thread(target=invoke);worker.start();assert started.wait(1.)
    assert not c.events
    c.node._gui_direct_mode=False;lock.release();worker.join(1.)
    assert not worker.is_alive() and len(done)==1 and not done[0].success
    assert not c.events and c.node._physical_reference_lease is c.lease
