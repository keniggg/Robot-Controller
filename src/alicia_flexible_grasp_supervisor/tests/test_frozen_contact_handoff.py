"""Observed-contact task freeze -> dedicated notice -> actual gateway receipt.

No ROS master/hardware. Existing motion/collision fixtures model transport;
this validates authority handoff, not real grasp feasibility.
"""
from copy import deepcopy
from types import MethodType, SimpleNamespace as NS
from unittest.mock import Mock
import json
import threading

import pytest
import rospy
from std_msgs.msg import String

from test_contact_pregrasp_gateway import case, captured_session
from test_observed_contact_base_execution import current_view_case
from alicia_grasp_modes.observe_return_task import ObserveReturnTask
from alicia_flexible_grasp.grasp.frozen_contact_handoff import (
    build_contact_freeze_notice, parse_contact_freeze_notice)
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages


def connected(monkeypatch):
    task = current_view_case(monkeypatch)
    gateway = case(monkeypatch, receipt=False)
    gateway.plan = deepcopy(task.contact)
    origin = captured_session(gateway, monkeypatch)
    gateway.node._contact_pregrasp_receipts = {}
    gateway.node._compensation_visual_context = None
    gateway.node._contact_frozen_notifications = {}
    gateway.node._contact_frozen_admissions = {}
    task.node._mode_bound_selection = deepcopy(gateway.params['/grasp_mode/selection'])
    task.node._observe_return_session = deepcopy(origin['view_session'])
    task.node._observe_return_origin_proof = dict(epoch_ns=gateway.node._control_reference_epoch_ns)
    task.node._following_evidence_lock = threading.RLock()
    task.node._following_epoch_ns = gateway.node._control_reference_epoch_ns
    task.node._near_field_phase_id = 1
    task.node._near_field_phase_deadline_sec = 20.
    task.node._published_near_field_phase = NS(active=True, phase_id=1,
        planning_stage='contact_replan', reference_target_track_id=task.contact.target_track_id,
        deadline=rospy.Time.from_sec(20.))
    task.node._selection_current = lambda s:s == gateway.params['/grasp_mode/selection']
    task.node._observe_return_live = MethodType(ObserveReturnTask._observe_return_live,task.node)
    task.node._after_near_field_contact_freeze = MethodType(
        ObserveReturnTask._after_near_field_contact_freeze,task.node)
    task.node._frozen_contact_pub = Mock(side_effect=None)
    def publish(msg):
        assert task.node._bound_execution_plan.plan_id == task.contact.plan_id
        assert not task.node._execution_authority_revoked
        gateway.state['now'] = task.clock[0]
        gateway.node._frozen_contact_plan_cb(msg)
        assert gateway.node._contact_pregrasp_receipts == {}
    task.node._frozen_contact_pub.publish.side_effect = publish
    return NS(task=task,gateway=gateway,origin=origin)


def freeze(c):
    result = c.task.node._maybe_rebind_near_field_grasp6d_plan(c.task.cfg,{},c.task.observation)
    assert result is not None,c.task.states
    return result


def test_real_preview_loop_freezes_then_notifies_then_gateway_checks_without_motion(monkeypatch):
    c=connected(monkeypatch)
    # Same Preview object is not a receipt, even with valid current context.
    assert c.gateway.node._contact_pregrasp_receipts == {}
    frozen=freeze(c)
    notice=json.loads(c.task.node._frozen_contact_pub.publish.call_args.args[0].data)
    assert notice['source_ns']==c.task.contact.header.stamp.to_nsec()
    assert notice['issued_ns']>notice['source_ns']
    assert notice['phase_deadline_ns']==20_000_000_000
    assert c.gateway.node._contact_pregrasp_receipts == {}
    result=c.gateway.node.handle_contact_pregrasp(NS(plan=frozen,execute=False))
    assert result.success,result.message
    assert c.gateway.node._contact_pregrasp_receipts[frozen.plan_id]['source_ns']==notice['source_ns']
    assert c.gateway.node._contact_frozen_admissions[frozen.plan_id]==notice
    assert c.gateway.state['moves']==0


@pytest.mark.parametrize('fault',['epoch','selection','session','view','digest','source','expired'])
def test_delayed_or_changed_notice_cannot_admit_contact(monkeypatch,fault):
    c=connected(monkeypatch);frozen=freeze(c)
    notice=c.gateway.node._contact_frozen_notifications[frozen.plan_id]
    if fault=='epoch':notice['epoch_ns']+=1
    elif fault=='selection':notice['selection']['generation']+=1
    elif fault=='session':notice['session']['session_id']='other'
    elif fault=='view':notice['session']['view_index']+=1
    elif fault=='digest':notice['plan_sha256']='0'*64
    elif fault=='source':notice['source_ns']+=1
    elif fault=='expired':c.gateway.state['now']=20.1
    result=c.gateway.node.handle_contact_pregrasp(NS(plan=frozen,execute=False))
    assert not result.success,result.message
    assert c.gateway.node._contact_pregrasp_receipts == {}
    c.gateway.select.assert_not_called()
    assert not c.gateway.calls


@pytest.mark.parametrize('fault',['manual','terminal','invalid','epoch','reactivated'])
def test_revocation_before_late_notice_never_recreates_authority(monkeypatch,fault):
    c=connected(monkeypatch);frozen=freeze(c)
    message=c.task.node._frozen_contact_pub.publish.call_args.args[0]
    if fault=='manual':
        c.gateway.state['manual']=True
        c.gateway.node._revoke_observation_task()
    elif fault=='terminal':c.gateway.node._revoke_observation_task()
    elif fault=='reactivated':
        c.gateway.node._revoke_observation_task()
        c.gateway.node._observation_task_state_stamp_ns=0
        c.gateway.state['now']=10.1
        for active,stamp in ((False,10.06),(True,10.07)):
            c.gateway.node._observation_task_state_cb(NS(header=NS(stamp=rospy.Time.from_sec(stamp)),
                stage=GraspStages.MOVE_PREGRASP,state='MOVE_PREGRASP',active=active,message='planning'))
        assert c.gateway.node._observation_contexts.active
        assert not c.gateway.node._observation_task_requires_inactive
    elif fault=='invalid':
        invalid=deepcopy(c.gateway.plan);invalid.valid=False
        c.gateway.node._committed_observation_plan_cb(invalid)
    else:c.gateway.node._control_reference_epoch_ns+=1
    c.gateway.node._frozen_contact_plan_cb(message)
    result=c.gateway.node.handle_contact_pregrasp(NS(plan=frozen,execute=False))
    assert not result.success,result.message
    assert not c.gateway.node._contact_pregrasp_receipts
    c.gateway.select.assert_not_called()


@pytest.mark.parametrize('fault',['stale_source','target_moved','changed_content','epoch'])
def test_failed_task_admission_or_freeze_never_notifies_gateway(monkeypatch,fault):
    c=connected(monkeypatch)
    if fault=='stale_source':
        c.task.contact.header.stamp=rospy.Time.from_sec(9.9)
        c.task.contact.plan_id=compute_plan_id(c.task.contact)
    elif fault=='target_moved':c.task.node.latest_obj=c.task.helper._object_at(.46,stamp_sec=10.)
    elif fault=='changed_content':c.task.contact.poses[0].position.x+=.01
    else:c.task.node._following_epoch_ns+=1
    assert c.task.node._maybe_rebind_near_field_grasp6d_plan(c.task.cfg,{},c.task.observation) is None
    c.task.node._frozen_contact_pub.publish.assert_not_called()
    assert not c.gateway.node._contact_frozen_notifications


def test_admitted_contact_keeps_original_session_after_computation_phase_ends(monkeypatch):
    c=connected(monkeypatch);frozen=freeze(c)
    first=c.gateway.node.handle_contact_pregrasp(NS(plan=frozen,execute=False))
    assert first.success,first.message
    first_report=json.loads(first.message)
    c.gateway.state['now']=20.1
    second=c.gateway.node.handle_contact_pregrasp(NS(plan=frozen,execute=False))
    assert second.success,second.message
    second_report=json.loads(second.message)
    assert first_report['outer_session_deadline']['ros_deadline_ns']==second_report['outer_session_deadline']['ros_deadline_ns']==25_000_000_000
    assert second_report['outer_session_deadline']['monotonic_deadline']<first_report['outer_session_deadline']['monotonic_deadline']
    c.gateway.state['now']=25.1
    assert not c.gateway.node.handle_contact_pregrasp(NS(plan=frozen,execute=False)).success
    assert not c.gateway.calls


def test_admitted_receipt_cannot_follow_same_session_into_different_view(monkeypatch):
    c=connected(monkeypatch);frozen=freeze(c)
    assert c.gateway.node.handle_contact_pregrasp(NS(plan=frozen,execute=False)).success
    c.origin['view_session']['view_index']=2
    c.origin['view_session']['view_plan_id']='f'*24
    c.origin['view_session']['view_request_id']='other-view'
    result=c.gateway.node.handle_contact_pregrasp(NS(plan=frozen,execute=False))
    assert not result.success and 'FROZEN_ADMISSION_CHANGED' in result.message
    assert not c.gateway.calls


def test_exact_decimal_wire_stamps_normalize_without_relaxing_integrity(monkeypatch):
    c=connected(monkeypatch);freeze(c)
    notice=json.loads(c.task.node._frozen_contact_pub.publish.call_args.args[0].data)
    changed=deepcopy(notice)
    for k in ('source_ns','issued_ns','epoch_ns','phase_deadline_ns'):changed[k]=str(changed[k])
    changed['selection']['stamp_ns']=str(changed['selection']['stamp_ns'])
    for k in ('stamp_ns','deadline_ns','origin_snapshot_stamp_ns','view_source_stamp_ns'):
        changed['session'][k]=str(changed['session'][k])
    assert parse_contact_freeze_notice(json.dumps(changed))==notice
    changed['source_ns']=float(changed['source_ns'])
    with pytest.raises(ValueError):parse_contact_freeze_notice(changed)
