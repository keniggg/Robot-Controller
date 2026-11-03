"""Actual producer support loss/recovery routed through both real consumers.

Synthetic RGB-D/clock/ROS transport only. The producer's support and RGB checks,
registered-parent envelope, invalid geometry callback, source joins and all
retirement decisions remain production implementations.
"""
from copy import deepcopy
from types import SimpleNamespace as NS
import pytest

from test_station_requalification_three_owners import owners, task_deliver
from test_unknown_pending_identity import publication, deliver
from test_support_measurement_hold import support_gap_depth
from test_locked_rgb_hold import custom_frame


def wire(c,row,order):
    deliver(c.node,row,order)
    task_deliver(NS(node=c.task),row,order)


@pytest.mark.parametrize('order',[('status','object','mask'),('object','mask','status'),('mask','status','object')])
def test_actual_producer_support_hold_withdraws_both_owners_then_full_new_ready_recovers(monkeypatch,order):
    c=owners(monkeypatch);p=c.producer;r=c.node;t=c.task
    parent=deepcopy(c.ready['status']['station_compute_wait']); old_identity=c.identity
    oldmono=(r._station_compute_reference['identity_monotonic'],t._station_task_record['identity_mono'])
    # Include the real remote invalid geometry -> task status boundary, which
    # formerly could turn a local hold into TARGET_LOST before fresh READY.
    original_publish=r.geometry_pub.publish
    def geometry(msg):
        original_publish(msg);t.target_geometry_cb(deepcopy(msg))
    monkeypatch.setattr(r.geometry_pub,'publish',geometry)
    c.clock[0]+=.15;custom_frame(p,c.clock,c.joint,depth_image=support_gap_depth())
    pending=publication(p)
    assert pending['status']['state']=='processing_deadline_pending',pending['status']
    assert pending['status']['detail']=='support_measurement_pending_revalidation'
    assert pending['status']['station_compute_wait']==parent
    gap=deepcopy(pending['status']['station_support_gap'])
    wire(c,pending,order)
    assert p.first_loss is None and not p.tracker.lost
    assert r._station_compute_pending and not r._station_compute_retired
    assert t._station_task_record['pending'] and not t._station_task_retired
    assert t._unknown_target_loss is None
    assert r._current_stream_target_identity()==old_identity
    assert not r.latest_object_geometry.valid and not r.latest_preview_rich_plan.valid
    assert t.latest_obj is None
    assert oldmono==(r._station_compute_reference['identity_monotonic'],t._station_task_record['identity_mono'])
    c.clock[0]+=.15;custom_frame(p,c.clock,c.joint,target_depth=.35)
    ready=publication(p)
    assert ready['status']['state']=='ready',ready['status']
    assert ready['status']['station_support_gap_recovery']==gap
    assert ready['status']['station_compute_recovery']==parent
    for key in order[:-1]:
        wire(c,ready,(key,))
        assert r._station_compute_pending and t._station_task_record['pending']
    wire(c,ready,order[-1:])
    assert not r._station_compute_pending and not r._station_compute_retired
    assert not t._station_task_record['pending'] and not t._station_task_retired
    assert r._station_compute_reference['contract']==ready['status']['station_compute_wait']
    assert t._station_task_record['contract']==ready['status']['station_compute_wait']
    assert t.latest_obj.detected and r.latest_object.detected
    assert r._current_stream_target_identity()==old_identity and t._unknown_target_loss is None
    assert not getattr(r.latest_preview_rich_plan, 'valid', False)  # No new or old execution plan.
    # Late old pending/false cannot revoke the fully joined newer measurement.
    wire(c,pending,order)
    assert not r._station_compute_retired and not t._station_task_retired
    assert not r._station_compute_pending and not t._station_task_record['pending']
    assert t.latest_obj.header.stamp==ready['object'].header.stamp


@pytest.mark.parametrize('fault',['deadline','true_loss','manual'])
def test_fixed_identity_deadline_and_real_revocation_still_retire_both_owners(monkeypatch,fault):
    c=owners(monkeypatch);p=c.producer;r=c.node;t=c.task
    c.clock[0]+=.15;custom_frame(p,c.clock,c.joint,depth_image=support_gap_depth())
    pending=publication(p);wire(c,pending,('status','object','mask'))
    gap=pending['status']['station_support_gap']
    parent=pending['status']['station_compute_wait']
    assert gap['deadline_ns']==parent['identity_deadline_ns']
    if fault=='deadline':
        c.clock[0]=gap['deadline_ns']/1e9+.001;c.joint(c.clock[0]-.001)
        r._station_compute_poll();t._station_task_check()
    elif fault=='manual':
        c.params['/gui/joint_direct_mode']=True
        r._station_compute_poll();t._station_task_check()
    else:
        loss=deepcopy(pending);loss['status']['target_lost']=True
        wire(c,loss,('status','object','mask'))
    assert r._station_compute_retired
    assert t._station_task_retired
    assert not getattr(r.latest_preview_rich_plan, 'valid', False)


def test_actual_complete_ready_can_arrive_before_old_pending_without_reviving_old_geometry(monkeypatch):
    c=owners(monkeypatch);p=c.producer;r=c.node;t=c.task
    c.clock[0]+=.15;custom_frame(p,c.clock,c.joint,depth_image=support_gap_depth())
    delayed=publication(p)
    assert delayed['status']['state']=='processing_deadline_pending'
    c.clock[0]+=.15;custom_frame(p,c.clock,c.joint,target_depth=.35)
    ready=publication(p)
    assert ready['status']['station_support_gap_recovery']==delayed['status']['station_support_gap']
    wire(c,ready,('object','mask','status'))
    assert not r._station_compute_retired and not t._station_task_retired
    assert r.latest_object.header.stamp==ready['object'].header.stamp
    assert t.latest_obj.header.stamp==ready['object'].header.stamp
    wire(c,delayed,('object','status','mask'))
    assert not r._station_compute_pending and not t._station_task_record['pending']
    assert not r._station_compute_retired and not t._station_task_retired
    assert r.latest_object.header.stamp==ready['object'].header.stamp
    assert t.latest_obj.header.stamp==ready['object'].header.stamp
