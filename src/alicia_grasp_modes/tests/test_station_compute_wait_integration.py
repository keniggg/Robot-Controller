"""One actual producer publication drives both real consumers.

Image/geometry algorithms, contract construction, callbacks, and wait/recovery
are real. ROS transport, clocks, and physical task plan creation are inert.
"""
from collections import deque
from copy import deepcopy
import json
import threading
import pytest
from types import SimpleNamespace as NS

from sensor_msgs.msg import JointState
from test_observation_measurement_remote import context, at
from test_task_modes import configured, module as task_module
from test_contact_linear_compensation import recorded
from alicia_grasp_modes import station_compute_wait_task as task_wait
import test_station_compute_wait_remote as remote_case


@pytest.mark.parametrize('connect_geometry_transport', [False, True])
def test_same_actual_producer_ready_overrun_recovery_joins_remote_and_task(context, monkeypatch, connect_geometry_transport):
    c=context; states=[]; task=[None]
    original_deliver=remote_case.deliver
    def both(remote, row, order=('mask','object','status')):
        original_deliver(remote,row,order)
        first = task[0] is None
        if first:
            node,_=configured.__wrapped__(monkeypatch)
            monkeypatch.setattr(task_module.rospy,'get_param',lambda name,default=None:c.params.get(name,default))
            monkeypatch.setattr(task_module.rospy,'logwarn_throttle',lambda *a:None)
            monkeypatch.setattr(task_wait.time,'monotonic',lambda:c.clock[0])
            node._mode_selection=deepcopy(c.node._mode_selection)
            node._mode_bound_selection=deepcopy(c.node._mode_selection)
            node._mode_ready=True;node.active=True;node.stage=2
            _,_,plan,_,_=recorded(2)
            plan.header=deepcopy(c.node._observe_return_prefix.header)
            plan.plan_id=c.node._observe_return_prefix.plan_id
            plan.target_track_id=c.identity.track_id
            plan.object_geometry.target_track_id=c.identity.track_id
            plan.diagnostic='FAR_FIELD_OBSERVATION_PLAN'
            plan.object_geometry.pose_base=deepcopy(row['object'].pose_base.pose)
            node._bound_execution_plan=plan
            node._bound_execution_plan_id=plan.plan_id
            node._bound_execution_plan_digest=node._execution_plan_digest(plan)
            node.latest_grasp6d_plan=deepcopy(plan)
            node._execution_authority_revoked=False
            node._observe_return_session=deepcopy(c.node._observe_return_session)
            node._observation_measurement_request=deepcopy(c.node._measurement_remote_request)
            node._published_near_field_phase=deepcopy(c.node._station_compute_phase)
            node._near_field_phase_id=node._published_near_field_phase.phase_id
            node._near_field_phase_deadline_sec=node._published_near_field_phase.deadline.to_sec()
            node._near_field_active=True
            node._following_evidence_lock=threading.RLock()
            node._following_epoch_ns=c.producer._rgb_interest_epoch
            node._following_accepted_history=deque(maxlen=256)
            node._following_sdk_history=deque(maxlen=256)
            task[0]=node
            if connect_geometry_transport:
                original_geometry_publish = c.node.geometry_pub.publish
                def publish_geometry(msg):
                    original_geometry_publish(msg)
                    node.target_geometry_cb(deepcopy(msg))
                c.node.geometry_pub.publish = publish_geometry
            original_joint=c.joint
            def joint(stamp,q=None):
                original_joint(stamp,q)
                ns,actual_q=c.producer._rgb_interest_feedback[-1]
                msg=JointState();msg.header.stamp=at(ns);msg.header.frame_id='sdk_measured'
                msg.name=['Joint%d'%i for i in range(1,7)];msg.position=list(actual_q)
                node.accepted_joint_cb(msg)
            c.joint=joint
        n=task[0]
        for ns,q in (c.producer._rgb_interest_feedback if first else ()):
            msg=JointState();msg.header.stamp=at(ns);msg.header.frame_id='sdk_measured'
            msg.name=['Joint%d'%i for i in range(1,7)];msg.position=list(q)
            n.accepted_joint_cb(msg)
        for key in order:
            if key=='status':n._unknown_support_status_cb(NS(data=json.dumps(row[key])))
            elif key=='mask':n._station_compute_task_mask_cb(deepcopy(row[key]))
            else:n.obj_cb(deepcopy(row[key]))
        record=n._station_task_record
        if record is None:
            n._station_task_context(row['status']['station_compute_wait'], admission=True,
                source_ns=row['status']['source_stamp_ns'])
            assert n._observed_target_matches_plan(n._bound_execution_plan, row['object'], c.params.get('/grasp',{}))
        assert record is not None and not n._station_task_retired
        if row['status']['state']=='processing_deadline_pending':
            assert record['pending'] and n.latest_obj is None
            assert c.node._station_compute_pending
            fault=n._station_task_plan_failure(n._bound_execution_plan)
            assert fault.code=='TARGET_PROCESSING_PENDING'
        else:
            assert not record['pending']
            assert record['contract']==row['status']['station_compute_wait']
            assert record['contract']==c.node._station_compute_reference['contract']
            assert n.latest_obj.header.stamp==row['object'].header.stamp
        states.append((row['status']['state'],deepcopy(record)))
    monkeypatch.setattr(remote_case,'deliver',both)
    remote_case.test_actual_producer_three_frame_arrival_then_slow_discard_and_fresh_recovery(c,monkeypatch)
    assert [state for state,_ in states]==['ready','processing_deadline_pending','ready']
    assert states[0][1]['contract']==states[1][1]['contract']
    assert states[0][1]['identity_mono']==states[1][1]['identity_mono']
    assert states[2][1]['contract']['source_ns']>states[0][1]['contract']['source_ns']
