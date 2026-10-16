"""A real producer's arrival gap/recovery crosses both three-topic consumers.

GEN18 source/feedback timing is recorded. Images, fixed station pose, ROS
transport and task plan are inert fixtures; no physical recovery is claimed.
"""
from collections import deque
from copy import deepcopy
import itertools
import json
import threading
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
from sensor_msgs.msg import JointState
from alicia_flexible_grasp_supervisor.msg import NearFieldPlanningPhase, GraspState
from alicia_grasp_modes import station_compute_wait_task as task_wait

from test_arrival_feedback_bracket_gap import RECORD, frame
from test_contact_linear_compensation import recorded
from test_locked_rgb_hold import custom_frame, module as producer_module
from test_observation_measurement_remote import (
    context, at, begin, qualified_ack, request, deliver_one,
)
from test_observation_measurement_unknown import send, ack, arrival, next_frame
from test_station_compute_wait_remote import feedback
from test_task_modes import configured, module as task_module
from test_unknown_pending_identity import publication, deliver


def _feed_consumers(c):
    """Deliver only actual fixture accepted samples; never synthesize freshness."""
    c.node.latest_joint_state.header.stamp = at(c.node._ros_source_clock_ns())
    for ns, q in c.producer._rgb_interest_feedback:
        feedback(c, ns, q)
        if getattr(c, 'task', None) is not None:
            msg = JointState()
            msg.header.stamp = at(ns)
            msg.header.frame_id = 'sdk_measured'
            msg.name = ['Joint%d' % i for i in range(1, 7)]
            msg.position = list(q)
            c.task.accepted_joint_cb(msg)


def _deliver_both(c, row, order=('mask', 'object', 'status')):
    for key in order:
        deliver(c.node, row, (key,))
        if key == 'status':
            c.task._unknown_support_status_cb(NS(data=json.dumps(row[key])))
        elif key == 'mask':
            c.task._station_compute_task_mask_cb(deepcopy(row[key]))
        else:
            c.task.obj_cb(deepcopy(row[key]))


@pytest.fixture
def chain(context, monkeypatch):
    c = context
    p, n = c.producer, c.node
    p._observation_measurement_ack_pub = Mock()
    started = begin(c)
    send(p, started)
    reached = arrival(p, c.clock, c.joint, started)
    assert request(n, reached)['accepted']
    for _ in range(2):
        next_frame(p, c.clock, c.joint)
        assert not qualified_ack(c)
    next_frame(p, c.clock, c.joint)
    row = publication(p)
    row['ack'] = ack(p)
    for key in ('status', 'ack', 'object', 'mask'):
        deliver_one(c, row, key)
    assert qualified_ack(c)
    c.clock[0] += .01
    done = dict(reached, state='qualified', request_id='completed-gap-integration',
        stamp_ns=n._ros_source_clock_ns(), source_stamps_ns=row['ack']['source_stamps_ns'])
    send(p, done)
    assert ack(p)['accepted'] and request(n, done)['accepted']
    n._observe_return_phase = 'observing_contact'
    n._observe_return_paused = False
    n._station_compute_task_state = 'PLAN_PREGRASP'
    n.robot_execution_active = True
    phase = NearFieldPlanningPhase()
    phase.active = True
    phase.phase_id = 7
    phase.planning_stage = 'contact_replan'
    phase.header.stamp = producer_module.rospy.Time.now()
    phase.header.frame_id = 'base_link'
    phase.deadline = at(n._ros_source_clock_ns() + 30_000_000_000)
    phase.reference_source_stamp = at(row['ack']['source_stamps_ns'][-1])
    phase.reference_target_track_id = c.identity.track_id
    phase.reference_center_valid = True
    n._station_compute_phase = phase
    p._station_compute_phase_cb(phase)
    p._station_compute_state_cb(GraspState(state='PLAN_PREGRASP', active=True))
    n.latest_joint_state = JointState(name=['Joint%d' % k for k in range(1, 7)],
        position=p._locked_rgb_hold_reference['observation_arrival_retry']['arrival_q'].tolist())
    _feed_consumers(c)
    c.clock[0] += .3
    custom_frame(p, c.clock, c.joint, target_depth=.35)
    row = publication(p)
    assert row['status']['state'] == 'ready' and 'station_compute_wait' in row['status']

    task, _ = configured.__wrapped__(monkeypatch)
    monkeypatch.setattr(task_module.rospy, 'get_param', lambda name, default=None: c.params.get(name, default))
    monkeypatch.setattr(task_module.rospy, 'logwarn_throttle', lambda *a: None)
    monkeypatch.setattr(task_wait.time, 'monotonic', lambda: c.clock[0])
    task._mode_selection = deepcopy(n._mode_selection)
    task._mode_bound_selection = deepcopy(n._mode_selection)
    task._mode_ready = True
    task.active = True
    task.stage = 2
    _, _, plan, _, _ = recorded(2)
    plan.header = deepcopy(n._observe_return_prefix.header)
    plan.plan_id = n._observe_return_prefix.plan_id
    plan.target_track_id = c.identity.track_id
    plan.object_geometry.target_track_id = c.identity.track_id
    plan.diagnostic = 'FAR_FIELD_OBSERVATION_PLAN'
    plan.object_geometry.pose_base = deepcopy(row['object'].pose_base.pose)
    task._bound_execution_plan = plan
    task._bound_execution_plan_id = plan.plan_id
    task._bound_execution_plan_digest = task._execution_plan_digest(plan)
    task.latest_grasp6d_plan = deepcopy(plan)
    task._execution_authority_revoked = False
    task._observe_return_session = deepcopy(n._observe_return_session)
    task._observation_measurement_request = deepcopy(n._measurement_remote_request)
    task._published_near_field_phase = deepcopy(phase)
    task._near_field_phase_id = phase.phase_id
    task._near_field_phase_deadline_sec = phase.deadline.to_sec()
    task._near_field_active = True
    task._following_evidence_lock = threading.RLock()
    task._following_epoch_ns = p._rgb_interest_epoch
    task._following_accepted_history = deque(maxlen=256)
    task._following_sdk_history = deque(maxlen=256)
    c.task = task
    original_publish = n.geometry_pub.publish

    def geometry_transport(message):
        original_publish(message)
        task.target_geometry_cb(deepcopy(message))

    n.geometry_pub.publish = geometry_transport
    _feed_consumers(c)
    _deliver_both(c, row)
    assert n._station_compute_reference is not None
    assert task._station_task_record is not None
    assert n._station_compute_reference['contract'] == task._station_task_record['contract']
    c.gap_scene = NS(p=p, clock=c.clock, joint=c.joint, params=c.params, ready=row,
        shift=RECORD['reference_ns'] - p.last_good_source_stamp_ns, lookup=p.buffer.lookup_transform)
    return c


@pytest.mark.parametrize('order', list(itertools.permutations(('object', 'mask', 'status'))))
@pytest.mark.parametrize('deliver_gap_before_recovery', [True, False])
def test_real_arrival_gap_three_way_recovery_and_late_messages(
        chain, order, deliver_gap_before_recovery):
    c = chain
    p, n, task = c.producer, c.node, c.task
    contract = deepcopy(n._station_compute_reference['contract'])
    original_reference = p._locked_rgb_hold_reference
    original_target = p.tracker.last
    identity_mono = task._station_task_record['identity_mono']
    phase_deadline = task._near_field_phase_deadline_sec
    bad = frame(c.gap_scene)
    assert bad['status']['state'] == 'processing_deadline_pending'
    assert bad['status']['detail'] == 'station_accepted_feedback_bracket_pending_revalidation'
    assert not bad['object'].detected
    assert p._locked_rgb_hold_reference is original_reference and p.tracker.last is original_target
    assert bad['status']['station_compute_wait'] == contract
    assert bad['status']['station_feedback_gap']['feedback_gap_evidence']['span_ns'] == 226932289
    _feed_consumers(c)
    if deliver_gap_before_recovery:
        _deliver_both(c, bad, order)
        assert n._station_compute_pending and not n._station_compute_soft_wait
        assert task._station_task_record['pending'] and task.latest_obj is None
        assert task._station_task_plan_failure(task._bound_execution_plan).code == 'TARGET_PROCESSING_PENDING'
        assert not n.geometry_pub.messages[-1].valid
        assert task._station_task_record['contract'] == contract
        assert task._station_task_record['identity_mono'] == identity_mono
        assert not task._station_task_retired and not task._execution_authority_revoked

    recovered = frame(c.gap_scene, good=True)
    assert recovered['status']['state'] == 'ready', recovered['status']
    assert recovered['status']['station_feedback_gap_recovery'] == bad['status']['station_feedback_gap']
    assert recovered['status']['station_compute_recovery'] == contract
    assert n._ros_source_clock_ns() > contract['geometry_deadline_ns']
    new_source = recovered['object'].header.stamp.to_nsec()
    _feed_consumers(c)
    for key in order[:-1]:
        _deliver_both(c, recovered, (key,))
        assert n._station_compute_reference['contract']['source_ns'] != new_source
        assert task._station_task_record['contract']['source_ns'] != new_source
    _deliver_both(c, recovered, order[-1:])
    assert not n._station_compute_pending and not n._station_compute_retired
    assert not task._station_task_record['pending'] and not task._station_task_retired
    assert n.latest_object.header.stamp.to_nsec() == task.latest_obj.header.stamp.to_nsec() == new_source
    assert n._station_compute_reference['contract'] == task._station_task_record['contract']
    assert n._current_stream_target_identity() == c.identity
    assert task._near_field_phase_deadline_sec == phase_deadline
    assert not task._execution_authority_revoked

    # Even when *all* gap packets missed their original window, the complete
    # READY proof has taught both consumers precisely which old source to ignore.
    new_contract = deepcopy(task._station_task_record['contract'])
    for key in order:
        _deliver_both(c, bad, (key,))
        assert not n._station_compute_pending and not n._station_compute_retired
        assert not task._station_task_record['pending'] and not task._station_task_retired
        assert task._station_task_record['contract'] == new_contract
        assert n.latest_object.header.stamp.to_nsec() == task.latest_obj.header.stamp.to_nsec() == new_source
        assert not task._execution_authority_revoked
