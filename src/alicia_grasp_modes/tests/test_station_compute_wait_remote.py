"""Actual remote callbacks and original source gates; no ROS or robot calls."""
from copy import deepcopy
import hashlib
import itertools
from types import SimpleNamespace as NS

import pytest
import numpy as np
from sensor_msgs.msg import JointState

from test_observation_measurement_remote import context, arrival, fresh, deliver_one, request, at
from test_unknown_pending_identity import deliver
from test_remote_modes import base
from alicia_grasp_modes.selection import SELECTION_PARAM
from alicia_grasp_modes.station_compute_reacquisition import build_contract
from alicia_grasp_modes.unknown_pending_identity import reference_binding
from alicia_grasp_modes import station_compute_wait_remote as implementation


def clock(c, ns):
    c.clock[0] = ns / 1e9
    c.node.latest_joint_state.header.stamp = at(c.node._ros_source_clock_ns())
    feedback(c, c.node._ros_source_clock_ns())


def feedback(c, ns, q=None):
    msg = deepcopy(c.node.latest_joint_state)
    msg.header.stamp = at(ns); msg.header.frame_id = 'sdk_measured'
    if q is not None:
        msg.position = list(q)
    c.node._station_compute_accepted_cb(msg)


def row_at(c, ns, recovery=None):
    row = deepcopy(c.ready)
    for name in ('object', 'mask'):
        row[name].header.stamp = at(ns)
    for key in ('source_stamp_ns', 'color_stamp_ns', 'depth_stamp_ns', 'tf_lookup_stamp_ns'):
        row['status'][key] = ns
    row['status'].pop('observation_measurement', None)
    row['status']['station_compute_wait'] = build_contract(source_ns=ns,
        maximum_frame_age_sec=.8, session_deadline_ns=c.session_deadline,
        phase_deadline_ns=c.phase.deadline.to_nsec(), binding=c.binding)
    mask = c.node.bridge.imgmsg_to_cv2(row['mask'], desired_encoding='mono8')
    row['status']['station_compute_mask'] = dict(
        sha256=hashlib.sha256(np.ascontiguousarray(mask != 0, dtype=np.uint8).tobytes()).hexdigest(),
        shape=list(mask.shape), frame_id=row['mask'].header.frame_id)
    if recovery is not None:
        row['status']['station_compute_recovery'] = deepcopy(recovery)
    return row


@pytest.fixture
def station(context, monkeypatch):
    c = context
    req = arrival(c)
    row, sources = fresh(c, req)
    for key in ('object', 'mask', 'status', 'ack'):
        deliver_one(c, row, key)
    c.clock[0] += .001
    qualified = dict(req, state='qualified', request_id='complete',
        stamp_ns=c.node._ros_source_clock_ns(), source_stamps_ns=sources)
    request(c.node, qualified)
    n = c.node
    n._observe_return_phase = 'observing_contact'
    n._observe_return_paused = False
    n._station_compute_task_state = 'PLAN_PREGRASP'
    n.robot_execution_active = True  # Waiting is an active task, not motion.
    c.phase = NS(active=True, planning_stage='contact_replan', phase_id=1,
        header=NS(stamp=at(n._ros_source_clock_ns())),
        deadline=at(n._ros_source_clock_ns() + 20_000_000_000),
        reference_source_stamp=at(sources[-1]), reference_center_valid=True,
        reference_target_track_id=c.identity.track_id)
    n._station_compute_phase = c.phase
    n.near_field_planning_active = True
    c.session_deadline = qualified['deadline_ns']
    n.latest_joint_state = JointState()
    n.latest_joint_state.name = ['joint%d' % k for k in range(1, 7)]
    n.latest_joint_state.position = [0.] * 6
    feedback(c, sources[-1])
    c.binding = dict(selection=deepcopy(n._mode_selection),
        **{k: qualified[k] for k in ('lease_id', 'session_id', 'view_plan_id',
            'view_index', 'target_epoch', 'target_track_id', 'control_epoch_ns')},
        phase_id=1, phase_started_ns=c.phase.header.stamp.to_nsec(),
        phase_reference_source_ns=sources[-1],
        reference_binding=reference_binding(row['status'], n._mode_selection),
        source_id='a'*64, camera_configuration_sha256='b'*64,
        config_sha256='c'*64, station_q_rad=[0.]*6)
    c.ready = row
    monkeypatch.setattr(implementation.time, 'monotonic', lambda: c.clock[0])
    ns = n._ros_source_clock_ns() + 100_000_000
    clock(c, ns+10_000_000)
    c.ready = row_at(c, ns)
    return c


def arm(c):
    deliver(c.node, c.ready)
    assert c.node._station_compute_reference is not None
    return deepcopy(c.node._station_compute_reference['contract'])


def pending(c):
    contract = arm(c)
    clock(c, contract['geometry_deadline_ns'] + 20_000_000)
    assert c.node._station_compute_poll()
    assert c.node._station_compute_pending
    return contract


@pytest.mark.parametrize('order', list(itertools.permutations(('object', 'mask', 'status'))))
def test_registration_needs_real_fresh_three_topic_join(station, order):
    c = station
    for name in order[:-1]:
        deliver(c.node, c.ready, (name,))
        assert c.node._station_compute_reference is None
    deliver(c.node, c.ready, order[-1:])
    assert c.node._station_compute_reference['contract'] == c.ready['status']['station_compute_wait']
    assert not c.node._station_compute_pending
    assert c.node._current_stream_target_identity() == c.identity


@pytest.mark.parametrize('order', list(itertools.permutations(('object', 'mask', 'status'))))
def test_expiry_cancels_geometry_then_original_new_join_recovers(station, order):
    c = station; n = c.node
    rich_before = len(n.rich_plan_pub.messages)
    contract = pending(c)
    assert len(n.rich_plan_pub.messages) == rich_before  # Historical view, never reissued/revived.
    assert not n.geometry_pub.messages[-1].valid
    assert n.latest_preview_rich_plan.diagnostic.startswith('TARGET_PROCESSING_PENDING:')
    old = NS(payload=(NS(stamp_ns=contract['source_ns'], sample_stamp_ns=(contract['source_ns'],)),))
    assert n._station_compute_ticket_code(old) == 'TARGET_PROCESSING_PENDING'
    ns = n._ros_source_clock_ns() + 100_000_000
    clock(c, ns+10_000_000)
    row = row_at(c, ns, recovery=contract)
    for name in order[:-1]:
        deliver(n, row, (name,))
        assert n._station_compute_pending
    deliver(n, row, order[-1:])
    assert not n._station_compute_pending
    assert n.latest_object.detected and n.latest_object.header.stamp.to_nsec() == ns
    assert n._current_stream_target_identity() == c.identity
    assert n._station_compute_ticket_code(old) == 'TARGET_PROCESSING_PENDING'
    mixed = NS(payload=(NS(stamp_ns=ns, sample_stamp_ns=(contract['source_ns'], ns)),))
    assert n._station_compute_ticket_code(mixed) == 'TARGET_PROCESSING_PENDING'
    assert not n.geometry_pub.messages[-1].valid  # Measurement is not a contact plan.


@pytest.mark.parametrize('order', list(itertools.permutations(('object', 'mask', 'status'))))
def test_paired_false_can_only_quarantine_registered_identity(station, order):
    c = station; contract = arm(c)
    ns = contract['source_ns'] + 200_000_000
    clock(c, ns+10_000_000)
    row = row_at(c, ns)
    row['object'].detected = False
    row['mask'].data = bytes(len(row['mask'].data))
    row['status'].update(state='processing_deadline_pending', station_compute_wait=contract)
    deliver(c.node, row, order)
    assert c.node._station_compute_pending
    assert c.node._current_stream_target_identity() == c.identity


@pytest.mark.parametrize('bad', ['missing_parent', 'changed_parent', 'anchor', 'empty_mask', 'low_confidence', 'mask_hash', 'mask_shape', 'mask_frame'])
def test_recovery_cannot_bypass_parent_or_original_measurement_gates(station, bad):
    c = station; contract = pending(c)
    ns = c.node._ros_source_clock_ns() + 100_000_000
    clock(c, ns+10_000_000)
    row = row_at(c, ns, recovery=contract)
    if bad == 'missing_parent': row['status'].pop('station_compute_recovery')
    if bad == 'changed_parent': row['status']['station_compute_recovery']['source_ns'] += 1
    if bad == 'anchor': row['status']['anchor_position_base'][0] += .01
    if bad == 'empty_mask': row['mask'].data = bytes(len(row['mask'].data))
    if bad == 'low_confidence': row['object'].confidence = .1
    if bad == 'mask_hash': row['status']['station_compute_mask']['sha256'] = 'f'*64
    if bad == 'mask_shape': row['status']['station_compute_mask']['shape'][0] += 1
    if bad == 'mask_frame': row['status']['station_compute_mask']['frame_id'] = 'foreign'
    deliver(c.node, row)
    if bad == 'missing_parent':
        assert c.node._station_compute_pending and not c.node._station_compute_retired
        assert c.node._station_compute_reference['contract'] == contract
        return
    assert c.node._station_compute_retired
    assert c.node._current_stream_target_identity().epoch == c.identity.epoch + 1


def test_late_registration_and_repeated_source_cannot_extend_deadline(station):
    c = station
    contract = c.ready['status']['station_compute_wait']
    clock(c, contract['geometry_deadline_ns'] + 1_000_000)
    deliver(c.node, c.ready)
    assert c.node._station_compute_reference is None


def test_same_ready_source_does_not_rearm_local_time(station):
    c = station; arm(c)
    original = deepcopy(c.node._station_compute_reference)
    c.clock[0] += .1; c.node.latest_joint_state.header.stamp = at(c.node._ros_source_clock_ns())
    deliver(c.node, c.ready)
    assert c.node._station_compute_reference['identity_monotonic'] == original['identity_monotonic']


@pytest.mark.parametrize('bad', ['deadline', 'epoch', 'manual', 'selection', 'untyped_false', 'true_loss'])
def test_registered_identity_retires_on_real_loss_or_fixed_deadline(station, bad):
    c = station; contract = arm(c)
    if bad == 'deadline': clock(c, contract['identity_deadline_ns']+1_000_000)
    elif bad == 'epoch': c.node._measurement_remote_epoch_ns += 1
    elif bad == 'manual': c.params['/gui/joint_direct_mode'] = True
    elif bad == 'selection': c.params[SELECTION_PARAM] = {}
    else:
        ns = contract['source_ns']+100_000_000
        clock(c, ns+10_000_000)
        row = row_at(c, ns)
        row['object'].detected = False
        row['status']['state'] = 'unavailable'
        if bad == 'true_loss': row['status']['target_lost'] = True
        deliver(c.node, row)
    c.node._station_compute_poll()
    assert c.node._station_compute_retired


def test_joint_excursion_and_return_cannot_restore_station(station):
    c = station; parent = arm(c)
    feedback(c, c.node._ros_source_clock_ns(), [0., .005, 0., 0., 0., 0.])
    feedback(c, c.node._ros_source_clock_ns()+1, [0.]*6)
    # The one explicitly bound new-measurement attempt retains identity only.
    # A returned encoder pose cannot resurrect the retired metric source.
    assert c.node._station_requalification_hold is not None
    assert c.node._station_compute_reference is None
    assert c.node._station_compute_pending
    assert c.node._station_compute_cutoff_ns >= parent['source_ns']
    assert not c.node.geometry_pub.messages[-1].valid


@pytest.mark.parametrize('stage', ['MOVE_PREGRASP', 'APPROACH', 'CLOSE'])
def test_contact_or_motion_does_not_enter_processing_exception(station, stage):
    c = station; arm(c)
    c.node._station_compute_task_state = stage
    c.node._station_compute_poll()
    assert c.node._station_compute_reference is None
    assert not c.node._station_compute_retired


def test_supplement_planning_at_same_station_keeps_scope(station):
    c = station; arm(c)
    c.node._observe_return_phase = 'observation_prefix'
    c.node._observe_return_pending_supplement = dict(session_id=c.binding['session_id'],
        target_epoch=c.binding['target_epoch'], target_track_id=c.binding['target_track_id'],
        previous_view_index=1, view_index=2, deadline_ns=c.session_deadline,
        contact_or_motion_authorized=False)
    assert c.node._station_compute_scope()
    c.node._station_compute_context(c.node._station_compute_reference['contract'])
    assert c.node._station_compute_reference is not None


def test_partial_worker_publication_is_canceled_after_independent_expiry(station):
    c = station; contract = arm(c)
    ticket = NS(payload=(NS(stamp_ns=contract['source_ns'], sample_stamp_ns=(contract['source_ns'],)),))
    clock(c, contract['geometry_deadline_ns']+1_000_000)
    with c.node._stream_condition, c.node._geometry_state_guard():
        assert c.node._stream_ticket_cancellation_code_locked(ticket) == 'TARGET_PROCESSING_PENDING'
    plan = deepcopy(c.node._observe_return_prefix)
    plan.header.stamp = at(contract['source_ns']); plan.valid = True
    with pytest.raises(base.StreamResultCancelled):
        c.node._publish_preview_plan(plan, base.PoseArray())


def test_expired_generation_cannot_be_revived_by_fresh_ready(station):
    c = station; contract = arm(c)
    clock(c, contract['identity_deadline_ns']+1_000_000)
    c.node._station_compute_poll()
    ns = c.node._ros_source_clock_ns()+10_000_000
    clock(c, ns+1_000_000)
    deliver(c.node, row_at(c, ns, recovery=contract))
    assert c.node._station_compute_retired
    assert c.node._station_compute_reference is None
    assert not c.node.latest_object.detected


def test_same_ros_clock_cannot_extend_monotonic_deadline(station, monkeypatch):
    c = station; contract = arm(c)
    frozen = c.node._station_compute_reference['identity_monotonic']
    monkeypatch.setattr(implementation.time, 'monotonic', lambda: frozen+.001)
    c.node._station_compute_poll()
    assert c.node._station_compute_retired


def test_pending_status_alone_with_old_source_withdraws_geometry(station):
    c = station; contract = arm(c)
    row = deepcopy(c.ready)
    row['status'].update(state='processing_deadline_pending', station_compute_wait=contract)
    deliver(c.node, row, ('status',))
    assert c.node._station_compute_pending
    assert not c.node.geometry_pub.messages[-1].valid
    assert c.node._current_stream_target_identity() == c.identity


def test_original_joint_name_case_normalization(station):
    c = station
    c.node.latest_joint_state.name = ['Joint%d' % k for k in range(1, 7)]
    feedback(c, c.node._ros_source_clock_ns())
    arm(c)
    assert not c.node._station_compute_retired


def test_missed_intermediate_ready_is_only_historical_parent_bridge(station):
    c = station; old = arm(c)
    missed_ns = old['source_ns']+300_000_000
    clock(c, missed_ns+10_000_000)
    missed = row_at(c, missed_ns)['status']['station_compute_wait']
    # No object/mask/status from B was delivered. A remains the only pin.
    clock(c, old['geometry_deadline_ns']+100_000_000)
    c.node._station_compute_poll()
    old_mono = c.node._station_compute_reference['identity_monotonic']
    pending_row = deepcopy(c.ready)
    pending_row['status'].update(state='processing_deadline_pending',
        station_compute_wait=missed, station_compute_recovery_chain=[old, missed])
    deliver(c.node, pending_row, ('status',))
    assert c.node._station_compute_pending
    assert c.node._station_compute_reference['contract'] == old
    assert c.node._station_compute_reference['identity_monotonic'] == old_mono
    ns = c.node._ros_source_clock_ns()+100_000_000
    clock(c, ns+10_000_000)
    ready = row_at(c, ns, recovery=missed)
    ready['status']['station_compute_recovery_chain'] = [old, missed]
    deliver(c.node, ready)
    assert not c.node._station_compute_pending and not c.node._station_compute_retired
    assert c.node.latest_object.header.stamp.to_nsec() == ns
    assert c.node._station_compute_cutoff_ns == ns-1


@pytest.mark.parametrize('part', ['mask', 'object', 'contract'])
def test_same_source_cannot_change_joined_content(station, part):
    c = station; arm(c)
    row = deepcopy(c.ready)
    if part == 'mask':
        data = bytearray(row['mask'].data); data[0] = 255 if data[0] == 0 else 0
        row['mask'].data = bytes(data)
        deliver(c.node, row, ('mask',))
    elif part == 'object':
        row['object'].confidence -= .01
        deliver(c.node, row, ('object',))
    else:
        row['status']['station_compute_wait']['identity_deadline_ns'] += 1
        deliver(c.node, row, ('status',))
    assert c.node._station_compute_retired


def test_heartbeat_cannot_substitute_for_accepted_source_feedback(station):
    c = station
    c.node._station_compute_feedback.clear()
    c.node.joint_cb(c.node.latest_joint_state)
    deliver(c.node, c.ready)
    assert c.node._station_compute_reference is None


def test_missing_proof_ready_expiring_does_not_retire_or_renew_old_wait(station):
    c = station; old = pending(c)
    original_mono = c.node._station_compute_reference['identity_monotonic']
    ns = c.node._ros_source_clock_ns()+100_000_000
    clock(c, ns+10_000_000)
    deliver(c.node, row_at(c, ns))
    assert c.node._station_compute_pending
    clock(c, ns+900_000_000)
    c.node._station_compute_poll()
    assert c.node._station_compute_pending and not c.node._station_compute_retired
    assert c.node._station_compute_reference['contract'] == old
    assert c.node._station_compute_reference['identity_monotonic'] == original_mono
    clock(c, old['identity_deadline_ns']+1_000_000)
    c.node._station_compute_poll()
    assert c.node._station_compute_retired


def test_fresh_normal_ready_needs_no_extra_recovery_proof(station):
    c = station; old = arm(c)
    ns = old['source_ns']+200_000_000
    clock(c, ns+10_000_000)
    deliver(c.node, row_at(c, ns))
    assert c.node.latest_object.header.stamp.to_nsec() == ns
    assert not c.node._station_compute_retired and not c.node._station_compute_pending
    assert c.node._station_compute_reference['contract']['source_ns'] == ns


def test_stale_complete_recovery_is_discarded_without_shortening_old_deadline(station):
    c = station; old = pending(c)
    ns = c.node._ros_source_clock_ns()+100_000_000
    row = row_at(c, ns, recovery=old)
    clock(c, ns+810_000_000)
    deliver(c.node, row)
    assert c.node._station_compute_pending and not c.node._station_compute_retired
    assert c.node._station_compute_reference['contract'] == old


@pytest.mark.parametrize('order', list(itertools.permutations(('object', 'mask', 'status'))))
def test_late_canceled_pending_cannot_withdraw_completed_new_recovery(station, order):
    c = station; old = pending(c)
    old_row = deepcopy(c.ready)
    old_row['object'].detected = False
    old_row['mask'].data = bytes(len(old_row['mask'].data))
    old_row['status'].update(state='processing_deadline_pending', station_compute_wait=old)
    ns = c.node._ros_source_clock_ns()+100_000_000
    clock(c, ns+10_000_000)
    deliver(c.node, row_at(c, ns, recovery=old))
    current = deepcopy(c.node._station_compute_reference)
    deliver(c.node, old_row, order)
    assert not c.node._station_compute_pending and not c.node._station_compute_retired
    assert c.node.latest_object.detected and c.node.latest_object.header.stamp.to_nsec() == ns
    assert c.node._station_compute_reference['contract'] == current['contract']
    assert c.node._station_compute_reference['identity_monotonic'] == current['identity_monotonic']


def test_delayed_true_loss_is_not_hidden_by_canceled_source_cutoff(station):
    c = station; old = pending(c)
    ns = c.node._ros_source_clock_ns()+100_000_000
    clock(c, ns+10_000_000)
    deliver(c.node, row_at(c, ns, recovery=old))
    terminal = deepcopy(c.ready)
    terminal['status'].update(state='unavailable', target_lost=True)
    deliver(c.node, terminal, ('status',))
    assert c.node._station_compute_retired


def test_actual_producer_three_frame_arrival_then_slow_discard_and_fresh_recovery(context, monkeypatch):
    """Real image algorithms/producer statuses/base remote gates, simulated transport/clock."""
    from unittest.mock import Mock
    from alicia_flexible_grasp_supervisor.msg import NearFieldPlanningPhase, GraspState
    from test_observation_measurement_remote import begin, qualified_ack
    from test_observation_measurement_unknown import send, ack, arrival as arrive, next_frame
    from test_unknown_pending_identity import publication
    from test_locked_rgb_hold import custom_frame, latest_status, module as producer_module
    from test_station_compute_wait_producer import overrun
    c = context; n = c.node; p = c.producer
    p._observation_measurement_ack_pub = Mock()
    started = begin(c); send(p, started)
    arrived = arrive(p, c.clock, c.joint, started)
    assert request(n, arrived)['accepted']
    for _ in range(2):
        next_frame(p, c.clock, c.joint)
        assert not qualified_ack(c)
    next_frame(p, c.clock, c.joint)
    row = publication(p); row['ack'] = ack(p)
    for key in ('status', 'ack', 'object', 'mask'):
        deliver_one(c, row, key)
    assert qualified_ack(c)
    c.clock[0] += .01
    done = dict(arrived, state='qualified', request_id='completed-real',
        stamp_ns=n._ros_source_clock_ns(), source_stamps_ns=row['ack']['source_stamps_ns'])
    send(p, done); assert ack(p)['accepted']; assert request(n, done)['accepted']
    n._observe_return_phase = 'observing_contact'; n._observe_return_paused = False
    n._station_compute_task_state = 'PLAN_PREGRASP'; n.robot_execution_active = True
    phase = NearFieldPlanningPhase()
    phase.active = True; phase.phase_id = 7; phase.planning_stage = 'contact_replan'
    phase.header.stamp = producer_module.rospy.Time.now(); phase.header.frame_id = 'base_link'
    phase.deadline = at(n._ros_source_clock_ns()+30_000_000_000)
    phase.reference_source_stamp = at(row['ack']['source_stamps_ns'][-1])
    phase.reference_target_track_id = c.identity.track_id; phase.reference_center_valid = True
    n._station_compute_phase = phase
    p._station_compute_phase_cb(phase)
    p._station_compute_state_cb(GraspState(state='PLAN_PREGRASP', active=True))
    n.latest_joint_state = JointState(name=['Joint%d' % k for k in range(1, 7)],
        position=p._locked_rgb_hold_reference['observation_arrival_retry']['arrival_q'].tolist())
    for ns, q in p._rgb_interest_feedback:
        feedback(c, ns, q)
    c.clock[0] += .3
    custom_frame(p, c.clock, c.joint, target_depth=.35)
    row = publication(p)
    assert row['status']['state'] == 'ready' and 'station_compute_wait' in row['status']
    n.latest_joint_state.header.stamp = at(n._ros_source_clock_ns())
    for ns, q in p._rgb_interest_feedback:
        feedback(c, ns, q)
    deliver(n, row)
    assert n._station_compute_reference is not None
    original_contract = deepcopy(n._station_compute_reference['contract'])
    original_last = p.tracker.last
    overrun(monkeypatch, p, c.clock, c.joint)
    row = publication(p)
    assert row['status']['state'] == 'processing_deadline_pending'
    assert p.tracker.last is original_last
    n.latest_joint_state.header.stamp = at(n._ros_source_clock_ns())
    for ns, q in p._rgb_interest_feedback:
        feedback(c, ns, q)
    deliver(n, row)
    assert n._station_compute_pending and n._current_stream_target_identity() == c.identity
    c.clock[0] += .05
    custom_frame(p, c.clock, c.joint, target_depth=.35)
    row = publication(p)
    assert row['status']['state'] == 'ready', latest_status(p)
    assert row['status']['station_compute_recovery'] == original_contract
    n.latest_joint_state.header.stamp = at(n._ros_source_clock_ns())
    for ns, q in p._rgb_interest_feedback:
        feedback(c, ns, q)
    deliver(n, row, ('object', 'status', 'mask'))
    assert not n._station_compute_pending and not n._station_compute_retired
    assert n._current_stream_target_identity() == c.identity
    assert n.latest_object.header.stamp.to_nsec() == p.last_good_source_stamp_ns
