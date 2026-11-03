"""A failed seed probe withdraws this frame while all three owners keep one cap.

The three-owner registration fixture is copied unchanged from the frozen
GEN60 test_station_requalification_three_owners.py::owners. Only inert ROS
transport and the numerical seed failure at the producer measurement boundary
are controlled. Real producer exception handling, status fan-in, quarantine,
expiry and recovery run. This is not a replay of a qualified GEN60 full frame.
"""
from copy import deepcopy
from collections import deque
import json
import threading
from types import SimpleNamespace as NS
from unittest.mock import Mock
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from alicia_flexible_grasp_supervisor.msg import NearFieldPlanningPhase, GraspState
from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest
from alicia_grasp_modes.unknown_pending_identity import reference_binding
from test_observation_measurement_remote import context, at
from test_observation_measurement_unknown import send, ack, arrival
from test_unknown_pending_identity import publication, deliver
from test_locked_rgb_hold import custom_frame, module
from test_remote_modes import Publisher, direct_observation, base
from test_task_modes import configured
from test_station_compute_wait_task import recorded
from test_station_compute_wait_task import deliver as task_deliver
import hashlib
from itertools import permutations
import pytest
from alicia_grasp_modes import source_connected_owner as source_owner
from alicia_grasp_modes.station_compute_reacquisition import validate_seed_insufficiency
from test_station_compute_reacquisition import contract
from test_gen61_seed_insufficiency import recorded_seed_inputs, recorded_seed_failure

def owners(monkeypatch):
    c=context.__wrapped__(monkeypatch); p=c.producer; r=c.node
    _,_,plan,_,_=recorded(2)
    plan.plan_id=c.request['view_plan_id'];plan.header.stamp=at(c.request['view_source_stamp_ns'])
    plan.header.frame_id='base_link';plan.diagnostic='FAR_FIELD_OBSERVATION_PLAN'
    plan.model_choice='unknown_tabletop';plan.target_track_id=c.identity.track_id
    plan.object_geometry.target_track_id=c.identity.track_id
    plan.object_geometry.pose_base=deepcopy(c.ready['object'].pose_base.pose)
    r._observe_return_prefix=deepcopy(plan);r._observe_return_origin_prefix=deepcopy(plan)
    c.request['plan_sha256']=committed_plan_digest(plan)
    p._observation_measurement_ack_pub=Mock()
    send(p,c.request);assert ack(p)['accepted']
    r._measurement_remote_request_cb(String(data=json.dumps(c.request)))
    assert json.loads(r._measurement_remote_ack_pub.messages[-1].data)['accepted']
    req=arrival(p,c.clock,c.joint,c.request)
    r._measurement_remote_request_cb(String(data=json.dumps(req)))
    for _ in range(3):
        c.clock[0]+=.2;custom_frame(p,c.clock,c.joint,target_depth=.35)
    row=publication(p);deliver(r,row)
    r._measurement_remote_ack_cb(String(data=json.dumps(ack(p))))
    assert json.loads(r._measurement_remote_ack_pub.messages[-1].data)['qualified']
    c.clock[0]+=.01
    req=dict(req,state='qualified',request_id='initial-qualified',stamp_ns=module.rospy.Time.now().to_nsec(),source_stamps_ns=ack(p)['source_stamps_ns'])
    send(p,req);r._measurement_remote_request_cb(String(data=json.dumps(req)))
    assert ack(p)['accepted']
    phase=NearFieldPlanningPhase(active=True,phase_id=7,planning_stage='contact_replan',reference_center_valid=True,reference_target_track_id=c.identity.track_id)
    phase.header.stamp=module.rospy.Time.now();phase.header.frame_id='base_link'
    phase.reference_source_stamp=at(req['source_stamps_ns'][-1]);phase.deadline=module.rospy.Time.from_sec(c.clock[0]+30.)
    p._station_compute_phase_cb(phase);p._station_compute_state_cb(GraspState(state='PLAN_PREGRASP',active=True))
    r._observe_return_phase='observing_contact';r._observe_return_paused=False
    r._station_compute_task_state='PLAN_PREGRASP';r.robot_execution_active=True
    r._station_compute_phase=deepcopy(phase);r.near_field_planning_active=True
    r.latest_joint_state=JointState(name=['joint%d'%i for i in range(1,7)],position=[0.]*6)
    r._measurement_remote_reference['binding']=reference_binding(row['status'],r._mode_selection)
    with monkeypatch.context() as init:
        t,_=configured.__wrapped__(init)
    t._mode_selection=deepcopy(r._mode_selection);t._mode_bound_selection=deepcopy(r._mode_selection)
    t.active=True;t.stage=2;t._execution_authority_revoked=False
    t._bound_execution_plan=deepcopy(plan);t._bound_execution_plan_id=plan.plan_id
    t._bound_execution_plan_digest=t._execution_plan_digest(plan);t.latest_grasp6d_plan=deepcopy(plan)
    t._observe_return_session=deepcopy(r._observe_return_session);t._observation_measurement_request=deepcopy(req)
    t._published_near_field_phase=deepcopy(phase);t._near_field_phase_id=phase.phase_id
    t._near_field_active=True;t._near_field_phase_deadline_sec=phase.deadline.to_sec()
    t._following_evidence_lock=threading.RLock();t._following_epoch_ns=req['control_epoch_ns']
    t._following_accepted_history=deque(maxlen=128);t._following_sdk_history=deque(maxlen=128)
    oldjoint=c.joint
    def joint(stamp,q=None):
        values=[0.]*6 if q is None else list(q)
        oldjoint(stamp,values)
        msg=JointState();msg.header.stamp=module.rospy.Time.from_sec(stamp);msg.header.frame_id='sdk_measured'
        msg.name=['Joint%d'%i for i in range(1,7)]+['right_finger'];msg.position=values+[.05]
        r._station_compute_accepted_cb(msg);t.accepted_joint_cb(msg)
    c.joint=joint;c.task=t;c.plan=plan;c.phase=phase
    for ns, values in tuple(p._rgb_interest_feedback):
        msg=JointState();msg.header.stamp=at(ns);msg.header.frame_id='sdk_measured'
        msg.name=['Joint%d'%i for i in range(1,7)]+['right_finger'];msg.position=list(values)+[.05]
        r._station_compute_accepted_cb(msg);t.accepted_joint_cb(msg)
    c.clock[0]+=.2;custom_frame(p,c.clock,c.joint,target_depth=.35)
    row=publication(p);deliver(r,row);task_deliver(NS(node=t),row)
    assert p._station_compute_registration is not None,row['status']
    assert r._station_compute_reference is not None,r.geometry_pub.messages[-1]
    assert t._station_task_record is not None,t._unknown_target_loss
    c.binding=deepcopy(row['status']['station_compute_wait']['binding'])
    c.ready=row
    return c


def seed_error(source_ns, shape):
    """Only a control-flow injection: counts are real, frame routing is inert."""
    failure, _ = recorded_seed_failure()
    arguments, _, _ = recorded_seed_inputs()
    small = arguments[0]
    region = np.zeros(shape, np.uint8)
    region[:small.shape[0], :small.shape[1]] = small
    evidence = dict(failure.evidence, source_ns=source_ns,
        stage='raw_only_extra_core', region_index=0, region_shape=list(shape),
        region_mask_sha256=hashlib.sha256(region.tobytes()).hexdigest())
    return source_owner.RawOnlySeedInsufficient(evidence)


def wire(c, row, order=('status', 'object', 'mask')):
    deliver(c.node, row, order)
    task_deliver(NS(node=c.task), row, order)


def fail_frame(c, monkeypatch, *, mutate=None, plain_error=None, path='single'):
    p = c.producer
    reached = []
    def insufficient(source, *args, **kwargs):
        assert p._station_compute_frame['source'] is source
        error = seed_error(source[0].header.stamp.to_nsec(),
                           (source[0].height, source[0].width))
        if mutate is not None:
            mutate(error.evidence)
        reached.append(error)
        raise plain_error if plain_error is not None else error
    with monkeypatch.context() as patch:
        hook = ('_recover_station_current_fragment_owner' if path == 'single'
                else '_recover_station_source_component_measurement')
        patch.setattr(p, hook, insufficient)
        c.clock[0] += .15
        source = custom_frame(p, c.clock, c.joint, target_depth=.35)
    assert len(reached) == 1
    return publication(p), source, reached[0]


def fresh_frame(c):
    c.clock[0] += .15
    custom_frame(c.producer, c.clock, c.joint, target_depth=.35)
    row = publication(c.producer)
    assert row['status']['state'] == 'ready', row['status']
    return row


def assert_nonmetric_wait(c):
    p, r, t = c.producer, c.node, c.task
    assert p.first_loss is None and not p.tracker.lost
    assert r._station_compute_pending and not r._station_compute_retired
    assert t._station_task_record['pending'] and not t._station_task_retired
    assert t._unknown_target_loss is None
    assert not r.latest_object_geometry.valid
    assert not getattr(r.latest_preview_rich_plan, 'valid', False)
    assert t.latest_obj is None


@pytest.mark.parametrize('order', list(permutations(('status', 'object', 'mask'))))
@pytest.mark.parametrize('path', ['single', 'multi'])
def test_failed_frame_is_false_empty_nonmetric_until_same_source_fanin_and_fresh_recovery(monkeypatch, order, path):
    c = owners(monkeypatch)
    p, r, t = c.producer, c.node, c.task
    parent = deepcopy(c.ready['status']['station_compute_wait'])
    original = (p.tracker.anchor, p.tracker.last, p._locked_rgb_hold_reference,
                p.last_good_source_stamp_ns)
    clocks = (p._station_compute_registration['monotonic_deadline'],
              r._station_compute_reference['identity_monotonic'],
              t._station_task_record['identity_mono'])
    before_acks = p._observation_measurement_ack_pub.publish.call_count
    remote_acks = len(r._measurement_remote_ack_pub.messages)
    original_geometry_publish = r.geometry_pub.publish
    def geometry(message):
        original_geometry_publish(message)
        t.target_geometry_cb(deepcopy(message))
    monkeypatch.setattr(r.geometry_pub, 'publish', geometry)
    pending, source, error = fail_frame(c, monkeypatch, path=path)
    status = pending['status']
    ns = source[0].header.stamp.to_nsec()
    assert status['state'] == 'station_measurement_pending'
    assert status['detail'] == 'station_seed_measurement_insufficient'
    assert status['target_lost'] is False and p.first_loss is None
    assert status['station_compute_wait'] == parent
    assert status['station_seed_insufficiency'] == dict(error.evidence,
        version=1, parent_source_ns=parent['source_ns'])
    assert status['source_stamp_ns'] == pending['object'].header.stamp.to_nsec() == pending['mask'].header.stamp.to_nsec() == ns
    assert not pending['object'].detected
    assert not np.any(p.bridge.imgmsg_to_cv2(pending['mask']))
    assert p.tracker.anchor is original[0] and p.tracker.last is original[1]
    assert p._locked_rgb_hold_reference is original[2]
    assert p.last_good_source_stamp_ns == original[3]
    wire(c, pending, order)
    assert_nonmetric_wait(c)
    assert clocks == (p._station_compute_registration['monotonic_deadline'],
        r._station_compute_reference['identity_monotonic'], t._station_task_record['identity_mono'])
    assert p._observation_measurement_ack_pub.publish.call_count == before_acks
    assert len(r._measurement_remote_ack_pub.messages) == remote_acks
    ready = fresh_frame(c)
    assert ready['status']['station_seed_insufficiency_recovery'] == status['station_seed_insufficiency']
    assert ready['status']['station_compute_recovery'] == parent
    assert ready['status']['source_stamp_ns'] > ns
    for key in order[:-1]:
        wire(c, ready, (key,))
        assert_nonmetric_wait(c)
    wire(c, ready, order[-1:])
    assert not r._station_compute_pending and not r._station_compute_retired
    assert not t._station_task_record['pending'] and not t._station_task_retired
    assert t.latest_obj.detected and r.latest_object.detected
    assert t.latest_obj.header.stamp == ready['object'].header.stamp
    assert r._current_stream_target_identity() == c.identity
    assert not getattr(r.latest_preview_rich_plan, 'valid', False)
    assert p._observation_measurement_ack_pub.publish.call_count == before_acks
    assert len(r._measurement_remote_ack_pub.messages) == remote_acks
    # Late duplicates cannot revoke or restore geometry from the rejected frame.
    wire(c, pending, order)
    assert not r._station_compute_pending and not t._station_task_record['pending']
    assert not r._station_compute_retired and not t._station_task_retired
    assert t.latest_obj.header.stamp == ready['object'].header.stamp


@pytest.mark.parametrize('expired_clock', ['ros', 'monotonic'])
def test_repeated_bad_frames_and_no_frame_poll_keep_original_absolute_and_monotonic_caps(monkeypatch, expired_clock):
    c = owners(monkeypatch)
    p, r, t = c.producer, c.node, c.task
    parent = deepcopy(c.ready['status']['station_compute_wait'])
    clocks = (p._station_compute_registration['monotonic_deadline'],
              r._station_compute_reference['identity_monotonic'], t._station_task_record['identity_mono'])
    last_good = p.last_good_source_stamp_ns
    for _ in range(3):
        pending, _, _ = fail_frame(c, monkeypatch)
        wire(c, pending)
        wire(c, pending, ('mask', 'object', 'status'))
        assert_nonmetric_wait(c)
        assert pending['status']['station_compute_wait'] == parent
        assert p.last_good_source_stamp_ns == last_good
        assert clocks == (p._station_compute_registration['monotonic_deadline'],
            r._station_compute_reference['identity_monotonic'], t._station_task_record['identity_mono'])
    if expired_clock == 'ros':
        c.clock[0] = parent['identity_deadline_ns'] / 1e9 + .001
        c.joint(c.clock[0] - .001)
    else:
        monkeypatch.setattr('time.monotonic', lambda: max(clocks) + .001)
    p.latest = None
    p.process(None)
    r._station_compute_poll()
    t._station_task_check()
    assert p.first_loss is not None
    assert r._station_compute_retired and t._station_task_retired
    assert not getattr(r.latest_preview_rich_plan, 'valid', False)


@pytest.mark.parametrize('fault', ['manual', 'epoch', 'selection'])
def test_positive_owner_revocation_while_waiting_cannot_be_hidden_by_new_pending(monkeypatch, fault):
    c = owners(monkeypatch)
    pending, _, _ = fail_frame(c, monkeypatch)
    wire(c, pending)
    assert_nonmetric_wait(c)
    if fault == 'manual':
        c.params['/gui/joint_direct_mode'] = True
    elif fault == 'epoch':
        c.producer._rgb_interest_epoch += 1
        c.node._measurement_remote_epoch_ns += 1
        c.task._following_epoch_ns += 1
    else:
        c.params['/grasp_mode/selection']['generation'] += 1
    c.producer.latest = None
    c.producer.process(None)
    c.node._station_compute_poll()
    c.task._station_task_check()
    assert c.producer.first_loss is not None or c.producer._station_compute_registration is None
    assert c.node._station_compute_retired and c.task._station_task_retired
    assert not getattr(c.node.latest_preview_rich_plan, 'valid', False)


@pytest.mark.parametrize('missing', ['status', 'object', 'mask'])
def test_missing_recovery_topic_cannot_restore_geometry_or_renew_the_wait(monkeypatch, missing):
    c = owners(monkeypatch)
    pending, _, _ = fail_frame(c, monkeypatch)
    wire(c, pending)
    old_remote = c.node._station_compute_reference['identity_monotonic']
    old_task = c.task._station_task_record['identity_mono']
    ready = fresh_frame(c)
    wire(c, ready, tuple(key for key in ('status', 'object', 'mask') if key != missing))
    assert_nonmetric_wait(c)
    assert c.node._station_compute_reference['identity_monotonic'] == old_remote
    assert c.task._station_task_record['identity_mono'] == old_task


def test_second_failed_frame_and_ready_cannot_erase_first_unjoined_false_input(monkeypatch):
    c = owners(monkeypatch)
    first, _, _ = fail_frame(c, monkeypatch)
    wire(c, first, ('object', 'mask'))
    assert not c.node._station_compute_retired and not c.task._station_task_retired
    second, _, _ = fail_frame(c, monkeypatch)
    wire(c, second)
    assert_nonmetric_wait(c)
    ready = fresh_frame(c)
    wire(c, ready)
    # Producer publishes only its latest receipt. That does not discharge
    # the consumers' independently observed earlier false-object obligation.
    assert_nonmetric_wait(c)
    assert c.task.latest_obj is None
    wire(c, first, ('status',))
    fresh = fresh_frame(c)
    wire(c, fresh)
    assert not c.node._station_compute_retired and not c.task._station_task_retired
    assert not c.node._station_compute_pending and not c.task._station_task_record['pending']
    assert c.task.latest_obj.header.stamp == fresh['object'].header.stamp
    assert not getattr(c.node.latest_preview_rich_plan, 'valid', False)


@pytest.mark.parametrize('reason', ['rgb_seed_core_too_small',
    'current_fragment_foreign_raw_source_component_overlap', 'current_fragment_single_pixel_measured_bridge'])
def test_generic_or_contradictory_failure_keeps_original_terminal_loss(monkeypatch, reason):
    c = owners(monkeypatch)
    row, _, _ = fail_frame(c, monkeypatch, plain_error=ValueError(reason))
    assert row['status']['state'] != 'station_measurement_pending'
    assert row['status'].get('station_seed_insufficiency') is None
    assert c.producer.first_loss is not None
    wire(c, row)
    assert c.node._station_compute_retired and c.task._station_task_retired
    assert not getattr(c.node.latest_preview_rich_plan, 'valid', False)


@pytest.mark.parametrize('fault', ['source', 'shape', 'count'])
def test_typed_error_still_requires_actual_current_frame_identity_and_consistent_evidence(monkeypatch, fault):
    c = owners(monkeypatch)
    def change(evidence):
        if fault == 'source': evidence['source_ns'] -= 1
        elif fault == 'shape': evidence['region_shape'][0] += 1
        else: evidence['trusted_core_pixels'] = 25
    row, _, _ = fail_frame(c, monkeypatch, mutate=change)
    assert row['status']['state'] != 'station_measurement_pending'
    assert c.producer.first_loss is not None


def test_typed_shortage_without_an_existing_registration_cannot_start_a_wait(monkeypatch):
    c = owners(monkeypatch)
    def remove_registration(_):
        c.producer._station_compute_registration = None
    row, _, _ = fail_frame(c, monkeypatch, mutate=remove_registration)
    assert row['status']['state'] != 'station_measurement_pending'
    assert row['status'].get('station_seed_insufficiency') is None
    assert c.producer.first_loss is not None


def receipt_for(parent):
    return dict(seed_error(parent['source_ns']+100_000_000, (240, 320)).evidence,
                version=1, parent_source_ns=parent['source_ns'])


@pytest.mark.parametrize('field,value', [
    ('version', True), ('parent_source_ns', 1), ('source_ns', True),
    ('stage', 'native_core'), ('region_index', -1), ('region_shape', [True, 320]),
    ('region_mask_sha256', 'x'*64), ('measured_pixels', 59), ('eroded_pixels', 97),
    ('reliable_eroded_pixels', 39), ('trusted_core_pixels', 25),
    ('minimum_core_pixels', 24), ('upper_surface_band_used', 1), ('unknown', None)])
def test_receipt_schema_cannot_forge_the_narrow_wait_or_measurement_counts(contract, field, value):
    receipt = receipt_for(contract)
    receipt[field] = value
    with pytest.raises(ValueError):
        validate_seed_insufficiency(receipt, contract, contract['source_ns']+200_000_000)


def test_receipt_keeps_parent_deadline_and_recovery_must_be_a_newer_source(contract):
    receipt = receipt_for(contract)
    now = contract['source_ns']+200_000_000
    assert validate_seed_insufficiency(receipt, contract, now) == receipt
    for recovery in (receipt['source_ns']-1, receipt['source_ns'], now+1):
        with pytest.raises(ValueError):
            validate_seed_insufficiency(receipt, contract, now, recovery_source_ns=recovery)
    assert validate_seed_insufficiency(receipt, contract, now,
        recovery_source_ns=now) == receipt
    with pytest.raises(ValueError):
        validate_seed_insufficiency(receipt, contract, contract['identity_deadline_ns'])


@pytest.mark.parametrize('cause', ['support', 'feedback', 'timeout'])
def test_seed_wait_cannot_be_relabelled_as_a_second_pending_cause(monkeypatch, cause):
    c = owners(monkeypatch)
    pending, source, _ = fail_frame(c, monkeypatch)
    wire(c, pending)
    p = c.producer
    registration = p._station_compute_registration
    receipt = deepcopy(registration['seed_insufficiency'])
    parent = deepcopy(registration['contract'])
    deadline = registration['monotonic_deadline']
    if cause == 'support':
        # Enter precisely the mixed-cause guard; this does not invent another
        # measured frame or claim successful support-gap registration.
        monkeypatch.setattr(p, '_station_compute_frame', {'registration': registration})
        with pytest.raises(ValueError, match='station_seed_pending_mixed_support_gap_unsupported'):
            p._station_compute_publish_support_gap(source, registration['reference'], 'support_plane_missing')
    elif cause == 'feedback':
        with pytest.raises(ValueError, match='station_seed_pending_mixed_feedback_gap_unsupported'):
            p._station_compute_publish_feedback_gap(source, NS(registration=registration, evidence={}))
    else:
        assert p._station_compute_timeout(source, ValueError('source_frame_stale_or_future')) is False
    assert p._station_compute_registration is registration
    assert registration['seed_insufficiency'] == receipt
    assert registration['contract'] == parent
    assert registration['monotonic_deadline'] == deadline
    assert not registration.get('support_gap') and not registration.get('feedback_gap')


@pytest.mark.parametrize('kind', ['status', 'object', 'mask'])
def test_seed_tombstone_after_registration_handoff_is_not_a_new_station_owner(monkeypatch, kind):
    c = owners(monkeypatch)
    pending, source, _ = fail_frame(c, monkeypatch)
    wire(c, pending)
    ready = fresh_frame(c)
    wire(c, ready)
    r = c.node
    assert not r._station_compute_pending and not r._station_compute_retired
    # Execution/contact/requalification may legitimately clear registration
    # while preserving history. Exercise the common pure entry boundary; the
    # existing handshake suites separately establish those transitions.
    r._station_compute_reference = None
    rows = deepcopy(r._station_private_pending_sources)
    ns = source[0].header.stamp.to_nsec()
    assert r._station_private_pending_input(kind, ns, pending[kind]) is False
    assert r._station_private_pending_reconcile() is True
    assert r._station_compute_reference is None
    assert not r._station_compute_retired
    assert r._station_private_pending_sources == rows


def interleaved_seed_wait(monkeypatch, missing):
    """Two actual rejected publications retain the first source's obligation."""
    c = owners(monkeypatch)
    first, _, _ = fail_frame(c, monkeypatch)
    wire(c, first, tuple(key for key in ('status', 'object', 'mask') if key != missing))
    assert_nonmetric_wait(c)
    second, _, _ = fail_frame(c, monkeypatch)
    wire(c, second)
    assert_nonmetric_wait(c)
    return c, first, second


@pytest.mark.parametrize('missing', ['status', 'object', 'mask'])
@pytest.mark.parametrize('order', list(permutations(('status', 'object', 'mask'))))
def test_new_ready_waits_for_each_earlier_source_obligation_without_becoming_discarded(monkeypatch, missing, order):
    c, first, second = interleaved_seed_wait(monkeypatch, missing)
    p, r, t = c.producer, c.node, c.task
    parent = deepcopy(r._station_compute_reference['contract'])
    clocks = (r._station_compute_reference['identity_monotonic'],
              t._station_task_record['identity_mono'])
    producer_acks = p._observation_measurement_ack_pub.publish.call_count
    remote_acks = len(r._measurement_remote_ack_pub.messages)
    ready = fresh_frame(c)
    ns = ready['status']['source_stamp_ns']
    assert ready['status']['station_seed_insufficiency_recovery'] == second['status']['station_seed_insufficiency']
    for key in order:
        wire(c, ready, (key,))
        assert_nonmetric_wait(c)
        assert r._station_compute_reference['contract'] == parent
        assert t._station_task_record['contract'] == parent
        assert clocks == (r._station_compute_reference['identity_monotonic'],
                          t._station_task_record['identity_mono'])
        assert r._station_compute_cutoff_ns < ns
        assert ns not in r._station_compute_ready_history
        assert p._observation_measurement_ack_pub.publish.call_count == producer_acks
        assert len(r._measurement_remote_ack_pub.messages) == remote_acks
    wire(c, first, (missing,))
    # Completing the independently observed earlier source can now admit the
    # still-fresh cached READY only through the original full measured join.
    assert not r._station_compute_retired and not t._station_task_retired
    assert not r._station_compute_pending and not t._station_task_record['pending']
    assert r._station_compute_reference['contract'] == ready['status']['station_compute_wait']
    assert t._station_task_record['contract'] == ready['status']['station_compute_wait']
    assert r.latest_object.header.stamp == ready['object'].header.stamp
    assert t.latest_obj.header.stamp == ready['object'].header.stamp
    assert r._current_stream_target_identity() == c.identity
    assert not getattr(r.latest_preview_rich_plan, 'valid', False)
    wire(c, first, order)
    wire(c, second, order)
    assert not r._station_compute_pending and not t._station_task_record['pending']
    assert not r._station_compute_retired and not t._station_task_retired
    assert t.latest_obj.header.stamp == ready['object'].header.stamp
    assert p._observation_measurement_ack_pub.publish.call_count == producer_acks
    assert len(r._measurement_remote_ack_pub.messages) == remote_acks


@pytest.mark.parametrize('missing', ['status', 'object', 'mask'])
def test_expired_cached_ready_cannot_register_or_renew_the_original_identity(monkeypatch, record_property, missing):
    c, first, _ = interleaved_seed_wait(monkeypatch, missing)
    r, t = c.node, c.task
    original_remote = r._station_compute_reference
    original_task = t._station_task_record
    parent = deepcopy(original_remote['contract'])
    clocks = (original_remote['identity_monotonic'], original_task['identity_mono'])
    ready = fresh_frame(c)
    wire(c, ready)
    assert_nonmetric_wait(c)
    ns = ready['status']['source_stamp_ns']
    expired_at = ready['status']['station_compute_wait']['geometry_deadline_ns'] + 1_000_000
    assert expired_at < parent['identity_deadline_ns']
    remote_retire = Mock(wraps=r._station_compute_retire)
    task_retire = Mock(wraps=t._station_task_retire)
    monkeypatch.setattr(r, '_station_compute_retire', remote_retire)
    monkeypatch.setattr(t, '_station_task_retire', task_retire)
    c.clock[0] = expired_at / 1e9
    c.joint(c.clock[0] - .001)
    wire(c, first, (missing,))
    # Expired measurements may be skipped or conservatively retire identity.
    # Neither outcome is permission to register old geometry or renew a cap.
    assert ns not in r._station_compute_ready_history
    assert not r.latest_object_geometry.valid
    assert not getattr(r.latest_preview_rich_plan, 'valid', False)
    assert t.latest_obj is None
    assert (original_remote['identity_monotonic'], original_task['identity_mono']) == clocks
    if r._station_compute_reference is not None:
        assert r._station_compute_reference is original_remote
        assert r._station_compute_pending and r._station_compute_reference['contract'] == parent
    else:
        assert r._station_compute_retired
    if t._station_task_record is not None:
        assert t._station_task_record is original_task
        assert t._station_task_record['pending'] and t._station_task_record['contract'] == parent
    else:
        assert t._station_task_retired
    record_property('expired_cached_ready_outcome', json.dumps(dict(
        missing=missing, cached_source_ns=ns, expired_at_ns=expired_at,
        original_identity_deadline_ns=parent['identity_deadline_ns'],
        remote_retired=r._station_compute_retired, task_retired=t._station_task_retired,
        remote_reasons=[call.args[0] for call in remote_retire.call_args_list],
        task_reasons=[call.args[0] for call in task_retire.call_args_list],
        cached_ready_admitted=False, original_identity_caps_unchanged=True), sort_keys=True))


def freeze_capacity_probe_monotonic(monkeypatch):
    # A bounded transport burst must test source ownership/capacity, not spend
    # its identity lifetime on numerical fixture work in the test process.
    import time
    frozen = time.monotonic()
    monkeypatch.setattr(time, 'monotonic', lambda: frozen)


@pytest.mark.parametrize('count', [64, 65])
def test_remote_pretyped_false_source_capacity_never_silently_forgets_an_obligation(monkeypatch, record_property, count):
    c = owners(monkeypatch)
    r = c.node
    freeze_capacity_probe_monotonic(monkeypatch)
    assert r._station_compute_reference is not None
    assert not r._station_compute_work
    assert not any('seed_receipt' in row for row in r._station_private_pending_sources.values())
    parent = deepcopy(r._station_compute_reference['contract'])
    deadline = r._station_compute_reference['identity_monotonic']
    first, _, _ = fail_frame(c, monkeypatch)
    first_ns = first['status']['source_stamp_ns']
    retire = Mock(wraps=r._station_compute_retire)
    monkeypatch.setattr(r, '_station_compute_retire', retire)
    for index in range(count):
        # These are source-distinct unavailable transport messages. No typed
        # seed evidence, positive measurement, or READY is fabricated for them.
        row = deepcopy(first)
        ns = first_ns + index * 1_000_000
        assert ns < r._ros_source_clock_ns()
        for key in ('object', 'mask'):
            row[key].header.stamp = at(ns)
        deliver(r, row, ('object', 'mask'))
    first_retained = (first_ns in r._station_compute_objects
                      or first_ns in r._station_compute_masks
                      or first_ns in r._station_private_pending_sources)
    record_property('capacity_observation', json.dumps(dict(
        consumer='remote', kind='unpaired_pretyped_sources', count=count,
        first_source_ns=first_ns, first_obligation_retained=first_retained,
        object_cache_count=len(r._station_compute_objects),
        mask_cache_count=len(r._station_compute_masks),
        private_ledger_count=len(r._station_private_pending_sources),
        retired=r._station_compute_retired,
        retire_reasons=[call.args[0] for call in retire.call_args_list]), sort_keys=True))
    assert not r.latest_object_geometry.valid
    assert not getattr(r.latest_preview_rich_plan, 'valid', False)
    if count == 64:
        assert not r._station_compute_retired
        assert first_retained
        assert r._station_compute_reference['contract'] == parent
        assert r._station_compute_reference['identity_monotonic'] == deadline
    else:
        assert r._station_compute_retired, 'source capacity must fail closed before forgetting unresolved A'


@pytest.mark.parametrize('consumer,capacity', [('remote', 64), ('task', 16)])
def test_completed_seed_tombstone_capacity_retires_without_reviving_authority(monkeypatch, record_property, consumer, capacity):
    c = owners(monkeypatch)
    r, t = c.node, c.task
    freeze_capacity_probe_monotonic(monkeypatch)
    node = r if consumer == 'remote' else t
    method = '_station_compute_retire' if consumer == 'remote' else '_station_task_retire'
    retire = Mock(wraps=getattr(node, method))
    monkeypatch.setattr(node, method, retire)
    completed = []
    for _ in range(capacity):
        failed, _, _ = fail_frame(c, monkeypatch)
        if consumer == 'remote':
            deliver(r, failed)
            assert not r._station_compute_retired
        else:
            task_deliver(NS(node=t), failed)
            assert not t._station_task_retired
        ready = fresh_frame(c)
        if consumer == 'remote':
            deliver(r, ready)
            assert not r._station_compute_retired and not r._station_compute_pending
            assert r._station_compute_reference['contract'] == ready['status']['station_compute_wait']
        else:
            task_deliver(NS(node=t), ready)
            assert not t._station_task_retired and not t._station_task_record['pending']
            assert t._station_task_record['contract'] == ready['status']['station_compute_wait']
        completed.append(failed['status']['source_stamp_ns'])
    ledger = (r._station_private_pending_sources if consumer == 'remote'
              else t._station_task_availability_verified)
    assert len(ledger) == capacity
    assert set(completed) == set(ledger)
    # The existing bounded negative-history contract keeps late-contradiction
    # evidence and ends this identity on overflow. It does not silently evict
    # completed rejection evidence to permit unbounded consecutive recovery.
    overflow, _, _ = fail_frame(c, monkeypatch)
    if consumer == 'remote':
        deliver(r, overflow)
        assert r._station_compute_retired and r._station_compute_reference is None
        assert not r.latest_object_geometry.valid
        assert retire.call_args_list[-1].args[0].endswith('station_private_pending_source_capacity')
    else:
        task_deliver(NS(node=t), overflow)
        assert t._station_task_retired and t._station_task_record is None
        assert retire.call_args_list[-1].args[0] == 'station_availability_verified_capacity_exceeded'
    new = fresh_frame(c)
    if consumer == 'remote':
        deliver(r, new)
        assert r._station_compute_retired and r._station_compute_reference is None
        assert not r.latest_object_geometry.valid
    else:
        task_deliver(NS(node=t), new)
        assert t._station_task_retired and t._station_task_record is None
    assert not getattr(r.latest_preview_rich_plan, 'valid', False)
    record_property('capacity_observation', json.dumps(dict(
        consumer=consumer, kind='completed_seed_tombstones', completed_count=len(completed),
        all_completed_evidence_retained_until_overflow=True, retired=True,
        post_overflow_fresh_frame_revived_authority=False,
        retire_reasons=[call.args[0] for call in retire.call_args_list]), sort_keys=True))


def test_remote_positive_ready_history_can_age_out_without_becoming_a_missing_source(monkeypatch, record_property):
    c = owners(monkeypatch)
    r = c.node
    freeze_capacity_probe_monotonic(monkeypatch)
    first = r._station_compute_reference['contract']['source_ns']
    for _ in range(65):
        ready = fresh_frame(c)
        deliver(r, ready)
        assert not r._station_compute_retired and not r._station_compute_pending
        assert r._station_compute_reference['contract'] == ready['status']['station_compute_wait']
    latest = ready['status']['source_stamp_ns']
    assert first not in r._station_compute_ready_history
    assert latest in r._station_compute_ready_history
    assert len(r._station_compute_ready_history) <= 64
    assert not r._station_private_pending_sources
    assert not getattr(r.latest_preview_rich_plan, 'valid', False)
    record_property('capacity_observation', json.dumps(dict(
        consumer='remote', kind='positive_ready_history', fresh_sources=65,
        history_count=len(r._station_compute_ready_history), first_positive_evicted=True,
        latest_source_ns=latest, retired=False), sort_keys=True))


def expired_seed_recovery_declaration(monkeypatch, *, missing='status', declaration=True):
    c, first, second = interleaved_seed_wait(monkeypatch, missing)
    r, t = c.node, c.task
    parent = deepcopy(r._station_compute_reference['contract'])
    clocks = (r._station_compute_reference['identity_monotonic'],
              t._station_task_record['identity_mono'])
    cached = fresh_frame(c)
    wire(c, cached, ('status', 'object', 'mask') if declaration else ('object', 'mask'))
    assert_nonmetric_wait(c)
    expiry = cached['status']['station_compute_wait']['geometry_deadline_ns'] + 1_000_000
    assert expiry < parent['identity_deadline_ns']
    c.clock[0] = expiry / 1e9
    c.joint(c.clock[0] - .001)
    return c, first, second, cached, parent, clocks


@pytest.mark.parametrize('missing', ['status', 'object', 'mask'])
def test_expired_recovery_declaration_can_bind_a_new_fresh_frame_without_reviving_old_geometry(monkeypatch, missing):
    c, first, _, cached, parent, clocks = expired_seed_recovery_declaration(monkeypatch, missing=missing)
    r, t = c.node, c.task
    old_ns = cached['status']['source_stamp_ns']
    wire(c, first, (missing,))
    assert_nonmetric_wait(c)
    assert old_ns not in r._station_compute_ready_history
    assert r._station_compute_reference['contract'] == t._station_task_record['contract'] == parent
    assert clocks == (r._station_compute_reference['identity_monotonic'],
                      t._station_task_record['identity_mono'])
    new = fresh_frame(c)
    assert new['status'].get('station_seed_insufficiency_recovery') is None
    assert cached['status']['station_compute_wait'] in new['status']['station_compute_recovery_chain']
    for key in ('status', 'object'):
        wire(c, new, (key,))
        assert_nonmetric_wait(c)
        assert clocks == (r._station_compute_reference['identity_monotonic'],
                          t._station_task_record['identity_mono'])
    wire(c, new, ('mask',))
    assert not r._station_compute_retired and not t._station_task_retired
    assert not r._station_compute_pending and not t._station_task_record['pending']
    assert r._station_compute_reference['contract'] == new['status']['station_compute_wait']
    assert t._station_task_record['contract'] == new['status']['station_compute_wait']
    assert r.latest_object.header.stamp == new['object'].header.stamp
    assert t.latest_obj.header.stamp == new['object'].header.stamp
    assert old_ns not in r._station_compute_ready_history
    assert r._current_stream_target_identity() == c.identity
    assert not getattr(r.latest_preview_rich_plan, 'valid', False)


@pytest.mark.parametrize('fault', ['no_cached_declaration', 'changed_cached_declaration',
    'cached_parent_binding_changed', 'cached_contract_deadline_changed',
    'new_chain_omits_cached_contract', 'new_chain_changes_cached_binding',
    'newer_rejection_not_covered', 'original_ros_deadline', 'original_monotonic_deadline'])
def test_expired_recovery_declaration_cannot_bypass_cause_chain_or_original_identity(monkeypatch, record_property, fault):
    c, first, _, cached, parent, clocks = expired_seed_recovery_declaration(
        monkeypatch, declaration=fault != 'no_cached_declaration')
    r, t = c.node, c.task
    old_ns = cached['status']['source_stamp_ns']
    producer_acks = c.producer._observation_measurement_ack_pub.publish.call_count
    remote_acks = len(r._measurement_remote_ack_pub.messages)
    remote_retire = Mock(wraps=r._station_compute_retire)
    task_retire = Mock(wraps=t._station_task_retire)
    monkeypatch.setattr(r, '_station_compute_retire', remote_retire)
    monkeypatch.setattr(t, '_station_task_retire', task_retire)
    if fault in ('changed_cached_declaration', 'cached_parent_binding_changed',
                  'cached_contract_deadline_changed'):
        changed = deepcopy(cached)
        if fault == 'changed_cached_declaration':
            proof = changed['status']['station_seed_insufficiency_recovery']
            old_hash = proof['region_mask_sha256']
            proof['region_mask_sha256'] = ('0' if old_hash[0] != '0' else '1') + old_hash[1:]
        elif fault == 'cached_parent_binding_changed':
            changed['status']['station_compute_recovery']['binding']['target_track_id'] += '-changed'
        else:
            changed['status']['station_compute_wait']['identity_deadline_ns'] += 1
        wire(c, changed, ('status',))
    elif fault == 'newer_rejection_not_covered':
        later_bad, _, _ = fail_frame(c, monkeypatch)
        wire(c, later_bad)
        assert later_bad['status']['source_stamp_ns'] > old_ns
    wire(c, first, ('status',))
    if fault == 'original_ros_deadline':
        c.clock[0] = parent['identity_deadline_ns'] / 1e9 + .001
        c.joint(c.clock[0] - .001)
    elif fault == 'original_monotonic_deadline':
        expired = max(clocks) + .001
        assert expired < c.producer._station_compute_registration['monotonic_deadline']
        monkeypatch.setattr('time.monotonic', lambda: expired)
    new = fresh_frame(c)
    if fault == 'new_chain_omits_cached_contract':
        original_chain = new['status']['station_compute_recovery_chain']
        assert cached['status']['station_compute_wait'] in original_chain
        new['status']['station_compute_recovery_chain'] = [row for row in original_chain
            if row != cached['status']['station_compute_wait']]
    elif fault == 'new_chain_changes_cached_binding':
        chain = new['status']['station_compute_recovery_chain']
        matched = [row for row in chain if row == cached['status']['station_compute_wait']]
        assert len(matched) == 1
        matched[0]['binding']['target_track_id'] += '-changed'
    elif fault == 'newer_rejection_not_covered':
        assert new['status']['station_seed_insufficiency_recovery'] == later_bad['status']['station_seed_insufficiency']
        del new['status']['station_seed_insufficiency_recovery']
    wire(c, new)
    record_property('causal_recovery_rejection', json.dumps(dict(
        fault=fault, cached_source_ns=old_ns, offered_new_source_ns=new['status']['source_stamp_ns'],
        remote_retired=r._station_compute_retired, task_retired=t._station_task_retired,
        task_record_source_ns=(t._station_task_record or {}).get('contract', {}).get('source_ns'),
        task_contract_unchanged=(t._station_task_record or {}).get('contract') == parent,
        task_identity_monotonic=(t._station_task_record or {}).get('identity_mono'),
        original_task_identity_monotonic=clocks[1],
        original_identity_deadline_ns=parent['identity_deadline_ns'],
        task_identity_deadline_ns=(t._station_task_record or {}).get('contract', {}).get('identity_deadline_ns'),
        task_pending=(t._station_task_record or {}).get('pending'),
        task_latest_object_is_none=t.latest_obj is None,
        offered_new_source_registered_locally=new['status']['source_stamp_ns'] in t._station_contact_sources,
        task_preview_valid=bool(getattr(getattr(t, 'latest_grasp6d_preview_plan', None), 'valid', False)),
        remote_reasons=[call.args[0] for call in remote_retire.call_args_list],
        task_reasons=[call.args[0] for call in task_retire.call_args_list]), sort_keys=True))
    assert r._station_compute_retired and r._station_compute_reference is None
    if fault in ('no_cached_declaration', 'new_chain_omits_cached_contract',
                 'new_chain_changes_cached_binding', 'newer_rejection_not_covered'):
        # The task's existing join gate rejects this candidate while retaining
        # the original pending owner and its unchanged absolute/monotonic cap.
        assert not t._station_task_retired
        assert t._station_task_record['pending']
        assert t._station_task_record['contract'] == parent
        assert t._station_task_record['identity_mono'] == clocks[1]
        assert t.latest_obj is None
    else:
        assert t._station_task_retired and t._station_task_record is None
        # Ordinary observation callbacks may cache a raw object after terminal
        # loss; the independent station and execution gates must stay revoked.
        provenance = t._station_contact_provenance(c.plan)
        assert not provenance.ok and provenance.code == 'TARGET_LOST'
        assert provenance.terminal_for_request
        assert t._execution_authority_revoked
        assert t._unknown_target_loss is not None
        assert t.latest_grasp6d_plan is None
    assert old_ns not in t._station_contact_sources
    assert new['status']['source_stamp_ns'] not in t._station_contact_sources
    assert not getattr(getattr(t, 'latest_grasp6d_preview_plan', None), 'valid', False)
    assert old_ns not in r._station_compute_ready_history
    assert new['status']['source_stamp_ns'] not in r._station_compute_ready_history
    assert not r.latest_object_geometry.valid
    assert not getattr(r.latest_preview_rich_plan, 'valid', False)
    assert c.producer._observation_measurement_ack_pub.publish.call_count == producer_acks
    assert len(r._measurement_remote_ack_pub.messages) == remote_acks


def test_remote_full_pretyped_cache_rejects_a_new_older_source_before_immediate_eviction(monkeypatch, record_property):
    c = owners(monkeypatch)
    r = c.node
    freeze_capacity_probe_monotonic(monkeypatch)
    parent_ns = r._station_compute_reference['contract']['source_ns']
    failed, _, _ = fail_frame(c, monkeypatch)
    earlier = failed['status']['source_stamp_ns']
    assert parent_ns < earlier
    for index in range(64):
        row = deepcopy(failed)
        source = earlier + (10 + index) * 1_000_000
        assert source < r._ros_source_clock_ns()
        for key in ('object', 'mask'):
            row[key].header.stamp = at(source)
        deliver(r, row, ('object', 'mask'))
    assert not r._station_compute_retired
    previous_objects = set(r._station_compute_objects)
    previous_masks = set(r._station_compute_masks)
    assert len(previous_objects) == len(previous_masks) == 64
    assert earlier < min(previous_objects) and earlier < min(previous_masks)
    retire = Mock(wraps=r._station_compute_retire)
    monkeypatch.setattr(r, '_station_compute_retire', retire)
    deliver(r, failed, ('object', 'mask'))
    record_property('capacity_observation', json.dumps(dict(
        consumer='remote', kind='older_source_arrives_to_full_pretyped_cache',
        offered_source_ns=earlier, parent_source_ns=parent_ns,
        former_min_source_ns=min(previous_objects), retired=r._station_compute_retired,
        old_object_sources_preserved=set(r._station_compute_objects) == previous_objects,
        old_mask_sources_preserved=set(r._station_compute_masks) == previous_masks,
        retire_reasons=[call.args[0] for call in retire.call_args_list]), sort_keys=True))
    assert r._station_compute_retired and r._station_compute_reference is None
    assert set(r._station_compute_objects) == previous_objects
    assert set(r._station_compute_masks) == previous_masks
    assert retire.call_args_list[-1].args[0] == 'station_unqualified_input_cache_capacity'
    assert not r.latest_object_geometry.valid
    assert not getattr(r.latest_preview_rich_plan, 'valid', False)
