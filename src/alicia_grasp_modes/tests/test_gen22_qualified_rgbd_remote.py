"""Offline GEN21 diagnosis: actual callbacks and RGB-D cache, no ROS graph.

Only geometry estimation after successful exact-sample admission is replaced;
image delivery, measurement joins, MRO, source keys and cache waits are real.
"""
from copy import deepcopy
import threading
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest
from sensor_msgs.msg import JointState
from alicia_flexible_grasp.vision import rgbd_snapshot

from test_observation_measurement_remote import (
    context, arrival, fresh, deliver_one, qualified_ack, request, at)
from test_remote_modes import base, direct_observation

REAL_MONOTONIC = time.monotonic


@pytest.fixture(autouse=True)
def real_wait_clock(monkeypatch):
    # The inherited producer fixture freezes the shared time.monotonic module.
    # Preserve its explicit simulated source/cache clock, but make the actual
    # Condition timeout progress with the real wall clock.
    monkeypatch.setattr(rgbd_snapshot, 'time', NS(monotonic=REAL_MONOTONIC,
        time=time.time, time_ns=time.time_ns))


def images(c, source):
    mask = c.node.bridge.imgmsg_to_cv2(c.ready['mask'], desired_encoding='mono8')
    color = c.node.bridge.cv2_to_imgmsg(
        np.full(mask.shape + (3,), 40, dtype=np.uint8), encoding='bgr8')
    depth = c.node.bridge.cv2_to_imgmsg(
        np.full(mask.shape, 350, dtype=np.uint16), encoding='16UC1')
    for msg in (color, depth):
        msg.header.stamp = at(source)
        msg.header.frame_id = c.ready['mask'].header.frame_id
    return color, depth


def complete_measurement(c, *, with_images):
    c.node.planning_snapshot_max_age_sec = .35
    c.node.planning_snapshot_max_inference_latency_sec = 5.
    c.node.frames.configure_retention(.35, 5.)
    c.node.joint_cb(JointState(name=['joint%d' % i for i in range(1, 7)], position=[0.]*6))
    req = arrival(c)
    row, stamps = fresh(c, req)
    color, depth = images(c, stamps[-1])
    if with_images:
        c.node.color_cb(color)
        c.node.depth_cb(depth)
    for key in ('object', 'mask', 'status', 'ack'):
        deliver_one(c, row, key)
    assert qualified_ack(c)[-1]['source_stamps_ns'] == stamps
    assert c.node.latest_object.header.stamp.to_nsec() == stamps[-1]
    c.clock[0] += .01
    done = dict(req, state='qualified', request_id='done',
                stamp_ns=c.node._ros_source_clock_ns(), source_stamps_ns=stamps)
    assert request(c.node, done)['accepted']
    return stamps[-1], color, depth


def phase(c, source):
    # Recorded observation -> near-field delay: source age about 0.355 sec;
    # object/mask completion is later, so the .35-sec completion gate passes.
    c.clock[0] = (source + 355_304_242) / 1e9
    now = c.node._ros_source_clock_ns()
    c.node._observe_return_session['stamp_ns'] = now - 10_000_000
    c.node.near_field_direct_timeout_sec = 60.
    return NS(active=True, phase_id=1, planning_stage='contact_replan',
        header=NS(stamp=at(now), frame_id='base_link'), deadline=at(now+30_000_000_000),
        reference_center_valid=True, reference_source_stamp=at(source),
        reference_target_track_id=c.identity.track_id,
        reference_center_base=NS(x=.1, y=.2, z=.3))


def capture_after_exact_sample(c, monkeypatch, success=False):
    captured = []
    c.node.camera_depth_scale = .001
    def geometry(snapshot, stamp):
        captured.append(snapshot)
        return NS(ok=success, failure_code='DIAGNOSTIC_AFTER_EXACT_SAMPLE',
                  failure_reason='geometry estimation deliberately outside this cache test'), None, np.eye(4)
    monkeypatch.setattr(c.node, '_prepare_snapshot_geometry', geometry)
    if success:
        monkeypatch.setattr(base, 'observation_from_snapshot',
            lambda snapshot, *_: direct_observation(c.node, snapshot.stamp_ns))
    return captured


def test_real_join_and_qualified_release_preserve_exact_frame_in_buffer(context, monkeypatch):
    c = context
    source, _, _ = complete_measurement(c, with_images=True)
    msg = phase(c, source)
    entry = c.node.frames._entries[source]
    assert all(k in entry for k in ('color_bgr', 'depth_raw', 'object_mask', 'object_msg'))
    assert entry['target_identity'] == c.identity
    assert entry['object_msg'].detected
    captured = capture_after_exact_sample(c, monkeypatch)
    assert c.node._resolve_reached_view_observation(msg) is None
    assert len(captured) == 1, 'None due to geometry stub only, never missing RGB-D'
    assert captured[0].stamp_ns == source
    assert captured[0].original_sample.stamp_ns == source


def test_original_ack_can_confirm_without_rgbd_then_one_shot_phase_stays_observing(context, monkeypatch):
    c = context
    source, _, _ = complete_measurement(c, with_images=False)
    entry = c.node.frames._entries[source]
    assert 'object_msg' in entry and 'object_mask' in entry
    assert 'color_bgr' not in entry and 'depth_raw' not in entry
    msg = phase(c, source)
    captured = capture_after_exact_sample(c, monkeypatch)
    started = REAL_MONOTONIC()
    c.node.near_field_state_cb(msg)
    assert REAL_MONOTONIC() - started >= .49
    assert captured == []
    assert c.node._observe_return_phase == 'observing'
    assert c.node._observe_return_paused
    assert c.node._poll_stream_snapshot() is False


def test_camera_callbacks_can_complete_during_real_nearfield_wait_without_lock_deadlock(context, monkeypatch):
    c = context
    source, color, depth = complete_measurement(c, with_images=False)
    msg = phase(c, source)
    captured = capture_after_exact_sample(c, monkeypatch)
    errors = []
    def deliver_images():
        try:
            time.sleep(.05)
            c.node.color_cb(color)
            c.node.depth_cb(depth)
        except Exception as exc:
            errors.append(exc)
    worker = threading.Thread(target=deliver_images)
    worker.start()
    c.node.near_field_state_cb(msg)
    worker.join(timeout=2.)
    assert not worker.is_alive() and not errors
    assert len(captured) == 1
    assert captured[0].stamp_ns == source


def test_late_rgbd_after_phase_wait_does_not_automatically_retry_phase(context, monkeypatch):
    c = context
    source, color, depth = complete_measurement(c, with_images=False)
    msg = phase(c, source)
    captured = capture_after_exact_sample(c, monkeypatch)
    c.node.near_field_state_cb(msg)
    assert not captured and c.node._observe_return_phase == 'observing'
    # Move the source clock as well: late completion is allowed by the actual
    # existing 5-sec inference / .35-sec completion cache gates. It grants no
    # current contact geometry or motion authority.
    c.clock[0] += .51
    c.node.color_cb(color)
    c.node.depth_cb(depth)
    entry = c.node.frames._entries[source]
    assert all(k in entry for k in ('color_bgr', 'depth_raw', 'object_mask', 'object_msg'))
    assert c.node._poll_stream_snapshot() is False
    assert not captured, 'late camera callbacks never retry the one-shot phase'
    c.node._resolve_reached_view_observation(msg)
    assert len(captured) == 1, 'manual same-phase retry reaches the unchanged sample gate'
    assert captured[0].stamp_ns == source


@pytest.mark.parametrize('ns', [1791970215573863267,1791970215909271240])
def test_actual_epoch_integer_source_keys_survive_without_float_rounding(context, ns):
    c = context
    color, depth = images(c, ns)
    row = deepcopy(c.ready)
    row['object'].header.stamp = at(ns)
    row['mask'].header.stamp = at(ns)
    buffer = base.SynchronizedRgbdBuffer(source_clock_ns=lambda: ns+330_000_000)
    buffer.update_color(c.node.bridge.imgmsg_to_cv2(color), color.header.stamp, color.header.frame_id)
    buffer.update_depth(c.node.bridge.imgmsg_to_cv2(depth), depth.header.stamp, depth.header.frame_id)
    buffer.update_mask(c.node.bridge.imgmsg_to_cv2(row['mask']), row['mask'].header.stamp,
                       row['mask'].header.frame_id)
    buffer.update_object(row['object'], row['object'].header.stamp,
                         target_epoch=c.identity.epoch, target_identity=c.identity)
    assert base._stamp_to_nsec(at(ns)) == ns
    sample = buffer.wait_for_exact_sample(ns, c.identity, timeout_sec=0., require_mask=True,
        max_age_sec=.35, max_inference_latency_sec=5.)
    assert sample is not None and sample.stamp_ns == ns


def pending_reached_retry(c, monkeypatch, before_phase=None):
    source, color, depth = complete_measurement(c, with_images=False)
    msg = phase(c, source)
    n = c.node
    n.robot_execution_active = True
    n._station_compute_task_state = 'PLAN_PREGRASP'
    n.near_field_planning_active = False  # Real observing session transition.
    initial = direct_observation(n, n._observe_return_session['origin_snapshot_stamp_ns'])
    reference = base.surface_view_from_observation(initial)
    surface = base.fused_surface_from_view(reference, voxel_size_m=.0025)
    n._near_field_reference_view = reference
    n._near_field_fused_surface = surface
    n._near_field_surface_phase_id = n._near_field_phase_id
    n._near_field_surface_generation = n._stream_generation
    n._observe_return_saved = (surface, reference)
    n._observe_return_view_uncertainty_m = .001
    captured = capture_after_exact_sample(c, monkeypatch, success=True)
    if before_phase is not None:
        before_phase(msg, source)
    n.near_field_state_cb(msg)
    assert captured == [] and n._observe_return_reached_retry is not None
    c.clock[0] += .51
    return source, color, depth, msg, captured


def test_late_exact_source_retries_real_registration_without_emitting_execution_plan(context, monkeypatch):
    c = context
    source, color, depth, msg, captured = pending_reached_retry(c, monkeypatch)
    n = c.node
    deadline = msg.deadline.to_nsec()
    plan_count = len(n.rich_plan_pub.messages)
    n.color_cb(color)
    n.depth_cb(depth)
    assert n._poll_stream_snapshot() is False
    assert len(captured) == 1 and captured[0].stamp_ns == source
    assert n._latest_registration_evidence.result.ok
    assert n._latest_registration_evidence.stamp_ns == source
    assert n._observe_return_phase == 'observing_contact' and not n._observe_return_paused
    assert n._observe_return_reached_retry is None
    assert n._observe_return_reached_contact_context['deadline_ns'] == deadline
    assert not any(msg.valid for msg in n.rich_plan_pub.messages[plan_count:])
    assert c.identity == n._current_stream_target_identity()


@pytest.mark.parametrize('fault', ['manual', 'selection', 'epoch', 'session', 'phase',
    'qualified_source', 'task_inactive', 'task_failed', 'identity', 'retired', 'deadline',
    'source_clock_rollback', 'monotonic_expired', 'age_configuration',
    'latency_configuration', 'mask_configuration'])
def test_retry_revoked_owner_cannot_register_late_source(context, monkeypatch, fault):
    c = context
    source, color, depth, msg, captured = pending_reached_retry(c, monkeypatch)
    n = c.node
    n.color_cb(color)
    n.depth_cb(depth)
    pending = n._observe_return_reached_retry
    if fault == 'manual': c.params['/gui/joint_direct_mode'] = True
    elif fault == 'selection': c.params['/grasp_mode/selection'] = dict(n._mode_selection, generation=999)
    elif fault == 'epoch': n._measurement_remote_epoch_ns += 1
    elif fault == 'session': n._observe_return_session = deepcopy(n._observe_return_session)
    elif fault == 'phase': n._station_compute_phase = deepcopy(msg); n._station_compute_phase.phase_id += 1
    elif fault == 'qualified_source': n._measurement_remote_request['source_stamps_ns'][-1] += 1
    elif fault == 'task_inactive': n.robot_execution_active = False
    elif fault == 'task_failed': n._station_compute_task_state = 'FAILED'
    elif fault == 'identity': n.target_instance_epoch += 1
    elif fault == 'retired': n._unknown_identity_retired_selection = deepcopy(n._mode_selection)
    elif fault == 'deadline': c.clock[0] = pending['deadline_ns']/1e9 + .001
    elif fault == 'source_clock_rollback': c.clock[0] = (pending['queued_ns']-1_000_000)/1e9
    elif fault == 'age_configuration': n.planning_snapshot_max_age_sec += .1
    elif fault == 'latency_configuration': n.planning_snapshot_max_inference_latency_sec += 1.
    elif fault == 'mask_configuration': monkeypatch.setattr(n, '_active_profile_requires_mask', lambda: not pending['require_mask'])
    else: pending['deadline_monotonic'] = time.monotonic() - .001
    assert n._observe_return_retry_reached_view() is False
    assert n._observe_return_reached_retry is None
    assert not captured and n._observe_return_phase == 'observing'


def test_retry_never_substitutes_a_new_source_or_renews_its_deadline(context, monkeypatch):
    c = context
    source, color, depth, msg, captured = pending_reached_retry(c, monkeypatch)
    n = c.node
    pending = n._observe_return_reached_retry
    bounds = (pending['queued_ns'], pending['deadline_ns'], pending['deadline_monotonic'])
    color.header.stamp = depth.header.stamp = at(source+100_000_000)
    n.color_cb(color)
    n.depth_cb(depth)
    for _ in range(3):
        assert n._observe_return_retry_reached_view()
        assert (pending['queued_ns'],pending['deadline_ns'],pending['deadline_monotonic']) == bounds
    assert not captured and n._observe_return_phase == 'observing'


def test_retry_revalidates_takeover_after_real_sample_resolution(context, monkeypatch):
    c = context
    source, color, depth, msg, captured = pending_reached_retry(c, monkeypatch)
    n = c.node
    n.color_cb(color); n.depth_cb(depth)
    original = n._resolve_reached_view_observation
    def resolve(phase):
        value = original(phase)
        c.params['/gui/joint_direct_mode'] = True
        return value
    monkeypatch.setattr(n, '_resolve_reached_view_observation', resolve)
    assert n._observe_return_retry_reached_view()
    assert len(captured) == 1
    assert n._observe_return_phase == 'observing' and n._observe_return_paused
    assert n._observe_return_reached_retry is None


@pytest.mark.parametrize('expired', ['inference_latency', 'completion_age'])
def test_retry_preserves_original_rgbd_cache_age_gates(context, monkeypatch, expired):
    c = context
    source, color, depth, msg, captured = pending_reached_retry(c, monkeypatch)
    n = c.node
    if expired == 'inference_latency':
        c.clock[0] = (source+5_100_000_000)/1e9
    n.color_cb(color); n.depth_cb(depth)
    if expired == 'completion_age':
        original = n.frames._monotonic_clock
        n.frames._monotonic_clock = lambda: original()+.36
    assert n._observe_return_retry_reached_view()
    assert not captured and n._observe_return_phase == 'observing'


def test_poll_retry_readiness_does_not_hold_camera_control_lock(context, monkeypatch):
    c = context
    source, color, depth, msg, captured = pending_reached_retry(c, monkeypatch)
    n = c.node
    n.color_cb(color); n.depth_cb(depth)
    original = n.frames.wait_for_exact_sample
    checked = []
    def wait(*args, **kwargs):
        if not checked:
            assert kwargs['timeout_sec'] == 0.
            checked.append(True)
            finished = threading.Event()
            def camera_callback():
                n.color_cb(color)
                finished.set()
            worker = threading.Thread(target=camera_callback)
            worker.start()
            assert finished.wait(1.), 'retry probe holds the camera callback lock'
            worker.join(1.)
        return original(*args, **kwargs)
    monkeypatch.setattr(n.frames, 'wait_for_exact_sample', wait)
    assert n._poll_stream_snapshot() is False
    assert checked == [True] and len(captured) == 1
    assert n._observe_return_phase == 'observing_contact'


def station_binding(c, msg, source):
    from alicia_grasp_modes.unknown_pending_identity import reference_binding
    from test_station_compute_wait_remote import feedback
    n = c.node
    qualified = n._measurement_remote_request
    c.phase = msg
    c.session_deadline = qualified['deadline_ns']
    c.binding = dict(selection=deepcopy(n._mode_selection),
        **{k: qualified[k] for k in ('lease_id', 'session_id', 'view_plan_id',
            'view_index', 'target_epoch', 'target_track_id', 'control_epoch_ns')},
        phase_id=msg.phase_id, phase_started_ns=msg.header.stamp.to_nsec(),
        phase_reference_source_ns=source,
        reference_binding=reference_binding(c.ready['status'], n._mode_selection),
        source_id='a'*64, camera_configuration_sha256='b'*64,
        config_sha256='c'*64, station_q_rad=[0.]*6)
    feedback(c, source)


def reached_station_rows(c, monkeypatch):
    from test_station_compute_wait_remote import row_at, clock, feedback
    source, color, depth, msg, captured = pending_reached_retry(c, monkeypatch)
    station_binding(c, msg, source)
    n = c.node
    ready_ns = n._ros_source_clock_ns()+100_000_000
    feedback(c, ready_ns)
    clock(c, ready_ns+10_000_000)
    ready = row_at(c, ready_ns)
    false_ns = ready_ns+200_000_000
    pending = row_at(c, false_ns)
    pending['object'].detected = False
    pending['mask'].data = bytes(len(pending['mask'].data))
    pending['status'].update(state='processing_deadline_pending', detail='processing_deadline_pending',
        station_compute_wait=ready['status']['station_compute_wait'])
    return source, color, depth, msg, captured, ready, pending


@pytest.mark.parametrize('order', [('status','mask','object'), ('object','mask','status')])
def test_station_pending_preserves_original_late_registration_but_blocks_motion(context, monkeypatch, order):
    from test_station_compute_wait_remote import row_at, clock, feedback
    from test_unknown_pending_identity import deliver
    c = context
    source, color, depth, msg, captured, ready, pending = reached_station_rows(c, monkeypatch)
    n = c.node
    count = len(n.rich_plan_pub.messages)
    deliver(n, ready)
    assert n._current_stream_target_identity() == c.identity
    assert n._station_compute_reference is not None
    assert n._observe_return_phase == 'observing' and n._observe_return_paused
    assert n._station_compute_snapshot_policy() is None
    assert not n.near_field_planning_active
    clock(c, pending['object'].header.stamp.to_nsec()+10_000_000)
    deliver(n, pending, order)
    assert n._current_stream_target_identity() == c.identity
    assert n._station_compute_pending
    n.color_cb(color); n.depth_cb(depth)
    assert n._poll_stream_snapshot() is False
    assert len(captured) == 1 and captured[0].stamp_ns == source
    assert n._latest_registration_evidence.result.ok
    assert n._observe_return_phase == 'observing_contact' and not n._observe_return_paused
    assert n._observe_return_reached_retry is None
    assert n._station_compute_pending
    assert n._poll_stream_snapshot() is False
    assert not any(msg.valid for msg in n.rich_plan_pub.messages[count:])
    # Only a newly qualified three-topic READY can release current planning.
    recovered_ns = n._ros_source_clock_ns()+100_000_000
    feedback(c, recovered_ns)
    clock(c, recovered_ns+10_000_000)
    recovered = row_at(c, recovered_ns, recovery=ready['status']['station_compute_wait'])
    deliver(n, recovered)
    assert not n._station_compute_pending
    assert n._station_compute_reference['contract']['source_ns'] == recovered_ns
    assert n._current_stream_target_identity() == c.identity


@pytest.mark.parametrize('fault', ['no_ready', 'missing_mask', 'wrong_mask', 'wrong_binding'])
def test_unregistered_station_pending_false_still_retires(context, monkeypatch, fault):
    from test_station_compute_wait_remote import clock
    from test_unknown_pending_identity import deliver
    c = context
    source, color, depth, msg, captured, ready, pending = reached_station_rows(c, monkeypatch)
    n = c.node
    if fault == 'missing_mask': deliver(n, ready, ('status','object'))
    elif fault == 'wrong_mask':
        ready['status']['station_compute_mask']['sha256'] = 'd'*64
        deliver(n, ready)
    elif fault == 'wrong_binding':
        ready['status']['station_compute_wait']['binding']['phase_id'] += 1
        deliver(n, ready)
    assert n._station_compute_reference is None
    clock(c, pending['object'].header.stamp.to_nsec()+10_000_000)
    deliver(n, pending)
    assert n._current_stream_target_identity() != c.identity
    n.color_cb(color); n.depth_cb(depth)
    assert n._observe_return_retry_reached_view() is False
    assert not captured


def test_duplicate_phase_never_renews_retry_or_registers_twice(context, monkeypatch):
    c = context
    source, color, depth, msg, captured = pending_reached_retry(c, monkeypatch)
    n = c.node
    token = n._observe_return_reached_retry
    mono = token['deadline_monotonic']
    n.near_field_state_cb(deepcopy(msg))
    assert n._observe_return_reached_retry is token
    assert token['deadline_monotonic'] == mono
    n.color_cb(color); n.depth_cb(depth)
    assert n._poll_stream_snapshot() is False
    evidence = n._latest_registration_evidence
    surface = n._near_field_fused_surface
    phase_id = n._near_field_phase_id
    n.near_field_state_cb(deepcopy(msg))
    assert len(captured) == 1
    assert n._latest_registration_evidence is evidence
    assert n._near_field_fused_surface is surface
    assert n._near_field_phase_id == phase_id


def test_queue_monotonic_deadline_starts_before_context_validation(context, monkeypatch):
    from alicia_grasp_modes import observe_return_remote as impl
    c = context
    source, _, _ = complete_measurement(c, with_images=False)
    msg = phase(c, source)
    n = c.node
    n.robot_execution_active = True
    n._station_compute_task_state = 'PLAN_PREGRASP'
    n._station_compute_phase = msg
    mono = [10.]
    monkeypatch.setattr(impl, 'time', NS(monotonic=lambda: mono[0]))
    context_check = n._measurement_remote_context
    def check(request):
        result = context_check(request)
        mono[0] += .25
        return result
    monkeypatch.setattr(n, '_measurement_remote_context', check)
    n._observe_return_queue_reached_retry(msg, n._observe_return_session)
    pending = n._observe_return_reached_retry
    assert pending is not None
    assert pending['deadline_monotonic'] == pytest.approx(
        10. + (pending['deadline_ns']-pending['queued_ns'])*1e-9)


@pytest.mark.parametrize('clock_kind', ['source', 'monotonic'])
def test_context_validation_cannot_cross_fixed_retry_deadline(context, monkeypatch, clock_kind):
    from alicia_grasp_modes import observe_return_remote as impl
    c = context
    source, color, depth, msg, captured = pending_reached_retry(c, monkeypatch)
    n = c.node
    n.color_cb(color); n.depth_cb(depth)
    pending = n._observe_return_reached_retry
    context_check = n._measurement_remote_context
    def check(request):
        result = context_check(request)
        if clock_kind == 'source':
            c.clock[0] = pending['deadline_ns']/1e9+.01
        else:
            monkeypatch.setattr(impl, 'time', NS(monotonic=lambda: pending['deadline_monotonic']+.01))
        return result
    monkeypatch.setattr(n, '_measurement_remote_context', check)
    assert n._observe_return_retry_reached_view() is False
    assert not captured and n._observe_return_phase == 'observing'
    assert n._observe_return_reached_retry is None


@pytest.mark.parametrize('fault', ['manual', 'configuration', 'monotonic_expired'])
@pytest.mark.parametrize('clear_via_poll', [False, True])
def test_duplicate_phase_cannot_bypass_retry_revocation(context, monkeypatch, fault, clear_via_poll):
    c = context
    source, color, depth, msg, captured = pending_reached_retry(c, monkeypatch)
    n = c.node
    n.color_cb(color); n.depth_cb(depth)
    if fault == 'manual': c.params['/gui/joint_direct_mode'] = True
    elif fault == 'configuration': n.planning_snapshot_max_age_sec += 1.
    else: n._observe_return_reached_retry['deadline_monotonic'] = time.monotonic()-.01
    if clear_via_poll:
        assert n._observe_return_retry_reached_view() is False
    n.near_field_state_cb(deepcopy(msg))
    assert not captured and n._observe_return_phase == 'observing'
    assert n._observe_return_reached_retry is None


@pytest.mark.parametrize('changed', ['source', 'header', 'lease'])
def test_duplicate_phase_with_changed_binding_cannot_replace_queued_source(context, monkeypatch, changed):
    c = context
    source, color, depth, msg, captured = pending_reached_retry(c, monkeypatch)
    n = c.node
    n.color_cb(color); n.depth_cb(depth)
    changed_phase = deepcopy(msg)
    if changed == 'source': changed_phase.reference_source_stamp = at(source+1)
    elif changed == 'header': changed_phase.header.stamp = at(msg.header.stamp.to_nsec()+1)
    else: n._measurement_remote_request['lease_id'] += '-changed'
    n.near_field_state_cb(changed_phase)
    assert not captured and n._observe_return_phase == 'observing'
    assert n._observe_return_reached_retry is None


def test_ready_during_initial_rgbd_wait_is_registered_before_first_false_object(context, monkeypatch):
    from test_station_compute_wait_remote import row_at, clock, feedback
    from test_unknown_pending_identity import deliver
    c = context
    n = c.node
    staged = {}
    def before_phase(msg, source):
        station_binding(c, msg, source)
        original = n._resolve_reached_view_observation
        def resolve(phase):
            ready_ns = n._ros_source_clock_ns()+100_000_000
            feedback(c, ready_ns)
            clock(c, ready_ns+10_000_000)
            ready = row_at(c, ready_ns)
            deliver(n, ready)
            assert n._station_compute_reference is None
            staged['ready'] = ready
            return original(phase)
        staged['original'] = original
        monkeypatch.setattr(n, '_resolve_reached_view_observation', resolve)
    source, color, depth, msg, captured = pending_reached_retry(c, monkeypatch, before_phase)
    monkeypatch.setattr(n, '_resolve_reached_view_observation', staged['original'])
    ready = staged['ready']
    assert n._station_compute_reference['contract']['source_ns'] == ready['object'].header.stamp.to_nsec()
    assert n._observe_return_paused and n._station_compute_snapshot_policy() is None
    false_ns = n._ros_source_clock_ns()+10_000_000
    clock(c, false_ns+10_000_000)
    false = row_at(c, false_ns)
    false['object'].detected = False
    false['mask'].data = bytes(len(false['mask'].data))
    false['status'].update(state='processing_deadline_pending', detail='processing_deadline_pending',
        station_compute_wait=ready['status']['station_compute_wait'])
    deliver(n, false, ('object','mask','status'))
    assert n._current_stream_target_identity() == c.identity and n._station_compute_pending
    n.color_cb(color); n.depth_cb(depth)
    assert n._poll_stream_snapshot() is False
    assert len(captured) == 1 and captured[0].stamp_ns == source
    assert n._observe_return_phase == 'observing_contact' and n._station_compute_pending
