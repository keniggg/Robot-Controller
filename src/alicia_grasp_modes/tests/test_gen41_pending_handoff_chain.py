"""A successful precheck must survive a bounded, non-authorizing station wait.

The unchanged GEN35 task main loop and GEN39 exact reached-source adapter run
real source qualification, fusion, candidate generation and all geometry
consumers. SDK/FK/IK, MoveIt/MuJoCo and actuators are explicit deterministic
simulators. This does not replay a recorded successful robot curve or prove a
physical grasp. No historical failed source is made qualified.
"""
from copy import deepcopy
import json
from types import SimpleNamespace as NS

import numpy as np
import pytest

import test_gen35_minimal_chain_replay as chain
from test_gen38_cuboid_chain import cuboid_scene
from test_gen39_registration_chain import _install_real_reached_source_adapter


_GAP_SPAN_NS = 280_072_042
_RECOVERY_DELAY_NS = 521_733_523


class _InjectionFixtureFailure(BaseException):
    """A failed injector is never swallowed as a rejected candidate."""


class _FirstHandoffFailure(BaseException):
    """Stop the offline replay at its original first failed handoff only."""
    def __init__(self, context, original):
        self.context, self.original = context, original
        super().__init__(str(original))


@pytest.fixture(autouse=True)
def no_unsimulated_external_boundaries(monkeypatch):
    import rospy
    import socket

    def forbidden(*args, **kwargs):
        raise chain.UnsimulatedExternalCall('GEN41 unsimulated boundary '+repr(args))

    for name in ('connect', 'connect_ex', 'sendto', 'bind', 'listen'):
        monkeypatch.setattr(socket.socket, name, forbidden)
    monkeypatch.setattr(socket, 'create_connection', forbidden)
    for name in ('init_node', 'Publisher', 'Subscriber', 'Service', 'ServiceProxy',
                 'wait_for_service', 'set_param', 'delete_param'):
        monkeypatch.setattr(rospy, name, forbidden)


def _now(c):
    return chain.producer_module.rospy.Time.now().to_nsec()


def _at(ns):
    return chain.producer_module.rospy.Time(ns//10**9, ns % 10**9)


def _sdk(c, ns, q=None):
    """An explicit encoder-provider sample through original accepted callbacks."""
    from sensor_msgs.msg import JointState
    q = [0.]*6 if q is None else list(q)
    msg = JointState()
    msg.header.stamp, msg.header.frame_id = _at(ns), 'sdk_measured'
    msg.name = ['Joint%d' % i for i in range(1, 7)]+['right_finger']
    msg.position = q+[.05]
    c.producer._rgb_interest_joint_cb(deepcopy(msg))
    c.remote.latest_joint_state = deepcopy(msg)
    c.remote._station_compute_accepted_cb(deepcopy(msg))
    previous = c.task.latest_accepted_joint_state
    old_ns = previous.header.stamp.to_nsec()
    before = len(c.task._following_accepted_history)
    # Both this explicit station provider and the unchanged motion simulator
    # share one encoder. Reuse an already emitted identical sample; never
    # manufacture a second encoder measurement or reorder consumer history.
    reused = ns == old_ns
    if reused:
        assert msg == previous, 'two SDK providers disagree at the same source stamp'
    else:
        assert ns > old_ns, 'SDK provider attempted to backfill an older sample'
        c.task.accepted_joint_cb(deepcopy(msg))
        c.task.joint_cb(deepcopy(msg))
    sent = deepcopy(msg)
    sent.header.frame_id = 'sdk_transmitted'
    sent.name, sent.position = sent.name[:6], sent.position[:6]
    if not reused:
        c.task.following_sdk_cb(sent)
    c.gen41_sdk_provider.append(dict(kind='explicit_sample', source_ns=ns,
        positions=list(msg.position), reused_original_identical_sample=reused,
        task_history_count_before=before,
        task_history_count_after=len(c.task._following_accepted_history)))


def _source_with_real_feedback_gap(c):
    """Real producer rejects a fresh source because its SDK span is too wide."""
    p = c.producer
    ns = _now(c)+200_000_000
    lo, hi = ns-24_935_299, ns-24_935_299+_GAP_SPAN_NS
    finish = ns+283_157_825
    c.clock[0] = finish*1e-9
    _sdk(c, lo)
    _sdk(c, hi)
    pair = (p.bridge.cv2_to_imgmsg(c.rgb, encoding='bgr8'),
            p.bridge.cv2_to_imgmsg(c.scene.depth, encoding='16UC1'))
    for message in pair:
        message.header.stamp = _at(ns)
        message.header.frame_id = p.camera_frame
    # These are original analytic pixels, not a generated status/model proof.
    c.sources.append(pair)
    p.latest = pair
    p.process(None)
    row = chain.publication(p)
    status = row['status']
    assert status['state'] == 'processing_deadline_pending', status
    assert status['detail'] == 'station_accepted_feedback_bracket_pending_revalidation', status
    assert status['source_stamp_ns'] == ns
    assert status['target_lost'] is False and not row['object'].detected
    proof = status['station_feedback_gap']
    assert proof['feedback_gap_evidence']['span_ns'] == _GAP_SPAN_NS
    assert proof['feedback_gap_evidence']['maximum_span_ns'] == 200_000_000
    assert all(proof[key] is False for key in ('source_qualified', 'identity_verified',
        'metric_geometry_authority', 'motion_authorized', 'reference_refreshed'))
    assert not np.any(p.bridge.imgmsg_to_cv2(row['mask'], 'mono8'))
    return row


def _deliver(c, row, order):
    """Original remote and task callback routes; ordering is the only transport fake."""
    for key in order:
        chain.deliver(c.remote, row, (key,))
        if key == 'object':
            c.task.obj_cb(deepcopy(row[key]))
        elif key == 'mask':
            c.task._station_compute_task_mask_cb(deepcopy(row[key]))
        else:
            c.task._unknown_support_status_cb(NS(data=json.dumps(row[key])))


def _ready(c, *, deliver_bracket=True):
    # The unchanged motion simulator has already emitted real simulated SDK
    # samples while c.advance walks the delay. Forward those same samples to
    # the remote subscriber, without backfilling the task's encoder history.
    c.feed()
    row = chain.publication(c.producer)
    assert row['status']['state'] == 'ready', row['status']
    assert row['object'].detected
    source = row['object'].header.stamp.to_nsec()
    if deliver_bracket:
        records = tuple(c.task._following_accepted_history)
        stamps = [m.header.stamp.to_nsec() for m in records]
        assert all(a < b for a, b in zip(stamps, stamps[1:])), stamps
        left = [m for m in records if m.header.stamp.to_nsec() <= source][-1]
        right = [m for m in records if m.header.stamp.to_nsec() >= source][0]
        before = len(c.task._following_accepted_history)
        for msg in (left, right):
            c.remote._station_compute_accepted_cb(deepcopy(msg))
        assert len(c.task._following_accepted_history) == before
        c.gen41_sdk_provider.append(dict(kind='existing_encoder_bracket_relay',
            source_ns=source, bracket_ns=[left.header.stamp.to_nsec(), right.header.stamp.to_nsec()],
            positions=[list(left.position), list(right.position)],
            task_history_count_before=before, task_history_count_after=before))
    c.remote.frames.update_color(c.rgb, _at(source), row['object'].pose_camera.header.frame_id)
    c.remote.frames.update_depth(c.scene.depth, _at(source), row['object'].pose_camera.header.frame_id)
    _sdk(c, _now(c))
    from alicia_grasp_modes.rgb_observation_source import bracket_feedback
    records = list(c.remote._station_compute_feedback)
    if deliver_bracket:
        bracket_feedback(records, source, c.producer._rgb_interest_epoch)
    else:
        with pytest.raises(ValueError, match='accepted_feedback_does_not_bracket_source'):
            bracket_feedback(records, source, c.producer._rgb_interest_epoch)
        c.gen41_recovery_sdk_rejection = 'accepted_feedback_does_not_bracket_source'
    return row


def _checkpoint(c, name):
    n = c.remote
    remote_contract = (getattr(n, '_station_compute_reference', None) or {}).get('contract', {})
    task_contract = (getattr(c.task, '_station_task_record', None) or {}).get('contract', {})
    row = dict(name=name, now_ns=_now(c),
        ticket_current=n._stream_ticket_cancellation_code_locked(c.ticket),
        private_certificate=bool(n._station_compute_work_certificate(c.ticket)),
        pending=bool(n._station_compute_pending),
        task_record_source_ns=(getattr(c.task, '_station_task_record', None) or {}).get('contract', {}).get('source_ns'),
        task_record_pending=bool((getattr(c.task, '_station_task_record', None) or {}).get('pending', False)),
        remote_current_source_ns=remote_contract.get('source_ns'),
        remote_geometry_deadline_ns=remote_contract.get('geometry_deadline_ns'),
        remote_identity_deadline_ns=remote_contract.get('identity_deadline_ns'),
        task_geometry_deadline_ns=task_contract.get('geometry_deadline_ns'),
        task_identity_deadline_ns=task_contract.get('identity_deadline_ns'),
        remote_retired=bool(getattr(n, '_station_compute_retired', False)),
        task_retired=bool(getattr(c.task, '_station_task_retired', False)),
        task_authority_revoked=bool(getattr(c.task, '_execution_authority_revoked', False)),
        task_hard_cutoff_ns=getattr(c.task, '_station_contact_hard_cutoff', 0),
        public_preview_valid=bool(getattr(n.latest_preview_rich_plan, 'valid', False)),
        phase_deadline_sec=n._near_field_phase_deadline_sec,
        snapshot_sources=list(c.ticket.payload[0].sample_stamp_ns),
        contact_motion_count=sum(x[0] == 'motion' and x[1] != c.origin.plan_id for x in c.device_calls),
        close_count=sum(x[0] == 'gripper' and x[2] < .05 for x in c.device_calls))
    c.gen41_trace.append(row)
    return row


def _install_injection(monkeypatch, tmp_path, order, *, late_gap=False, fault=None,
                       missing_recovery_sdk=False, idle_before_recovery=False,
                       corrupt_idle_echo=False, late_echo_after_parent_expiry=False):
    original = chain.accepted_same_scene
    captured = []

    def accepted(c, patch, path):
        if not captured:
            captured.append(c)
            c.gen41_trace, c.gen41_rows = [], []
            c.gen41_sdk_provider = []
            c.gen41_geometry_publications = []
            n = c.remote
            original_geometry_publish = n.geometry_pub.publish

            def geometry_to_task(message):
                original_geometry_publish(message)
                c.gen41_geometry_publications.append(dict(
                    valid=bool(message.valid), source_stamp_ns=message.header.stamp.to_nsec(),
                    failure_reason=str(message.failure_reason)))
                # This public handoff is real; do not bypass the task's
                # invalidation cutoff or provenance consumer.
                c.task.target_geometry_cb(deepcopy(message))

            monkeypatch.setattr(n.geometry_pub, 'publish', geometry_to_task)
            original_condition_wait = n._stream_condition.wait

            def deterministic_condition_wait(timeout=None):
                # Match the unchanged task fixture's deterministic clock: a
                # bounded remote wait must advance that same simulated time.
                c.clock[0] += max(.001, float(timeout or .01))
                return original_condition_wait(0.)

            monkeypatch.setattr(n._stream_condition, 'wait', deterministic_condition_wait)
            final_precheck = n._final_contact_sequence_evaluation
            fired = []

            def after_precheck(*args, **kwargs):
                result = final_precheck(*args, **kwargs)
                if fired or not result[0].reachable or not result[0].collision_free:
                    return result
                try:
                    fired.append(True)
                    c.gen41_prechecked_plan_id = args[0].plan_id
                    before = _checkpoint(c, 'original_gateway_precheck_accepted')
                    assert before['private_certificate']
                    phase_deadline = n._near_field_phase_deadline_sec
                    p = c.producer
                    parent = deepcopy(p._station_compute_registration['contract'])
                    parent_mono = p._station_compute_registration['monotonic_deadline']
                    pending = _source_with_real_feedback_gap(c)
                    c.gen41_rows.append(dict(kind='feedback_gap', row=deepcopy(pending)))
                    assert p._station_compute_registration['contract'] == parent
                    assert p._station_compute_registration['monotonic_deadline'] == parent_mono
                    failure_receive = _now(c)
                    if not late_gap:
                        _deliver(c, pending, order)
                        _deliver(c, pending, tuple(reversed(order)))  # Duplicate, still no authority.
                        _checkpoint(c, 'typed_feedback_gap_delivered')
                    elif late_gap == 'partial':
                        _deliver(c, pending, ('object',))
                        _checkpoint(c, 'partial_feedback_gap_delivered')
                    recovery_receive = failure_receive+_RECOVERY_DELAY_NS
                    if idle_before_recovery:
                        # The original producer emits this status-only idle
                        # echo while the rejected SDK source remains unresolved.
                        # Its header stays at the registered READY source;
                        # its discarded stamp and full gap proof stay unchanged.
                        idle_receive = parent['geometry_deadline_ns']+86_251_926
                        assert failure_receive < idle_receive < recovery_receive
                        assert recovery_receive < parent['identity_deadline_ns']
                        c.advance((idle_receive-_now(c))*1e-9)
                        assert abs(_now(c)-idle_receive) <= 2
                        assert p._station_compute_idle_wait()
                        echo = chain.publication(p)
                        status = echo['status']
                        assert status['state'] == 'processing_deadline_pending'
                        assert status['detail'] == 'processing_deadline_pending'
                        assert status['source_stamp_ns'] == parent['source_ns']
                        assert status['discarded_source_stamp_ns'] == pending['status']['source_stamp_ns']
                        assert status['station_feedback_gap'] == pending['status']['station_feedback_gap']
                        assert status['station_compute_wait'] == parent
                        assert status['target_lost'] is False and status.get('first_loss') is None
                        c.gen41_rows.append(dict(kind='feedback_idle_echo', row=deepcopy(echo)))
                        if corrupt_idle_echo:
                            # Only corrupt the transport copy. Both redundant
                            # span fields agree with each other but contradict
                            # the original accepted encoder endpoints.
                            echo = deepcopy(echo)
                            gap = echo['status']['station_feedback_gap']
                            gap['feedback_gap_evidence']['span_ns'] += 1
                            gap['discarded_source_evidence'][0]['feedback_gap_evidence']['span_ns'] += 1
                            c.gen41_rows.append(dict(kind='feedback_idle_echo_wire_conflict', row=deepcopy(echo)))
                        _deliver(c, echo, ('status',))
                        observed = _checkpoint(c, 'feedback_idle_echo_badproof_delivered'
                            if corrupt_idle_echo else 'feedback_idle_echo_delivered')
                        if (corrupt_idle_echo or observed['ticket_current']
                                or not observed['private_certificate']):
                            # Let the original acceptance boundary report its
                            # own first cancellation; do not swallow it inside
                            # candidate search or replace it with a fixture error.
                            return result
                    c.advance((recovery_receive-_now(c))*1e-9)
                    assert abs(_now(c)-failure_receive-_RECOVERY_DELAY_NS) <= 2
                    recovered = _ready(c, deliver_bracket=not missing_recovery_sdk)
                    c.gen41_rows.append(dict(kind='feedback_recovery', row=deepcopy(recovered)))
                    assert recovered['status']['station_feedback_gap_recovery'] == pending['status']['station_feedback_gap']
                    assert recovered['status']['station_compute_recovery'] == parent
                    assert p._station_compute_registration['contract']['phase_deadline_ns'] == parent['phase_deadline_ns']
                    assert p._station_compute_registration['contract']['session_deadline_ns'] == parent['session_deadline_ns']
                    assert p._station_compute_registration['monotonic_deadline'] >= parent_mono
                    _deliver(c, recovered, tuple(reversed(order)))
                    if late_gap is True and not missing_recovery_sdk:
                        # Complete rejected-bracket endpoints travel in this
                        # READY's recovery proof. Its own fresh SDK/geometry
                        # and full three-topic join suffice; no old empty
                        # messages are required to certify a non-metric gap.
                        proof_ready = _checkpoint(c, 'complete_fresh_ready_before_any_old_fragment')
                        assert not proof_ready['pending']
                        assert proof_ready['ticket_current'] == ''
                        assert proof_ready['private_certificate']
                        assert proof_ready['task_record_source_ns'] == recovered['object'].header.stamp.to_nsec()
                        assert not proof_ready['task_record_pending']
                    if late_gap:
                        _deliver(c, pending, order)
                        _deliver(c, pending, order)
                    _checkpoint(c, 'fresh_ready_after_feedback_gap')
                    if missing_recovery_sdk:
                        return result
                    if fault:
                        assert c.gen41_trace[-1]['ticket_current'] == '', 'negative case never recovered its original ticket'
                        assert c.gen41_trace[-1]['private_certificate'], 'negative case starts only after healthy recovery'
                        if fault == 'manual':
                            c.params['/gui/joint_direct_mode'] = True
                        elif fault == 'epoch':
                            n._measurement_remote_epoch_ns += 1
                        elif fault == 'pose':
                            _sdk(c, _now(c)+1, [.02, 0., 0., 0., 0., 0.])
                        elif fault == 'current_false':
                            changed = deepcopy(recovered['object'])
                            changed.header.stamp = _at(_now(c)+1)
                            changed.detected = False
                            n.object_cb(changed)
                            c.task.obj_cb(deepcopy(changed))
                        elif fault == 'same_source_conflict':
                            changed = deepcopy(recovered['object'])
                            changed.pose_base.pose.position.x += .02
                            n.object_cb(changed)
                            c.task.obj_cb(deepcopy(changed))
                        elif fault == 'support':
                            p.tracker.plane_base = p.tracker.plane_base.copy()
                            p.tracker.plane_base[-1] += .02
                            c.advance(.2)
                            c.feed()
                            changed = chain.publication(p)
                            assert changed['status']['state'] != 'ready'
                            _deliver(c, changed, order)
                        else:
                            raise AssertionError(fault)
                        _checkpoint(c, 'true_negative_change')
                        return result
                    # Now reproduce the other actual branch: no new failed frame,
                    # only the enrolled READY geometry age passes its old .8s clock.
                    reg = p._station_compute_registration
                    c.clock[0] = (reg['contract']['geometry_deadline_ns']+86_251_926)*1e-9
                    _sdk(c, _now(c)-10_000_000)
                    assert p._station_compute_idle_wait()
                    idle = chain.publication(p)
                    assert idle['status']['state'] == 'processing_deadline_pending'
                    assert idle['status']['detail'] == 'processing_deadline_pending'
                    assert idle['status']['discarded_source_stamp_ns'] is None
                    assert not idle['status']['target_lost']
                    c.gen41_rows.append(dict(kind='idle_expiry', row=deepcopy(idle)))
                    _deliver(c, idle, ('status',))  # The real idle branch publishes status only.
                    _checkpoint(c, 'producer_idle_expiry_delivered')
                    c.advance(.2)
                    final = _ready(c)
                    c.gen41_rows.append(dict(kind='idle_recovery', row=deepcopy(final)))
                    _deliver(c, final, order)
                    _deliver(c, final, tuple(reversed(order)))
                    _checkpoint(c, 'fresh_ready_after_idle_expiry')
                    if late_echo_after_parent_expiry:
                        assert idle_before_recovery and not corrupt_idle_echo
                        # Continue the same original producer/SDK at the same
                        # station. Every new source joins before the current
                        # geometry expires; no old source or clock is renewed.
                        count = 0
                        max_steps = max(1, (parent['identity_deadline_ns']-
                            final['status']['source_stamp_ns'])//400_000_000+2)
                        while final['status']['source_stamp_ns'] <= parent['identity_deadline_ns']:
                            assert count < max_steps, 'fixture did not cross the original parent deadline'
                            c.advance(.4)
                            final = _ready(c)
                            c.gen41_rows.append(dict(kind='late_parent_fresh_ready', row=deepcopy(final)))
                            _deliver(c, final, order)
                            current = _checkpoint(c, 'fresh_ready_while_parent_ages')
                            count += 1
                            if current['ticket_current'] or not current['private_certificate']:
                                return result  # Preserve the original first cancellation.
                            assert current['remote_current_source_ns'] == final['status']['source_stamp_ns']
                            assert current['task_record_source_ns'] == final['status']['source_stamp_ns']
                            assert not current['pending'] and not current['task_record_pending']
                        current = deepcopy(c.gen41_trace[-1])
                        c.gen41_late_delivery = dict(
                            original_parent_identity_deadline_ns=parent['identity_deadline_ns'],
                            current_before_late_delivery=current,
                            delivered_status_sources=[pending['status']['source_stamp_ns'],
                                echo['status']['source_stamp_ns']])
                        assert _now(c) > parent['identity_deadline_ns']
                        assert _now(c) < current['remote_geometry_deadline_ns']
                        assert _now(c) < current['task_geometry_deadline_ns']
                        for old in (pending, echo, echo, pending):
                            _deliver(c, old, ('status',))
                        late = _checkpoint(c, 'late_gap_and_idle_echo_after_parent_expiry')
                        if late['ticket_current'] or not late['private_certificate']:
                            return result  # No fixture error may mask an old-callback cancellation.
                    assert n._near_field_phase_deadline_sec == phase_deadline
                    return result
                except Exception as exc:
                    _save(c, tmp_path, original_error=repr(exc))
                    raise _InjectionFixtureFailure(repr(exc))

            monkeypatch.setattr(n, '_final_contact_sequence_evaluation', after_precheck)
        try:
            return original(c, patch, path)
        except Exception as exc:
            _save(c, tmp_path, original_error=repr(exc))
            raise _FirstHandoffFailure(c, exc)

    monkeypatch.setattr(chain, 'accepted_same_scene', accepted)
    return captured


def _save(c, tmp_path, original_error=None):
    row = dict(diagnostic_only=True, execution_authorized=False,
        original_error=original_error, trace=getattr(c, 'gen41_trace', []),
        rows=getattr(c, 'gen41_rows', []),
        prechecked_plan_id=getattr(c, 'gen41_prechecked_plan_id', None),
        public_geometry_to_original_task=getattr(c, 'gen41_geometry_publications', []),
        recovery_sdk_rejection=getattr(c, 'gen41_recovery_sdk_rejection', None),
        sdk_provider=getattr(c, 'gen41_sdk_provider', []),
        final_task_encoder_history=[dict(source_ns=m.header.stamp.to_nsec(),
            positions=list(m.position)) for m in c.task._following_accepted_history],
        late_old_parent_delivery=getattr(c, 'gen41_late_delivery', None),
        device_calls=c.device_calls, states=[str(s) for s in c.states],
        physical_grasp_verified=False,
        external_providers='explicit SDK/FK/IK/MoveIt/MuJoCo/actuator simulators; no original successful robot curve')
    (tmp_path/'gen41_handoff.json').write_text(json.dumps(row, default=str, indent=2)+'\n')


@pytest.mark.parametrize('order,late_gap', [
    (('status', 'object', 'mask'), False),
    (('mask', 'object', 'status'), False),
    (('object', 'mask', 'status'), True),
    (('object', 'status', 'mask'), 'partial'),
], ids=['status-first-gap', 'mask-first-gap',
        'READY-before-any-gap-fragment', 'READY-after-only-false-object'])
def test_prechecked_candidate_crosses_feedback_gap_and_idle_expiry_once(
        monkeypatch, tmp_path, order, late_gap):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'runtime_logs'))
    monkeypatch.setattr(chain, 'ray_scene', cuboid_scene)
    _install_real_reached_source_adapter(monkeypatch)
    captured = _install_injection(monkeypatch, tmp_path, order, late_gap=late_gap)
    try:
        c = chain.unified_replay(monkeypatch, tmp_path, perturb=True)
    except _FirstHandoffFailure as exc:
        pytest.fail('original first handoff failed after injection: '+repr(exc.original))
    _save(c, tmp_path)
    assert captured == [c]
    assert c.gen41_prechecked_plan_id
    assert c.contact.header.stamp.to_nsec() == c.ticket.payload[0].stamp_ns
    assert [row['kind'] for row in c.gen41_rows] == [
        'feedback_gap', 'feedback_recovery', 'idle_expiry', 'idle_recovery']
    assert [row['name'] for row in c.gen41_trace] == ([
        'original_gateway_precheck_accepted', 'complete_fresh_ready_before_any_old_fragment',
        'fresh_ready_after_feedback_gap', 'producer_idle_expiry_delivered', 'fresh_ready_after_idle_expiry']
        if late_gap is True else ([
        'original_gateway_precheck_accepted', 'partial_feedback_gap_delivered',
        'fresh_ready_after_feedback_gap', 'producer_idle_expiry_delivered',
        'fresh_ready_after_idle_expiry'] if late_gap == 'partial' else [
        'original_gateway_precheck_accepted', 'typed_feedback_gap_delivered',
        'fresh_ready_after_feedback_gap', 'producer_idle_expiry_delivered',
        'fresh_ready_after_idle_expiry']))
    assert c.execution_result is False and c.states[-1].state == 'HOLDING'
    assert len(c.task._frozen_contact_pub.messages) == 1
    assert sum(x[0] == 'motion' and x[1] == c.origin.plan_id for x in c.device_calls) == 1
    assert sum(x[0] == 'motion' and x[1] == c.contact.plan_id for x in c.device_calls) == 3
    assert sum(x[0] == 'gripper' and x[2] < .05 for x in c.device_calls) == 1
    assert not any(x[0] == 'lift' for x in c.device_calls)
    for checkpoint in c.gen41_trace:
        assert checkpoint['contact_motion_count'] == checkpoint['close_count'] == 0
    assert any(not x['valid'] and 'TARGET_PROCESSING_PENDING' in x['failure_reason']
               for x in c.gen41_geometry_publications)
    assert c.task._last_measured_aperture_sample['ok']
    assert 'duplicate_start_rejected' in c.injected_events
    _save(c, tmp_path)


@pytest.mark.parametrize('fault', ['manual', 'epoch', 'pose', 'current_false',
                                 'same_source_conflict', 'support'])
def test_true_changes_still_cancel_prechecked_private_candidate(monkeypatch, tmp_path, fault):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'runtime_logs'))
    monkeypatch.setattr(chain, 'ray_scene', cuboid_scene)
    _install_real_reached_source_adapter(monkeypatch)
    captured = _install_injection(monkeypatch, tmp_path, ('mask', 'object', 'status'), fault=fault)
    with pytest.raises(_FirstHandoffFailure) as stopped:
        chain.unified_replay(monkeypatch, tmp_path, perturb=True)
    c = stopped.value.context
    assert captured == [c] and c.gen41_prechecked_plan_id
    assert any(row['name'] == 'true_negative_change' for row in c.gen41_trace), 'failed before the intended negative injection'
    assert not any(x[0] == 'motion' and x[1] != c.origin.plan_id for x in c.device_calls)
    assert not any(x[0] == 'gripper' and x[2] < .05 for x in c.device_calls)
    assert not c.task._frozen_contact_pub.messages
    _save(c, tmp_path, original_error=repr(stopped.value.original))


def test_recovery_without_consumer_sdk_bracket_never_publishes_or_moves(monkeypatch, tmp_path):
    """Producer READY cannot replace each consumer's original 200ms proof."""
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'runtime_logs'))
    monkeypatch.setattr(chain, 'ray_scene', cuboid_scene)
    _install_real_reached_source_adapter(monkeypatch)
    captured = _install_injection(monkeypatch, tmp_path, ('status', 'object', 'mask'),
        missing_recovery_sdk=True)
    with pytest.raises(_FirstHandoffFailure) as stopped:
        chain.unified_replay(monkeypatch, tmp_path, perturb=True)
    c = stopped.value.context
    assert captured == [c] and c.gen41_prechecked_plan_id
    assert c.gen41_recovery_sdk_rejection == 'accepted_feedback_does_not_bracket_source'
    assert [row['kind'] for row in c.gen41_rows] == ['feedback_gap', 'feedback_recovery']
    assert not any(x[0] == 'motion' and x[1] != c.origin.plan_id for x in c.device_calls)
    assert not any(x[0] == 'gripper' and x[2] < .05 for x in c.device_calls)
    assert not c.task._frozen_contact_pub.messages
    assert not bool(getattr(c.remote.latest_preview_rich_plan, 'valid', False))
    _save(c, tmp_path, original_error=repr(stopped.value.original))


def _install_frozen_production_plan_validity(monkeypatch, tmp_path):
    """Opt only this long scenario into the unchanged production plan policy.

    The original GEN35 fixture's five-second policy remains in every other
    case, including the explicit long-scenario stale-plan negative below.
    All geometry, current-target, SDK and original task freshness gates run.
    """
    import hashlib
    from pathlib import Path
    import yaml

    source = (Path(chain.__file__).resolve().parents[2]
        / 'alicia_flexible_grasp_supervisor/config/grasp_params.yaml')
    raw = source.read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    historical_digest = '0655edccd1de224d0b95249165e2bfde46c47ac36ac2d38110fd61ea608fc30f'
    current_digest = '2b340b56b3bf6da963495af253144e4874a0073856e639cd7d20206678b5068a'
    # Version the current source explicitly. Recovering the historical bytes
    # below verifies the sole delta; those bytes are never loaded as policy.
    mapping_delta = (
        b'  # Explicit physical JTC reference with a frozen SDK feedforward offset.\n'
        b'  # Requires matching typed driver/HW interfaces and complete per-stage proofs.\n'
        b'  # Deploy only with a source-bound acceptance profile; legacy remains default.\n'
        b'  unknown_physical_reference_mapping_enabled: false\n')
    assert digest == current_digest
    assert raw.count(mapping_delta) == 1
    assert hashlib.sha256(raw.replace(mapping_delta, b'', 1)).hexdigest() == historical_digest
    configuration = yaml.safe_load(raw)
    assert configuration['grasp']['unknown_physical_reference_mapping_enabled'] is False
    policy = configuration['grasp_6d']['plan_validity_sec']
    assert type(policy) in (int, float) and policy == 120.0
    original = chain.arrived_same_scene

    def arrived(*args, **kwargs):
        c = original(*args, **kwargs)
        inherited = c.cfg['plan_validity_sec']
        assert inherited == 5.0
        # unified_replay builds its actual local cfg from this original c.cfg
        # immediately after arrived returns. No method or freshness gate changes.
        c.cfg = dict(c.cfg, plan_validity_sec=policy)
        c.gen41_plan_policy = dict(
            configuration_source=str(source), configuration_sha256=digest,
            historical_configuration_sha256=historical_digest,
            configuration_delta={
                'grasp.unknown_physical_reference_mapping_enabled': False},
            historical_bytes_used_as_policy=False,
            field='grasp_6d.plan_validity_sec', configured_value_sec=policy,
            unchanged_fixture_value_sec=inherited,
            scope='only the new long idle-echo and expired-parent positive',
            live_parameter_queried=False, production_configuration_changed=False,
            historical_plan_timestamp_changed=False)
        (tmp_path/'plan_policy.json').write_text(
            json.dumps(c.gen41_plan_policy, indent=2)+'\n')
        return c

    monkeypatch.setattr(chain, 'arrived_same_scene', arrived)


def test_feedback_gap_idle_echo_before_fresh_recovery_keeps_original_handoff(monkeypatch, tmp_path):
    """The real status-only echo must not turn an existing gap into pure clock."""
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'runtime_logs'))
    monkeypatch.setattr(chain, 'ray_scene', cuboid_scene)
    _install_real_reached_source_adapter(monkeypatch)
    _install_frozen_production_plan_validity(monkeypatch, tmp_path)
    captured = _install_injection(monkeypatch, tmp_path, ('status', 'object', 'mask'),
        idle_before_recovery=True, late_echo_after_parent_expiry=True)
    try:
        c = chain.unified_replay(monkeypatch, tmp_path, perturb=True)
    except _FirstHandoffFailure as exc:
        pytest.fail('original handoff cancelled at legal producer idle echo: '+repr(exc.original))
    _save(c, tmp_path)
    assert captured == [c] and c.gen41_prechecked_plan_id
    assert c.cfg['plan_validity_sec'] == c.gen41_plan_policy['configured_value_sec'] == 120.0
    assert c.contact.header.stamp.to_nsec() == c.ticket.payload[0].stamp_ns
    assert [row['kind'] for row in c.gen41_rows[:5]] == [
        'feedback_gap', 'feedback_idle_echo', 'feedback_recovery', 'idle_expiry', 'idle_recovery']
    assert c.gen41_rows[5:] and all(row['kind'] == 'late_parent_fresh_ready' for row in c.gen41_rows[5:])
    assert [row['name'] for row in c.gen41_trace[:6]] == [
        'original_gateway_precheck_accepted', 'typed_feedback_gap_delivered',
        'feedback_idle_echo_delivered', 'fresh_ready_after_feedback_gap',
        'producer_idle_expiry_delivered', 'fresh_ready_after_idle_expiry']
    assert all(row['name'] == 'fresh_ready_while_parent_ages' for row in c.gen41_trace[6:-1])
    assert c.gen41_trace[-1]['name'] == 'late_gap_and_idle_echo_after_parent_expiry'
    echo = c.gen41_trace[2]
    assert echo['pending'] and echo['task_record_pending']
    assert echo['ticket_current'] == '' and echo['private_certificate']
    assert not echo['public_preview_valid']
    late = c.gen41_trace[-1]
    current = c.gen41_late_delivery['current_before_late_delivery']
    assert late['now_ns'] > c.gen41_late_delivery['original_parent_identity_deadline_ns']
    assert late['now_ns'] < late['remote_geometry_deadline_ns']
    assert late['now_ns'] < late['task_geometry_deadline_ns']
    assert late['remote_current_source_ns'] == current['remote_current_source_ns']
    assert late['task_record_source_ns'] == current['task_record_source_ns']
    assert not late['pending'] and not late['task_record_pending']
    assert late['ticket_current'] == '' and late['private_certificate']
    assert all(row['contact_motion_count'] == row['close_count'] == 0 for row in c.gen41_trace)
    assert len({row['phase_deadline_sec'] for row in c.gen41_trace}) == 1
    assert c.execution_result is False and c.states[-1].state == 'HOLDING'
    assert len(c.task._frozen_contact_pub.messages) == 1
    assert sum(x[0] == 'motion' and x[1] == c.origin.plan_id for x in c.device_calls) == 1
    assert sum(x[0] == 'motion' and x[1] == c.contact.plan_id for x in c.device_calls) == 3
    assert sum(x[0] == 'gripper' and x[2] < .05 for x in c.device_calls) == 1
    assert not any(x[0] == 'lift' for x in c.device_calls)
    assert c.task._last_measured_aperture_sample['ok']
    assert 'duplicate_start_rejected' in c.injected_events
    assert any(not x['valid'] and 'TARGET_PROCESSING_PENDING' in x['failure_reason']
               for x in c.gen41_geometry_publications)


def test_idle_echo_with_inconsistent_gap_span_never_authorizes_original_plan(monkeypatch, tmp_path):
    """A repeat header cannot excuse a full proof inconsistent with its SDK."""
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'runtime_logs'))
    monkeypatch.setattr(chain, 'ray_scene', cuboid_scene)
    _install_real_reached_source_adapter(monkeypatch)
    captured = _install_injection(monkeypatch, tmp_path, ('status', 'object', 'mask'),
        idle_before_recovery=True, corrupt_idle_echo=True)
    with pytest.raises(_FirstHandoffFailure) as stopped:
        chain.unified_replay(monkeypatch, tmp_path, perturb=True)
    c = stopped.value.context
    assert captured == [c] and c.gen41_prechecked_plan_id
    assert [row['kind'] for row in c.gen41_rows] == [
        'feedback_gap', 'feedback_idle_echo', 'feedback_idle_echo_wire_conflict']
    assert c.gen41_trace[-1]['name'] == 'feedback_idle_echo_badproof_delivered'
    assert c.gen41_trace[-1]['ticket_current']
    assert not c.gen41_trace[-1]['private_certificate']
    assert (c.gen41_trace[-1]['remote_retired'] or c.gen41_trace[-1]['task_retired']
            or c.gen41_trace[-1]['task_authority_revoked']
            or c.gen41_trace[-1]['task_hard_cutoff_ns'] >= c.gen41_rows[0]['row']['status']['source_stamp_ns'])
    assert not any(x[0] == 'motion' and x[1] != c.origin.plan_id for x in c.device_calls)
    assert not any(x[0] == 'gripper' and x[2] < .05 for x in c.device_calls)
    assert not c.task._frozen_contact_pub.messages
    assert all(row['contact_motion_count'] == row['close_count'] == 0 for row in c.gen41_trace)
    _save(c, tmp_path, original_error=repr(stopped.value.original))


def test_same_long_idle_echo_scenario_keeps_five_second_plan_expiry_hard(
        monkeypatch, tmp_path, caplog):
    """Fresh station binding cannot override the inherited plan-age policy."""
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'runtime_logs'))
    monkeypatch.setattr(chain, 'ray_scene', cuboid_scene)
    _install_real_reached_source_adapter(monkeypatch)
    captured = _install_injection(monkeypatch, tmp_path, ('status', 'object', 'mask'),
        idle_before_recovery=True, late_echo_after_parent_expiry=True)
    c = chain.unified_replay(monkeypatch, tmp_path, perturb=True)
    _save(c, tmp_path)
    assert captured == [c] and c.gen41_prechecked_plan_id
    assert c.cfg['plan_validity_sec'] == 5.0  # Original GEN35 fixture, unchanged.
    assert c.contact.header.stamp.to_nsec() == c.ticket.payload[0].stamp_ns
    assert [row['kind'] for row in c.gen41_rows[:5]] == [
        'feedback_gap', 'feedback_idle_echo', 'feedback_recovery', 'idle_expiry', 'idle_recovery']
    assert c.gen41_rows[5:] and all(row['kind'] == 'late_parent_fresh_ready' for row in c.gen41_rows[5:])
    assert c.gen41_trace[-1]['name'] == 'late_gap_and_idle_echo_after_parent_expiry'
    echo, late = c.gen41_trace[2], c.gen41_trace[-1]
    assert echo['pending'] and echo['task_record_pending']
    assert echo['ticket_current'] == '' and echo['private_certificate']
    assert late['now_ns'] > c.gen41_late_delivery['original_parent_identity_deadline_ns']
    assert late['now_ns'] < late['remote_geometry_deadline_ns']
    assert late['now_ns'] < late['task_geometry_deadline_ns']
    current = c.gen41_late_delivery['current_before_late_delivery']
    assert late['remote_current_source_ns'] == current['remote_current_source_ns']
    assert late['task_record_source_ns'] == current['task_record_source_ns']
    assert not late['pending'] and not late['task_record_pending']
    assert late['ticket_current'] == '' and late['private_certificate']
    source_ns = c.contact.header.stamp.to_nsec()
    age_sec = (late['now_ns']-source_ns)*1e-9
    assert age_sec > c.cfg['plan_validity_sec']
    # This is the original task _copy -> base validate_execution_plan result,
    # caught only by the unchanged service fixture and recorded in its log.
    assert 'FINAL_REFINE_3D_INVALID' in caplog.text
    assert 'PLAN_STALE: plan source timestamp is stale' in caplog.text
    assert c.execution_result is False and c.states[-1].state == 'FAILED'
    assert not c.task._frozen_contact_pub.messages
    assert sum(x[0] == 'motion' and x[1] == c.origin.plan_id for x in c.device_calls) == 1
    assert not any(x[0] == 'motion' and x[1] != c.origin.plan_id for x in c.device_calls)
    assert not any(x[0] == 'gripper' and x[2] < .05 for x in c.device_calls)
    assert all(row['contact_motion_count'] == row['close_count'] == 0 for row in c.gen41_trace)
    assert len({row['phase_deadline_sec'] for row in c.gen41_trace}) == 1
    (tmp_path/'plan_freshness_boundary.json').write_text(json.dumps(dict(
        policy_source='unchanged GEN35 fixture c.cfg.plan_validity_sec',
        configured_value_sec=c.cfg['plan_validity_sec'],
        original_source_stamp_ns=source_ns, copy_now_ns=late['now_ns'],
        source_age_sec=age_sec, expected_original_code='PLAN_STALE',
        current_registration_still_fresh=True,
        historical_plan_timestamp_changed=False, simulated_contact_motion_count=0,
        simulated_close_count=0, physical_grasp_verified=False), indent=2)+'\n')
