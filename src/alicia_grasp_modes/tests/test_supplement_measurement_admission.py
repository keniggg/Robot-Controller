"""Actual producer RGB-D completion and next-lease admission, no ROS master."""
from copy import deepcopy
import json
import threading

import pytest
from alicia_flexible_grasp_supervisor.msg import GraspState

from test_station_compute_wait_producer import armed
from test_locked_rgb_hold import custom_frame, latest_status, module
from test_observation_measurement_unknown import send, ack, arrival, next_frame


def next_request(node):
    request = deepcopy(node._observation_measurement_status['request'])
    for key in ('settle', 'source_stamps_ns'):
        request.pop(key, None)
    request.update(state='executing', lease_id='second_lease', request_id='second_execution',
        view_plan_id='second_view', view_index=2, view_source_stamp_ns=node.last_good_source_stamp_ns,
        stamp_ns=module.rospy.Time.now().to_nsec(), arrival_stamp_ns=0, capture_deadline_ns=0)
    return request


def at_inflight(monkeypatch, node, clock, joint, action, *, fresh=False):
    """Release lock during segmenting so the real callback can interleave."""
    entered, release = threading.Event(), threading.Event()
    original = node._frame_timing_stage
    old = node._locked_rgb_hold_reference
    errors, sources = [], []
    base = old['source_stamp_ns'] * 1e-9
    node._station_compute_state_cb(GraspState(state='PLAN_PREGRASP', active=True,
        message='admitting supplemental observation view'))
    def timing(stage, *args, **kwargs):
        result = original(stage, *args, **kwargs)
        if stage == 'post_segment':
            entered.set()
            assert release.wait(3.)
        return result
    def worker():
        try:
            sources.append(custom_frame(node, clock, joint, target_depth=.35))
        except Exception as exc:
            errors.append(exc)
    clock[0] = base + .500645971
    with monkeypatch.context() as changed:
        changed.setattr(node, '_frame_timing_stage', timing)
        thread = threading.Thread(target=worker)
        thread.start()
        assert entered.wait(3.)
        try:
            clock[0] = base + (.7 if fresh else .804669380)
            joint(clock[0]-.001)
            action()
        finally:
            release.set()
        thread.join(3.)
        assert not thread.is_alive() and not errors, errors
    return sources[0]


def test_gen5_boundary_waits_for_exact_real_ready_then_second_lease_three_frames(monkeypatch):
    node, clock, joint, params, marker, first = armed(monkeypatch)
    anchor = node.tracker.anchor
    previous = node._locked_rgb_hold_reference
    acks_before = node._observation_measurement_ack_pub.publish.call_count
    object_before = node.pubs['object'].publish.call_count
    requests, captured = [], []
    def queue():
        request = next_request(node); requests.append(request)
        send(node, request)
        admission = node._supplement_measurement_admission
        captured.append(admission)
        deadlines = (admission['deadline_ns'], admission['monotonic_deadline'])
        assert admission is not None and admission['reference'] is previous
        assert admission['deadline_ns'] <= admission['source_ns'] + 800000000
        assert admission['source_ns'] + 800000000 <= request['stamp_ns'] + 800000000
        assert node._observation_measurement is None
        assert node._observation_measurement_ack_pub.publish.call_count == acks_before
        assert node.last_good_source_stamp_ns == previous['source_stamp_ns']
        assert node.pubs['object'].publish.call_count == object_before
        # A repeated envelope neither ACKs old geometry nor advances clocks.
        send(node, request)
        assert node._supplement_measurement_admission is admission
        assert (admission['deadline_ns'], admission['monotonic_deadline']) == deadlines
        assert node._observation_measurement_ack_pub.publish.call_count == acks_before
    source = at_inflight(monkeypatch, node, clock, joint, queue)
    request = requests[0]
    assert ack(node)['accepted'] and ack(node)['request_id'] == request['request_id'], ack(node)
    assert node._supplement_measurement_admission is None
    assert node._observation_measurement['reference_stamp_ns'] == source[0].header.stamp.to_nsec()
    assert source[0].header.stamp.to_nsec() <= request['stamp_ns']
    assert node._observation_measurement['reference_stamp_ns'] != previous['source_stamp_ns']
    assert node.pubs['object'].publish.call_count == object_before + 1
    assert latest_status(node)['state'] == 'ready'
    assert latest_status(node)['source_stamp_ns'] == source[0].header.stamp.to_nsec()
    assert node.first_loss is None and not node.tracker.lost
    assert node._locked_rgb_hold_reference is None
    assert marker['session_deadline_ns'] == request['deadline_ns']
    # The original executing/requalifying/qualified protocol remains intact.
    node._station_compute_state_cb(GraspState(state='MOVE_PREGRASP', active=True))
    second = arrival(node, clock, joint, request)
    stamps = []
    for _ in range(3):
        pair = next_frame(node, clock, joint)
        stamps.append(pair[0].header.stamp.to_nsec())
    assert ack(node)['qualified'] and ack(node)['source_stamps_ns'] == stamps
    done = dict(second, state='qualified', request_id='second_done',
        stamp_ns=module.rospy.Time.now().to_nsec(), source_stamps_ns=stamps)
    send(node, done)
    assert ack(node)['accepted'] and ack(node)['qualified']
    assert node._locked_rgb_hold_reference['observation_arrival_retry']['measurement_binding']['view_index'] == 2
    assert node.tracker.anchor is anchor and not node.tracker.lost and node.first_loss is None


def test_fresh_reference_freezes_normally_and_old_inflight_frame_cannot_report_loss(monkeypatch):
    node, clock, joint, params, marker, first = armed(monkeypatch)
    old = node._locked_rgb_hold_reference
    count = node.pubs['object'].publish.call_count
    def freeze():
        send(node, next_request(node))
        assert ack(node)['accepted'], ack(node)
        assert getattr(node, '_supplement_measurement_admission', None) is None
        assert node._observation_measurement['reference_stamp_ns'] == old['source_stamp_ns']
    at_inflight(monkeypatch, node, clock, joint, freeze, fresh=True)
    assert node.first_loss is None and not node.tracker.lost
    assert node.pubs['object'].publish.call_count == count
    assert latest_status(node)['state'] == 'pending'


@pytest.mark.parametrize('fault', ['expired', 'monotonic', 'manual', 'epoch', 'pose_out_back', 'terminal_restore',
    'config', 'camera', 'model', 'handeye', 'selection', 'lost', 'revoke'])
def test_queued_admission_cannot_survive_invalidated_context(monkeypatch, fault):
    node, clock, joint, params, marker, first = armed(monkeypatch)
    requests = []
    ack_start = node._observation_measurement_ack_pub.publish.call_count
    def queue_and_change():
        request = next_request(node); requests.append(request)
        send(node, request)
        assert node._supplement_measurement_admission is not None
        if fault == 'expired':
            clock[0] = node._supplement_measurement_admission['deadline_ns']*1e-9+.001
            joint(clock[0]-.001)
        elif fault == 'monotonic':
            deadline = node._supplement_measurement_admission['monotonic_deadline']
            monkeypatch.setattr(module.time, 'monotonic', lambda: deadline + .001)
        elif fault == 'manual': params['/gui/joint_direct_mode'] = True
        elif fault == 'epoch': node._rgb_interest_epoch += 1
        elif fault == 'pose_out_back':
            joint(clock[0], [.005, 0, 0, 0, 0, 0]); joint(clock[0]+.001)
        elif fault == 'terminal_restore':
            node._station_compute_state_cb(GraspState(state='FAILED', active=False))
            node._station_compute_state_cb(GraspState(state='PLAN_PREGRASP', active=True,
                message='admitting supplemental observation view'))
        elif fault == 'config': node.config.support_threshold_m = .0019
        elif fault == 'camera': params['/camera']['fx'] += 1
        elif fault == 'model': params['/robot_description'] = '<robot name="changed"/>'
        elif fault == 'handeye': params['/handeye']['changed'] = True
        elif fault == 'selection': params['/grasp_mode/selection']['generation'] += 1
        elif fault == 'lost': node.tracker.lost = True
        else:
            clock[0] += .001
            send(node, dict(request, state='revoked', request_id='second_revoke',
                stamp_ns=module.rospy.Time.now().to_nsec(), reason='user_cancelled'))
    at_inflight(monkeypatch, node, clock, joint, queue_and_change)
    assert node._supplement_measurement_admission is None
    assert node._observation_measurement is None
    receipts = [json.loads(c.args[0].data) for c in
        node._observation_measurement_ack_pub.publish.call_args_list[ack_start:]]
    assert receipts and not any(r['accepted'] for r in receipts), receipts
    # A terminated request cannot resurrect itself after later fresh imagery.
    assert requests[0]['lease_id'] in node._observation_measurement_seen


def test_conflicting_request_does_not_replace_pending_exact_frame(monkeypatch):
    node, clock, joint, params, marker, first = armed(monkeypatch)
    requests = []
    def queue():
        request = next_request(node); requests.append(request)
        send(node, request)
        admission = node._supplement_measurement_admission
        conflicting = dict(request, lease_id='replacement', request_id='replacement_request')
        send(node, conflicting)
        assert not ack(node)['accepted'] and ack(node)['request_id'] == 'replacement_request'
        assert node._supplement_measurement_admission is admission
        assert admission['request'] == request
    source = at_inflight(monkeypatch, node, clock, joint, queue)
    assert ack(node)['accepted'] and ack(node)['request_id'] == requests[0]['request_id']
    assert node._observation_measurement['reference_stamp_ns'] == source[0].header.stamp.to_nsec()


def test_completion_that_fails_real_refinement_never_acks_old_or_later_source(monkeypatch):
    node, clock, joint, params, marker, first = armed(monkeypatch)
    original = module.refine_candidate
    ack_count = node._observation_measurement_ack_pub.publish.call_count
    requests = []
    def queue():
        request = next_request(node); requests.append(request)
        send(node, request)
        assert node._supplement_measurement_admission is not None
    def reject(*args, **kwargs):
        raise ValueError('test_measured_refinement_rejection')
    monkeypatch.setattr(module, 'refine_candidate', reject)
    at_inflight(monkeypatch, node, clock, joint, queue)
    assert node._supplement_measurement_admission is None and node._observation_measurement is None
    receipts = [json.loads(c.args[0].data) for c in node._observation_measurement_ack_pub.publish.call_args_list[ack_count:]]
    assert receipts and not any(r['accepted'] for r in receipts)
    monkeypatch.setattr(module, 'refine_candidate', original)
    clock[0] += .1
    custom_frame(node, clock, joint, target_depth=.35)
    send(node, requests[0])
    assert not ack(node)['accepted'] and node._observation_measurement is None


@pytest.mark.parametrize('fault', ['completion_deadline', 'manual', 'epoch', 'camera'])
def test_ready_freeze_rechecks_after_reference_feature_construction(monkeypatch, fault):
    node, clock, joint, params, marker, first = armed(monkeypatch)
    ack_count = node._observation_measurement_ack_pub.publish.call_count
    original = module.FrozenObservationAppearance
    def features(*args, **kwargs):
        result = original(*args, **kwargs)
        admission = getattr(node, '_supplement_measurement_admitting', None)
        if admission is not None:
            if fault == 'completion_deadline':
                clock[0] = admission['deadline_ns']*1e-9+.001
                joint(clock[0]-.001)
            elif fault == 'manual': params['/gui/joint_direct_mode'] = True
            elif fault == 'epoch': node._rgb_interest_epoch += 1
            else: params['/camera']['fx'] += 1
        return result
    monkeypatch.setattr(module, 'FrozenObservationAppearance', features)
    def queue():
        send(node, next_request(node))
        assert node._supplement_measurement_admission is not None
    at_inflight(monkeypatch, node, clock, joint, queue)
    assert node._supplement_measurement_admission is None and node._observation_measurement is None
    receipts = [json.loads(c.args[0].data) for c in node._observation_measurement_ack_pub.publish.call_args_list[ack_count:]]
    assert receipts and not any(r['accepted'] for r in receipts)


@pytest.mark.parametrize('fault', ['no_completion', 'request_before_source', 'wrong_view',
    'wrong_origin', 'wrong_session', 'wrong_deadline', 'wrong_label'])
def test_stale_reference_alone_never_creates_a_queue(monkeypatch, fault):
    node, clock, joint, params, marker, first = armed(monkeypatch)
    def reject():
        request = next_request(node)
        if fault == 'no_completion': node._arrival_frame_completion = None
        elif fault == 'request_before_source': request['stamp_ns'] = node._station_compute_frame['source_ns']-1
        elif fault == 'wrong_view': request['view_index'] += 1
        elif fault == 'wrong_origin': request['origin_plan_id'] = 'other'
        elif fault == 'wrong_session': request['session_id'] = 'other'
        elif fault == 'wrong_deadline': request['deadline_ns'] -= 1
        else: node._station_compute_task_state.message = 'planning other action'
        send(node, request)
        assert not ack(node)['accepted'], ack(node)
        assert getattr(node, '_supplement_measurement_admission', None) is None
    at_inflight(monkeypatch, node, clock, joint, reject)
    assert node._observation_measurement is None
