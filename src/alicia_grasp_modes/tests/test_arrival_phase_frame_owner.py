"""A timely first phase retires only unfinished work owned by its predecessor."""
from copy import deepcopy

import pytest
from alicia_flexible_grasp_supervisor.msg import NearFieldPlanningPhase, GraspState
from test_observation_arrival_station import completed
from test_locked_rgb_hold import custom_frame, latest_status, module


def enter_phase(node, clock):
    phase = NearFieldPlanningPhase(active=True, phase_id=1, planning_stage='contact_replan')
    phase.header.stamp = module.rospy.Time.now()
    ns = node.last_good_source_stamp_ns
    phase.reference_source_stamp = module.rospy.Time(ns//10**9, ns%10**9)
    phase.reference_target_track_id = 'target1'
    phase.deadline = module.rospy.Time.from_sec(clock[0]+30.)
    node._station_compute_state_cb(GraspState(state='PLAN_PREGRASP', active=True))
    node._station_compute_phase_cb(phase)
    return phase


@pytest.mark.parametrize('stage', ['post_tf', 'post_segment', 'post_refine'])
def test_first_phase_during_old_frame_discards_without_publication_then_measures_new_frame(monkeypatch, stage):
    n, clock, _, joint, _, marker = completed(monkeypatch)
    old, oldlast = n._locked_rgb_hold_reference, n.tracker.last
    original = n._frame_timing_stage
    called = []
    def timing(value):
        original(value)
        if value == stage and not called:
            called.append(enter_phase(n, clock))
    monkeypatch.setattr(n, '_frame_timing_stage', timing)
    count = n.pubs['object'].publish.call_count
    status_count = n.pubs['detector_status'].publish.call_count
    clock[0] += .2
    source = custom_frame(n, clock, joint, target_depth=.35)
    assert called
    assert n.pubs['object'].publish.call_count == count
    assert n.pubs['detector_status'].publish.call_count == status_count
    assert n._locked_rgb_hold_reference is old and n.tracker.last is oldlast
    assert n.last_good_source_stamp_ns == old['source_stamp_ns'] and n.first_loss is None
    assert n._station_compute_registration is None and n._arrival_phase_frame is None
    assert old['observation_arrival_retry'] is marker
    clock[0] += .2
    fresh = custom_frame(n, clock, joint, target_depth=.35)
    status = latest_status(n)
    assert status['state'] == 'ready', status
    contract = status['station_compute_wait']
    assert contract['source_ns'] == fresh[0].header.stamp.to_nsec()
    assert contract['binding']['phase_started_ns'] < contract['source_ns']
    assert contract['binding']['phase_reference_source_ns'] == old['source_stamp_ns']
    assert source[0].header.stamp.to_nsec() < called[0].header.stamp.to_nsec()
    assert n.last_good_source_stamp_ns == fresh[0].header.stamp.to_nsec()


@pytest.mark.parametrize('fault', ['manual', 'epoch', 'selection', 'pose', 'camera', 'source',
    'anchor', 'plane', 'expired_reference', 'session_deadline'])
def test_first_phase_never_hides_actual_station_invalidity(monkeypatch, fault):
    n, clock, _, joint, params, marker = completed(monkeypatch)
    old = n.last_good_source_stamp_ns
    original = n._frame_timing_stage
    def timing(value):
        original(value)
        if value != 'post_tf': return
        enter_phase(n, clock)
        if fault == 'manual': params['/gui/joint_direct_mode'] = True
        elif fault == 'epoch': n._rgb_interest_epoch += 1
        elif fault == 'selection': params['/grasp_mode/selection']['stamp_ns'] += 1
        elif fault == 'pose': joint(clock[0], [.005, 0., 0., 0., 0., 0.]); joint(clock[0]+.001)
        elif fault == 'camera': params['/camera']['fx'] += 1.
        elif fault == 'source': params['/handeye']['changed'] = True
        elif fault == 'anchor': n.tracker.anchor.position_base[0] += .001
        elif fault == 'plane': n.tracker.plane_base[3] += .001
        elif fault == 'expired_reference': clock[0] = old*1e-9+.800001; joint(clock[0])
        elif fault == 'session_deadline': marker['session_deadline_ns'] = module.rospy.Time.now().to_nsec()
    monkeypatch.setattr(n, '_frame_timing_stage', timing)
    clock[0] += .2
    custom_frame(n, clock, joint, target_depth=.35)
    assert n.last_good_source_stamp_ns == old
    assert n.first_loss is not None and n.tracker.lost
    assert not n.pubs['object'].publish.call_args[0][0].detected
    assert n._station_compute_registration is None


def test_phase_before_frame_and_duplicate_phase_are_not_retirement(monkeypatch):
    n, clock, _, joint, _, _ = completed(monkeypatch)
    phase = enter_phase(n, clock)
    original = n._frame_timing_stage
    def timing(value):
        original(value)
        if value == 'post_tf': n._station_compute_phase_cb(deepcopy(phase))
    monkeypatch.setattr(n, '_frame_timing_stage', timing)
    clock[0] += .2
    pair = custom_frame(n, clock, joint, target_depth=.35)
    assert latest_status(n)['state'] == 'ready'
    assert n.last_good_source_stamp_ns == pair[0].header.stamp.to_nsec()


def test_late_next_unique_frame_cannot_inherit_expired_arrival_or_register_station(monkeypatch):
    n, clock, _, joint, _, _ = completed(monkeypatch)
    old = n.last_good_source_stamp_ns
    original = n._frame_timing_stage
    def timing(value):
        original(value)
        if value == 'post_tf': enter_phase(n, clock)
    monkeypatch.setattr(n, '_frame_timing_stage', timing)
    clock[0] += .2
    custom_frame(n, clock, joint, target_depth=.35)
    assert n.last_good_source_stamp_ns == old and n.first_loss is None
    clock[0] = old*1e-9 + .8173
    fresh = custom_frame(n, clock, joint, target_depth=.35)
    # The original unique ordinary tracker may accept this new measurement,
    # but expired station authority does not survive with it.
    assert n.last_good_source_stamp_ns == fresh[0].header.stamp.to_nsec()
    assert n._locked_rgb_hold_reference.get('observation_arrival_retry') is None
    assert n._station_compute_registration is None
    assert 'station_compute_wait' not in latest_status(n)


@pytest.mark.parametrize('operation', ['refine_candidate', 'segment_with_support_fallback'])
def test_true_processing_error_after_phase_message_is_not_reclassified_as_cancellation(monkeypatch, operation):
    n, clock, _, joint, _, _ = completed(monkeypatch)
    old = n.last_good_source_stamp_ns
    original = getattr(module, operation)
    def failed_refinement(*args, **kwargs):
        original(*args, **kwargs)
        enter_phase(n, clock)
        raise ValueError('deliberate_original_identity_rejection')
    monkeypatch.setattr(module, operation, failed_refinement)
    clock[0] += .2
    custom_frame(n, clock, joint, target_depth=.35)
    if operation == 'segment_with_support_fallback':
        assert n.tracker.lost and n.first_loss is not None
    else:
        # Ordinary native refinement already has an explicit lose=False
        # failure path; preserve that behavior rather than inventing loss.
        assert not n.tracker.lost and n.first_loss is None
    assert not (n._processing_timing or {}).get('arrival_phase_frame_cancelled')
    assert n.last_good_source_stamp_ns == old
    assert 'deliberate_original_identity_rejection' in latest_status(n)['detail']
    assert not n.pubs['object'].publish.call_args[0][0].detected


@pytest.mark.parametrize('callback_first', [False, True])
def test_real_callback_thread_and_commit_lock_define_publication_order(monkeypatch, callback_first):
    import threading
    n, clock, _, joint, _, _ = completed(monkeypatch)
    old, old_last = n._locked_rgb_hold_reference, n.tracker.last
    phase = NearFieldPlanningPhase(active=True, phase_id=1, planning_stage='contact_replan')
    phase.reference_source_stamp = module.rospy.Time(old['source_stamp_ns']//10**9, old['source_stamp_ns']%10**9)
    phase.reference_target_track_id = 'target1'
    started, finished = threading.Event(), threading.Event()
    errors, threads = [], []
    def callback():
        started.set()
        try:
            n._station_compute_state_cb(GraspState(state='PLAN_PREGRASP', active=True))
            n._station_compute_phase_cb(phase)
        except Exception as exc: errors.append(exc)
        finally: finished.set()
    original_timing = n._frame_timing_stage
    original_commit = n._station_compute_commit_check
    def begin_thread():
        phase.header.stamp = module.rospy.Time.now()
        phase.deadline = module.rospy.Time.from_sec(clock[0]+30.)
        thread = threading.Thread(target=callback)
        threads.append(thread); thread.start()
        assert started.wait(1.)
    def timing(value):
        original_timing(value)
        if value == 'post_refine' and callback_first:
            begin_thread()
            assert finished.wait(1.)
    def commit(*args, **kwargs):
        if not callback_first:
            # Real callback blocks behind process's final input lock, rather
            # than a same-thread recursive RLock pretending to be a race.
            begin_thread()
            assert not finished.wait(.01)
        return original_commit(*args, **kwargs)
    monkeypatch.setattr(n, '_frame_timing_stage', timing)
    monkeypatch.setattr(n, '_station_compute_commit_check', commit)
    count = n.pubs['object'].publish.call_count
    clock[0] += .2
    source = custom_frame(n, clock, joint, target_depth=.35)
    for thread in threads: thread.join(1.)
    assert finished.is_set() and not errors
    if callback_first:
        assert n.pubs['object'].publish.call_count == count
        assert n._locked_rgb_hold_reference is old and n.tracker.last is old_last
    else:
        assert n.pubs['object'].publish.call_count == count+1
        assert n.last_good_source_stamp_ns == source[0].header.stamp.to_nsec()
        assert latest_status(n)['state'] == 'ready'
        assert 'station_compute_wait' not in latest_status(n)
    assert n.first_loss is None and n._arrival_phase_frame is None


@pytest.mark.parametrize('phase_reference', ['qualified', 'unmeasured_between'])
def test_phase_keeps_original_qualified_reference_when_a_new_ready_arrived(monkeypatch, phase_reference):
    n, clock, _, joint, _, marker = completed(monkeypatch)
    qualified = marker['qualified_source_ns']
    clock[0] += .2
    custom_frame(n, clock, joint, target_depth=.35)
    old = n._locked_rgb_hold_reference
    assert old['source_stamp_ns'] > qualified
    count = n.pubs['object'].publish.call_count
    original = n._frame_timing_stage
    def timing(value):
        original(value)
        if value == 'post_tf':
            phase = enter_phase(n, clock)
            ns = qualified if phase_reference == 'qualified' else qualified+1
            phase.reference_source_stamp = module.rospy.Time(ns//10**9, ns%10**9)
            n._station_compute_phase_cb(phase)
    monkeypatch.setattr(n, '_frame_timing_stage', timing)
    clock[0] += .2
    pair = custom_frame(n, clock, joint, target_depth=.35)
    if phase_reference == 'qualified':
        assert n.pubs['object'].publish.call_count == count
        assert n._locked_rgb_hold_reference is old
    else:
        assert n.pubs['object'].publish.call_count == count+1
        assert n.last_good_source_stamp_ns == pair[0].header.stamp.to_nsec()
    assert n._station_compute_registration is None and n.first_loss is None
