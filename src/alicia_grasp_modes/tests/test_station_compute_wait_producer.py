"""Actual three-frame arrival, measured normal READY, discard and reacquire."""
from copy import deepcopy
import hashlib

import numpy as np
import pytest
from alicia_flexible_grasp_supervisor.msg import NearFieldPlanningPhase, GraspState

from test_observation_arrival_station import completed
from test_arrival_fragment_group import completed_fragment_arrival, split_depth
from test_locked_rgb_hold import custom_frame, latest_status, module, locked


def armed(monkeypatch, fragment=False):
    setup = completed_fragment_arrival if fragment else completed
    node, clock, feed, joint, params, marker = setup(monkeypatch)
    phase = NearFieldPlanningPhase()
    phase.active = True; phase.phase_id = 7; phase.planning_stage = 'contact_replan'
    phase.header.stamp = module.rospy.Time.now()
    phase.reference_source_stamp = module.rospy.Time.from_sec(node.last_good_source_stamp_ns*1e-9)
    phase.reference_target_track_id = 'target1'
    phase.deadline = module.rospy.Time.from_sec(clock[0]+30)
    node._station_compute_phase_cb(phase)
    node._station_compute_state_cb(GraspState(state='PLAN_PREGRASP', active=True))
    clock[0] += .3
    source = custom_frame(node, clock, joint, target_depth=split_depth() if fragment else .35)
    status = latest_status(node)
    assert status['state'] == 'ready', status
    assert 'station_compute_wait' in status, status
    assert status['station_compute_mask']['sha256'] == hashlib.sha256(
        np.ascontiguousarray(node._locked_rgb_hold_reference['mask'] > 0, dtype=np.uint8).tobytes()).hexdigest()
    return node, clock, joint, params, marker, source


def overrun(monkeypatch, node, clock, joint, fragment=False, mutation=None):
    name = 'recover_arrival_fragment_group' if fragment else 'refine_candidate'
    original = getattr(module, name)
    old = node._locked_rgb_hold_reference
    oldlast = node.tracker.last
    def expensive(*args, **kwargs):
        result = original(*args, **kwargs)
        assert node.tracker.last is oldlast
        # Same source age as the recorded GEN31 failure; independent feedback
        # continues while this synchronous pure calculation returns late.
        clock[0] += 3.197677
        joint(clock[0]-.01)
        if mutation is not None:
            mutation(node, clock, joint)
        return result
    with monkeypatch.context() as change:
        change.setattr(module, name, expensive)
        clock[0] += .2
        pair = custom_frame(node, clock, joint, target_depth=split_depth() if fragment else .35)
    assert node.tracker.last is oldlast
    assert node.last_good_source_stamp_ns == old['source_stamp_ns']
    assert not node.pubs['object'].publish.call_args[0][0].detected
    return pair, old


@pytest.mark.parametrize('fragment', [False, True])
def test_complete_arrival_ready_overrun_discards_then_fresh_full_measurement_recovers(monkeypatch, fragment):
    node, clock, joint, params, marker, ready = armed(monkeypatch, fragment)
    contract = deepcopy(latest_status(node)['station_compute_wait'])
    mono = node._station_compute_registration['monotonic_deadline']
    pair, old = overrun(monkeypatch, node, clock, joint, fragment)
    status = latest_status(node)
    assert status['state'] == 'processing_deadline_pending', status
    assert status['station_compute_wait'] == contract
    assert not status['target_lost'] and node.first_loss is None
    assert node._locked_rgb_hold_reference is old
    assert node._station_compute_registration['monotonic_deadline'] == mono
    clock[0] += .2
    fresh = custom_frame(node, clock, joint, target_depth=split_depth() if fragment else .35)
    status = latest_status(node)
    assert status['state'] == 'ready', status
    assert status['station_compute_recovery'] == contract
    assert status['station_compute_wait']['source_ns'] == fresh[0].header.stamp.to_nsec()
    assert node.last_good_source_stamp_ns == fresh[0].header.stamp.to_nsec()
    assert old['source_stamp_ns'] == contract['source_ns']
    assert node._locked_rgb_hold_reference['observation_arrival_retry'] is marker
    assert node._station_compute_frame is None


@pytest.mark.parametrize('change', ['manual', 'motion', 'epoch', 'phase', 'state', 'camera', 'selection'])
def test_overrun_with_positive_change_never_becomes_identity_wait(monkeypatch, change):
    node, clock, joint, params, marker, source = armed(monkeypatch)
    def mutation(n, clock, joint):
        if change == 'manual': params['/gui/joint_direct_mode'] = True
        elif change == 'motion': joint(clock[0], [.005, 0, 0, 0, 0, 0]); joint(clock[0]+.001)
        elif change == 'epoch': n._rgb_interest_epoch += 1
        elif change == 'phase':
            phase = deepcopy(n._station_compute_phase); phase.phase_id += 1; n._station_compute_phase_cb(phase)
        elif change == 'state': n._station_compute_state_cb(GraspState(state='MOVE_PREGRASP', active=True))
        elif change == 'camera': params['/camera']['fx'] += 1
        elif change == 'selection': params['/grasp_mode/selection']['generation'] += 1
    overrun(monkeypatch, node, clock, joint, mutation=mutation)
    assert latest_status(node)['state'] != 'processing_deadline_pending'
    assert node.first_loss is not None


def test_fixed_five_seconds_and_monotonic_expiry_cannot_reenroll_from_new_failed_input(monkeypatch):
    node, clock, joint, params, marker, source = armed(monkeypatch)
    original = node._station_compute_registration
    overrun(monkeypatch, node, clock, joint)
    clock[0] = original['contract']['identity_deadline_ns']*1e-9+.001
    custom_frame(node, clock, joint, target_depth=.35)
    assert node.first_loss is not None and node._station_compute_registration is None


def test_ready_duplicate_does_not_refresh_monotonic_deadline(monkeypatch):
    node, clock, joint, params, marker, source = armed(monkeypatch)
    original = node._station_compute_registration
    clock[0] += .02
    node.status('ready', 'tracked', source, node.tracker.last)
    assert node._station_compute_registration is original


def test_initial_and_absent_contact_phase_ready_keep_original_path(monkeypatch):
    node, clock, feed, joint, params = locked(monkeypatch)
    assert 'station_compute_wait' not in latest_status(node)
    node, clock, feed, joint, params, marker = completed(monkeypatch)
    clock[0] += .2
    custom_frame(node, clock, joint, target_depth=.35)
    assert latest_status(node)['state'] == 'ready'
    assert 'station_compute_wait' not in latest_status(node)


@pytest.mark.parametrize('kind', ['empty', 'stale_slot'])
def test_registered_wait_with_no_fresh_input_keeps_fixed_identity_only_deadline(monkeypatch, kind):
    node, clock, joint, params, marker, source = armed(monkeypatch)
    registration = node._station_compute_registration
    before = node.last_good_source_stamp_ns
    contract = deepcopy(registration['contract'])
    clock[0] += 1.3
    joint(clock[0]-.01)
    if kind == 'empty':
        node.latest = None; node.process(None)
    else:
        # Existing latest-only slot can itself age during a slow prior call.
        custom_frame(node, clock, joint, target_depth=.35, old_stamp=True)
    assert latest_status(node)['state'] == 'processing_deadline_pending', latest_status(node)
    assert latest_status(node)['station_compute_wait'] == contract
    assert node._station_compute_registration is registration
    assert node.last_good_source_stamp_ns == before and node.first_loss is None


@pytest.mark.parametrize('mutation', ['rgb', 'support', 'epoch_at_commit'])
def test_recovery_requires_current_identity_support_and_final_station_commit(monkeypatch, mutation):
    node, clock, joint, params, marker, source = armed(monkeypatch)
    overrun(monkeypatch, node, clock, joint)
    before = node.last_good_source_stamp_ns; last = node.tracker.last
    if mutation == 'support':
        original = module.segment_with_support_fallback
        def bad(*args, **kwargs):
            result = original(*args, **kwargs)
            result.metrics['support_current_audit_inlier_ratio'] = .01
            return result
        monkeypatch.setattr(module, 'segment_with_support_fallback', bad)
    elif mutation == 'epoch_at_commit':
        original = node._station_compute_commit_check
        def bad(*args):
            node._rgb_interest_epoch += 1
            return original(*args)
        monkeypatch.setattr(node, '_station_compute_commit_check', bad)
    clock[0] += .2
    custom_frame(node, clock, joint, target_depth=.35, rgb_missing=mutation == 'rgb')
    assert node.last_good_source_stamp_ns == before and node.tracker.last is last
    assert not node.pubs['object'].publish.call_args[0][0].detected
    assert latest_status(node)['state'] != 'ready'


def test_clock_wait_rechecks_phase_after_context_before_pending_publication(monkeypatch):
    node, clock, joint, params, marker, source = armed(monkeypatch)
    original = node._rgb_interest_source_context
    def changed(stamp):
        context = original(stamp)
        if (module.rospy.Time.now().to_nsec()-stamp.to_nsec()) > 800_000_000:
            node._station_compute_state_cb(GraspState(state='MOVE_PREGRASP', active=True))
        return context
    monkeypatch.setattr(node, '_rgb_interest_source_context', changed)
    overrun(monkeypatch, node, clock, joint)
    assert latest_status(node)['state'] != 'processing_deadline_pending'
    assert node.first_loss is not None


def test_new_observation_lease_revokes_previous_compute_enrollment(monkeypatch):
    from test_observation_measurement_unknown import send, ack
    node, clock, joint, params, marker, source = armed(monkeypatch)
    request = deepcopy(node._observation_measurement_status['request'])
    for key in ('source_stamps_ns', 'settle'):
        request.pop(key, None)
    request.update(state='executing', lease_id='new_lease', request_id='new_execution',
        stamp_ns=module.rospy.Time.now().to_nsec(), arrival_stamp_ns=0, capture_deadline_ns=0,
        view_source_stamp_ns=node.last_good_source_stamp_ns)
    send(node, request)
    assert ack(node)['accepted'], ack(node)
    assert node._station_compute_registration is None
    assert node._locked_rgb_hold_reference is None


@pytest.mark.parametrize('kind', ['support', 'metric', 'manual'])
def test_failed_or_ambiguous_status_cannot_arm_even_with_old_fresh_reference(monkeypatch, kind):
    node, clock, joint, params, marker, source = armed(monkeypatch)
    if kind == 'support': node._locked_source_pending = {'state': 'pending'}
    elif kind == 'metric': node._metric_ambiguity_pending = {'state': 'pending'}
    else: params['/gui/joint_direct_mode'] = True
    node.status('ready', 'tracked', source, node.tracker.last)
    assert 'station_compute_wait' not in latest_status(node)
    assert node._station_compute_registration is None


def test_complete_ready_chain_survives_missed_intermediate_without_adding_failed_inputs(monkeypatch):
    from alicia_grasp_modes.station_compute_reacquisition import validate_recovery_parent
    node, clock, joint, params, marker, source = armed(monkeypatch)
    oldest = deepcopy(latest_status(node)['station_compute_wait'])
    clock[0] += .2
    intermediate = custom_frame(node, clock, joint, target_depth=.35)
    middle = deepcopy(latest_status(node)['station_compute_wait'])
    assert latest_status(node)['station_compute_recovery_chain'] == [oldest]
    clock[0] += .2
    current = custom_frame(node, clock, joint, target_depth=.35)
    status = latest_status(node)
    assert status['station_compute_recovery_chain'] == [oldest, middle]
    assert status['station_compute_recovery'] == middle
    validate_recovery_parent(oldest, parent=middle, chain=status['station_compute_recovery_chain'],
        binding=oldest['binding'], now_ns=module.rospy.Time.now().to_nsec(),
        source_ns=current[0].header.stamp.to_nsec())
    original_chain = deepcopy(status['station_compute_recovery_chain'])
    node.status('ready', 'tracked', current, node.tracker.last)
    assert latest_status(node)['station_compute_recovery_chain'] == original_chain
    current_contract = deepcopy(status['station_compute_wait'])
    overrun(monkeypatch, node, clock, joint)
    assert latest_status(node)['station_compute_wait'] == current_contract
    assert node._station_compute_registration['recovery_chain'] == original_chain
    pending_chain = latest_status(node)['station_compute_recovery_chain']
    assert pending_chain == original_chain+[current_contract]
    validate_recovery_parent(oldest, parent=current_contract, chain=pending_chain,
        binding=oldest['binding'], now_ns=module.rospy.Time.now().to_nsec())


def test_optional_bridge_proof_failure_does_not_change_timely_normal_ready(monkeypatch):
    node, clock, joint, params, marker, source = armed(monkeypatch)
    original = node._station_compute_registration['contract']
    def insufficient(*args):
        raise ValueError('station_compute_recovery_rgb_identity_required')
    monkeypatch.setattr(node, '_station_compute_measure_recovery', insufficient)
    clock[0] += .2
    current = custom_frame(node, clock, joint, target_depth=.35)
    status = latest_status(node)
    assert status['state'] == 'ready', status
    assert status['station_compute_wait']['source_ns'] == current[0].header.stamp.to_nsec()
    assert 'station_compute_recovery' not in status
    assert 'station_compute_recovery_chain' not in status
    assert node._station_compute_registration['recovery_chain'] == [original]
    # Identical missing proof is mandatory once our actual reference expires.
    clock[0] += .9
    custom_frame(node, clock, joint, target_depth=.35)
    assert latest_status(node)['state'] != 'ready'
    assert node.first_loss is not None
