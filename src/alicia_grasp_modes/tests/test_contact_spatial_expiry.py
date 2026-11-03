"""A finite crop hint may expire without declaring the measured target lost.

These process tests use the existing synthetic RGB-D station fixture and the
real producer/refinement/commit chain. The recorded 20261024 failure supplies
the ordering (retire, refine across phase expiry, commit), not substitute pixels.
"""
from unittest.mock import Mock

import pytest

from test_contact_station_retirement import producer_context, retirement, send
from test_locked_rgb_hold import module, latest_status
from test_observation_measurement_unknown import next_frame


def retiring_crop(monkeypatch, *, phase_seconds=.3, session_at_phase=False):
    c = producer_context(monkeypatch)
    c.expires_at = c.clock[0] + phase_seconds
    c.phase.deadline = module.rospy.Time.from_sec(c.expires_at)
    if session_at_phase:
        c.marker['session_deadline_ns'] = c.phase.deadline.to_nsec()
    assert send(c.node, c.req)['accepted']
    assert send(c.node, retirement(c.req))['accepted']
    c.context = c.node._contact_spatial_context
    assert c.context is not None
    return c


@pytest.mark.parametrize('after_deadline_sec', [-.000001, 0., .000001, .14414549])
def test_phase_expiry_during_refinement_discards_frame_without_first_loss(
        monkeypatch, after_deadline_sec):
    c = retiring_crop(monkeypatch)
    original = module.refine_candidate
    used = []
    def crossing(*args, **kwargs):
        used.append(kwargs.get('contact_spatial_reference'))
        result = original(*args, **kwargs)
        c.clock[0] = c.expires_at + after_deadline_sec
        return result
    monkeypatch.setattr(module, 'refine_candidate', crossing)
    anchor, last = c.node.tracker.anchor, c.node.tracker.last
    reference = c.node._locked_rgb_hold_reference
    before = {key:pub.publish.call_count for key,pub in c.node.pubs.items()}
    source = c.node.last_good_source_stamp_ns
    deadline = c.phase.deadline.to_nsec()
    session_deadline = c.context['deadline_monotonic']
    next_frame(c.node, c.clock, c.joint, q=[.006, 0., 0., 0., 0., 0.])
    assert used == [c.context['spatial_reference']]
    assert c.node.first_loss is None and not c.node.tracker.lost
    assert c.node.tracker.anchor is anchor
    assert c.phase.deadline.to_nsec() == deadline
    assert c.context['deadline_monotonic'] == session_deadline
    if after_deadline_sec < 0.:
        assert c.node.last_good_source_stamp_ns > source
        assert c.node._contact_spatial_context is c.context
        assert latest_status(c.node)['state'] == 'ready'
        return
    assert c.node._contact_spatial_context is None
    assert c.node.tracker.last is last
    assert c.node._locked_rgb_hold_reference is reference
    assert c.node.last_good_source_stamp_ns == source
    assert {key:pub.publish.call_count for key,pub in c.node.pubs.items()} == before
    # The expired frame cannot be republished, and the next fresh RGB-D frame
    # must pass the ordinary geometry path with no stationary/crop privilege.
    used.clear()
    def fresh(*args, **kwargs):
        used.append(kwargs)
        return original(*args, **kwargs)
    monkeypatch.setattr(module, 'refine_candidate', fresh)
    pair = next_frame(c.node, c.clock, c.joint, q=[.012, 0., 0., 0., 0., 0.])
    assert latest_status(c.node)['state'] == 'ready', latest_status(c.node)
    assert c.node.last_good_source_stamp_ns == pair[0].header.stamp.to_nsec() > source
    assert used and all('contact_spatial_reference' not in kw for kw in used)
    assert all(not kw.get('observation_arrival_retry', False) for kw in used)
    assert c.node._contact_spatial_context is None
    assert 'observation_arrival_retry' not in (c.node._locked_rgb_hold_reference or {})
    assert c.node.first_loss is None and not c.node.tracker.lost


def test_crop_already_expired_before_new_frame_uses_current_rgbd(monkeypatch):
    c = retiring_crop(monkeypatch, phase_seconds=.1)
    original = module.refine_candidate
    capture = Mock(side_effect=original)
    monkeypatch.setattr(module, 'refine_candidate', capture)
    pair = next_frame(c.node, c.clock, c.joint, q=[.006, 0., 0., 0., 0., 0.])
    assert capture.call_count == 1
    assert 'contact_spatial_reference' not in capture.call_args.kwargs
    assert latest_status(c.node)['state'] == 'ready'
    assert c.node.last_good_source_stamp_ns == pair[0].header.stamp.to_nsec()
    assert c.node._contact_spatial_context is None
    assert c.node.first_loss is None and not c.node.tracker.lost


@pytest.mark.parametrize('fault', ['manual', 'epoch', 'selection', 'track',
                                  'stop_callback', 'lost', 'camera',
                                  'session_ros', 'session_monotonic'])
def test_phase_expiry_never_masks_real_owner_or_session_revocation(monkeypatch, fault):
    from types import SimpleNamespace as NS
    import alicia_grasp_modes.contact_station_producer as contract
    c = retiring_crop(monkeypatch, session_at_phase=fault == 'session_ros')
    original = module.refine_candidate
    calls = []
    def revoke(*args, **kwargs):
        calls.append(kwargs.get('contact_spatial_reference'))
        result = original(*args, **kwargs)
        c.clock[0] = c.expires_at
        if fault == 'manual': c.params['/gui/joint_direct_mode'] = True
        elif fault == 'epoch': c.node._rgb_interest_epoch += 1
        elif fault == 'selection': c.params['/grasp_mode/selection']['generation'] += 1
        elif fault == 'track': c.phase.reference_target_track_id = 'another-target'
        elif fault == 'stop_callback':
            c.node._station_compute_state_cb(NS(state='STOP', active=False))
        elif fault == 'lost': c.node.tracker.lost = True
        elif fault == 'camera': c.params['/camera']['fx'] += 1
        elif fault == 'session_monotonic':
            c.context['deadline_monotonic'] = contract.time.monotonic() - 1.
        return result
    monkeypatch.setattr(module, 'refine_candidate', revoke)
    source = c.node.last_good_source_stamp_ns
    next_frame(c.node, c.clock, c.joint, q=[.006, 0., 0., 0., 0., 0.])
    assert calls == [c.context['spatial_reference']]
    assert c.node.last_good_source_stamp_ns == source
    assert c.node._contact_spatial_context is None
    assert not c.node.pubs['object'].publish.call_args.args[0].detected
    assert c.node.first_loss is not None and c.node.tracker.lost
    assert 'contact_spatial_context_revoked_before_commit' in latest_status(c.node)['detail']


def refinement_crop(monkeypatch):
    from copy import deepcopy
    import test_grasp_task_sequence as sequence
    from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest
    from alicia_flexible_grasp_supervisor.msg import GraspState, NearFieldPlanningPhase
    from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages
    c = producer_context(monkeypatch)
    helper = sequence.GraspTaskSequenceTest()
    plan = helper._mark_valid_3d(helper._rich_plan(stamp_sec=c.req['plan_source_ns'] * 1e-9))
    stamp = module.rospy.Time(c.req['plan_source_ns']//10**9, c.req['plan_source_ns']%10**9)
    plan.header.stamp = plan.object_geometry.header.stamp = stamp
    plan.model_choice = 'unknown_tabletop'
    plan.diagnostic = 'CONTACT_EXECUTION_PLAN'
    plan.target_track_id = plan.object_geometry.target_track_id = c.req['target_track_id']
    plan.plan_id = sequence.compute_plan_id(plan)
    c.req.update(plan_id=plan.plan_id, plan_sha256=committed_plan_digest(plan))
    assert send(c.node, c.req)['accepted']
    assert send(c.node, retirement(c.req))['accepted']
    c.context = c.node._contact_spatial_context
    def messages():
        state = GraspState()
        state.header.stamp = module.rospy.Time.now()
        state.active, state.stage, state.state = True, GraspStages.PLAN_PREGRASP, 'PLAN_PREGRASP'
        state.message = 'waiting for bounded final visual refinement'
        phase = NearFieldPlanningPhase()
        phase.header.stamp, phase.header.frame_id = state.header.stamp, plan.header.frame_id
        phase.active, phase.phase_id = True, c.req['phase_id'] + 1
        phase.planning_stage = 'final_refinement'
        phase.deadline = phase.header.stamp + module.rospy.Duration(20.)
        phase.reference_center_valid = True
        phase.reference_target_track_id = c.req['target_track_id']
        phase.reference_source_stamp = state.header.stamp
        phase.reference_contact_plan = deepcopy(plan)
        # Use the actual published-phase contract, independently of the new
        # crop-ending predicate. No geometry is granted by this test fixture.
        from alicia_flexible_grasp.grasp.refinement_continuity import binding_from_phase
        assert binding_from_phase(phase, {}).plan_id == c.req['plan_id']
        return state, phase
    c.messages = messages
    return c


@pytest.mark.parametrize('order', ['state', 'phase', 'state_phase', 'phase_state'])
@pytest.mark.parametrize('checkpoint', ['refine', 'hold'])
def test_normal_refinement_transition_discards_only_inflight_crop(monkeypatch, order, checkpoint):
    c = refinement_crop(monkeypatch)
    target, name = ((module, 'refine_candidate') if checkpoint == 'refine' else
                    (c.node, '_prepare_locked_rgb_hold_reference'))
    original = getattr(target, name)
    delivered = []
    def transition(*args, **kwargs):
        result = original(*args, **kwargs)
        state, phase = c.messages()
        for channel in order.split('_'):
            delivered.append(channel)
            if channel == 'state': c.node._station_compute_state_cb(state)
            else: c.node._station_compute_phase_cb(phase)
        return result
    monkeypatch.setattr(target, name, transition)
    anchor, last, reference = c.node.tracker.anchor, c.node.tracker.last, c.node._locked_rgb_hold_reference
    source = c.node.last_good_source_stamp_ns
    before = {key:pub.publish.call_count for key,pub in c.node.pubs.items()}
    deadline, monotonic_deadline = c.context['deadline_ns'], c.context['deadline_monotonic']
    next_frame(c.node, c.clock, c.joint, q=[.006, 0., 0., 0., 0., 0.])
    assert delivered == order.split('_')
    assert c.node.first_loss is None and not c.node.tracker.lost
    assert c.node.tracker.anchor is anchor and c.node.tracker.last is last
    assert c.node._locked_rgb_hold_reference is reference
    assert c.node.last_good_source_stamp_ns == source
    assert {key:pub.publish.call_count for key,pub in c.node.pubs.items()} == before
    assert (c.context['deadline_ns'], c.context['deadline_monotonic']) == (deadline, monotonic_deadline)
    assert c.node._contact_spatial_context is None
    monkeypatch.setattr(target, name, original)
    pair = next_frame(c.node, c.clock, c.joint, q=[.012, 0., 0., 0., 0., 0.])
    assert latest_status(c.node)['state'] == 'ready', latest_status(c.node)
    assert c.node.last_good_source_stamp_ns == pair[0].header.stamp.to_nsec() > source
    assert c.node.first_loss is None and not c.node.tracker.lost


@pytest.mark.parametrize('fault', ['epoch', 'manual_observed', 'selection_observed',
                                  'target_phase', 'stop'])
def test_normal_end_then_observed_revocation_cannot_be_restored(monkeypatch, fault):
    from copy import deepcopy
    from types import SimpleNamespace as NS
    from std_msgs.msg import Header
    c = refinement_crop(monkeypatch)
    source_capture = []
    check = c.node._contact_spatial_reference
    def capture(source, camera, expected=None):
        result = check(source, camera, expected)
        if expected is None and result is not None: source_capture.append((source, camera))
        return result
    monkeypatch.setattr(c.node, '_contact_spatial_reference', capture)
    original = module.refine_candidate
    def revoke(*args, **kwargs):
        result = original(*args, **kwargs)
        state, phase = c.messages()
        c.node._station_compute_state_cb(state)
        if fault == 'epoch':
            old = c.node._rgb_interest_epoch
            c.node._rgb_interest_epoch_cb(Header(frame_id='sdk_reference_epoch',
                stamp=module.rospy.Time(old//10**9, old%10**9+1)))
            c.node._rgb_interest_epoch_cb(Header(frame_id='sdk_reference_epoch',
                stamp=module.rospy.Time(old//10**9, old%10**9)))
        elif fault in ('manual_observed', 'selection_observed'):
            key = '/gui/joint_direct_mode' if fault == 'manual_observed' else '/grasp_mode/selection'
            old = deepcopy(c.params.get(key, False))
            c.params[key] = True if fault == 'manual_observed' else dict(old, generation=old['generation']+1)
            with pytest.raises(ValueError, match='revoked_before_commit'):
                check(*source_capture[-1], expected=c.context)
            c.params[key] = old
        elif fault == 'target_phase':
            wrong = deepcopy(phase); wrong.reference_target_track_id = 'other-target'
            c.node._station_compute_phase_cb(wrong)
            c.node._station_compute_phase_cb(phase)
        else:
            c.node._station_compute_state_cb(NS(state='STOP', active=False))
            c.node._station_compute_state_cb(state)
        return result
    monkeypatch.setattr(module, 'refine_candidate', revoke)
    source = c.node.last_good_source_stamp_ns
    next_frame(c.node, c.clock, c.joint, q=[.006, 0., 0., 0., 0., 0.])
    assert source_capture
    assert c.node.last_good_source_stamp_ns == source
    assert c.node._contact_spatial_context is None
    assert c.node.first_loss is not None and c.node.tracker.lost
    assert not c.node.pubs['object'].publish.call_args.args[0].detected


@pytest.mark.parametrize('fault', ['state_enum', 'inactive', 'wrong_label', 'wrong_plan', 'wrong_track'])
def test_lookalike_refinement_transition_is_not_normal_crop_completion(monkeypatch, fault):
    c = refinement_crop(monkeypatch)
    original = module.refine_candidate
    def wrong_transition(*args, **kwargs):
        result = original(*args, **kwargs)
        state, phase = c.messages()
        if fault == 'state_enum': state.stage = 3
        elif fault == 'inactive': state.active = False
        elif fault == 'wrong_label': state.message = 'planning another contact plan'
        elif fault == 'wrong_plan': phase.reference_contact_plan.poses[0].position.x += .01
        else: phase.reference_target_track_id = 'other-target'
        if fault in ('wrong_plan', 'wrong_track'): c.node._station_compute_phase_cb(phase)
        else: c.node._station_compute_state_cb(state)
        return result
    monkeypatch.setattr(module, 'refine_candidate', wrong_transition)
    source = c.node.last_good_source_stamp_ns
    next_frame(c.node, c.clock, c.joint, q=[.006, 0., 0., 0., 0., 0.])
    assert c.node.last_good_source_stamp_ns == source
    assert c.node._contact_spatial_context is None
    assert c.node.first_loss is not None and c.node.tracker.lost
