"""RGB acquisition ordering/budget tests; no ROS or invented collision verdicts."""
from dataclasses import FrozenInstanceError, asdict, replace
import json

import pytest

from alicia_grasp_modes.rgb_acquisition_session import (
    AcquisitionContext, InterestSource, ObservationPreview, SubmissionEvidence,
    MeasuredTargetEvidence, RgbAcquisitionOwner)
from alicia_grasp_modes.selection import parse_selection


S = 1_000_000_000
H = 'a'*64


def context(**changes):
    values = dict(source_id='camera-profile-1', selection_generation=2,
        selection_stamp_ns=S, camera_geometry_sha256=H, handeye_sha256=H,
        robot_description_sha256=H, control_epoch_ns=2*S, perception_policy_sha256=H)
    values.update(changes)
    return AcquisitionContext(**values)


def interest(**changes):
    values = dict(context=context(), candidate_id='rgb-interest-1',
        source_stamps_ns=(9*S+400_000_000, 9*S+600_000_000, 9*S+800_000_000),
        evidence_sha256=H, rgb_sha256=H, region_mask_sha256=H,
        base_camera_sha256=H, feedback_sha256=H,
        image_shape=(480,640), bbox_xywh=(300,220,40,40))
    values.update(changes)
    return InterestSource(**values)


def live(now=10*S, **changes):
    values = dict(current_context=context(), now_ns=now,
        monotonic_ns=100*S+now-10*S, control_available=True)
    values.update(changes)
    return values


def start(*, duration=20*S, limit=3, item=None):
    owner = RgbAcquisitionOwner()
    owner.start(session_id='session-1', interest=item or interest(),
        duration_ns=duration, max_observations=limit, now_ns=10*S,
        monotonic_ns=100*S, control_available=True)
    return owner


def preview(item=None, name='preview-1'):
    return ObservationPreview(name, item or interest(), H, H)


def submit(owner, item=None, name='preview-1', now=10*S):
    p = preview(item, name)
    owner.preview(p, **live(now))
    evidence = SubmissionEvidence(p.binding_sha256, H, H, H, H)
    return owner.reserve_submission(evidence, **live(now))


def finish_and_settle(owner, reservation, completed=10*S+400_000_000,
                      settled=10*S+600_000_000):
    owner.record_execution_completed(submission_id=reservation.submission_id,
        trajectory_sha256=H, completed_stamp_ns=completed,
        gateway_receipt_sha256='b'*64, **live(completed))
    owner.record_settled(submission_id=reservation.submission_id,
        settled_source_stamp_ns=settled, measured_settle_record_sha256='c'*64,
        **live(settled))


def target(**changes):
    values = dict(context=context(), candidate_id='rgb-interest-1',
        target_track_id='real-measured-target-1', target_epoch=7,
        source_stamps_ns=(10*S+700_000_000,10*S+900_000_000,11*S+100_000_000),
        perception_acceptance_sha256=H, measured_geometry_sha256=H,
        measured_points_sha256=H, support_evidence_sha256=H, rgb_association_sha256=H)
    values.update(changes)
    return MeasuredTargetEvidence(**values)


def test_successful_bookkeeping_still_requires_gateway_and_standard_contact_checks():
    owner = start()
    assert owner.snapshot().phase == 'interest'
    assert not hasattr(owner.snapshot(), 'target_track_id')
    assert not interest().measured_target_identity
    reservation = submit(owner)
    assert reservation.requires_independent_gateway_admission
    assert not reservation.motion_authorized and not reservation.contact_authorized
    assert not reservation.certifies_scene_or_path
    assert owner.snapshot().consumed_submissions == 1
    finish_and_settle(owner, reservation)
    handoff = owner.handoff_measured_target(target(), **live(11*S+200_000_000))
    assert handoff.requires_standard_grasp_validation
    assert not handoff.carries_rgb_synthetic_geometry
    assert not handoff.motion_authorized and not handoff.contact_authorized
    assert handoff.submission_id == reservation.submission_id
    assert handoff.gateway_receipt_sha256 == 'b'*64
    assert handoff.measured_settle_record_sha256 == 'c'*64
    assert handoff.settled_source_stamp_ns == 10*S+600_000_000
    assert owner.snapshot().phase == 'handed_off'
    json.dumps(asdict(handoff), allow_nan=False)
    with pytest.raises(ValueError, match='OWNER_NOT_ACTIVE'):
        owner.handoff_measured_target(target(), **live(11*S+200_000_000))


def test_replacing_many_previews_cannot_consume_motion_or_extend_budget():
    owner = start()
    initial = owner.snapshot()
    for index in range(100):
        snapshot = owner.preview(preview(name='preview-'+str(index)), **live())
        assert snapshot.consumed_submissions == 0
        assert not snapshot.motion_authorized
        assert (snapshot.deadline_ns, snapshot.monotonic_deadline_ns) == (
            initial.deadline_ns, initial.monotonic_deadline_ns)
    with pytest.raises(FrozenInstanceError):
        snapshot.deadline_ns += S


@pytest.mark.parametrize('new_candidate', [False, True])
def test_same_selection_restart_cannot_renew_after_revoke_or_timeout(new_candidate):
    owner = start(duration=S)
    owner.revoke('operator_pause')
    original = owner.snapshot()
    item = interest(candidate_id='another' if new_candidate else 'rgb-interest-1',
        source_stamps_ns=(12*S,12*S+100_000_000,12*S+200_000_000))
    with pytest.raises(ValueError, match='EPISODE_ALREADY_EXISTS'):
        owner.start(session_id='different-request', interest=item, duration_ns=100*S,
            max_observations=3, now_ns=12*S+300_000_000,
            monotonic_ns=102*S+300_000_000, control_available=True)
    assert owner.snapshot() == original


def test_new_operator_selection_requires_prior_revocation_and_new_session():
    owner = start()
    next_context = context(selection_generation=3, selection_stamp_ns=11*S)
    item = interest(context=next_context, source_stamps_ns=(12*S,12*S+100_000_000,12*S+200_000_000))
    kwargs = dict(interest=item,duration_ns=20*S,max_observations=3,
                  now_ns=12*S+300_000_000,monotonic_ns=102*S+300_000_000,control_available=True)
    with pytest.raises(ValueError, match='REVOKE_PREVIOUS_OWNER_FIRST'):
        owner.start(session_id='session-2', **kwargs)
    owner.revoke('selection_changed')
    with pytest.raises(ValueError, match='NEW_SESSION_ID_REQUIRED'):
        owner.start(session_id='session-1', **kwargs)
    snapshot = owner.start(session_id='session-2', **kwargs)
    assert snapshot.context == next_context and snapshot.consumed_submissions == 0


@pytest.mark.parametrize('changed', [
    dict(source_id='different-camera'), dict(selection_generation=3),
    dict(selection_stamp_ns=S+1), dict(control_epoch_ns=2*S+1),
    dict(camera_geometry_sha256='b'*64), dict(handeye_sha256='b'*64),
    dict(robot_description_sha256='b'*64), dict(perception_policy_sha256='b'*64)])
def test_live_context_changes_revoke_existing_owner(changed):
    owner = start()
    with pytest.raises(ValueError, match='OWNER_REVOKED_CONTEXT_CHANGED'):
        owner.preview(preview(), **live(current_context=context(**changed)))
    assert owner.snapshot().phase == 'revoked'
    with pytest.raises(ValueError, match='OWNER_NOT_ACTIVE'):
        owner.preview(preview(), **live())


@pytest.mark.parametrize('changes,reason', [
    (dict(control_available=False),'CONTROL_OWNER_CHANGED'),
    (dict(now_ns=10*S-1),'CLOCK_ROLLBACK'),
    (dict(monotonic_ns=100*S-1),'CLOCK_ROLLBACK')])
def test_manual_takeover_or_clock_rollback_invalidates_owner(changes, reason):
    owner = start()
    with pytest.raises(ValueError, match=reason):
        owner.preview(preview(), **live(**changes))
    assert owner.snapshot().phase == 'revoked'


@pytest.mark.parametrize('changes', [dict(now_ns=11*S),dict(monotonic_ns=101*S)])
def test_either_shared_deadline_expires_even_if_other_clock_freezes(changes):
    owner = start(duration=S)
    with pytest.raises(ValueError, match='SHARED_DEADLINE_EXPIRED'):
        owner.preview(preview(), **live(**changes))
    assert owner.snapshot().phase == 'expired'


def test_newer_current_view_refresh_retires_preview_without_extending_deadline():
    owner = start()
    owner.preview(preview(), **live())
    initial = owner.snapshot()
    item = interest(source_stamps_ns=(9*S+500_000_000,9*S+700_000_000,9*S+900_000_000))
    current = owner.refresh_interest(item, **live())
    assert current.phase == 'interest' and current.preview_id == ''
    assert current.deadline_ns == initial.deadline_ns
    with pytest.raises(ValueError, match='PREVIEW_SOURCE_MISMATCH'):
        owner.preview(preview(), **live())


def test_trajectory_proof_for_different_preview_cannot_reserve():
    owner = start()
    owner.preview(preview(), **live())
    wrong = SubmissionEvidence(preview(name='different').binding_sha256,H,H,H,H)
    with pytest.raises(ValueError, match='PATH_PREVIEW_BINDING_MISMATCH'):
        owner.reserve_submission(wrong, **live())
    assert owner.snapshot().consumed_submissions == 0


def test_preview_source_must_still_be_fresh_at_submission():
    owner = start()
    p = preview()
    owner.preview(p, **live())
    with pytest.raises(ValueError, match='SOURCE_STALE_OR_FUTURE'):
        owner.reserve_submission(SubmissionEvidence(p.binding_sha256,H,H,H,H),
            **live(10*S+600_000_001))
    assert owner.snapshot().consumed_submissions == 0


def test_submission_is_once_only_and_rejection_does_not_refund_or_allow_replay():
    owner = start(limit=2)
    reservation = submit(owner)
    with pytest.raises(ValueError, match='WRONG_PHASE'):
        owner.reserve_submission(reservation.evidence, **live())
    owner.record_submission_rejected(submission_id=reservation.submission_id,
        rejection_receipt_sha256=H, **live())
    assert owner.snapshot().consumed_submissions == 1
    with pytest.raises(ValueError, match='PREVIEW_ALREADY_CONSUMED'):
        owner.preview(preview(), **live())
    second = submit(owner, name='preview-2')
    owner.record_submission_rejected(submission_id=second.submission_id,
        rejection_receipt_sha256=H, **live())
    assert owner.snapshot().phase == 'exhausted'
    assert owner.snapshot().consumed_submissions == 2


@pytest.mark.parametrize('field,value', [('submission_id','unrelated'),
    ('trajectory_sha256','b'*64),('completed_stamp_ns',10*S),
    ('completed_stamp_ns',11*S),('gateway_receipt_sha256','not-a-digest')])
def test_execution_receipt_must_bind_reserved_trajectory_and_valid_time(field,value):
    owner = start()
    reservation = submit(owner)
    values = dict(submission_id=reservation.submission_id,trajectory_sha256=H,
        completed_stamp_ns=10*S+400_000_000,gateway_receipt_sha256=H)
    values[field] = value
    with pytest.raises(ValueError):
        owner.record_execution_completed(**values, **live(10*S+400_000_000))
    assert owner.snapshot().phase == 'submission_reserved'


def test_handoff_requires_execution_then_actual_measured_settle_receipt():
    owner = start()
    reservation = submit(owner)
    with pytest.raises(ValueError, match='WRONG_PHASE'):
        owner.record_settled(submission_id=reservation.submission_id,
            settled_source_stamp_ns=10*S+100_000_000,measured_settle_record_sha256=H,
            **live(10*S+100_000_000))
    owner.record_execution_completed(submission_id=reservation.submission_id,
        trajectory_sha256=H,completed_stamp_ns=10*S+400_000_000,
        gateway_receipt_sha256=H,**live(10*S+400_000_000))
    with pytest.raises(ValueError, match='WRONG_PHASE'):
        owner.handoff_measured_target(target(), **live(11*S+200_000_000))


@pytest.mark.parametrize('sources', [
    (10*S+500_000_000,10*S+900_000_000,11*S+100_000_000),
    (10*S+600_000_000,10*S+900_000_000,11*S+100_000_000),
    (12*S,12*S+100_000_000,12*S+200_000_000)])
def test_pre_settle_or_future_measurements_do_not_become_handoff(sources):
    owner = start()
    reservation = submit(owner)
    finish_and_settle(owner,reservation)
    with pytest.raises(ValueError):
        owner.handoff_measured_target(target(source_stamps_ns=sources), **live(11*S+200_000_000))
    assert owner.snapshot().phase == 'awaiting_measured_target'


def test_supplement_keeps_one_shared_budget_and_requires_same_source_visual_deficit():
    owner = start(limit=2)
    initial = owner.snapshot()
    first = submit(owner)
    finish_and_settle(owner,first)
    item = interest(source_stamps_ns=target().source_stamps_ns, association_sha256=H)
    kwargs = dict(deficit_code='no_center_instance',deficit_source_stamp_ns=item.source_stamp_ns,
        deficit_record_sha256=H,**live(11*S+200_000_000))
    owner.request_supplement(item, **kwargs)
    assert owner.snapshot().deadline_ns == initial.deadline_ns
    assert owner.snapshot().monotonic_deadline_ns == initial.monotonic_deadline_ns
    assert owner.snapshot().consumed_submissions == 1
    second = submit(owner,item,name='preview-2',now=11*S+200_000_000)
    finish_and_settle(owner,second,completed=11*S+400_000_000,settled=11*S+600_000_000)
    last = interest(source_stamps_ns=(11*S+700_000_000,11*S+900_000_000,12*S+100_000_000),association_sha256=H)
    with pytest.raises(ValueError, match='OBSERVATION_BUDGET_EXHAUSTED'):
        owner.request_supplement(last, deficit_code='no_center_instance',
            deficit_source_stamp_ns=last.source_stamp_ns,deficit_record_sha256=H,
            **live(12*S+200_000_000))
    assert owner.snapshot().phase == 'exhausted'


def test_last_allowed_observation_can_still_handoff():
    owner = start(limit=1)
    reservation = submit(owner)
    finish_and_settle(owner,reservation)
    assert owner.snapshot().phase == 'awaiting_measured_target'
    owner.handoff_measured_target(target(), **live(11*S+200_000_000))
    assert owner.snapshot().phase == 'handed_off'


@pytest.mark.parametrize('code', ['missing_exact_tf','collision','ik_failure','stale_frame'])
def test_nonvisual_failures_do_not_request_more_views(code):
    owner = start()
    reservation = submit(owner)
    finish_and_settle(owner,reservation)
    item = interest(source_stamps_ns=target().source_stamps_ns, association_sha256=H)
    with pytest.raises(ValueError, match='CURRENT_VISUAL_DEFICIT_REQUIRED'):
        owner.request_supplement(item,deficit_code=code,
            deficit_source_stamp_ns=item.source_stamp_ns,deficit_record_sha256=H,
            **live(11*S+200_000_000))


def test_supplement_cannot_claim_identity_without_rgb_association_record():
    owner = start()
    reservation = submit(owner)
    finish_and_settle(owner,reservation)
    item = interest(source_stamps_ns=target().source_stamps_ns)
    with pytest.raises(ValueError, match='FRESH_ASSOCIATED_VIEW_REQUIRED'):
        owner.request_supplement(item,deficit_code='no_center_instance',
            deficit_source_stamp_ns=item.source_stamp_ns,deficit_record_sha256=H,
            **live(11*S+200_000_000))


@pytest.mark.parametrize('changes', [dict(source_stamps_ns=(4,4,5)),
    dict(source_stamps_ns=(4,5)),dict(source_stamps_ns=[4,5,6]),
    dict(region_mask_sha256='fake'),dict(image_shape=(False,640)),
    dict(bbox_xywh=(-1,0,1,1)),dict(bbox_xywh=(630,1,30,1))])
def test_malformed_rgb_source_or_pixel_binding_rejected(changes):
    with pytest.raises(ValueError):
        interest(**changes)


@pytest.mark.parametrize('changes', [dict(selection_generation=True),
    dict(control_epoch_ns=0),dict(strategy='two_stage'),
    dict(mode='carton'),dict(handeye_sha256='F'*64)])
def test_context_is_exact_and_supported(changes):
    with pytest.raises(ValueError):
        context(**changes)


@pytest.mark.parametrize('generation', [-1, 0, 1])
def test_generation_boundary_matches_actual_mode_selection_contract(generation):
    selected = dict(mode='unknown', strategy='observe_return',
                    generation=generation, stamp_ns=S)
    if generation < 1:
        with pytest.raises(ValueError):
            parse_selection(selected)
        with pytest.raises(ValueError):
            context(selection_generation=generation)
    else:
        accepted = parse_selection(selected)
        assert context(selection_generation=generation).selection_key == (
            accepted['generation'], accepted['stamp_ns'])


def test_rgb_interest_id_is_not_a_real_target_id():
    with pytest.raises(ValueError, match='INTEREST_ID_IS_NOT_TARGET_ID'):
        target(target_track_id='rgb-interest-1')


@pytest.mark.parametrize('duration,limit', [(601*S,3),(0,3),(S,0),(S,4),(S,True)])
def test_total_budget_must_be_explicit_and_bounded(duration,limit):
    with pytest.raises(ValueError):
        start(duration=duration,limit=limit)
