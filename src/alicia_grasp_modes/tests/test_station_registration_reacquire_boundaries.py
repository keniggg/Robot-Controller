"""Offline scope boundaries after an actual failed station registration.

The source batch, station registration and surface registration are real.
Direct helper calls deliberately model preparation context for rejection
tests; the companion worker test exercises the actual context lifecycle.
"""
from dataclasses import replace

import numpy as np
import pytest

from test_observation_measurement_remote import context
from test_station_compute_wait_remote import station, clock
from test_station_compute_work_handoff import submitted
from test_remote_modes import base
from test_remote_grasp6d_streaming import measured_observation
from test_multiview_surface import rectangular_prism_views, transform_points_inverse


def failed_registration(c):
    n = c.node
    _start, _row, ticket = submitted(c)
    snapshot = ticket.payload[0]
    points, _ = rectangular_prism_views()
    initial = replace(measured_observation(n, points=points),
                      stamp_ns=snapshot.stamp_ns-300_000_000)
    reference = base.surface_view_from_observation(initial)
    surface = base.fused_surface_from_view(reference)
    second = replace(initial, stamp_ns=snapshot.stamp_ns-100_000_000)
    result, surface = base.append_registered_view(
        surface, reference, base.surface_view_from_observation(second))
    assert result.ok
    n._near_field_reference_view = reference
    n._near_field_fused_surface = surface
    n._near_field_surface_phase_id = n._near_field_phase_id
    n._near_field_surface_generation = n._stream_generation
    n.multiview_registration_config = base.RegistrationConfig(correspondence_max_m=.040)
    observation = replace(initial, stamp_ns=snapshot.stamp_ns,
        points_base=transform_points_inverse(points, yaw_deg=10.757952, translation=(0., 0., 0.)))
    n._latest_target_observation = observation
    result = n._register_near_field_surface(observation)
    assert not result.ok and result.code == 'YAW_BOUND_EXCEEDED', result.code
    assert n._near_field_fused_surface is surface
    n._observe_return_registration_ticket = ticket
    return snapshot, ticket, surface


def test_actual_yaw_failure_only_cancels_geometry_with_original_scope(station):
    c = station; n = c.node
    snapshot, ticket, surface = failed_registration(c)
    identity = n._current_stream_target_identity()
    deadline = n._near_field_phase_deadline_sec
    phase = n._near_field_phase_id
    generation = n._stream_generation
    evidence = n._latest_registration_evidence
    with pytest.raises(base.StreamResultCancelled) as error:
        n._observe_return_reacquire_failed_registration(snapshot)
    assert error.value.code == 'TARGET_PROCESSING_PENDING'
    assert n._latest_registration_evidence is evidence
    assert evidence.result.ok is False
    assert n._near_field_fused_surface is surface
    assert n._current_stream_target_identity() == identity
    assert n._near_field_phase_deadline_sec == deadline
    assert n._near_field_phase_id == phase
    assert n._stream_generation == generation
    assert not n.latest_object_geometry.valid
    assert not n.latest_preview_rich_plan.valid
    assert n.last_submitted_stamp_ns == ticket.payload[0].stamp_ns


@pytest.mark.parametrize('fault', [
    'owner', 'manual', 'epoch', 'phase_id', 'phase_inactive', 'phase_deadline',
    'session_deadline', 'no_station', 'retired', 'published', 'publishing',
    'valid_preview', 'initial_contact', 'observation_prefix', 'evidence_stamp',
    'evidence_identity', 'observation_stamp', 'other_code', 'accepted_result',
    'no_ticket', 'foreign_ticket', 'contact_execution',
])
def test_failed_registration_cannot_reacquire_outside_original_contract(station, fault):
    c = station; n = c.node
    snapshot, ticket, _surface = failed_registration(c)
    if fault == 'owner':
        c.params['/grasp_mode/selection'] = dict(n._mode_selection, generation=n._mode_selection['generation']+1)
    elif fault == 'manual': c.params['/gui/joint_direct_mode'] = True
    elif fault == 'epoch': n._measurement_remote_epoch_ns += 1
    elif fault == 'phase_id': n._station_compute_phase.phase_id += 1
    elif fault == 'phase_inactive': n._station_compute_phase.active = False
    elif fault == 'phase_deadline': clock(c, c.phase.deadline.to_nsec())
    elif fault == 'session_deadline': clock(c, n._observe_return_session['deadline_ns'])
    elif fault == 'no_station':
        n._station_compute_reference = None
        # Removing only the joined record leaves a valid complete triplet
        # which the original ingress can legitimately join again.
        n._station_compute_status.clear()
        n._station_compute_objects.clear()
        n._station_compute_masks.clear()
    elif fault == 'retired': n._station_compute_retired = True
    elif fault == 'published': n._station_compute_work_published = True
    elif fault == 'publishing': n._station_compute_publication_started = True
    elif fault == 'valid_preview': n.latest_preview_rich_plan.valid = True
    elif fault in ('initial_contact', 'observation_prefix'): n._observe_return_phase = fault
    elif fault == 'evidence_stamp':
        n._latest_registration_evidence = replace(n._latest_registration_evidence, stamp_ns=snapshot.stamp_ns-1)
    elif fault == 'evidence_identity':
        from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
        n._latest_registration_evidence = replace(n._latest_registration_evidence,
            identity=TargetTrackIdentity.from_stream(99, 99))
    elif fault == 'observation_stamp':
        n._latest_target_observation = replace(n._latest_target_observation, stamp_ns=snapshot.stamp_ns-1)
    elif fault == 'other_code':
        n._latest_registration_evidence = replace(n._latest_registration_evidence,
            result=replace(n._latest_registration_evidence.result, code='SUPPORT_OFFSET_MISMATCH'))
    elif fault == 'accepted_result':
        n._latest_registration_evidence = replace(n._latest_registration_evidence,
            result=replace(n._latest_registration_evidence.result, ok=True))
    elif fault == 'no_ticket': n._observe_return_registration_ticket = None
    elif fault == 'foreign_ticket':
        payload = (replace(snapshot),) + tuple(ticket.payload[1:])
        n._observe_return_registration_ticket = replace(ticket, payload=payload)
    elif fault == 'contact_execution': n._station_compute_task_state = 'MOVE_PREGRASP'
    deadline = n._near_field_phase_deadline_sec
    generation = n._stream_generation
    try:
        result = n._observe_return_reacquire_failed_registration(snapshot)
    except base.StreamResultCancelled:
        # Existing ticket gates may cancel with pending or harder loss.
        # This is not the new registration retry; the original worker release
        # must still reject the invalid owner/station/execution scope.
        n._stream_worker_busy = False
        with n._stream_condition:
            assert not n._release_preparation_cancelled_direct_snapshot_locked(
                ticket, base.StreamResultCancelled('TARGET_PROCESSING_PENDING'))
    else:
        assert result is False
    assert getattr(n, '_observe_return_registration_pending', None) is None
    # Original hard retirement can clear the deadline to zero; never renew it.
    assert n._near_field_phase_deadline_sec <= deadline
    assert n._stream_generation == generation


def test_actual_depth_failure_keeps_original_geometry_failure(station):
    c = station; n = c.node
    snapshot, _ticket, _surface = failed_registration(c)
    n.geometry_min_object_points = int(np.asarray(snapshot.depth_raw).size) + 1
    evidence = n._latest_registration_evidence
    estimate, _depth, _transform = n._prepare_snapshot_geometry(
        snapshot, base.rospy.Time(*divmod(snapshot.stamp_ns, 1_000_000_000)))
    assert estimate.ok is False
    assert estimate.failure_code == 'DEPTH_INSUFFICIENT'
    assert n._latest_registration_evidence is evidence
