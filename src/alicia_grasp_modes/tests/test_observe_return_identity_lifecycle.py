"""Identity retirement cannot keep another target's missing-view authority."""
from copy import deepcopy
import types
import threading

import pytest

from test_remote_modes import configured, module, base, direct_observation
from test_observe_return_remote import setup_observe_return, deficit, prefix_session, start_session, send


def enter_prefix(configured):
    node, params, clock = setup_observe_return(configured)
    deadline = node._near_field_phase_deadline_sec
    identity = node._current_stream_target_identity()
    assert node._observe_return_fallback(types.SimpleNamespace(remote_diagnostics=deficit()))
    node._observe_return_view_gaps = (dict(target_epoch=identity.epoch,
        target_track_id=identity.track_id, source_view_stamps_ns=[11_000_000_000]),)
    node._observe_return_view_uncertainty_m = .003
    node._observe_return_directed_views = ('old_rotation',)
    return node, params, clock, identity, deadline


def test_unpublished_prefix_restarts_actual_initial_search_with_new_source_and_old_deadline(configured):
    node, params, clock, identity, deadline = enter_prefix(configured)
    selection = deepcopy(node._mode_selection)
    old_surface = node._observe_return_saved[0]
    old_ticket = types.SimpleNamespace(generation=node._stream_generation,
                                     target_epoch=identity.epoch)
    assert node._advance_target_instance_epoch('TARGET_LOST')
    assert node._mode_selection == selection
    assert node._current_stream_target_identity() != identity
    assert node._stream_ticket_cancellation_code_locked(old_ticket) == 'TARGET_EPOCH_STALE'
    assert node._observe_return_phase == 'initial_contact'
    assert node.near_field_planning_active and not node._observe_return_paused
    assert node._near_field_phase_deadline_sec == deadline
    assert node._near_field_phase_started_sec == clock[0] / 1e9
    assert node._observe_return_saved is None
    assert node._observe_return_view_gaps == ()
    assert node._observe_return_directed_views == ()
    assert node._observe_return_initial_stamp == 0
    assert node._active_multiview_surface()[0] is None
    with pytest.raises(base.StreamResultCancelled, match='TARGET_EPOCH_STALE'):
        node._observe_return_fallback(types.SimpleNamespace(
            ticket=old_ticket, remote_diagnostics=deficit()))
    # A stale fallback cannot merely relabel the old retained surface.
    with pytest.raises(ValueError, match='MEASURED_SURFACE_BINDING_LOST'):
        node._observe_return_fallback(types.SimpleNamespace(remote_diagnostics=deficit()))
    clock[0] += 1_000_000_000
    fresh = direct_observation(node, clock[0])
    node._latest_target_observation = fresh
    node._register_near_field_surface(fresh)
    assert node._active_multiview_surface()[0].identity == fresh.identity
    assert node._active_multiview_surface()[0].identity != old_surface.identity
    assert node._active_multiview_surface()[0].view_stamps_ns == (clock[0],)
    # The normal structured failure path must run again for this new source.
    assert node._observe_return_fallback(types.SimpleNamespace(remote_diagnostics=deficit()))
    assert node._observe_return_saved[0].view_stamps_ns == (clock[0],)
    assert node._observe_return_initial_contact_deadline_ns == int(deadline * 1e9)


def test_repeated_identity_changes_do_not_renew_contact_budget(configured):
    node, _, clock, _, deadline = enter_prefix(configured)
    for _ in range(2):
        node._advance_target_instance_epoch('TARGET_INSTANCE_CHANGED')
        assert node._near_field_phase_deadline_sec == deadline
        clock[0] += 2_000_000_000
        fresh = direct_observation(node, clock[0])
        node._latest_target_observation = fresh
        node._register_near_field_surface(fresh)
        assert node._observe_return_fallback(types.SimpleNamespace(remote_diagnostics=deficit()))
    clock[0] = int((deadline + 1.) * 1e9)
    node._advance_target_instance_epoch('TARGET_LOST')
    assert node._near_field_phase_deadline_sec == deadline
    assert node._near_field_phase_deadline_sec < node._near_field_phase_started_sec


@pytest.mark.parametrize('reserved', ['published_prefix', 'authority_ticket',
                                     'controller_reservation', 'active_execution', 'preview'])
def test_reserved_or_published_observation_terminates_instead_of_restarting(configured, reserved):
    node, _, _, _, deadline = enter_prefix(configured)
    if reserved == 'published_prefix':
        prefix_session(node)
    elif reserved == 'authority_ticket':
        node._execution_authority_ticket = dict(request_id=123)
    elif reserved == 'controller_reservation':
        node.execution_plan_controller = types.SimpleNamespace(has_execution=True)
    elif reserved == 'active_execution':
        node.robot_execution_active = True
    else:
        node.latest_preview_rich_plan = types.SimpleNamespace(valid=True)
    node._advance_target_instance_epoch('TARGET_LOST')
    assert node._observe_return_phase == 'identity_lost'
    assert node._observe_return_paused and not node.near_field_planning_active
    assert node._observe_return_saved is None
    assert node._observe_return_view_gaps == ()
    assert node._observe_return_prefix is None
    assert node._observe_return_initial_contact_deadline_ns == int(deadline * 1e9)
    assert node._observe_return_identity_transition['action'] == 'terminate_observation'
    assert node._poll_stream_snapshot() is False
    assert node.latest_preview_rich_plan.valid is False
    assert node.latest_preview_rich_plan.diagnostic.startswith('OBSERVE_RETURN_TARGET_CHANGED:')
    assert node.latest_preview_rich_plan.candidate_source == 'near_field_terminal'
    count = len(node.preview_rich_plan_pub.messages)
    assert node._poll_stream_snapshot() is False
    assert len(node.preview_rich_plan_pub.messages) == count


def test_active_multiview_session_cannot_transfer_surfaces_or_extend_deadline(configured):
    node, _, _, session, initial = start_session(configured)
    original_deadline = session['deadline_ns']
    node._observe_return_pending_supplement = dict(deadline_ns=original_deadline)
    node._observe_return_step_gain = dict(ok=True)
    node._advance_target_instance_epoch('TARGET_INSTANCE_CHANGED')
    assert node._observe_return_phase == 'identity_lost'
    assert node._observe_return_session is None
    assert node._observe_return_pending_supplement is None
    assert node._observe_return_step_gain is None
    assert node._observe_return_saved is None
    assert node._active_multiview_surface()[0] is None
    assert node._observe_return_identity_terminal is not None
    # No replacement session/deadline or new view is issued.
    assert session['deadline_ns'] == original_deadline
    assert node._current_stream_target_identity() != initial.identity


def test_silhouette_sources_and_collision_geometry_are_retired_together(configured):
    node, _, _, identity, _ = enter_prefix(configured)
    node._observe_return_diagnostic_observation_only = True
    node._observe_return_diagnostic_attempted = True
    node._observe_return_silhouette_evidence = dict(target_epoch=identity.epoch)
    node._observe_return_diagnostic_collision_geometry = object()
    node._observe_return_diagnostic_collision_source = dict(target_epoch=identity.epoch)
    node._silhouette_observation_envelope_audit = dict(ok=True)
    node._advance_target_instance_epoch('TARGET_LOST')
    assert node._observe_return_phase == 'initial_contact'
    for name in ('_observe_return_silhouette_evidence',
                 '_observe_return_diagnostic_collision_geometry',
                 '_observe_return_diagnostic_collision_source',
                 '_silhouette_observation_envelope_audit'):
        assert getattr(node, name) is None
    assert node._observe_return_diagnostic_observation_only is False


def test_identity_preserving_phase_change_keeps_original_evidence(configured):
    node, _, _, identity, _ = enter_prefix(configured)
    saved, gaps = node._observe_return_saved, node._observe_return_view_gaps
    node._advance_target_instance_epoch('OBSERVE_RETURN_PHASE_CHANGED', preserve_identity=True)
    assert node._current_stream_target_identity() == identity
    assert node._observe_return_phase == 'observation_prefix'
    assert node._observe_return_saved is saved and node._observe_return_view_gaps is gaps
    assert node._observe_return_identity_transition is None


def test_identity_retired_while_surface_is_saved_cannot_be_resurrected(configured, monkeypatch):
    node, _, clock, identity, _ = enter_prefix(configured)
    session = prefix_session(node)
    clock[0] = 15_000_000_000
    save = node._observe_return_save_surface

    def retire_after_save():
        saved = save()
        node._advance_target_instance_epoch('TARGET_LOST')
        return saved

    monkeypatch.setattr(node, '_observe_return_save_surface', retire_after_save)
    ack = send(node, session)
    assert ack['accepted'] is False and ack['reason'] == 'OBSERVE_RETURN_TARGET_CHANGED'
    assert node._current_stream_target_identity() != identity
    assert node._observe_return_phase == 'identity_lost'
    assert node._observe_return_session is None
    assert node._observe_return_saved is None
    assert node._active_multiview_surface()[0] is None


@pytest.mark.parametrize('interleave', ['surface_saved', 'phase_advanced'])
def test_identity_callback_cannot_split_phase_commit_and_ack_revalidates(configured, monkeypatch, interleave):
    node, _, clock, identity, _ = enter_prefix(configured)
    session = prefix_session(node)
    clock[0] = 15_000_000_000
    callback_started = threading.Event()
    callback_finished = threading.Event()
    workers = []

    def retire_identity():
        callback_started.set()
        node._advance_target_instance_epoch('TARGET_LOST')
        callback_finished.set()

    def contend_after(operation):
        def wrapped(*args, **kwargs):
            result = operation(*args, **kwargs)
            worker = threading.Thread(target=retire_identity, daemon=True)
            workers.append(worker)
            worker.start()
            assert callback_started.wait(1.)
            # The real input thread must wait for the complete phase/session
            # commit; saving alone or only locking the restore is insufficient.
            assert not callback_finished.wait(.03)
            return result
        return wrapped

    method = ('_observe_return_save_surface' if interleave == 'surface_saved'
              else '_observe_return_advance')
    monkeypatch.setattr(node, method, contend_after(getattr(node, method)))
    publish = node._observe_return_publish_session_ack

    def retire_before_ack(ack, committed):
        # The phase transaction has released its locks here, as it must to
        # avoid input/work lock inversion. Force retirement before delivery.
        assert callback_finished.wait(1.)
        publish(ack, committed)

    monkeypatch.setattr(node, '_observe_return_publish_session_ack', retire_before_ack)
    ack = send(node, session)
    for worker in workers:
        worker.join(timeout=1.)
        assert not worker.is_alive()
    assert ack['accepted'] is False and ack['reason'] == 'OBSERVE_RETURN_TARGET_CHANGED'
    assert node._current_stream_target_identity() != identity
    assert node._observe_return_phase == 'identity_lost'
    assert node._observe_return_session is None
    assert node._observe_return_saved is None
    assert node._active_multiview_surface()[0] is None


def test_selection_change_before_ack_does_not_acknowledge_old_session(configured, monkeypatch):
    node, params, clock, _, _ = enter_prefix(configured)
    session = prefix_session(node)
    clock[0] = 15_000_000_000
    publish = node._observe_return_publish_session_ack

    def change_selection_before_ack(ack, committed):
        params[module.SELECTION_PARAM] = dict(params[module.SELECTION_PARAM],
            generation=int(params[module.SELECTION_PARAM]['generation']) + 1)
        publish(ack, committed)

    monkeypatch.setattr(node, '_observe_return_publish_session_ack', change_selection_before_ack)
    ack = send(node, session)
    assert ack['accepted'] is False and ack['reason'] == 'OBSERVE_RETURN_SELECTION_CHANGED'
