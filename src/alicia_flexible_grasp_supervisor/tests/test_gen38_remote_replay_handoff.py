"""Actual remote diagnostic handoffs with inert inputs, never ROS clients.

Registration, enclosure, coordinate transforms, storage and phase cancellation
run production code. Only RGB-D estimation/TF and the endpoint service response
are explicit offline adapters. Saved diagnostics never authorize execution.
"""
from copy import deepcopy
import hashlib
import itertools
import json
from pathlib import Path
import threading
import sys

import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'tools/tests'))
from storage_os_adapter import storage_os, tmp_path

from alicia_flexible_grasp.robot import contact_replay_receipt as receipt_module
from alicia_flexible_grasp.robot.observation_path_guard import FrozenObservationScene
from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface, SurfaceView
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from test_contact_precheck_phase_cancellation import context as phase_context
from test_fused_scene_pipeline import (
    node, current_top_geometry, exact_snapshot, run_prepare,
)
from test_gen38_contact_replay_receipt import inputs
from test_remote_grasp6d_streaming import remote_node as remote


W = Path(__file__).resolve().parents[3]
SCRIPT = W/'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py'


def restore_surface(data):
    """Rebuild the actual immutable production dataclasses, including provenance."""
    identity = TargetTrackIdentity(**data['identity'])
    surface = FusedTargetSurface(identity=identity,
        points_base=np.asarray(data['points_base']),
        view_indices=np.asarray(data['view_indices'], dtype=np.int64),
        view_stamps_ns=tuple(data['view_stamps_ns']),
        view_sources=tuple(tuple(row) for row in data['view_sources']))
    reference = SurfaceView(identity=identity, **{
        key: value for key, value in data['reference'].items() if key != 'identity'})
    return surface, reference


def assert_exact_surface(restored, actual):
    surface, reference = restored; expected, expected_reference = actual
    assert surface.identity == expected.identity == reference.identity
    np.testing.assert_array_equal(surface.points_base, expected.points_base)
    np.testing.assert_array_equal(surface.view_indices, expected.view_indices)
    assert surface.view_stamps_ns == expected.view_stamps_ns
    assert surface.view_sources == expected.view_sources
    assert reference.stamp_ns == expected_reference.stamp_ns
    np.testing.assert_array_equal(reference.points_base, expected_reference.points_base)
    np.testing.assert_array_equal(reference.support_normal_base, expected_reference.support_normal_base)
    assert reference.support_offset_m == expected_reference.support_offset_m
    assert reference.edge_clearance_px == expected_reference.edge_clearance_px


@pytest.mark.parametrize('yaw,displacement', [
    (0., (0., 0., 0.)),
    (np.deg2rad(3.), (.003, -.002, .001)),
])
def test_production_surface_receipts_restore_both_exact_coordinate_frames(
        node, tmp_path, monkeypatch, yaw, displacement):
    assert Path(remote.__file__).resolve() == SCRIPT
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'ros'))
    original = current_top_geometry(yaw, displacement)
    (geometry, depth, transform), snapshot = run_prepare(node, monkeypatch, original)
    assert geometry.ok
    audit = node._latest_fused_scene_audit
    receipt = audit['surface_replay']
    assert receipt['status'] == 'stored'
    assert receipt['diagnostic_only'] is True and receipt['execution_authorized'] is False
    assert set(receipt['references']) == {'snapshot', 'reference'}
    restored = {}
    for frame, reference in receipt['references'].items():
        data = remote.DiagnosticStore(reference['store_root']).get(reference)
        restored[frame] = data
        assert data['snapshot_stamp_ns'] == snapshot.stamp_ns
        assert data['execution_authorized'] is False and data['future_following_verified'] is False
        binding = data['request_binding']
        assert binding['source_stamp_ns'] == snapshot.stamp_ns
        assert binding['generation'] == node._stream_generation
        assert binding['phase_id'] == node._near_field_phase_id
        assert binding['target_track_id'] == snapshot.target_identity.track_id
        assert binding['object_points_sha256'] == audit['enclosed_points_sha256']
        assert 'task_run_id' not in binding  # not carried by this producer API
    assert restored['snapshot']['coordinate_frame'] == 'current_snapshot_base_estimate'
    assert restored['reference']['coordinate_frame'] == 'phase_reference_base_estimate'
    assert_exact_surface(restore_surface(restored['snapshot']), node._snapshot_multiview_surface())
    assert_exact_surface(restore_surface(restored['reference']), node._active_multiview_surface())
    correction = np.asarray(restored['snapshot']['snapshot_to_reference'])
    np.testing.assert_array_equal(correction, node._latest_registration_evidence.result.transform_base)
    np.testing.assert_allclose(np.asarray(restored['snapshot']['points_base']) @ correction[:3, :3].T
        + correction[:3, 3], restored['reference']['points_base'], atol=1e-12, rtol=0.)
    if yaw:
        assert not np.array_equal(restored['snapshot']['points_base'], restored['reference']['points_base'])
    # The external diagnostic does not replace the actual source/union scene.
    surface, _ = node._snapshot_multiview_surface()
    expected, _ = remote.enclose_registered_surface(original, surface,
        expected_identity=snapshot.target_identity)
    np.testing.assert_array_equal(geometry.object_points_base, expected.object_points_base)
    np.testing.assert_array_equal(geometry.center_base, expected.center_base)
    np.testing.assert_array_equal(depth, snapshot.target_depth_raw)
    np.testing.assert_array_equal(transform, np.eye(4))


@pytest.mark.parametrize('changed', ['phase', 'generation', 'audit_owner'])
def test_surface_io_releases_locks_and_old_receipt_cannot_replace_new_cache(
        node, tmp_path, monkeypatch, changed):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'ros'))
    original_store = remote._store_contact_replay
    old_lifecycle = node._multiview_lifecycle_token()
    captured = []; old_owners = []; new_audit = {'new_source': 'sentinel',
        'surface_replay': {'new_receipt': 'sentinel'}}
    completed = threading.Event(); errors = []

    def callback_during_io():
        try:
            with node._stream_condition, node._geometry_state_guard():
                if changed == 'phase': node._near_field_phase_id += 1
                if changed == 'generation': node._stream_generation += 1
                node._latest_fused_scene_audit = new_audit
        except BaseException as exc:
            errors.append(exc)
        finally:
            completed.set()

    def store(data):
        if not captured:
            old_owners.append(node._latest_fused_scene_audit)
            callback = threading.Thread(target=callback_during_io)
            callback.start()
            assert completed.wait(2.), 'surface storage retained a stream/geometry lock'
            callback.join(2.)
            assert not callback.is_alive() and not errors
        reference = original_store(data)
        captured.append((deepcopy(data), reference))
        return reference

    monkeypatch.setattr(remote, '_store_contact_replay', store)
    (geometry, _, _), snapshot = run_prepare(node, monkeypatch,
        current_top_geometry(np.deg2rad(3.), (.003, -.002, .001)))
    assert geometry.ok and len(captured) == 2
    assert node._latest_fused_scene_audit is new_audit
    assert new_audit == {'new_source': 'sentinel', 'surface_replay': {'new_receipt': 'sentinel'}}
    assert 'surface_replay' not in old_owners[0]
    for data, reference in captured:
        assert data['request_binding']['generation'] == old_lifecycle[0]
        assert data['request_binding']['phase_id'] == old_lifecycle[1]
        assert data['snapshot_stamp_ns'] == snapshot.stamp_ns
        assert remote.DiagnosticStore(reference['store_root']).get(reference) == data


@pytest.mark.parametrize('storage', ['unset', 'system_root', 'missing_mount'])
def test_surface_storage_unavailable_preserves_geometry_without_local_fallback(
        node, tmp_path, monkeypatch, storage, storage_os):
    forbidden = W/'.runtime/gen38_forbidden_remote_replay_root'
    assert not forbidden.exists()
    if storage == 'unset': monkeypatch.delenv('ROS_LOG_DIR', raising=False)
    elif storage == 'system_root': monkeypatch.setenv('ROS_LOG_DIR', str(forbidden))
    else:
        monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'ros'))
        storage_os.states['ssd']['mounted'] = False
    original = current_top_geometry(np.deg2rad(3.), (.003, -.002, .001))
    (geometry, depth, transform), snapshot = run_prepare(node, monkeypatch, original)
    assert geometry.ok and node._latest_registration_evidence.result.ok
    receipt = node._latest_fused_scene_audit['surface_replay']
    assert receipt['status'] == 'unavailable' and receipt['references'] == {}
    assert receipt['diagnostic_only'] and receipt['execution_authorized'] is False
    assert receipt['reason'].startswith('ReplayStorageUnavailable: ')
    surface, _ = node._snapshot_multiview_surface()
    expected, _ = remote.enclose_registered_surface(original, surface,
        expected_identity=snapshot.target_identity)
    for name in ('object_points_base', 'center_base', 'axes_base', 'size_xyz_m'):
        np.testing.assert_array_equal(getattr(geometry, name), getattr(expected, name))
    np.testing.assert_array_equal(depth, snapshot.target_depth_raw)
    np.testing.assert_array_equal(transform, np.eye(4))
    assert not forbidden.exists() and not (tmp_path/'ros/contact_replay_receipts').exists()


def failure_reply(plan, curve_reference):
    reason = 'nominal continuous geometry: target OBB collision at 8-end/right_finger'
    return dict(scope='read_only_candidate_constant_bias_prediction', plan_id=plan.plan_id,
        execution_authorized=False, success=False,
        failure_code='CONTACT_SEQUENCE_PRECHECK_FAILED', failed_stage='grasp', reason=reason,
        contact_failure_replay=dict(status='stored', diagnostic_only=True,
            execution_authorized=False, stage='grasp', curve='nominal', reference=curve_reference))


@pytest.mark.parametrize('changed', ['phase', 'generation'])
@pytest.mark.parametrize('oversized', [False, True], ids=['small', 'oversized'])
def test_late_service_reply_retains_frozen_historical_join_without_current_evidence(
        tmp_path, monkeypatch, changed, oversized):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'ros'))
    node, prepared, plan, clock, calls, _, proxy, response = phase_context(monkeypatch)
    try:
        # The inert response points to a genuinely stored pure-data curve
        # fixture. It is a diagnostic message adapter, not a planning verdict.
        data = inputs(); scene = FrozenObservationScene.from_contact_plan(plan)
        data['scene'] = scene
        data['binding'].update(plan_id=plan.plan_id, plan_sha256=scene.digest,
            source_stamp_ns=scene.source_ns, target_track_id=scene.track_id)
        curve_reference = remote._store_contact_replay(receipt_module.contact_failure_data(**data))
        evidence = failure_reply(plan, curve_reference)
        if oversized:
            evidence['large_diagnostic_detail'] = 'x'*(1024*1024)
        raw = json.dumps(evidence, sort_keys=True)
        response.success = False; response.failure_code = evidence['failure_code']
        response.failed_stage = 'grasp'; response.message = evidence['reason']; response.evidence_json = raw
        old = dict(plan_id=plan.plan_id, source_stamp_ns=plan.header.stamp.to_nsec(),
            plan_wire_sha256=remote.wire_digest(plan), request_id=prepared.ticket.request_id,
            generation=prepared.ticket.generation, phase_id=prepared.near_field_phase_id)
        mono = itertools.count(1_000_000, 100); wall = itertools.count(1_900_000_000_000_000_000, 100)
        monkeypatch.setattr(remote.time, 'monotonic_ns', lambda: next(mono))
        monkeypatch.setattr(remote.time, 'time_ns', lambda: next(wall))
        events = []
        monkeypatch.setattr(remote.rospy, 'loginfo', lambda fmt, payload: events.append((fmt, json.loads(payload))))
        original_phase_check = node._require_current_contact_precheck_phase
        checks = []

        def phase_check(*args):
            checks.append(True)
            if len(checks) == 3:
                # A later callback changes the response object after delivery
                # capture but before the historical log is emitted.
                response.message = 'later mutation'
                response.evidence_json = '{"later_mutation":true}'
            return original_phase_check(*args)

        node._require_current_contact_precheck_phase = phase_check

        def service(request):
            calls.append(request)
            if changed == 'phase': node._near_field_phase_id += 1
            else: node._stream_generation += 1
            prepared.ticket.request_id += 100
            plan.plan_id = 'later-plan'
            plan.header.stamp = remote.rospy.Time(999)
            clock.value = 101.
            return response

        proxy.return_value = service
        runtime = {'current_plan_sentinel': {'unchanged': True}}
        with pytest.raises(remote.StreamResultCancelled):
            node._final_contact_sequence_evaluation(plan, (), prepared, runtime,
                'late_handoff_test', deadline_sec=190.)
        assert runtime == {'current_plan_sentinel': {'unchanged': True}}
        assert len(calls) == 1 and len(checks) == 3 and len(events) == 1
        fmt, event = events[0]
        assert fmt == 'remote 6D contact precheck delivery: %s'
        assert event['consumer_outcome'] == 'ticket_or_phase_cancelled'
        assert event['diagnostic_only'] and event['execution_authorized'] is False
        assert event['plan_id'] == old['plan_id'] and event['phase_id'] == old['phase_id']
        assert event['plan_wire_sha256'] == old['plan_wire_sha256']
        assert event['request']['request_id'] == old['request_id']
        assert event['request']['generation'] == old['generation']
        assert event['source_stamp_ns'] == old['source_stamp_ns']
        assert event['source_stamp_ns'] != event['response']['received_wall_ns']
        assert event['request_monotonic_ns'] < event['response']['received_monotonic_ns'] < event['recorded_monotonic_ns']
        assert event['request_wall_ns'] < event['response']['received_wall_ns'] < event['recorded_wall_ns']
        if oversized:
            # The bounded log may omit large unrelated text, but must retain
            # the small immutable curve join even when the plan is cancelled.
            assert event.get('precheck', {}).get('plan_id') == old['plan_id']
            assert event['precheck']['contact_failure_replay'] == evidence['contact_failure_replay']
            assert len(json.dumps(event)) < len(raw)
        else:
            assert event['precheck'] == evidence
        assert event['response']['message'] == evidence['reason']
        assert event['response']['evidence_json_sha256'] == hashlib.sha256(raw.encode()).hexdigest()
        assert event['response']['evidence_json_bytes'] == len(raw.encode())
        assert event['precheck']['contact_failure_replay']['reference'] == curve_reference
        saved = deepcopy(event)
        prepared.ticket.generation += 100; calls[0].plan.header.stamp = remote.rospy.Time(123)
        response.evidence_json = '{}'; response.failure_code = 'later'
        assert event == saved  # request/response mutation cannot rewrite history
    finally:
        node.shutdown_streaming_worker()


def test_historical_log_failure_does_not_replace_original_phase_cancellation(monkeypatch):
    node, prepared, plan, _, _, _, proxy, response = phase_context(monkeypatch)
    try:
        response.failed_stage = ''
        def service(_):
            node._near_field_phase_id += 1
            return response
        def broken_log(*_):
            raise OSError('injected diagnostic log failure')
        proxy.return_value = service
        monkeypatch.setattr(remote.rospy, 'loginfo', broken_log)
        runtime = {}
        with pytest.raises(remote.StreamResultCancelled, match='NEAR_FIELD_PHASE_STALE'):
            node._final_contact_sequence_evaluation(plan, (), prepared, runtime,
                'late_log_failure_test', deadline_sec=190.)
        assert not runtime
    finally:
        node.shutdown_streaming_worker()
