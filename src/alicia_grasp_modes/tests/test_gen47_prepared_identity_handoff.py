"""Real Mode prepare/bind/source-accept ownership, with no hardware services.

G46 request2 metadata pins the observed failure. The RGBD, geometry/TF and empty
endpoint proposals below are declared synthetic boundary providers. Original
Mode routing, independent preparation, measured source construction, bind,
require and accept are used. No original bind/require is replaced by a success
stub, and reaching the existing no-endpoint gate is not a grasp/path proof.
"""
from copy import deepcopy
from dataclasses import replace
import hashlib
import json
from pathlib import Path
from types import MethodType, SimpleNamespace as NS

import numpy as np
import pytest

from test_remote_modes import configured, module, base
from test_silhouette_observation_view_source import view_context
from test_unknown_observation_view_source import generic_context
from test_silhouette_observation_planning import context as planning_context
from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample
from alicia_grasp_modes.silhouette_observation_planning import SilhouetteObservationPlanningMixin
from alicia_grasp_modes.observation_arrival_projection import capture_arrival_projection
from alicia_grasp_modes.unknown_pending_identity import reference_binding
from alicia_grasp_modes.observation_policy import WORKING_VOLUME_POLICY


FIXTURE = Path(__file__).parent/'fixtures/gen47_g46_request2_metadata.json'
FIXTURE_SHA = '2c6b3a884baa9449bc00cf4e0bc3e9391437f386cac6d600f193d57d20f1cb6c'
IDENTITY_ERROR = 'observation view request, camera, handeye or source points changed'


def _measured_context(configured, monkeypatch, tmp_path):
    # Unlike the legacy generic_context (which intentionally starts from a
    # silhouette trigger), this fixture uses the original measured preflight
    # to establish the missing-side evidence and independent measured route.
    node, prepared, _checks, _variant = planning_context(
        configured, monkeypatch, tmp_path, ordinary=True)
    assert node._observe_return_trigger == 'BILATERAL_SURFACE_EVIDENCE_MISSING'
    assert node._observe_return_view_gaps
    raw = np.full((240, 320), 300, np.uint16)
    raw[106:134, 145:175] = 280
    color = np.zeros((240, 320, 3), np.uint8)
    mask = np.zeros(raw.shape, np.uint8);mask[106:134, 145:175] = 255
    snapshot = replace(prepared.snapshot, color_bgr=color,
        depth_raw=np.full_like(raw, 900), target_depth_raw=raw,
        object_mask=mask, bbox=(145, 106, 30, 28))
    ticket = replace(prepared.ticket, payload=(snapshot, None))
    optical = np.eye(4)
    optical[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    geometry = replace(prepared.geometry, center_base=np.array([.28, 0., 0.]),
        axes_base=np.eye(3), size_xyz_m=np.array([.02, .02, .02]),
        support_normal_base=np.array([-1., 0., 0.]), support_offset_m=.3)
    prepared = replace(prepared, ticket=ticket, snapshot=snapshot,
        geometry=geometry, pose_estimator=NS(_T_base_optical=optical,
                                            T_base_camera_link=np.eye(4)))
    sample = RgbdSample(color.copy(), raw.copy(), mask.copy(), snapshot.bbox,
        deepcopy(snapshot.object_msg), snapshot.stamp_sec, snapshot.frame_id,
        np.zeros(6), snapshot.stamp_ns, snapshot.target_epoch, snapshot.target_identity)
    camera = NS(width=320, height=240, fx=240., fy=240., cx=160., cy=120., depth_scale=.001)
    monkeypatch.setattr(node, '_camera_intrinsics', lambda: camera)
    monkeypatch.setattr(node, '_tool_from_camera_matrix', lambda: np.eye(4))
    for name in ('_capture_diagnostic_observation_view_source',
                 '_bind_diagnostic_observation_view_source',
                 '_require_diagnostic_observation_view_source'):
        monkeypatch.setattr(node, name, MethodType(
            getattr(SilhouetteObservationPlanningMixin, name), node))
    ns = snapshot.stamp_ns;selection = node._mode_selection
    status = dict(mode='unknown', generation=selection['generation'],
        selection_stamp_ns=selection['stamp_ns'], source_stamp_ns=ns,
        state='ready', target_locked=True, target_lost=False, first_loss=None,
        exact_rgbd=True, projection_frame_convention='ros_camera_link',
        color_stamp_ns=ns, depth_stamp_ns=ns, tf_lookup_stamp_ns=ns,
        color_frame_id='camera_link', depth_frame_id='camera_link',
        anchor_source_stamp_ns=ns, anchor_position_base=[.28, 0., 0.],
        anchor_extent_m=[.02, .02, .02], reference_support_plane_base=[-1., 0., 0., .3],
        reference_object_height_m=.02,
        reference_support_footprint_base=[[.3, -.01, -.01], [.3, .01, -.01],
                                         [.3, .01, .01], [.3, -.01, .01]])
    configured[1]['/camera'] = vars(camera).copy()
    node._unknown_identity_status = {ns: status}
    node._unknown_identity_reference = {'binding': reference_binding(status, selection)}
    node._measurement_remote_reference = None
    arrival = capture_arrival_projection(status, selection, ns, configured[1]['/camera'])
    node._bind_diagnostic_observation_view_source(prepared, dict(sample=sample,
        intrinsics=camera, tool_from_camera=np.eye(4),
        view_limits=node._observation_view_limit_configuration(), arrival_projection=arrival))
    return node, prepared, sample, camera, None


def _case(configured, monkeypatch, tmp_path, route='measured_independent'):
    ordinary = route != 'silhouette_independent'
    factory = (_measured_context if route == 'measured_independent' else
               generic_context if ordinary else view_context)
    node, template, sample, camera, _fk = factory(configured, monkeypatch, tmp_path)
    # Same actual request number, with the fixture's explicitly synthetic
    # source coordinates/timestamps. We never relabel the historical pixels.
    ticket = replace(template.ticket, request_id=2)
    template = replace(template, ticket=ticket)
    state = NS(node=node, template=template, ticket=ticket, camera=camera,
        sample=sample, phase='prepare', binds=[], validations=[], captures=[],
        prediction_calls=[], endpoint_provider_calls=[], cancel_after_bind=False)
    if route == 'normal_view':
        # This is an existing non-independent unknown observation route.
        node._observe_return_trigger = 'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE'
    elif route == 'normal_none':
        node._observe_return_phase = 'initial_contact'
    assert node._independent_observation_planning_active() is (
        route in ('measured_independent', 'silhouette_independent'))
    assert node._unknown_observation_planning_active() is (route != 'normal_none')

    def exact_sample(*args, **kwargs):
        state.captures.append((args, kwargs))
        assert args == (template.snapshot.stamp_ns, template.snapshot.target_identity)
        return sample

    node.frames = NS(wait_for_exact_sample=exact_sample)
    # The original helper builds its real PreparedPrediction and source from
    # this already-qualified synthetic geometry/TF boundary. It does not call
    # a WSL or MoveIt service.
    monkeypatch.setattr(node, '_prepare_snapshot_geometry', lambda *_:
        (template.geometry, template.snapshot.target_depth_raw,
         np.array(template.pose_estimator._T_base_optical, copy=True)))
    monkeypatch.setattr(node, '_capture_silhouette_source', lambda _ticket: None)

    def predict(_node, actual_ticket):
        state.prediction_calls.append(actual_ticket)
        assert actual_ticket is state.ticket
        return state.template

    monkeypatch.setattr(base.RemoteGrasp6DNode, '_prepare_and_predict', predict)
    original_bind = node._bind_diagnostic_observation_view_source
    original_require = node._require_diagnostic_observation_view_source

    def require(prepared):
        try:
            source = original_require(prepared)
        except BaseException as error:
            state.validations.append((state.phase, prepared, False, type(error).__name__))
            raise
        state.validations.append((state.phase, prepared, True, source['binding']['request_id']))
        return source

    def bind(prepared, captured):
        result = original_bind(prepared, captured)
        assert node._diagnostic_observation_view_prepared is prepared
        state.binds.append(prepared)
        if state.cancel_after_bind:
            node._stream_generation += 1
        return result

    # These are call-through observations, not alternative authority checks.
    monkeypatch.setattr(node, '_require_diagnostic_observation_view_source', require)
    monkeypatch.setattr(node, '_bind_diagnostic_observation_view_source', bind)
    monkeypatch.setattr(node, '_far_field_observation_variants', MethodType(
        module.ModeAwareRemoteGrasp6DNode._far_field_observation_variants, node))
    node.grasp_config['observation_acceptance_policy'] = WORKING_VOLUME_POLICY

    def endpoint_provider(_node, *args, **kwargs):
        state.endpoint_provider_calls.append(args[-1] if args else kwargs.get('prepared'))
        return (), ()

    monkeypatch.setattr(base.RemoteGrasp6DNode, '_far_field_observation_variants', endpoint_provider)
    monkeypatch.setattr(node, '_joint_space_observation_candidates', lambda _prepared: ((), {}))
    return state


def _no_execution(state):
    assert not any(plan.valid for plan in state.node.rich_plan_pub.messages)
    assert not getattr(state.node, '_execution_authority_ticket', None)


def test_original_g46_request2_metadata_identifies_the_failed_handoff():
    assert hashlib.sha256(FIXTURE.read_bytes()).hexdigest() == FIXTURE_SHA
    record = json.loads(FIXTURE.read_text())
    assert record['gen46_mode_source_sha256'] == '504d45fcb21bae603113080b4ab7df9e3c33689ba149e805c92ec4687d798028'
    request = record['original_request']['metrics']
    assert request['request_id'] == 2 and request['generation'] == 3 and request['target_epoch'] == 4
    assert request['status'] == 'ACCEPT_FAILED' and request['error'] == IDENTITY_ERROR
    assert '_require_diagnostic_observation_view_source' in request['error_traceback']
    profile = request['request_preparation_profile']
    assert profile['ticket']['request_id'] == request['request_id']
    components = profile['scopes'][0]['components']
    assert any(row['name'] == 'diagnostic_observation' and row['completed'] is True
               for row in components)
    assert request['snapshot_evidence']['snapshot_stamp_ns'] == 1792311813511866807
    assert request['snapshot_evidence']['unique_source_frames'] == 5
    assert request['wsl_total_ms'] == 0.


def test_real_measured_independent_prepare_reaches_original_accept_source_gate(
        configured, monkeypatch, tmp_path):
    state = _case(configured, monkeypatch, tmp_path)
    prepared = state.node._prepare_and_predict(state.ticket)
    state.phase = 'accept'
    # This is the original consumer path that failed on G46. The declared
    # empty proposal provider then reaches the existing no-endpoint gate.
    result = state.node._accept_prediction(prepared)
    assert result['status'] == 'UNKNOWN_OBSERVATION_NO_SAFE_VIEW'
    assert prepared.request_preparation_profile is None
    assert prepared.ros_prepare_ms > 0.
    assert state.binds == [prepared]
    assert state.node._diagnostic_observation_view_prepared is prepared
    assert any(phase == 'accept' and actual is prepared and ok
               for phase, actual, ok, _ in state.validations)
    assert state.endpoint_provider_calls
    assert state.prediction_calls == []  # Independent source never enters WSL.
    assert len(state.captures) == 1
    _no_execution(state)


def test_silhouette_independent_helper_keeps_the_exact_original_bound_instance(
        configured, monkeypatch, tmp_path):
    state = _case(configured, monkeypatch, tmp_path, 'silhouette_independent')
    prepared = state.node._prepare_and_predict(state.ticket)
    source = state.node._require_diagnostic_observation_view_source(prepared)
    assert prepared.request_preparation_profile is None
    assert prepared.ros_prepare_ms > 0.
    assert state.binds == [prepared]
    assert state.node._diagnostic_observation_view_prepared is prepared
    assert source['binding']['request_id'] == state.ticket.request_id
    assert source['audit']['synthetic_support_points'] is False
    assert state.prediction_calls == [] and len(state.captures) == 1
    _no_execution(state)


def test_normal_observation_profiles_the_final_instance_before_original_bind(
        configured, monkeypatch, tmp_path):
    state = _case(configured, monkeypatch, tmp_path, 'normal_view')
    prepared = state.node._prepare_and_predict(state.ticket)
    source = state.node._require_diagnostic_observation_view_source(prepared)
    assert state.prediction_calls == [state.ticket]
    assert len(state.captures) == 1 and state.binds == [prepared]
    assert state.node._diagnostic_observation_view_prepared is prepared
    assert source['binding']['request_id'] == 2
    assert prepared.request_preparation_profile['ticket']['request_id'] == 2
    assert prepared.geometry is state.template.geometry
    _no_execution(state)


def test_unbound_normal_prepare_preserves_profile_without_transfer_of_old_owner(
        configured, monkeypatch, tmp_path):
    state = _case(configured, monkeypatch, tmp_path, 'normal_none')
    old_owner = state.node._diagnostic_observation_view_prepared
    prepared = state.node._prepare_and_predict(state.ticket)
    assert state.prediction_calls == [state.ticket]
    assert state.captures == [] and state.binds == []
    assert state.node._diagnostic_observation_view_prepared is old_owner
    assert prepared.request_preparation_profile['ticket']['request_id'] == 2
    assert prepared.geometry is state.template.geometry
    _no_execution(state)


@pytest.mark.parametrize('late_input', ['equal_copy', 'earlier_prepared_after_new_owner'])
def test_duplicate_or_delayed_accept_cannot_transfer_the_bound_instance(
        configured, monkeypatch, tmp_path, late_input):
    state = _case(configured, monkeypatch, tmp_path)
    prepared = state.node._prepare_and_predict(state.ticket)
    assert state.node._diagnostic_observation_view_prepared is prepared
    if late_input == 'equal_copy':
        rejected = replace(prepared)
        assert rejected is not prepared and rejected.ticket is prepared.ticket
    else:
        rejected = prepared
        state.ticket = replace(state.ticket, request_id=3)
        state.template = replace(state.template, ticket=state.ticket)
        prepared = state.node._prepare_and_predict(state.ticket)
        assert state.node._diagnostic_observation_view_prepared is prepared
    state.phase = 'delayed_accept'
    with pytest.raises(ValueError, match=IDENTITY_ERROR):
        state.node._accept_prediction(rejected)
    assert state.node._diagnostic_observation_view_prepared is prepared
    assert any(phase == 'delayed_accept' and actual is rejected and not ok
               for phase, actual, ok, _ in state.validations)
    _no_execution(state)


@pytest.mark.parametrize('fault,expected', [
    ('generation_before', 'GENERATION_STALE'),
    ('target_before', 'TARGET_EPOCH_STALE'),
    ('generation_after_bind', 'GENERATION_STALE')])
def test_original_typed_cancellation_cannot_acquire_or_reassign_owner(
        configured, monkeypatch, tmp_path, fault, expected):
    state = _case(configured, monkeypatch, tmp_path)
    old_owner = state.node._diagnostic_observation_view_prepared
    if fault == 'generation_before':
        state.node._stream_generation += 1
    elif fault == 'target_before':
        state.node.target_instance_epoch += 1
    else:
        state.cancel_after_bind = True
    with pytest.raises(base.StreamResultCancelled) as error:
        prepared = state.node._prepare_and_predict(state.ticket)
        state.phase = 'cancelled_accept'
        state.node._accept_prediction(prepared)
    assert error.value.code == expected
    if fault != 'generation_after_bind':
        profile = getattr(error.value, 'request_preparation_profile')
        assert profile['ticket'] == dict(request_id=state.ticket.request_id,
            generation=state.ticket.generation, target_epoch=state.ticket.target_epoch,
            snapshot_stamp_sec=state.ticket.snapshot_stamp_sec)
        assert state.binds == [] and state.node._diagnostic_observation_view_prepared is old_owner
    else:
        assert len(state.binds) == 1
        assert state.node._diagnostic_observation_view_prepared is state.binds[0]
    _no_execution(state)


@pytest.mark.parametrize('changed_input', ['camera', 'handeye'])
def test_original_live_calibration_gate_remains_strict_after_identity_handoff(
        configured, monkeypatch, tmp_path, changed_input):
    state = _case(configured, monkeypatch, tmp_path)
    prepared = state.node._prepare_and_predict(state.ticket)
    assert state.node._diagnostic_observation_view_prepared is prepared
    if changed_input == 'camera':
        state.camera.fx += 1.
    else:
        changed = np.eye(4);changed[0, 3] = .001
        monkeypatch.setattr(state.node, '_tool_from_camera_matrix', lambda: changed)
    with pytest.raises(ValueError, match=IDENTITY_ERROR):
        state.node._accept_prediction(prepared)
    _no_execution(state)


def test_normal_prediction_cancellation_precedes_original_bind(
        configured, monkeypatch, tmp_path):
    state = _case(configured, monkeypatch, tmp_path, 'normal_view')
    old_owner = state.node._diagnostic_observation_view_prepared

    def cancelled_response(_node, _ticket):
        state.node._stream_generation += 1
        return state.template

    monkeypatch.setattr(base.RemoteGrasp6DNode, '_prepare_and_predict', cancelled_response)
    with pytest.raises(base.StreamResultCancelled) as error:
        state.node._prepare_and_predict(state.ticket)
    assert error.value.code == 'GENERATION_STALE'
    profile = getattr(error.value, 'request_preparation_profile')
    assert profile['ticket'] == dict(request_id=state.ticket.request_id,
        generation=state.ticket.generation, target_epoch=state.ticket.target_epoch,
        snapshot_stamp_sec=state.ticket.snapshot_stamp_sec)
    assert state.binds == [] and state.node._diagnostic_observation_view_prepared is old_owner
    _no_execution(state)
