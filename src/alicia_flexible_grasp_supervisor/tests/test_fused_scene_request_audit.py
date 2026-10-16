"""Enclosure evidence stays with the request across remote wait and activation.

No ROS workers or services run. Registration/enclosure, request preparation,
cache clearing, message activation and audit extraction are production code.
"""
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest

import test_remote_grasp6d_node as processing
import test_remote_grasp6d_streaming as streaming
from test_fused_scene_pipeline import current_top_geometry, exact_snapshot

remote = streaming.remote_node


@pytest.fixture
def prepared_case(monkeypatch):
    node = remote.RemoteGrasp6DNode.__new__(remote.RemoteGrasp6DNode)
    node.__dict__.update(processing.make_processing_node().__dict__)
    node._initialize_streaming_state(
        result_max_age_sec=1.2, performance_window_size=100,
        source_clock=streaming.MutableClock(20.2), start_worker=False)
    streaming.seed_measured_reference_surface(node)
    source = exact_snapshot(node)
    source.object_msg.detected = True
    original = current_top_geometry()
    monkeypatch.setattr(remote, 'estimate_object_geometry', lambda **_: original)
    monkeypatch.setattr(node, '_snapshot_base_optical_transform', lambda *_: np.eye(4))
    monkeypatch.setattr(node, '_camera_intrinsics', lambda: SimpleNamespace(depth_scale=.001))
    monkeypatch.setattr(node, '_require_graspnet_input_prerequisites', lambda *_: None)
    monkeypatch.setattr(node, '_require_stream_ticket_current', lambda *_: None)
    monkeypatch.setattr(node, '_require_stream_ticket_current_locked', lambda *_: None)
    monkeypatch.setattr(node, '_gripper_contract_mismatch_reason', lambda: '')
    monkeypatch.setattr(node, '_generate_tabletop_candidates', lambda *_a, **_k: ((), {}))
    monkeypatch.setattr(node, '_build_frozen_graspnet_input',
                        lambda *_a, **_k: (SimpleNamespace(), {}))
    captured = {}

    def remote_wait(*_args, **_kwargs):
        # Happens after the request's geometry has been fully prepared. Mimic
        # a reset/new request replacing the global cache while inference waits.
        captured['scene'] = deepcopy(node._latest_fused_scene_audit)
        node._clear_geometry_cache()
        node._latest_fused_scene_audit = {'unrelated_request': True}
        return (), {}, {}, 0., 0., 0.

    monkeypatch.setattr(node, '_predict_remote', remote_wait)
    ticket = SimpleNamespace(payload=(source, processing.frozen_input_config(node)))
    prepared = remote.RemoteGrasp6DNode._prepare_and_predict(node, ticket)
    yield node, prepared, captured
    node.shutdown_streaming_worker()


def test_real_prepare_remote_overwrite_activate_and_audit(prepared_case):
    node, prepared, captured = prepared_case
    expected = captured['scene']
    assert expected and expected['snapshot_stamp_ns'] == prepared.snapshot.stamp_ns
    assert expected['enclosed_point_count'] == len(prepared.geometry.object_points_base)
    assert remote._thaw_request_evidence(prepared.fused_scene_audit) == expected
    assert node._latest_fused_scene_audit == {'unrelated_request': True}
    assert node._activate_prepared_geometry(prepared)
    assert node._latest_fused_scene_audit == expected
    # The request's explicit audit must also survive a later preparation's
    # overwrite; clearing the cache no longer erases the frozen source facts.
    node._latest_fused_scene_audit = {'another_request': True}
    result = node._request_multiview_surface_audit(prepared)['fused_scene_geometry']
    assert result == expected
    result['old_size_xyz_m'][0] = 99.
    captured['scene']['old_size_xyz_m'][0] = 88.
    assert prepared.fused_scene_audit['old_size_xyz_m'][0] not in (88., 99.)
    with pytest.raises(TypeError):
        prepared.fused_scene_audit['new_key'] = True
    assert node.latest_object_geometry.valid


@pytest.mark.parametrize('fault', ['stamp', 'epoch', 'track', 'center', 'axes', 'size', 'points', 'missing'])
def test_same_request_requires_exact_enclosure_geometry(prepared_case, fault):
    _node, prepared, captured = prepared_case
    audit = deepcopy(captured['scene'])
    if fault == 'stamp':
        audit['snapshot_stamp_ns'] += 1
    elif fault == 'epoch':
        audit['target_epoch'] += 1
    elif fault == 'track':
        audit['target_track_id'] += '-other'
    elif fault == 'center':
        audit['center_base_m'][0] += .001
    elif fault == 'axes':
        audit['axes_base'][0][0] = -.5
    elif fault == 'size':
        audit['new_size_xyz_m'][1] += .001
    elif fault == 'points':
        audit['enclosed_points_sha256'] = 'unrelated-points-same-frame'
    else:
        audit = None
    assert remote._snapshot_fused_scene_evidence(prepared.snapshot, prepared.geometry, audit) == {}


def test_missing_request_audit_never_uses_latest_request(prepared_case):
    from dataclasses import replace
    node, prepared, _captured = prepared_case
    empty = replace(prepared, fused_scene_audit=None)
    assert node._activate_prepared_geometry(empty)
    assert node._latest_fused_scene_audit is None
    node._latest_fused_scene_audit = {'later_request': True}
    assert node._request_multiview_surface_audit(empty)['fused_scene_geometry'] is None


def test_invalidated_activation_does_not_restore_old_request_audit(prepared_case):
    node, prepared, _captured = prepared_case
    node._geometry_invalidation_generation += 1
    latest = deepcopy(node._latest_fused_scene_audit)
    assert not node._activate_prepared_geometry(prepared)
    assert node._latest_fused_scene_audit == latest
