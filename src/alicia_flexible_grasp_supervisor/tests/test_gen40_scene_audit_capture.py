"""Unapplied diagnostic capture regression; no grasp qualification is simulated.

The original/planning OBB metadata comes from GEN39 source1792220759499861717.
A small explicit measured cloud is a unit input. Geometry preparation and the
inferred-enclosure stage are adapters, while the actual request preparation,
strict source/geometry audit helper, freezing, and request extraction run.
No RAW model, plan, collision approval, ROS client, or hardware is created.
"""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from alicia_flexible_grasp.vision.fused_scene_geometry import _array_digest
from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
import test_remote_grasp6d_node as processing
import test_remote_grasp6d_streaming as streaming
from test_fused_scene_pipeline import exact_snapshot

remote = streaming.remote_node

ORIGINAL_AXES = np.asarray([
    [-.6398652557084927, -.7671769755376258, -.04485468472798142],
    [.763838377516204, -.6413206003196884, .07251772637781549],
    [-.084400163332297, .012139843925517237, .9963579861771299]])
PLANNING_AXES = np.asarray([
    [-.6813486114027913, -.730583415495754, -.04485468472798142],
    [.727169380413396, -.6826169288482307, .07251772637781549],
    [-.08359881535456129, .016792898867327703, .9963579861771299]])


def prepare_case(monkeypatch, fault=None):
    node = remote.RemoteGrasp6DNode.__new__(remote.RemoteGrasp6DNode)
    node.__dict__.update(processing.make_processing_node().__dict__)
    node._initialize_streaming_state(result_max_age_sec=1.2,
        performance_window_size=100, source_clock=streaming.MutableClock(20.2),
        start_worker=False)
    streaming.seed_measured_reference_surface(node)
    snapshot = exact_snapshot(node)
    snapshot.object_msg.detected = True
    points = np.asarray([[-.09, -.36, .02], [-.08, -.37, .02], [-.10, -.38, .01]])
    measured = GeometryEstimate(True, '', '',
        [-.08661955892181153, -.36737793284696846, .015144704178169653],
        ORIGINAL_AXES, [.06110794219247761, .04575425843274332, .024042875839723716],
        ORIGINAL_AXES[:, 2], .01784868227526578, .9, points, 'instance_mask')
    planning = replace(measured,
        center_base=np.asarray([-.08550724653956507, -.3685646310895578, .01529814221852577]),
        axes_base=PLANNING_AXES,
        size_xyz_m=np.asarray([.06681408223819799, .04906652361178271, .02407673611416046]))
    audit = dict(schema_version=2, policy='registered_measured_surface_tangent_enclosure_v2',
        snapshot_stamp_ns=snapshot.stamp_ns,
        target_epoch=snapshot.target_identity.epoch,
        target_track_id=snapshot.target_identity.track_id,
        center_base_m=measured.center_base.tolist(), axes_base=measured.axes_base.tolist(),
        new_size_xyz_m=measured.size_xyz_m.tolist(),
        enclosed_points_sha256=_array_digest(points),
        registration_choice=dict(kind='explicit unit diagnostic marker'),
        surface_replay=dict(status='stored', diagnostic_only=True, execution_authorized=False,
                            references=dict(before_registration_reference='unit marker')))
    expected = deepcopy(audit)
    if fault == 'stamp': audit['snapshot_stamp_ns'] += 1
    elif fault == 'epoch': audit['target_epoch'] += 1
    elif fault == 'track': audit['target_track_id'] += '-other'
    elif fault == 'center': audit['center_base_m'][0] += .001
    elif fault == 'axes': audit['axes_base'][0][0] = -.5
    elif fault == 'size': audit['new_size_xyz_m'][1] += .001
    elif fault == 'points': audit['enclosed_points_sha256'] = 'different measured source'
    elif fault == 'missing': audit = None
    events = []

    def measured_stage(*_):
        node._latest_fused_scene_audit = deepcopy(audit)
        return measured, snapshot.target_depth_raw, np.eye(4)

    def inferred_planning_stage(src, geometry, transform):
        assert src is snapshot and geometry is measured
        events.append('planning_enclosure')
        return planning

    actual_capture = remote._snapshot_fused_scene_evidence
    def capture(src, geometry, current_audit):
        events.append('capture_measured' if geometry is measured else 'capture_planning')
        return actual_capture(src, geometry, current_audit)

    def remote_wait(*_args, **_kwargs):
        node._latest_fused_scene_audit = {'later_request': True}
        return (), {}, {}, 0., 0., 0.

    monkeypatch.setattr(node, '_prepare_snapshot_geometry', measured_stage)
    monkeypatch.setattr(node, '_prepare_regular_box_snapshot', inferred_planning_stage)
    monkeypatch.setattr(remote, '_snapshot_fused_scene_evidence', capture)
    monkeypatch.setattr(node, '_require_graspnet_input_prerequisites', lambda *_: None)
    monkeypatch.setattr(node, '_require_stream_ticket_current', lambda *_: None)
    monkeypatch.setattr(node, '_require_stream_ticket_current_locked', lambda *_: None)
    monkeypatch.setattr(node, '_gripper_contract_mismatch_reason', lambda: '')
    monkeypatch.setattr(node, '_generate_tabletop_candidates', lambda *_a, **_k: ((), {}))
    monkeypatch.setattr(node, '_build_frozen_graspnet_input', lambda *_a, **_k: (SimpleNamespace(), {}))
    monkeypatch.setattr(node, '_predict_remote', remote_wait)
    ticket = SimpleNamespace(payload=(snapshot, processing.frozen_input_config(node)))
    try:
        prepared = remote.RemoteGrasp6DNode._prepare_and_predict(node, ticket)
    finally:
        node.shutdown_streaming_worker()
    return node, prepared, measured, planning, expected, events


def test_measured_fusion_receipt_survives_actual_planning_axes_center_size_change(monkeypatch):
    node, prepared, measured, planning, expected, events = prepare_case(monkeypatch)
    assert not np.array_equal(measured.axes_base, planning.axes_base)
    assert not np.array_equal(measured.center_base, planning.center_base)
    assert not np.array_equal(measured.size_xyz_m, planning.size_xyz_m)
    np.testing.assert_array_equal(measured.object_points_base, planning.object_points_base)
    assert prepared.geometry is planning
    assert remote._thaw_request_evidence(prepared.fused_scene_audit) == expected
    assert events == ['capture_measured', 'planning_enclosure']
    assert node._latest_fused_scene_audit == {'later_request': True}
    assert node._request_multiview_surface_audit(prepared)['fused_scene_geometry'] == expected
    # Existing strict helper still rejects attaching the measured receipt to
    # planning geometry; the change must fix call ordering, not relax equality.
    assert remote._snapshot_fused_scene_evidence(prepared.snapshot, planning, expected) == {}
    with pytest.raises(TypeError):
        prepared.fused_scene_audit['replacement'] = True


@pytest.mark.parametrize('fault', ['stamp', 'epoch', 'track', 'center', 'axes', 'size', 'points', 'missing'])
def test_mismatched_measured_receipt_stays_unavailable_before_planning_enclosure(monkeypatch, fault):
    node, prepared, _measured, planning, _expected, events = prepare_case(monkeypatch, fault)
    assert prepared.geometry is planning
    assert events == ['capture_measured', 'planning_enclosure']
    assert not prepared.fused_scene_audit
    assert node._request_multiview_surface_audit(prepared)['fused_scene_geometry'] is None
