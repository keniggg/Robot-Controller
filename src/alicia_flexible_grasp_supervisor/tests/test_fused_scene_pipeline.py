"""Offline production routing from exact RGB-D source to the shared scene OBB.

Only the depth estimator and TF/intrinsics inputs are replaced. Observation
binding, registration, inverse frame mapping, enclosure and ROS serialization
run their production implementations; no services or worker threads start.
"""
from dataclasses import replace
from copy import deepcopy
import io
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

import test_remote_grasp6d_streaming as streaming
from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
from alicia_flexible_grasp.vision.rgbd_snapshot import DepthQuality, SnapshotResult

remote = streaming.remote_node


@pytest.fixture
def node(monkeypatch):
    result = streaming.streaming_node(start_worker=False)
    streaming.seed_measured_reference_surface(result)
    monkeypatch.setattr(remote.rospy.Time, 'now', lambda: remote.rospy.Time(20, 500_000_000))
    monkeypatch.setattr(result, '_snapshot_base_optical_transform', lambda *_: np.eye(4))
    monkeypatch.setattr(result, '_camera_intrinsics', lambda: SimpleNamespace(depth_scale=.001))
    yield result
    result.shutdown_streaming_worker()


def exact_snapshot(node, stamp_ns=20_100_000_000):
    identity = node._current_stream_target_identity()
    mask = np.zeros((32, 32), dtype=np.uint8)
    mask[8:24, 8:24] = 255
    depth = np.full((32, 32), 350, dtype=np.uint16)
    return SnapshotResult(
        ok=True, failure_code='', failure_reason='',
        color_bgr=np.zeros((32, 32, 3), dtype=np.uint8), depth_raw=depth,
        target_depth_raw=np.where(mask, depth, 0), object_mask=mask,
        bbox=(8, 8, 16, 16), object_msg=SimpleNamespace(label='unknown'),
        stamp_sec=stamp_ns / 1e9, stamp_ns=stamp_ns,
        frame_id='camera_color_optical_frame',
        quality=DepthQuality(1, 256, 256, 1., .350, .001),
        source_mode='instance_mask', target_epoch=identity.epoch,
        target_identity=identity, sample_stamp_ns=(stamp_ns,))


def current_top_geometry(yaw=0., displacement=(0., 0., 0.)):
    """A current view sees only the top; the accepted reference sees a side."""
    c, s = np.cos(yaw), np.sin(yaw)
    rotation = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    displacement = np.asarray(displacement)
    points = streaming.measured_surface_points()
    top = points[np.isclose(points[:, 2], .034)]
    return GeometryEstimate(
        True, '', '', (np.array([0., 0., .030]) - displacement) @ rotation,
        rotation.T, np.array([.080, .050, .010]), np.array([0., 0., 1.]),
        float(displacement[2]), .95, (top - displacement) @ rotation,
        'instance_mask')


def run_prepare(node, monkeypatch, estimate, snapshot=None):
    snapshot = exact_snapshot(node) if snapshot is None else snapshot
    monkeypatch.setattr(remote, 'estimate_object_geometry', lambda **_: estimate)
    stamp = remote.rospy.Time(*divmod(snapshot.stamp_ns, 1_000_000_000))
    return node._prepare_snapshot_geometry(snapshot, stamp), snapshot


def point_rows(points):
    return {tuple(row) for row in points}


@pytest.mark.parametrize('yaw,displacement', [
    (0., (0., 0., 0.)),
    (np.deg2rad(3.), (.003, -.002, .001)),
])
def test_registered_historical_surface_expands_actual_returned_scene(
        node, monkeypatch, yaw, displacement):
    original = current_top_geometry(yaw, displacement)
    old_surface = node._near_field_fused_surface
    old_points = old_surface.points_base.copy()
    enclosure = Mock(wraps=remote.enclose_registered_surface)
    monkeypatch.setattr(remote, 'enclose_registered_surface', enclosure)

    (prepared, target_depth, transform), snapshot = run_prepare(node, monkeypatch, original)

    assert prepared.ok, prepared.failure_reason
    evidence = node._latest_registration_evidence
    assert evidence.result.ok, evidence.result.code
    assert evidence.stamp_ns == snapshot.stamp_ns
    assert evidence.identity == snapshot.target_identity
    enclosure.assert_called_once()
    args, kwargs = enclosure.call_args
    assert args[0] is original and kwargs['expected_identity'] == snapshot.target_identity
    snapshot_surface = args[1]
    assert snapshot_surface.view_stamps_ns[-1] == snapshot.stamp_ns
    assert len(snapshot_surface.view_stamps_ns) == 2
    # The stored reference cloud stays in reference coordinates; only the
    # enclosure's input uses this exact source's inverse registration.
    np.testing.assert_allclose(
        snapshot_surface.points_base @ evidence.result.transform_base[:3, :3].T
        + evidence.result.transform_base[:3, 3],
        node._near_field_fused_surface.points_base, atol=1e-12)
    np.testing.assert_array_equal(old_surface.points_base, old_points)
    if yaw:
        assert not np.allclose(snapshot_surface.points_base, node._near_field_fused_surface.points_base)
    assert abs(float((prepared.center_base-original.center_base) @ original.support_normal_base)) < 1e-12
    np.testing.assert_array_equal(node._latest_target_observation.points_base, original.object_points_base)
    np.testing.assert_array_equal(prepared.axes_base, original.axes_base)
    assert prepared.support_offset_m == original.support_offset_m
    assert prepared.size_xyz_m[2] > .050
    assert np.all(prepared.size_xyz_m >= original.size_xyz_m)
    assert point_rows(prepared.object_points_base) == (
        point_rows(original.object_points_base) | point_rows(snapshot_surface.points_base))
    local = (snapshot_surface.points_base - prepared.center_base) @ prepared.axes_base
    assert np.all(np.abs(local) <= prepared.size_xyz_m / 2. + 1e-12)
    np.testing.assert_array_equal(target_depth, snapshot.target_depth_raw)
    np.testing.assert_array_equal(transform, np.eye(4))
    audit = node._latest_fused_scene_audit
    assert audit['expanded'] and audit['snapshot_stamp_ns'] == snapshot.stamp_ns
    assert audit['target_track_id'] == snapshot.target_identity.track_id
    assert audit['coordinate_frame'] == 'current_snapshot_base_estimate'
    assert audit['synthetic_points_added'] == 0


def test_shared_geometry_message_preserves_enlargement_and_source_binding(node, monkeypatch):
    (prepared, _, _), snapshot = run_prepare(node, monkeypatch, current_top_geometry())
    assert prepared.ok
    stamp = remote.rospy.Time(*divmod(snapshot.stamp_ns, 1_000_000_000))
    message = remote.geometry_estimate_to_message(prepared, snapshot=snapshot, stamp=stamp)
    wire = io.BytesIO()
    message.serialize(wire)
    decoded = remote.ObjectGeometry().deserialize(wire.getvalue())
    assert decoded.valid
    assert decoded.header.stamp.to_nsec() == snapshot.stamp_ns
    assert decoded.header.frame_id == 'base_link'
    assert decoded.target_track_id == snapshot.target_identity.track_id
    assert [decoded.size_xyz_m.x, decoded.size_xyz_m.y, decoded.size_xyz_m.z] == pytest.approx(prepared.size_xyz_m)
    assert decoded.size_xyz_m.z > .050
    assert [decoded.pose_base.position.x, decoded.pose_base.position.y,
            decoded.pose_base.position.z] == pytest.approx(prepared.center_base)
    assert decoded.object_point_count == len(prepared.object_points_base)
    assert decoded.object_point_count > len(current_top_geometry().object_points_base)
    assert decoded.support_offset_m == pytest.approx(prepared.support_offset_m)


@pytest.mark.parametrize('previous_success', [False, True])
def test_rejected_registration_cannot_enclose_or_reuse_previous_view_geometry(
        node, monkeypatch, previous_success):
    if previous_success:
        (prepared, _, _), _ = run_prepare(node, monkeypatch, current_top_geometry())
        assert prepared.ok and node._latest_fused_scene_audit['expanded']
    original_surface = node._near_field_fused_surface
    original = replace(current_top_geometry(), support_offset_m=(
        node.multiview_registration_config.maximum_support_offset_delta_m + .00001))
    snapshot = exact_snapshot(node, 20_200_000_000)
    enclosure = Mock(wraps=remote.enclose_registered_surface)
    monkeypatch.setattr(remote, 'enclose_registered_surface', enclosure)

    (prepared, _, _), _ = run_prepare(node, monkeypatch, original, snapshot)

    assert prepared is original
    enclosure.assert_not_called()
    assert node._near_field_fused_surface is original_surface
    assert node._latest_fused_scene_audit is None
    evidence = node._latest_registration_evidence
    assert evidence.stamp_ns == snapshot.stamp_ns
    assert not evidence.result.ok and evidence.result.code == 'SUPPORT_OFFSET_MISMATCH'
    # Returning an unexpanded estimate cannot turn the failed source into an
    # accepted execution plan; the same source's refinement remains invalid.
    observation = node._latest_target_observation
    plan = streaming.bound_surface_plan(observation)
    assert node._apply_registration_evidence_to_plan(plan, observation)
    assert plan.refinement_status == 'INVALID_3D'


@pytest.mark.parametrize('preserve_identity', [False, True])
def test_geometry_crossing_generation_rejects_before_mutating_new_surface(
        node, monkeypatch, preserve_identity):
    snapshot = exact_snapshot(node)
    original = current_top_geometry()
    new_surfaces = []
    enclosure = Mock(wraps=remote.enclose_registered_surface)
    monkeypatch.setattr(remote, 'enclose_registered_surface', enclosure)

    def estimate_crossing_generation(**_):
        node._advance_target_instance_epoch('TEST_NEW_SOURCE', preserve_identity=preserve_identity)
        streaming.seed_measured_reference_surface(node)
        new_surfaces.append(node._near_field_fused_surface)
        return original

    monkeypatch.setattr(remote, 'estimate_object_geometry', estimate_crossing_generation)
    prepared, _, _ = node._prepare_snapshot_geometry(snapshot, remote.rospy.Time(20, 100_000_000))

    assert not prepared.ok
    assert prepared.failure_code == 'TARGET_GEOMETRY_GENERATION_STALE'
    enclosure.assert_not_called()
    assert node._near_field_fused_surface is new_surfaces[0]
    assert node._latest_registration_evidence is None
    assert node._latest_fused_scene_audit is None


def test_malformed_exact_source_binding_cannot_feed_historical_enclosure(node, monkeypatch):
    snapshot = replace(exact_snapshot(node), sample_stamp_ns=(20_099_999_999,))
    original_surface = node._near_field_fused_surface
    enclosure = Mock(wraps=remote.enclose_registered_surface)
    monkeypatch.setattr(remote, 'enclose_registered_surface', enclosure)

    (prepared, _, _), _ = run_prepare(node, monkeypatch, current_top_geometry(), snapshot)

    assert not prepared.ok and prepared.failure_code == 'TARGET_GEOMETRY_TRACK_INVALID'
    enclosure.assert_not_called()
    assert node._near_field_fused_surface is original_surface
    assert node._latest_fused_scene_audit is None


def test_accepted_history_exceeding_caller_bound_rejects_instead_of_trimming(node, monkeypatch):
    node.geometry_max_height_m = .040
    (prepared, _, _), _ = run_prepare(node, monkeypatch, current_top_geometry())
    assert node._latest_registration_evidence.result.ok
    assert not prepared.ok and prepared.failure_code == 'FUSED_SCENE_GEOMETRY_INVALID'
    assert node._latest_fused_scene_audit is None
    assert len(node._near_field_fused_surface.view_stamps_ns) == 2


def test_initial_single_view_keeps_original_scene_without_enclosure(node, monkeypatch):
    node.near_field_planning_active = False
    original = current_top_geometry()
    enclosure = Mock(wraps=remote.enclose_registered_surface)
    monkeypatch.setattr(remote, 'enclose_registered_surface', enclosure)
    (prepared, _, _), _ = run_prepare(node, monkeypatch, original)
    assert prepared is original
    enclosure.assert_not_called()
    assert node._latest_fused_scene_audit is None


def test_recentered_scene_keeps_raw_registration_and_binds_new_center_through_wire_and_plan(node, monkeypatch):
    from geometry_msgs.msg import PoseStamped
    from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id, plan_id_matches_content
    from alicia_flexible_grasp.robot.observation_path_guard import FrozenObservationScene, wire_digest
    original = current_top_geometry()
    original = replace(original, center_base=original.center_base + [.007, .004, 0.])
    (prepared, _, _), snapshot = run_prepare(node, monkeypatch, original)
    assert prepared.ok
    assert np.linalg.norm(prepared.center_base - original.center_base) > .001
    observation = node._latest_target_observation
    assert observation.identity == snapshot.target_identity
    assert observation.stamp_ns == snapshot.stamp_ns
    np.testing.assert_array_equal(observation.points_base, original.object_points_base)
    assert node._latest_registration_evidence.stamp_ns == snapshot.stamp_ns
    assert node._latest_registration_evidence.result.ok
    message = remote.geometry_estimate_to_message(prepared, snapshot=snapshot,
        stamp=remote.rospy.Time(*divmod(snapshot.stamp_ns, 10**9)))
    stamped = PoseStamped()
    stamped.header = deepcopy(message.header)
    stamped.pose.orientation.w = 1.
    stamped.pose.position.z = .20
    sequence = SimpleNamespace(**{name: deepcopy(stamped)
        for name in ('pregrasp', 'approach', 'grasp', 'lift')})
    candidate = SimpleNamespace(score=.9, candidate_source='tabletop_geometry',
        source_lineage=('tabletop_geometry',), model_width_m=None,
        required_open_width_m=.044, grasp_sequence=sequence)
    plan = remote.build_rich_plan(candidate, message, message.header, 'unknown_tabletop')
    plan.diagnostic = 'CONTACT_EXECUTION_PLAN'
    plan.plan_id = compute_plan_id(plan)
    wire = io.BytesIO()
    plan.serialize(wire)
    decoded = remote.Grasp6DPlan().deserialize(wire.getvalue())
    assert plan_id_matches_content(decoded)
    scene = FrozenObservationScene.from_contact_plan(decoded)
    np.testing.assert_allclose(scene.center, prepared.center_base, atol=1e-15)
    np.testing.assert_allclose(scene.size, prepared.size_xyz_m, atol=1e-15)
    assert scene.track_id == observation.identity.track_id
    assert scene.source_ns == observation.stamp_ns
    assert scene.geometry_digest == wire_digest(decoded.object_geometry)
    # Reusing an old center with a new geometry/plan digest is rejected.
    stale = deepcopy(decoded)
    for axis, value in zip(('x', 'y', 'z'), original.center_base):
        setattr(stale.object_geometry.pose_base.position, axis, value)
    assert not plan_id_matches_content(stale)
    assert compute_plan_id(stale) != decoded.plan_id
    with pytest.raises(ValueError):
        FrozenObservationScene.from_contact_plan(stale)
