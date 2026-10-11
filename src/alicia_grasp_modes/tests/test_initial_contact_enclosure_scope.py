"""Initial/direct contact remains a single measured view at the real producer."""
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from test_remote_modes import configured, module, base, select_unknown
from test_fused_scene_pipeline import exact_snapshot, current_top_geometry
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry


@pytest.mark.parametrize('strategy', ['direct', 'observe_return'])
def test_real_initial_contact_prepare_never_uses_cross_view_enclosure(configured, monkeypatch, strategy):
    node, params = configured
    select_unknown(params)
    params[module.SELECTION_PARAM]['strategy'] = strategy
    assert node.reset_mode_cb(None).success
    node._ros_source_clock_ns = lambda: 10_000_000_000
    node.near_field_direct_timeout_sec = 60.
    assert node.start_streaming()
    assert node.near_field_planning_active
    assert node._observe_return_phase == 'initial_contact'
    node.gripper_geometry = GripperGeometry(.05, .002, np.array([.0434, .0286, .060]),
                                           np.array([.1175, .155, .0774]), .003)
    monkeypatch.setattr(node, '_snapshot_base_optical_transform', lambda *_: np.eye(4))
    monkeypatch.setattr(node, '_camera_intrinsics', lambda: SimpleNamespace(depth_scale=.001))
    original = current_top_geometry()
    monkeypatch.setattr(base, 'estimate_object_geometry', lambda **_: original)
    enclosure = Mock(side_effect=AssertionError('initial contact must not enclose historical views'))
    monkeypatch.setattr(base, 'enclose_registered_surface', enclosure)
    for source_ns in (11_000_000_000, 11_200_000_000):
        snapshot = exact_snapshot(node, source_ns)
        prepared, _, _ = node._prepare_snapshot_geometry(snapshot, base.rospy.Time(*divmod(source_ns, 10**9)))
        assert prepared is original
        assert node._latest_target_observation.stamp_ns == source_ns
        np.testing.assert_array_equal(node._latest_target_observation.points_base, original.object_points_base)
        surface, reference = node._snapshot_multiview_surface()
        assert surface.view_stamps_ns == (source_ns,)
        assert reference.stamp_ns == source_ns
        assert node._latest_registration_evidence is None
        assert node._latest_fused_scene_audit is None
    enclosure.assert_not_called()
