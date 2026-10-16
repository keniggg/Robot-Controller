"""Real parameter staging must not rewind concurrent callback-owned state."""
from copy import deepcopy
import threading
import types

import pytest

from test_remote_grasp6d_node import make_processing_node, remote_node


def runtime_node():
    node = make_processing_node()
    node._initialize_streaming_state(start_worker=False)
    node.execution_plan_controller = remote_node.ExecutionPlanController()
    node.rate_hz = 1.5
    node.target_instance_association_threshold_m = .08
    node.target_absolute_sanity_distance_m = .15
    node.moveit_top_n = 5
    node.candidate_frame_convention = 'opencv_optical'
    node.tabletop_geometry_enabled = True
    node.tabletop_geometry_config = remote_node.TabletopGeometryConfig()
    node.hybrid_merge_config = remote_node.MergeConfig()
    # Constructor defaults required by successful late-stage parameter reads.
    node.candidate_max_target_distance_m = .04
    node.candidate_min_relative_z_m = -.015
    node.candidate_max_relative_z_m = .08
    node._refresh_runtime_params = types.MethodType(
        remote_node.RemoteGrasp6DNode._refresh_runtime_params, node)
    return node


CALLBACK_CHANGES = (
    ('_observe_return_paused', False, True),
    ('_mode_ready', True, False),
    ('_observe_return_session', {'stamp_ns': 1}, {'stamp_ns': 2}),
    ('_mode_selection', {'generation': 5}, {'generation': 6}),
    ('_stream_generation', 9, 10),
    ('latest_joint_state', {'stamp_ns': 1}, {'stamp_ns': 2}),
    ('latest_object', {'detected': True}, {'detected': False}),
    ('_execution_authority_ticket', {'owner': 'old'}, None),
    ('robot_execution_active', True, False),
    ('_measurement_remote_epoch_ns', 1, 2),
    ('_station_compute_reference', {'source_ns': 1}, None),
    ('_latest_target_observation', {'source_ns': 1}, {'source_ns': 2}),
)


@pytest.mark.parametrize('field,old,new', CALLBACK_CHANGES)
@pytest.mark.parametrize('block_at', ['/grasp_6d/remote', '/grasp_6d/remote/max_candidates'])
def test_full_valid_stage_preserves_callback_changes_and_commits_configuration(
        monkeypatch, field, old, new, block_at):
    node = runtime_node()
    setattr(node, field, deepcopy(old))
    replacement = deepcopy(new)
    blocked, release = threading.Event(), threading.Event()
    errors = []
    remote_config = {'request_hz': 2., 'geometry_min_size_m': .008}
    before_grasp = dict(node.grasp_config)

    def get_param(name, default=None):
        if name == block_at:
            blocked.set()
            if not release.wait(2.):
                raise RuntimeError('test release was not signaled')
        return {'/grasp_6d/remote': remote_config}.get(name, default)

    monkeypatch.setattr(remote_node.rospy, 'get_param', get_param)
    def refresh():
        try:
            node._refresh_runtime_params()
        except Exception as exc:
            errors.append(exc)
    worker = threading.Thread(target=refresh)
    try:
        worker.start()
        assert blocked.wait(2.)
        with node._stream_condition:
            assert node.rate_hz == 1.5
            assert node.grasp_config == before_grasp
            setattr(node, field, replacement)
        release.set()
        worker.join(2.)
        assert not worker.is_alive()
        assert not errors, errors
        assert getattr(node, field) is replacement
        assert node.rate_hz == 2.
        assert node.geometry_min_size_m == .008
    finally:
        release.set()
        worker.join(2.)
        node.shutdown_streaming_worker()


@pytest.mark.parametrize('remote_config', [
    {'request_hz': 2., 'tracking_position_threshold_m': float('nan')},
    {'request_hz': 2., 'geometry_min_size_m': 'invalid'},
    {'request_hz': 2., 'camera_visibility_margin_px': 'invalid'},
])
def test_actual_invalid_stage_keeps_callback_changes_without_partial_config(monkeypatch, remote_config):
    node = runtime_node()
    node._observe_return_paused = False
    before = node.gripper_geometry
    def get_param(name, default=None):
        if name == '/grasp_6d/remote':
            node._observe_return_paused = True
            return remote_config
        return default
    monkeypatch.setattr(remote_node.rospy, 'get_param', get_param)
    try:
        with pytest.raises((ValueError, remote_node.CandidateContractError)):
            node._refresh_runtime_params()
        assert node._observe_return_paused
        assert node.rate_hz == 1.5
        assert node.gripper_geometry is before
    finally:
        node.shutdown_streaming_worker()
