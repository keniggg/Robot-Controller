"""Runtime model snapshots keep pure geometry offline and invalidate old bundles.

All geometry verdicts in the gate tests are produced by the public kernel.
The refresh transport is the existing local parameter fixture; no ROS graph,
service, robot, camera or live scene is contacted.
"""
from copy import deepcopy
import threading
from types import SimpleNamespace as NS

import numpy as np
import pytest
from tf.transformations import quaternion_from_matrix

from static_cad_contract_fixtures import XML, candidate_fixture, context, gg
from test_remote_grasp6d_node import make_processing_node, remote_node
from test_runtime_refresh_callback_transaction import runtime_node


@pytest.mark.parametrize('description', [XML, '<robot name="unsupported"/>', None],
                         ids=['supported', 'unsupported', 'missing'])
@pytest.mark.parametrize('block_at', [
    '/grasp_6d/remote', '/grasp_6d/remote/max_candidates'])
def test_runtime_model_snapshot_commits_with_valid_configuration(
        monkeypatch, description, block_at):
    node = runtime_node()
    previous = '<robot name="previous-runtime-snapshot"/>'
    node._geometry_robot_description = previous
    node._observe_return_paused = False
    blocked, release = threading.Event(), threading.Event()
    reads, errors = [], []
    before_grasp = deepcopy(node.grasp_config)

    def get_param(name, default=None):
        if name == '/robot_description':
            reads.append(name)
            return description
        if name == block_at:
            blocked.set()
            if not release.wait(5.):
                raise RuntimeError('bounded fixture release was not signaled')
        return {
            '/grasp_6d/remote': {
                'request_hz': 2., 'geometry_min_size_m': .008},
        }.get(name, default)

    monkeypatch.setattr(remote_node.rospy, 'get_param', get_param)
    monkeypatch.setattr(remote_node.rospy, 'get_master', lambda: NS(
        getParam=lambda name: (1, 'offline fixture',
                               deepcopy(get_param(name, before_grasp)))))

    def refresh():
        try:
            node._refresh_runtime_params()
        except Exception as exc:
            errors.append(exc)

    worker = threading.Thread(target=refresh)
    try:
        worker.start()
        assert blocked.wait(5.)
        with node._stream_condition:
            # The model read already happened, but no staged value is visible.
            assert node._geometry_robot_description is previous
            assert node.rate_hz == 1.5
            assert node.grasp_config == before_grasp
            node._observe_return_paused = True
        release.set()
        worker.join(5.)
        assert not worker.is_alive()
        assert not errors, errors
        assert reads == ['/robot_description']
        assert node._geometry_robot_description is description
        assert node.rate_hz == 2.
        assert node.geometry_min_size_m == .008
        assert node._observe_return_paused is True
    finally:
        release.set()
        worker.join(5.)
        node.shutdown_streaming_worker()


@pytest.mark.parametrize('remote_config', [
    {'request_hz': 2., 'tracking_position_threshold_m': float('nan')},
    {'request_hz': 2., 'geometry_min_size_m': 'invalid'},
    {'request_hz': 2., 'camera_visibility_margin_px': 'invalid'},
])
def test_invalid_runtime_stage_preserves_previous_model_and_configuration(
        monkeypatch, remote_config):
    node = runtime_node()
    node._geometry_robot_description = XML
    node._observe_return_paused = False
    previous_geometry = node.gripper_geometry
    reads = []

    def get_param(name, default=None):
        if name == '/robot_description':
            reads.append(name)
            return '<robot name="new-uncommitted-model"/>'
        if name == '/grasp_6d/remote':
            node._observe_return_paused = True
            return remote_config
        return default

    monkeypatch.setattr(remote_node.rospy, 'get_param', get_param)
    monkeypatch.setattr(remote_node.rospy, 'get_master', lambda: NS(
        getParam=lambda name: (1, 'offline fixture',
                               deepcopy(get_param(name, node.grasp_config)))))
    try:
        with pytest.raises((ValueError, remote_node.CandidateContractError)):
            node._refresh_runtime_params()
        assert reads == ['/robot_description']
        assert node._geometry_robot_description is XML
        assert node.gripper_geometry is previous_geometry
        assert node.rate_hz == 1.5
        assert node._observe_return_paused is True
    finally:
        node.shutdown_streaming_worker()


def test_model_parameter_read_failure_never_partially_commits(monkeypatch):
    node = runtime_node()
    node._geometry_robot_description = XML
    previous_geometry = node.gripper_geometry
    failure = RuntimeError('offline model parameter provider unavailable')

    def get_param(name, default=None):
        if name == '/robot_description':
            raise failure
        return default

    monkeypatch.setattr(remote_node.rospy, 'get_param', get_param)
    monkeypatch.setattr(remote_node.rospy, 'get_master', lambda: NS(
        getParam=lambda name: (1, 'offline fixture',
                               deepcopy(get_param(name, node.grasp_config)))))
    try:
        with pytest.raises(RuntimeError) as caught:
            node._refresh_runtime_params()
        assert caught.value is failure
        assert node._geometry_robot_description is XML
        assert node.gripper_geometry is previous_geometry
        assert node.rate_hz == 1.5
    finally:
        node.shutdown_streaming_worker()


def pose_from_matrix(transform):
    return remote_node.make_pose_stamped(
        'base_link', np.asarray(transform)[:3, 3],
        quaternion_from_matrix(transform), remote_node.rospy.Time(1))


@pytest.mark.parametrize('model_state', ['bound', 'uninitialized', 'none', 'unsupported'])
@pytest.mark.parametrize('support_collision', [False, True])
def test_real_node_geometry_reaches_public_kernel_without_parameter_io(
        monkeypatch, model_state, support_collision):
    node = make_processing_node()
    description = {
        'bound': XML, 'uninitialized': None, 'none': None,
        'unsupported': '<robot name="unsupported"/>',
    }[model_state]
    if model_state != 'uninitialized':
        node._geometry_robot_description = description
    else:
        assert not hasattr(node, '_geometry_robot_description')
    values = candidate_fixture()
    if support_collision:
        values['pregrasp_T_base_tool'][2, 3] = -.1
    node.gripper_geometry = values['gripper']
    camera = NS(
        _center_base_xyz=values['candidate_center_base'].copy(),
        depth_m=values['candidate_depth_m'],
        width_m=values['candidate_width_m'])
    geometry = NS(
        ok=True, center_base=values['obb_center_base'],
        axes_base=values['R_base_obb'], size_xyz_m=values['obb_size_xyz_m'],
        support_normal_base=values['support_normal_base'],
        support_offset_m=values['support_offset_m'], object_points_base=None)
    plan = NS(**{
        name: pose_from_matrix(values[key])
        for name, key in [
            ('pregrasp', 'pregrasp_T_base_tool'),
            ('approach', 'approach_T_base_tool'),
            ('grasp', 'grasp_T_base_tool'),
            ('lift', 'lift_T_base_tool')]
    })
    public = gg.evaluate_candidate
    expected = public(**values, robot_description=description)
    model_sha = context().model_sha256 if model_state == 'bound' else None
    calls, box_calls = [], []
    real_boxes = gg._check_boxes

    def record_public(**kwargs):
        calls.append(kwargs['robot_description'])
        # Forward every argument unchanged; the real gate decides success.
        return public(**kwargs)

    def record_boxes(*args, **kwargs):
        ctx = kwargs.get('finger_cad_context')
        box_calls.append(ctx)
        if model_sha is not None:
            assert ctx is not None and ctx.model_sha256 == model_sha
        else:
            assert ctx is None
        return real_boxes(*args, **kwargs)

    def forbidden_parameter_io(*args, **kwargs):
        raise AssertionError('pure geometry attempted ROS parameter IO')

    monkeypatch.setattr(remote_node, 'evaluate_candidate', record_public)
    monkeypatch.setattr(gg, '_check_boxes', record_boxes)
    monkeypatch.setattr(remote_node.rospy, 'get_param', forbidden_parameter_io)
    monkeypatch.setattr(remote_node.rospy, 'get_master', forbidden_parameter_io)
    result = node._evaluate_candidate_geometry(
        None, camera, plan.grasp, plan, geometry=geometry,
        contact_execution_phase=False)
    assert calls == [description]
    assert result.ok == expected.ok
    assert result.failed_gate == expected.failed_gate
    assert result.failure_code == expected.failure_code
    assert result.required_open_width_m == pytest.approx(expected.required_open_width_m, abs=1e-12)
    assert result.support_clearance_m == pytest.approx(expected.support_clearance_m, abs=1e-12)
    if support_collision:
        assert not result.ok and 'support' in result.failure_reason
        assert box_calls
    else:
        assert result.ok, (result.failed_gate, result.failure_reason)
        assert len(box_calls) > 4


@pytest.mark.parametrize('description', [
    XML, '<robot name="replacement-unregistered-model"/>'],
    ids=['supported', 'unsupported'])
def test_local_model_change_invalidates_real_retained_bundle(monkeypatch, description):
    from test_gen56_contact_bundle_independent import qualified_chain, base
    env = qualified_chain(monkeypatch)
    previous_params = deepcopy(env.params)
    previous_calls = len(env.service_calls)
    previous_builds = len(env.builds)
    previous_digest = env.node._contact_bundle_inputs(env.candidate, env.runtime)
    env.node._geometry_robot_description = description
    assert env.params == previous_params
    assert env.node._contact_bundle_inputs(env.candidate, env.runtime) != previous_digest
    with pytest.raises((base.CandidateContractError, base.StreamResultCancelled, RuntimeError)):
        env.node._publish_selected_preview(
            env.candidate, ticket=env.prepared.ticket)
    assert not env.published
    assert len(env.builds) == previous_builds
    assert len(env.service_calls) == previous_calls

