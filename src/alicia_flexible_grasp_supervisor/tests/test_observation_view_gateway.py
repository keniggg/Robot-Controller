"""Execution integration: visibility rejection cannot fall through to motion."""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace as NS

import numpy as np
import pytest
from geometry_msgs.msg import TransformStamped

from alicia_flexible_grasp.robot import observation_view_gateway as view
from alicia_flexible_grasp.robot.observation_path_guard import ObservationPathError, FrozenObservationScene
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
from test_observation_path_guard import gateway_fixture, rich_plan


def configured(monkeypatch):
    gateway = view.ObservationViewGateway()
    transform = TransformStamped()
    transform.transform.rotation.w = 1.
    calls = []
    def lookup(parent, child, stamp, timeout):
        calls.append((parent, child, stamp.to_nsec()))
        return deepcopy(transform)
    gateway._observation_view_tf_buffer = NS(lookup_transform=lookup)
    params = {'/camera': dict(fx=500., fy=500., cx=320., cy=240., width=640,
                             height=480, frame_id='camera_link', align_depth_to_color=True),
              '/handeye': dict(mode='eye_on_hand', parent_frame='tool0',
                               camera_frame='camera_link', child_frame='camera_link')}
    monkeypatch.setattr(view.rospy, 'get_param', lambda key, default=None: params.get(key, default))
    scene = replace(FrozenObservationScene.from_plan(rich_plan()), require_target_visibility=True)
    return gateway, scene, params, transform, calls


def test_diagnostic_plan_requires_execution_visibility():
    plan = rich_plan()
    plan.candidate_source = 'silhouette_observation'
    plan.candidate_source_lineage = ['silhouette_observation']
    plan.poses = plan.poses[:1]
    plan.model_choice = 'unknown_tabletop'
    plan.refinement_status = 'NOT_EVALUATED'
    plan.candidate_width_m = plan.required_open_width_m = 0.
    plan.has_candidate_model_width = False
    plan.contact_geometry_evidence = ''
    plan.plan_id = compute_plan_id(plan)
    assert FrozenObservationScene.from_plan(plan).require_target_visibility
    assert not FrozenObservationScene.from_plan(rich_plan()).require_target_visibility


def test_exact_source_calibration_and_all_obb_corners(monkeypatch):
    gateway, scene, params, transform, calls = configured(monkeypatch)
    seen = {}
    def screen(path, fk, tool_camera, corners, intrinsics, width, height, desired, **limits):
        seen.update(corners=corners, intrinsics=intrinsics, limits=limits)
        return {'scope':'bounded_sampled_visibility', 'ok':True}
    monkeypatch.setattr(view, 'validate_observation_target_trajectory', screen)
    audit = gateway._unknown_observation_view_audit(scene, object(), object(), object())
    assert calls == [('tool0','camera_link',scene.source_ns), ('tool0','camera_link',0)]
    assert seen['corners'].shape == (8, 3)
    assert np.allclose(np.mean(seen['corners'], axis=0), scene.center)
    gateway._revalidate_unknown_observation_view(scene, audit)
    assert audit['endpoint_support'] == 'requires_fresh_measured_arrival_not_certified_by_this_screen'


@pytest.mark.parametrize('mutation', ['intrinsics','handeye','margin','plan_id','missing_audit',
                                           'runtime_profile','depth_scale','calibration_config'])
def test_changed_projection_or_binding_blocks_submission(monkeypatch, mutation):
    gateway, scene, params, transform, calls = configured(monkeypatch)
    monkeypatch.setattr(view, 'validate_observation_target_trajectory', lambda *a, **k: {'ok':True})
    audit = gateway._unknown_observation_view_audit(scene, None, None, None)
    if mutation == 'intrinsics': params['/camera']['fx'] += 1.
    if mutation == 'handeye': transform.transform.translation.z = .001
    if mutation == 'runtime_profile': params['/camera']['runtime_profile'] = {'source':'simulation'}
    if mutation == 'depth_scale': params['/camera']['depth_scale'] = .001
    if mutation == 'calibration_config': params['/handeye']['translation_xyz'] = [0., 0., .01]
    if mutation == 'margin': params['/grasp_6d/remote/camera_visibility_margin_px'] = 20
    if mutation == 'plan_id': audit['plan_id'] = 'another-plan'
    if mutation == 'missing_audit': audit = None
    with pytest.raises(ObservationPathError):
        gateway._revalidate_unknown_observation_view(scene, audit)


def test_missing_exact_tf_is_not_replaced_by_configuration(monkeypatch):
    gateway, scene, params, transform, calls = configured(monkeypatch)
    def missing(*a): raise RuntimeError('no exact TF')
    gateway._observation_view_tf_buffer.lookup_transform = missing
    with pytest.raises(RuntimeError, match='no exact TF'):
        gateway._unknown_observation_view_audit(scene, None, None, None)


def test_legacy_observation_does_not_gain_new_authority_or_request_tf():
    gateway = view.ObservationViewGateway()
    scene = FrozenObservationScene.from_plan(rich_plan())
    assert gateway._unknown_observation_view_audit(scene, None, None, None) is None
    gateway._revalidate_unknown_observation_view(scene, None)


def test_real_gateway_rejects_bad_view_before_collision_proof(monkeypatch):
    module, gateway, context, planner, path, params = gateway_fixture(monkeypatch)
    seen = []
    def reject(*args):
        seen.append('view')
        raise ObservationPathError('OBSERVATION_TARGET_OUT_OF_VIEW')
    monkeypatch.setattr(gateway, '_unknown_observation_view_audit', reject)
    monkeypatch.setattr(module, 'validate_observation_trajectory', lambda *a, **k: seen.append('collision'))
    with pytest.raises(ObservationPathError, match='OUT_OF_VIEW'):
        gateway._validate_frozen_observation_path(context, path, planner)
    assert seen == ['view']


def test_real_gateway_rechecks_projection_after_collision_proof(monkeypatch):
    module, gateway, context, planner, path, params = gateway_fixture(monkeypatch)
    seen = []
    monkeypatch.setattr(gateway, '_unknown_observation_view_audit', lambda *a: {'ok':True})
    def changed(*a):
        seen.append('revalidate')
        raise ObservationPathError('OBSERVATION_VIEW_CONFIGURATION_CHANGED')
    monkeypatch.setattr(gateway, '_revalidate_unknown_observation_view', changed)
    with pytest.raises(ObservationPathError, match='CONFIGURATION_CHANGED'):
        gateway._validate_frozen_observation_path(context, path, planner)
    assert seen == ['revalidate']


def test_changed_current_mount_cannot_reuse_old_source_transform(monkeypatch):
    gateway, scene, params, transform, calls = configured(monkeypatch)
    def changed(parent, child, stamp, timeout):
        value = deepcopy(transform)
        if stamp.to_nsec() == 0:
            value.transform.translation.z = .001
        return value
    gateway._observation_view_tf_buffer.lookup_transform = changed
    with pytest.raises(ObservationPathError, match='MOUNT_CHANGED_SINCE_SOURCE'):
        gateway._unknown_observation_view_audit(scene, None, None, None)


@pytest.mark.parametrize('source', ['tabletop_geometry', 'graspnet'])
def test_all_unknown_observation_sources_require_visibility(source):
    plan = rich_plan()
    plan.model_choice = 'unknown_tabletop'
    plan.candidate_source = source
    plan.candidate_source_lineage = [source]
    if source == 'graspnet':
        plan.has_candidate_model_width = True
        plan.candidate_width_m = .035
    plan.plan_id = compute_plan_id(plan)
    assert FrozenObservationScene.from_plan(plan).require_target_visibility
    plan.diagnostic = 'CONTACT_EXECUTION_PLAN'
    plan.plan_id = compute_plan_id(plan)
    assert not FrozenObservationScene.from_contact_plan(plan).require_target_visibility


@pytest.mark.parametrize('key,value', [
    ('camera_visibility_margin_px', 20.),
    ('camera_visibility_min_depth_m', .02),
    ('camera_visibility_max_depth_m', .8),
    ('camera_visibility_margin_px', True),
    ('camera_visibility_max_depth_m', float('nan')),
])
def test_weakened_or_invalid_limits_rejected_before_projection(monkeypatch, key, value):
    gateway, scene, params, transform, calls = configured(monkeypatch)
    params['/grasp_6d/remote/' + key] = value
    seen = []
    monkeypatch.setattr(view, 'validate_observation_target_trajectory', lambda *a, **k: seen.append(k))
    with pytest.raises(ObservationPathError, match='LIMITS_INVALID'):
        gateway._unknown_observation_view_audit(scene, None, None, None)
    assert not seen
    assert not calls


def test_tighter_limits_are_used_and_bound_to_submission(monkeypatch):
    gateway, scene, params, transform, calls = configured(monkeypatch)
    for key, value in [('margin_px', 50.), ('min_depth_m', .10), ('max_depth_m', .40)]:
        params['/grasp_6d/remote/camera_visibility_' + key] = value
    seen = []
    def project(*args, **limits):
        seen.append(limits)
        return {'ok': True}
    monkeypatch.setattr(view, 'validate_observation_target_trajectory', project)
    audit = gateway._unknown_observation_view_audit(scene, None, None, None)
    assert seen == [dict(roi_margin_px=50., minimum_depth_m=.10, maximum_depth_m=.40)]
    assert audit['configuration']['requested_visibility_limits_snapshot']['camera_visibility_margin_px'] == 50.
    gateway._revalidate_unknown_observation_view(scene, audit)
    params['/grasp_6d/remote/camera_visibility_margin_px'] = 51.
    with pytest.raises(ObservationPathError, match='CONFIGURATION_CHANGED'):
        gateway._revalidate_unknown_observation_view(scene, audit)
