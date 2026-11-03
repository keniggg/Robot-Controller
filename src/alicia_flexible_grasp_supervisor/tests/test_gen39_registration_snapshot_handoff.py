"""Snapshot registration selection uses real source qualification and fusion.

Depth estimation and calibration are explicit inputs. These cases exercise
source ownership, lock release, lifecycle cancellation and final model binding;
they do not replace registration/contact gates or establish a physical grasp.
"""
from dataclasses import replace
import threading
from types import SimpleNamespace
from pathlib import Path
import sys

import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'tools/tests'))
from storage_os_adapter import storage_os, tmp_path

from alicia_flexible_grasp.vision.multiview_surface import (
    SurfaceView, fused_surface_from_view,
)
from alicia_flexible_grasp.vision.target_observation import observation_from_snapshot
from test_fused_scene_pipeline import node
from test_gen26_regular_box_inference_independent import ray_scene
from test_gen31_outer_rgb_box import precision_scene_snapshot
from test_remote_grasp6d_streaming import remote_node as remote


def source(node, monkeypatch):
    scene = ray_scene()
    scene.identity = node._current_stream_target_identity()
    scene.stamp = 20_100_000_000
    scene.color = np.full(scene.depth.shape + (3,), 180, np.uint8)
    scene.color[scene.mask > 0] = 30
    scene.K.depth_scale = .0001
    snapshot = precision_scene_snapshot(scene)
    obs = observation_from_snapshot(snapshot, scene.geometry, scene.T, 'base_link')
    prior = replace(obs, stamp_ns=obs.stamp_ns - 100_000_000)
    side = scene.geometry.center_base + scene.geometry.axes_base @ np.array([
        .5 * scene.geometry.size_xyz_m[0], 0., -.0025])
    reference = SurfaceView(prior.identity, prior.stamp_ns,
        np.vstack((prior.points_base, side)), prior.support_normal_base,
        prior.support_offset_m, prior.edge_clearance_px)
    node._near_field_reference_view = reference
    node._near_field_fused_surface = fused_surface_from_view(reference)
    node._latest_target_observation = prior
    monkeypatch.setattr(node, '_snapshot_base_optical_transform', lambda *_: scene.T)
    monkeypatch.setattr(node, '_camera_intrinsics', lambda: scene.K)
    monkeypatch.setattr(remote, 'estimate_object_geometry', lambda **_: scene.geometry)
    monkeypatch.setattr(node, '_box_requires_measured_bilateral_contact', lambda: False)
    return scene, snapshot, prior


def prepare(node, snapshot):
    stamp = remote.rospy.Time(*divmod(snapshot.stamp_ns, 1_000_000_000))
    return node._prepare_snapshot_geometry(snapshot, stamp)


def test_source_box_only_selects_registration_final_model_binds_full_union(node, monkeypatch):
    scene, snapshot, _ = source(node, monkeypatch)
    actual = remote.qualify_regular_box_snapshot
    qualified = []

    def record(*args, **kwargs):
        result = actual(*args, **kwargs)
        qualified.append((args[4], result))
        return result

    monkeypatch.setattr(remote, 'qualify_regular_box_snapshot', record)
    geometry, _, transform = prepare(node, snapshot)
    assert geometry.ok
    assert len(qualified) == 1 and qualified[0][1].ok
    raw_model = qualified[0][1].model
    assert qualified[0][0] is scene.geometry
    choice = node._latest_box_registration_choice
    assert choice['observation'] is node._latest_target_observation
    assert choice['lifecycle'] == node._multiview_lifecycle_token()
    assert node._latest_fused_scene_audit['registration_choice'] == choice['audit']
    assert len(geometry.object_points_base) > len(scene.geometry.object_points_base)
    node._prepare_regular_box_snapshot(snapshot, geometry, transform)
    assert len(qualified) == 2 and qualified[1][1].ok
    final_model = node._regular_box_snapshot['result'].model
    assert qualified[1][0] is geometry
    assert raw_model.source['stamp_ns'] == final_model.source['stamp_ns'] == snapshot.stamp_ns
    assert raw_model.source['geometry_sha256'] != final_model.source['geometry_sha256']
    assert raw_model.model_sha256 != final_model.model_sha256


@pytest.mark.parametrize('changed', ['phase', 'generation'])
def test_qualified_raw_source_cannot_commit_after_lifecycle_change(node, monkeypatch, changed):
    _, snapshot, prior = source(node, monkeypatch)
    surface = node._near_field_fused_surface
    actual = remote.qualify_regular_box_snapshot

    def takeover(*args, **kwargs):
        result = actual(*args, **kwargs)
        assert result.ok
        if changed == 'phase':
            node._near_field_phase_id += 1
        else:
            node._stream_generation += 1
        return result

    monkeypatch.setattr(remote, 'qualify_regular_box_snapshot', takeover)
    geometry, _, _ = prepare(node, snapshot)
    assert not geometry.ok and geometry.failure_code == 'TARGET_GEOMETRY_GENERATION_STALE'
    assert node._near_field_fused_surface is surface
    assert node._latest_target_observation is prior
    assert node._latest_box_registration_choice is None
    assert node._latest_fused_scene_audit is None


def test_source_qualification_holds_neither_stream_nor_geometry_lock(node, monkeypatch):
    _, snapshot, _ = source(node, monkeypatch)
    actual = remote.qualify_regular_box_snapshot
    acquired = threading.Event()

    def qualify(*args, **kwargs):
        def callback():
            with node._stream_condition, node._geometry_state_guard():
                acquired.set()
        worker = threading.Thread(target=callback)
        worker.start()
        assert acquired.wait(1.), 'source qualification held a callback lock'
        worker.join(1.)
        assert not worker.is_alive()
        return actual(*args, **kwargs)

    monkeypatch.setattr(remote, 'qualify_regular_box_snapshot', qualify)
    geometry, _, _ = prepare(node, snapshot)
    assert acquired.is_set() and geometry.ok


@pytest.mark.parametrize('unavailable', ['missing_original', 'wrong_source', 'bilateral_policy'])
def test_unavailable_box_choice_preserves_original_registered_measured_path(
        node, monkeypatch, unavailable):
    _, snapshot, _ = source(node, monkeypatch)
    if unavailable == 'missing_original':
        snapshot = replace(snapshot, original_sample=None)
    elif unavailable == 'wrong_source':
        snapshot = replace(snapshot, original_sample=replace(
            snapshot.original_sample, stamp_ns=snapshot.stamp_ns - 1))
    else:
        monkeypatch.setattr(node, '_box_requires_measured_bilateral_contact', lambda: True)

    def forbidden(*args, **kwargs):
        pytest.fail('an unavailable raw box must not alter registration selection')
    monkeypatch.setattr(remote, 'append_box_consistent_registered_view', forbidden)
    geometry, _, _ = prepare(node, snapshot)
    assert geometry.ok
    assert node._latest_registration_evidence.result.ok
    assert node._near_field_fused_surface.view_stamps_ns[-1] == snapshot.stamp_ns
    assert node._latest_box_registration_choice is None
    assert 'registration_choice' not in node._latest_fused_scene_audit


def test_reached_context_uses_owned_raw_geometry_and_original_calibration(node, monkeypatch):
    scene, snapshot, _ = source(node, monkeypatch)
    geometry, _, _ = prepare(node, snapshot)
    observation = node._latest_target_observation
    raw = node._latest_raw_geometry_source
    assert raw['geometry'] is scene.geometry and raw['geometry'] is not geometry
    monkeypatch.setattr(node, '_camera_intrinsics',
        lambda: pytest.fail('reached source must use its original intrinsics'))
    context = node._prepare_reached_box_registration(observation)
    assert context is not None and node._reached_box_registration_current(observation, context)
    assert context['inputs']['raw_geometry'] is scene.geometry
    assert context['inputs']['raw_box_model'].source['stamp_ns'] == snapshot.stamp_ns


@pytest.mark.parametrize('when', ['before_qualification', 'during_qualification',
                                 'after_qualification'])
@pytest.mark.parametrize('takeover', ['phase', 'generation', 'same_target_new_owner'])
def test_reached_context_distinguishes_initial_unavailability_from_revocation(
        node, monkeypatch, when, takeover):
    _, snapshot, _ = source(node, monkeypatch)
    prepare(node, snapshot)
    observation = node._latest_target_observation
    surface = node._near_field_fused_surface
    actual = remote.qualify_regular_box_snapshot

    def change():
        if takeover == 'phase':
            node._near_field_phase_id += 1
        elif takeover == 'generation':
            node._stream_generation += 1
        else:
            newer = replace(observation, stamp_ns=observation.stamp_ns + 1)
            node._latest_target_observation = newer
            node._latest_raw_geometry_source = dict(node._latest_raw_geometry_source,
                observation=newer)

    if when == 'before_qualification':
        change()
    elif when == 'during_qualification':
        def interrupted(*args, **kwargs):
            result = actual(*args, **kwargs)
            assert result.ok
            change()
            return result
        monkeypatch.setattr(remote, 'qualify_regular_box_snapshot', interrupted)
    context = node._prepare_reached_box_registration(observation)
    assert context is not None
    if when == 'before_qualification':
        # The callback owns the earlier lifecycle check. The helper cannot
        # infer revocation merely from a cache already unavailable at entry.
        assert context['inputs'] == {}
        # This fixture derives target identity from stream generation. A
        # changed target still fails the original identity check at entry.
        assert node._reached_box_registration_current(observation, context) == (
            takeover != 'generation')
    else:
        if when == 'after_qualification':
            change()
        assert not node._reached_box_registration_current(observation, context)
    assert node._near_field_fused_surface is surface


def test_initially_cleared_source_preserves_measured_path_until_owner_changes(node, monkeypatch):
    _, snapshot, _ = source(node, monkeypatch)
    prepare(node, snapshot)
    observation = node._latest_target_observation
    node._latest_raw_geometry_source = None
    node._latest_target_observation = replace(observation, stamp_ns=observation.stamp_ns + 1)
    context = node._prepare_reached_box_registration(observation)
    assert context is not None
    assert context['inputs'] == {}
    assert node._reached_box_registration_current(observation, context)
    node._latest_target_observation = replace(observation, stamp_ns=observation.stamp_ns + 2)
    assert not node._reached_box_registration_current(observation, context)


@pytest.mark.parametrize('availability', ['missing', 'older_source', 'alias_owner'])
@pytest.mark.parametrize('changed_owner', ['raw_source', 'latest_observation'])
def test_unavailable_context_retains_measured_path_and_detects_later_replacement(
        node, monkeypatch, availability, changed_owner):
    scene, snapshot, prior = source(node, monkeypatch)
    observation = observation_from_snapshot(snapshot, scene.geometry, scene.T, 'base_link')
    node._latest_target_observation = observation
    node._latest_raw_geometry_source = None
    if availability == 'older_source':
        node._latest_raw_geometry_source = dict(observation=prior,
            lifecycle=node._multiview_lifecycle_token())
    elif availability == 'alias_owner':
        node._latest_raw_geometry_source = dict(observation=replace(observation),
            lifecycle=node._multiview_lifecycle_token())
    monkeypatch.setattr(node, '_raw_box_registration_model',
        lambda *_args, **_kwargs: pytest.fail('unowned cache cannot qualify a box'))
    context = node._prepare_reached_box_registration(observation)
    assert context['inputs'] == {} and context['raw_source_matched'] is False
    assert node._reached_box_registration_current(observation, context)
    if changed_owner == 'raw_source':
        node._latest_raw_geometry_source = dict(observation=observation,
            lifecycle=node._multiview_lifecycle_token())
    else:
        node._latest_target_observation = replace(observation)
    assert not node._reached_box_registration_current(observation, context)


def test_real_reached_resolver_returns_the_prepared_raw_source_owner(node, monkeypatch):
    scene, snapshot, _ = source(node, monkeypatch)
    node.near_field_planning_active = False
    node._latest_target_observation = None
    node.frames = remote.SynchronizedRgbdBuffer()
    monkeypatch.setattr(node, '_active_profile_requires_mask', lambda: True)
    monkeypatch.setattr(node, '_snapshot_depth_config', lambda: (.0001, .03, 2.))

    def exact(stamp_ns, identity, **kwargs):
        assert stamp_ns == snapshot.stamp_ns and identity == snapshot.target_identity
        assert kwargs['require_mask'] is True
        return snapshot.original_sample
    monkeypatch.setattr(node.frames, 'wait_for_exact_sample', exact)
    msg = SimpleNamespace(reference_center_valid=True,
        reference_target_track_id=scene.identity.track_id,
        reference_source_stamp=remote.rospy.Time(*divmod(scene.stamp, 1_000_000_000)),
        header=SimpleNamespace(stamp=remote.rospy.Time(20, 200_000_000)),
        reference_center_base=SimpleNamespace(**dict(zip('xyz', scene.geometry.center_base))))
    observation = node._resolve_reached_view_observation(msg)
    assert observation is not None
    assert observation is node._latest_target_observation
    assert observation is node._latest_raw_geometry_source['observation']
    context = node._prepare_reached_box_registration(observation)
    assert context is not None and context['inputs']['raw_box_model'] is not None
    assert node._reached_box_registration_current(observation, context)


def test_saved_preappend_receipt_restores_complete_history_and_reference(node, monkeypatch):
    from test_gen38_remote_replay_handoff import restore_surface, assert_exact_surface
    _, snapshot, _ = source(node, monkeypatch)
    before = node._active_multiview_surface()
    prepare(node, snapshot)
    receipt = node._latest_fused_scene_audit['surface_replay']
    assert receipt['status'] == 'stored'
    reference = receipt['references']['before_registration_reference']
    stored = remote.DiagnosticStore(reference['store_root']).get(reference)
    assert stored['coordinate_frame'] == 'preappend_phase_reference_base_estimate'
    assert stored['snapshot_stamp_ns'] == snapshot.stamp_ns
    assert stored['request_binding']['current_source_not_appended'] is True
    assert stored['execution_authorized'] is False
    assert snapshot.stamp_ns not in stored['view_stamps_ns']
    assert_exact_surface(restore_surface(stored), before)
