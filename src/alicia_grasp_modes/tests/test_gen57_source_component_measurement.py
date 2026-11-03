"""Current-measurement recovery across independent geometry and refusal cases."""
from dataclasses import replace
import math

import cv2
import numpy as np
import pytest

from alicia_grasp_modes import source_connected_owner as owner
from alicia_grasp_modes.rgb_instance import freeze_observation_spatial_reference
from alicia_grasp_modes.source_sampling_topology import SourceSamplingTopology
from alicia_grasp_modes.tabletop import Config, Segmentation, _matching_candidates, _project
from test_gen30_source_connected_owner import _candidate, source_scene
from test_gen30_source_sampling_topology import identity_camera


def measured_scene(size=(.06, .042, .024), yaw=0., fragments=2):
    """Ray-cast one finite prism; split its real current samples, not geometry."""
    cv2.setNumThreads(1)
    shape = (240, 320)
    K = np.array([420., 420., 160., 120.])
    origin = np.array([0., -.30, .25])
    forward = np.array([0., .30, -.235])
    forward /= np.linalg.norm(forward)
    left = np.array([-1., 0., 0.])
    T = np.eye(4)
    T[:3, :3] = np.column_stack((forward, left, np.cross(forward, left)))
    T[:3, 3] = origin
    yy, xx = np.indices(shape)
    rays = _project(np.ones(shape, np.float32), K).astype(float)
    world = rays @ T[:3, :3].T
    table_depth = -origin[2] / world[..., 2]
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    object_origin, object_rays = origin @ rotation, world @ rotation
    low = np.array([-size[0]/2., -size[1]/2., 0.])
    high = np.array([size[0]/2., size[1]/2., size[2]])
    with np.errstate(divide='ignore', invalid='ignore'):
        times_a = (low - object_origin) / object_rays
        times_b = (high - object_origin) / object_rays
    entries, exits = np.minimum(times_a, times_b), np.maximum(times_a, times_b)
    near, far = np.max(entries, axis=-1), np.min(exits, axis=-1)
    truth = (near > 0.) & (near <= far) & (near < table_depth)
    top = truth & (np.argmax(entries, axis=-1) == 2)
    depth = np.where(truth, near, table_depth).astype(np.float32)
    rgb = np.full(shape + (3,), (55, 100, 185), np.uint8)
    rgb[truth] = (215, 225, 205)
    plane = np.array([0., 0., 1., 0.])
    config = Config()
    points = _project(depth, K)
    pc = T.T @ plane
    heights = np.einsum('ijk,k->ij', points, pc[:3]) + pc[3]
    measured = truth & (heights >= config.foreground_min_m)
    full = _candidate(truth.astype(np.uint8)*255, depth, K, T, plane)
    ys, xs = np.nonzero(measured)
    diagonal = (xx-xs.mean()) / max(1, np.ptp(xs)) + (yy-ys.mean()) / max(1, np.ptp(ys))
    masks = [measured & (diagonal <= 0), measured & (diagonal > 0)]
    if fragments == 3:
        # Give the third real measured fragment enough interior to satisfy
        # the unchanged native trusted-core gate.
        strip = measured & (xx < np.quantile(xs, .25))
        masks = [mask & ~strip for mask in masks] + [strip]
    assert fragments in (2, 3) and all(mask.sum() >= config.min_object_points for mask in masks)
    candidates = [_candidate(mask.astype(np.uint8)*255, depth, K, T, plane) for mask in masks]
    result = Segmentation(candidates, plane, {'support_current_audit_available': True})
    assert len(_matching_candidates(candidates, full, full, config)) >= 2
    camera = identity_camera(shape, K)
    arguments = dict(result=result, rgb=rgb, depth=depth, intrinsics=K, transform=T,
        config=config, anchor=full, last=full,
        spatial_reference=freeze_observation_spatial_reference(full, plane),
        primary_refined=None, minimum_quality=.5,
        source_topology=SourceSamplingTopology(camera), camera=camera, source_stamp_ns=100,
        rgb_valid=np.ones(shape, bool), check_current=lambda: None)
    return dict(truth=truth, measured=measured, top=top, plane=plane,
                table_depth=table_depth, arguments=arguments)


def recover(scene):
    return owner.recover_source_connected_measurement(**scene['arguments'])


@pytest.mark.parametrize('size,yaw,fragments', [
    ((.06, .042, .024), 0., 2),
    ((.08, .052, .030), .35, 2),
    ((.05, .035, .018), -.42, 2),
    ((.06, .042, .024), 0., 3),
])
def test_complete_measurement_preserves_real_geometry_across_size_pose_and_fragment_count(size, yaw, fragments):
    scene = measured_scene(size, yaw, fragments)
    a = scene['arguments']
    before = [candidate.mask.copy() for candidate in a['result'].candidates]
    depth = a['depth'].copy()
    outcome = recover(scene)
    assert outcome is not None and outcome.original is a['result']
    assert outcome.normalized_result is not outcome.original
    assert outcome.normalized_result.candidates == [outcome.raw]
    assert len(outcome.audit['original_metric_match_indices']) >= 2
    assert outcome.audit['member_indices'] == list(range(fragments))
    assert len(outcome.audit['original_candidate_records']) == fragments
    assert all(row['retained_fraction'] >= .85 for row in outcome.audit['native_members'])
    assert not np.any((outcome.refined.mask > 0) & ~scene['truth'])
    assert np.mean(outcome.refined.mask[scene['truth']] > 0) >= .85
    assert np.linalg.norm(outcome.raw.position_base-outcome.refined.position_base) <= .025
    assert outcome.audit['depth_filled'] is False
    assert outcome.audit['motion_authorized'] is False
    assert outcome.audit['fixed_station_identity_required'] is True
    for original, mask in zip(a['result'].candidates, before):
        np.testing.assert_array_equal(original.mask, mask)
    np.testing.assert_array_equal(depth, a['depth'])
    assert outcome.require_unchanged() is outcome


def _foreign(scene, y, x, width=12):
    a = scene['arguments']
    mask = np.zeros(a['depth'].shape, bool)
    mask[y:y+width, x:x+width] = True
    assert not np.any(mask & scene['truth'])
    rays = _project(np.ones(mask.shape, np.float32), a['intrinsics'])
    pc = a['transform'].T @ scene['plane']
    depth = ((.028-pc[3]) / np.einsum('ijk,k->ij', rays, pc[:3])).astype(np.float32)
    a['depth'][mask] = depth[mask]
    a['rgb'][mask] = (215, 225, 205)
    candidate = _candidate(mask.astype(np.uint8)*255, a['depth'], a['intrinsics'],
                           a['transform'], scene['plane'])
    return mask, candidate


@pytest.mark.parametrize('order', [(0, 1, 2, 3), (3, 2, 1, 0), (1, 3, 0, 2)])
def test_candidate_permutations_keep_all_nonmember_objects_and_original_order(order):
    scene = measured_scene()
    a = scene['arguments']
    _, left = _foreign(scene, 20, 20)
    _, right = _foreign(scene, 20, 280)
    first, second = a['result'].candidates
    values = [left, first, right, second]
    a['result'].candidates[:] = [values[index] for index in order]
    original = tuple(a['result'].candidates)
    outcome = recover(scene)
    assert outcome is not None
    assert tuple(a['result'].candidates) == original
    nonmembers = [candidate for candidate in original if candidate is left or candidate is right]
    actual = [candidate for candidate in outcome.normalized_result.candidates if candidate is not outcome.raw]
    assert len(actual) == len(nonmembers) == 2
    assert all(a is b for a, b in zip(actual, nonmembers))
    assert all(original[index] is not outcome.raw for index in outcome.audit['member_indices'])
    assert len(_matching_candidates(outcome.normalized_result.candidates, a['anchor'],
                                    a['last'], a['config'])) == 1
    outcome.require_unchanged()


@pytest.mark.parametrize('separation', ['missing_column', 'diagonal_only'])
def test_distinct_occupied_source_components_do_not_become_one_measurement(separation):
    scene = measured_scene()
    a = scene['arguments']
    yy, xx = np.indices(a['depth'].shape)
    ys, xs = np.nonzero(scene['measured'])
    signed = xx-int(np.median(xs)) if separation == 'missing_column' else xx+yy-int(np.median(xs+ys))
    masks = [scene['measured'] & (signed < 0), scene['measured'] & (signed > 0)]
    missing = scene['measured'] & (signed == 0)
    a['depth'][missing] = 0.
    a['result'].candidates[:] = [_candidate(mask.astype(np.uint8)*255, a['depth'],
        a['intrinsics'], a['transform'], scene['plane']) for mask in masks]
    assert len(_matching_candidates(a['result'].candidates, a['anchor'], a['last'], a['config'])) == 2
    original = tuple(a['result'].candidates)
    assert recover(scene) is None
    assert all(a is b for a, b in zip(a['result'].candidates, original))


def test_raw_only_single_pixel_bridge_is_rejected_before_refinement(monkeypatch):
    scene = measured_scene()
    a = scene['arguments']
    y = int(np.median(np.nonzero(scene['top'])[0]))
    edge = int(np.flatnonzero(scene['top'][y])[0])
    extra = np.zeros(a['depth'].shape, bool)
    extra[y-6:y+6, edge-19:edge-7] = True
    extra[y, edge-7:edge] = True
    assert not np.any(extra & scene['truth'])
    rays = _project(np.ones(extra.shape, np.float32), a['intrinsics'])
    pc = a['transform'].T @ scene['plane']
    depth = ((.024-pc[3])/np.einsum('ijk,k->ij', rays, pc[:3])).astype(np.float32)
    a['depth'][extra] = depth[extra]
    a['rgb'][extra] = (215, 225, 205)
    monkeypatch.setattr(owner._FrameRefinement, 'refine',
        lambda *args, **kwargs: pytest.fail('raw bridge reached RGB growth'))
    with pytest.raises(ValueError, match='single_pixel_measured_bridge'):
        recover(scene)


@pytest.mark.parametrize('overlap', [1, 144])
def test_foreign_raw_component_is_not_hidden_by_candidate_prefilter(monkeypatch, overlap):
    scene = measured_scene()
    actual = recover(scene)
    foreign, _ = _foreign(scene, 20, 20)
    mask = actual.refined.mask.copy()
    mask.ravel()[np.flatnonzero(foreign)[:overlap]] = 255
    monkeypatch.setattr(owner._FrameRefinement, 'refine',
        lambda *args, **kwargs: replace(actual.refined, mask=mask))
    with pytest.raises(ValueError, match='foreign_raw_source_component_overlap'):
        recover(scene)


def test_a_whole_member_core_cannot_be_dropped(monkeypatch):
    scene = measured_scene()
    actual = recover(scene)
    mask = actual.refined.mask.copy()
    mask[scene['arguments']['result'].candidates[0].mask > 0] = 0
    monkeypatch.setattr(owner._FrameRefinement, 'refine',
        lambda *args, **kwargs: replace(actual.refined, mask=mask))
    with pytest.raises(ValueError, match='native_core_not_owned'):
        recover(scene)


@pytest.mark.parametrize('invalid', ['topology', 'intrinsics', 'activation', 'support',
    'native_overlap', 'common_metadata', 'quality', 'candidate_budget', 'spatial'])
def test_invalid_source_or_measurement_evidence_is_not_optional_success(invalid):
    scene = measured_scene()
    a = scene['arguments']
    if invalid == 'topology':
        a['source_topology'] = None
    elif invalid == 'intrinsics':
        a['intrinsics'] = a['intrinsics'] + [1., 0., 0., 0.]
    elif invalid == 'activation':
        a['camera']['runtime_profile']['geometry_activation_stamp_ns'] = '100'
    elif invalid == 'support':
        a['result'].metrics['support_current_audit_available'] = False
    elif invalid == 'native_overlap':
        a['result'].candidates[1].mask |= a['result'].candidates[0].mask
    elif invalid == 'common_metadata':
        a['result'].candidates[1].metrics['support_residual_m'] += .0001
    elif invalid == 'quality':
        a['result'].candidates[1].quality = .1
    elif invalid == 'candidate_budget':
        a['result'].candidates *= 9
    elif invalid == 'spatial':
        a['spatial_reference'] = None
    with pytest.raises(ValueError):
        recover(scene)


@pytest.mark.parametrize('stage', ['source_scope_and_map', 'source_four_neighbor_component',
    'raw_source_component_evidence', 'source_four_neighbor_articulation',
    'complete_original_rgb_refinement', 'complete'])
def test_typed_owner_or_deadline_failure_propagates_without_a_proposal(stage):
    scene = measured_scene()
    current = {'expired': False}
    class ExpiredSource(ValueError):
        pass
    def check():
        if current['expired']:
            raise ExpiredSource('original_owner_or_deadline')
    def record(name):
        if name == stage:
            current['expired'] = True
    scene['arguments'].update(check_current=check,
        record_stage=record)
    with pytest.raises(ExpiredSource, match='original_owner_or_deadline'):
        recover(scene)


@pytest.mark.parametrize('changed', ['original_mask', 'original_position', 'original_metrics',
    'original_candidate_identity', 'rgb', 'depth', 'transform', 'config', 'camera',
    'anchor', 'last', 'spatial', 'raw', 'refined', 'normalized', 'audit', 'topology'])
def test_post_proof_input_or_output_mutation_is_rejected(changed):
    scene = measured_scene()
    a = scene['arguments']
    value = recover(scene)
    value.require_unchanged()
    if changed == 'original_mask':
        a['result'].candidates[0].mask.flat[0] ^= 1
    elif changed == 'original_position':
        a['result'].candidates[0].position_base[0] += .0001
    elif changed == 'original_metrics':
        a['result'].candidates[0].metrics['valid_points'] += 1
    elif changed == 'original_candidate_identity':
        a['result'].candidates[0] = replace(a['result'].candidates[0])
    elif changed == 'rgb':
        a['rgb'].flat[0] ^= 1
    elif changed == 'depth':
        a['depth'].flat[0] += .0001
    elif changed == 'transform':
        a['transform'][0, 3] += .0001
    elif changed == 'config':
        a['config'].association_distance_m += .001
    elif changed == 'camera':
        a['camera']['fx'] += 1.
    elif changed == 'anchor':
        a['anchor'].position_base[0] += .0001
    elif changed == 'last':
        a['last'].position_base[1] += .0001
    elif changed == 'spatial':
        a['spatial_reference']['height_m'] += .0001
    elif changed == 'raw':
        value.raw.mask.flat[0] ^= 1
    elif changed == 'refined':
        value.refined.position_base[0] += .0001
    elif changed == 'normalized':
        value.normalized_result.metrics['support_current_audit_available'] = False
    elif changed == 'audit':
        value.audit['source_stamp_ns'] += 1
    elif changed == 'topology':
        a['source_topology'].map_sha256 = '0'*64
    with pytest.raises(ValueError, match='current_measurement_'):
        value.require_unchanged()


def test_old_single_match_api_retains_its_behavior_and_new_api_does_not_take_over():
    scene, arguments = source_scene()
    old = owner.recover_source_connected_fragment_owner(**arguments)
    assert old is not None and old.audit['original_metric_matches'] == 1
    assert owner.recover_source_connected_measurement(**arguments) is None
    assert not np.any((old.owner.mask > 0) & ~scene['truth'])
