"""Invocation-local reuse cannot change view evidence or survive a new camera."""
from copy import deepcopy
from types import SimpleNamespace as NS
import numpy as np
import pytest

from test_remote_modes import configured, base
from test_observation_information_selection import information_context, candidate_view, qualify_variants


def test_many_directed_directions_do_not_repeat_camera_rpc_or_pose_conversion(configured, monkeypatch):
    node, geometry, _, camera = information_context(configured, monkeypatch)
    variants = [candidate_view(geometry, .012, distance=d) for d in (.2, .3, .4)]
    matching = base.pose_matrix(variants[0]['sequence'].pregrasp)[:3, :3]
    # Preserve the first matching direction's metadata, even after many misses.
    node._observe_return_directed_views = tuple((np.eye(3), {'sentinel': i}) for i in range(20)) + (
        (matching, {'sentinel': 'first'}), (matching, {'sentinel': 'second'}))
    calls = {'camera': 0, 'pose': 0}
    original = base.pose_matrix
    def intrinsics():
        calls['camera'] += 1
        return camera
    def pose(value):
        calls['pose'] += 1
        return original(value)
    monkeypatch.setattr(node, '_camera_intrinsics', intrinsics)
    monkeypatch.setattr(base, 'pose_matrix', pose)
    passed, rejected = qualify_variants(node, geometry, variants, monkeypatch)
    assert len(passed) == 3 and not rejected
    assert calls == {'camera': 1, 'pose': 6}  # volume + independent live gain
    for candidate in passed:
        audit = candidate['observation_view']
        assert audit['observe_return_directed_missing_side']['sentinel'] == 'first'
        assert audit['working_volume']['ok'] is True
        assert audit['observe_return_missing_side_gain']['ok'] is True
        assert audit['information_utility']['prediction_only'] is True


def test_next_invocation_reads_changed_intrinsics_and_retains_candidate_order(configured, monkeypatch):
    node, geometry, _, camera = information_context(configured, monkeypatch)
    variants = [candidate_view(geometry, .012, distance=d) for d in (.2, .3, .4)]
    first, rejected = qualify_variants(node, geometry, deepcopy(variants), monkeypatch)
    assert not rejected
    camera.fy *= .5
    second, rejected = qualify_variants(node, geometry, deepcopy(variants), monkeypatch)
    assert not rejected
    assert len(first) == len(second) == 3
    for a, b in zip(first, second):
        np.testing.assert_array_equal(base.pose_matrix(a['sequence'].pregrasp),
                                      base.pose_matrix(b['sequence'].pregrasp))
        assert a['observe_return_missing_side_gain'] == b['observe_return_missing_side_gain']
        assert b['observation_view']['information_utility']['utility'] < a['observation_view']['information_utility']['utility']


def cached_context(configured, monkeypatch):
    node, geometry, prepared, camera = information_context(configured, monkeypatch)
    camera.depth_scale = .001
    node._observe_return_view_uncertainty_m = .003
    source = NS(stamp_ns=10_000_000_000)
    reference = object()
    cache = {}
    templates = [candidate_view(geometry, .012, distance=d) for d in (.2, .3, .4)]
    state = dict(geometry=geometry, prepared=prepared, source=source, reference=reference,
        templates=templates, family=0, tool=np.eye(4))
    monkeypatch.setattr(node, '_tool_from_camera_matrix', lambda: state['tool'])
    counts = {'gain': 0}
    actual_gain = node._observe_return_view_gain
    def gain(sequence):
        counts['gain'] += 1
        return actual_gain(sequence)
    monkeypatch.setattr(node, '_observe_return_view_gain', gain)
    def base_variants(*args, **kwargs):
        result = deepcopy(state['templates'])
        for candidate in result:
            candidate['sequence'].grasp = NS(contact_family=state['family'])
        cache.update(prepared=state['prepared'], geometry=state['geometry'],
            snapshot=state['source'], reference=state['reference'])
        return tuple(result), ()
    monkeypatch.setattr(base.RemoteGrasp6DNode, '_far_field_observation_variants', base_variants)
    def run():
        return node._far_field_observation_variants(geometry=state['geometry'],
            snapshot=state['source'], prepared=state['prepared'],
            observation_reference_pose=state['reference'], view_cache=cache)
    return node, camera, state, cache, counts, run


def test_request_cache_reuses_only_endpoint_evidence_and_returns_independent_audits(configured, monkeypatch):
    node, _, state, cache, counts, run = cached_context(configured, monkeypatch)
    first, rejected = run()
    assert len(first) == 3 and not rejected and counts['gain'] == 3
    state['family'] = 1
    second, rejected = run()
    assert len(second) == 3 and not rejected and counts['gain'] == 3
    for before, after in zip(first, second):
        assert before['observation_view'] == after['observation_view']
        assert before['observe_return_missing_side_gain'] == after['observe_return_missing_side_gain']
        assert before['sequence'].grasp.contact_family == 0
        assert after['sequence'].grasp.contact_family == 1
    second[0]['observation_view']['information_utility']['utility'] = -1.
    second[0]['sequence'].observation_view_audit['working_volume']['ok'] = False
    second[0]['observe_return_missing_side_gain']['ok'] = False
    third, _ = run()
    assert counts['gain'] == 3
    assert third[0]['observation_view'] == first[0]['observation_view']
    assert third[0]['sequence'].observation_view_audit == first[0]['sequence'].observation_view_audit
    assert third[0]['observe_return_missing_side_gain'] == first[0]['observe_return_missing_side_gain']


@pytest.mark.parametrize('changed', [
    'fx', 'fy', 'cx', 'cy', 'width', 'height', 'depth_scale', 'limits',
    'handeye', 'corners', 'source_stamp', 'selection', 'identity',
    'gap', 'visited', 'directed', 'source_uncertainty', 'profile_uncertainty',
    'pregrasp', 'baseline', 'base_audit', 'base_envelope',
    'prepared_object', 'geometry_object', 'snapshot_object', 'reference_object',
])
def test_each_endpoint_input_change_invalidates_request_cache(configured, monkeypatch, changed):
    node, camera, state, cache, counts, run = cached_context(configured, monkeypatch)
    first, rejected = run()
    assert len(first) == 3 and not rejected and counts['gain'] == 3
    if changed in ('fx', 'fy', 'cx', 'cy', 'depth_scale'):
        setattr(camera, changed, getattr(camera, changed) + .001)
    elif changed in ('width', 'height'):
        setattr(camera, changed, getattr(camera, changed) + 1)
    elif changed == 'limits':
        monkeypatch.setattr(node, '_observation_view_limit_configuration', lambda: {
            'effective_visibility_limits': dict(roi_margin_px=37., minimum_depth_m=.07, maximum_depth_m=.5)})
    elif changed == 'handeye': state['tool'][0, 3] += .00001
    elif changed == 'corners': state['geometry'].size_xyz_m[0] += .00001
    elif changed == 'source_stamp': state['source'].stamp_ns += 1
    elif changed == 'selection': node._mode_selection = dict(node._mode_selection, generation=4)
    elif changed == 'identity':
        identity = node._current_stream_target_identity()
        monkeypatch.setattr(node, '_current_stream_target_identity', lambda: NS(epoch=identity.epoch+1, track_id='other'))
    elif changed == 'gap':
        node._observe_return_view_gaps = tuple(dict(g, object_height_m=g['object_height_m']+.00001)
                                              for g in node._observe_return_view_gaps)
    elif changed == 'visited': node._observe_return_visited_camera_origins = ([100., 100., 100.],)
    elif changed == 'directed': node._observe_return_directed_views = ((np.eye(3), {'changed': True}),)
    elif changed == 'source_uncertainty': node._observe_return_view_uncertainty_m += .00001
    elif changed == 'profile_uncertainty': state['templates'][0]['sequence'].adaptive_stage_profile.depth_uncertainty_m += .00001
    elif changed == 'pregrasp': state['templates'][0]['sequence'].pregrasp.pose.position.x += .00001
    elif changed == 'baseline': state['templates'][0]['observation_translation_delta_m'] += .00001
    elif changed == 'base_audit': state['templates'][0]['sequence'].observation_view_audit['new_evidence'] = True
    elif changed == 'base_envelope': state['templates'][0]['observation_envelope'] = base.ObservationEnvelopeResult(True, '', '', .019)
    elif changed == 'prepared_object': state['prepared'] = deepcopy(state['prepared'])
    elif changed == 'geometry_object': state['geometry'] = deepcopy(state['geometry'])
    elif changed == 'snapshot_object': state['source'] = deepcopy(state['source'])
    elif changed == 'reference_object': state['reference'] = object()
    run()
    assert counts['gain'] == 6, changed
