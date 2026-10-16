"""Independent frozen arrival-prism/source guards; no robot or image mutation."""
import base64
from copy import deepcopy
import hashlib
import json
from types import SimpleNamespace as NS
import zlib

import numpy as np
import pytest

from test_remote_modes import configured, base, module as remote_module
from test_reachable_observation_views import recorded_inputs
from test_reachable_observation_integration import context, recorded_proposal
from alicia_grasp_modes import observation_arrival_projection as projection
from alicia_grasp_modes import reachable_observation_views as reachable
from alicia_flexible_grasp.vision import rgbd_rectification as rectification


def receipt():
    selection = dict(mode='unknown', strategy='observe_return', generation=2, stamp_ns=1000000000)
    status = dict(mode='unknown', generation=2, selection_stamp_ns=1000000000,
        state='ready', source_stamp_ns=3000000000, anchor_source_stamp_ns=2000000000,
        target_locked=True, target_lost=False, first_loss=None, exact_rgbd=True,
        projection_frame_convention='ros_camera_link', color_stamp_ns=3000000000,
        depth_stamp_ns=3000000000, tf_lookup_stamp_ns=3000000000,
        color_frame_id='camera_link', depth_frame_id='camera_link',
        anchor_position_base=[.29, 0., 0.], anchor_extent_m=[.02, .04, .04],
        reference_support_plane_base=[-1., 0., 0., .3],
        reference_support_footprint_base=[[.3, -.02, -.02], [.3, .02, -.02],
                                          [.3, .02, .02], [.3, -.02, .02]],
        reference_object_height_m=.02)
    camera = dict(width=320, height=240, fx=240., fy=240., cx=160., cy=120.)
    return status, selection, camera


def capture(status=None, selection=None, camera=None):
    original_status, original_selection, original_camera = receipt()
    return projection.capture_arrival_projection(status or original_status,
        selection or original_selection, 3000000000, camera or original_camera)


def check(contract, transform=None, **kwargs):
    return projection.validate_arrival_projection(contract,
        np.eye(4) if transform is None else transform, [240., 240., 160., 120.], 320, 240, **kwargs)


def camera_with_validity(valid):
    camera = receipt()[2]
    mapper = rectification.PairedPinholeRectification(dict(camera, model='none', coeffs=[0.] * 5))
    contract = deepcopy(mapper.contract)
    contract['rgb_valid_mask_zlib_base64'] = base64.b64encode(
        zlib.compress(np.packbits(valid).tobytes())).decode('ascii')
    contract['rgb_valid_mask_sha256'] = hashlib.sha256(np.ascontiguousarray(valid).tobytes()).hexdigest()
    contract.pop('digest')
    contract['digest'] = rectification._digest(contract)
    camera['runtime_profile_geometry'] = dict(rectification=contract,
        geometry_intrinsics=mapper.output_intrinsics, geometry_activation_stamp_ns=1500000000)
    return camera


def test_real_acquired_producer_publishes_exact_anchor_height_and_capture_is_immutable(monkeypatch):
    from test_locked_rgb_hold import locked, latest_status
    node, _, _, _, params = locked(monkeypatch, initial_station=True)
    status = latest_status(node)
    assert status['reference_object_height_m'] == node.tracker.anchor.metrics['median_height_m']
    contract = projection.capture_arrival_projection(status, params['/grasp_mode/selection'],
        status['source_stamp_ns'], params['/camera'])
    audit = contract['audit']
    assert audit['source_stamp_ns'] == status['source_stamp_ns']
    assert audit['anchor_source_stamp_ns'] == node.anchor_source_stamp_ns
    assert audit['measured_height_m'] == node.tracker.anchor.metrics['median_height_m']
    for key in ('prism_base', 'rgb_valid', 'invalid_integral'):
        assert not contract[key].flags.writeable
    before = contract['prism_base'].copy()
    status['reference_support_footprint_base'][0][0] += .1
    np.testing.assert_array_equal(contract['prism_base'], before)
    assert audit['historical_pixels_counted'] == 0 and audit['execution_authority'] is False
    assert audit['certifies_arrival_or_tracking'] is False


@pytest.mark.parametrize('field,value', [
    ('state', 'pending'), ('source_stamp_ns', 3000000001), ('generation', 3),
    ('selection_stamp_ns', 1000000001), ('anchor_source_stamp_ns', 4000000000),
    ('target_locked', False), ('target_lost', True), ('first_loss', {'reason': 'retired'}),
    ('exact_rgbd', False), ('color_stamp_ns', 2999999999), ('depth_stamp_ns', 2999999999),
    ('tf_lookup_stamp_ns', 2999999999), ('reference_object_height_m', None),
    ('reference_object_height_m', True), ('reference_object_height_m', float('nan')),
    ('reference_object_height_m', .00399), ('reference_object_height_m', .12001),
    ('reference_support_plane_base', [0., 0., 0., 0.]),
    ('reference_support_footprint_base', [[.3, 0., 0.]] * 3),
    ('reference_support_footprint_base', [[.301, -.02, -.02], [.3, .02, -.02], [.3, .02, .02]])])
def test_incomplete_or_foreign_source_cannot_form_prediction(field, value):
    status, _, _ = receipt(); status[field] = value
    with pytest.raises((ValueError, TypeError)):
        capture(status=status)


@pytest.mark.parametrize('strategy', ['direct', 'two_stage'])
def test_non_observation_strategy_cannot_create_new_contract(strategy):
    _, selection, _ = receipt(); selection['strategy'] = strategy
    with pytest.raises(ValueError):
        capture(selection=selection)


def test_missing_height_does_not_fall_back_to_current_obb_or_old_metadata():
    status, _, _ = receipt(); status.pop('reference_object_height_m')
    with pytest.raises(KeyError):
        capture(status=status)


def test_declared_rectification_missing_contract_and_old_geometry_source_are_rejected():
    camera = receipt()[2]; camera['pair_rectification'] = {'enabled': True}
    with pytest.raises(ValueError, match='contract_missing'):
        capture(camera=camera)
    camera = camera_with_validity(np.ones((240, 320), bool))
    camera['runtime_profile_geometry']['geometry_activation_stamp_ns'] = 3000000000
    with pytest.raises(ValueError, match='predates'):
        capture(camera=camera)


def test_convex_prism_interior_validity_is_required_even_if_all_vertices_are_observed():
    status, _, _ = receipt()
    status['reference_support_footprint_base'] = [[.3, -.024, 0.], [.3, 0., -.024],
                                                [.3, .024, 0.], [.3, 0., .024]]
    valid = np.ones((240, 320), bool); valid[120, 160] = False
    contract = capture(status=status, camera=camera_with_validity(valid))
    uv, _ = projection._project(contract['prism_base'], np.eye(4), np.array([240., 240., 160., 120.]))
    assert all(valid[int(round(v)), int(round(u))] for u, v in uv)
    with pytest.raises(ValueError, match='unobserved'):
        check(contract)


def test_derived_integral_cannot_erase_a_real_unobserved_pixel():
    valid = np.ones((240, 320), bool); valid[120, 160] = False
    contract = capture(camera=camera_with_validity(valid))
    forged = dict(contract, invalid_integral=np.zeros((241, 321), np.int64))
    with pytest.raises(ValueError):
        check(forged)


def test_invalid_padding_outside_required_projection_is_not_mistaken_for_target_occlusion():
    valid = np.ones((240, 320), bool); valid[:, :3] = False
    contract = capture(camera=camera_with_validity(valid))
    result = check(contract)
    assert result['ok'] and result['endpoint_only'] and result['unobserved_rgb_pixels'] == 0
    assert result['required_observed_box_xyxy'][0] > 3
    assert result['certifies_arrival_or_tracking'] is False
    assert result['certifies_continuous_visibility'] is False


@pytest.mark.parametrize('fault', ['nonfinite_prism', 'invalid_transform', 'too_near', 'too_far', 'left_margin',
                                  'relax_margin', 'relax_depth'])
def test_projection_cannot_relax_original_depth_or_image_limits(fault):
    contract = capture(); transform = np.eye(4); kwargs = {}
    if fault == 'nonfinite_prism':
        contract = dict(contract, prism_base=contract['prism_base'].copy()); contract['prism_base'][0, 0] = np.nan
    elif fault == 'invalid_transform': transform[0, 0] = 2.
    elif fault == 'too_near': transform[0, 3] = .25
    elif fault == 'too_far': transform[0, 3] = -.25
    elif fault == 'left_margin': transform[1, 3] = -.16
    elif fault == 'relax_margin': kwargs['roi_margin_px'] = 23.99
    else: kwargs['minimum_depth_m'] = .069
    with pytest.raises(ValueError):
        check(contract, transform, **kwargs)


def test_arrival_contract_audit_changes_invalidate_fk_cache_without_new_budget(
        configured, monkeypatch, recorded_inputs, recorded_proposal):
    node, params, source, prepared, _ = context(configured, monkeypatch, recorded_inputs)
    names = ['Joint%d' % i for i in range(1, 7)]
    params['/alicia_controller/constraints'] = {name: {'trajectory': .2, 'goal': .035} for name in names}
    node.latest_joint_state = NS(name=names, position=list(recorded_inputs['current_joint_positions']),
        header=NS(stamp=prepared.stamp))
    monkeypatch.setattr(remote_module.rospy.Time, 'now', lambda: prepared.stamp)
    monkeypatch.setattr(node, '_adaptive_stage_profiles', lambda *_: [
        NS(depth_uncertainty_m=.013, execution_position_error_m=.01)])
    contract = capture()
    source['arrival_projection'] = contract
    source['audit']['arrival_projection'] = deepcopy(contract['audit'])
    calls = []
    def generate(**kwargs):
        calls.append(kwargs)
        return (deepcopy(recorded_proposal),), dict(elapsed_sec=min(1.2, kwargs['max_seconds']))
    monkeypatch.setattr(reachable, 'generate_reachable_observation_views', generate)
    assert node._joint_space_observation_candidates(prepared)[0]
    assert node._joint_space_observation_candidates(prepared)[1]['cache_hit']
    source['audit']['arrival_projection']['measured_height_m'] = .021
    assert node._joint_space_observation_candidates(prepared)[0]
    assert len(calls) == 2 and calls[1]['max_seconds'] == pytest.approx(.8)
    source['audit']['arrival_projection']['rgb_valid_sha256'] = 'f' * 64
    rows, report = node._joint_space_observation_candidates(prepared)
    assert not rows and report['status'] == 'budget_exhausted' and len(calls) == 2


@pytest.mark.parametrize('key,value', [('prism_sha256', 'a' * 64),
    ('rgb_valid_sha256', 'b' * 64), ('invalid_integral_sha256', 'c' * 64),
    ('camera_signature', 'd' * 64)])
def test_endpoint_cache_signature_includes_exact_arrival_evidence(
        configured, monkeypatch, recorded_inputs, key, value):
    node, _, source, _, _ = context(configured, monkeypatch, recorded_inputs)
    camera = NS(**dict(source['camera'], depth_scale=.001))
    limits = source['view_limits']['effective_visibility_limits']
    args = (camera, limits, source['corners_base'], source['tool_from_camera'])
    contract = capture()
    original = node._observation_endpoint_cache_context(*args, arrival_projection=contract)
    changed = dict(contract, audit=dict(contract['audit'], **{key: value}))
    assert node._observation_endpoint_cache_context(*args, arrival_projection=changed) != original
    assert node._observation_endpoint_cache_context(*args) == node._observation_endpoint_cache_context(
        *args, arrival_projection=None)
    assert node._observation_endpoint_cache_context(*args) != original


@pytest.mark.parametrize('fault', ['missing', 'source', 'prism', 'validity', 'activation'])
def test_bound_ordinary_source_rejects_lost_or_changed_projection(configured, monkeypatch, tmp_path, fault):
    from test_unknown_observation_view_source import generic_context
    node, prepared, _, _, _ = generic_context(configured, monkeypatch, tmp_path)
    source = base._thaw_request_evidence(node._diagnostic_observation_view_source)
    assert node._require_diagnostic_observation_view_source(prepared)
    if fault == 'missing': source.pop('arrival_projection')
    elif fault == 'source': source['arrival_projection']['audit']['source_stamp_ns'] += 1
    elif fault == 'prism': source['arrival_projection']['prism_base'][0, 0] += .001
    elif fault == 'validity': source['arrival_projection']['rgb_valid'][0, 0] = False
    else:
        configured[1]['/camera']['runtime_profile_geometry'] = {'geometry_activation_stamp_ns': 1}
    node._diagnostic_observation_view_source = base._freeze_request_evidence(source)
    with pytest.raises((ValueError, KeyError)):
        node._require_diagnostic_observation_view_source(prepared)
