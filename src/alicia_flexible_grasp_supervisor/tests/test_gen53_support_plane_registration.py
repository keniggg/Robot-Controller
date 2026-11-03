"""Original G52 R18 complete history, real registration gates, no ROS or motion."""
from dataclasses import replace
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from alicia_flexible_grasp.vision import box_surface_registration as choice
from alicia_flexible_grasp.vision import multiview_surface as surface
from alicia_flexible_grasp.vision import regular_box_inference as box
from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity


@pytest.fixture(scope='module')
def recorded():
    path = Path(__file__).parent/'fixtures/gen53_g52_r18_support_registration.json'
    assert hashlib.sha256(path.read_bytes()).hexdigest() == 'd2bde5957ff5fddbfd2c24d83d59ebe45b8f24d5ceedafc4df1d251a38441618'
    data = json.loads(path.read_text())
    model = box.RegularBoxInference.from_evidence_dict(data['raw_model'])
    values = dict(data['raw_geometry'])
    for key in ('center_base', 'axes_base', 'size_xyz_m', 'support_normal_base', 'object_points_base'):
        values[key] = np.asarray(values[key])
    geometry = GeometryEstimate(**values)
    assert box.regular_box_geometry_sha256(geometry) == model.source['geometry_sha256']
    rd = dict(data['reference']); rd['identity'] = TargetTrackIdentity(**rd['identity'])
    reference = surface.SurfaceView(**rd)
    bd = dict(data['before']); bd['identity'] = TargetTrackIdentity(**bd['identity'])
    before = surface.FusedTargetSurface(**bd)
    moving = surface.SurfaceView(reference.identity, data['moving_stamp_ns'], geometry.object_points_base,
        geometry.support_normal_base, geometry.support_offset_m, data['edge_clearance_px'])
    return dict(data=data, raw_box_model=model, raw_geometry=geometry, surface=before,
        reference=reference, moving=moving, config=surface.RegistrationConfig(**data['registration_config']))


def run(recorded, **changes):
    args = {k: v for k, v in recorded.items() if k != 'data'}
    args.update(changes)
    return choice.append_box_consistent_registered_view(**args)


def baseline(recorded):
    return surface.append_registered_view(recorded['surface'], recorded['reference'],
        recorded['moving'], recorded['config'])


def assert_original(result, fused, expected, expected_fused):
    np.testing.assert_array_equal(result.transform_base, expected.transform_base)
    assert result.ok == expected.ok and result.code == expected.code
    np.testing.assert_array_equal(fused.points_base, expected_fused.points_base)
    assert fused.view_stamps_ns == expected_fused.view_stamps_ns
    assert fused.view_sources == expected_fused.view_sources


def test_original_r18_plane_explanation_passes_real_gates_and_complete_history(recorded):
    history = recorded['surface'].points_base.copy()
    raw = recorded['raw_geometry'].object_points_base.copy()
    evidence = recorded['raw_box_model'].to_evidence_dict()
    result, fused, audit = run(recorded)
    assert result.ok and audit['selected_hypothesis'] == 'support_plane_minimal'
    assert audit['selection_reason'] == 'SUPPORT_PLANE_ORIGINAL_GATES_AND_COMPLETE_UNION_CONSISTENT'
    for name in ('original_icp', 'identity'):
        row = audit['hypotheses'][name]
        assert row['registration']['ok'] and not row['consistency']['ok']
        original = recorded['data']['original_choice']['hypotheses'][name]
        assert row['consistency']['maximum_observed_model_excess_m'] == pytest.approx(
            original['consistency']['maximum_observed_model_excess_m'], abs=1e-14)
    row = audit['hypotheses']['support_plane_minimal']
    assert row['registration']['ok'] and row['consistency']['ok']
    assert row['consistency']['before_history_point_count'] == 1563
    assert row['consistency']['raw_point_count'] == 806
    assert row['consistency']['exact_union_point_count'] == 2369
    assert row['consistency']['exact_duplicates_removed'] == 0
    assert row['consistency']['maximum_observed_model_excess_m'] == pytest.approx(
        recorded['data']['expected_support_plane_maximum_family_excess_m'], abs=1e-14)
    repeated = surface.evaluate_registration_transform(recorded['reference'], recorded['moving'],
        result.transform_base, recorded['config'])
    assert repeated.ok and repeated.inlier_count == 659
    np.testing.assert_array_equal(history, recorded['surface'].points_base)
    np.testing.assert_array_equal(raw, recorded['raw_geometry'].object_points_base)
    assert recorded['raw_box_model'].to_evidence_dict() == evidence
    assert fused.view_stamps_ns == recorded['surface'].view_stamps_ns+(recorded['moving'].stamp_ns,)
    assert audit['execution_authorized'] is False and audit['final_fused_model_authorized'] is False
    assert audit['selection_uses_opening_or_path_score'] is False


def test_history_counterexample_cannot_be_removed_to_make_plane_hypothesis_pass(recorded):
    before = recorded['surface']; model = recorded['raw_box_model']
    extra = model.center_base+model.axes_base[:, 0]*.1
    expanded = replace(before, points_base=np.vstack((before.points_base, extra)),
        view_indices=np.r_[before.view_indices, 0], view_sources=before.view_sources+((0,),))
    expected, expected_fused = surface.append_registered_view(expanded,
        recorded['reference'], recorded['moving'], recorded['config'])
    result, fused, audit = run(recorded, surface=expanded)
    assert_original(result, fused, expected, expected_fused)
    assert audit['selection_reason'] == 'NO_BOX_CONSISTENT_HYPOTHESIS_KEEP_MEASURED'
    for row in audit['hypotheses'].values():
        assert not row['consistency']['ok']
        assert row['consistency']['before_history_point_count'] == 1564
    assert any(np.array_equal(p, extra) for p in fused.points_base)


@pytest.mark.parametrize('stage,error_type', [('transform', ValueError),
    ('registration', np.linalg.LinAlgError), ('append', FloatingPointError)])
def test_optional_numerical_failure_preserves_original_measured_fallback(recorded, monkeypatch, stage, error_type):
    expected, expected_fused = baseline(recorded)
    def fail(*args, **kwargs): raise error_type('injected numerical contract failure')
    if stage == 'transform':
        monkeypatch.setattr(choice, '_minimal_support_plane_transform', fail)
    elif stage == 'registration':
        original = choice.evaluate_registration_transform
        def evaluate(reference, moving, transform, config):
            return original(reference, moving, transform, config) if np.array_equal(transform, np.eye(4)) else fail()
        monkeypatch.setattr(choice, 'evaluate_registration_transform', evaluate)
    else:
        original = choice.append_registered_view
        def append(*args, **kwargs):
            return fail() if 'transform_base' in kwargs else original(*args, **kwargs)
        monkeypatch.setattr(choice, 'append_registered_view', append)
    result, fused, audit = run(recorded)
    assert_original(result, fused, expected, expected_fused)
    assert audit['selection_reason'] == 'SUPPORT_PLANE_EVALUATION_FAILED_KEEP_MEASURED'
    assert audit['selected_hypothesis'] == 'original_icp'
    assert audit['selected_box_explanation'] is False


@pytest.mark.parametrize('stage', ['transform', 'append'])
def test_control_cancellation_is_not_swallowed_as_optional_geometry_failure(recorded, monkeypatch, stage):
    class ControlCancelled(RuntimeError): pass
    def fail(*args, **kwargs): raise ControlCancelled('task owner revoked')
    if stage == 'transform':
        monkeypatch.setattr(choice, '_minimal_support_plane_transform', fail)
    else:
        original = choice.append_registered_view
        def append(*args, **kwargs):
            return fail() if 'transform_base' in kwargs else original(*args, **kwargs)
        monkeypatch.setattr(choice, 'append_registered_view', append)
    with pytest.raises(ControlCancelled, match='owner revoked'): run(recorded)


def test_fixed_append_gate_rejection_never_replaces_original_success(recorded, monkeypatch):
    expected, expected_fused = baseline(recorded)
    original = choice.append_registered_view
    def append(*args, **kwargs):
        if 'transform_base' in kwargs:
            return surface._failure('RMSE_EXCEEDED'), args[0]
        return original(*args, **kwargs)
    monkeypatch.setattr(choice, 'append_registered_view', append)
    result, fused, audit = run(recorded)
    assert_original(result, fused, expected, expected_fused)
    assert audit['selection_reason'] == 'SUPPORT_PLANE_APPEND_REJECTED_KEEP_MEASURED'
    assert audit['selected_box_explanation'] is False


def test_plane_transform_uses_only_plane_values_and_normalizes_offsets_together(recorded):
    reference, moving = recorded['reference'], recorded['moving']
    transform = choice._minimal_support_plane_transform(reference, moving)
    # Valid tiny normal rescaling preserves the same physical plane equation.
    scaled_ref = replace(reference, support_normal_base=reference.support_normal_base*(1.+1e-8),
        support_offset_m=reference.support_offset_m*(1.+1e-8), points_base=reference.points_base+1.)
    scaled_moving = replace(moving, support_normal_base=moving.support_normal_base*(1.-1e-8),
        support_offset_m=moving.support_offset_m*(1.-1e-8), points_base=moving.points_base-1.)
    np.testing.assert_allclose(choice._minimal_support_plane_transform(scaled_ref, scaled_moving),
        transform, rtol=0., atol=1e-14)
    np.testing.assert_allclose(transform[:3, :3]@moving.support_normal_base,
        reference.support_normal_base, rtol=0., atol=1e-14)
    assert np.linalg.norm(np.cross(transform[:3, 3], reference.support_normal_base)) < 1e-14


@pytest.mark.parametrize('fault', ['zero', 'nan', 'infinite', 'offset_nan', 'opposite'])
def test_corrupt_plane_cannot_emit_nonfinite_or_arbitrary_transform(recorded, fault):
    # Explicitly corrupt a boundary carrier to test numerical failure handling.
    from types import SimpleNamespace
    moving = recorded['moving']; normal = moving.support_normal_base.copy()
    offset = moving.support_offset_m
    if fault == 'zero': normal[:] = 0.
    elif fault == 'nan': normal[0] = np.nan
    elif fault == 'infinite': normal[0] = np.inf
    elif fault == 'offset_nan': offset = np.nan
    else: normal = -recorded['reference'].support_normal_base
    with pytest.raises(ValueError):
        choice._minimal_support_plane_transform(recorded['reference'],
            SimpleNamespace(support_normal_base=normal, support_offset_m=offset))


def test_original_registration_failure_prevents_new_explanation(recorded, monkeypatch):
    def forbidden(*args, **kwargs): pytest.fail('original rejected registration is final')
    monkeypatch.setattr(choice, '_minimal_support_plane_transform', forbidden)
    result, fused, audit = run(recorded, config=replace(recorded['config'], maximum_translation_m=.001))
    assert not result.ok and result.code == 'TRANSLATION_BOUND_EXCEEDED'
    assert fused is recorded['surface'] and 'support_plane_minimal' not in audit['hypotheses']


def test_missing_raw_model_cannot_enable_plane_choice(recorded, monkeypatch):
    def forbidden(*args, **kwargs): pytest.fail('raw source model is required')
    monkeypatch.setattr(choice, '_minimal_support_plane_transform', forbidden)
    expected, expected_fused = baseline(recorded)
    result, fused, audit = run(recorded, raw_box_model=None)
    assert_original(result, fused, expected, expected_fused)
    assert audit['selection_reason'] == 'RAW_MODEL_BINDING_UNAVAILABLE'
    assert 'support_plane_minimal' not in audit['hypotheses']
