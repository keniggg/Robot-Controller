"""Isolated wrapper handoff regression using failed qualifier responses.

These tests simulate the three producer returns to exercise error propagation.
They do not validate the interval evidence, fabricate a qualified model, or
establish an end-to-end source-to-motion result.
"""
import copy
from types import MappingProxyType

import pytest

from alicia_flexible_grasp.vision import regular_box_inference as box


def _failed(code, reason, **evidence):
    return box.BoxInferenceResult(False, code, None,
        MappingProxyType(dict(reason=reason, **evidence)))


def _run_wrapper(monkeypatch, returns, *, color=True):
    args = tuple(object() for _ in range(9))
    image = object() if color else None
    calls = []

    def qualifier(*actual_args, **kwargs):
        assert len(actual_args) == len(args)
        assert all(actual is expected for actual, expected in zip(actual_args, args))
        assert kwargs['color_bgr'] is image
        calls.append(dict(kwargs))
        assert len(calls) <= len(returns), 'unexpected repeated qualification'
        return returns[len(calls)-1]

    monkeypatch.setattr(box, '_qualify_regular_box_top', qualifier)
    result = box.qualify_regular_box_top(*args, color_bgr=image)
    return result, calls


@pytest.mark.parametrize('cuboid_code,cuboid_reason', [
    ('BOX_MEASURED_GEOMETRY_CONFLICT', 'measured target extends outside inferred regular prism'),
    ('BOX_TOP_CONTOUR_INCOMPLETE', 'missing or holed top occupancy'),
    ('BOX_INFERENCE_PRECISION_VIEW_REQUIRED', 'height uncertainty leaves no robust contact interval'),
])
def test_complete_interval_failure_remains_primary_with_all_later_gate_evidence(
        monkeypatch, cuboid_code, cuboid_reason):
    first = _failed('BOX_TOP_NOT_RECTANGULAR', 'original measured top corners incomplete',
        original_marker={'retained': True})
    second = _failed('BOX_TOP_NOT_RECTANGULAR', 'top-only mask projection is incomplete',
        top_projection_marker={'retained': True})
    interval = dict(complete_raster_responses_used=True,
        source_mask_sha256='test-source-marker', unknown_mask_pixel_count=17)
    witness = dict(point_index=966, axis_index=1, excess_m=.001825601,
        permitted_observed_excess_m=.001, execution_authorized=False)
    third = _failed(cuboid_code, cuboid_reason, cuboid_rgb_interval=interval,
        top_occupancy_fraction=.71, maximum_internal_hole_fraction=.014,
        maximum_observed_model_excess_m=.001825601,
        measured_geometry_conflict=witness)
    originals = [copy.deepcopy(dict(item.audit)) for item in (first, second, third)]

    result, calls = _run_wrapper(monkeypatch, [first, second, third])

    assert not result.ok and result.model is None
    assert result.code == cuboid_code
    assert result.audit['reason'] == cuboid_reason
    assert result.audit['cuboid_rgb_interval'] is interval
    assert result.audit['measured_geometry_conflict'] is witness
    assert result.audit['top_occupancy_fraction'] == .71
    assert result.audit['maximum_internal_hole_fraction'] == .014
    assert result.audit['maximum_observed_model_excess_m'] == .001825601
    assert result.audit['original_qualification_code'] == first.code
    assert result.audit['original_qualification_reason'] == first.audit['reason']
    assert result.audit['top_projection_qualification_code'] == second.code
    assert result.audit['top_projection_qualification_reason'] == second.audit['reason']
    assert [set(call)-{'color_bgr'} for call in calls] == [
        set(), {'outer_fallback'}, {'cuboid_fallback'}]
    assert calls[1]['outer_fallback'] is True
    assert calls[2]['cuboid_fallback'] is True
    assert [dict(item.audit) for item in (first, second, third)] == originals


@pytest.mark.parametrize('interval', [
    None, {}, {'complete_raster_responses_used': False},
    {'complete_raster_responses_used': 1},
    {'complete_raster_responses_used': 'true'},
])
def test_no_completed_interval_keeps_original_top_projection_fallback(monkeypatch, interval):
    first = _failed('BOX_TOP_CONTOUR_INCOMPLETE', 'initial top incomplete')
    second = _failed('BOX_INFERENCE_PRECISION_VIEW_REQUIRED', 'top projection needs precision',
        maximum_observed_model_excess_m=.0004, top_occupancy_fraction=.73)
    third = _failed('BOX_MEASURED_GEOMETRY_CONFLICT', 'incomplete diagnostic only',
        cuboid_rgb_interval=interval, maximum_observed_model_excess_m=.003)
    originals = [copy.deepcopy(dict(item.audit)) for item in (first, second, third)]

    result, calls = _run_wrapper(monkeypatch, [first, second, third])

    assert len(calls) == 3
    assert not result.ok and result.model is None
    assert result.code == second.code and result.audit['reason'] == second.audit['reason']
    assert result.audit['maximum_observed_model_excess_m'] == .0004
    assert result.audit['top_occupancy_fraction'] == .73
    assert result.audit['original_qualification_code'] == first.code
    assert result.audit['original_qualification_reason'] == first.audit['reason']
    assert result.audit['cuboid_qualification_code'] == third.code
    assert result.audit['cuboid_qualification_reason'] == third.audit['reason']
    assert result.audit['cuboid_rgb_interval'] is interval
    assert 'top_projection_qualification_code' not in result.audit
    assert [dict(item.audit) for item in (first, second, third)] == originals


@pytest.mark.parametrize('color,first_code', [
    (False, 'BOX_TOP_NOT_RECTANGULAR'),
    (True, 'BOX_SUPPORT_INCONSISTENT'),
])
def test_legacy_first_failure_route_does_not_call_another_qualifier(monkeypatch, color, first_code):
    first = _failed(first_code, 'original source or support failure')
    result, calls = _run_wrapper(monkeypatch, [first], color=color)
    assert result is first and len(calls) == 1
    assert not result.ok and result.model is None


def test_legacy_terminal_top_projection_failure_remains_primary(monkeypatch):
    first = _failed('BOX_TOP_NOT_RECTANGULAR', 'original rectangle failure')
    second = _failed('BOX_SUPPORT_INCONSISTENT', 'support changed during retry',
        support_inlier_ratio=.1)
    result, calls = _run_wrapper(monkeypatch, [first, second])
    assert len(calls) == 2
    assert not result.ok and result.model is None
    assert result.code == second.code and result.audit['reason'] == second.audit['reason']
    assert result.audit['support_inlier_ratio'] == .1
    assert result.audit['original_qualification_code'] == first.code
    assert 'cuboid_qualification_code' not in result.audit
