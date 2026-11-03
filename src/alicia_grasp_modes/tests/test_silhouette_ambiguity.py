"""Ownership disagreement must remain uncertainty, including real side faces."""
import json

import cv2
import numpy as np
import pytest

from alicia_grasp_modes.measured_mask_seeds import confidence_labels
from alicia_grasp_modes.silhouette_ambiguity import evaluate_silhouette_ambiguity
from alicia_grasp_modes.tabletop import Config, _project


NAMES = ('rgb', 'depth', 'seed', 'hard_foreground', 'hard_background', 'height')


def true_multicolour_object(red=110, depth_variant='holes'):
    # Independently specified physical silhouette: white top and real red
    # side/platform on a red table. These are test truth, never algorithm inputs.
    color = np.full((240, 320, 3), (20, 20, 180), np.uint8)
    top = np.zeros((240, 320), bool); top[88:125, 120:190] = True
    side = np.zeros_like(top); side[125:143, 120:190] = True
    color[top] = 215; color[side] = (max(5, round(red/9)), max(5, round(red/9)), red)
    y, x = np.indices(top.shape)
    height = .02*(1-np.clip((y-125)/17., 0., 1.))+.006*np.clip((y-125)/17., 0., 1.)
    depth = np.full(top.shape, .3); depth[top] = .28; depth[side] = .3-height[side]
    if depth_variant == 'holes': depth[side & ((x % 4 == 0) | (y % 5 == 0))] = 0.
    elif depth_variant == 'flat': depth[side] = .294
    else: raise AssertionError(depth_variant)
    seed = (top | side) & (depth > 0) & (.3-depth >= .004)
    points = _project(depth, [300., 300., 160., 120.]); heights = .3-depth
    sl = np.s_[80:151, 112:198]
    core, background, evidence = confidence_labels(seed[sl], points[sl], depth[sl],
        heights[sl], np.array([-1., 0., 0.]), Config(), bgr=color[sl])
    background[:3] = background[-3:] = True
    background[:, :3] = background[:, -3:] = True
    inputs = [color[sl].copy(), depth[sl].copy(), seed[sl].copy(), core,
              background, heights[sl].copy()]
    options = dict(source_stamps_ns={key: 123456789 for key in NAMES},
        upper_seed_height_m=evidence['upper_seed_height_m'],
        support_height_separated=evidence['support_background_height_separated'])
    return inputs, options, side[sl], (top | side)[sl]


@pytest.mark.parametrize('red,variant', [(70, 'holes'), (110, 'holes'),
                                        (150, 'holes'), (220, 'holes'), (110, 'flat')])
def test_five_independent_true_side_counterexamples_report_uncertainty_not_a_smaller_object(red, variant):
    inputs, options, side, truth = true_multicolour_object(red, variant)
    before = [array.copy() for array in inputs]
    report = evaluate_silhouette_ambiguity(*inputs, **options)
    assert report['ambiguous']
    assert report['conclusion'] == 'SILHOUETTE_INITIALIZATION_SENSITIVE'
    assert report['original']['pixel_count'] == np.count_nonzero(truth)
    assert report['counterfactual']['pixel_count'] == np.count_nonzero(truth & ~side)
    assert report['disagreement']['pixel_count'] == np.count_nonzero(side) == 1260
    assert report['disagreement']['valid_depth_pixels'] == np.count_nonzero(side & (inputs[1] > 0))
    assert report['disagreement']['hard_foreground_pixels'] == 0
    assert not report['disagreement']['hard_labels_touched']
    assert report['preferred_hypothesis'] is None
    assert report['diagnostic_only'] and not report['executable_geometry']
    assert not report['execution_authority'] and not report['observation_authority']
    assert not report['physical_contour_certified']
    assert json.loads(json.dumps(report, allow_nan=False)) == report
    for actual, expected in zip(inputs, before):
        np.testing.assert_array_equal(actual, expected)


@pytest.mark.parametrize('case', ['low_profile', 'no_measured_support', 'same_colour', 'dim_side', 'weak_chroma'])
def test_no_disagreement_is_never_a_physical_or_execution_certificate(case):
    inputs, options, side, _ = true_multicolour_object()
    if case == 'low_profile': options['support_height_separated'] = False
    if case == 'no_measured_support': inputs[5][inputs[4]] = .01
    if case == 'same_colour': inputs[0][inputs[3]] = (12, 12, 110)
    if case == 'dim_side': inputs[0][side] = (5, 5, 5)
    if case == 'weak_chroma': inputs[0][side] = (100, 100, 100)
    report = evaluate_silhouette_ambiguity(*inputs, **options)
    assert report['counterfactual_initialization']['probable_background_pixels'] == 0
    assert not report['ambiguous']
    assert report['hypotheses_evaluated'] == 2
    assert report['preferred_hypothesis'] is None
    assert not report['physical_contour_certified']
    assert not report['execution_authority'] and not report['observation_authority']


def test_two_hypotheses_have_fresh_models_fixed_budget_and_unchanged_hard_labels(monkeypatch):
    inputs, options, _, _ = true_multicolour_object()
    calls = []; original = cv2.grabCut
    def observed(image, labels, rect, bg_model, fg_model, iterations, mode):
        calls.append((labels.copy(), bg_model, fg_model, iterations, mode))
        return original(image, labels, rect, bg_model, fg_model, iterations, mode)
    monkeypatch.setattr(cv2, 'grabCut', observed)
    evaluate_silhouette_ambiguity(*inputs, **options)
    assert len(calls) == 2
    assert calls[0][3:] == calls[1][3:] == (3, cv2.GC_INIT_WITH_MASK)
    assert not np.shares_memory(calls[0][1], calls[1][1])
    assert not np.shares_memory(calls[0][2], calls[1][2])
    for labels, *_ in calls:
        assert np.all(labels[inputs[3]] == cv2.GC_FGD)
        assert np.all(labels[inputs[4]] == cv2.GC_BGD)


def test_separate_frame_call_does_not_change_first_frame_result():
    inputs, options, _, _ = true_multicolour_object()
    expected = evaluate_silhouette_ambiguity(*inputs, **options)
    other = [a.copy() for a in inputs]; other[0][other[3]] = (12, 12, 110)
    other_options = dict(options, source_stamps_ns={key: 987654321 for key in NAMES})
    assert not evaluate_silhouette_ambiguity(*other, **other_options)['ambiguous']
    assert evaluate_silhouette_ambiguity(*inputs, **options) == expected


@pytest.mark.parametrize('change', ['mismatch', 'missing', 'boolean', 'zero', 'extra'])
def test_mixed_or_unbound_sources_fail_before_any_image_computation(monkeypatch, change):
    inputs, options, _, _ = true_multicolour_object()
    stamps = options['source_stamps_ns']
    if change == 'mismatch': stamps['depth'] += 1
    if change == 'missing': del stamps['seed']
    if change == 'boolean': stamps['rgb'] = True
    if change == 'zero': stamps['height'] = 0
    if change == 'extra': stamps['old_frame'] = 123456789
    def forbidden(*args): raise AssertionError('unbound inputs reached GrabCut')
    monkeypatch.setattr(cv2, 'grabCut', forbidden)
    with pytest.raises(ValueError, match='source_stamp'):
        evaluate_silhouette_ambiguity(*inputs, **options)


@pytest.mark.parametrize('change', ['shape', 'overlap', 'unmeasured_core', 'unseeded_core', 'height_nan'])
def test_invalid_measurement_or_hard_labels_cannot_produce_diagnostic_evidence(change):
    inputs, options, _, _ = true_multicolour_object()
    y, x = np.argwhere(inputs[3])[0]
    if change == 'shape': inputs[1] = inputs[1][:-1]
    if change == 'overlap': inputs[4][y, x] = True
    if change == 'unmeasured_core': inputs[1][y, x] = 0
    if change == 'unseeded_core': inputs[2][y, x] = False
    if change == 'height_nan': inputs[5][y, x] = np.nan
    with pytest.raises(ValueError):
        evaluate_silhouette_ambiguity(*inputs, **options)
