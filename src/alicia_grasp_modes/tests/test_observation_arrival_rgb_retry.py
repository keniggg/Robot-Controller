"""Bounded same-frame arrival retry must retain ordinary segmentation gates."""
import cv2
import numpy as np
import pytest

from alicia_grasp_modes.rgb_instance import instance_mask_from_seed


def scene():
    color = np.full((480, 640, 3), 100, np.uint8)
    seed = np.zeros((480, 640), np.uint8)
    seed[230:250, 310:330] = 255
    return color, seed


def grabcut_stub(monkeypatch, outcomes):
    calls = []

    def segment(patch, labels, *args):
        core_y, core_x = np.nonzero(labels == cv2.GC_FGD)
        y0, x0 = 231-int(core_y.min()), 311-int(core_x.min())
        sy, sx = 230-y0, 310-x0
        core = np.zeros((480, 640), bool)
        core[y0:y0+labels.shape[0], x0:x0+labels.shape[1]] = labels == cv2.GC_FGD
        calls.append((labels.shape, core))
        outcome = outcomes[len(calls)-1]
        labels[:] = cv2.GC_BGD
        if outcome == 'boundary':
            labels[3:-3, 3:-3] = cv2.GC_PR_FGD
        elif isinstance(outcome, tuple):
            labels[sy:sy+20, sx:sx+20] = cv2.GC_PR_FGD
            if 'left' in outcome:
                labels[sy:sy+20, 3:sx+20] = cv2.GC_PR_FGD
            if 'top' in outcome:
                labels[3:sy+20, sx:sx+20] = cv2.GC_PR_FGD
            if 'right' in outcome:
                labels[sy:sy+20, sx:-3] = cv2.GC_PR_FGD
            if 'bottom' in outcome:
                labels[sy:-3, sx:sx+20] = cv2.GC_PR_FGD
        elif outcome == 'valid':
            labels[sy:sy+20, sx:sx+20] = cv2.GC_PR_FGD
        elif outcome == 'large':
            labels[sy-15:sy+35, sx-15:sx+35] = cv2.GC_PR_FGD
        elif outcome == 'fragmented':
            labels[sy:sy+9, sx:sx+20] = cv2.GC_PR_FGD
        else:
            raise AssertionError(outcome)

    monkeypatch.setattr(cv2, 'grabCut', segment)
    return calls


@pytest.mark.parametrize('enabled,attempts', [(False, 2), (True, 3)])
def test_boundary_retry_is_bounded(monkeypatch, enabled, attempts):
    calls = grabcut_stub(monkeypatch, ['boundary'] * attempts)
    with pytest.raises(ValueError, match='^rgb_instance_reaches_search_boundary$'):
        instance_mask_from_seed(*scene(), observation_arrival_retry=enabled)
    assert [shape for shape, _ in calls] == [(52, 52), (84, 84), (148, 148)][:attempts]


def test_default_still_stops_at_second_boundary(monkeypatch):
    calls = grabcut_stub(monkeypatch, ['boundary', 'boundary'])
    with pytest.raises(ValueError, match='^rgb_instance_reaches_search_boundary$'):
        instance_mask_from_seed(*scene())
    assert len(calls) == 2


@pytest.mark.parametrize('failures', [0, 1, 2])
def test_success_needs_only_boundary_retries_and_same_frozen_core(monkeypatch, failures):
    calls = grabcut_stub(monkeypatch, ['boundary'] * failures + ['valid'])
    color, seed = scene()
    mask, evidence = instance_mask_from_seed(color, seed, observation_arrival_retry=True)
    np.testing.assert_array_equal(mask, seed)
    assert evidence['rgb_search_attempts'] == failures + 1
    assert evidence['rgb_search_margin_px'] == (16, 32, 64)[failures]
    expected_core = np.zeros(seed.shape, bool)
    expected_core[231:249, 311:329] = True
    for _, core in calls:
        np.testing.assert_array_equal(core, expected_core)


@pytest.mark.parametrize('boundaries', [0, 1, 2])
def test_fragmented_core_never_gets_another_retry(monkeypatch, boundaries):
    calls = grabcut_stub(monkeypatch, ['boundary'] * boundaries + ['fragmented'])
    with pytest.raises(ValueError, match='^rgb_seed_component_ambiguous_or_fragmented$'):
        instance_mask_from_seed(*scene(), observation_arrival_retry=True)
    assert len(calls) == boundaries + 1


def test_third_crop_cannot_train_on_unobserved_rgb(monkeypatch):
    calls = grabcut_stub(monkeypatch, ['boundary', 'boundary'])
    color, seed = scene()
    valid = np.ones(seed.shape, bool)
    valid[190, 300] = False
    with pytest.raises(ValueError, match='^rgb_search_contains_unobserved_rgb$'):
        instance_mask_from_seed(color, seed, rgb_valid=valid, observation_arrival_retry=True)
    assert len(calls) == 2


def test_third_crop_keeps_original_area_limit(monkeypatch):
    calls = grabcut_stub(monkeypatch, ['boundary', 'boundary', 'large'])
    with pytest.raises(ValueError, match='^rgb_instance_not_supported_by_depth_seed$'):
        instance_mask_from_seed(*scene(), observation_arrival_retry=True)
    assert len(calls) == 3


@pytest.mark.parametrize('sides', [
    ('left',), ('top',), ('right',), ('bottom',), ('left', 'bottom'),
    ('left', 'top', 'right', 'bottom')])
def test_third_crop_expands_only_second_boundary_sides_with_same_core(monkeypatch, sides):
    calls = grabcut_stub(monkeypatch, ['boundary', sides, 'valid'])
    color, seed = scene()
    mask, evidence = instance_mask_from_seed(color, seed, observation_arrival_retry=True)
    expected = [64 if side in sides else 48 for side in ('left', 'top', 'right', 'bottom')]
    assert evidence['rgb_search_margins_ltrb_px'] == expected
    assert evidence['rgb_search_expanded_boundary_sides'] == list(sides)
    left, top, right, bottom = expected
    assert evidence['rgb_search_box_xyxy'] == [310-left, 230-top, 330+right, 250+bottom]
    assert calls[-1][0] == (20+top+bottom, 20+left+right)
    np.testing.assert_array_equal(mask, seed)
    for _, core in calls[1:]:
        np.testing.assert_array_equal(core, calls[0][1])
    assert len(calls) == 3


def test_unneeded_side_keeps_48_and_does_not_import_unobserved_rgb(monkeypatch):
    calls = grabcut_stub(monkeypatch, ['boundary', ('left',), 'valid'])
    color, seed = scene()
    valid = np.ones(seed.shape, bool)
    valid[305, 320] = False  # In symmetric 64 bottom expansion, outside bottom 48.
    mask, evidence = instance_mask_from_seed(
        color, seed, rgb_valid=valid, observation_arrival_retry=True)
    assert evidence['rgb_search_margins_ltrb_px'] == [64, 48, 48, 48]
    assert len(calls) == 3
    np.testing.assert_array_equal(mask, seed)


def test_needed_expansion_rejects_unobserved_rgb_before_third_grabcut(monkeypatch):
    calls = grabcut_stub(monkeypatch, ['boundary', ('left',)])
    color, seed = scene()
    valid = np.ones(seed.shape, bool)
    valid[240, 250] = False  # In newly required left 64, outside old left 48.
    with pytest.raises(ValueError, match='^rgb_search_contains_unobserved_rgb$'):
        instance_mask_from_seed(color, seed, rgb_valid=valid, observation_arrival_retry=True)
    assert len(calls) == 2


def test_new_boundary_on_third_crop_is_not_a_fourth_retry(monkeypatch):
    calls = grabcut_stub(monkeypatch, [('left',), ('left',), ('right',)])
    with pytest.raises(ValueError, match='^rgb_instance_reaches_search_boundary$'):
        instance_mask_from_seed(*scene(), observation_arrival_retry=True)
    assert len(calls) == 3


@pytest.mark.parametrize('invalid', [1, 0, 'yes', None, np.bool_(True)])
def test_retry_flag_requires_boolean(invalid):
    with pytest.raises(ValueError, match='^invalid_observation_arrival_retry$'):
        instance_mask_from_seed(*scene(), observation_arrival_retry=invalid)
