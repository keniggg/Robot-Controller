"""Typed seed insufficiency keeps the original measurement/ownership gates.

The recorded crop is a rejected raw-only region, never a qualified frame.
Synthetic ray casts below test where the typed exception may be translated;
the original geometry, 25-pixel threshold and final ownership checks run.
"""
from copy import deepcopy
from dataclasses import replace
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from alicia_grasp_modes import measured_mask_seeds as seeds
from alicia_grasp_modes import source_connected_owner as owner
from alicia_grasp_modes.tabletop import Config, _project
from test_measured_mask_seeds import surface
from test_gen57_source_component_measurement import measured_scene, recover, _foreign


FIXTURES = Path(__file__).parent / 'fixtures'
NPZ_SHA = '0f359e36a2f67d105702ea0c30f217eca6ffddf4d86d9df4d2f93dc5c53711ad'
JSON_SHA = '71dda60a466baee811ccace2419a452053d5a6393f3a11b233343f9af97fe814'


def recorded_seed_inputs():
    npz = FIXTURES / 'gen60_raw_only_seed_insufficient.npz'
    meta = FIXTURES / 'gen60_raw_only_seed_insufficient.json'
    assert hashlib.sha256(npz.read_bytes()).hexdigest() == NPZ_SHA
    assert hashlib.sha256(meta.read_bytes()).hexdigest() == JSON_SHA
    metadata = json.loads(meta.read_text())
    with np.load(str(npz), allow_pickle=False) as archive:
        arrays = {key: archive[key].copy() for key in archive.files}
    assert set(arrays) == {'seed', 'points', 'depth', 'heights', 'plane_normal', 'bgr'}
    arguments = [arrays[key] for key in ('seed', 'points', 'depth', 'heights', 'plane_normal')]
    arguments.append(Config(**metadata['config']))
    return arguments, arrays['bgr'], metadata


def recorded_seed_failure():
    args, bgr, metadata = recorded_seed_inputs()
    with pytest.raises(seeds.MeasuredSeedCoreInsufficient) as caught:
        seeds.confidence_labels(*args, bgr=bgr)
    return caught.value, metadata


def test_recorded_raw_only_crop_still_rejects_with_its_actual_96_and_18_counts():
    args, bgr, metadata = recorded_seed_inputs()
    before = [value.copy() for value in args[:-1]] + [bgr.copy()]
    with pytest.raises(seeds.MeasuredSeedCoreInsufficient) as caught:
        seeds.confidence_labels(*args, bgr=bgr)
    error = caught.value
    assert isinstance(error, ValueError)
    assert str(error) == metadata['expected_error'] == 'rgb_seed_core_too_small'
    evidence = error.evidence
    assert evidence['measured_pixels'] == metadata['measured_points'] == 96
    assert evidence['eroded_pixels'] == metadata['eroded_points'] == 38
    assert evidence['original_top_pixels'] == metadata['original_top_points'] == 0
    assert evidence['trusted_side_pixels'] == metadata['side_points'] == 0
    assert evidence['trusted_core_pixels'] == metadata['final_foreground_points'] == 18
    assert evidence['minimum_core_pixels'] == 25
    assert evidence['minimum_measured_points'] == 60
    assert evidence['upper_surface_band_used'] is True
    for old, current in zip(before, args[:-1] + [bgr]):
        np.testing.assert_array_equal(current, old)
    json.dumps(evidence, allow_nan=False)


@pytest.mark.parametrize('width,height,core_pixels', [(20, 3, 18), (3, 20, 18),
                                                      (26, 3, 24), (3, 26, 24)])
def test_real_measured_thin_interiors_below_25_remain_rejected(width, height, core_pixels):
    data = surface(width=width, height=height, elevation=.015, noise=0.)
    with pytest.raises(seeds.MeasuredSeedCoreInsufficient) as caught:
        seeds.confidence_labels(*data)
    expected = cv2.erode(data[0].astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
    assert int(expected.sum()) == core_pixels
    assert caught.value.evidence['measured_pixels'] == width * height
    assert caught.value.evidence['eroded_pixels'] == core_pixels
    # The original seed rule does not promote a top/side component below 25.
    # Erosion size alone is not a count of trusted foreground.
    assert caught.value.evidence['trusted_core_pixels'] == 0
    assert caught.value.evidence['minimum_core_pixels'] == 25


@pytest.mark.parametrize('width,height', [(27, 3), (3, 27)])
def test_original_exactly_25_pixel_core_still_passes_without_a_new_threshold(width, height):
    data = surface(width=width, height=height, elevation=.015, noise=0.)
    foreground, background, evidence = seeds.confidence_labels(*data)
    expected = cv2.erode(data[0].astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
    np.testing.assert_array_equal(foreground, expected)
    assert foreground.sum() == 25 and not np.any(foreground & background)
    assert evidence['trusted_core_pixels'] == 25


def raw_thin_region_scene():
    scene = measured_scene()
    a = scene['arguments']
    y = int(np.median(np.nonzero(scene['top'])[0]))
    edge = int(np.flatnonzero(scene['top'][y])[0])
    extra = np.zeros(a['depth'].shape, bool)
    extra[y-1:y+2, edge-20:edge] = True
    assert extra.sum() == 60 and not np.any(extra & scene['truth'])
    rays = _project(np.ones(extra.shape, np.float32), a['intrinsics'])
    plane_camera = a['transform'].T @ scene['plane']
    depth = ((.024-plane_camera[3]) / np.einsum('ijk,k->ij', rays, plane_camera[:3])).astype(np.float32)
    a['depth'][extra] = depth[extra]
    a['rgb'][extra] = (215, 225, 205)
    return scene, extra


def test_only_real_raw_only_extra_failure_becomes_source_bound_insufficiency(monkeypatch):
    scene, extra = raw_thin_region_scene()
    a = scene['arguments']
    frozen_depth = a['depth'].copy()
    frozen_masks = [candidate.mask.copy() for candidate in a['result'].candidates]
    monkeypatch.setattr(owner._FrameRefinement, 'refine',
        lambda *args, **kwargs: pytest.fail('insufficient raw-only input reached complete RGB growth'))
    with pytest.raises(owner.RawOnlySeedInsufficient) as caught:
        recover(scene)
    evidence = caught.value.evidence
    assert evidence['stage'] == 'raw_only_extra_core'
    assert evidence['source_ns'] == a['source_stamp_ns']
    assert evidence['region_index'] == 0
    assert evidence['region_shape'] == list(extra.shape)
    assert evidence['region_mask_sha256'] == hashlib.sha256(
        np.ascontiguousarray(extra, dtype=np.uint8).tobytes()).hexdigest()
    assert evidence['measured_pixels'] == 60
    assert 0 <= evidence['trusted_core_pixels'] < evidence['minimum_core_pixels'] == 25
    np.testing.assert_array_equal(a['depth'], frozen_depth)
    for candidate, frozen in zip(a['result'].candidates, frozen_masks):
        np.testing.assert_array_equal(candidate.mask, frozen)


@pytest.mark.parametrize('reason', ['rgb_seed_core_too_small', 'rgb_depth_foreground_support_conflict'])
def test_plain_value_error_at_raw_only_stage_is_not_a_typed_wait(monkeypatch, reason):
    scene, _ = raw_thin_region_scene()
    stage = [None]
    scene['arguments']['record_stage'] = lambda name: stage.__setitem__(0, name)
    original = owner._FrameRefinement.confidence_labels
    def fail_only_extra(self, *args, **kwargs):
        if stage[0] == 'raw_source_component_evidence':
            raise ValueError(reason)
        return original(self, *args, **kwargs)
    monkeypatch.setattr(owner._FrameRefinement, 'confidence_labels', fail_only_extra)
    with pytest.raises(ValueError, match=reason) as caught:
        recover(scene)
    assert type(caught.value) is ValueError


@pytest.mark.parametrize('stage', ['native', 'complete_refinement'])
def test_native_or_complete_proposal_seed_failure_cannot_be_relabelled_raw_only(monkeypatch, stage):
    scene = measured_scene()
    original_error, _ = recorded_seed_failure()
    def fail(*args, **kwargs):
        raise original_error
    monkeypatch.setattr(owner._FrameRefinement,
        'confidence_labels' if stage == 'native' else 'refine', fail)
    with pytest.raises(seeds.MeasuredSeedCoreInsufficient) as caught:
        recover(scene)
    assert caught.value is original_error
    assert not isinstance(caught.value, owner.RawOnlySeedInsufficient)


@pytest.mark.parametrize('overlap', [1, 144])
def test_actual_foreign_component_overlap_remains_a_hard_original_rejection(monkeypatch, overlap):
    scene = measured_scene()
    valid = recover(scene)
    foreign, _ = _foreign(scene, 20, 20)
    mask = valid.refined.mask.copy()
    mask.ravel()[np.flatnonzero(foreign)[:overlap]] = 255
    monkeypatch.setattr(owner._FrameRefinement, 'refine',
        lambda *args, **kwargs: replace(valid.refined, mask=mask))
    with pytest.raises(ValueError, match='foreign_raw_source_component_overlap') as caught:
        recover(scene)
    assert type(caught.value) is owner.CurrentSourceOwnershipConflict


def test_actual_one_pixel_raw_bridge_remains_hard_rejection(monkeypatch):
    from test_gen57_source_component_measurement import test_raw_only_single_pixel_bridge_is_rejected_before_refinement
    # Runs the original real-geometry bridge veto under the candidate. The
    # unchanged old test also forbids reaching complete RGB refinement.
    test_raw_only_single_pixel_bridge_is_rejected_before_refinement(monkeypatch)


@pytest.mark.parametrize('method', ['confidence_labels', 'refine'])
def test_optional_matching_fragment_keeps_original_small_core_rejection_envelope(monkeypatch, method):
    from alicia_grasp_modes import arrival_fragment_group as group
    from test_arrival_matching_fragments import scene
    inputs = scene()
    typed, _ = recorded_seed_failure()
    original_candidates = tuple(inputs['result'].candidates)
    def insufficient(*args, **kwargs):
        raise typed
    monkeypatch.setattr(group._FrameRefinement, method, insufficient)
    with pytest.raises(ValueError, match='^matching_fragment_original_gate_rejected: rgb_seed_core_too_small$') as caught:
        group.recover_arrival_matching_fragments(**inputs)
    assert type(caught.value) is ValueError
    assert tuple(inputs['result'].candidates) == original_candidates
