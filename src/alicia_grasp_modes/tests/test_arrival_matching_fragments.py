"""Independent ownership must account for every original geometric competitor."""
from copy import deepcopy
from dataclasses import replace

import numpy as np
import pytest

from alicia_grasp_modes import arrival_fragment_group as group
from alicia_grasp_modes.tabletop import _matching_candidates
from test_arrival_fragment_recovery_guards import fragment_scene, measured_candidate


def scene():
    inputs = fragment_scene()
    depth = np.full(inputs['depth'].shape, .3, np.float32)
    depth[210:250, 285:345] = 0.
    members = []
    for left, right in ((285, 313), (317, 345)):
        mask = np.zeros(depth.shape, np.uint8)
        mask[210:250, left:right] = 255
        depth[mask > 0] = .28
        members.append(measured_candidate(mask, depth))
    inputs['depth'] = depth
    inputs['result'].candidates = members
    assert len(_matching_candidates(members, inputs['anchor'], inputs['last'], inputs['config'])) == 2
    return inputs


def test_two_independent_seeds_prove_one_current_instance_without_historical_pixels():
    inputs = scene()
    before_depth = inputs['depth'].copy()
    before_masks = [c.mask.copy() for c in inputs['result'].candidates]
    before_anchor = inputs['anchor'].support_footprint_base.copy()
    result, pair = group.recover_arrival_matching_fragments(**inputs)
    raw, refined = pair
    assert result is not inputs['result'] and result.candidates == [raw]
    audit = raw.metrics['observation_fragment_group']
    assert audit is refined.metrics['observation_fragment_group']
    assert audit['original_metric_matches'] == 2
    assert audit['all_original_matches_accounted_for']
    assert audit['historical_depth_points'] == audit['historical_rgb_pixels'] == 0
    assert len(audit['independent_refinements']) == 2
    assert audit['pairwise_outlines'][0]['mask_iou'] >= .85
    for row in audit['independent_refinements']:
        assert len(row['retained_native_cores']) == 2
        assert row['trusted_core_pixels'] >= 25
    assert audit['intersection_rgb_pixels'] == np.count_nonzero(refined.mask)
    assert audit['final_measured_points'] == np.count_nonzero(raw.mask)
    assert not np.any((raw.mask > 0) & (inputs['depth'] == 0.))
    assert _matching_candidates([raw], inputs['anchor'], inputs['last'], inputs['config'])
    np.testing.assert_array_equal(inputs['depth'], before_depth)
    np.testing.assert_array_equal(inputs['anchor'].support_footprint_base, before_anchor)
    for c, mask in zip(inputs['result'].candidates, before_masks):
        np.testing.assert_array_equal(c.mask, mask)


@pytest.mark.parametrize('kind', ['raw_zero', 'raw_one', 'unbound'])
def test_new_helper_never_changes_existing_scope(monkeypatch, kind):
    inputs = fragment_scene() if kind == 'raw_zero' else scene()
    if kind == 'raw_one':
        inputs['result'].candidates = [inputs['anchor']]
    elif kind == 'unbound':
        inputs['spatial_reference'] = None
    monkeypatch.setattr(group._FrameRefinement, 'refine', lambda *a, **k: pytest.fail('wrong scope'))
    result, pair = group.recover_arrival_matching_fragments(**inputs)
    assert result is inputs['result'] and pair is None


def test_original_raw_zero_helper_still_refuses_multi_matching(monkeypatch):
    inputs = scene()
    monkeypatch.setattr(group._FrameRefinement, 'refine', lambda *a, **k: pytest.fail('old path changed'))
    result, pair = group.recover_arrival_fragment_group(**inputs)
    assert result is inputs['result'] and pair is None


def test_second_independent_failure_cannot_select_first_winner(monkeypatch):
    inputs = scene(); original = group._FrameRefinement.refine; calls = []
    def refine(self, rgb, candidate, **kwargs):
        calls.append(candidate)
        if len(calls) == 2:
            raise ValueError('rgb_instance_reaches_search_boundary')
        return original(self, rgb, candidate, **kwargs)
    monkeypatch.setattr(group._FrameRefinement, 'refine', refine)
    with pytest.raises(ValueError, match='^matching_fragment_original_gate_rejected: rgb_instance_reaches_search_boundary$'):
        group.recover_arrival_matching_fragments(**inputs)
    assert len(calls) == 2 and len(inputs['result'].candidates) == 2


def test_two_distinct_current_instances_cannot_be_merged_by_seed_union(monkeypatch):
    inputs = scene(); seen = []
    def independent(self, rgb, candidate, **kwargs):
        seen.append(candidate)
        return candidate
    monkeypatch.setattr(group._FrameRefinement, 'refine', independent)
    # Both dark half-boxes satisfy the original colour and contour bounds, but
    # each output excludes the other original competitor's current hard core.
    with pytest.raises(ValueError, match='^matching_fragment_competitor_not_owned$'):
        group.recover_arrival_matching_fragments(**inputs)
    assert len(seen) == 2


def test_domain_clipping_cannot_hide_an_original_competitor(monkeypatch):
    inputs = scene()
    source = inputs['result'].candidates[1]
    native = np.roll(source.mask, 18, axis=1)
    inputs['depth'][native > 0] = .28
    outside = measured_candidate(native, inputs['depth'])
    inputs['result'].candidates[1] = outside
    assert len(_matching_candidates(inputs['result'].candidates, inputs['anchor'], inputs['last'], inputs['config'])) == 2
    first = inputs['result'].candidates[0]
    full = np.zeros(inputs['depth'].shape, np.uint8); full[210:250, 285:345] = 255
    chosen = replace(measured_candidate(full * (inputs['depth'] > 0), inputs['depth']), mask=full)
    calls = []
    def independent(self, rgb, candidate, **kwargs):
        calls.append(candidate)
        assert np.array_equal(candidate.mask, first.mask if candidate is first else native)
        return chosen
    monkeypatch.setattr(group._FrameRefinement, 'refine', independent)
    with pytest.raises(ValueError, match='^matching_fragment_competitor_not_owned$'):
        group.recover_arrival_matching_fragments(**inputs)
    assert len(calls) == 2


def test_original_competitor_without_a_measured_core_is_not_discarded(monkeypatch):
    inputs = scene(); original = group._FrameRefinement.confidence_labels
    def no_second_core(self, seed, points, depth, *args, **kwargs):
        core, support, evidence = original(self, seed, points, depth, *args, **kwargs)
        return np.zeros_like(core), support, evidence
    # Refine is controlled so this isolates the mandatory original native
    # per-competitor core check, rather than the refiner's own core gate.
    full = np.zeros(inputs['depth'].shape, np.uint8); full[210:250, 285:345] = 255
    chosen = replace(measured_candidate(full * (inputs['depth'] > 0), inputs['depth']), mask=full)
    monkeypatch.setattr(group._FrameRefinement, 'refine', lambda *a, **k: chosen)
    monkeypatch.setattr(group._FrameRefinement, 'confidence_labels', no_second_core)
    with pytest.raises(ValueError, match='^matching_fragment_native_core_unproven$'):
        group.recover_arrival_matching_fragments(**inputs)


def test_missing_current_depth_cannot_use_reference_to_complete_geometry():
    inputs = scene()
    inputs['depth'][inputs['depth'] < .3] = 0.
    with pytest.raises(ValueError, match='^matching_fragment_original_gate_rejected: rgb_seed_has_insufficient_measured_foreground$'):
        group.recover_arrival_matching_fragments(**inputs)


def test_consensus_area_still_requires_enough_current_target_depth(monkeypatch):
    inputs = scene()
    inputs['rgb'][:] = 20
    measured = np.zeros(inputs['depth'].shape, np.uint8)
    measured[210:250, 285:345] = 255
    rgb_mask = np.zeros_like(measured); rgb_mask[170:290, 255:375] = 255
    chosen = replace(measured_candidate(measured * (inputs['depth'] > 0), inputs['depth']), mask=rgb_mask)
    monkeypatch.setattr(group._FrameRefinement, 'refine', lambda *a, **k: chosen)
    with pytest.raises(ValueError, match='^matching_fragment_consensus_current_support_required$'):
        group.recover_arrival_matching_fragments(**inputs)


def test_consensus_intersection_is_symmetric_without_selecting_a_seed():
    inputs = scene()
    _, first = group.recover_arrival_matching_fragments(**inputs)
    reverse = dict(inputs, result=replace(inputs['result'], candidates=list(reversed(inputs['result'].candidates))))
    _, second = group.recover_arrival_matching_fragments(**reverse)
    for attr in ('mask', 'position_base', 'extent_m', 'support_footprint_base'):
        np.testing.assert_array_equal(getattr(first[1], attr), getattr(second[1], attr))


def test_programming_error_is_not_relabelled_as_ordinary_ambiguity(monkeypatch):
    inputs = scene()
    def broken(*args, **kwargs):
        raise RuntimeError('unexpected backend bug')
    monkeypatch.setattr(group._FrameRefinement, 'refine', broken)
    with pytest.raises(RuntimeError, match='unexpected backend bug'):
        group.recover_arrival_matching_fragments(**inputs)


def test_actual_independent_rgb_objects_remain_separate_without_union_hard_foreground():
    inputs = scene()
    # Both pieces still satisfy every original geometric association gate.
    # Their intervening pixels now show observed table RGB and measured support
    # depth, rather than a missing-depth stripe on one continuous dark object.
    inputs['rgb'][210:250, 313:317] = (60, 90, 190)
    inputs['depth'][210:250, 313:317] = .3
    assert len(_matching_candidates(inputs['result'].candidates,
        inputs['anchor'], inputs['last'], inputs['config'])) == 2
    masks = [candidate.mask.copy() for candidate in inputs['result'].candidates]
    with pytest.raises(ValueError, match='^matching_fragment_competitor_not_owned$'):
        group.recover_arrival_matching_fragments(**inputs)
    for candidate, before in zip(inputs['result'].candidates, masks):
        np.testing.assert_array_equal(candidate.mask, before)
