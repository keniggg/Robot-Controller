"""Independent proposal geometry checks; original gates and no ROS I/O."""
from copy import deepcopy
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_remote_modes import configured, base

from alicia_grasp_modes.observe_return_visual import (
    directed_missing_side_rotations, distance_bound_missing_side_rotation,
    missing_side_view_gain,
)


def case():
    gap = dict(target_center_base_m=[0., 0., 0.],
        initial_camera_origin_base_m=[0., 0., .3], jaw_axis_base=[1., 0., 0.],
        object_height_m=.02, whole_obb_width_m=.04, required_open_width_m=.05,
        angle_deg=0., missing_side_signs=[1], target_epoch=7,
        target_track_id='independent-target', source_view_stamps_ns=[100])
    reference = np.asarray([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
    generated = directed_missing_side_rotations((gap,), reference, [0., 0., 1.], .003, .26)
    assert len(generated) == 2
    return gap, generated


@pytest.mark.parametrize('distance', [.018, .06, .18, .26, .8, 3.])
@pytest.mark.parametrize('shift', [-.03, -.004, 0., .003, .05])
def test_actual_center_and_distance_never_invent_gate_pass_or_discard_old_pass(distance, shift):
    gap, generated = case()
    rotation, metadata = generated[0]
    center = np.asarray([shift, .001, .002])
    old = missing_side_view_gain((gap,), center-distance*rotation[:, 0], .003)
    old_rotation, old_metadata, old_gap = rotation.copy(), deepcopy(metadata), deepcopy(gap)
    fixed, evidence = distance_bound_missing_side_rotation(
        (gap,), rotation, metadata, center, distance, .003, [0., 0., 1.])
    assert np.array_equal(rotation, old_rotation) and metadata == old_metadata and gap == old_gap
    if old['ok']:
        assert np.array_equal(fixed, rotation) and evidence is metadata
    elif evidence is not metadata:
        assert missing_side_view_gain((gap,), center-distance*fixed[:, 0], .003)['ok']
        assert fixed[2, 0] < -1e-9
        assert metadata['camera_rotation_offset_deg'] <= evidence['camera_rotation_offset_deg'] < 45.
        assert evidence['distance_bound_construction']['execution_authority'] is False
    else:
        assert np.array_equal(fixed, rotation)


def test_another_valid_gap_keeps_original_pose_exactly():
    gap, generated = case()
    rotation, metadata = generated[0]
    second = dict(gap, angle_deg=90., jaw_axis_base=[0., 1., 0.])
    center = np.asarray([-.004, .08, 0.])
    origin = center-.26*rotation[:, 0]
    assert not missing_side_view_gain((gap,), origin, .003)['ok']
    assert missing_side_view_gain((second,), origin, .003)['ok']
    fixed, evidence = distance_bound_missing_side_rotation(
        (gap, second), rotation, metadata, center, .26, .003, [0., 0., 1.])
    assert np.array_equal(fixed, rotation) and evidence is metadata


@pytest.mark.parametrize('upper_slot,duplicate,missing_binding', [
    (True, False, False), (False, True, False), (False, False, True)])
def test_upper_or_ambiguous_or_missing_evidence_keeps_original(upper_slot, duplicate, missing_binding):
    gap, generated = case()
    rotation, metadata = generated[int(upper_slot)]
    gaps = (gap, gap) if duplicate else (() if missing_binding else (gap,))
    fixed, evidence = distance_bound_missing_side_rotation(
        gaps, rotation, metadata, [-1., 0., 0.], .26, .003, [0., 0., 1.])
    assert np.array_equal(fixed, rotation) and evidence is metadata


@pytest.mark.parametrize('distance,center,normal', [
    (0., [0., 0., 0.], [0., 0., 1.]),
    (-1., [0., 0., 0.], [0., 0., 1.]),
    (float('inf'), [0., 0., 0.], [0., 0., 1.]),
    (.26, [float('nan'), 0., 0.], [0., 0., 1.]),
    (.26, [0., 0., 0.], [0., 0., 0.]),
])
def test_invalid_geometry_is_explicitly_rejected(distance, center, normal):
    gap, generated = case()
    with pytest.raises(ValueError):
        distance_bound_missing_side_rotation((gap,), *generated[0], center, distance, .003, normal)


def test_unreachable_bracket_never_claims_no_ik_solution():
    gap, generated = case()
    rotation, metadata = generated[0]
    fixed, evidence = distance_bound_missing_side_rotation(
        (gap,), rotation, metadata, [-1., 0., 0.], .005, .003, [0., 0., 1.])
    assert np.array_equal(fixed, rotation) and evidence is metadata


def test_zero_turn_is_not_upgraded_to_a_new_direction():
    gap, generated = case()
    rotation, original = generated[0]
    metadata = dict(original, camera_rotation_offset_deg=0., minimum_required_rotation_deg=0.)
    fixed, evidence = distance_bound_missing_side_rotation(
        (gap,), rotation, metadata, [-1., 0., 0.], .26, .003, [0., 0., 1.])
    assert np.array_equal(fixed, rotation) and evidence is metadata


def test_actual_generated_upper_bound_cannot_be_replaced_by_nominal_45_degree_cap():
    gap, generated = case()
    rotation, metadata = generated[0]
    bounded_upper = metadata['camera_rotation_offset_deg'] + .01
    fixed, evidence = distance_bound_missing_side_rotation(
        (gap,), rotation, metadata, [-.004, 0., 0.], .26, .003, [0., 0., 1.],
        existing_upper_angle_deg=bounded_upper)
    assert np.array_equal(fixed, rotation) and evidence is metadata


def test_mode_hook_filters_unrelated_epoch_before_old_pass_preservation(configured, monkeypatch):
    node, _ = configured
    gap, generated = case()
    rotation, metadata = generated[0]
    foreign = dict(gap, angle_deg=90., jaw_axis_base=[0., 1., 0.], target_epoch=999)
    center = np.asarray([-.004, .08, 0.])
    assert not missing_side_view_gain((gap,), center-.26*rotation[:, 0], .003)['ok']
    assert missing_side_view_gain((foreign,), center-.26*rotation[:, 0], .003)['ok']
    monkeypatch.setattr(node, '_observe_return_selected', lambda: True)
    node._observe_return_phase = 'observation_prefix'
    node._observe_return_diagnostic_observation_only = False
    node._observe_return_view_gaps = (gap, foreign)
    node._observe_return_directed_views = generated
    node._observe_return_view_uncertainty_m = .003
    monkeypatch.setattr(node, '_current_stream_target_identity',
        lambda: NS(epoch=7, track_id='independent-target'))
    monkeypatch.setattr(node, '_tool_from_camera_matrix', lambda: np.eye(4))
    transform = np.eye(4)
    transform[:3, :3] = rotation
    variant = dict(reference_pose=base.make_pose_stamped('base_link', [0., 0., 0.],
        base.quaternion_from_matrix(transform), stamp=base.rospy.Time(10)),
        camera_tilt_offset_deg=metadata['camera_rotation_offset_deg'],
        camera_roll_offset_deg=0., preference_index=1)
    changed = node._observation_distance_reference_variant(variant,
        NS(center_base=center, support_normal_base=np.asarray([0., 0., 1.])), .26)
    assert changed is not variant
    fixed = base.pose_matrix(changed['reference_pose'])[:3, :3]
    assert missing_side_view_gain((gap,), center-.26*fixed[:, 0], .003)['ok']

