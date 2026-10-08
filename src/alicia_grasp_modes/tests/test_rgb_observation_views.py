"""Visibility/utility previews carry no physical feasibility or contact rights."""
from dataclasses import asdict
import json
import math

import numpy as np
import pytest

from alicia_grasp_modes.rgb_observation_views import (
    ObservationPoseCandidate, generate_local_rgb_observation_candidates,
    rank_rgb_observation_candidates)
from test_rgb_observation_volume import build, K, T


def pose(name='small', shift=(.015, 0., 0.), **kwargs):
    transform = T.copy(); transform[:3, 3] += shift
    return ObservationPoseCandidate(name, transform, **kwargs)


def rank(candidates, volume=None, **overrides):
    args = dict(current_T_base_camera=T, camera_matrix=K,
                image_shape=(480, 640), working_depth_range_m=(.07, .7))
    args.update(overrides)
    return rank_rgb_observation_candidates(build() if volume is None else volume,
                                          candidates, **args)


def test_small_valid_camera_displacement_is_preview_only_with_no_guessed_geometry():
    result = rank([pose()])
    assert len(result['ranked']) == 1 and not result['rejected']
    row = result['ranked'][0]
    assert row['all_interest_vertices_visible']
    assert row['minimum_view_ray_change_rad'] > 0.
    assert 0. < row['bounded_utility'] < 1.
    assert row['camera_translation_m'] == pytest.approx(.015)
    assert row['joint_l1_delta_rad'] is row['tool_translation_m'] is None
    for name in ('execution_authority', 'motion_authorized', 'collision_checked',
                 'reachability_verified', 'measured_information_gain',
                 'complete_collision_geometry', 'object_surface_reconstructed'):
        assert result[name] is False
    assert row['execution_authority'] is row['collision_checked'] is False
    assert result['independent_motion_owner_required']
    json.dumps(result, allow_nan=False)
    assert 'object_points_base' not in row and 'grasp_pose' not in row


def test_unchanged_pose_and_pure_rotation_do_not_fake_new_viewpoint_information():
    roll = np.array([[1., 0., 0.], [0., math.cos(.05), -math.sin(.05)],
                     [0., math.sin(.05), math.cos(.05)]])
    transform = T.copy(); transform[:3, :3] = transform[:3, :3]@roll
    result = rank([pose('same', (0., 0., 0.)), ObservationPoseCandidate('rotate', transform)])
    assert result['ranked'] == []
    assert {r['reason'] for r in result['rejected']} == {'RGB_OBSERVATION_VIEW_NO_PREDICTED_GAIN'}


def test_closer_range_predicts_conservative_angular_sampling_gain():
    result = rank([pose('closer', (0., 0., -.03))])
    row = result['ranked'][0]
    assert row['minimum_angular_sampling_gain_ratio'] > .05
    assert row['resolution_utility'] > 0.


def test_farther_camera_cannot_claim_resolution_from_larger_uncertainty_projection():
    result = rank([pose('farther', (.04, 0., .02))])
    row = result['ranked'][0]
    assert row['minimum_angular_sampling_gain_ratio'] == 0.
    assert row['resolution_utility'] == 0.
    assert row['minimum_view_ray_change_rad'] > 0.


def test_same_camera_with_lower_joint_travel_ranks_first_and_does_not_wrap_angles():
    result = rank([pose('large', joint_positions=(2.*math.pi, 0.)),
                   pose('small', joint_positions=(.1, 0.))], current_joint_positions=(0., 0.))
    assert [r['candidate_id'] for r in result['ranked']] == ['small', 'large']
    assert result['ranked'][1]['joint_l1_delta_rad'] == pytest.approx(2.*math.pi)
    assert result['ranked'][0]['joint_motion_cost_available']
    assert result['joint_and_tool_fk_association_verified'] is False


def test_tool_motion_and_rotation_costs_are_included_as_endpoint_proxies():
    tool_near = T.copy(); tool_near[0, 3] += .01
    tool_far = T.copy(); tool_far[0, 3] += .1
    result = rank([pose('far_tool', T_base_tool=tool_far),
                   pose('near_tool', T_base_tool=tool_near)], current_T_base_tool=T)
    assert [r['candidate_id'] for r in result['ranked']] == ['near_tool', 'far_tool']
    assert result['ranked'][1]['tool_translation_m'] == pytest.approx(.1)
    assert result['ranked'][0]['tool_motion_cost_available']
    assert 'not_duration_or_path' in result['ranked'][0]['motion_cost_kind']


def test_missing_hints_cannot_outrank_fully_costed_joint_or_tool_candidate():
    result = rank([pose('unknown'), pose('known', joint_positions=(.1,))],
                  current_joint_positions=(0.,))
    assert [r['candidate_id'] for r in result['ranked']] == ['known']
    assert result['rejected'][0]['reason'].endswith('JOINT_HINT_MISSING')
    result = rank([pose('unknown'), pose('known', T_base_tool=T)], current_T_base_tool=T)
    assert [r['candidate_id'] for r in result['ranked']] == ['known']
    assert result['rejected'][0]['reason'].endswith('TOOL_HINT_MISSING')


@pytest.mark.parametrize('candidate,kwargs,reason', [
    (pose(joint_positions=(.1,)), {}, 'JOINT_BINDING_MISSING'),
    (pose(joint_positions=(.1, .2)), dict(current_joint_positions=(0.,)), 'JOINT_BINDING_MISSING'),
    (pose(T_base_tool=T), {}, 'TOOL_BINDING_MISSING'),
])
def test_fk_hint_does_not_establish_missing_current_binding(candidate, kwargs, reason):
    result = rank([candidate], **kwargs)
    assert not result['ranked'] and result['rejected'][0]['reason'].endswith(reason)


def test_large_motion_cannot_earn_unbounded_utility_and_small_useful_move_is_preferred():
    result = rank([pose('large', (.1, 0., 0.)), pose('small', (.015, 0., 0.))])
    assert len(result['ranked']) == 2
    assert [r['candidate_id'] for r in result['ranked']] == ['small', 'large']
    assert all(0. < r['bounded_utility'] < 1. for r in result['ranked'])


def test_whole_interest_not_just_reference_point_must_remain_in_view():
    volume = build(bbox_xywh=(0, 220, 640, 40))
    result = rank([pose('partial', (.001, 0., 0.))], volume)
    assert not result['ranked']
    assert result['rejected'][0]['reason'].endswith('PROJECTED_VOLUME_OUTSIDE_IMAGE')


def test_sensor_working_range_is_the_only_depth_limit_and_not_a_nominal_distance_band():
    near = rank([pose('closer', (0., 0., -.13))])
    far = rank([pose('farther_side', (.02, 0., .15))])
    assert near['ranked'] and far['ranked']
    beyond = rank([pose('outside', (0., 0., .3))])
    assert beyond['rejected'][0]['reason'].endswith('PROJECTED_VERTEX_OUTSIDE_WORKING_DEPTH')


def test_generated_candidates_are_bounded_local_multiaxis_and_have_no_fake_ik():
    generated = generate_local_rgb_observation_candidates(build(), current_T_base_camera=T)
    candidates = generated['candidates']
    assert len(candidates) == 20
    assert len({c.candidate_id for c in candidates}) == 20
    assert generated['observation_distance_band'] is None
    assert 'not_object_centre' in generated['interest_reference_semantics']
    assert generated['execution_authority'] is False
    assert all(c.joint_positions is None and c.T_base_tool is None for c in candidates)
    result = rank(list(candidates))
    assert result['ranked']
    assert all(row['camera_rotation_rad'] <= math.radians(6.)+1e-10 for row in result['ranked'])
    json.dumps(dict(generated, candidates=[asdict(c) for c in candidates]), allow_nan=False)


def test_orbits_preserve_interest_reference_camera_coordinates_without_claiming_object_centre():
    generated = generate_local_rgb_observation_candidates(build(), current_T_base_camera=T)
    reference = np.asarray(generated['interest_reference_base_m'])
    before = (reference-T[:3, 3])@T[:3, :3]
    orbits = [c for c in generated['candidates'] if 'orbit' in c.candidate_id]
    assert len(orbits) == 8
    for candidate in orbits:
        transform = np.asarray(candidate.T_base_camera)
        after = (reference-transform[:3, 3])@transform[:3, :3]
        np.testing.assert_allclose(after, before, atol=1e-12)


def test_generated_camera_pose_set_is_rigid_frame_covariant():
    from test_rgb_observation_volume import evidence, PLANE
    a = math.radians(35.)
    frame = np.array([[math.cos(a), -math.sin(a), 0., 2.],
                      [math.sin(a), math.cos(a), 0., -3.],
                      [0., 0., 1., 1.], [0., 0., 0., 1.]])
    plane = (0., 0., 1., -1.)
    moved_volume = build(T_base_camera=frame@T, support_plane_base=plane,
                        support_provenance=evidence(support_plane_base=plane))
    original = generate_local_rgb_observation_candidates(build(), current_T_base_camera=T)
    changed = generate_local_rgb_observation_candidates(moved_volume, current_T_base_camera=frame@T)
    for before, after in zip(original['candidates'], changed['candidates']):
        assert before.candidate_id == after.candidate_id
        np.testing.assert_allclose(after.T_base_camera, frame@np.asarray(before.T_base_camera), atol=1e-12)
    result1 = rank(list(original['candidates']))
    result2 = rank(list(changed['candidates']), moved_volume, current_T_base_camera=frame@T)
    # Symmetric candidates may exchange order at roundoff; per-ID utilities are invariant.
    by_id = {r['candidate_id']: r for r in result2['ranked']}
    for row in result1['ranked']:
        assert by_id[row['candidate_id']]['bounded_utility'] == pytest.approx(row['bounded_utility'], abs=1e-12)
        assert by_id[row['candidate_id']]['motion_cost_proxy'] == pytest.approx(row['motion_cost_proxy'], abs=1e-7)


@pytest.mark.parametrize('kwargs', [
    dict(translation_steps_m=(0.,)), dict(translation_steps_m=(True,)),
    dict(translation_steps_m=(.01, .02, .03)), dict(translation_steps_m=(.01, .01)),
    dict(orbit_angles_rad=(0.,)), dict(orbit_angles_rad=(float('nan'),)),
    dict(orbit_angles_rad=(math.radians(16.),)), dict(orbit_angles_rad=(.05, .05)),
])
def test_invalid_local_generation_preferences_are_rejected(kwargs):
    with pytest.raises(ValueError):
        generate_local_rgb_observation_candidates(build(), current_T_base_camera=T, **kwargs)


def test_zero_generation_steps_produce_no_candidates_and_no_actions():
    generated = generate_local_rgb_observation_candidates(build(), current_T_base_camera=T,
        translation_steps_m=(), orbit_angles_rad=())
    assert generated['candidates'] == () and generated['motion_authorized'] is False


@pytest.mark.parametrize('kwargs', [dict(translation_cost_scale_m=0.), dict(rotation_cost_scale_rad=True),
    dict(joint_cost_scale_rad=-1.), dict(novelty_scale_rad=np.nan), dict(resolution_gain_scale='1')])
def test_invalid_ranking_scales_rejected(kwargs):
    with pytest.raises(ValueError):
        rank([pose()], **kwargs)


@pytest.mark.parametrize('kwargs', [dict(joint_positions=(True,)), dict(joint_positions=('1',)),
    dict(joint_positions=()), dict(joint_positions=(float('inf'),)), dict(candidate_id=''),
    dict(T_base_tool=np.zeros((4, 4)))])
def test_invalid_fk_candidate_fields_rejected(kwargs):
    values = dict(candidate_id='test', T_base_camera=T)
    values.update(kwargs)
    with pytest.raises(ValueError):
        ObservationPoseCandidate(**values)


def test_untyped_or_duplicate_candidates_rejected_instead_of_silently_costed():
    with pytest.raises(ValueError, match='CANDIDATE_TYPE_INVALID'):
        rank([{}])
    with pytest.raises(ValueError, match='DUPLICATE_CANDIDATE_ID'):
        rank([pose(), pose()])
    with pytest.raises(ValueError, match='CANDIDATE_SET_INVALID'):
        rank([pose(str(i)) for i in range(129)])


def test_invalid_current_pose_or_interest_type_rejected_before_ranking():
    with pytest.raises(ValueError):
        rank([pose()], current_T_base_camera=np.zeros((4, 4)))
    with pytest.raises(ValueError, match='VOLUME_TYPE_INVALID'):
        rank([pose()], volume={})


@pytest.mark.parametrize('margin', [-1., True, '36', np.nan, np.inf, 239.5, 240.])
def test_invalid_roi_margins_are_rejected(margin):
    with pytest.raises(ValueError, match='ROI_MARGIN_INVALID'):
        rank([pose()], roi_margin_px=margin)


def test_positive_roi_margin_is_inclusive_on_existing_pixel_centre_boundary():
    # Full pixel-area box edge x-.5 equals the existing 36px centre limit.
    volume = build(bbox_xywh=(36.5, 220, 40, 40))
    result = rank([pose('inward', (-.001, 0., 0.))], volume, roi_margin_px=36.)
    assert result['ranked'] and result['roi_margin_px'] == 36.
    assert 'pixel_centre_limits' in result['roi_margin_convention']
    for row in result['ranked']:
        assert np.min(np.asarray(row['projected_interest_vertices_px'])[:, 0]) >= 36.


def test_roi_rejects_current_edge_outside_even_when_complete_volume_is_in_full_fov():
    volume = build(bbox_xywh=(36.49, 220, 40, 40))
    assert rank([pose('inward', (-.001, 0., 0.))], volume, roi_margin_px=0.)['ranked']
    with pytest.raises(ValueError, match='CURRENT_VOLUME_OUTSIDE_ROI'):
        rank([pose('inward', (-.001, 0., 0.))], volume, roi_margin_px=36.)


def test_roi_rejects_candidate_margin_violation_without_discarding_other_candidates():
    volume = build(bbox_xywh=(36.5, 220, 40, 40))
    result = rank([pose('outward', (.001, 0., 0.)), pose('inward', (-.001, 0., 0.))],
                  volume, roi_margin_px=36.)
    assert [row['candidate_id'] for row in result['ranked']] == ['inward']
    assert result['rejected'] == [dict(candidate_id='outward',
        reason='RGB_OBSERVATION_VIEW_CANDIDATE_VOLUME_OUTSIDE_ROI')]


def test_positive_roi_margin_checks_right_and_bottom_full_corner_extents():
    for box in ((600., 220., 3.6, 40.), (300., 440., 40., 3.6)):
        volume = build(bbox_xywh=box)
        with pytest.raises(ValueError, match='CURRENT_VOLUME_OUTSIDE_ROI'):
            rank([pose('inward', (-.001, -.001, 0.))], volume, roi_margin_px=36.)


def test_acquisition_window_unchecked_is_explicit_by_default():
    result = rank([pose()])
    assert result['acquisition_window_checked'] is False
    assert result['acquisition_radius_px'] is None
    assert result['current_maximum_acquisition_radius_px'] is None
    assert result['ranked'][0]['acquisition_window_checked'] is False
    assert result['ranked'][0]['maximum_acquisition_radius_px'] is None


def test_acquisition_window_checks_slab_top_not_just_floor_or_bbox_centre():
    candidates = [pose('top_leaves_window', (.04, 0., 0.)), pose('retained', (.02, 0., 0.))]
    unchecked = rank(candidates)
    assert len(unchecked['ranked']) == 2
    failing = next(row for row in unchecked['ranked'] if row['candidate_id'] == 'top_leaves_window')
    pixels = np.asarray(failing['projected_interest_vertices_px'])
    centre = np.array([320., 240.])
    assert np.max(np.linalg.norm(pixels[:4]-centre, axis=1)) < 60.
    assert np.linalg.norm(np.mean(pixels[4:], axis=0)-centre) < 60.
    assert np.max(np.linalg.norm(pixels[4:]-centre, axis=1)) > 60.
    result = rank(candidates, acquisition_radius_fraction=.125)
    assert [row['candidate_id'] for row in result['ranked']] == ['retained']
    assert result['rejected'][0]['reason'].endswith('CANDIDATE_VOLUME_OUTSIDE_ACQUISITION_WINDOW')
    assert result['rejected'][0]['maximum_acquisition_radius_px'] > 60.
    assert result['acquisition_radius_px'] == 60.
    assert result['acquisition_scope'] == 'existing_perception_entry_window_not_rgbd_distance_band'
    assert result['acquisition_centres_px'] == ((320., 240.), (319.5, 239.5))
    assert result['ranked'][0]['acquisition_window_checked'] is True


def test_acquisition_current_boundary_is_inclusive_and_does_not_widen_radius():
    volume = build()
    points = np.asarray(volume.pixel_boundary_xy)
    centres = np.array([[320., 240.], [319.5, 239.5]])
    boundary = float(np.max(np.linalg.norm(points[:, None, :]-centres[None, :, :], axis=2)))
    result = rank([], volume, acquisition_radius_fraction=boundary/480.)
    assert result['current_maximum_acquisition_radius_px'] == pytest.approx(boundary)
    with pytest.raises(ValueError, match='CURRENT_VOLUME_OUTSIDE_ACQUISITION_WINDOW'):
        rank([], volume, acquisition_radius_fraction=(boundary-1e-6)/480.)


def test_acquisition_candidate_boundary_is_inclusive_for_all_eight_vertices():
    candidate = pose('boundary', (.025, 0., 0.))
    unchecked = rank([candidate])['ranked'][0]
    points = np.asarray(unchecked['projected_interest_vertices_px'])
    centres = np.array([[320., 240.], [319.5, 239.5]])
    boundary = float(np.max(np.linalg.norm(points[:, None, :]-centres[None, :, :], axis=2)))
    assert rank([candidate], acquisition_radius_fraction=boundary/480.)['ranked']
    result = rank([candidate], acquisition_radius_fraction=(boundary-1e-6)/480.)
    assert result['ranked'] == []
    assert result['rejected'][0]['reason'].endswith('CANDIDATE_VOLUME_OUTSIDE_ACQUISITION_WINDOW')


def test_noncentral_acquisition_interest_obeys_explicit_normalized_centre():
    volume = build(bbox_xywh=(140., 100., 40., 40.))
    result = rank([pose('small', (.005, 0., 0.))], volume,
                  acquisition_center_xy=(.25, .25), acquisition_radius_fraction=.125)
    assert result['ranked']
    assert result['acquisition_centres_px'] == ((160., 120.), (159.75, 119.75))
    with pytest.raises(ValueError, match='CURRENT_VOLUME_OUTSIDE_ACQUISITION_WINDOW'):
        rank([pose()], volume, acquisition_radius_fraction=.125)


@pytest.mark.parametrize('fraction', [0., -.1, True, '0.125', np.nan, np.inf, 1.001])
def test_invalid_acquisition_radius_is_rejected(fraction):
    with pytest.raises(ValueError, match='ACQUISITION_RADIUS_INVALID'):
        rank([pose()], acquisition_radius_fraction=fraction)


@pytest.mark.parametrize('centre', [(True, .5), ('0.5', .5), (-.1, .5), (1.01, .5),
                                    (.5,), (.5, np.nan), None])
def test_invalid_acquisition_centre_is_rejected_even_when_window_unchecked(centre):
    with pytest.raises(ValueError, match='ACQUISITION_CENTRE_INVALID'):
        rank([pose()], acquisition_center_xy=centre)


def test_history_returns_to_initial_or_first_settled_view_have_no_incremental_gain():
    first=T.copy();first[0,3]+=.025
    current=T.copy();current[1,3]+=.025
    result=rank([ObservationPoseCandidate('initial',T),ObservationPoseCandidate('first',first)],
        current_T_base_camera=current,observed_T_base_cameras=(T,first,current))
    assert result['ranked']==[]
    assert all(row['reason'].endswith('NO_PREDICTED_GAIN') for row in result['rejected'])
    assert result['history_aware_utility'] and result['observed_view_count']==3
    assert result['gain_deadzone_pixels']==1. and not result['measured_information_gain']


@pytest.mark.parametrize('kind',['ulp','subpixel','rotate'])
def test_history_roundoff_subpixel_drift_and_same_optical_centre_rotation_do_not_fake_gain(kind):
    previous=T.copy();current=T.copy();current[0,3]+=.025
    candidate=previous.copy()
    if kind=='ulp': candidate[0,3]=np.nextafter(candidate[0,3],np.inf)
    elif kind=='subpixel': candidate[0,3]+=.0001;candidate[2,3]-=.00001
    else:
        angle=.05
        candidate[:3,:3]=candidate[:3,:3]@np.array([[1.,0.,0.],[0.,math.cos(angle),-math.sin(angle)],
                                                             [0.,math.sin(angle),math.cos(angle)]])
    result=rank([ObservationPoseCandidate(kind,candidate)],current_T_base_camera=current,
        observed_T_base_cameras=(previous,current))
    assert not result['ranked'] and result['rejected'][0]['reason'].endswith('NO_PREDICTED_GAIN')


def test_history_new_direction_and_materially_closer_view_still_have_positive_gain():
    first=T.copy();first[0,3]+=.025
    farther=T.copy();farther[2,3]+=.01
    result=rank([pose('new_direction',(0.,.025,0.)),pose('closer',(0.,0.,-.06))],
        current_T_base_camera=farther,observed_T_base_cameras=(T,first,farther))
    by_id={row['candidate_id']:row for row in result['ranked']}
    assert by_id['new_direction']['novelty_utility']>0.
    assert by_id['closer']['resolution_utility']>0.
    assert all(row['bounded_utility']>0. for row in by_id.values())


def test_history_resolution_compares_best_observed_distance_not_current():
    closer=T.copy();closer[2,3]-=.03
    farther=T.copy();farther[2,3]+=.03
    row=rank([pose('side',(.015,0.,0.))],current_T_base_camera=farther,
        observed_T_base_cameras=(closer,farther))['ranked'][0]
    assert row['resolution_utility']==0. and row['minimum_angular_sampling_gain_ratio']==0.
    assert row['novelty_utility']>0.


@pytest.mark.parametrize('history',[(T,)*5,(np.zeros((4,4)),),None])
def test_history_is_bounded_and_rigid(history):
    with pytest.raises(ValueError):rank([pose()],observed_T_base_cameras=history)


def test_original_frozen_frustum_cannot_be_replaced_by_current_smaller_frustum():
    original = build(bbox_xywh=(36.5,220.,40.,40.))
    vertices = original.vertices_base_m
    candidates = [pose('outward',(.001,0.,0.)),pose('inward',(-.001,0.,0.))]
    assert len(rank(candidates,roi_margin_px=36.)['ranked']) == 2
    result = rank(candidates,roi_margin_px=36.,frozen_reference_vertices=(vertices,vertices))
    assert [row['candidate_id'] for row in result['ranked']] == ['inward']
    assert result['rejected'][0]['reason'].endswith('CANDIDATE_FROZEN_REFERENCE_OUTSIDE_ROI')
    assert result['rejected'][0]['frozen_reference_index'] == 0
    assert result['utility_geometry'] == 'original_frozen_envelope'
    assert result['frozen_reference_volume_count'] == 2
    assert original.vertices_base_m == vertices
    assert len(result['ranked'][0]['projected_frozen_reference_vertices_px']) == 2


def test_every_original_member_is_checked_even_if_envelope_and_current_are_visible():
    centre = build().vertices_base_m
    edge = build(bbox_xywh=(36.5,220.,40.,40.)).vertices_base_m
    result = rank([pose('outward',(.001,0.,0.))],roi_margin_px=36.,
        frozen_reference_vertices=(centre,centre,edge))
    assert not result['ranked']
    assert result['rejected'][0]['frozen_reference_index'] == 2


def test_original_frozen_working_depth_is_preserved_alongside_current_depth():
    vertices = build(foreground_max_m=.4).vertices_base_m
    candidate = pose('closer',(0.,0.,-.06))
    assert rank([candidate])['ranked']
    result = rank([candidate],frozen_reference_vertices=(vertices,vertices))
    assert not result['ranked']
    assert result['rejected'][0]['reason'].endswith('PROJECTED_VERTEX_OUTSIDE_WORKING_DEPTH')


def test_frozen_geometry_supplies_history_gain_despite_current_frustum_changes():
    vertices = build().vertices_base_m
    options = dict(frozen_reference_vertices=(vertices,vertices),observed_T_base_cameras=(T,))
    first = rank([pose()],**options)['ranked'][0]
    second = rank([pose()],build(bbox_xywh=(400,240,40,40)),**options)['ranked'][0]
    for key in ('bounded_utility','raw_minimum_view_ray_change_rad',
                'raw_minimum_angular_sampling_gain_ratio','resolution_deadzone_ratio'):
        assert first[key] == second[key]
    current=T.copy();current[0,3]+=.025
    result=rank([ObservationPoseCandidate('revisit',T)],current_T_base_camera=current,**options)
    assert not result['ranked'] and result['rejected'][0]['reason'].endswith('NO_PREDICTED_GAIN')


def test_current_volume_constraints_remain_when_frozen_reference_is_visible():
    vertices=build().vertices_base_m
    edge=build(bbox_xywh=(36.5,220,40,40))
    result=rank([pose('outward',(.001,0.,0.))],edge,roi_margin_px=36.,
        frozen_reference_vertices=(vertices,vertices))
    assert result['rejected'][0]['reason'].endswith('CANDIDATE_VOLUME_OUTSIDE_ROI')


def test_current_original_reference_outside_margin_cannot_start_a_new_rank():
    vertices=build(bbox_xywh=(36.,220,40,40)).vertices_base_m
    with pytest.raises(ValueError,match='CURRENT_FROZEN_REFERENCE_OUTSIDE_ROI'):
        rank([pose()],roi_margin_px=36.,frozen_reference_vertices=(vertices,vertices))


@pytest.mark.parametrize('vertices',[None,(np.zeros((7,3)),)*2,
    (np.full((8,3),np.nan),)*2,(np.ones((8,3),dtype=bool),)*2])
def test_frozen_input_schema_is_strict_and_never_grants_authority(vertices):
    with pytest.raises(ValueError):rank([pose()],frozen_reference_vertices=vertices)


@pytest.mark.parametrize('count',[1,6])
def test_frozen_set_requires_envelope_plus_bounded_members(count):
    with pytest.raises(ValueError,match='FROZEN_REFERENCE_SET_INVALID'):
        rank([pose()],frozen_reference_vertices=(build().vertices_base_m,)*count)
