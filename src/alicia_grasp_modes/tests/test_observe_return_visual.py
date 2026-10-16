"""Independent visual gaps, exact request binding, and no implicit motion."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS
import json

import numpy as np
import pytest

from alicia_grasp_modes.observe_return_visual import bounded_bilateral_view_gaps
from test_observe_return_remote import configured, setup_observe_return
from test_tabletop_geometry_candidates import (GRIPPER, TabletopGeometryConfig,
    SurfaceView, TargetTrackIdentity, fused_surface_from_view, generate_tabletop_proposals,
    rotation_about_z)


def recorded():
    data = json.loads((Path(__file__).parent / 'fixtures/reoriented_initial_view_gap.json').read_text())
    g = data['geometry']
    geometry = NS(axes_base=np.array(g['R_base_obb']), size_xyz_m=np.array(g['obb_size_xyz_m']),
                  support_normal_base=np.array(g['support_normal_base']),
                  center_base=np.array([0., 0., .5*g['obb_size_xyz_m'][2]]))
    return data, geometry, TabletopGeometryConfig()


def test_recorded_mixed_rejections_retain_only_whole_body_fitting_missing_sides():
    data, geometry, cfg = recorded()
    assert data['tabletop_geometry']['materialized_count'] > 0
    gaps = bounded_bilateral_view_gaps(data, geometry, cfg)
    assert gaps
    assert gaps[0]['angle_deg'] == pytest.approx(41.816391655457124)
    assert gaps[0]['required_open_width_m'] == pytest.approx(.04303185044395056)
    assert all(not gap['contact_or_motion_authorized'] for gap in gaps)
    assert all(gap['required_open_width_m'] <= cfg.max_inner_gap_m for gap in gaps)
    # Centre-line fits at 65--75 degrees are not whole-object aperture fits.
    assert all(gap['angle_deg'] < 60. for gap in gaps)


@pytest.mark.parametrize('yaw', [0., 73., -137.])
@pytest.mark.parametrize('size', [(.064,.035,.02), (.047,.027,.018)])
def test_generated_missing_surfaces_are_frame_and_size_independent(yaw, size):
    r = rotation_about_z(np.deg2rad(yaw))
    p = np.array([(x,y,size[2]) for x in np.linspace(-size[0]/2,size[0]/2,21)
                  for y in np.linspace(-size[1]/2,size[1]/2,17)]) @ r.T
    view = SurfaceView(TargetTrackIdentity.from_stream(3,9), 1_000_000_000, p,
                       np.array([0.,0.,1.]), 0., 12)
    cfg = TabletopGeometryConfig()
    generated = generate_tabletop_proposals(p,np.array([0.,0.,size[2]/2]),r,np.array(size),
        np.zeros(3),np.array([0.,0.,1.]),cfg,fused_surface_from_view(view),GRIPPER)
    diagnostics = {'tabletop_geometry':dict(plan_phase='CONTACT_EXECUTION_PLAN',
        contact_execution_gate_deferred=False,surface_evidence_rejections=generated.surface_evidence_rejections)}
    geometry = NS(axes_base=r,size_xyz_m=np.array(size),support_normal_base=np.array([0.,0.,1.]))
    gaps = bounded_bilateral_view_gaps(diagnostics,geometry,cfg)
    assert gaps
    assert gaps[0]['whole_obb_width_m'] == pytest.approx(min(size[:2]))
    assert abs(np.dot(gaps[0]['jaw_axis_base'],r[:,1])) == pytest.approx(1.)
    assert not generated.proposals  # An OBB has not invented the missing side.


@pytest.mark.parametrize('fault', ['no_rows','pure_collision','oversize','nonrigid','nan',
    'normal','angle_bool','negative_reach','fake_limit','deferred','wrong_phase'])
def test_no_missing_side_or_bad_geometry_cannot_authorize_view(fault):
    data,g,cfg=recorded();table=data['tabletop_geometry']
    if fault=='no_rows':table['surface_evidence_rejections']=[]
    elif fault=='pure_collision':
        for row in table['surface_evidence_rejections']:row['failure_code']='GRIPPER_SWEEP_COLLISION'
    elif fault=='oversize':g.size_xyz_m=np.array([.08,.07,.02])
    elif fault=='nonrigid':g.axes_base[0,0]+=0.1
    elif fault=='nan':g.size_xyz_m[0]=float('nan')
    elif fault=='normal':g.support_normal_base=-g.support_normal_base
    elif fault in ('angle_bool','negative_reach','fake_limit'):
        key,value={'angle_bool':('angle_deg',True),'negative_reach':('negative_reach_m',-.01),
                   'fake_limit':('maximum_side_reach_m',.026)}[fault]
        for row in table['surface_evidence_rejections']:row[key]=value
    elif fault=='deferred':table['contact_execution_gate_deferred']=True
    elif fault=='wrong_phase':table['plan_phase']='FAR_FIELD_OBSERVATION_PLAN'
    assert not bounded_bilateral_view_gaps(data,g,cfg)


def mixed_request(configured):
    node,_,_=setup_observe_return(configured)
    data,g,cfg=recorded();audit=data['audit'];node._active_gate_audit_report=deepcopy(audit)
    node.tabletop_geometry_config=cfg
    audit = node._active_gate_audit_report
    audit.update(generation=node._stream_generation, target_epoch=node.target_instance_epoch,
                 snapshot_stamp_ns=node._active_multiview_surface()[0].view_stamps_ns[0])
    prepared=NS(ticket=NS(**{k:audit[k] for k in ('request_id','generation','target_epoch')}),
        snapshot=NS(stamp_ns=audit['snapshot_stamp_ns'], target_identity=node._current_stream_target_identity()),geometry=g,
        remote_diagnostics=data,remote_failure_code='')
    return node,prepared


@pytest.mark.parametrize('fault', ['','stale_request','generation','epoch','stamp','selected',
    'reachable','unchecked','terminated','empty_search','server','phase','strategy','no_gap','terminal'])
def test_mixed_opportunity_requires_completed_bound_initial_request(configured,fault):
    node,p=mixed_request(configured);report=node._active_gate_audit_report
    code='NEAR_FIELD_NO_REACHABLE_CANDIDATE'
    if fault in ('stale_request','generation','epoch','stamp'):
        key={'stale_request':'request_id','generation':'generation','epoch':'target_epoch','stamp':'snapshot_stamp_ns'}[fault]
        report[key]+=1
    elif fault=='selected':report['selected']={'plan_id':'another-valid-contact'}
    elif fault=='reachable':report['moveit_selection']['reachable_count']=1
    elif fault=='unchecked':report['moveit_selection']['shortlist_count']+=1
    elif fault=='terminated':report['moveit_selection']['terminated_early']=True
    elif fault=='empty_search':report['moveit_selection']['checked_count']=0
    elif fault=='server':p.remote_failure_code='WSL_UNAVAILABLE'
    elif fault=='phase':node._observe_return_phase='observing_contact'
    elif fault=='strategy':node._mode_selection['strategy']='two_stage'
    elif fault=='no_gap':p.remote_diagnostics['tabletop_geometry']['surface_evidence_rejections']=[]
    elif fault=='terminal':code='NEAR_FIELD_DIRECT_TIMEOUT'
    gap=node._observe_return_mixed_visual_failure(p,code)
    assert bool(gap) is (not fault)


def test_view_transition_retains_measured_surface_and_records_reason_without_motion(configured):
    node,p=mixed_request(configured);surface=node._active_multiview_surface()[0]
    before=deepcopy(p.remote_diagnostics)
    node._publish_direct_near_field_terminal(p,'NEAR_FIELD_NO_REACHABLE_CANDIDATE','paths rejected')
    assert node._observe_return_phase=='observation_prefix'
    assert node._observe_return_saved[0] is surface
    assert node._observe_return_trigger=='BILATERAL_SURFACE_EVIDENCE_MISSING'
    assert node._observe_return_trigger_evidence['contact_or_motion_authorized'] is False
    assert p.remote_diagnostics==before
    assert not node.preview_rich_plan_pub.messages[-1].valid
    assert node.status_pub.messages[-1].data.startswith('OBSERVE_RETURN_VIEW_REQUIRED')


def test_opportunity_cannot_be_reused_after_request_changes(configured):
    node,p=mixed_request(configured)
    gap=node._observe_return_mixed_visual_failure(p,'NEAR_FIELD_NO_REACHABLE_CANDIDATE')
    node._active_gate_audit_report['request_id']+=1
    assert not node._observe_return_fallback(p,visual_opportunity=gap)
    assert node._observe_return_phase=='initial_contact'


from alicia_grasp_modes.observe_return_visual import measured_bilateral_view_gaps, missing_side_view_gain


def measured_gap_fixture(yaw=0., size=(.045, .030, .020), include_negative=True):
    rotation = rotation_about_z(np.deg2rad(yaw))
    center = np.array([0., 0., size[2]/2])
    xs, ys, zs = np.linspace(-size[0]/2, size[0]/2, 25), np.linspace(-size[1]/2, size[1]/2, 21), np.linspace(0., size[2], 19)
    points = [(x, y, size[2]) for x in xs for y in ys]
    if include_negative:
        points += [(x, -size[1]/2, z) for x in xs for z in zs]
    points = np.asarray(points) @ rotation.T
    surface = fused_surface_from_view(SurfaceView(TargetTrackIdentity.from_stream(4, 2),
        1_000_000_000, points, np.array([0., 0., 1.]), 0., 12))
    cfg = TabletopGeometryConfig()
    geometry = NS(center_base=center, axes_base=rotation, size_xyz_m=np.array(size),
                  support_normal_base=np.array([0., 0., 1.]))
    generated = generate_tabletop_proposals(points, center, rotation, np.array(size),
        np.zeros(3), geometry.support_normal_base, cfg, surface, GRIPPER)
    diagnostics = {'tabletop_geometry':dict(plan_phase='CONTACT_EXECUTION_PLAN',
        contact_execution_gate_deferred=False, surface_evidence_rejections=generated.surface_evidence_rejections)}
    initial = center + rotation @ np.array([0., -.15, .15])
    gaps = measured_bilateral_view_gaps(diagnostics, geometry, cfg, surface, GRIPPER, initial)
    return rotation, center, surface, initial, gaps


@pytest.mark.parametrize('yaw', [0., 73., -137.])
@pytest.mark.parametrize('size', [(.045,.030,.020), (.060,.035,.027)])
def test_measured_gap_selects_missing_side_across_rotations_and_sizes(yaw, size):
    rotation, center, surface, initial, gaps = measured_gap_fixture(yaw, size)
    assert gaps
    gap = gaps[0]
    jaw = np.array(gap['jaw_axis_base'])
    positive_normal = rotation[:, 1]
    expected_sign = 1 if jaw @ positive_normal > 0. else -1
    assert gap['missing_side_signs'] == [expected_sign]
    assert gap['source_view_stamps_ns'] == list(surface.view_stamps_ns)
    opposite = center + rotation @ np.array([0., .15, .15])
    gain = missing_side_view_gain((gap,), opposite, .002)
    assert gain['ok'] and gain['missing_side_sign'] == expected_sign
    assert gain['predicted_new_side_gain_m'] > .002
    assert not gain['contact_or_motion_authorized'] and gain['measured_registration_required']
    # Same optical bearing with >15 mm translation is not a new side.
    assert not missing_side_view_gain((gap,), center + .7*(initial-center), .002)['ok']
    assert not missing_side_view_gain((gap,), initial, .002)['ok']


def test_missing_side_gain_is_prediction_and_must_exceed_live_uncertainty():
    rotation, center, _, _, gaps = measured_gap_fixture()
    gap = gaps[0]
    assert not missing_side_view_gain((gap,), center + [0., .15, .15], .03)['ok']
    assert not missing_side_view_gain((gap,), center + [0., .005, .02], 0.)['ok']
    assert not missing_side_view_gain((), center + [0., .15, .15], .001)['ok']
    assert not missing_side_view_gain(gaps, [float('nan'),0.,0.], .001)['ok']


def test_top_only_retains_both_missing_sides_without_inventing_points():
    rotation, center, surface, initial, gaps = measured_gap_fixture(include_negative=False)
    before = surface.points_base.copy()
    assert gaps and gaps[0]['missing_side_signs'] == [-1,1]
    assert missing_side_view_gain((gaps[0],), center + [0., .15, .15], .002)['ok']
    assert np.array_equal(before, surface.points_base)


def test_view_filter_rejects_all_existing_same_side_views(configured, monkeypatch):
    from test_remote_modes import base
    node, _, _ = setup_observe_return(configured)
    node.grasp_config = {}
    _, center, _, initial, gaps = measured_gap_fixture()
    identity = node._current_stream_target_identity()
    node._observe_return_phase = 'observation_prefix'
    node._observe_return_view_gaps = tuple(dict(g, target_epoch=identity.epoch,
        target_track_id=identity.track_id) for g in gaps[:1])
    node._tool_from_camera_matrix = lambda: np.eye(4)
    def sequence(camera):
        pose = base.PoseStamped()
        pose.pose.orientation.w = 1.
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = camera
        return NS(pregrasp=pose, adaptive_stage_profile=NS(depth_uncertainty_m=.002),
            observation_view_audit=dict(working_volume=dict(target_corner_depths_m=[.3]*8)))
    monkeypatch.setattr(node, '_camera_intrinsics', lambda: NS(fx=400.,fy=400.))
    same = NS(**vars(sequence(center + .7*(initial-center))))
    opposite = sequence(center + [0., .15, .15])
    variants = tuple(dict(sequence=item, observation_translation_delta_m=.1) for item in (same, opposite))
    monkeypatch.setattr(base.RemoteGrasp6DNode, '_far_field_observation_variants',
                        lambda self, *args, **kwargs: (variants, ()))
    passing, rejected = node._far_field_observation_variants()
    assert len(passing) == 1 and passing[0]['sequence'] is opposite
    assert rejected[0]['failure_code'] == 'OBSERVE_RETURN_MISSING_SIDE_NOT_EXPOSED'
    monkeypatch.setattr(base.RemoteGrasp6DNode, '_far_field_observation_variants',
                        lambda self, *args, **kwargs: (variants[:1], ()))
    passing, rejected = node._far_field_observation_variants()
    assert not passing and len(rejected) == 1
    node._observe_return_view_gaps = ()
    assert not node._far_field_observation_variants()[0]


@pytest.mark.parametrize('fault', ['generation', 'target_epoch', 'surface_identity', 'source_stamp'])
def test_fallback_rechecks_live_ticket_and_saved_source(configured, fault):
    from test_remote_modes import base
    node, prepared = mixed_request(configured)
    gap = node._observe_return_mixed_visual_failure(prepared, 'NEAR_FIELD_NO_REACHABLE_CANDIDATE')
    if fault in ('generation', 'target_epoch'):
        if fault == 'generation': node._stream_generation += 1
        else: node.target_instance_epoch += 1
        with pytest.raises(base.StreamResultCancelled):
            node._observe_return_fallback(prepared, visual_opportunity=gap)
    else:
        if fault == 'surface_identity': prepared.snapshot.target_identity = TargetTrackIdentity.from_stream(999,999)
        else:
            prepared.snapshot.stamp_ns += 1
            node._active_gate_audit_report['snapshot_stamp_ns'] += 1
            gap = node._observe_return_mixed_visual_failure(prepared, 'NEAR_FIELD_NO_REACHABLE_CANDIDATE')
        assert not node._observe_return_fallback(prepared, visual_opportunity=gap)
    assert node._observe_return_phase == 'initial_contact'


def test_final_view_selection_repeats_missing_side_screen_and_target_binding(configured, monkeypatch):
    from test_remote_modes import base
    node, _, _ = setup_observe_return(configured)
    _, center, _, initial, gaps = measured_gap_fixture()
    identity = node._current_stream_target_identity()
    node._observe_return_phase = 'observation_prefix'
    node._observe_return_view_gaps = tuple(dict(g, target_epoch=identity.epoch,
        target_track_id=identity.track_id) for g in gaps[:1])
    node._tool_from_camera_matrix = lambda: np.eye(4)
    node._frozen_observation_translation_delta_m = lambda prepared, pose: .1
    node._parse_plan_metrics = lambda reason: {'execution_duration_lower_bound_sec': float(reason)}
    candidates = []
    node._stable_variant_runtime = {}
    for index, camera in enumerate((center + .7*(initial-center), center + [0.,.15,.15])):
        pose = base.PoseStamped()
        pose.pose.orientation.w = 1.
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = camera
        sequence = NS(pregrasp=pose, adaptive_stage_profile=NS(depth_uncertainty_m=.002))
        candidate = NS(track_id=index, variant_index=0, pre_moveit_score=0.,
            moveit_result=NS(reason=str(index+1), joint_max_delta_rad=.1, joint_path_cost=.1))
        node._stable_variant_runtime[(index,0)] = dict(observation_sequence=sequence,
            prepared=NS(), observation_view=dict(information_utility=dict(utility=.5)),
            soft_evidence=dict(observation_side_evidence_deficit_m=0.,
            observation_translation_delta_m=.1, observation_projected_side_evidence_m=.01))
        candidates.append(candidate)
    # A faster same-side path cannot win even if it escaped proposal screening.
    assert node._select_far_field_observation(candidates) is candidates[1]
    assert node._select_far_field_observation(candidates[:1]) is None
    node._observe_return_view_gaps = tuple(dict(g, target_epoch=g['target_epoch']+1)
                                         for g in node._observe_return_view_gaps)
    assert node._select_far_field_observation(candidates) is None


def test_resolved_measured_sides_cannot_be_reused_as_a_missing_direction():
    _, center, surface, initial, gaps = measured_gap_fixture()
    points = np.vstack((surface.points_base, surface.points_base * np.array([1.,-1.,1.])))
    completed = fused_surface_from_view(SurfaceView(surface.identity, 1_000_000_000,
        points, np.array([0.,0.,1.]), 0., 12))
    geometry = NS(center_base=center, axes_base=np.eye(3), size_xyz_m=np.array([.045,.030,.020]),
                  support_normal_base=np.array([0.,0.,1.]))
    diagnostics = dict(tabletop_geometry=dict(plan_phase='CONTACT_EXECUTION_PLAN',
        contact_execution_gate_deferred=False, surface_evidence_rejections=list(gaps[:1])))
    assert not measured_bilateral_view_gaps(diagnostics, geometry, TabletopGeometryConfig(),
        completed, GRIPPER, initial)


from alicia_grasp_modes.observe_return_visual import directed_missing_side_rotations


def camera_rotation_toward(center, origin, normal=np.array([0.,0.,1.])):
    optical = np.asarray(center)-np.asarray(origin)
    optical /= np.linalg.norm(optical)
    second = np.cross(normal, optical); second /= np.linalg.norm(second)
    return np.column_stack((optical, second, np.cross(optical, second)))


@pytest.mark.parametrize('yaw', [0., 73., -137.])
@pytest.mark.parametrize('size', [(.045,.030,.020), (.060,.035,.027)])
def test_directed_family_is_covariant_and_uses_measured_weak_side(yaw, size):
    rotation, center, _, _, gaps = measured_gap_fixture(yaw, size)
    camera = center + rotation @ np.array([0.,-.08,.20])
    gap = dict(gaps[0], initial_camera_origin_base_m=camera.tolist())
    initial_rotation = camera_rotation_toward(center, camera)
    views = directed_missing_side_rotations((gap,), initial_rotation, np.array([0.,0.,1.]), .002, .18)
    assert len(views) == 2
    for proposed, metadata in views:
        assert metadata['missing_side_sign'] in gap['missing_side_signs']
        assert 0. < metadata['camera_rotation_offset_deg'] <= 45. + 1e-9
        assert metadata['predicted_new_side_gain_m'] > .002
        assert metadata['orientation_source'] == 'measured_missing_side_geodesic'
        assert not metadata['contact_or_motion_authorized']
        assert np.allclose(proposed.T@proposed, np.eye(3), atol=1e-12)
        assert missing_side_view_gain((gap,), center-.18*proposed[:,0], .002)['ok']
    _, zero_center, _, _, zero_gaps = measured_gap_fixture(0., size)
    zero_camera = zero_center + np.array([0.,-.08,.20])
    zero_gap = dict(zero_gaps[0], initial_camera_origin_base_m=zero_camera.tolist())
    zero_views = directed_missing_side_rotations((zero_gap,),camera_rotation_toward(zero_center,zero_camera),
        np.array([0.,0.,1.]), .002, .18)
    for (actual, _), (zero, _) in zip(views, zero_views):
        assert np.allclose(actual, rotation@zero, atol=1e-10)
    assert views[0][1]['camera_rotation_offset_deg'] < views[1][1]['camera_rotation_offset_deg']


def test_directed_family_cannot_expand_45_degree_bound_or_accept_wrong_inputs():
    _,center,_,initial,gaps = measured_gap_fixture()
    # The original 45-degree off-normal view needs >45 degrees to expose the opposite side.
    assert not directed_missing_side_rotations(gaps[:1], camera_rotation_toward(center,initial),
        [0.,0.,1.], .003, .18)
    near = center + [0.,-.08,.20]
    gap=dict(gaps[0],initial_camera_origin_base_m=near.tolist())
    assert not directed_missing_side_rotations((gap,),camera_rotation_toward(center,near),
        [0.,0.,1.], .003, .18, maximum_rotation_deg=46.)
    assert not directed_missing_side_rotations((gap,),camera_rotation_toward(center,near),
        [0.,0.,1.], .03, .18)
    assert not directed_missing_side_rotations((gap,),camera_rotation_toward(center,near),
        [0.,0.,1.], .003, .18, maximum_directions=3)


def directed_expansion_fixture(minimum_deg=46., uncertainty=.003, height=.020,
                               yaw=0., center=(.12, -.07, .015)):
    """A measured missing +Y side with an analytically known minimum turn."""
    center = np.asarray(center, dtype=float)
    frame = rotation_about_z(np.deg2rad(yaw))
    # Opposite-side initial view: previous exposure is zero. Keep the side
    # plane requirement below the information requirement in this fixture.
    width = .2 * (uncertainty + 1e-6)
    initial_tilt = np.deg2rad(minimum_deg) - np.arcsin((uncertainty + 1e-6) / height)
    initial = center + frame @ (.26*np.array([0., -np.sin(initial_tilt), np.cos(initial_tilt)]))
    gap = dict(angle_deg=(90.+yaw) % 180., jaw_axis_base=(frame @ [0., 1., 0.]).tolist(),
        target_center_base_m=center.tolist(), initial_camera_origin_base_m=initial.tolist(),
        object_height_m=height, whole_obb_width_m=width, required_open_width_m=width+.004,
        missing_side_signs=[1], target_epoch=4, target_track_id='synthetic-measured-deficit',
        source_view_stamps_ns=[1_000_000_000])
    return gap, camera_rotation_toward(center, initial), np.array([0., 0., 1.])


@pytest.mark.parametrize('minimum_deg', [45.30, 46.98, 67.])
@pytest.mark.parametrize('height,uncertainty', [(.005, .00075), (.02, .003), (.04, .006)])
def test_geometry_expansion_uses_two_information_derived_angles_not_maximum(minimum_deg, height, uncertainty):
    gap, rotation, normal = directed_expansion_fixture(minimum_deg, uncertainty, height)
    assert not directed_missing_side_rotations((gap,), rotation, normal, uncertainty, .26)
    views = directed_missing_side_rotations((gap,), rotation, normal, uncertainty, .26,
        allow_geometry_directed_expansion=True)
    assert len(views) == 2
    angles = [metadata['camera_rotation_offset_deg'] for _, metadata in views]
    assert angles[0] == pytest.approx(minimum_deg)
    assert minimum_deg < angles[1] < 90.
    for index, (proposed, metadata) in enumerate(views):
        assert np.allclose(proposed.T @ proposed, np.eye(3), atol=1e-12)
        assert proposed[:, 0] @ normal < -1e-9
        assert metadata['predicted_new_side_gain_m'] == pytest.approx((index+1)*uncertainty+1e-6)
        assert metadata['depth_uncertainty_m'] == uncertainty
        assert metadata['proposal_tier'] == 'geometry_directed_expansion'
        assert metadata['original_maximum_rotation_deg'] == 45.
        assert metadata['expanded_maximum_rotation_deg'] == 90.
        assert metadata['maximum_rotation_deg'] == 90.
        assert metadata['minimum_required_rotation_deg'] == pytest.approx(minimum_deg)
        assert metadata['geometry_derived_angles_deg'] == angles
        assert metadata['original_family_candidate_count'] == 0
        assert metadata['expansion_reason'] == 'original_family_empty_and_measured_gap_requires_over_45_deg'
        assert metadata['expansion_opt_in'] and metadata['observation_only']
        assert metadata['measured_registration_required']
        assert not metadata['maximum_angle_probe_generated']
        assert not metadata['contact_or_motion_authorized']
        assert not metadata['global_path_optimality_claimed']
        assert metadata['target_epoch'] == gap['target_epoch']
        assert metadata['source_view_stamps_ns'] == gap['source_view_stamps_ns']
        assert missing_side_view_gain((gap,), np.asarray(gap['target_center_base_m'])-.26*proposed[:,0],
                                      uncertainty)['ok']


@pytest.mark.parametrize('yaw', [0., 73., -137.])
def test_geometry_expansion_is_frame_covariant_and_translation_independent(yaw):
    gap, rotation, normal = directed_expansion_fixture(yaw=yaw, center=(-.31, .27, .04))
    original_gap, original_rotation, _ = directed_expansion_fixture()
    views = directed_missing_side_rotations((gap,), rotation, normal, .003, .26,
        allow_geometry_directed_expansion=True)
    original = directed_missing_side_rotations((original_gap,), original_rotation, normal, .003, .26,
        allow_geometry_directed_expansion=True)
    frame = rotation_about_z(np.deg2rad(yaw))
    assert len(views) == len(original) == 2
    for (actual, audit), (zero, zero_audit) in zip(views, original):
        assert np.allclose(actual, frame @ zero, atol=1e-12)
        assert audit['camera_rotation_offset_deg'] == pytest.approx(zero_audit['camera_rotation_offset_deg'])
        assert audit['predicted_new_side_gain_m'] == pytest.approx(zero_audit['predicted_new_side_gain_m'])


def test_geometry_expansion_preserves_available_original_family_exactly():
    gap, rotation, normal = directed_expansion_fixture(30.)
    original = directed_missing_side_rotations((gap,), rotation, normal, .003, .26)
    opted_in = directed_missing_side_rotations((gap,), rotation, normal, .003, .26,
        allow_geometry_directed_expansion=True)
    assert len(original) == len(opted_in) == 2
    for (old_rotation, old_audit), (new_rotation, new_audit) in zip(original, opted_in):
        assert np.array_equal(old_rotation, new_rotation)
        assert old_audit == new_audit
        assert new_audit['camera_rotation_offset_deg'] <= 45.


def test_geometry_expansion_does_not_mix_larger_directions_when_first_tier_has_a_view():
    gap, rotation, normal = directed_expansion_fixture(46.)
    smaller = deepcopy(gap)
    smaller['jaw_axis_base'] = (rotation_about_z(np.deg2rad(35.)) @ np.array(gap['jaw_axis_base'])).tolist()
    smaller['angle_deg'] += 35.
    old = directed_missing_side_rotations((gap, smaller), rotation, normal, .003, .26)
    opted_in = directed_missing_side_rotations((gap, smaller), rotation, normal, .003, .26,
        allow_geometry_directed_expansion=True)
    assert old and len(old) == len(opted_in)
    for (old_rotation, old_audit), (new_rotation, new_audit) in zip(old, opted_in):
        assert np.array_equal(old_rotation, new_rotation)
        assert old_audit == new_audit


def test_geometry_expansion_is_covariant_when_support_frame_is_tilted():
    gap, rotation, normal = directed_expansion_fixture()
    axis = np.array([1., 2., 3.]); axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    angle = np.deg2rad(47.)
    frame = np.eye(3) + np.sin(angle)*skew + (1.-np.cos(angle))*(skew@skew)
    shifted = deepcopy(gap)
    for key in ('target_center_base_m', 'initial_camera_origin_base_m'):
        shifted[key] = (frame @ gap[key] + [.4, -.2, .3]).tolist()
    shifted['jaw_axis_base'] = (frame @ gap['jaw_axis_base']).tolist()
    old = directed_missing_side_rotations((gap,), rotation, normal, .003, .26,
        allow_geometry_directed_expansion=True)
    new = directed_missing_side_rotations((shifted,), frame@rotation, frame@normal, .003, .26,
        allow_geometry_directed_expansion=True)
    assert len(old) == len(new) == 2
    for (old_rotation, old_audit), (new_rotation, new_audit) in zip(old, new):
        assert np.allclose(new_rotation, frame@old_rotation, atol=1e-12)
        assert new_audit['camera_rotation_offset_deg'] == pytest.approx(old_audit['camera_rotation_offset_deg'])


def test_geometry_expansion_is_invariant_under_equivalent_jaw_axis_sign():
    gap, rotation, normal = directed_expansion_fixture()
    flipped = dict(gap, jaw_axis_base=(-np.array(gap['jaw_axis_base'])).tolist(), missing_side_signs=[-1])
    old = directed_missing_side_rotations((gap,), rotation, normal, .003, .26,
        allow_geometry_directed_expansion=True)
    new = directed_missing_side_rotations((flipped,), rotation, normal, .003, .26,
        allow_geometry_directed_expansion=True)
    assert len(old) == len(new) == 2
    for (old_rotation, _), (new_rotation, new_audit) in zip(old, new):
        assert np.array_equal(new_rotation, old_rotation)
        assert new_audit['missing_side_sign'] == -1


@pytest.mark.parametrize('opt_in', [None, 0, 1, 'true', [], np.bool_(True)])
def test_geometry_expansion_opt_in_must_be_literal_boolean(opt_in):
    gap, rotation, normal = directed_expansion_fixture(30.)
    assert not directed_missing_side_rotations((gap,), rotation, normal, .003, .26,
        allow_geometry_directed_expansion=opt_in)


@pytest.mark.parametrize('minimum_deg', [90.01, 93.])
def test_geometry_expansion_never_exceeds_ninety_degrees(minimum_deg):
    gap, rotation, normal = directed_expansion_fixture(minimum_deg)
    assert not directed_missing_side_rotations((gap,), rotation, normal, .003, .26,
        allow_geometry_directed_expansion=True)


def test_geometry_expansion_omits_stronger_angle_if_over_limit_instead_of_clamping():
    gap, rotation, normal = directed_expansion_fixture(86.)
    views = directed_missing_side_rotations((gap,), rotation, normal, .003, .26,
        allow_geometry_directed_expansion=True)
    assert len(views) == 1
    assert views[0][1]['camera_rotation_offset_deg'] == pytest.approx(86.)
    assert views[0][1]['geometry_derived_angles_deg'] == pytest.approx([86.])


def test_geometry_expansion_omits_unresolvable_second_gain_instead_of_max_probe():
    gap, rotation, normal = directed_expansion_fixture(46., .003, .005)
    views = directed_missing_side_rotations((gap,), rotation, normal, .003, .26,
        allow_geometry_directed_expansion=True)
    assert len(views) == 1
    assert views[0][1]['camera_rotation_offset_deg'] == pytest.approx(46.)


@pytest.mark.parametrize('fault', ['nan_uncertainty', 'negative_uncertainty', 'unresolvable',
    'side_plane_unreachable', 'wrong_sign', 'nonrigid', 'upward', 'horizontal', 'old_maximum_invalid'])
def test_geometry_expansion_cannot_bypass_input_gain_side_plane_or_downward_checks(fault):
    gap, rotation, normal = directed_expansion_fixture()
    uncertainty, maximum = .003, 45.
    if fault == 'nan_uncertainty': uncertainty = float('nan')
    elif fault == 'negative_uncertainty': uncertainty = -.001
    elif fault == 'unresolvable': uncertainty = gap['object_height_m']
    elif fault == 'side_plane_unreachable': gap['whole_obb_width_m'] = 2.*.26
    elif fault == 'wrong_sign': gap['missing_side_signs'] = [True, '1', 0]
    elif fault == 'nonrigid': rotation[0,0] += .1
    elif fault == 'upward': normal = -normal
    elif fault == 'horizontal': normal = np.array([1.,0.,0.])
    elif fault == 'old_maximum_invalid': maximum = 46.
    assert not directed_missing_side_rotations((gap,), rotation, normal, uncertainty, .26,
        maximum_rotation_deg=maximum, allow_geometry_directed_expansion=True)


def test_geometry_expansion_does_not_override_smaller_custom_first_tier_for_sub_45_gap():
    gap, rotation, normal = directed_expansion_fixture(30.)
    assert not directed_missing_side_rotations((gap,), rotation, normal, .003, .26,
        maximum_rotation_deg=20., allow_geometry_directed_expansion=True)


@pytest.mark.parametrize('maximum_directions', [1, 2])
def test_geometry_expansion_deduplicates_directions_and_obeys_total_budget(maximum_directions):
    gap, rotation, normal = directed_expansion_fixture(60.)
    gaps = []
    for angle in [0., 0., 10., -10., 20., -20.]:
        item = deepcopy(gap)
        item['jaw_axis_base'] = (rotation_about_z(np.deg2rad(angle)) @ np.array(gap['jaw_axis_base'])).tolist()
        item['angle_deg'] += angle
        gaps.append(item)
    views = directed_missing_side_rotations(gaps, rotation, normal, .003, .26,
        maximum_directions=maximum_directions, allow_geometry_directed_expansion=True)
    assert len(views) == 2*maximum_directions
    directions = {tuple(audit['jaw_axis_base']) for _, audit in views}
    assert len(directions) == maximum_directions
    assert all(45. < audit['camera_rotation_offset_deg'] <= 90. for _, audit in views)


def test_geometry_expansion_deduplicates_angles_when_uncertainty_is_zero():
    gap, rotation, normal = directed_expansion_fixture(46., 0.)
    views = directed_missing_side_rotations((gap,), rotation, normal, 0., .26,
        allow_geometry_directed_expansion=True)
    assert len(views) == 1
    assert views[0][1]['predicted_new_side_gain_m'] == pytest.approx(1e-6)


def test_common_height_deficit_targets_only_weaker_measured_side():
    _,center,surface,initial,gaps=measured_gap_fixture()
    xs=np.linspace(-.0225,.0225,25)
    points=np.array([(x,-.015,z) for x in xs for z in np.linspace(0.,.0025,21)]
                    +[(x,.015,z) for x in xs for z in np.linspace(.006,.020,21)])
    weak=type(surface)(surface.identity,points,np.zeros(len(points),dtype=np.int64),surface.view_stamps_ns)
    geometry=NS(center_base=center,axes_base=np.eye(3),size_xyz_m=np.array([.045,.030,.020]),
                support_normal_base=np.array([0.,0.,1.]))
    diagnostics=dict(tabletop_geometry=dict(plan_phase='CONTACT_EXECUTION_PLAN',
        contact_execution_gate_deferred=False,surface_evidence_rejections=list(gaps[:1])))
    measured=measured_bilateral_view_gaps(diagnostics,geometry,TabletopGeometryConfig(),weak,GRIPPER,initial)
    assert measured and measured[0]['missing_side_signs']==[-1]
    assert measured[0]['negative_measured_points']>=12 and measured[0]['positive_measured_points']>=12
    assert not missing_side_view_gain(measured,center+[0.,.2,.1],.002)['ok']


def test_mode_directed_family_keeps_old_parser_and_records_true_source(configured,monkeypatch):
    from test_remote_modes import base
    node,_,_=setup_observe_return(configured)
    _,center,_,_,gaps=measured_gap_fixture()
    camera=center+[0.,-.08,.20];rotation=camera_rotation_toward(center,camera)
    identity=node._current_stream_target_identity()
    node._observe_return_phase='observation_prefix'
    node._observe_return_view_gaps=(dict(gaps[0],initial_camera_origin_base_m=camera.tolist(),
        target_epoch=identity.epoch,target_track_id=identity.track_id),)
    node._observe_return_view_uncertainty_m=.002
    node.grasp_config={'observation_camera_target_min_distance_m':.18}
    node.observation_camera_tilt_offsets_deg=(10.,20.,30.)
    node._tool_from_camera_matrix=lambda:np.eye(4)
    matrix=np.eye(4);matrix[:3,:3]=rotation
    reference=base.make_pose_stamped('base_link',camera,base.quaternion_from_matrix(matrix),stamp=base.rospy.Time(1))
    variants=node._observation_reference_tilt_variants(reference,np.array([0.,0.,1.]))
    assert 0<len(node._observe_return_directed_views)<=4*25
    assert node.observation_camera_tilt_offsets_deg==(10.,20.,30.)
    with pytest.raises(base.CandidateContractError):
        node._parse_observation_camera_tilt_offsets_deg([45.])
    directed_rotation,metadata=node._observe_return_directed_views[0]
    transform=np.eye(4);transform[:3,:3]=directed_rotation
    pose=base.make_pose_stamped('base_link',center-.18*directed_rotation[:,0],
        base.quaternion_from_matrix(transform),stamp=base.rospy.Time(1))
    sequence=NS(pregrasp=pose,adaptive_stage_profile=NS(depth_uncertainty_m=.002),
        observation_view_audit=dict(orientation_source='frozen_snapshot_support_normal_geodesic_tilt',
            working_volume=dict(target_corner_depths_m=[.3]*8)))
    monkeypatch.setattr(node, '_camera_intrinsics', lambda: NS(fx=400.,fy=400.))
    old=dict(sequence=sequence,observation_translation_delta_m=.1)
    monkeypatch.setattr(base.RemoteGrasp6DNode,'_far_field_observation_variants',lambda self,*args,**kw:((old,),()))
    passing,rejected=node._far_field_observation_variants()
    assert passing and not rejected
    audit=passing[0]['sequence'].observation_view_audit
    assert audit['orientation_source']=='measured_missing_side_geodesic'
    assert audit['object_yaw_used'] and audit['measured_target_geometry_direction_used']
    assert not audit['contact_candidate_yaw_used']
    assert audit['observe_return_directed_missing_side']['maximum_rotation_deg']==45.
    assert passing[0]['observation_view']==audit
    runtime={'observation_variants':passing}
    node._activate_far_field_observation_variant(runtime,passing[0],1)
    assert runtime['observation_orientation_search']['object_yaw_used']
    assert runtime['observation_orientation_search']['measured_target_geometry_direction_used']
    assert not runtime['observation_orientation_search']['contact_candidate_yaw_used']
    assert node._observation_search_source_audit(passing)['object_yaw_used']
    assert not node._observation_search_source_audit((old,))['object_yaw_used']
    # Original orientation generator is unchanged outside the auxiliary prefix.
    node._observe_return_phase='initial_contact'
    assert len(node._observation_reference_tilt_variants(reference,np.array([0.,0.,1.])))<len(variants)


from alicia_grasp_modes.observe_return_visual import bounded_existing_camera_rolls, diversified_observation_checks


def test_existing_roll_coverage_preserves_only_configured_angles_and_starts_dispersed():
    configured=(0.,15.,-15.,30.,-30.,45.,-45.,60.,-60.,75.,-75.,90.,-90.,180.)
    selected=bounded_existing_camera_rolls(configured)
    assert selected[:4]==(0.,90.,-90.,180.)
    assert set(selected)==set(configured)
    assert len(selected)==len(configured)
    assert bounded_existing_camera_rolls((0.,15.,30.)) == (0.,15.,30.)
    assert not bounded_existing_camera_rolls(configured, maximum=26)


def test_directed_rolls_preserve_camera_optical_axis_and_centered_origin(configured):
    from test_remote_modes import base
    node,_,_=setup_observe_return(configured)
    _,center,_,_,gaps=measured_gap_fixture()
    camera=center+[0.,-.08,.20];rotation=camera_rotation_toward(center,camera)
    identity=node._current_stream_target_identity()
    node._observe_return_phase='observation_prefix'
    node._observe_return_view_gaps=(dict(gaps[0],initial_camera_origin_base_m=camera.tolist(),
        target_epoch=identity.epoch,target_track_id=identity.track_id),)
    node._observe_return_view_uncertainty_m=.002
    node.grasp_config={'observation_camera_target_min_distance_m':.22}
    node.observation_camera_tilt_offsets_deg=(10.,20.,30.)
    node.observation_camera_roll_offsets_deg=(0.,30.,60.,90.,-90.,180.)
    tool_camera=np.eye(4);tool_camera[:3,3]=[.03,.01,.02]
    node._tool_from_camera_matrix=lambda:tool_camera
    matrix=np.eye(4);matrix[:3,:3]=rotation
    reference=base.make_pose_stamped('base_link',camera,base.quaternion_from_matrix(matrix),stamp=base.rospy.Time(1))
    node._observation_reference_tilt_variants(reference,np.array([0.,0.,1.]))
    groups={}
    for proposed,meta in node._observe_return_directed_views:
        assert meta['camera_roll_offset_deg'] in node.observation_camera_roll_offsets_deg
        groups.setdefault(meta['camera_rotation_offset_deg'],[]).append((proposed,meta))
    assert len(groups)==2
    for variants in groups.values():
        assert len(variants)==len(node.observation_camera_roll_offsets_deg)
        optical=variants[0][0][:,0]
        for proposed,meta in variants:
            assert np.allclose(proposed[:,0],optical,atol=1e-12)
            assert np.allclose(center-.26*proposed[:,0],center-.26*optical,atol=1e-12)
            # Tool translation changes with offset camera roll; it must be recomputed,
            # never copied from another roll's endpoint/collision result.
            transform=np.eye(4);transform[:3,:3]=proposed
            pose=base.make_pose_stamped('base_link',camera,base.quaternion_from_matrix(transform),stamp=base.rospy.Time(1))
            endpoint,audit=base.centered_observation_pose_at_camera_distance(
                pose,center,tool_camera,.26,.22,.30)
            actual=base.pose_matrix(endpoint)@tool_camera
            assert np.allclose(actual[:3,3],center-.26*optical,atol=1e-12)
            assert np.allclose(actual[:3,:3],proposed,atol=1e-12)


def view_scheduler_fixture():
    return [dict(sequence=NS(observation_view_audit={'orientation_source':'measured_missing_side_geodesic'}),
        camera_roll_offset_deg=roll,camera_target_distance_m=distance,camera_tilt_offset_deg=tilt,
        preference_index=index,observation_envelope=NS(minimum_support_clearance_m=.04+distance/10))
        for index,(roll,distance,tilt) in enumerate((r,d,t) for r in (0.,30.,60.,90.,-90.,180.)
            for d in (.26,.24,.28,.22,.30) for t in (38.,45.))]


def test_unchanged_small_check_budget_sees_distance_and_roll_diversity():
    variants=view_scheduler_fixture()
    ordered=diversified_observation_checks(variants,.26)
    assert len(ordered)==len(variants) and {id(v) for v in ordered}=={id(v) for v in variants}
    assert [(v['camera_roll_offset_deg'],v['camera_target_distance_m']) for v in ordered[:4]]==[
        (0.,.26),(90.,.22),(-90.,.30),(180.,.24)]
    assert all(v['camera_tilt_offset_deg']==38. for v in ordered[:4])
    # Every retained candidate receives its own strict check; no outcome is reused.
    missing=[v for v in variants if v['camera_roll_offset_deg']!=90.]
    assert all(v['camera_roll_offset_deg']!=90. for v in diversified_observation_checks(missing,.26))


def test_modes_scheduler_requires_source_for_all_unknown_observation_families(configured):
    from test_remote_modes import base
    node,_,_=setup_observe_return(configured)
    node._observe_return_phase='observation_prefix';node.grasp_config={'observation_camera_target_nominal_distance_m':.26}
    variants=view_scheduler_fixture()
    legacy=dict(sequence=NS(observation_view_audit={}),camera_roll_offset_deg=0.,
        camera_target_distance_m=.26,camera_tilt_offset_deg=10.,preference_index=-1,
        observation_envelope=NS(minimum_support_clearance_m=.5))
    # Missing-side and old roll observations now need the same frozen current
    # tool reference, not a roll-label/contact-yaw-only preference.
    with pytest.raises(base.CandidateContractError) as error:
        node._order_observation_checks([legacy]+variants)
    assert error.value.code=='UNKNOWN_OBSERVATION_REFERENCE_INVALID'
    node._observe_return_phase='initial_contact'
    assert node._order_observation_checks([legacy]+variants)==base.RemoteGrasp6DNode._order_observation_checks([legacy]+variants)


@pytest.mark.parametrize('legacy_tuple', [False, True])
def test_measured_missing_side_audit_roundtrip_preserves_promotion_authority(
        configured, tmp_path, legacy_tuple):
    # Exercise the real atomic audit writer and disk/SHA/object validation, with
    # real measured gaps rather than a JSON-shaped hand-written replacement.
    import hashlib
    from test_remote_grasp6d_streaming import (cached_preview_audit_report,
        promotion_node, promotion_plan, RecordingPublisher, PromotionDecision,
        remote_node)

    modes_node, _, _ = setup_observe_return(configured)
    _, _, _, _, gaps = measured_gap_fixture()
    assert gaps
    modes_node._observe_return_view_gaps = gaps
    modes_node._observe_return_phase = 'observation_prefix'
    modes_node._observe_return_trigger = 'BILATERAL_SURFACE_EVIDENCE_MISSING'
    preview_report = cached_preview_audit_report('plan-B')
    modes_node._observe_return_audit(preview_report,
        NS(snapshot=NS(stamp_ns=20_000_000_000)))
    # Export must not mutate the retained evidence. This injected old type
    # reproduces the regression even though both JSON byte streams/hash match.
    assert modes_node._observe_return_view_gaps is gaps
    if legacy_tuple:
        preview_report['observe_return']['measured_missing_sides'] = deepcopy(gaps)

    node = promotion_node()
    node.gate_audit_enabled = True
    node.gate_audit_output_path = str(tmp_path / 'preview.json')
    node.mujoco_audit_output_path = str(tmp_path / 'mujoco.json')
    node.gate_audit_pub = RecordingPublisher()
    node._execution_promotion_audit_ready = (
        remote_node.RemoteGrasp6DNode._execution_promotion_audit_ready
        .__get__(node, remote_node.RemoteGrasp6DNode))
    node._active_gate_audit_report = preview_report
    plan = promotion_plan('plan-B')
    ready, reason = node._write_cached_preview_execution_audit(plan,
        PromotionDecision(True, 'PROMOTE_REPLAN', 'explicit execution replan requested'))

    payload = Path(node.gate_audit_output_path + '.execution').read_bytes()
    disk_report = json.loads(payload.decode('utf-8'))
    retained_report = node._active_execution_audit_report
    reference = node._latest_execution_audit_reference
    assert reference['atomic_committed']
    assert hashlib.sha256(payload).hexdigest() == reference['report_sha256']
    assert disk_report['observe_return']['phase'] == 'observation_prefix'
    assert (disk_report == retained_report) is (not legacy_tuple)
    assert ready is (not legacy_tuple)
    assert node._execution_promotion_audit_ready(plan) is (not legacy_tuple)
    assert reason == ''


def recorded_zero_safe_request(configured):
    from dataclasses import replace
    node, _, clock = setup_observe_return(configured)
    fixture_dir = Path(__file__).parent / 'fixtures'
    data = json.loads((fixture_dir / 'generation13_initial_view_gap.json').read_text())
    r = data['geometry']
    geometry = NS(center_base=np.asarray(r['obb_center_base_m']),
        axes_base=np.asarray(r['R_base_obb']), size_xyz_m=np.asarray(r['obb_size_xyz_m']),
        support_normal_base=np.asarray(r['support_normal_base']))
    with np.load(fixture_dir / 'generation13_initial_view_gap_surface.npz') as saved:
        points = saved['points_base'].copy()
    report = data['audit']
    report.update(generation=node._stream_generation, target_epoch=node.target_instance_epoch)
    report['pipeline_funnel'] = dict(stage_counts=dict(locally_valid=data['locally_valid']))
    node._active_gate_audit_report = report
    stamp = report['snapshot_stamp_ns']
    identity = node._current_stream_target_identity()
    view = SurfaceView(identity, stamp, points, geometry.support_normal_base,
                       float(r['support_offset_m']), 12)
    node._near_field_fused_surface = fused_surface_from_view(view)
    node._near_field_reference_view = view
    node.gripper_geometry = GRIPPER
    node.tabletop_geometry_config = replace(TabletopGeometryConfig(),
        opening_fit_clearance_each_side_m=data['opening_fit_clearance_each_side_m'])
    clock[0] = stamp + 1_000_000_000
    prepared = NS(ticket=NS(**{k:report[k] for k in ('request_id','generation','target_epoch')}),
        snapshot=NS(stamp_ns=stamp, target_identity=identity), geometry=geometry,
        remote_diagnostics=data['diagnostics'], remote_failure_code='',
        pose_estimator=NS(T_base_camera_link=np.asarray(data['T_base_camera_link'])))
    return node, prepared


@pytest.mark.parametrize('fault', ['', 'request', 'generation', 'epoch', 'stamp', 'winner',
    'collision_only', 'oversized', 'timeout', 'terminated', 'unchecked', 'checked',
    'reachable', 'local_pass', 'incomplete_local', 'bool_count', 'server', 'phase'])
def test_zero_safe_independent_gap_requires_complete_bound_request(configured, fault):
    node, p = recorded_zero_safe_request(configured)
    report = node._active_gate_audit_report
    search = report['moveit_selection']
    local = report['pipeline_funnel']['stage_counts']['locally_valid']
    code = 'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE'
    if fault in ('request', 'generation', 'epoch', 'stamp'):
        key = dict(request='request_id', generation='generation', epoch='target_epoch',
                   stamp='snapshot_stamp_ns')[fault]
        report[key] += 1
    elif fault == 'winner': report['selected'] = {'plan_id':'winner'}
    elif fault == 'collision_only':
        p.remote_diagnostics['tabletop_geometry']['surface_evidence_rejections'] = []
    elif fault == 'oversized': p.geometry.size_xyz_m *= 2.
    elif fault == 'timeout': code = 'NEAR_FIELD_DIRECT_TIMEOUT'
    elif fault == 'terminated': search['terminated_early'] = True
    elif fault == 'unchecked': search['shortlist_count'] = 1
    elif fault == 'checked': search.update(checked_count=1, shortlist_count=1)
    elif fault == 'reachable': search['reachable_count'] = 1
    elif fault == 'local_pass': local['passed'] = 1
    elif fault == 'incomplete_local': local['rejected'] -= 1
    elif fault == 'bool_count': search['checked_count'] = False
    elif fault == 'server': p.remote_failure_code = 'WSL_UNAVAILABLE'
    elif fault == 'phase': node._observe_return_phase = 'observing_contact'
    opportunity = node._observe_return_mixed_visual_failure(p, code)
    assert bool(opportunity) is (not fault)
    if opportunity:
        assert opportunity['contact_search_code'] == code
        assert opportunity['checked_count'] == 0
        assert opportunity['missing_surface_direction']['required_open_width_m'] == pytest.approx(.048431855979094836)
        assert not opportunity['contact_or_motion_authorized']


def test_recorded_zero_safe_sweep_summary_preserves_measured_observation_opportunity(configured):
    from alicia_grasp_modes.observe_return_session import structured_view_deficit
    node, p = recorded_zero_safe_request(configured)
    assert not structured_view_deficit(p.remote_diagnostics)
    assert p.remote_diagnostics['tabletop_geometry']['failure_code'] == 'GRIPPER_SWEEP_COLLISION'
    source = node._active_multiview_surface()[0]
    node._publish_direct_near_field_terminal(p, 'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE', 'local proposals rejected')
    assert node._observe_return_phase == 'observation_prefix'
    assert node._observe_return_saved[0] is source
    assert len(node._observe_return_view_gaps) == 1
    gap = node._observe_return_view_gaps[0]
    assert gap['missing_side_signs'] == [-1]
    assert gap['negative_measured_points'] == 27
    assert gap['positive_measured_points'] == 64
    assert gap['negative_measured_height_m'] == pytest.approx(.003791959253727258)
    assert gap['positive_measured_height_m'] == pytest.approx(.00809522953277061)
    assert not gap['contact_or_motion_authorized']
    assert node._observe_return_trigger_evidence['contact_search_code'] == 'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE'
    assert not node.preview_rich_plan_pub.messages[-1].valid
    assert node.status_pub.messages[-1].data.startswith('OBSERVE_RETURN_VIEW_REQUIRED')


def test_zero_safe_opportunity_is_rechecked_after_local_evidence_changes(configured):
    node, p = recorded_zero_safe_request(configured)
    opportunity = node._observe_return_mixed_visual_failure(p, 'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE')
    node._active_gate_audit_report['pipeline_funnel']['stage_counts']['locally_valid']['passed'] = 1
    assert not node._observe_return_fallback(p, visual_opportunity=opportunity)
    assert node._observe_return_phase == 'initial_contact'
