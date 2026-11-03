"""Independent RGB local scene proofs never create a grasp target or action."""
from dataclasses import replace, FrozenInstanceError
import hashlib
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest
import rospy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from alicia_grasp_modes.rgb_noncontact_scene import (
    build_rgb_noncontact_scene, validate_rgb_noncontact_trajectory,
    revalidate_rgb_noncontact_scene, retain_observed_obstacles, RgbNoncontactError)
from alicia_grasp_modes.rgb_observation_volume import (
    build_rgb_observation_volume, SupportPlaneProvenance)
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry


def volume(stamp=10**9, *, camera_x=0., max_height=.12, plane=(0., 0., 1., 0.),bbox=(70,50,20,20)):
    transform=np.array([[0.,-1.,0.,camera_x],[0.,0.,1.,0.],[-1.,0.,0.,.6],[0.,0.,0.,1.]])
    return build_rgb_observation_volume(bbox_xywh=bbox, image_shape=(120,160),
        camera_matrix=((100.,0.,80.),(0.,100.,60.),(0.,0.,1.)), T_base_camera=transform,
        support_plane_base=plane, support_provenance=SupportPlaneProvenance(
            source_stamp_ns=stamp,support_plane_base=plane,source_record_id='source',
            inlier_count=200,inlier_ratio=.9,fit_rmse_m=.001,spatial_quadrants=4),
        foreground_max_m=max_height, source_stamp_ns=stamp,working_depth_range_m=(.07,.8))


def scene(stamp=10**9, *, depth=None, interest=None, reserve=0., **kwargs):
    data=np.full((120,160),.6,np.float32) if depth is None else depth
    v=volume(stamp) if interest is None else interest
    values=dict(interest_volume=v,depth_image=data,depth_scale_m=1.,source_stamp_ns=stamp,
        source_depth_sha256=hashlib.sha256(data.tobytes()).hexdigest(),depth_uncertainty_m=.002,
        valid_depth_range_m=(.03,2.),source_record_id='source',containment_reserve_m=reserve)
    values.update(kwargs)
    return build_rgb_noncontact_scene(**values)


def fk():
    return SerialUrdfFk('<robot name="test"><joint name="J1" type="revolute">'
        '<parent link="base_link"/><child link="one"/><origin xyz="1 0 0"/>'
        '<axis xyz="0 1 0"/><limit lower="-3" upper="3"/></joint>'
        '<joint name="tip" type="fixed"><parent link="one"/><child link="tool0"/>'
        '<origin xyz="0 0 .4"/></joint></robot>',('J1',))


def gripper():
    return GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)


def trajectory(start=0., end=.04):
    plan=RobotTrajectory();plan.joint_trajectory.joint_names=['J1']
    for t,q in ((0.,start),(1.,end)):
        p=JointTrajectoryPoint();p.positions=[q];p.velocities=[0.];p.accelerations=[0.]
        p.time_from_start=rospy.Duration.from_sec(t);plan.joint_trajectory.points.append(p)
    return plan


def check(frozen=None,plan=None,**kwargs):
    p=trajectory() if plan is None else plan
    hold=NS(positions=p.joint_trajectory.points[0].positions,velocities=[0.],accelerations=[0.])
    values=dict(joint_error_bounds_rad=[.12076699039394281],max_seconds=5.)
    values.update(kwargs)
    return validate_rgb_noncontact_trajectory(p,scene() if frozen is None else frozen,
        fk(),gripper(),.05,hold,**values)


def test_same_source_depth_scene_and_noncontact_path_prove_original_tracking_band():
    frozen=scene();report=check(frozen)
    assert report['ok'] and report['certified_minimum_clearance_m'] >= .003
    assert report['following_support']['includes_controller_bridge']
    assert report['following_support']['local_depth_and_rgb_keepout_tracking_uncertainty_checked']
    assert report['following_support']['joint_error_bounds_rad']==[.12076699039394281]
    assert report['motion_authorized'] is False
    assert report['complete_environment_certified'] is False
    assert report['camera_housing_collision_certified'] is False
    assert not hasattr(frozen,'center') and not hasattr(frozen,'size') and not hasattr(frozen,'track_id')
    assert 'additional_keepout' in report['rgb_interest_role']
    assert report['certified_swept_cad_bounds_base_m']


def test_every_valid_pixel_is_enclosed_by_a_conservative_full_cell_beam():
    image=np.zeros((120,160),np.float32);image[50,80]=.41;image[51,81]=.61
    frozen=scene(depth=image,pixel_block_size=8)
    assert frozen.valid_depth_pixel_count==2 and len(frozen.depth_bounds)==1
    lo,hi=frozen.depth_bounds[0]
    for u,v,d in ((80,50,.41),(81,51,.61)):
        for uu in (u-.5,u+.5):
            for vv in (v-.5,v+.5):
                for dd in (d-.002,d+.002):
                    point=np.array([(uu-80)*dd/100.,-(vv-60)*dd/100.,.6-dd])
                    assert np.all(point>=lo-1e-7) and np.all(point<=hi+1e-7)
    assert frozen.unavailable_depth_pixel_count==image.size-2


def _original_scalar_depth_beams(depth, valid, matrix, transform, block_size, uncertainty, reserve):
    """Frozen pre-batching oracle: preserve original source-scene digest bits."""
    boxes, block_ids = [], []
    height, width = depth.shape
    for y in range(0, height, block_size):
        for x in range(0, width, block_size):
            yy, xx = np.nonzero(valid[y:y+block_size, x:x+block_size])
            if not len(xx):
                continue
            xx, yy = xx+x, yy+y
            measured = depth[yy, xx]
            lo = max(0., float(np.min(measured))-uncertainty)
            hi = float(np.max(measured))+uncertainty
            pixels = np.array([[xx.min()-.5, yy.min()-.5, 1.],
                [xx.max()+.5, yy.min()-.5, 1.], [xx.max()+.5, yy.max()+.5, 1.],
                [xx.min()-.5, yy.max()+.5, 1.]])
            optical = np.linalg.solve(matrix, pixels.T).T
            rays = np.c_[optical[:, 2], -optical[:, 0], -optical[:, 1]]
            corners = np.vstack((rays*lo, rays*hi)) @ transform[:3, :3].T+transform[:3, 3]
            boxes.append([corners.min(axis=0)-reserve, corners.max(axis=0)+reserve])
            block_ids.append((y//block_size)*((width+block_size-1)//block_size)+x//block_size)
    return np.asarray(boxes, dtype=float), block_ids


@pytest.mark.parametrize('shape,block_size', [((17,23),1),((17,23),8),((17,23),32),
    ((65,67),1),((9,2050),8)])
def test_batched_depth_beams_are_bit_identical_with_holes_edges_and_rotated_camera(shape, block_size):
    from alicia_grasp_modes.rgb_noncontact_scene import _depth_beam_bounds
    rng = np.random.RandomState(134)
    depth = rng.uniform(.03,2.,shape)
    valid = rng.uniform(size=shape) > .32
    # Invalid values include values that would expand the boxes if any hole
    # were used. A partially populated final block must keep its true pixels.
    depth[~valid] = np.resize([np.nan,np.inf,-np.inf,-10.,100.], np.count_nonzero(~valid))
    valid[:min(8,shape[0]),:min(8,shape[1])] = False
    valid[-1,-1] = True; depth[-1,-1] = .0301
    matrix = np.array([[513.25,.37,shape[1]/2.-.31],[0.,495.7,shape[0]/2.+.17],[0.,0.,1.]])
    for angle in (.0000000005,.39,1.8):
        c,s = np.cos(angle),np.sin(angle)
        transform = np.array([[c,0.,s,.127],[0.,1.,0.,-.281],[-s,0.,c,.467],[0.,0.,0.,1.]])
        actual, ids = _depth_beam_bounds(depth,valid,matrix,transform,block_size,.032,.003)
        expected, expected_ids = _original_scalar_depth_beams(
            depth,valid,matrix,transform,block_size,.032,.003)
        assert ids == expected_ids
        assert actual.tobytes() == expected.tobytes()


@pytest.mark.parametrize('dtype,scale', [('uint16',.0001),('float32',1.),('float64',1.)])
def test_batched_scene_preserves_entire_binding_and_continuous_path_result(monkeypatch,dtype,scale):
    import alicia_grasp_modes.rgb_noncontact_scene as module
    rng = np.random.RandomState(135)
    depth = ((.6+rng.uniform(-.001,.001,(120,160)))/scale).astype(dtype)
    depth[30:36,40:49] = 0
    optimized = scene(depth=depth,depth_scale_m=scale,pixel_block_size=7,reserve=.003)
    with monkeypatch.context() as patch:
        patch.setattr(module,'_depth_beam_bounds',_original_scalar_depth_beams)
        original = scene(depth=depth,depth_scale_m=scale,pixel_block_size=7,reserve=.003)
    assert optimized.binding_json == original.binding_json
    assert optimized.digest == original.digest
    assert optimized.scope() == original.scope()
    a,b = check(optimized),check(original)
    for key in ('certified_minimum_clearance_m','box_checks','interval_checks',
                'following_support','certified_swept_cad_bounds_base_m'):
        assert a[key] == b[key]


def test_depth_beam_solver_batches_are_bounded_and_do_not_repeat_per_pixel(monkeypatch):
    from alicia_grasp_modes.rgb_noncontact_scene import _depth_beam_bounds
    original = np.linalg.solve
    shapes = []
    def solve(matrix, rhs):
        shapes.append(rhs.shape)
        return original(matrix,rhs)
    monkeypatch.setattr(np.linalg,'solve',solve)
    depth = np.full((65,67),.6)
    actual, ids = _depth_beam_bounds(depth,np.ones_like(depth,dtype=bool),
        np.eye(3),np.eye(4),1,.002,0.)
    assert shapes == [(4096,3,4),(259,3,4)]
    assert actual.shape == (4355,2,3) and ids == list(range(4355))
    assert np.all(actual[:,0] <= actual[:,1])


def test_scene_is_immutable_and_source_bindings_cannot_be_replaced():
    frozen=scene()
    with pytest.raises(ValueError):frozen.depth_bounds[0,0,0]=10.
    with pytest.raises(FrozenInstanceError):frozen.offset=3.
    with pytest.raises(RgbNoncontactError,match='BINDING_INVALID'):
        replace(frozen,source_ns=frozen.source_ns+1)
    with pytest.raises(RgbNoncontactError,match='BINDING_INVALID'):
        replace(frozen,valid_depth_range_m=(1.,2.))


@pytest.mark.parametrize('kwargs',[
    dict(source_stamp_ns=100),dict(source_depth_sha256='0'*64),
    dict(depth_scale_m=0.),dict(depth_scale_m=True),dict(depth_uncertainty_m=0.),
    dict(depth_uncertainty_m=np.nan),dict(source_record_id=''),dict(pixel_block_size=True),
    dict(pixel_block_size=33),dict(containment_reserve_m=-.01),dict(containment_reserve_m=True),
    dict(valid_depth_range_m=(1.,1.))])
def test_invalid_or_unbound_source_rejects(kwargs):
    with pytest.raises(ValueError):scene(**kwargs)


def test_rgb_interest_keepout_rejects_overlap_without_claiming_measured_collision():
    frozen=scene(interest=volume(camera_x=1.,max_height=.5))
    with pytest.raises(RgbNoncontactError,match='LOCAL_KEEPOUT_OVERLAP') as error:
        check(frozen)
    assert error.value.audit['failure_kind']=='rgb_interest_keepout'
    assert error.value.audit['physical_collision_proven'] is False


def test_measured_surface_outside_rgb_interest_is_still_an_obstacle():
    # Move the interest sideways in its source while source depth spans the robot.
    v=volume(camera_x=1.)
    depth=np.full((120,160),.3,np.float32)
    frozen=scene(depth=depth,interest=v)
    with pytest.raises(RgbNoncontactError,match='LOCAL_KEEPOUT_OVERLAP') as error:
        check(frozen)
    assert error.value.audit['failure_kind']=='source_depth_beam'


def test_following_contract_cannot_be_omitted_to_skip_uncertainty():
    with pytest.raises(RgbNoncontactError,match='FOLLOWING_CONTRACT_REQUIRED'):
        check(joint_error_bounds_rad=None)


def test_continuous_path_cannot_skip_midpoint_keepout_with_clear_endpoints():
    frozen=scene(interest=volume(camera_x=1.,max_height=.5))
    with pytest.raises(RgbNoncontactError):
        check(frozen,trajectory(-.75,.75),joint_error_bounds_rad=[.04])


def test_exhausted_proof_budget_never_returns_success():
    with pytest.raises(RgbNoncontactError):check(max_checks=1)


def test_refresh_identical_new_frame_consumes_real_plane_margin_without_renewing_stamp():
    old=scene(reserve=.003);report=check(old)
    fresh=scene(stamp=10**9+100_000_000)
    receipt=revalidate_rgb_noncontact_scene(old,fresh,report)
    assert receipt['ok'] and receipt['source_timestamp_renewed'] is False
    assert receipt['fresh_source_stamp_ns']==fresh.source_ns
    assert receipt['contained_depth_boxes']==len(fresh.depth_bounds)
    assert receipt['motion_authorized'] is False


def test_refresh_rejects_reused_frame_and_missing_path_proof():
    old=scene();report=check(old)
    with pytest.raises(RgbNoncontactError,match='FRESH_BINDING_INVALID'):
        revalidate_rgb_noncontact_scene(old,old,report)
    with pytest.raises(RgbNoncontactError,match='SWEPT_ENCLOSURES_MISSING'):
        revalidate_rgb_noncontact_scene(old,scene(stamp=2*10**9),dict(report,certified_swept_cad_bounds_base_m=[]))


def test_refresh_cannot_shift_support_up_beyond_certified_clearance():
    old=scene();report=check(old)
    fresh=scene(stamp=2*10**9,interest=volume(2*10**9,plane=(0.,0.,1.,-.30)))
    with pytest.raises(RgbNoncontactError,match='FRESH_SUPPORT_MARGIN_EXCEEDED'):
        revalidate_rgb_noncontact_scene(old,fresh,report)


def test_refresh_depth_changes_far_from_proved_path_need_no_scene_lifetime_extension():
    old=scene();report=check(old)
    fresh=scene(stamp=2*10**9,depth=np.full((120,160),.65,np.float32))
    receipt=revalidate_rgb_noncontact_scene(old,fresh,report)
    assert receipt['ok'] and receipt['independently_clear_new_depth_boxes'] > 0


def test_refresh_new_holes_are_accounted_and_cannot_be_deleted_as_free_space():
    old=scene();report=check(old)
    depth=np.full((120,160),.6,np.float32);depth[55:57,75:77]=0.
    fresh=scene(stamp=2*10**9,depth=depth)
    receipt=revalidate_rgb_noncontact_scene(old,fresh,report)
    assert receipt['newly_unavailable_depth_pixels']==4
    assert receipt['unknown_space_assumed_free'] is False


def test_refresh_new_hole_that_intersects_proved_path_is_rejected():
    # Synthetic proof enclosure exercises the unknown-beam rejection independent
    # of whether a real robot could occupy this test region.
    old=scene();report=check(old)
    report=dict(report,certified_swept_cad_bounds_base_m=[[[-.01,-.01,.25],[.01,.01,.35]]])
    depth=np.full((120,160),.6,np.float32);depth[59:61,79:81]=0.
    fresh=scene(stamp=2*10**9,depth=depth)
    with pytest.raises(RgbNoncontactError,match='NEW_DEPTH_HOLE_INTERSECTS_SWEPT_REGION'):
        revalidate_rgb_noncontact_scene(old,fresh,report)


def test_refresh_new_surface_inside_certified_swept_region_is_rejected():
    old=scene();report=check(old)
    report=dict(report,certified_swept_cad_bounds_base_m=[[[-.01,-.01,.25],[.01,.01,.35]]])
    depth=np.full((120,160),.6,np.float32);depth[59:61,79:81]=.3
    fresh=scene(stamp=2*10**9,depth=depth)
    with pytest.raises(RgbNoncontactError,match='FRESH_DEPTH_NOT_COVERED'):
        revalidate_rgb_noncontact_scene(old,fresh,report)


def test_retained_scene_keeps_baseline_previous_and_current_only():
    first=scene();second=scene(stamp=2*10**9)
    bundle=retain_observed_obstacles(first,second)
    third=scene(stamp=3*10**9);fourth=scene(stamp=4*10**9)
    bundle=retain_observed_obstacles(retain_observed_obstacles(bundle,third),fourth)
    assert [s.source_ns for s in bundle.scenes]==[10**9,3*10**9,4*10**9]
    report=check(bundle)
    assert report['source_stamp_ns']==fourth.source_ns
    assert report['scene_sha256']==bundle.digest and report['retained_frame_count']==3
    assert report['whole_depth_range_unknown_beams_created'] is False
    assert report['certifies_new_unobserved_dynamic_obstacles'] is False


def test_retention_cannot_erase_previous_measured_obstacle_when_depth_is_lost():
    old=scene(depth=np.full((120,160),.3,np.float32),interest=volume(camera_x=1.))
    fresh=scene(stamp=2*10**9,interest=volume(2*10**9,camera_x=1.))
    check(fresh)
    bundle=retain_observed_obstacles(old,fresh)
    with pytest.raises(RgbNoncontactError,match='LOCAL_KEEPOUT_OVERLAP'):
        check(bundle)


def test_retention_preserves_each_support_plane_without_averaging():
    old=scene(interest=volume(plane=(0.,0.,1.,-.35)))
    fresh=scene(stamp=2*10**9)
    check(fresh)
    with pytest.raises(RgbNoncontactError,match='FOLLOWING_SUPPORT_INVALID'):
        check(retain_observed_obstacles(old,fresh))


def test_retention_requires_actual_new_source_and_matching_depth_contract():
    old=scene()
    with pytest.raises(RgbNoncontactError,match='RETAINED_SOURCE_MISMATCH'):
        retain_observed_obstacles(old,old)
    with pytest.raises(RgbNoncontactError,match='RETAINED_SOURCE_MISMATCH'):
        retain_observed_obstacles(old,scene(stamp=2*10**9,valid_depth_range_m=(.1,1.)))


def test_same_frame_members_keep_independent_keepouts_and_source_bindings():
    main=volume();extra=volume(bbox=(30,30,10,10))
    frozen=scene(interest=main,additional_interest_volumes=(extra,))
    assert len(frozen.interest_bounds_all)==2
    assert np.array_equal(frozen.interest_bounds_all[1],extra.bounds_base_m)
    assert check(frozen)['rgb_interest_keepout_count']==2
    with pytest.raises(ValueError):frozen.additional_interest_bounds[0,0,0]=1.
    with pytest.raises(RgbNoncontactError,match='BINDING_INVALID'):
        replace(frozen,additional_interest_bounds=np.empty((0,2,3)))


@pytest.mark.parametrize('extra',[
    volume(2*10**9),volume(camera_x=.001),volume(max_height=.11),
    volume(plane=(0.,0.,1.,-.001)),None])
def test_member_from_another_stamp_pose_plane_or_assumption_is_rejected(extra):
    with pytest.raises(RgbNoncontactError,match='INTEREST_SET_SOURCE_MISMATCH'):
        scene(additional_interest_volumes=(extra,))


def test_more_than_four_same_frame_members_are_rejected_without_truncation():
    with pytest.raises(RgbNoncontactError,match='INTEREST_SET_BUDGET_INVALID'):
        scene(additional_interest_volumes=(volume(),)*4)


def test_additional_member_collision_cannot_hide_behind_first_clear_member():
    main=volume(camera_x=1.,max_height=.5,bbox=(150,1,4,4))
    extra=volume(camera_x=1.,max_height=.5)
    check(scene(interest=main),joint_error_bounds_rad=[.01])
    with pytest.raises(RgbNoncontactError,match='LOCAL_KEEPOUT_OVERLAP') as error:
        check(scene(interest=main,additional_interest_volumes=(extra,)),joint_error_bounds_rad=[.01])
    assert error.value.audit['failure_kind']=='rgb_interest_keepout'


def test_temporal_retention_preserves_all_members_from_each_measured_frame():
    old=scene(additional_interest_volumes=(volume(bbox=(30,30,10,10)),))
    fresh=scene(stamp=2*10**9,additional_interest_volumes=(volume(2*10**9,bbox=(32,30,10,10)),))
    bundle=retain_observed_obstacles(old,fresh)
    assert check(bundle)['rgb_interest_keepout_count']==4


def test_batched_nominal_collision_matches_original_scalar_sat_on_rotated_boxes():
    from alicia_grasp_modes.rgb_noncontact_scene import _JointBoxObstacles
    from alicia_flexible_grasp.grasp.gripper_geometry import _stage_boxes,_obb_overlap
    rng=np.random.RandomState(123)
    checker=_JointBoxObstacles(fk(),gripper(),.05,scene())
    for _ in range(80):
        q=np.array([rng.uniform(-.7,.7)])
        pose=fk()(q);parts=_stage_boxes(pose,gripper(),.05,'y','z')
        _,centre,size=parts[rng.randint(3)]
        other_centre=centre+rng.uniform(-.12,.12,3)
        other_size=rng.uniform(.001,.10,3)
        checker.obstacles=np.array([[other_centre-.5*other_size,other_centre+.5*other_size]])
        expected=any(_obb_overlap(c,pose[:3,:3],s,other_centre,np.eye(3),other_size)
                     for _,c,s in parts)
        try:
            checker.check(q,q,budget=[1000],clock=time.monotonic,started=time.monotonic(),max_seconds=2.)
            actual=False
        except RgbNoncontactError as error:
            assert str(error)=='RGB_NONCONTACT_LOCAL_KEEPOUT_OVERLAP'
            actual=True
        assert actual==expected


def test_parent_certificates_prune_only_already_separated_obstacles():
    from alicia_grasp_modes.rgb_noncontact_scene import _JointBoxObstacles
    class Counted(_JointBoxObstacles):
        examined=0
        full_scan=False
        def unresolved_pairs(self,bounds,inherited=None):
            if self.full_scan:inherited=None
            self.examined+=(len(self.obstacles)*len(bounds) if inherited is None
                            else sum(len(indices) for _,indices in inherited))
            return super().unresolved_pairs(bounds,inherited)
    optimized=Counted(fk(),gripper(),.05,scene())
    reference=Counted(fk(),gripper(),.05,scene());reference.full_scan=True
    # Joint1 rotates about Y. This box remains just outside the palm's Y face;
    # conservative Taylor remainder requires subdivision before certification.
    near=np.array([[.94,.0788,.29],[.98,.0808,.32]])
    far=np.array([[[10+i*.01,10.,10.],[10.001+i*.01,10.001,10.001]] for i in range(1000)])
    optimized.obstacles=reference.obstacles=np.concatenate((near[None],far))
    budgets=[]
    for checker in (optimized,reference):
        budget=[1000]
        assert checker.check(np.array([-.2]),np.array([.2]),budget=budget,
            clock=time.monotonic,started=time.monotonic(),max_seconds=2.)
        budgets.append(budget[0])
    assert budgets[0]==budgets[1]<997
    assert optimized.examined<reference.examined/2


def test_repeated_stationary_boxes_reuse_only_this_scene_contract_certificate():
    report=check(plan=trajectory(0.,0.))
    assert report['ok'] and report['repeated_joint_box_certificate_hits']>=2
    assert report['following_support']['includes_controller_bridge']
    with pytest.raises(RgbNoncontactError,match='LOCAL_KEEPOUT_OVERLAP'):
        check(scene(interest=volume(camera_x=1.,max_height=.5)),plan=trajectory(0.,0.))


@pytest.mark.parametrize('angle',[0.,5e-11,5e-10,2e-9])
@pytest.mark.parametrize('gap',[0.,1e-4])
def test_nearly_parallel_sat_axes_cannot_turn_contact_into_clearance(angle,gap):
    from alicia_grasp_modes.rgb_noncontact_scene import _JointBoxObstacles
    from alicia_flexible_grasp.grasp.gripper_geometry import _stage_boxes,_box_corners,_obb_overlap
    checker=_JointBoxObstacles(fk(),gripper(),.05,scene())
    q=np.array([angle]);pose=fk()(q)
    _,center,size=_stage_boxes(pose,gripper(),.05,'y','z')[-1]
    # Isolate the palm so a large obstacle face can contain its extremal
    # corner without accidentally testing a separate finger collision.
    checker.local=checker.local[-8:];checker.hessian=checker.hessian[-8:]
    checker.part_names=('palm',)
    corners=_box_corners(center,pose[:3,:3],size)
    other_size=np.array([.01,.20,.20]);other=center.copy()
    other[0]=corners[:,0].max()+other_size[0]/2.+gap
    checker.obstacles=np.array([[other-other_size/2.,other+other_size/2.]])
    expected=_obb_overlap(center,pose[:3,:3],size,other,np.eye(3),other_size)
    if expected:
        with pytest.raises(RgbNoncontactError,match='LOCAL_KEEPOUT_OVERLAP'):
            checker.check(q,q,budget=[1000],clock=time.monotonic,started=time.monotonic(),max_seconds=2.)
    else:
        assert checker.check(q,q,budget=[1000],clock=time.monotonic,started=time.monotonic(),max_seconds=2.)


def test_separated_finger_pair_cannot_clear_an_obstacle_from_palm_pair():
    from alicia_grasp_modes.rgb_noncontact_scene import _JointBoxObstacles
    from alicia_flexible_grasp.grasp.gripper_geometry import _stage_boxes
    checker=_JointBoxObstacles(fk(),gripper(),.05,scene())
    _,center,_=_stage_boxes(fk()([0.]),gripper(),.05,'y','z')[-1]
    checker.obstacles=np.array([[center-.002,center+.002]])
    with pytest.raises(RgbNoncontactError,match='LOCAL_KEEPOUT_OVERLAP') as error:
        checker.check(np.array([0.]),np.array([0.]),budget=[1000],clock=time.monotonic,
                      started=time.monotonic(),max_seconds=2.)
    assert error.value.audit['gripper_part']=='palm'
