from copy import deepcopy
from types import SimpleNamespace as NS
import math
import json
import hashlib
from pathlib import Path

import numpy as np
import pytest
import rospy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from alicia_flexible_grasp.robot.observation_endpoint_compensation import (
    ObservationEndpointCompensation, observation_endpoint_view,
    validate_observation_correction_path_bounds)
from alicia_flexible_grasp.robot.pregrasp_compensation import pose_matrix
from alicia_flexible_grasp.grasp.rich_plan_integrity import pose_values
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SDK_QUANTUM_RAD as Q
from test_pregrasp_feedforward import case
from test_endpoint_correction import sample


def view_inputs(goal):
    tool = pose_matrix(pose_values(goal))
    scene = NS(center=tool[:3,3]+tool[:3,:3]@np.array([.25,0.,0.]),
        rotation=tool[:3,:3],size=np.array([.02,.02,.02]),source_ns=9_000_000_000)
    configuration = dict(camera=dict(width=640,height=480,fx=400.,fy=400.,cx=320.,cy=240.),
        tool_from_camera=np.eye(4).tolist(),limits=dict(roi_margin_px=36.,minimum_depth_m=.07,maximum_depth_m=.5))
    return scene,configuration,np.ones((480,640),dtype=bool)


def test_actual_existing_feedforward_is_reused_once_without_double_bias():
    base,nominal,sdk,actual = case()
    scene,config,valid = view_inputs(base.goal)
    correction = ObservationEndpointCompensation(base.contract.fk,sample(base.contract.fk,sdk,actual),
        base.goal,scene,config,valid)
    step,report = correction.propose(nominal)
    assert (np.array(step.target_counts)-step.baseline_counts).tolist() == [-5,16,16,2,-5,5]
    assert report['sdk_minus_encoder_rad'] == pytest.approx(sdk-actual)
    assert report['predicted_view']['actual_arrival_measurement_required']
    correction.committed(step,12.)
    response = sample(base.contract.fk,step.positions,np.array(step.positions)-(sdk-actual),14.)
    result = correction.evaluate(response)
    assert result['hardware_response_verified']
    assert result['view_audit']['source_freshness_authority'] is False
    with pytest.raises(ValueError,match='ALREADY_PROPOSED'):
        correction.propose(nominal)


@pytest.mark.parametrize('fault',['depth','rgb_hole','mask_shape','mount'])
def test_non_margin_failures_do_not_become_correction_opportunities(fault):
    base,_,sdk,actual = case();scene,config,valid = view_inputs(base.goal)
    if fault=='depth': scene.center += pose_matrix(pose_values(base.goal))[:3,:3]@np.array([1.,0.,0.])
    if fault=='rgb_hole': valid[240,320]=False
    if fault=='mask_shape': valid=valid[:-1]
    if fault=='mount': config['tool_from_camera'][0][0]=2.
    with pytest.raises((ValueError,TypeError)):
        ObservationEndpointCompensation(base.contract.fk,sample(base.contract.fk,sdk,actual),base.goal,scene,config,valid)


def test_only_margin_failure_is_classified_as_missing_view():
    base,_,sdk,actual = case();scene,config,valid = view_inputs(base.goal)
    correction = ObservationEndpointCompensation(base.contract.fk,sample(base.contract.fk,sdk,actual),base.goal,scene,config,valid)
    # A different FK query still uses the immutable source geometry. This is
    # a geometric unit test, not a physical feedback or calibration claim.
    moved=pose_matrix(pose_values(base.goal));moved[:3,3]+=moved[:3,1]*.2
    with pytest.raises(ValueError,match='image margin'):
        correction.view(moved)


@pytest.mark.parametrize('required_margin', [36., 24.])
def test_gen4_recorded_endpoint_selects_correction_or_no_motion(required_margin):
    """Exact recorded OBB/TF classification, not an IK/robot-response replay.

    The recorded model's FK must match the exact recorded accepted-q transform.
    The timestamped stationary sample is a test transport fixture; SDK=accepted
    is deliberately not claimed to be GEN4's measured following-error pair.
    """
    from geometry_msgs.msg import Pose
    from tf.transformations import quaternion_from_matrix
    from alicia_flexible_grasp.robot.stationary_following import SerialUrdfFk
    record=json.loads((Path(__file__).parent/'fixtures/gen4_view_margin_policy.json').read_text())
    row=record['measured_source_frames'][0]
    actual=np.asarray(row['accepted_q_before_source'])
    # Same report and immutable source as the shared recorded margin fixture:
    # planned_vs_measured_observation_projection.json, mount_tool_from_camera.
    mount=np.array([
        [-.03977481663336216,-.020275293579598223,-.9990029411528496,-.08124712569175627],
        [.003172496405352382,.9997865075298131,-.020417507876746316,.008418611687707862],
        [.9992036325135838,-.003981435871651682,-.03970200171350913,-.10832724917416199],
        [0.,0.,0.,1.]])
    actual_tool=np.asarray(row['base_from_camera'])@np.linalg.inv(mount)
    goal_tool=np.asarray(record['planned']['base_from_camera'])@np.linalg.inv(mount)
    relative=Path('deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf')
    model=next(parent/relative for parent in Path(__file__).parents if (parent/relative).is_file())
    assert hashlib.sha256(model.read_bytes()).hexdigest()=='7812d6b69e854ed9f37b4e3cbd813ecd53dab3504065b2d04d287abc6fd852e0'
    fk=SerialUrdfFk(model.read_text(),ARM_NAMES)
    assert np.allclose(fk(actual),actual_tool,atol=1e-12,rtol=0)
    state=sample(fk,actual,actual)
    goal=Pose()
    goal.position.x,goal.position.y,goal.position.z=goal_tool[:3,3]
    goal.orientation.x,goal.orientation.y,goal.orientation.z,goal.orientation.w=quaternion_from_matrix(goal_tool)
    scene=NS(center=np.asarray(record['geometry_center_base_m']),rotation=np.asarray(record['geometry_axes_base']),
        size=np.asarray(record['geometry_size_xyz_m']),source_ns=record['frozen_geometry_source_ns'])
    camera=record['camera'];fx,fy,cx,cy=camera['intrinsics']
    config=dict(camera=dict(width=camera['width'],height=camera['height'],fx=fx,fy=fy,cx=cx,cy=cy),
        tool_from_camera=mount.tolist(),limits=dict(roi_margin_px=required_margin,
            minimum_depth_m=.07,maximum_depth_m=.5))
    # Source report records zero invalid pixels in this exact OBB+background
    # rectangle. This bitmap fixture does not certify the full RGB canvas.
    valid=np.zeros((camera['height'],camera['width']),dtype=bool)
    for region in (record['planned']['obb'],row['obb_projection']):
        x0,y0,x1,y1=region['background16_box_xyxy'];valid[y0:y1,x0:x1]=True
    correction=ObservationEndpointCompensation(fk,state,goal,scene,config,valid)
    result=correction.current_view(state)
    assert correction.original_view['minimum_image_edge_distance_px']==pytest.approx(59.540175527635796)
    if required_margin==36.:
        assert result is None
    else:
        assert result['minimum_image_edge_distance_px']==pytest.approx(31.139970754524143)
        assert result['actual_arrival_measurement_required']
        assert not result['rgb_identity_verified']


@pytest.mark.parametrize('fault',['stall','reverse','expired','wrong_epoch'])
def test_post_command_response_never_reuses_prediction_as_arrival(fault):
    base,nominal,sdk,actual=case();scene,config,valid=view_inputs(base.goal)
    c=ObservationEndpointCompensation(base.contract.fk,sample(base.contract.fk,sdk,actual),base.goal,scene,config,valid)
    step,_=c.propose(nominal);c.committed(step,12.)
    delta=np.array(step.target_counts)-step.baseline_counts
    q=actual+delta*Q
    if fault=='stall':q=actual
    if fault=='reverse':q=actual-delta*Q
    s=sample(base.contract.fk,step.positions,q,14.)
    if fault=='expired':s['stamp_sec']=90.
    if fault=='wrong_epoch':s['epoch_ns']+=1
    with pytest.raises(ValueError):c.evaluate(s)


def path(goal,velocity=None):
    return RobotTrajectory(joint_trajectory=JointTrajectory(joint_names=list(ARM_NAMES),points=[
        JointTrajectoryPoint(positions=[0.]*6,velocities=[0.]*6,accelerations=[0.]*6,time_from_start=rospy.Duration(0)),
        JointTrajectoryPoint(positions=list(goal),velocities=velocity or [0.]*6,accelerations=[0.]*6,time_from_start=rospy.Duration(2))]))


def test_complete_timed_curve_cannot_leave_32_count_box_between_endpoints():
    goal=[4*Q]*6
    assert validate_observation_correction_path_bounds(path(goal),[0.]*6,goal)['complete_timed_curve_checked']
    with pytest.raises(ValueError,match='COUNT_BUDGET'):
        validate_observation_correction_path_bounds(path(goal,[1.]*6),[0.]*6,goal)


@pytest.mark.parametrize('fault',['goal','start','time','nan'])
def test_path_binding_is_not_only_endpoint_reachability(fault):
    goal=[4*Q]*6;p=path(goal)
    if fault=='goal':p.joint_trajectory.points[-1].positions[0]+=Q
    if fault=='start':p.joint_trajectory.points[0].positions[0]+=Q
    if fault=='time':p.joint_trajectory.points[-1].time_from_start=rospy.Duration(0)
    if fault=='nan':p.joint_trajectory.points[-1].velocities[0]=math.nan
    with pytest.raises(ValueError):validate_observation_correction_path_bounds(p,[0.]*6,goal)
