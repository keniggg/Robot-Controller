#!/usr/bin/env python3
"""Counterfactual offline path certificate; no MoveIt plan and no motion authority."""
import json
from pathlib import Path
from copy import deepcopy
import numpy as np
from genpy.message import fill_message_args
from rospy import Duration
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
from alicia_flexible_grasp.robot.stationary_following import SerialUrdfFk, ARM_NAMES,SDK_QUANTUM_RAD as Q
from alicia_flexible_grasp.robot.observation_path_guard import FrozenObservationScene,validate_observation_trajectory
from alicia_flexible_grasp.robot.observation_tracking_contract import qualify_contract_path,command_derivative_bounds
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry

HERE=Path(__file__).resolve().parents[1]
OUT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/unknown_grasp_20260923/approach_failure_20260927')
def ros_dict(v):
    if isinstance(v,dict):
        if set(v)=={'sec','ns'}: return {'secs':v['ns']//1000000000,'nsecs':v['ns']%1000000000}
        return {k:ros_dict(w) for k,w in v.items()}
    if isinstance(v,list):return [ros_dict(w) for w in v]
    return v
def main():
    if not Path('/media/zhuyupei/SanStand').is_mount():raise SystemExit('SanStand not mounted')
    d=json.loads((OUT/'bag_forensics.json').read_text())
    replay=json.loads((OUT/'fk_replay.json').read_text())
    msg=next(e['msg'] for e in d['events']['/grasp_6d/plan_enriched'] if e['msg']['plan_id']==replay['plan_id'])
    plan=Grasp6DPlan();fill_message_args(plan,[ros_dict(msg)])
    scene=FrozenObservationScene.from_contact_plan(plan)
    xml=json.loads((HERE/'src/alicia_flexible_grasp_supervisor/tests/fixtures/pregrasp_following_20260919.json').read_text())['robot_description']
    fk=SerialUrdfFk(xml,ARM_NAMES)
    audit=next(json.loads(e['msg']['msg'].split(': ',1)[1]) for e in d['events']['/rosout_agg'] if e['msg']['msg'].startswith('Frozen observation continuous path audit: ') and 'held_axes_following_feedforward' in e['msg']['msg'])
    cfg=audit['gripper_geometry_snapshot']
    gripper=GripperGeometry(cfg['max_inner_gap_m'],cfg['width_safety_margin_per_side_m'],np.array(cfg['finger_box_xyz_m']),np.array(cfg['palm_box_xyz_m']),.003)
    state=min(d['series']['/alicia_controller/state'],key=lambda e:abs(e['t']-1790491081.5))
    hold=JointTrajectoryPoint();fill_message_args(hold,[ros_dict(state['msg']['desired'])])
    origin=np.array(replay['states']['pregrasp_after']['sdk_rad'])
    actual=np.array(replay['states']['pregrasp_after']['measured_rad'])
    delta=np.array([-4,0,-18,0,0,32])*Q
    reports=[]
    for duration in (45.203904309,47.0):
        trajectory=RobotTrajectory();trajectory.joint_trajectory.joint_names=list(ARM_NAMES)
        for q,t in ((origin,0.),(origin+delta,duration)):
            trajectory.joint_trajectory.points.append(JointTrajectoryPoint(positions=q.tolist(),velocities=[0.]*6,accelerations=[0.]*6,time_from_start=Duration.from_sec(t)))
        row={'duration_sec':duration,'deadline_sum_motion_goal_time_reserve_sec':duration+8+2,
             'within_original_60sec':duration+8+2<60.,'command_start':origin.tolist(),'command_goal':(origin+delta).tolist(),
             'command_derivatives':command_derivative_bounds(trajectory,hold)}
        for label,tr,h in [('command',trajectory,hold)]:
            try:row[label+'_cad_nominal']=validate_observation_trajectory(tr,scene,fk,gripper,.04975,h)
            except ValueError as e:row[label+'_cad_nominal']={'ok':False,'reason':str(e)}
            row[label+'_tracking']=qualify_contract_path(tr,h,scene,fk,gripper,.04975,audit['controller_constraints_snapshot'],.5,trajectory_stop_reserve=True)
        nominal=deepcopy(trajectory)
        for p in nominal.joint_trajectory.points:p.positions=(np.array(p.positions)-(origin-actual)).tolist()
        nh=deepcopy(nominal.joint_trajectory.points[0])
        try:row['conditional_measured_cad']=validate_observation_trajectory(nominal,scene,fk,gripper,.04975,nh)
        except ValueError as e:row['conditional_measured_cad']={'ok':False,'reason':str(e)}
        reports.append(row)
    result={'kind':'counterfactual_synthetic_quintic_not_MoveIt_not_executed',
      'does_not_certify_hardware_response_or_motion_authority':True,
      'scene_sha256':scene.digest,'model_sha256':fk.model_sha256,'plan_id':plan.plan_id,
      'frozen_pose_index':1,'held_joint_names':['Joint2','Joint5'],'delta_counts':[-4,0,-18,0,0,32],
      'controller_state_bag_time_sec':state['t'],'controller_hold':ros_dict(state['msg']['desired']),
      'controller_constraints':audit['controller_constraints_snapshot'],'results':reports}
    (OUT/'held_approach_counterfactual_path.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
