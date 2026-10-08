#!/usr/bin/env python3
"""Offline FK and bounded-guide replay; never initializes ROS."""
import json
import math
from pathlib import Path
import sys
import numpy as np
from tf.transformations import quaternion_from_matrix
from alicia_flexible_grasp.robot.pregrasp_compensation import PregraspCompensation,pose_matrix
from alicia_flexible_grasp.robot.stationary_following import SerialUrdfFk,ARM_NAMES,SDK_QUANTUM_RAD as Q

HERE=Path(__file__).resolve().parents[1]
OUT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/unknown_grasp_20260923/approach_failure_20260927')
sys.path.insert(0,str(HERE/'src/alicia_flexible_grasp_supervisor/tests'))
from test_endpoint_correction import sample

def pose_values(p):
    return [p['position'][k] for k in ('x','y','z')]+[p['orientation'][k] for k in ('x','y','z','w')]

def main():
    if not Path('/media/zhuyupei/SanStand').is_mount():
        raise SystemExit('SanStand not mounted')
    d=json.loads((OUT/'bag_forensics.json').read_text())
    xml=json.loads((HERE/'src/alicia_flexible_grasp_supervisor/tests/fixtures/pregrasp_following_20260919.json').read_text())['robot_description']
    fk=SerialUrdfFk(xml,ARM_NAMES)
    assert fk.model_sha256=='890592a24b8ad818d995e69beb670ea4254bf84f589615bfac3c8562f62449b7'
    plan=next(e['msg'] for e in d['events']['/grasp_6d/plan_enriched'] if e['msg']['plan_id']=='a37bd0366fc9fbbbf059d2e4')
    goals=[pose_values(p) for p in plan['poses']]
    values={}
    for tag,when in [('pregrasp_after',1790491081.5),('approach_failed',1790491095.8)]:
        sdk=np.array(min(d['series']['/alicia_d/sdk_command'], key=lambda r:abs(r['t']-when))['position'][:6])
        actual=np.array(min(d['series']['/alicia_d/accepted_joint_states'], key=lambda r:abs(r['t']-when))['position'][:6])
        transform=fk(actual)
        row={'sdk_rad':sdk.tolist(),'measured_rad':actual.tolist(),
             'sdk_counts':np.rint(sdk/Q+2048).astype(int).tolist(),'measured_counts':np.rint(actual/Q+2048).astype(int).tolist(),
             'sdk_minus_measured_rad':(sdk-actual).tolist(),
             'measured_fk_xyz':transform[:3,3].tolist(),'measured_fk_quaternion_xyzw':quaternion_from_matrix(transform).tolist(),
             'residuals':{}}
        for i,goal in enumerate(goals):
            g=pose_matrix(goal)
            delta=transform[:3,3]-g[:3,3]
            angle=math.acos(np.clip((np.trace(g[:3,:3].T@transform[:3,:3])-1)/2,-1,1))
            r={'position_m':float(np.linalg.norm(delta)),'delta_xyz_m':delta.tolist(),'orientation_rad':angle,'orientation_deg':math.degrees(angle)}
            try:
                c=PregraspCompensation(fk,sample(fk,sdk,actual),goal,position_tolerance_m=.010)
                guide,best=c._remaining_budget_guide(actual)
                pp,pa=c.residual(actual+guide*Q)
                r['guide']={'counts':guide.tolist(),'predicted_position_m':pp,'predicted_angle_deg':math.degrees(pa),'within_10mm':pp<=.01}
            except ValueError as e:
                r['guide_rejected']=str(e)
            row['residuals'][str(i)]=r
        values[tag]=row
    out={'model_sha256':fk.model_sha256,'plan_id':plan['plan_id'],'goal_poses':goals,'states':values}
    (OUT/'fk_replay.json').write_text(json.dumps(out,indent=2))
    print(json.dumps(out,indent=2))

if __name__=='__main__':main()
