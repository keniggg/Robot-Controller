#!/usr/bin/env python3
"""Construct an explicitly simulated path between closed G43 numeric inputs."""
import base64
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import socket
import sys
import time
W=Path(__file__).resolve().parent.parent
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N=E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
OUT=N/'verification/gen44_reconstructed_contact_path_preparation_r1'
DEST=W/'src/alicia_flexible_grasp_supervisor/tests/fixtures/gen44_reconstructed_contact_path.json'
sys.path[:0]=[str(W/'src/alicia_flexible_grasp_supervisor/src'),str(W/'devel/lib/python3/dist-packages')]
boundary=[]
class OfflineSocket(socket.socket):
    def __new__(cls,*a,**k):
        boundary.append('socket');raise RuntimeError('offline path provider only')
socket.socket=OfflineSocket
import numpy as np
from scipy.optimize import least_squares
import rospy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
from alicia_flexible_grasp.grasp.contact_execution import plan_aperture_evaluator
from alicia_flexible_grasp.grasp.rich_plan_integrity import pose_values
from alicia_flexible_grasp.robot.pregrasp_compensation import pose_matrix
from alicia_flexible_grasp.robot.observation_path_guard import FrozenObservationScene,SerialUrdfFk,validate_observation_trajectory,wire_digest
from alicia_flexible_grasp.robot.contact_sequence_precheck import contact_precheck_gripper
from alicia_flexible_grasp.robot.contact_linear_compensation import compensate_cartesian_path
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def put(path,value):
    path.write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    return dict(path=str(path),sha256=sha(path))
def path(qs):
    result=RobotTrajectory();result.joint_trajectory.joint_names=list(ARM_NAMES)
    for i,q in enumerate(qs):
        result.joint_trajectory.points.append(JointTrajectoryPoint(positions=list(q),velocities=[0.]*6,
            accelerations=[0.]*6,time_from_start=rospy.Duration(float(i)*4.)))
    return result
def main():
    assert E.is_dir() and not DEST.exists();OUT.mkdir(exist_ok=False)
    source=W/'src/alicia_flexible_grasp_supervisor/tests/fixtures/gen44_g43_stage0_replay.json'
    assert sha(source)=='da66a49b1141e943e1b9e734621400a4130c17443c09516fabc9bc1e8a4c50f0'
    data=json.loads(source.read_text());plan=Grasp6DPlan();plan.deserialize(base64.b64decode(data['plan_wire']['wire_base64']))
    fk=SerialUrdfFk((W/data['robot_model_path']).read_text(),ARM_NAMES);assert fk.model_sha256==data['model_sha256']
    qstart=np.asarray(data['original_precheck']['held_approach_prediction']['predicted_approach']['predicted_positions_rad'])
    goal=pose_matrix(pose_values(plan.poses[2]));sample=deepcopy(data['original_precheck_sample'])
    offset=np.asarray(sample['sdk_positions_rad'])-sample['accepted_positions_rad']
    sample.update(accepted_positions_rad=qstart.tolist(),sdk_positions_rad=(qstart+offset).tolist())
    def inverse(T,seed):
        def residual(q):
            actual=fk(q)
            return np.r_[actual[:3,3]-T[:3,3],(.1*(actual[:3,:3]-T[:3,:3])).ravel()]
        solved=least_squares(residual,seed,bounds=(fk.limits[:,0],fk.limits[:,1]),max_nfev=120,
            ftol=1e-13,xtol=1e-13,gtol=1e-13)
        assert np.linalg.norm(residual(solved.x))<1e-8,(solved.message,residual(solved.x))
        return solved.x
    qend=inverse(goal,qstart)
    nominal=path([qstart,qend]);command,derivation=compensate_cartesian_path(nominal,fk,sample,plan.poses[2])
    scene=FrozenObservationScene.from_contact_plan(plan);gripper=contact_precheck_gripper({});opening=data['original_precheck']['opening_m']
    nominal_audit=validate_observation_trajectory(nominal,scene,fk,gripper,opening,nominal.joint_trajectory.points[0])
    command_audit=validate_observation_trajectory(command,scene,fk,gripper,opening,command.joint_trajectory.points[0])
    # The deliberately bad provider returns an out-of-plane detour. It is a
    # negative planner-output fixture, never a proposed real robot motion.
    below=goal.copy();below[2,3]-=.07
    qbad=inverse(below,qend)
    bad=path([qstart,qbad,qend]);rejection=None
    try:validate_observation_trajectory(bad,scene,fk,gripper,opening,bad.joint_trajectory.points[0])
    except ValueError as exc:rejection=dict(type=type(exc).__name__,reason=str(exc))
    assert rejection is not None,'Bad independent path was not rejected by original kernel'
    result=dict(kind='explicit_reconstructed_G43_contact_path_provider',source_fixture_sha256=sha(source),
        model_sha256=fk.model_sha256,plan_id=plan.plan_id,source='original conditional corrected-approach q and original frozen contact pose',
        historical_MoveIt_trajectory=False,hardware_executed=False,planner_and_response_provider_simulated=True,
        nominal_positions_rad=[qstart.tolist(),qend.tolist()],bad_positions_rad=[qstart.tolist(),qbad.tolist(),qend.tolist()],
        waypoint_interval_sec=4.,sample=sample,opening_m=opening,
        nominal_wire_sha256=wire_digest(nominal),command_wire_sha256=wire_digest(command),
        original_nominal_full_kernel=nominal_audit,original_command_full_kernel=command_audit,
        original_bad_path_rejection=rejection,compensation_derivation=derivation,
        rigid_stage1_report=plan_aperture_evaluator(plan,{},opening,1)(fk(qstart)),
        helper_sha256=sha(Path(__file__)),network_boundary_attempts=boundary)
    assert not boundary
    fixture=put(DEST,result);receipt=put(OUT/'result.json',dict(passed=True,fixture=fixture,ros_init_called=False,
        hardware_contacted=False,temperature_read=False,production_or_existing_tests_modified=False))
    print(json.dumps(receipt,ensure_ascii=False))
if __name__=='__main__':main()
