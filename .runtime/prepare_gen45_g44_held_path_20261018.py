"""One bounded original held prediction plus original full tracking kernel."""
from extract_gen45_g44_pregrasp_inputs_20261018 import *
import math
import numpy as np
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_flexible_grasp.grasp.rich_plan_integrity import pose_values
from alicia_flexible_grasp.robot.observation_path_guard import FrozenObservationScene,wire_digest
from alicia_flexible_grasp.robot.observation_tracking_contract import qualify_contract_path
from alicia_flexible_grasp.robot.held_axes_prediction import predict_held_axes_endpoint


def main():
    out=V/'gen45_g44_held_path_preparation_r1';assert not out.exists();out.mkdir()
    raw=F.read_bytes();assert hashlib.sha256(raw).hexdigest()=='40349dcb9d3e245b13211c2a13880fc1cf9389048b5814db56495f296fb2fe7c'
    data=json.loads(raw);plan=Grasp6DPlan();plan.deserialize(base64.b64decode(data['plan_wire']['wire_base64']))
    fk=SerialUrdfFk((W/data['robot_model_path']).read_text(),ARM_NAMES);initial=data['recomputed_original_snapshots'][0]
    target,prediction=predict_held_axes_endpoint(fk,initial['sdk_positions_rad'],initial['accepted_positions_rad'],
        pose_values(plan.poses[1]),position_tolerance_m=.006,orientation_tolerance_rad=math.radians(5),allow_already_converged=True)
    trajectory=RobotTrajectory();trajectory.joint_trajectory.joint_names=list(ARM_NAMES)
    for q,t in [(initial['sdk_positions_rad'],0.),((target-2048)*2*math.pi/4096,12.)]:
        trajectory.joint_trajectory.points.append(JointTrajectoryPoint(positions=list(q),velocities=[0.]*6,
            accelerations=[0.]*6,time_from_start=rospy.Duration(t)))
    audits=json.loads((M/'same_plan_continuous_trajectory_audits.json').read_text());original=audits[-1]
    constraints=original['controller_constraints_snapshot'];stop=original['execution_tracking_contract']['stop_trajectory_duration_sec']
    g=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)
    checked=qualify_contract_path(trajectory,trajectory.joint_trajectory.points[0],FrozenObservationScene.from_contact_plan(plan),
        fk,g,initial['accepted_gripper_opening_m'],constraints,stop,trajectory_stop_reserve=True)
    d=dict(kind='reconstructed_original_G44_held_path_provider',source_fixture_sha256=hashlib.sha256(raw).hexdigest(),
        plan_id=plan.plan_id,model_sha256=fk.model_sha256,prediction=prediction,
        curve_positions_rad=[list(p.positions)for p in trajectory.joint_trajectory.points],duration_sec=12.,
        trajectory_sha256=wire_digest(trajectory),controller_constraints=constraints,stop_duration_sec=stop,
        original_full_tracking_kernel=checked,historical_MoveIt_trajectory=False,
        future_following_response_simulated=True,hardware_executed=False,
        original_input_sha256={str(p):sha(p)for p in [F,M/'same_plan_continuous_trajectory_audits.json']})
    dst=W/'src/alicia_flexible_grasp_supervisor/tests/fixtures/gen45_g44_held_path.json';assert not dst.exists()
    save(out/'diagnostic.json',d)
    if checked['ok']:save(dst,d)
    save(out/'result.json',dict(prepared=True,full_kernel_passed=checked['ok'],fixture=str(dst)if checked['ok']else None,
        fixture_sha256=sha(dst)if checked['ok']else None,external_boundary_attempts=attempts,ROS_initialized=False,
        hardware_contacted=False,temperature_read=False,production_or_existing_tests_modified=False))
    (out/'helper_source.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps(dict(qualified=checked['ok'],reason=checked.get('reason'),fixture=str(dst),sha256=sha(dst)if dst.exists()else None)))
if __name__=='__main__':main()
