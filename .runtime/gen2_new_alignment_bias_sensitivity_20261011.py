"""New-station measured bias on old candidate: sensitivity only, never a plan."""
from pathlib import Path
import hashlib,json,sys,math
import numpy as np
from tf.transformations import quaternion_matrix,quaternion_from_matrix
WORK=Path(__file__).resolve().parents[1];sys.path.insert(0,str(WORK/'src/alicia_flexible_grasp_supervisor/src'))
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk,FrozenObservationScene,validate_observation_trajectory
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES,SDK_QUANTUM_RAD
from alicia_flexible_grasp.robot.contact_sequence_precheck import contact_precheck_gripper
from alicia_flexible_grasp.grasp.gripper_geometry import _stage_boxes,_box_corners,_obb_overlap
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from genpy import Duration
assert Path('/media/zhuyupei/SanStand').is_mount()
BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_no_skin_20261011T000625Z/gen2_analysis')
src=BASE/'contact_collision/offline_reconstruction.json';r=json.loads(src.read_text())
news=BASE/'new_alignment_following_1791679671670862832.json';sample=json.loads(news.read_text())
fk=SerialUrdfFk(Path(r['model_path']).read_text(),ARM_NAMES);assert sample['model_sha256']==fk.model_sha256
offset=np.array(sample['sdk_positions_rad'])-np.array(sample['accepted_positions_rad']);gripper=contact_precheck_gripper(r['gripper_config']);opening=r['opening_m'];obj=r['scene']
normal=np.array(obj['support_normal_base']);center=np.array(obj['pose_xyz_xyzw'][:3]);rotation=quaternion_matrix(obj['pose_xyz_xyzw'][3:])[:3,:3];size=np.array(obj['size_xyz_m']);scene=FrozenObservationScene(r['plan_id'],'old_scene_new_bias_sensitivity',r['source_stamp_ns'],normal,obj['support_offset_m'],center,rotation,size)
tr=RobotTrajectory();tr.joint_trajectory.joint_names=list(ARM_NAMES)
for i,p in enumerate(r['paths']):tr.joint_trajectory.points.append(JointTrajectoryPoint(positions=(np.array(p['predicted_actual']['q_rad'])+offset).tolist(),velocities=[0.]*6,accelerations=[0.]*6,time_from_start=Duration(i)))
hold=JointTrajectoryPoint(positions=tr.joint_trajectory.points[0].positions,velocities=[0.]*6,accelerations=[0.]*6)
try:continuous=dict(success=True,report=validate_observation_trajectory(tr,scene,fk,gripper,opening,hold,max_seconds=5.))
except ValueError as exc:continuous=dict(success=False,reason=str(exc))
T=fk(tr.joint_trajectory.points[-1].positions);goal=quaternion_matrix(r['registered_goal_pose'][3:]);goal[:3,3]=r['registered_goal_pose'][:3];delta=T[:3,3]-goal[:3,3];boxes=[]
for name,c,s in _stage_boxes(T,gripper,opening,'y','z'):
 axes=[*T[:3,:3].T,*rotation.T]+[np.cross(a,b)for a in T[:3,:3].T for b in rotation.T]
 sep=max(abs(float((c-center)@(a/np.linalg.norm(a))))-float(abs(T[:3,:3].T@(a/np.linalg.norm(a)))@(s/2))-float(abs(rotation.T@(a/np.linalg.norm(a)))@(size/2))for a in axes if np.linalg.norm(a)>1e-10)
 boxes.append(dict(body=name,sat_gap_m=sep,overlap=bool(_obb_overlap(c,T[:3,:3],s,center,rotation,size)),support_clearance_m=float(np.min(_box_corners(c,T[:3,:3],s)@normal+obj['support_offset_m']))))
report=dict(scope='old_frozen_goal_with_new_station_bias_sensitivity_only',old_goal_plan_id=r['plan_id'],old_reconstruction_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),new_sample_path=str(news),new_sample_sha256=hashlib.sha256(news.read_bytes()).hexdigest(),new_measured_station_sample=sample,model_sha256=fk.model_sha256,sdk_minus_encoder_counts=(offset/SDK_QUANTUM_RAD).tolist(),command_endpoint_pose=np.r_[T[:3,3],quaternion_from_matrix(T)].tolist(),command_offset_tool_xyz_m=(goal[:3,:3].T@delta).tolist(),command_offset_world_m=delta.tolist(),command_offset_norm_m=float(np.linalg.norm(delta)),command_orientation_error_rad=float(math.acos(np.clip((np.trace(T[:3,:3].T@goal[:3,:3])-1)/2,-1,1))),boxes=boxes,continuous=continuous,new_actual_goal_measured=False,future_bias_verified=False,execution_authorized=False,limitations=['New bias at new alignment is applied to an OLD frozen goal and old independently reconstructed curve only as sensitivity.','New epoch/selection/source/context and current geometry must be freshly validated for any real trial; this is not a current executable plan.'])
out=BASE/'contact_collision/new_alignment_bias_sensitivity.json';out.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(dict(output=str(out),counts=report['sdk_minus_encoder_counts'],tool_offset_m=report['command_offset_tool_xyz_m'],offset_norm_m=report['command_offset_norm_m'],boxes=boxes,continuous=continuous),indent=2))
