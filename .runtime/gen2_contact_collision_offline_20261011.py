"""Offline recorded candidate/local IK reconstruction. No ROS calls or motion."""
from pathlib import Path
import hashlib, json, math, sys, time
import numpy as np
from scipy.optimize import least_squares
import yaml
from tf.transformations import quaternion_matrix, quaternion_from_matrix, quaternion_slerp

WORK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORK/'src/alicia_flexible_grasp_supervisor/src'))
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk, FrozenObservationScene, validate_observation_trajectory
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from genpy import Duration
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SDK_QUANTUM_RAD
from alicia_flexible_grasp.robot.contact_sequence_precheck import contact_precheck_gripper
from alicia_flexible_grasp.grasp.gripper_geometry import _stage_boxes, _box_corners, _obb_overlap

EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
assert Path('/media/zhuyupei/SanStand').is_mount()
NEW=EXT/'ros_log/full_latest_no_skin_20261011T000625Z'
OUT=NEW/'gen2_analysis/contact_collision'; OUT.mkdir(parents=True, exist_ok=True)
log=NEW/'gen24_fragment_compute_deployment/consumers.log'
rows=[]
for number,line in enumerate(log.read_text().splitlines(),1):
    if 'remote 6D rejected contact evidence:' in line:
        rows.append(dict(line_number=number, log_prefix=line[:line.index('{')], evidence=json.loads(line[line.index('{'): ])))
assert rows and rows[0]['evidence']['plan_id']=='6ce3133fa3ec08a1dfa4ebf0'
(OUT/'recorded_candidates.json').write_text(json.dumps(rows,indent=2)+'\n')
r=rows[0]['evidence']; pc=r['precheck']; sample=pc['source_sample']; g=r['object_geometry']
model=WORK/'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf'
fk=SerialUrdfFk(model.read_text(),ARM_NAMES); assert fk.model_sha256==pc['model_sha256']
cfg=yaml.safe_load((WORK/'src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml').read_text())['grasp_6d']['remote']['gripper_geometry']
gripper=contact_precheck_gripper(cfg); opening=pc['opening_m']
center=np.array(g['pose_xyz_xyzw'][:3]); rotation=quaternion_matrix(g['pose_xyz_xyzw'][3:])[:3,:3]
size=np.array(g['size_xyz_m']); normal=np.array(g['support_normal_base']); plane=g['support_offset_m']
offset=np.array(sample['sdk_positions_rad'])-np.array(sample['accepted_positions_rad'])
start=np.array(pc['stage_start_prediction']['approach']['predicted_actual_joint_positions_rad'])
start_T=fk(start); start_quat=quaternion_from_matrix(start_T)
goal_pose=np.array(r['poses_xyz_xyzw'][2]); goal_T=quaternion_matrix(goal_pose[3:]); goal_T[:3,3]=goal_pose[:3]

def sat_gap(c,R,s):
    axes=[*R.T,*rotation.T]+[np.cross(a,b)for a in R.T for b in rotation.T]
    return max(abs(float((c-center)@a/np.linalg.norm(a)))-float(abs(R.T@(a/np.linalg.norm(a)))@(s/2))-float(abs(rotation.T@(a/np.linalg.norm(a)))@(size/2)) for a in axes if np.linalg.norm(a)>1e-10)

def geometry(q):
    T=fk(q); boxes=[]
    for name,c,s in _stage_boxes(T,gripper,opening,'y','z'):
        boxes.append(dict(name=name, overlap=bool(_obb_overlap(c,T[:3,:3],s,center,rotation,size)),sat_best_separating_gap_m=float(sat_gap(c,T[:3,:3],s)),minimum_support_clearance_m=float(np.min(_box_corners(c,T[:3,:3],s)@normal+plane))))
    return dict(q_rad=np.asarray(q).tolist(), pose_xyz_xyzw=np.r_[T[:3,3],quaternion_from_matrix(T)].tolist(),goal_position_error_m=float(np.linalg.norm(T[:3,3]-goal_T[:3,3])),goal_orientation_error_rad=float(math.acos(np.clip((np.trace(T[:3,:3].T@goal_T[:3,:3])-1)/2,-1,1))),boxes=boxes)

began=time.monotonic(); path=[]; q=start.copy()
steps=int(math.ceil(np.linalg.norm(start_T[:3,3]-goal_pose[:3])/.003))
for index in range(steps+1):
    fraction=index/steps
    target=quaternion_matrix(quaternion_slerp(start_quat,goal_pose[3:],fraction));target[:3,3]=(1-fraction)*start_T[:3,3]+fraction*goal_pose[:3]
    if index:
        def residual(v):
            T=fk(v)
            return np.r_[(T[:3,3]-target[:3,3])*20.,(T[:3,:3]-target[:3,:3]).ravel()]
        sol=least_squares(residual,q,bounds=(fk.limits[:,0],fk.limits[:,1]),max_nfev=100,xtol=1e-12,ftol=1e-12,gtol=1e-12)
        q=sol.x
        assert np.linalg.norm(fk(q)[:3,3]-target[:3,3])<1e-6
    path.append(dict(index=index,fraction=fraction,predicted_actual=geometry(q),command=geometry(q+offset)))
terminal=path[-1]
continuous={}
scene=FrozenObservationScene(r['plan_id'],'offline-reconstructed-no-execution',r['source_stamp_ns'],normal,plane,center,rotation,size)
for kind in ('predicted_actual','command'):
    tr=RobotTrajectory();tr.joint_trajectory.joint_names=list(ARM_NAMES)
    for i,p in enumerate(path):
        tr.joint_trajectory.points.append(JointTrajectoryPoint(positions=p[kind]['q_rad'],velocities=[0.]*6,accelerations=[0.]*6,time_from_start=Duration(i)))
    hold=JointTrajectoryPoint(positions=path[0][kind]['q_rad'],velocities=[0.]*6,accelerations=[0.]*6)
    try:continuous[kind]=dict(success=True,report=validate_observation_trajectory(tr,scene,fk,gripper,opening,hold,max_seconds=5.))
    except ValueError as exc:continuous[kind]=dict(success=False,reason=str(exc))
delta=fk(np.asarray(terminal['command']['q_rad']))[:3,3]-goal_T[:3,3]
frame_delta=dict(world_m=delta.tolist(),tool_xyz_m=(goal_T[:3,:3].T@delta).tolist(),support_normal_m=float(delta@normal),obb_xyz_m=(rotation.T@delta).tolist())
report=dict(scope='offline_local_ik_reconstruction_not_recorded_MoveIt_path',plan_id=r['plan_id'],request=r['request'],source_stamp_ns=r['source_stamp_ns'],model_path=str(model),model_sha256=fk.model_sha256,source_log=str(log),recorded_first_failure=pc['reason'],original_failed_trajectory_recorded=False,original_collision_joint_state_recorded=False,moveit_or_ros_called=False,hardware_motion=False,limits_unchanged=True,configuration_scope='deployed-source gripper physical contract, current measured opening from recorded audit',gripper_config=cfg,opening_m=opening,source_sample=sample,sdk_minus_encoder_rad=offset.tolist(),sdk_minus_encoder_counts=(offset/SDK_QUANTUM_RAD).tolist(),conditional_constant_bias=True,future_bias_verified=False,registered_goal_pose=goal_pose.tolist(),scene=g,steps=steps,paths=path,elapsed_sec=time.monotonic()-began,summary={k:dict(colliding_waypoints=[p['index']for p in path if any(b['overlap'] for b in p[k]['boxes'])],terminal=terminal[k])for k in ('predicted_actual','command')},interpretation='Exact recorded goal/start/bias/model, independent local IK branch; not exact recorded retimed curve or execution proof.')
sources=[Path(__file__),model,WORK/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/contact_sequence_precheck.py',WORK/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/contact_linear_compensation.py',WORK/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_path_guard.py',WORK/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/gripper_geometry.py']
report['source_sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest()for p in sources}
report['reconstructed_complete_spline']=continuous
report['command_relative_to_fixed_goal_translation']=frame_delta
report['reconstructed_timing_scope']='Independent zero-derivative waypoint timing used only for offline continuous CAD proof; original MoveIt timing is not recorded.'
(OUT/'offline_reconstruction.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(dict(output=str(OUT/'offline_reconstruction.json'),steps=steps,elapsed_sec=report['elapsed_sec'],offset_counts=report['sdk_minus_encoder_counts'],continuous=continuous,command_relative_to_fixed_goal_translation=frame_delta,summary=report['summary']),indent=2))
