"""Offline height-window diagnosis. No ROS master, HTTP, production edits."""
from pathlib import Path
import sys,os,json,hashlib,math,time,copy
os.environ['MUJOCO_GL']='disable';os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1';sys.dont_write_bytecode=True
WORK=Path(__file__).resolve().parents[1]
# Only definitions from diagnostic tracer; do not execute its two replay jobs.
exec((WORK/'.runtime/frozen_gen2_mujoco_contact_trace_20261012.py').read_text().split('reports=[]')[0])
from scipy.optimize import least_squares
from tf.transformations import quaternion_matrix,quaternion_from_matrix,quaternion_slerp
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk,FrozenObservationScene,validate_observation_trajectory
from alicia_flexible_grasp.robot.contact_sequence_precheck import contact_precheck_gripper
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES
from alicia_flexible_grasp.grasp.gripper_geometry import _stage_boxes,_box_corners,_obb_overlap
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from genpy import Duration
import yaml
OUT=BASE/'full_latest_frozen_no_skin_20261012T021050Z/gen2_analysis/height_intersection_consistent_close';OUT.mkdir(parents=True,exist_ok=True)
model=WORK/'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf';fk=SerialUrdfFk(model.read_text(),ARM_NAMES)
cfg=yaml.safe_load((WORK/'src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml').read_text())['grasp_6d']['remote']['gripper_geometry'];gripper=contact_precheck_gripper(cfg)
reconstructed=BASE/'full_latest_frozen_no_skin_20261012T021050Z/gen2_analysis/compensation_collision'
allreports=[]
for directory,source,original_raise in zip(sorted((SERVICE/'requests').iterdir()),(3001,3154),(.002581253301646225,.002778816101082598)):
 base_payload=json.loads((directory/'request.raw.json').read_text());old=json.loads((reconstructed/str(source)/'recorded_candidates.json').read_text())[0]['evidence'];pc=old['precheck'];geo=old['object_geometry'];offset=np.array(pc['source_sample']['sdk_positions_rad'])-np.array(pc['source_sample']['accepted_positions_rad']);start=np.array(pc['stage_start_prediction']['approach']['predicted_actual_joint_positions_rad']);initial=fk(start);initialquat=quaternion_from_matrix(initial);normal=np.array(geo['support_normal_base']);rotation=quaternion_matrix(geo['pose_xyz_xyzw'][3:])[:3,:3];center=np.array(geo['pose_xyz_xyzw'][:3]);size=np.array(geo['size_xyz_m'])
 scene=FrozenObservationScene(old['plan_id'],'offline-independent-height-scan',old['source_stamp_ns'],normal,geo['support_offset_m'],center,rotation,size)
 for height_mm in (0.,1.1,1.3,1.5,1.8,original_raise*1000.):
  payload=copy.deepcopy(base_payload);payload['gripper']['close_target_inner_gap_m']+=.003;delta=height_mm*.001-original_raise
  for pose in payload['trajectory']:pose['position_m']=(np.array(pose['position_m'])+delta*normal).tolist()
  backend=Trace();backend._import_mujoco();checked=engine._validate_v3_payload(payload,now_sec=float(payload['snapshot_stamp_sec'])+1.,max_snapshot_age_sec=120.)
  response=backend._simulate_validated_grasp(checked,payload['plan_id'])
  response['candidate_source']=payload['candidate_source'];response['candidate_source_lineage']=payload['candidate_source_lineage']
  from alicia_flexible_grasp.vision.mujoco_digital_twin_client import validate_mujoco_gate_response
  gate=validate_mujoco_gate_response(response,payload['plan_id'],80,expected_candidate_source=payload['candidate_source'],expected_candidate_source_lineage=payload['candidate_source_lineage'],require_lift_success=False)
  goal=payload['trajectory'][2];goalpos=np.array(goal['position_m']);goalquat=np.array(goal['quaternion_xyzw']);path=[];q=start.copy();steps=int(math.ceil(np.linalg.norm(initial[:3,3]-goalpos)/.003))
  for index in range(steps+1):
   a=index/steps;T=quaternion_matrix(quaternion_slerp(initialquat,goalquat,a));T[:3,3]=(1-a)*initial[:3,3]+a*goalpos
   if index:
    def residual(v):
     actual=fk(v);return np.r_[(actual[:3,3]-T[:3,3])*20.,(actual[:3,:3]-T[:3,:3]).ravel()]
    sol=least_squares(residual,q,bounds=(fk.limits[:,0],fk.limits[:,1]),max_nfev=100,xtol=1e-12,ftol=1e-12,gtol=1e-12);q=sol.x;assert np.linalg.norm(fk(q)[:3,3]-T[:3,3])<1e-6
   path.append(q.copy())
  continuous={}
  for name,bias in (('nominal',np.zeros(6)),('predicted_command',offset)):
   tr=RobotTrajectory();tr.joint_trajectory.joint_names=list(ARM_NAMES)
   for i,p in enumerate(path):tr.joint_trajectory.points.append(JointTrajectoryPoint(positions=(p+bias).tolist(),velocities=[0.]*6,accelerations=[0.]*6,time_from_start=Duration(i)))
   hold=JointTrajectoryPoint(positions=(path[0]+bias).tolist(),velocities=[0.]*6,accelerations=[0.]*6)
   try:continuous[name]=dict(ok=True,report=validate_observation_trajectory(tr,scene,fk,gripper,pc['opening_m'],hold,max_seconds=5.))
   except ValueError as exc:continuous[name]=dict(ok=False,reason=str(exc))
  preload=[s for s in backend.trace if s['phase']=='preload_or_lift'];closure=[s for s in backend.trace if s['phase']=='closure_scan'];lastpre=preload[399]if len(preload)>=400 else None
  rec=dict(source_plan_id=base_payload['plan_id'],close_counterfactual_increment_m=.003,current_no_lift_gate=dict(ok=gate.ok,code=gate.code,reason=gate.reason,score=gate.score),height_offset_from_original_m=height_mm*.001,delta_from_recorded_proposal_m=delta,simulated_payload=payload,response=response,local_contact_curve=continuous,local_contact_start_note='unchanged recorded conditional approach joint start; altered Cartesian goal; all preceding current-stage policies require fresh strict revalidation',local_contact_q_path=[p.tolist()for p in path],recorded_following_bias_rad=offset.tolist(),closure_two_sided=any(s['classification']['two_sided']for s in closure),preload_completed=lastpre is not None,preload_two_sided=(lastpre['classification']['two_sided']if lastpre else False),last_preload_state=lastpre,trace=backend.trace,motion_authorized=False,ros_called=False,http_called=False,production_changed=False,physics_changed=False,full_moveit_sequence_checked=False,original_timed_curve_replayed=False,execution_authorized=False)
  out=OUT/(base_payload['plan_id']+'_'+str(height_mm).replace('.','p')+'mm.json');out.write_text(json.dumps(rec,indent=2,ensure_ascii=False)+'\n')
  summary=dict(path=str(out),sha256=sha(out),height_mm=height_mm,plan=base_payload['plan_id'],geometry_ok=all(v['ok']for v in continuous.values()),contact_curve=continuous,closure_two_sided=rec['closure_two_sided'],preload_two_sided=rec['preload_two_sided'],failure=response.get('failure_reason'),overall_simulation_ok=response.get('simulation_ok'),no_lift_gate_ok=gate.ok,no_lift_gate_code=gate.code)
  allreports.append(summary);print(json.dumps({k:v for k,v in summary.items()if k!='contact_curve'},ensure_ascii=False),flush=True)
(OUT/'summary.json').write_text(json.dumps(dict(rows=allreports,scope='independent_offline_diagnostic_only',unchanged_pose_orientation=True,unchanged_thresholds=True,full_moveit_sequence_checked=False,required_before_execution=['fresh genuine measured start/bias','all changed stages under actual held-axis controller policy','full retimed command curve and whole-arm scene checks','current target identity/registration and contact geometry','current MuJoCo closed-contact gate','real physical closure/holding verification']),indent=2)+'\n')
