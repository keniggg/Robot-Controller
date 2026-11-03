"""Frozen failed correction: actual response and existing held-axis alternative."""
from pathlib import Path
from types import SimpleNamespace as NS
import json,hashlib,os,math
W=Path(__file__).resolve().parents[1]
R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1/actual_gen63_baseline_validation_20261023T041149Z/high_accuracy_grasp_r1/arrival_margin_repair_r1')
D=R/'supplemental_view_r8/feedback_diagnosis';assert Path('/media/zhuyupei/SanStand').is_mount()
os.environ['OFFLINE_BOUNDARY_LOG']=str(D/'held_probe_boundary.jsonl');exec((R/'guard.py').read_bytes())
import numpy as np
import rosbag
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES,SDK_QUANTUM_RAD as Q
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.robot.held_axes_prediction import predict_held_axes_endpoint,HeldAxesPredictionError
from alicia_flexible_grasp.robot.observation_endpoint_compensation import observation_endpoint_view
record=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261023T133606.819991Z_a9e156ed5e6046d6b4ee3143e9c66dec')
failure=json.loads((D/'correction_failure.json').read_text());p=failure['proposal']
summary=json.loads((R/'supplemental_view_r8/actual_validation/task_resources_and_result.json').read_text());motion=summary['goals'][1];lo=motion['ns'];hi=motion['result']['ns']
feedback=[];references=[];controllers=[]
with rosbag.Bag(str(record/'keyframes.bag')) as b:
 for topic,m,t in b.read_messages(topics=['/alicia_d/accepted_joint_states','/alicia_d/control_reference','/alicia_controller/state']):
  ns=m.header.stamp.to_nsec()
  if not lo-800_000_000<=ns<=hi+300_000_000:continue
  if topic.endswith('/state'):
   controllers.append(dict(ns=ns,names=list(m.joint_names),desired=list(m.desired.positions),actual=list(m.actual.positions),error=list(m.error.positions)))
  else:
   row=dict(ns=ns,frame_id=m.header.frame_id,positions=dict(zip(m.name,m.position)))
   (feedback if 'accepted' in topic else references).append(row)
(D/'correction_feedback.json').write_text(json.dumps(dict(goal=motion,feedback=feedback,references=references,controllers=controllers),indent=2)+'\n')
pre=np.array(p['measured_start_positions_rad']);sdk=np.array(p['command_start_positions_rad']);last=next(r for r in reversed(feedback) if r['ns']<=hi)
actual=np.array([last['positions'][n] for n in ARM_NAMES]);response=(actual-pre)/Q
ref=next(r for r in reversed(references) if r['ns']<=hi);final_command=np.array([ref['positions'][n] for n in ARM_NAMES])
print(json.dumps(dict(command_delta_counts=p['proposed_delta_counts'],measured_delta_counts=response.tolist(),following_before_counts=((sdk-pre)/Q).tolist(),following_after_counts=((final_command-actual)/Q).tolist()),ensure_ascii=False),flush=True)
fk=SerialUrdfFk((W/'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf').read_text(),ARM_NAMES)
audit=json.loads((record/'keyframes_audits/b629e5212657e8af43254569b9a1b008e6744bfa1847936f82016c2231fe88d0.json').read_text());source=audit['selected']['observation_source_audit'];g=audit['replay_geometry'];view=audit['selected']['observation_view']['working_volume']
with np.load(R/'supplemental_view_r7/collision_diagnosis/view_search_source.npz') as a:mount=a['tool_from_camera'].copy();valid=a['rgb_valid'].copy()
assert hashlib.sha256(mount.tobytes()).hexdigest()==source['tool_camera_sha256']
assert hashlib.sha256(valid.tobytes()).hexdigest()==source['arrival_projection']['rgb_valid_sha256']
scene=NS(center=np.array(g['obb_center_base_m']),rotation=np.array(g['R_base_obb']),size=np.array(g['obb_size_xyz_m']),source_ns=source['snapshot_stamp_ns'])
config=dict(camera=source['camera_parameters'],tool_from_camera=mount.tolist(),limits=dict(roi_margin_px=view['required_image_margin_px'],minimum_depth_m=view['minimum_camera_depth_m'],maximum_depth_m=view['maximum_camera_depth_m']))
result=dict(scope='conditional frozen endpoint alternative; no path or hardware-response proof',measured_delta_counts=response.tolist(),initial_sdk=sdk.tolist(),initial_measured=pre.tolist(),goal=failure['fixed_goal_pose'],scene=dict(center=scene.center.tolist(),rotation=scene.rotation.tolist(),size=scene.size.tolist(),source_ns=scene.source_ns),configuration=config,model_sha256=fk.model_sha256,source_audit_sha256='b629e5212657e8af43254569b9a1b008e6744bfa1847936f82016c2231fe88d0')
try:
 target,evidence=predict_held_axes_endpoint(fk,sdk,pre,failure['fixed_goal_pose'],position_tolerance_m=.010,orientation_tolerance_rad=math.radians(5))
 result.update(held_endpoint=evidence,target_counts=target.tolist())
 try:result['predicted_view']=observation_endpoint_view(scene,config,fk(evidence['predicted_positions_rad']),valid)
 except Exception as exc:result['view_error']=str(exc)
except HeldAxesPredictionError as exc:result.update(held_error=str(exc),held_endpoint=exc.evidence)
(D/'held_axes_probe.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k in ('held_error','held_endpoint','predicted_view','view_error')},ensure_ascii=False),flush=True)
