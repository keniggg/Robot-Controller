"""Replay the sealed r9 endpoint only, with no ROS/network or motion access."""
from pathlib import Path
from types import SimpleNamespace as NS
import json, os, hashlib, traceback, time
W=Path(__file__).resolve().parents[1]
R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1/actual_gen63_baseline_validation_20261023T041149Z/high_accuracy_grasp_r1/arrival_margin_repair_r1')
assert Path('/media/zhuyupei/SanStand').is_mount()
D=R/'supplemental_view_r9/feedback_diagnosis';D.mkdir(exist_ok=True)
os.environ['OFFLINE_BOUNDARY_LOG']=str(D/'offline_boundary.jsonl');exec((R/'guard.py').read_bytes())
import numpy as np
import rosbag
from geometry_msgs.msg import Pose
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES,SDK_QUANTUM_RAD as Q
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.robot.observation_endpoint_compensation import ObservationEndpointCompensation
from test_endpoint_correction import sample
s=json.loads((R/'supplemental_view_r9/actual_validation/task_resources_and_result.json').read_text());rec=Path(s['record_dir'])
failure=json.loads(s['machine_result']['failure_reason'].split('OBSERVATION_COMPENSATION_FAILED: ',1)[1])
audits=[(p,json.loads(p.read_text())) for p in (rec/'keyframes_audits').glob('*.json')]
ap,a=next((p,a) for p,a in audits if a['plan_id']==failure['plan_id'])
source=a['selected']['observation_source_audit'];g=a['replay_geometry'];view=a['selected']['observation_view']['working_volume']
with np.load(R/'supplemental_view_r7/collision_diagnosis/view_search_source.npz') as z:mount=z['tool_from_camera'].copy();valid=z['rgb_valid'].copy()
assert hashlib.sha256(mount.tobytes()).hexdigest()==source['tool_camera_sha256']
assert hashlib.sha256(valid.tobytes()).hexdigest()==source['arrival_projection']['rgb_valid_sha256']
scene=NS(center=np.array(g['obb_center_base_m']),rotation=np.array(g['R_base_obb']),size=np.array(g['obb_size_xyz_m']),source_ns=source['snapshot_stamp_ns'])
config=dict(camera=source['camera_parameters'],tool_from_camera=mount.tolist(),limits=dict(roi_margin_px=view['required_image_margin_px'],minimum_depth_m=view['minimum_camera_depth_m'],maximum_depth_m=view['maximum_camera_depth_m']))
cut=int(s['endpoint_snapshots'][0]['stamp_sec']*1e9)
rows={};history={}
with rosbag.Bag(str(rec/'keyframes.bag')) as bag:
 for topic,m,t in bag.read_messages(topics=['/alicia_d/sdk_command','/alicia_d/accepted_joint_states']):
  ns=m.header.stamp.to_nsec()
  if cut-500_000_000 <= ns <= cut+1_200_000_000:
   row=dict(ns=ns,frame_id=m.header.frame_id,positions=dict(zip(m.name,m.position)))
   history.setdefault(topic,[]).append(row)
   if ns<=cut:rows[topic]=row
sdk=np.array([rows['/alicia_d/sdk_command']['positions'][n] for n in ARM_NAMES]);actual=np.array([rows['/alicia_d/accepted_joint_states']['positions'][n] for n in ARM_NAMES])
assert rows['/alicia_d/sdk_command']['frame_id']=='sdk_transmitted'
model=W/'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf';fk=SerialUrdfFk(model.read_text(),ARM_NAMES)
goal=Pose();goal.position.x,goal.position.y,goal.position.z=failure['fixed_goal_pose'][:3];goal.orientation.x,goal.orientation.y,goal.orientation.z,goal.orientation.w=failure['fixed_goal_pose'][3:]
state=sample(fk,sdk,actual)
out=dict(scope='sealed endpoint replay; timestamps in state are simulated, no path or hardware proof',record=str(rec),source_audit_sha256=ap.stem,initial_sdk=sdk.tolist(),initial_measured=actual.tolist(),goal=failure['fixed_goal_pose'],scene=dict(center=scene.center.tolist(),rotation=scene.rotation.tolist(),size=scene.size.tolist(),source_ns=scene.source_ns),configuration=config,model_sha256=fk.model_sha256,model_relative_path=str(model.relative_to(W)),following_counts=((sdk-actual)/Q).tolist(),feedback_history=history)
try:
 start=time.monotonic();c=ObservationEndpointCompensation(fk,state,goal,scene,config,valid);out['original_view']=c.original_view
 out['current_view']=c.current_view(state)
 step,ev=c.propose_held_axes();out.update(proposal=ev,target_counts=list(step.target_counts),elapsed_sec=time.monotonic()-start)
except Exception as exc:
 out.update(error=str(exc),traceback=traceback.format_exc())
 if 'c' in globals():out['proposal_before_view_gate']=c.recovery.evidence
(D/'endpoint_replay.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({k:v for k,v in out.items() if k in ('original_view','current_view','proposal','error','traceback','proposal_before_view_gate','following_counts')},ensure_ascii=False))
