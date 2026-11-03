"""Bounded derivation from sealed GEN30 evidence; no ROS or hardware."""
import json,hashlib,re,collections
from pathlib import Path
import numpy as np
import rosbag
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
R=E/'grasp_records/20261016T021226.502516Z_0494c8bc84a34a74933e8eae122dd1f4'
D=E/'ros_log/full_latest_gen29_restart_20261016T000728Z/verification/gen30_attempt1_timing_diagnosis'
O=D/'candidate_and_station_evidence';O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
summary=json.loads((D/'observed_timing.json').read_text()); checks=json.loads((D/'paired_strict_view_checks.json').read_text())
assert summary['sealed_inputs_checked'] and summary['strict_check_count']==39
pairs=[]
for i,r in enumerate(checks):
 m=re.search(r'xyz=\(([^)]+)\) q=\(([^)]+)\)',r['request_message']); xyz,q=[list(map(float,s.split(','))) for s in m.groups()]
 pairs.append(dict(index=i+1,request_id=r['interval_request_id'],xyz_m=xyz,quaternion_xyzw=q,start_log_ns=r['start_log_ns'],wall_sec=r['wall_sec'],execute=False,result_code='OBSERVATION_PLANNING_TIMEOUT'))
nearest=[]
for row in pairs[23:]:
 distances=[]
 for old in pairs[:23]:
  t=float(np.linalg.norm(np.array(row['xyz_m'])-old['xyz_m'])); q=np.array(row['quaternion_xyzw']);p=np.array(old['quaternion_xyzw']); a=float(np.degrees(2*np.arccos(min(1.,abs(q@p)/np.linalg.norm(q)/np.linalg.norm(p)))))
  distances.append(dict(previous_index=old['index'],translation_m=t,rotation_deg=a,ranking_only=t+np.deg2rad(a)*.1))
 near=min(distances,key=lambda x:x['ranking_only']);nearest.append(dict(index=row['index'],**near))
start=summary['phase_start_ns'];end=summary['phase_deadline_ns']+int(2e9)
feedback=[];remote=[];task=[];requests=[]
with rosbag.Bag(str(R/'keyframes.bag')) as bag:
 for topic,m,stamp in bag.read_messages(topics=['/alicia_d/accepted_joint_states','/rosout_agg','/grasp/state','/grasp_mode/observe_return_measurement_request']):
  if topic=='/alicia_d/accepted_joint_states':
   ns=m.header.stamp.to_nsec()
   if start<=ns<=end and m.header.frame_id=='sdk_measured':feedback.append(dict(source_stamp_ns=ns,names=list(m.name),position=list(m.position)))
  elif topic=='/rosout_agg':
   if m.name=='/remote_grasp6d_node' and start<=m.header.stamp.to_nsec()<=end and 'pipeline metrics:' not in m.msg:
    remote.append(dict(log_ns=m.header.stamp.to_nsec(),message=m.msg))
  elif topic=='/grasp/state':task.append(dict(source_stamp_ns=m.header.stamp.to_nsec(),state=m.state,message=m.message))
  else:
   d=json.loads(m.data); requests.append({k:d.get(k)for k in ['state','request_stamp_ns','view_index','motion_kind','lease_id']})
assert feedback and all(r['names']==feedback[0]['names']for r in feedback)
q=np.array([r['position']for r in feedback]);span=np.ptp(q,axis=0).tolist()
report=dict(inputs={p.name:sha(p)for p in [D/'observed_timing.json',D/'paired_strict_view_checks.json',R/'manifest.json']},bag_sha256=summary['bag_sha256'],candidates=pairs,rounded_exact_pose_repeat_count=len(pairs)-len({tuple(r['xyz_m']+r['quaternion_xyzw'])for r in pairs}),nearest_previous_request=nearest,pose_resolution=dict(xyz_m=.001,quaternion_component=.001,meaning='Logs rounded to 3 decimals; differences are approximate, not exact-pose equality or unreachable evidence. nearest ranking uses translation+0.1m*rotation only for report.'),station_feedback=dict(count=len(feedback),names=feedback[0]['names'],first_source_ns=feedback[0]['source_stamp_ns'],last_source_ns=feedback[-1]['source_stamp_ns'],peak_to_peak_rad=span,maximum_span_rad=max(span)),measurement_requests=requests,task_states=task,remote_nonmetric_logs=remote,conclusion=dict(strict_checks_39_plan_only=True,no_second_measurement_or_task_move=True,second_request_restarts_similar_poses=True,exact_negative_reuse_not_proven=True,ik_cause='Goal tree sampling failed during each original one-second planning allowance. Record does not distinguish true IK infeasibility, collision, solver exhaustion or joint-limit branch failure.',first_cause='No supplemental full path was found during repeated search; original phase deadline exhausted. Fresh target status continued through the deadline; terminal TARGET_LOST classified scope expiration, not prior visual loss.',remaining_unknown=['No completed request18/87 candidate/audit report; exact family IDs, IK hints, FK proposal failures and all rejected endpoints are absent.', 'Pre-first-check 9.879/10.183 seconds not attributed to CPU or a specific routine.', 'No guarantee another candidate is physically reachable; requires current full gates.']),no_ros_network_or_temperature_or_hardware=True)
(O/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
(O/'receipt.json').write_text(json.dumps(dict(result_sha256=sha(O/'result.json'),script_sha256=sha(Path(__file__)),readback=True),indent=2)+'\n')
print(json.dumps(dict(path=str(O),result_sha256=sha(O/'result.json'),feedback=report['station_feedback'],exact_repeat_count=report['rounded_exact_pose_repeat_count'],remote_nonmetric_log_count=len(remote)),indent=2))
