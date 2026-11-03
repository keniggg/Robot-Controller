#!/usr/bin/env python3
"""Read sealed gen15 state and non-temperature accepted joint feedback only."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
import rosbag

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW=ROOT/'ros_log/full_latest_ros_20261009T003407Z'
REC=ROOT/'grasp_records/20261009T052127.805554Z_6b8770587c5c4cd8abc679178eaa1c70'
OUT=NEW/'gen15_handoff_analysis/final_status_fields.json'
manifest_path=REC/'manifest.json';manifest_bytes=manifest_path.read_bytes();manifest=json.loads(manifest_bytes)
assert manifest['state']=='complete' and manifest['recorder_closed'] is True
summary_path=NEW/'gen15_handoff_analysis/arrivals_and_terminal.json';summary=json.loads(summary_path.read_text())
arrivals=summary['arrivals'];arrival3=arrivals[-1];epoch=arrival3['settle']['epoch_ns'];arrival_ns=arrival3['arrival_stamp_ns']
feedback=[];selection=[];task=[];epochs=[]
def utc(ns):return datetime.datetime.fromtimestamp(ns//10**9,datetime.timezone.utc).strftime('%Y-%m-%dT%H:%M:%S')+'.'+str(ns%10**9).zfill(9)+'Z'
with rosbag.Bag(str(REC/'keyframes.bag')) as bag:
 bounds=dict(start_time_utc=utc(int(bag.get_start_time()*1e9)),end_time_utc=utc(int(bag.get_end_time()*1e9)),timestamp_precision='bag API returns float seconds; exact message/header stamps below are integer nanoseconds')
 for topic,msg,received in bag.read_messages(topics=['/alicia_d/accepted_joint_states','/alicia_d/control_reference_epoch','/grasp_mode/selection','/grasp/state']):
  if topic=='/alicia_d/accepted_joint_states':
   if msg.header.frame_id=='sdk_measured' and msg.header.stamp.to_nsec()>=arrival_ns:
    feedback.append(dict(source_topic=topic,header_stamp_ns=msg.header.stamp.to_nsec(),bag_received_ns=received.to_nsec(),frame_id=msg.header.frame_id,seq=msg.header.seq,joint_names=list(msg.name),positions_rad=list(msg.position)))
  elif topic=='/alicia_d/control_reference_epoch':epochs.append(dict(stamp_ns=msg.stamp.to_nsec(),frame_id=msg.frame_id,bag_received_ns=received.to_nsec()))
  else:
   data=(dict(state=msg.state,message=msg.message,active=msg.active,success=msg.success,header_stamp_ns=msg.header.stamp.to_nsec()) if topic=='/grasp/state' else json.loads(msg.data));row=dict(topic=topic,bag_received_ns=received.to_nsec(),message=data)
   (selection if topic=='/grasp_mode/selection' else task).append(row)
assert feedback and task
if not selection:
 r=summary['measurement_requests'][-1]
 selection=[dict(topic='/grasp_mode/observe_return_measurement_request',bag_received_ns=r['received_ns'],message=r['message']['selection'],extraction='exact selection field from recorded lifecycle request; selection parameter itself is not a bag topic')]
last=feedback[-1];names=last['joint_names'];positions=np.array([row['positions_rad'] for row in feedback]);assert all(row['joint_names']==names for row in feedback)
assert positions.shape[1]>=6
expected=np.array(arrival3['settle']['accepted_positions_rad']);assert np.array_equal(positions[-1,:6],expected)
last.update(header_stamp_utc=utc(last['header_stamp_ns']),bag_received_utc=utc(last['bag_received_ns']))
maxdelta=np.max(np.abs(positions[:,:6]-expected),axis=0)
arr=[]
for row in arrivals:
 item=dict(row);item.update(arrival_stamp_utc=utc(row['arrival_stamp_ns']),both_qualified_ack_utc=utc(row['both_qualified_ack_ns']));arr.append(item)
loss=summary['first_loss']['message'];terminal=dict(generation=loss['generation'],reason=loss['first_loss']['reason'],failed_stage=loss['frame_timing']['failed_stage'],failed_source_stamp_ns=loss['source_stamp_ns'],failed_source_utc=utc(loss['source_stamp_ns']),observed_stamp_ns=loss['first_loss']['observed_stamp_ns'],observed_utc=utc(loss['first_loss']['observed_stamp_ns']),last_good_source_stamp_ns=loss['first_loss']['last_good_source_stamp_ns'],last_task_state=task[-1])
assert terminal['generation']==15
sources={str(manifest_path):hashlib.sha256(manifest_bytes).hexdigest(),str(summary_path):hashlib.sha256(summary_path.read_bytes()).hexdigest()}
for name in ['alignment_receipt.json','authorization.json','command_result.json','recorder_outcome.json','runner.log']:
 p=REC/name;sources[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
bag_manifest=next(row for row in manifest['files'] if row['path']=='keyframes.bag');assert (REC/'keyframes.bag').stat().st_size==bag_manifest['size_bytes']
assert manifest_path.read_bytes()==manifest_bytes
report=dict(scope='sealed gen15 actual experimental facts for status correction; never a live current pose or a new alignment authorization',record_id=manifest['record_id'],record_path=str(REC),manifest_state=manifest['state'],recorder_closed=True,result=manifest['result'],failure_reason=manifest['failure_reason'],started_at=manifest['started_at'],completed_at=manifest['completed_at'],bag_metadata=bounds,bag_manifest_declared_sha256=bag_manifest['sha256'],bag_size_bytes_verified=bag_manifest['size_bytes'],alignment_session_dir=str(NEW/'realigned_20261009T052115Z'),selection=selection[-1],control_reference_epoch_ns=epoch,recorded_control_epoch_messages=epochs,arrivals=arr,final_automatic_observation_endpoint=dict(last_recorded_accepted_feedback=last,arm_joint_names=names[:6],arm_positions_rad=last['positions_rad'][:6],equals_third_arrival_settle_exactly=True,accepted_feedback_samples_after_third_arrival=len(feedback),max_arm_delta_from_third_arrival_settle_rad=maxdelta.tolist(),joint_peak_to_peak_rad=np.ptp(positions,axis=0).tolist(),stationary_through_last_recorded_feedback=bool(np.all(maxdelta==0.)),only_valid_through_recorded_stamp_ns=last['header_stamp_ns']),terminal=terminal,physical_grasp_success_confirmed=False,grasp_task_states_seen=sorted({r['message']['state'] for r in task}),source_sha256=sources,manifest_unchanged_during_read=True,root_status_pointers_modified=False,production_modified=False,ros_calls=False,temperature_read=False,limitations=['Only the sealed recorder is inspected. User/manual movement after its final accepted feedback is neither excluded nor authorized by this file.','The final joint vector is the third automatic observation endpoint; the experiment ended before a successful physical grasp.','Bag SHA is copied from the sealed manifest and size checked here, not recomputed; individual small source files are freshly SHA256 hashed.'])
OUT.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(dict(path=str(OUT),sha256=hashlib.sha256(OUT.read_bytes()).hexdigest(),final_arm_q=last['positions_rad'][:6],last_feedback_ns=last['header_stamp_ns'],samples_after_arrival=len(feedback),delta=maxdelta.tolist(),result=manifest['result']),ensure_ascii=False,indent=2))
