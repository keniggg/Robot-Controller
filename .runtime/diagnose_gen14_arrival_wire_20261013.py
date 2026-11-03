from pathlib import Path
import json,hashlib
import numpy as np
import rosbag
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
R=N.parents[1]/'grasp_records/20261013T025604.344486Z_af3ecb9373264cc9b19a98143f7b5df5'
out=N/'verification/gen14_arrival_diagnosis';out.mkdir(parents=True,exist_ok=True)
requests=[];acks=[];statuses=[];joints=[];task=[]
with rosbag.Bag(str(R/'keyframes.bag')) as bag:
 for topic,msg,stamp in bag.read_messages(topics=['/perception/unknown/detector_status','/grasp_mode/observe_return_measurement_ack','/grasp_mode/observe_return_measurement_request','/alicia_d/accepted_joint_states','/grasp/state']):
  ns=stamp.to_nsec()
  if topic=='/grasp_mode/observe_return_measurement_request':requests.append(dict(bag_ns=ns,wire=json.loads(msg.data)))
  elif topic=='/grasp_mode/observe_return_measurement_ack':acks.append(dict(bag_ns=ns,wire=json.loads(msg.data)))
  elif topic=='/perception/unknown/detector_status':statuses.append(dict(bag_ns=ns,wire=json.loads(msg.data)))
  elif topic=='/alicia_d/accepted_joint_states':joints.append(dict(source_ns=msg.header.stamp.to_nsec(),q=list(msg.position),names=list(msg.name),frame_id=msg.header.frame_id))
  else:task.append(dict(bag_ns=ns,state=msg.state,active=msg.active,message=msg.message))
request=next(row for row in requests if row['wire']['state']=='requalifying')
r=request['wire'];start=r['arrival_stamp_ns'];end=r['capture_deadline_ns']
terminal=next(row for row in task if row['bag_ns']>=start and row['state']=='FAILED')
rows=[];raw=[]
for row in statuses:
 d=row['wire'];o=d.get('observation_measurement',{});src=d.get('source_stamp_ns')
 if start<row['bag_ns']<=terminal['bag_ns'] and src is not None and src>start:
  raw.append(row)
  appearances=o.get('appearance_identity',[])
  rows.append(dict(bag_ns=row['bag_ns'],source_ns=src,current_age_at_bag_sec=(row['bag_ns']-src)/1e9,
    state=d.get('state'),detail=d.get('detail'),measurement_state=o.get('state'),reason=o.get('reason'),
    qualified_stamps=o.get('source_stamps_ns'),support_audit=o.get('support_audit'),
    appearance=[{k:a.get(k) for k in ('code','colour_distance','chroma_distance','contour_distance','elapsed_sec','current_source_stamp_ns','reference_source_stamp_ns','current_mask_sha256','reference_mask_sha256','dense_metrics','proposal')} for a in appearances]))
requal_acks=[row for row in acks if row['wire']['request_id']==r['request_id']]
q=np.array([row['q'] for row in joints if start<=row['source_ns']<end and row['frame_id']=='sdk_measured'])
result=dict(version=1,root_cause='Original premotion RGB identity rejected every completed arrival measurement before three-frame qualification; transport receipts were delivered promptly.',
 request=request,requalifying_acks=requal_acks,
 ack_delay_from_request_ns={participant:min(row['bag_ns'] for row in requal_acks if row['wire']['participant']==participant and row['wire']['accepted'])-r['stamp_ns'] for participant in ('remote','unknown')},
 rows=rows,completed_arrival_frames=len(rows),qualified_sources=[],
 terminal=terminal,no_qualified_request=not any(row['wire']['state']=='qualified' for row in requests),
 actual_sdk_stationary_sample_count=len(q),actual_sdk_peak_to_peak_rad=np.ptp(q,axis=0).tolist() if len(q) else None,
 restrictions=['No frame/phase/SDK/geometry/appearance threshold change is justified by this wire evidence alone.',
 'GEN13 station template is unavailable until original three-frame qualification; it cannot be used to bootstrap these failed frames.',
 'The final partially processed frame at the capture deadline is separate from nine prior identity rejections.'],
 primary_bag=str(R/'keyframes.bag'),primary_bag_sha256=hashlib.sha256((R/'keyframes.bag').read_bytes()).hexdigest(),
 no_ros_actions=True,source_modifications=False)
(out/'wire_diagnosis.json').write_text(json.dumps(result,indent=2)+'\n')
(out/'arrival_status_wire.json').write_text(json.dumps(raw,indent=2)+'\n')
(out/'measurement_wire.json').write_text(json.dumps(dict(requests=requests,acks=acks,task=task),indent=2)+'\n')
print(json.dumps({k:result[k] for k in ('root_cause','completed_arrival_frames','ack_delay_from_request_ns','no_qualified_request','actual_sdk_stationary_sample_count','actual_sdk_peak_to_peak_rad')},indent=2))
