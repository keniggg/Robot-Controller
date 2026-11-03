#!/usr/bin/env python3
"""Offline closed-bag audit; never connects to ROS or device."""
import collections,json,math,pathlib,statistics
import rosbag
BASE=pathlib.Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
OUT=BASE/'ros_log/feedforward_speed_grasp_20260929T140844Z'
CASES={
 'gen17':('20260929T140308.991426Z_04dc02f4d2354aaba3c85d53798c5a6b',1790690593.,1790690599.),
 'gen18':('20260929T140845.412565Z_869bb8bd62ed4fe4ac81b8c1a5572b7b',1790690982.,1790690989.),
}
TOPICS=['/alicia_d/sdk_diagnostics','/alicia_d/accepted_joint_states','/tf','/joint_states','/alicia_d/sdk_command','/alicia_controller/state','/rosout_agg','/grasp_record/camera_runtime_metadata','/supervisor/camera/color/image_raw']
def stamp(m,t):
 if hasattr(m,'header'): return m.header.stamp.to_sec()
 if hasattr(m,'transforms') and m.transforms:return m.transforms[0].header.stamp.to_sec()
 return t.to_sec()
def summary(rows):
 ds=[b['stamp']-a['stamp'] for a,b in zip(rows,rows[1:])]
 delays=[r['bag_time']-r['stamp'] for r in rows]
 return {'n':len(rows),'median_gap_s':statistics.median(ds) if ds else None,'largest_gaps':[{'gap_s':b['stamp']-a['stamp'],'from':a,'to':b} for a,b in sorted(zip(rows,rows[1:]),key=lambda ab:ab[1]['stamp']-ab[0]['stamp'],reverse=True)[:8]],'delivery_delay_median_s':statistics.median(delays) if delays else None,'delivery_delay_max_s':max(delays) if delays else None,'seq_gaps':[{'from':a,'to':b,'missing':b['seq']-a['seq']-1} for a,b in zip(rows,rows[1:]) if 'seq'in a and b['seq']!=a['seq']+1][:30]}
result={}
for label,(rec,start,end) in CASES.items():
 p=BASE/'grasp_records'/rec/'keyframes.bag'
 data=collections.defaultdict(list);logs=[];metadata=[]
 with rosbag.Bag(str(p)) as bag:
  for topic,m,t in bag.read_messages(topics=TOPICS):
   s=stamp(m,t);r={'stamp':s,'bag_time':t.to_sec()}
   if topic=='/rosout_agg':
    if start-3<=s<=end+3 or ('driver' in m.name and any(x in m.msg.lower() for x in ['reject','invalid','corrupt','serial read','frame','timeout'])):
     logs.append({'stamp':s,'bag_time':t.to_sec(),'node':m.name,'level':m.level,'msg':m.msg})
    continue
   if topic=='/grasp_record/camera_runtime_metadata':
    metadata.append({'bag_time':t.to_sec(),'data':m.data});continue
   if hasattr(m,'header'):r['seq']=m.header.seq
   if topic=='/alicia_d/sdk_diagnostics':
    for status in m.status:
     vals={v.key:v.value for v in status.values}
     r.update({'diagnostic':status.name,'values':vals})
   elif topic in ['/alicia_d/accepted_joint_states','/alicia_d/sdk_command']:
    r['position']=list(m.position)
   elif topic=='/joint_states':
    r['frame']=m.header.frame_id
   elif topic=='/tf':
    r['frames']=[tr.child_frame_id for tr in m.transforms]
   data[topic].append(r)
 raw=data['/alicia_d/sdk_diagnostics'];accepted=data['/alicia_d/accepted_joint_states']
 rawj=[r for r in raw if r['values'].get('command_function_hex')=='06 00']
 paired=[];unmatched=[]
 # Associate accepted stamps with the raw parser interval, then verify decoded positions.
 # Both source timestamps are host parser timestamps; neither is a wire timestamp.
 for idx,r in enumerate(rawj):
  next_stamp=rawj[idx+1]['stamp'] if idx+1<len(rawj) else float('inf')
  candidates=[a for a in accepted if r['stamp']<=a['stamp']<next_stamp]
  words=[]
  payload=bytes.fromhex(r['values'].get('payload_hex',''))
  if len(payload)>=12: words=[payload[i]+256*payload[i+1] for i in range(0,12,2)]
  # SDK decoder verified separately: (word-2048)/4096*2*pi.
  rr={'raw':r,'raw_words':words,'accepted_candidates':candidates}
  if len(candidates)==1:
   a=candidates[0];rr['parse_to_accepted_s']=a['stamp']-r['stamp'];rr['accepted_to_bag_s']=a['bag_time']-a['stamp'];rr['decoded_positions_match']=len(words)==6 and max(abs(x-y) for x,y in zip(a['position'],[(-180+min(4095,w)/4096*360)*math.pi/180 for w in words]))<1e-12;paired.append(rr)
  else:unmatched.append(rr)
 window={k:[r for r in rs if start<=r['stamp']<=end] for k,rs in data.items()}
 result[label]={'path':str(p),'window':[start,end],'summary':{k:summary(rs) for k,rs in data.items()},'window_summary':{k:summary(rs) for k,rs in window.items()},'raw_joint_counts':{'raw':len(rawj),'accepted':len(accepted),'paired':len(paired),'unmatched':len(unmatched)},'raw_joint_window':[r for r in rawj if start<=r['stamp']<=end],'accepted_window':window['/alicia_d/accepted_joint_states'],'tf_window':window['/tf'],'pair_largest_latency':sorted(paired,key=lambda r:r['parse_to_accepted_s'],reverse=True)[:15],'unmatched_raw':unmatched,'window_logs':logs,'camera_metadata':metadata}
 print(label,json.dumps(result[label]['raw_joint_counts']),flush=True)
(OUT/'feedback_stall_audit_gen17_gen18.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
print('WROTE',OUT/'feedback_stall_audit_gen17_gen18.json',flush=True)
