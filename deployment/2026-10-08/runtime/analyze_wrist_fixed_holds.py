"""Analyze closed experiment bags only; no ROS node, publisher or service."""
import sys,json,math
from pathlib import Path
import numpy as np
import rosbag


def analyze(directory):
 p=Path(directory);events=[json.loads(s) for s in (p/'events.jsonl').read_text().splitlines()]
 commands=[e for e in events if e['event']=='command'];joint=commands[0]['edited_joints'][0]
 rows={k:[] for k in ('raw','accepted','sdk','control_reference')};statuses=[]
 with rosbag.Bag(str(p/'evidence.bag')) as bag:
  for topic,m,t in bag.read_messages(topics=['/alicia_d/sdk_command','/alicia_d/control_reference','/alicia_d/accepted_joint_states','/alicia_d/sdk_diagnostics','/alicia_d/actuation_status']):
   if topic.endswith('actuation_status'):statuses.append(m.data);continue
   if topic.endswith('sdk_diagnostics'):
    for status in m.status:
     d={v.key:v.value for v in status.values}
     if status.name!='alicia_d/sdk_raw_frame' or d.get('joint_evidence')!='raw_before_acceptance_filter':continue
     raw=bytes.fromhex(d['payload_hex'])
     if len(raw)!=15:continue
     q=[(int.from_bytes(raw[2*j:2*j+2],'little')-2048)*2*math.pi/4096 for j in range(6)]
     rows['raw'].append((m.header.stamp.to_nsec(),q))
   else:
    values=dict(zip(m.name,m.position));q=[values['Joint%d'%j] for j in range(1,7)]
    name='accepted' if topic.endswith('accepted_joint_states') else 'control_reference' if topic.endswith('control_reference') else 'sdk'
    rows[name].append((m.header.stamp.to_nsec(),q))
 report=dict(session=str(p),joint=joint,kind='closed_bag_fixed_target_hold_review',message_counts={k:len(v) for k,v in rows.items()},actuation_states=list(dict.fromkeys(statuses)),hold_windows=[],raw_transients=[],encoder_quantum_deg=360./4096,limits='Approximately 10 Hz raw feedback and one-word angular resolution; not a continuous mechanical vibration measurement.')
 for event in events:
  if event['event']!='settled':continue
  start=event['ros_stamp_ns'];end=start+3_000_000_000
  item=dict(label=event['label'],source_interval_ns=[start,end],channels={})
  for name,data in rows.items():
   subset=[(t,q) for t,q in data if start<=t<=end];q=np.array([v for _,v in subset]);times=np.array([t for t,_ in subset])
   changes=np.diff(q,axis=0) if len(q)>1 else np.empty((0,6))
   item['channels'][name]=dict(count=len(q),covered_sec=float((times[-1]-times[0])*1e-9) if len(times)>1 else 0.,max_gap_sec=float(np.max(np.diff(times))*1e-9) if len(times)>1 else None,joint_peak_to_peak_deg=np.degrees(np.ptp(q,axis=0)).tolist() if len(q) else None,joint_nonzero_change_counts=np.count_nonzero(np.abs(changes)>1e-10,axis=0).tolist())
  report['hold_windows'].append(item)
  command=next(e for e in commands if e['label']==event['label']);target=np.array(command['target_rad'][:6])
  refs=[(t,q) for t,q in rows['control_reference'] if t>=command['command_stamp_ns'] and np.max(np.abs(np.array(q)-target))<1e-8]
  first_target=refs[0][0] if refs else None
  if first_target is None:raise RuntimeError('No actual transmitted SDK target evidence for '+event['label'])
  raw=[(t,q[joint-1]) for t,q in rows['raw'] if first_target<=t<=end];collapsed=[]
  for t,q in raw:
   if not collapsed or abs(q-collapsed[-1][1])>1e-10:collapsed.append((t,q))
  report['raw_transients'].append(dict(label=event['label'],first_target_sdk_ns=first_target,settled_event_ns=start,raw_edited_joint_changes_after_fixed_target=[dict(after_target_sec=(t-first_target)*1e-9,position_deg=math.degrees(q)) for t,q in collapsed]))
 finish=next(e for e in events if e['event']=='completed_two_commands_same_sdk_return')
 final=next(e for e in reversed(events) if e['event']=='post_hold_observation')
 report['same_sdk_return']=dict(original_words=finish['original_words'],final_words=finish['final_words'],exact_same_words=finish['original_words']==finish['final_words'],actual_delta_from_initial_deg=final['actual_delta_deg'])
 (p/'fixed_target_hold_review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
 (p/'analyze_wrist_fixed_holds.py').write_bytes(Path(__file__).read_bytes())
 print(json.dumps(report,ensure_ascii=False))

if __name__=='__main__':
 for directory in sys.argv[1:]:analyze(directory)
