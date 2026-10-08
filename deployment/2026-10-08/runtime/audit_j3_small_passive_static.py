#!/usr/bin/env python3
"""Analyze an already closed no-command monitor. Never creates a ROS node."""
from pathlib import Path
import json,hashlib,collections,bisect
import numpy as np,rosbag
H=Path(__file__).resolve().parent;R=Path(json.load(open(H/'latest_restart_grasp.json'))['directory']);S=R.parent/'j3_small_passive_monitor_20260929T031950Z';O=R/'small_j3_static_monitor_audit';O.mkdir(exist_ok=True)
q=[];sdk=[];raw=[];status=collections.Counter();protection=collections.Counter();modes=collections.Counter();commands=[];epochs=[];temps=[]
with rosbag.Bag(str(S/'evidence.bag')) as b:
 for topic,m,t in b.read_messages():
  if topic in ('/alicia_d/accepted_joint_states','/alicia_d/control_reference','/alicia_d/sdk_command'):
   d=dict(zip(m.name,m.position));a=[d['Joint%d'%i] for i in range(1,7)];row=dict(source_ns=m.header.stamp.to_nsec(),receipt_ns=t.to_nsec(),seq=m.header.seq,words=np.rint(np.degrees(a)/360*4096+2048).astype(int).tolist())
   if topic.endswith('accepted_joint_states'):q.append(row)
   elif topic.endswith('sdk_command'):sdk.append(row)
  elif topic=='/joint_commands':commands.append(dict(receipt_ns=t.to_nsec(),source_ns=m.header.stamp.to_nsec(),frame_id=m.header.frame_id))
  elif topic=='/alicia_d/control_reference_epoch':epochs.append(m.stamp.to_nsec())
  elif topic=='/gui/joint_direct_mode':modes[str(m.data)]+=1
  elif topic=='/alicia_d/protection_latched':protection[str(m.data)]+=1
  elif topic=='/alicia_d/actuation_status':status[m.data]+=1
  elif topic=='/alicia_d/sdk_diagnostics':
   for st in m.status:
    f={v.key:v.value for v in st.values}
    if f.get('command_function_hex')=='06 00':
     p=bytes.fromhex(f['payload_hex']);raw.append(dict(source_ns=m.header.stamp.to_nsec(),words=[int.from_bytes(p[i:i+2],'little') for i in range(0,12,2)],run_status=p[14]))
    elif f.get('command_function_hex')=='06 01':temps.append(list(bytes.fromhex(f['payload_hex'])))
qt=np.asarray([x['source_ns'] for x in q]);qw=np.asarray([x['words'] for x in q]);rt=np.asarray([x['source_ns'] for x in raw]);pair=[]
for x in q:
 idx=bisect.bisect_right(rt,x['source_ns'])-1
 pair.append(idx>=0 and x['words']==raw[idx]['words'] and x['source_ns']-raw[idx]['source_ns']<=2000000)
changes=[];last=None
for x in q:
 if x['words'][3]!=last:changes.append(dict(source_ns=x['source_ns'],elapsed_sec=(x['source_ns']-int(qt[0]))/1e9,J4_word=x['words'][3],J4_deg=(x['words'][3]-2048)*360/4096));last=x['words'][3]
summary=dict(session=str(S),interpretation='entirely static recording BEFORE new +8 trial; not post-motion coverage',count=len(q),duration_seconds=float(qt[-1]-qt[0])/1e9,commands=commands,raw_count=len(raw),raw_accepted_matched=sum(pair),sdk_count=len(sdk),epoch_values=epochs,
 accepted_sequence_gaps=sum(y['seq']!=x['seq']+1 for x,y in zip(q,q[1:])),maximum_accepted_source_gap_ms=float(max(np.diff(qt)))/1e6,
 per_joint_word_counts={f'J{j+1}':dict(collections.Counter(int(v) for v in qw[:,j])) for j in range(6)},
 joint_peak_to_peak_deg=((qw.max(0)-qw.min(0))*360/4096).tolist(),J4_changes=changes,J4_max_absolute_deviation_old_anchor_deg=float(np.max(abs(qw[:,3]-2049)))*360/4096,old_J4_bound_deg=.4,old_J4_bound_violation_count=int(np.sum(abs(qw[:,3]-2049)*360/4096>.4)),
 SDK_word_vectors=[list(v) for v in sorted(set(tuple(x['words']) for x in sdk))],run_status_counts=dict(collections.Counter(x['run_status'] for x in raw)),actuation_status_counts=dict(status),protection_counts=dict(protection),gui_mode_counts=dict(modes),
 temperature_raw_min=min(min(v) for v in temps) if temps else None,temperature_raw_max=max(max(v) for v in temps) if temps else None,
 hardware_actions_by_analysis=0,terminal=json.loads((S/'terminal.json').read_text()))
(O/'report.json').write_text(json.dumps(summary,indent=2)+'\n');(O/'audit_j3_small_passive_static.py').write_bytes(Path(__file__).read_bytes())
print(json.dumps(summary,indent=2))
