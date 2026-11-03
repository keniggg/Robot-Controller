#!/usr/bin/env python3
"""Read only already closed +8 diagnostic/passive bags; preserve prior failure."""
from pathlib import Path
import json,bisect,collections,hashlib,math
import numpy as np,rosbag
H=Path(__file__).resolve().parent;R=Path(json.load(open(H/'latest_restart_grasp.json'))['directory']);A=R.parent
S=A/'bounded_j3_small_response_20260929T032235Z_244644';O=R/'small_j3_raw_response_audit';O.mkdir(exist_ok=True)
Q=360/4096;ANCHOR=2049;NOW0=1790652165738847732

def read(directory):
 if not (directory/'terminal.json').exists():raise RuntimeError('bag not independently marked closed: '+str(directory))
 rows=dict(accepted=[],sdk=[],reference=[],raw=[],temperature=[],commands=[],events=[],epoch=[],status=collections.Counter(),protection=collections.Counter())
 topics=['/alicia_d/accepted_joint_states','/alicia_d/sdk_command','/alicia_d/control_reference','/alicia_d/sdk_diagnostics','/joint_commands','/bounded_response/events','/alicia_d/control_reference_epoch','/alicia_d/actuation_status','/alicia_d/protection_latched']
 with rosbag.Bag(str(directory/'evidence.bag')) as bag:
  for topic,m,t in bag.read_messages(topics=topics):
   if topic in ('/alicia_d/accepted_joint_states','/alicia_d/sdk_command','/alicia_d/control_reference','/joint_commands'):
    key={'/alicia_d/accepted_joint_states':'accepted','/alicia_d/sdk_command':'sdk','/alicia_d/control_reference':'reference','/joint_commands':'commands'}[topic]
    p=dict(zip(m.name,m.position));words=np.rint(np.degrees([p['Joint%d'%i] for i in range(1,7)])/Q+2048).astype(int).tolist()
    row=dict(source_ns=m.header.stamp.to_nsec(),receipt_ns=t.to_nsec(),seq=m.header.seq,words=words,frame_id=m.header.frame_id)
    if key=='commands':row['effort']=list(m.effort)
    rows[key].append(row)
   elif topic=='/alicia_d/sdk_diagnostics':
    for st in m.status:
     f={v.key:v.value for v in st.values}
     if f.get('command_function_hex')=='06 00':
      p=bytes.fromhex(f['payload_hex']);rows['raw'].append(dict(source_ns=m.header.stamp.to_nsec(),words=[int.from_bytes(p[i:i+2],'little') for i in range(0,12,2)],run_status=p[14],payload_hex=f['payload_hex']))
     elif f.get('command_function_hex')=='06 01' or 'raw_max_c' in f:rows['temperature'].append(dict(source_ns=m.header.stamp.to_nsec(),fields=f))
   elif topic=='/bounded_response/events':rows['events'].append(json.loads(m.data))
   elif topic.endswith('control_reference_epoch'):rows['epoch'].append(m.stamp.to_nsec())
   elif topic.endswith('actuation_status'):rows['status'][m.data]+=1
   elif topic.endswith('protection_latched'):rows['protection'][str(m.data)]+=1
 return rows

def file_sha256(path):
 digest=hashlib.sha256()
 with path.open('rb') as stream:
  for block in iter(lambda:stream.read(1048576),b''):digest.update(block)
 return digest.hexdigest()

def audit(directory,t0):
 r=read(directory);at=np.asarray([a['source_ns'] for a in r['accepted']]);w=np.asarray([a['words'] for a in r['accepted']]);rt=[a['source_ns'] for a in r['raw']]
 pre=[a for a in r['accepted'] if a['source_ns']<t0];start=np.asarray(pre[-1]['words'] if pre else r['accepted'][0]['words']);post=np.asarray([a['words'] for a in r['accepted'] if a['source_ns']>=t0]);pair=[];timeline=[]
 for a in r['accepted']:
  i=bisect.bisect_right(rt,a['source_ns'])-1
  if i>=0:
   b=r['raw'][i];pair.append(dict(accepted_source_ns=a['source_ns'],raw_source_ns=b['source_ns'],same_words=a['words']==b['words'],parse_gap_us=(a['source_ns']-b['source_ns'])/1000))
  if t0-500_000_000<=a['source_ns']<=t0+8_000_000_000:
   timeline.append(dict(relative_command_ms=(a['source_ns']-t0)/1e6,words=a['words'],delta_start_deg=((np.asarray(a['words'])-start)*Q).tolist(),J4_from_old_anchor_deg=(a['words'][3]-ANCHOR)*Q,receipt_age_ms=(a['receipt_ns']-a['source_ns'])/1e6))
 transitions=[];last=None
 for a in r['accepted']:
  if a['words'][2:4]!=last:
   transitions.append(dict(relative_command_ms=(a['source_ns']-t0)/1e6,J3_word=a['words'][2],J4_word=a['words'][3]));last=a['words'][2:4]
 # Quantized code constancy, not proof of zero physical velocity.
 different=np.flatnonzero(np.any(w!=w[-1],axis=1));constant_start=int(different[-1]+1) if len(different) else 0
 nonzero_steps={}
 for j in (2,3):
  steps=np.diff(w[:,j]);signs=np.sign(steps[steps!=0])
  nonzero_steps['J%d'%(j+1)]=dict(nonzero_step_words=steps[steps!=0].tolist(),direction_reversals=int(np.sum(signs[1:]!=signs[:-1])))
 result=dict(session=str(directory),source_bag_sha256=file_sha256(directory/'evidence.bag'),accepted_count=len(w),raw_count=len(r['raw']),raw_accepted_same_words=sum(x['same_words'] for x in pair),raw_accepted_parse_gap_max_us=max(x['parse_gap_us'] for x in pair),raw_pairing_over_2ms=[x for x in pair if x['parse_gap_us']>2000],
 source_first_ns=int(at[0]),source_last_ns=int(at[-1]),relative_command_span_s=[(int(at[0])-t0)/1e9,(int(at[-1])-t0)/1e9],maximum_accepted_gap_ms=float(np.max(np.diff(at)))/1e6,
 accepted_sequence_gaps=sum(b['seq']!=a['seq']+1 for a,b in zip(r['accepted'],r['accepted'][1:])),SDK_sequence_gaps=sum(b['seq']!=a['seq']+1 for a,b in zip(r['sdk'],r['sdk'][1:])),SDK_unique_words=[list(x) for x in sorted(set(tuple(a['words']) for a in r['sdk']))],
 start_words=start.tolist(),post_command_max_abs_delta_words=np.max(abs(post-start),axis=0).tolist() if len(post) else None,post_command_max_abs_delta_deg=(np.max(abs(post-start),axis=0)*Q).tolist() if len(post) else None,last_minus_start_deg=((w[-1]-start)*Q).tolist(),
 J4_min_max_word=[int(w[:,3].min()),int(w[:,3].max())],J4_max_abs_old_anchor_deg=float(np.max(abs(w[:,3]-ANCHOR)))*Q,old_J4_anchor_bound_violations=int(np.sum(abs(w[:,3]-ANCHOR)*Q>.4)),
 other_held_axis_bound_violations=int(np.sum(np.any(abs(w[:,[0,1,3,4,5]]-start[[0,1,3,4,5]])*Q>.4,axis=1))),
 commands=r['commands'],events=r['events'],epochs=r['epoch'],status_counts=dict(r['status']),protection_counts=dict(r['protection']),raw_run_status_counts=dict(collections.Counter(x['run_status'] for x in r['raw'])),
 timeline_first8s=timeline,all_J3_J4_transitions=transitions,temperature=r['temperature'],terminal=json.loads((directory/'terminal.json').read_text()),
 last_words=w[-1].tolist(),final_joint_code_constant_start_ns=int(at[constant_start]),final_joint_code_constant_start_relative_command_ms=(int(at[constant_start])-t0)/1e6,
 final_joint_code_constant_recorded_span_sec=(int(at[-1])-int(at[constant_start]))/1e9,final_joint_code_constant_sample_count=int(len(w)-constant_start),
 nonzero_step_directions=nonzero_steps,sdk_count=len(r['sdk']),reference_count=len(r['reference']),
 interpretation='Only the different +8-word input; original +22 trial remains failed. Host parse stamps, not MCU acquisition clock. Quantized feedback cannot reveal sub-count motion.')
 return result

baseline=audit(A/'j3_small_passive_monitor_20260929T031950Z',NOW0)
baseline['interpretation']='Entirely pretrial static baseline; no commands and not post-motion coverage.'
(O/'baseline_report.json').write_text(json.dumps(baseline,indent=2)+'\n')
motion=audit(S,NOW0);(O/'motion_report.json').write_text(json.dumps(motion,indent=2)+'\n')
P=A/'j3_small_passive_monitor_20260929T032219Z'
if (P/'terminal.json').exists():
 independent=audit(P,NOW0);(O/'independent_report.json').write_text(json.dumps(independent,indent=2)+'\n')
 print('independent',json.dumps({k:independent[k] for k in ['accepted_count','raw_accepted_same_words','relative_command_span_s','J4_min_max_word','old_J4_anchor_bound_violations','all_J3_J4_transitions','post_command_max_abs_delta_deg','raw_run_status_counts']}))
else:print('independent recorder not yet closed; deliberately not read')
(O/'audit_j3_small_trial.py').write_bytes(Path(__file__).read_bytes())
print('motion',json.dumps({k:motion[k] for k in ['accepted_count','raw_accepted_same_words','relative_command_span_s','SDK_unique_words','start_words','J4_min_max_word','old_J4_anchor_bound_violations','all_J3_J4_transitions','post_command_max_abs_delta_deg','raw_run_status_counts','terminal']},indent=2))
