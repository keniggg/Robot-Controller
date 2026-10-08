#!/usr/bin/env python3
"""Closed-bag/raw SDK read-only audit; no ROS node, service or publisher."""
from pathlib import Path
import hashlib,json,math,bisect
import rosbag
import numpy as np
H=Path(__file__).resolve().parent;ROOT=H.parent
R=Path(json.loads((H/'latest_restart_grasp.json').read_text())['directory'])
A=R.parent;S=A/'bounded_shoulder_elbow_validation_20260929T025136Z_167696'
P=A/'fixed_board_passive_20260929T025403Z'
O=R/'j4_held_axis_feedback_audit';O.mkdir(exist_ok=True)
N=['Joint%d'%i for i in range(1,7)]
def words(q):return np.rint(np.degrees(q[:6])/360*4096+2048).astype(int).tolist()
def stamp(m):return m.header.stamp.to_nsec()
def read(session):
 rows=dict(raw=[],accepted=[],sdk=[],reference=[],commands=[],epochs=[],modes=[],status=[],protection=[])
 topics=['/alicia_d/sdk_diagnostics','/alicia_d/accepted_joint_states','/alicia_d/sdk_command','/alicia_d/control_reference','/joint_commands','/alicia_d/control_reference_epoch','/gui/joint_direct_mode','/alicia_d/actuation_status','/alicia_d/protection_latched']
 with rosbag.Bag(str(session/'evidence.bag')) as bag:
  for topic,m,t in bag.read_messages(topics=topics):
   rec=t.to_nsec()
   if topic=='/alicia_d/sdk_diagnostics':
    for st in m.status:
     fields={v.key:v.value for v in st.values}
     if fields.get('command_function_hex')!='06 00':continue
     payload=bytes.fromhex(fields['payload_hex'])
     raw=[int.from_bytes(payload[i:i+2],'little') for i in range(0,12,2)]
     rows['raw'].append(dict(source_ns=stamp(m),receipt_ns=rec,words=raw,run_status=payload[14],payload_hex=fields['payload_hex'],timestamp_kind=fields.get('timestamp_kind'),transport=fields.get('transport')))
   elif topic in ['/alicia_d/accepted_joint_states','/alicia_d/sdk_command','/alicia_d/control_reference','/joint_commands']:
    key={'/alicia_d/accepted_joint_states':'accepted','/alicia_d/sdk_command':'sdk','/alicia_d/control_reference':'reference','/joint_commands':'commands'}[topic]
    q=[dict(zip(m.name,m.position))[n] for n in N]
    row=dict(source_ns=stamp(m),receipt_ns=rec,words=words(q),q_rad=q,frame_id=m.header.frame_id,names=list(m.name))
    if key=='commands':row['effort']=list(m.effort);row['callerid']=getattr(m,'_connection_header',{}).get('callerid')
    rows[key].append(row)
   elif topic.endswith('control_reference_epoch'):rows['epochs'].append(m.stamp.to_nsec())
   elif topic=='/gui/joint_direct_mode':rows['modes'].append(dict(receipt_ns=rec,value=m.data))
   elif topic.endswith('actuation_status'):rows['status'].append(dict(receipt_ns=rec,value=m.data))
   elif topic.endswith('protection_latched'):rows['protection'].append(dict(receipt_ns=rec,value=m.data))
 return rows
r=read(S)
rawtimes=[row['source_ns'] for row in r['raw']]
pairs=[]
for a in r['accepted']:
 idx=bisect.bisect_right(rawtimes,a['source_ns'])-1
 if idx<0:pairs.append(dict(accepted=a,matched=False));continue
 raw=r['raw'][idx];delay=a['source_ns']-raw['source_ns']
 pairs.append(dict(accepted_source_ns=a['source_ns'],raw_source_ns=raw['source_ns'],parser_separation_us=delay/1000,matched=(a['words']==raw['words'] and 0<=delay<=2_000_000),words=a['words']))
command=r['commands'][-1];t0=command['source_ns'];near=[]
for a in r['accepted']:
 if not t0-600_000_000<=a['source_ns']<=t0+2_000_000_000:continue
 raw=r['raw'][bisect.bisect_right(rawtimes,a['source_ns'])-1]
 near.append(dict(relative_command_ms=(a['source_ns']-t0)/1e6,accepted_source_ns=a['source_ns'],receipt_age_ms=(a['receipt_ns']-a['source_ns'])/1e6,
  raw_source_ns=raw['source_ns'],raw_parse_to_accepted_us=(a['source_ns']-raw['source_ns'])/1000,raw_words=raw['words'],accepted_words=a['words'],joint3_deg=np.degrees(a['q_rad'][2]),joint4_deg=np.degrees(a['q_rad'][3]),payload_hex=raw['payload_hex'],run_status=raw['run_status']))
by_ns={row['source_ns']:row for row in r['reference']}
reference_matching=[row['source_ns'] in by_ns and row['words']==by_ns[row['source_ns']]['words'] for row in r['sdk']]
unique={key:{f'J{j+1}':sorted(set(row['words'][j] for row in r[key])) for j in range(6)} for key in ('sdk','reference','accepted','raw')}
phase=[]
for i,c in enumerate(r['commands']):
 stop=r['commands'][i+1]['source_ns'] if i+1<len(r['commands']) else 10**30
 q0=np.array([row for row in r['accepted'] if row['source_ns']<c['source_ns']][-1]['q_rad'])
 a=[row for row in r['accepted'] if c['source_ns']<=row['source_ns']<stop]
 dif=np.degrees(np.array([row['q_rad'] for row in a])-q0)
 phase.append(dict(command_axis=np.flatnonzero(np.array(c['effort'])>.5).tolist(),count=len(a),duration_after_command_sec=(a[-1]['source_ns']-c['source_ns'])/1e9,maximum_absolute_delta_deg=np.max(abs(dif),axis=0).tolist(),last_delta_deg=dif[-1].tolist(),
  all_command_words=[c['words']],first_sdk_after_command=next((row for row in r['sdk'] if row['source_ns']>c['source_ns']),None)))
# Separate historical same-epoch J4 experiment provides channel-specific response.
previous=A/'bounded_wrist_async_validation_20260929T024454Z_149611';pr=read(previous)
control=[]
for i,c in enumerate(pr['commands']):
 stop=pr['commands'][i+1]['source_ns'] if i+1<len(pr['commands']) else 10**30
 before=[v for v in pr['accepted'] if v['source_ns']<c['source_ns']][-1]
 after=[v for v in pr['accepted'] if c['source_ns']<=v['source_ns']<stop]
 control.append(dict(command_axis=np.flatnonzero(np.array(c['effort'])>.5).tolist(),sdk_words=c['words'],before_words=before['words'],last_words=after[-1]['words'],last_delta_words=(np.array(after[-1]['words'])-before['words']).tolist()))
rawgap=np.diff(rawtimes)/1e6;acceptedgap=np.diff([x['source_ns'] for x in r['accepted']])/1e6
report=dict(source_session=str(S),source_bag_sha256=hashlib.sha256((S/'evidence.bag').read_bytes()).hexdigest(),diagnostic_only=True,hardware_commands_sent_by_audit=0,
 raw_feedback_count=len(r['raw']),accepted_count=len(r['accepted']),raw_to_accepted_matching_count=sum(p['matched'] for p in pairs),raw_to_accepted_max_parse_separation_us=max(p.get('parser_separation_us',0) for p in pairs),
 raw_joint_status_values=sorted(set(x['run_status'] for x in r['raw'])),source_gap_ms=dict(raw_max=float(max(rawgap)),accepted_max=float(max(acceptedgap)),raw_median=float(np.median(rawgap))),
 sdk_count=len(r['sdk']),reference_count=len(r['reference']),identical_sdk_reference_same_source_count=sum(reference_matching),unique_words=unique,
 command_rows=r['commands'],epoch_values=r['epochs'],gui_mode=r['modes'],actuation_status_values=sorted(set(x['value'] for x in r['status'])),protection_values=sorted(set(x['value'] for x in r['protection'])),
 accepted_near_J3=near,phase_summaries=phase,same_epoch_previous_J4_control=dict(session=str(previous),epoch_values=pr['epochs'],commands=control),
 limits=['Raw feedback and accepted stamps are host parse time, not MCU acquisition time.', 'Reported SDK endpoints establish successful host write, not independent firmware/servo goal telemetry.', 'CRC-valid raw reported joint position does not independently prove physical motion or exclude firmware sensor error.', 'Feedback is nominal 10Hz; dynamics between frames and post-fault missing window remain unobserved.', 'No calibration fit, command compensation, boundary relaxation or production code changes were performed.'])
for path in ('src/real-arm/alicia_d_driver/src/alicia_d_driver_node.cpp','src/real-arm/alicia_d_driver/include/alicia_d_driver/sdk_position_codec.hpp','src/real-arm/alicia_d_driver/include/alicia_d_driver/gui_direct_hold.hpp'):
 report.setdefault('source_sha256',{})[path]=hashlib.sha256((ROOT/path).read_bytes()).hexdigest()
(O/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');(O/'raw_accepted_pairing.json').write_text(json.dumps(pairs,indent=2)+'\n')
(O/'audit_j4_held_axis_feedback.py').write_bytes(Path(__file__).read_bytes())
print(json.dumps(dict(output=str(O),matching=report['raw_to_accepted_matching_count'],accepted=report['accepted_count'],raw=report['raw_feedback_count'],pair_max_us=report['raw_to_accepted_max_parse_separation_us'],unique_J4={key:val['J4'] for key,val in unique.items()},near=near,previous_control=control),indent=2))
