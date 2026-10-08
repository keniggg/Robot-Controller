import sys,json,math
from pathlib import Path
import numpy as np,rosbag
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path(sys.argv[1]);events=[json.loads(s) for s in (p/'events.jsonl').read_text().splitlines()];data={'sdk':[],'accepted':[],'raw':[]};states=[]
with rosbag.Bag(str(p/'evidence.bag')) as bag:
 for topic,m,t in bag.read_messages(topics=['/alicia_d/sdk_command','/alicia_d/accepted_joint_states','/alicia_d/sdk_diagnostics','/alicia_d/actuation_status']):
  if topic in ('/alicia_d/sdk_command','/alicia_d/accepted_joint_states'):
   d=dict(zip(m.name,m.position));q=[d['Joint%d'%j] for j in range(1,7)];data['sdk' if topic.endswith('sdk_command') else 'accepted'].append((m.header.stamp.to_sec(),q))
  elif topic.endswith('actuation_status'):states.append((t.to_sec(),m.data))
  else:
   for s in m.status:
    d={v.key:v.value for v in s.values}
    if s.name!='alicia_d/sdk_raw_frame' or d.get('joint_evidence')!='raw_before_acceptance_filter':continue
    b=bytes.fromhex(d['payload_hex']);q=[(int.from_bytes(b[2*j:2*j+2],'little')-2048)*2*math.pi/4096 for j in range(6)]
    if len(b)==15:data['raw'].append((m.header.stamp.to_sec(),q))
start=min(t for rows in data.values() for t,q in rows); summary={'directory':str(p),'message_counts':{k:len(v) for k,v in data.items()},'events':events,'actuation_states':list(dict.fromkeys(s for _,s in states)),'post_settle_windows':[]}
for e in events:
 if e['event']!='settled':continue
 a=e['ros_stamp_ns']*1e-9;b=a+3
 result={'label':e['label'],'from_sec':a,'to_sec':b,'channels':{}}
 for k,rows in data.items():
  selected=np.array([q for t,q in rows if a<=t<=b]);result['channels'][k]={'count':len(selected),'joint_peak_to_peak_deg':np.degrees(np.ptp(selected,axis=0)).tolist() if len(selected) else None}
 summary['post_settle_windows'].append(result)
fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
for ax,j in zip(axes,[1,2,4]):
 for k,style in [('raw',':'),('sdk','-'),('accepted','--')]:
  rows=data[k]
  if rows:ax.plot([t-start for t,q in rows],[math.degrees(q[j]) for t,q in rows],style,label=k,linewidth=1.3)
 for e in events:
  if e['event'] in ('explicit_initial_sync_once','command'):ax.axvline(e['ros_stamp_ns']*1e-9-start,color='gray',alpha=.5,linewidth=.7)
 ax.set_ylabel('J%d (deg)'%(j+1));ax.grid(alpha=.2)
axes[0].legend();axes[-1].set_xlabel('Time from first recorded sample (s)');fig.suptitle('Transmitted SDK targets and measured joint response');fig.tight_layout();fig.savefig(str(p/'response.png'),dpi=150);plt.close(fig)
(p/'motion_analysis.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n');print(json.dumps({k:v for k,v in summary.items() if k!='events'},ensure_ascii=False))
