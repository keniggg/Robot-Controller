import hashlib,json
from pathlib import Path
from collections import defaultdict,Counter
import numpy as np
NEW=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
p=NEW/'gen8_handoff_analysis/station_compute_8_1791602335463383495.jsonl'
rows=[r for line in p.open() if (r:=json.loads(line))['kind']=='detector' and r.get('state')=='ready' and r.get('source_stamp_ns',0)>=1791602438925921440 and r['received_ros_ns']<1791602529913476467]
assert len(rows)==226
base=defaultdict(list);stages=defaultdict(list);components=defaultdict(list);counts=defaultdict(list);cpucomponents=defaultdict(list)
for r in rows:
 t=r['frame_timing']
 for k in ('processing_wall_sec','processing_thread_cpu_sec','status_source_age_sec','ingress_source_age_sec','ingress_to_processing_sec','input_lock_wait_sec','processing_start_source_age_sec','exact_pair_source_age_sec','exact_pair_to_input_slot_sec'):
  if isinstance(t.get(k),(int,float)):base[k].append(t[k])
 if isinstance(t.get('processing_wall_sec'),(int,float)) and isinstance(t.get('processing_thread_cpu_sec'),(int,float)):
  base['processing_wall_minus_thread_cpu_sec'].append(t['processing_wall_sec']-t['processing_thread_cpu_sec'])
 for k,v in t.get('stage_wall_sec',{}).items():stages[k].append(v)
 for group in ('segmentation_components','rgb_components','decode_components'):
  for k,value in t.get(group,{}).items():
   records=value if isinstance(value,list) else [value]
   values=[v for v in records if isinstance(v,dict) and isinstance(v.get('wall_sec'),(int,float))]
   if values:
    name=group+'.'+k;components[name].append(sum(v['wall_sec'] for v in values));counts[name].append(len(values))
    cpucomponents[name].append(sum(v.get('thread_cpu_sec',0) for v in values))
def stats(v,scale=1000):
 a=np.array(v,dtype=float)*scale
 return dict(n=len(a),min=float(a.min()),p50=float(np.quantile(a,.5)),p90=float(np.quantile(a,.9)),p99=float(np.quantile(a,.99)),max=float(a.max()),mean=float(a.mean()))
stamps=np.array([r['source_stamp_ns'] for r in rows],dtype=np.int64)
seq=np.array([r['frame_timing']['color_seq'] for r in rows if 'color_seq' in r['frame_timing']],dtype=np.int64)
component_report={k:dict(wall_ms=stats(v),thread_cpu_ms=stats(cpucomponents[k]),calls_per_frame=stats(counts[k],1)) for k,v in components.items()}
report=dict(count=len(rows),full_frame_timing_count=sum(bool(r['frame_timing'].get('stage_wall_sec')) for r in rows),source_observer_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),
 selected_source_intervals_ms=stats(np.diff(stamps)/1e9),selected_color_seq_step=stats(np.diff(seq),1),
 scalar_ms={k:stats(v) for k,v in base.items()},stage_wall_ms={k:stats(v) for k,v in stages.items()},
 components=component_report,
 dominant_stage_counts=dict(Counter(max(r['frame_timing']['stage_wall_sec'],key=r['frame_timing']['stage_wall_sec'].get) for r in rows if r['frame_timing'].get('stage_wall_sec'))),
 limitations=['Stage wall timings form the instrumented top-level partition; nested component timings overlap parent stages and cannot be summed across names.',
 'Approximately 400ms interval is between frames selected for processing, not a claim that the camera itself captures at 2.5Hz.',
 'Thread CPU versus wall time describes this measured thread. It is not a profiler stack attribution for every unknown interval.',
 'Recorded parameter/context timings do not expose the duration or count of every possible ROS RPC; no uninstrumented call is assigned a guessed duration.'])
out=NEW/'gen8_handoff_analysis/processing_timing_distribution.json';out.write_text(json.dumps(report,indent=2,sort_keys=True)+'\n')
top=sorted(component_report.items(),key=lambda kv:kv[1]['wall_ms']['mean'],reverse=True)[:9]
print(json.dumps(dict(path=str(out),sha256=hashlib.sha256(out.read_bytes()).hexdigest(),count=len(rows),scalar={k:v for k,v in report['scalar_ms'].items() if k in ('processing_wall_sec','processing_thread_cpu_sec','status_source_age_sec','ingress_to_processing_sec','input_lock_wait_sec')},stages=report['stage_wall_ms'],top_components=top,color_seq_step=report['selected_color_seq_step'],dominant=report['dominant_stage_counts'])))
