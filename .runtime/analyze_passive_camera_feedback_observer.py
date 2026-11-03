#!/usr/bin/env python3
"""Offline correlation of passive bounded observation; no causal claims."""
import argparse,bisect,collections,json,pathlib,statistics
p=argparse.ArgumentParser();p.add_argument('directory');a=p.parse_args();root=pathlib.Path(a.directory)
topics=json.loads((root/'topics.json').read_text());threads=json.loads((root/'thread_samples.json').read_text());resources=json.loads((root/'resource_samples.json').read_text());summary=json.loads((root/'summary.json').read_text());logs=json.loads((root/'camera_logs.json').read_text())
if (root/'root_event.json').exists():events=json.loads((root/'root_event.json').read_text())
else:events={}
start=summary['ready']['ready_wall'];end=summary['ended_wall'];tw=[r['wall'] for r in threads];rw=[r['wall'] for r in resources]

def between_bracket(arr,stamps,lo,hi):
 if not arr:return []
 i=max(0,bisect.bisect_right(stamps,lo)-1);j=min(len(arr)-1,bisect.bisect_left(stamps,hi))
 return arr[i:j+1]

def task_deltas(lo,hi):
 sampled=between_bracket(threads,tw,lo,hi);group=collections.defaultdict(list)
 for r in sampled:
  for t in r['tasks']:group[(t['pid'],t['tid'])].append((r,t))
 result={}
 for (pid,tid),entries in group.items():
  first,ft=entries[0];last,lt=entries[-1];out={'sample_count':len(entries),'bracket_wall':[first['wall'],last['wall']],'state_counts':dict(collections.Counter(e[1].get('state','missing') for e in entries)),'wchan_counts':dict(collections.Counter(str(e[1].get('wchan','missing')) for e in entries))}
  for k in ['runtime_ns','runqueue_wait_ns','timeslices','utime_ticks','stime_ticks']:
   if k in ft and k in lt:out[k+'_delta']=lt[k]-ft[k]
  result[f'{pid}/{tid}']=out
 return {'interval_wall':[lo,hi],'bracket_sample_count':len(sampled),'max_actual_sample_interval_sec':max((r['actual_interval_sec'] or 0 for r in sampled),default=None),'tasks':result}

def psi_totals(x):
 out={}
 for kind,raw in x.items():
  if not isinstance(raw,str):continue
  for line in raw.splitlines():
   name,*fields=line.split();vals=dict(f.split('=') for f in fields);out[kind+'/'+name]=int(vals['total'])
 return out

def res_deltas(lo,hi):
 ss=between_bracket(resources,rw,lo,hi)
 if len(ss)<2:return {'count':len(ss)}
 f,l=ss[0],ss[-1];fp,lp=psi_totals(f['pressure']),psi_totals(l['pressure']);cpu={k:l['cpu'].get(k,0)-v for k,v in f['cpu'].items()};vm={k:l['vmstat'].get(k,0)-v for k,v in f['vmstat'].items() if k not in ['nr_dirty','nr_writeback','nr_free_pages']};usb={}
 for k in ['runtime_active_time','runtime_suspended_time']:
  try:usb[k+'_delta_ms']=int(l['usb4_1_power'][k])-int(f['usb4_1_power'][k])
  except (ValueError,TypeError):usb[k+'_delta_ms']=None
 return {'count':len(ss),'bracket_wall':[f['wall'],l['wall']],'psi_total_delta_us':{k:lp.get(k,0)-v for k,v in fp.items()},'vmstat_deltas':vm,'cpu_tick_deltas':cpu,'usb_power':{**usb,'runtime_status_counts':dict(collections.Counter(str(r['usb4_1_power']['runtime_status']) for r in ss))},'nr_free_pages_range':[min(r['vmstat'].get('nr_free_pages',0) for r in ss),max(r['vmstat'].get('nr_free_pages',0) for r in ss)],'max_resource_sample_interval_sec':max(b['monotonic']-a['monotonic'] for a,b in zip(ss,ss[1:]))}

def stream_stats(rs,lo,hi):
 ss=[r for r in rs if lo<=r['arrival_wall']<hi]
 if len(ss)<2:return {'samples':len(ss),'duration_sec':hi-lo}
 sd=[b['source_stamp']-a['source_stamp'] for a,b in zip(ss,ss[1:])];ad=[b['monotonic']-a['monotonic'] for a,b in zip(ss,ss[1:])];delay=[r['arrival_wall']-r['source_stamp'] for r in ss]
 return {'samples':len(ss),'duration_sec':hi-lo,'source_max_sec':max(sd),'source_median_sec':statistics.median(sd),'source_ge250ms':sum(v>=.25 for v in sd),'receipt_max_sec':max(ad),'receipt_ge250ms':sum(v>=.25 for v in ad),'arrival_minus_source_max_sec':max(delay),'arrival_minus_source_median_sec':statistics.median(delay)}

streams=dict(topics);streams['/alicia_d/sdk_diagnostics:joint_06_00']=[r for r in topics['/alicia_d/sdk_diagnostics'] if any(d.get('command_function_hex')=='06 00' for d in r.get('diagnostics',[]))]
segments={'whole':[start,end]}
elist=events.get('events',[]) if isinstance(events,dict) else []
valid=[]
for ev in elist:
 wall=ev.get('wall_time',ev.get('wall'))
 if wall is not None and start<float(wall)<end:valid.append((float(wall),ev))
valid.sort()
if valid:
 segments['before_first_root_event']=[start,valid[0][0]]
 segments['after_last_root_event']=[valid[-1][0],end]
segment_stats={name:{'interval_wall':bounds,'streams':{k:stream_stats(rs,*bounds) for k,rs in streams.items()},'thread_deltas':task_deltas(*bounds),'resource_deltas':res_deltas(*bounds)} for name,bounds in segments.items()}
gaps=[]
for topic,rs in streams.items():
 if topic=='/alicia_d/sdk_diagnostics':continue
 for prev,cur in zip(rs,rs[1:]):
  for typ,dt,lo,hi in [('source',cur['source_stamp']-prev['source_stamp'],prev['source_stamp'],cur['source_stamp']),('receipt',cur['monotonic']-prev['monotonic'],prev['arrival_wall'],cur['arrival_wall'])]:
   if dt<.25:continue
   gaps.append({'topic':topic,'kind':typ,'gap_sec':dt,'wall_interval':[lo,hi],'seq_delta':cur['seq']-prev['seq'],'thread_deltas':task_deltas(lo,hi),'resource_deltas':res_deltas(lo,hi)})
gaps.sort(key=lambda g:g['gap_sec'],reverse=True)
cam=[]
for r in logs:
 msg=r['message']
 if 'Camera backend read stages:' in msg:
  try:
   obj=json.loads(msg.split('Camera backend read stages:',1)[1].strip());wait=obj.get('host_wait',{});cam.append({'stamp':r['source_stamp'],'wait_sec':wait.get('elapsed_sec'),'wait_thread_cpu_sec':wait.get('thread_cpu_elapsed_sec'),'wait_enter':wait.get('enter'),'wait_exit':wait.get('exit'),'stages_sec':obj.get('stages_sec'),'raw_frames':obj.get('raw_frames'),'previous_completed_raw_frames':obj.get('previous_completed_raw_frames')})
  except ValueError:pass
report={'root_events':events,'limitations':['Observational before/after comparison, not randomized or proof of causation. User may manually move the arm.','Source stamps are camera/driver host timestamps, not guaranteed wire/device acquisition times. SDK raw is timestamped after dequeue.','Schedstat counters report scheduling runtime and runnable wait; sleeping/USB wait is inferred only from instantaneous wchan samples. Thread attribution is limited by sample intervals.','Resource deltas bracket gaps at approximately 500 ms cadence and can include adjacent work. Lost sampling time is reported and never filled.','This passive observer itself incurs CPU/socket overhead; no images/payloads are retained, but image topic transport still transfers complete ROS message bytes.'], 'segments':segment_stats,'gap_count':len(gaps),'largest_gaps':[{k:v for k,v in g.items() if k not in ['thread_deltas','resource_deltas']} for g in gaps[:20]],'camera_slow_diagnostics':cam,'sampling':summary['sampling'],'changes':summary['changes'],'pair_unavailable_log_count_lower_bound':sum('Camera RGB-D pair unavailable' in r['message'] for r in logs),'pair_unavailable_diagnostics_lower_bound':[{'source_stamp':r['source_stamp'],'arrival_wall':r['arrival_wall'],'message':r['message']} for r in logs if 'Camera RGB-D pair unavailable' in r['message']],'pair_log_count_semantics':'Throttled logs provide a lower bound on pair rejection events, never a total rejected-frame count.'}
(root/'correlation_detail.json').write_text(json.dumps(gaps,indent=2)+'\n');(root/'correlation_summary.json').write_text(json.dumps(report,indent=2)+'\n')
lines=['# Passive camera and feedback observation','',f'Observed {end-start:.3f} s. Protected camera PID{summary["ready"]["camera_pid"]} and driver PID{summary["ready"]["driver_pid"]} were only read; the observer issued no services, parameter changes, publications, serial accesses or debugger attaches. User may have manually moved the arm.','',f"Thread sampling actual median/p99/max: {summary['sampling']['actual_interval_median_sec']:.6f}/{summary['sampling']['actual_interval_p99_sec']:.6f}/{summary['sampling']['actual_interval_max_sec']:.6f} s; mean proc-read cost {summary['sampling']['read_cost_mean_sec']:.6f} s. Missed periods are not reconstructed.",'']
for name,s in segment_stats.items():
 lines += [name, '', '| Stream | Samples | Max source gap (s) | Max receipt gap (s) | Source gaps ≥250ms |','|---|---:|---:|---:|---:|']
 for topic,d in s['streams'].items():lines.append(f"| {topic} | {d['samples']} | {d.get('source_max_sec',0):.6f} | {d.get('receipt_max_sec',0):.6f} | {d.get('source_ge250ms',0)} |")
 rd=s['resource_deltas'];nonzero={k:v for k,v in rd.get('vmstat_deltas',{}).items() if v};lines += ['',f"Resource counter deltas: `{json.dumps(nonzero,sort_keys=True)}`; USB `{json.dumps(rd.get('usb_power',{}),sort_keys=True)}`.",'']
lines += ['Camera slow-cycle details, per-thread runtime/runqueue/wchan and pressure/counter correlation are in `correlation_summary.json` and `correlation_detail.json`. Root reload timestamps are preserved in `root_event.json`.','','Limitations:']+['- '+s for s in report['limitations']]
(root/'observation_report.md').write_text('\n'.join(lines)+'\n')
print(json.dumps({'directory':str(root),'segments':{name:{'interval_wall':s['interval_wall'],'streams':s['streams'],'resource_deltas':s['resource_deltas']} for name,s in segment_stats.items()},'gap_count':len(gaps),'camera_slow':[{k:v for k,v in c.items() if k in ['stamp','wait_sec','wait_thread_cpu_sec']} for c in cam],'sampling':summary['sampling'],'changes':summary['changes'],'pair_unavailable_log_count_lower_bound':sum('Camera RGB-D pair unavailable' in r['message'] for r in logs),'pair_unavailable_diagnostics_lower_bound':[{'source_stamp':r['source_stamp'],'arrival_wall':r['arrival_wall'],'message':r['message']} for r in logs if 'Camera RGB-D pair unavailable' in r['message']],'pair_log_count_semantics':'Throttled logs provide a lower bound on pair rejection events, never a total rejected-frame count.'},ensure_ascii=False))
