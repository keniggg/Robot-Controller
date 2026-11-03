#!/usr/bin/env python3
"""Prepared read-only monitor. No collection without explicit --run. No ROS/USB access."""
import argparse,datetime,json,os,pathlib,statistics,time

def read(path):
 try:return pathlib.Path(path).read_text().strip()
 except OSError as e:return {'error':type(e).__name__,'errno':e.errno}

def parse_stat(text):
 if not isinstance(text,str):return {'error':text}
 fields=text.rsplit(')',1)[1].split()
 return {'state':fields[0],'minflt':int(fields[7]),'majflt':int(fields[9]),'utime_ticks':int(fields[11]),'stime_ticks':int(fields[12]),'start_ticks':int(fields[19]),'rss_pages':int(fields[21])}

def task(pid,tid):
 prefix=pathlib.Path(f'/proc/{pid}/task/{tid}');r={'pid':pid,'tid':tid,**parse_stat(read(prefix/'stat'))};sched=read(prefix/'schedstat');r['wchan']=read(prefix/'wchan')
 if isinstance(sched,str):
  vals=sched.split();r.update({'runtime_ns':int(vals[0]),'runqueue_wait_ns':int(vals[1]),'timeslices':int(vals[2])})
 else:r['schedstat_error']=sched
 return r

def discover():
 selected={}
 for path in pathlib.Path('/proc').iterdir():
  if not path.name.isdigit():continue
  name=read(path/'comm')
  if isinstance(name,str) and name.startswith(('kswapd','kcompactd','khugepaged','vmtools','vmware-vmblock')):
   s=parse_stat(read(path/'stat'))
   if 'start_ticks' in s:selected[int(path.name)]={'comm':name,'start_ticks':s['start_ticks']}
 return selected

def resource():
 vm={};text=read('/proc/vmstat')
 if isinstance(text,str):
  for line in text.splitlines():
   key,val=line.split()
   if key.startswith(('pswp','pgmajfault','pgscan','pgsteal','allocstall','compact_','thp_','balloon_')) or key in ('nr_free_pages','nr_dirty','nr_writeback','nr_anon_transparent_hugepages'):vm[key]=int(val)
 mem={};text=read('/proc/meminfo')
 if isinstance(text,str):
  for line in text.splitlines():
   key,value=line.split(':',1)
   if key in ('MemTotal','MemAvailable','MemFree','SwapTotal','SwapFree','SwapCached','AnonHugePages','AnonPages','Slab','SReclaimable','SUnreclaim','Dirty','Writeback'):mem[key]=value.strip()
 thp=pathlib.Path('/sys/kernel/mm/transparent_hugepage/khugepaged')
 return {'vmstat':vm,'meminfo':mem,'pressure':{k:read('/proc/pressure/'+k) for k in ('cpu','memory','io')},'khugepaged':{k:read(thp/k) for k in ('full_scans','pages_collapsed')},'cpu_stat':read('/proc/stat').splitlines()[0]}

def main():
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',action='store_true');parser.add_argument('--out-dir');parser.add_argument('--duration',type=float,default=180);parser.add_argument('--period',type=float,choices=(.1,.2,.5),default=.5);parser.add_argument('--watch-pid',action='append',default=[],help='Optional PID:start_ticks; pins identity.');parser.add_argument('--power-boundary-file');args=parser.parse_args()
 if not args.run:
  print('Prepared only. No collection started. Use --run with an external --out-dir only when root explicitly starts a future observation.');return
 if not 0<args.duration<=600:raise SystemExit('duration must be >0 and <=600 seconds')
 base=pathlib.Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log').resolve()
 if not os.path.ismount('/media/zhuyupei/SanStand'):raise SystemExit('SanStand unavailable; refusing local fallback')
 if not args.out_dir:raise SystemExit('--out-dir required with --run')
 out=pathlib.Path(args.out_dir).resolve()
 if base not in out.parents:raise SystemExit('Output must be a new directory under external ros_log')
 out.mkdir(exist_ok=False);selected=discover()
 for specification in args.watch_pid:
  pid,ticks=map(int,specification.split(':'));selected[pid]={'comm':'explicit_watch','start_ticks':ticks}
 started=time.monotonic();start_wall=time.time();deadline=started+args.duration;period=args.period;rows=[];events=[];previous=None;costs=[]
 ready={'ready_wall':start_wall,'ready_monotonic':started,'duration_sec':args.duration,'period_sec':period,'selected_processes':selected,'scope':'read-only /proc and THP counters; no ROS, hardware queries, sysctl writes, ptrace, or guest API calls','wchan_zero_semantics':'0 is ambiguous: running or inaccessible, especially other-user/kernel tasks. Never infer runnable from 0 alone.','power_boundary_file':args.power_boundary_file}
 (out/'ready.json').write_text(json.dumps(ready,indent=2)+'\n');print(json.dumps(ready),flush=True);target=started
 while time.monotonic()<deadline:
  wait=target-time.monotonic()
  if wait>0:time.sleep(min(wait,max(0,deadline-time.monotonic())))
  now=time.monotonic()
  if now>=deadline:break
  row={'monotonic':now,'wall':time.time(),'actual_interval_sec':None if previous is None else now-previous,'resource':resource(),'processes':{}}
  for pid,identity in selected.items():
   observed=parse_stat(read(f'/proc/{pid}/stat'))
   if observed.get('start_ticks')!=identity['start_ticks']:
    row['processes'][str(pid)]={'identity_error':observed,'expected':identity};continue
   try:ids=[int(p.name) for p in pathlib.Path(f'/proc/{pid}/task').iterdir()]
   except OSError as e:row['processes'][str(pid)]={'error':repr(e)};continue
   row['processes'][str(pid)]={'identity':identity,'tasks':[task(pid,tid) for tid in ids]}
  cost=time.monotonic()-now;row['read_cost_sec']=cost;costs.append(cost);rows.append(row);previous=now
  if period<.5 and len(costs)>=10 and statistics.mean(costs[-10:])>.01:
   events.append({'event':'period_increased_for_overhead','wall':time.time(),'old':period,'new':.5,'read_cost_last10_mean_sec':statistics.mean(costs[-10:])});period=.5
  target=now+period
 boundary=read(args.power_boundary_file) if args.power_boundary_file else None
 result={'ready':ready,'finished_wall':time.time(),'sample_count':len(rows),'events':events,'final_period_sec':period,'power_boundary_raw':boundary,'read_cost_mean_sec':statistics.mean(costs) if costs else None,'max_actual_interval_sec':max((r['actual_interval_sec'] or 0 for r in rows),default=None),'missing_intervals_not_interpolated':True}
 (out/'samples.json').write_text(json.dumps(rows,separators=(',',':'))+'\n');(out/'summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
if __name__=='__main__':main()
