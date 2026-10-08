#!/usr/bin/env python3
"""Bounded passive ROS/proc observer. Never invokes services or publishes."""
import argparse,collections,datetime,json,math,os,pathlib,statistics,struct,threading,time
import rospy
import xmlrpc.client
from diagnostic_msgs.msg import DiagnosticArray
from rosgraph_msgs.msg import Log

BASE=pathlib.Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')
TOPICS=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/alicia_d/accepted_joint_states','/alicia_d/sdk_diagnostics']
parser=argparse.ArgumentParser();parser.add_argument('--duration',type=float,default=180);parser.add_argument('--camera-pid',type=int,required=True);parser.add_argument('--camera-start-ticks',type=int);parser.add_argument('--camera-node',default='/supervisor_camera');args=parser.parse_args()
if not os.path.ismount('/media/zhuyupei/SanStand'):raise SystemExit('SanStand is not mounted; refusing local fallback')
START_TAG=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
OUT=BASE/('passive_camera_feedback_'+START_TAG);OUT.mkdir()
rows={k:[] for k in TOPICS};logs=[];thread_samples=[];resource_samples=[];changes=[];callback_errors=[]
lock=threading.Lock();started_mono=time.monotonic();started_wall=time.time();deadline=started_mono+args.duration
CAMERA_PID=args.camera_pid
DRIVER_PID=8750
expected_process_start_ticks={}
camera_candidates={}
next_camera_discovery=0.0
last_process_identity={}

def readtext(p):
 try:return pathlib.Path(p).read_text().strip()
 except OSError as e:return {'error':type(e).__name__,'errno':e.errno}

def proc_task(pid,tid):
 root=f'/proc/{pid}/task/{tid}';stat=readtext(root+'/stat');sched=readtext(root+'/schedstat');wchan=readtext(root+'/wchan')
 if not isinstance(stat,str):return {'pid':pid,'tid':tid,'error':stat}
 # comm may contain spaces/parens. Fields after final ) begin with state (field 3).
 tail=stat.rsplit(')',1)[1].split();r={'pid':pid,'tid':tid,'state':tail[0],'start_ticks':int(tail[19]),'utime_ticks':int(tail[11]),'stime_ticks':int(tail[12]),'wchan':wchan}
 if isinstance(sched,str):
  vals=[int(v) for v in sched.split()];r.update({'runtime_ns':vals[0],'runqueue_wait_ns':vals[1],'timeslices':vals[2]})
 else:r['schedstat_error']=sched
 return r

def process_identity(pid):
 raw=readtext(f'/proc/{pid}/stat')
 if not isinstance(raw,str):return {'pid':pid,'error':raw}
 tail=raw.rsplit(')',1)[1].split()
 actual=int(tail[19]);expected=expected_process_start_ticks.get(pid)
 return {'pid':pid,'start_ticks':actual,'expected_start_ticks':expected,'identity_matches':expected is None or actual==expected}

def proc_threads():
 global next_camera_discovery
 now=time.monotonic();tasks=[];errors={}
 for role,pid in [('camera',CAMERA_PID),('driver',DRIVER_PID)]:
  ident=process_identity(pid)
  state=(ident.get('start_ticks'),ident.get('identity_matches'),str(ident.get('error')))
  if state!=last_process_identity.get(role):
   changes.append({'event':'process_identity','role':role,'monotonic':now,'wall':time.time(),**ident});last_process_identity[role]=state
  if not ident.get('identity_matches',False):
   errors[role]=ident;tasks.append({'pid':pid,'tid':pid,'identity_error':ident});continue
  if role=='camera':
   if now>=next_camera_discovery:
    try:
     for path in pathlib.Path(f'/proc/{pid}/task').iterdir():
      tid=int(path.name)
      if tid==pid or tid in camera_candidates:continue
      if readtext(path/'wchan')=='do_select':
       sample=proc_task(pid,tid)
       if 'start_ticks' in sample:
        camera_candidates[tid]=sample['start_ticks']
        changes.append({'event':'camera_do_select_candidate_discovered','pid':pid,'tid':tid,'start_ticks':sample['start_ticks'],'monotonic':now,'wall':time.time()})
    except OSError as e:errors[role]={'error':type(e).__name__,'errno':e.errno}
    next_camera_discovery=now+.5
   ids=[pid]+sorted(camera_candidates)
  else:
   try:ids=sorted(int(path.name) for path in pathlib.Path(f'/proc/{pid}/task').iterdir())
   except OSError as e:errors[role]={'error':type(e).__name__,'errno':e.errno};ids=[pid]
  for tid in ids:
   item=proc_task(pid,tid)
   if role=='camera' and tid in camera_candidates and item.get('start_ticks') not in (None,camera_candidates[tid]):
    item={'pid':pid,'tid':tid,'identity_error':'thread_start_ticks_changed','expected_start_ticks':camera_candidates[tid],'actual_start_ticks':item.get('start_ticks')}
   tasks.append(item)
 return tasks,errors or None

class TimeoutTransport(xmlrpc.client.Transport):
 def make_connection(self,host):
  connection=super().make_connection(host);connection.timeout=3.0;return connection

def readonly_camera_identity():
 caller='/passive_camera_feedback_identity_check'
 with xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI','http://localhost:11311'),transport=TimeoutTransport()) as master:
  code,message,uri=master.lookupNode(caller,args.camera_node)
 if code!=1:raise RuntimeError('Camera node lookup failed: '+str(message))
 with xmlrpc.client.ServerProxy(uri,transport=TimeoutTransport()) as node:
  code,message,pid=node.getPid(caller)
 if code!=1 or int(pid)!=CAMERA_PID:raise RuntimeError(f'Camera node PID mismatch: expected {CAMERA_PID}, got {pid}; not observing a replacement')
 ident=process_identity(CAMERA_PID)
 if 'start_ticks' not in ident:raise RuntimeError('Camera process unavailable: '+str(ident))
 if args.camera_start_ticks is not None and ident['start_ticks']!=args.camera_start_ticks:raise RuntimeError('Camera start_ticks mismatch; refusing stale PID')
 return {'node':args.camera_node,'node_uri':uri,'pid':CAMERA_PID,'start_ticks':ident['start_ticks']}

def resources():
 p={k:readtext('/proc/pressure/'+k) for k in ['cpu','io','memory']}
 vm={};txt=readtext('/proc/vmstat')
 if isinstance(txt,str):
  for line in txt.splitlines():
   k,v=line.split()
   if k in ['pswpin','pswpout','pgmajfault','nr_dirty','nr_writeback','nr_free_pages'] or k.startswith(('pgscan','allocstall','compact_')):vm[k]=int(v)
 cpus=readtext('/proc/stat');cpu={}
 if isinstance(cpus,str):
  for line in cpus.splitlines():
   k,*vs=line.split()
   if k=='cpu':cpu={kk:int(v) for kk,v in zip(['user','nice','system','idle','iowait','irq','softirq','steal','guest','guest_nice'],vs)}
 usb={k:readtext('/sys/bus/usb/devices/4-1/power/'+k) for k in ['runtime_status','runtime_active_time','runtime_suspended_time','control','autosuspend_delay_ms']}
 return {'pressure':p,'vmstat':vm,'cpu':cpu,'usb4_1_power':usb,'observer':proc_task(os.getpid(),os.getpid())}

def cb(msg,topic):
 m=time.monotonic();w=time.time()
 if m>deadline:return
 try:
  b=msg._buff
  if len(b)<16:raise ValueError('short header')
  seq,sec,nsec,frame_len=struct.unpack_from('<IIII',b,0)
  r={'monotonic':m,'arrival_wall':w,'source_stamp':sec+nsec/1e9,'source_sec':sec,'source_nsec':nsec,'seq':seq,'datatype':msg._connection_header.get('type'),'callerid':msg._connection_header.get('callerid')}
  if topic=='/alicia_d/sdk_diagnostics':
   d=DiagnosticArray();d.deserialize(b)
   r['diagnostics']=[{'name':s.name,'level':s.level,**{v.key:v.value for v in s.values if v.key in ['command_function_hex','timestamp_kind','run_status_hex']}} for s in d.status]
  with lock:rows[topic].append(r)
 except Exception as e:
  with lock:callback_errors.append({'mono':m,'topic':topic,'error':repr(e)})

def log_cb(raw):
 m=time.monotonic();w=time.time()
 if m>deadline:return
 try:
  msg=Log();msg.deserialize(raw._buff)
  if msg.name!=args.camera_node or not any(s in msg.msg for s in ['Camera slow cycle','Camera backend read stages','Camera read failed','Camera RGB-D pair unavailable']):return
  r={'monotonic':m,'arrival_wall':w,'source_stamp':msg.header.stamp.to_sec(),'level':msg.level,'node':msg.name,'message':msg.msg[:18000]}
  with lock:logs.append(r)
 except Exception as e:
  with lock:callback_errors.append({'mono':m,'topic':'/rosout_agg','error':repr(e)})

camera_identity=readonly_camera_identity()
expected_process_start_ticks[CAMERA_PID]=camera_identity['start_ticks']
driver_identity=process_identity(DRIVER_PID)
if 'start_ticks' not in driver_identity:raise SystemExit('Driver PID unavailable; no observation started')
expected_process_start_ticks[DRIVER_PID]=driver_identity['start_ticks']
proc_threads() # discover initial do_select candidates outside timed sampling
rospy.init_node('passive_camera_feedback_observer',anonymous=True,disable_signals=True,disable_rosout=True)
subs=[rospy.Subscriber(k,rospy.AnyMsg,cb,callback_args=k,queue_size=300,buff_size=16*1024*1024,tcp_nodelay=True) for k in TOPICS]
subs.append(rospy.Subscriber('/rosout_agg',rospy.AnyMsg,log_cb,queue_size=400,buff_size=4*1024*1024,tcp_nodelay=True))
ready_wall=time.time();ready_mono=time.monotonic();deadline=ready_mono+args.duration
ready={'output_dir':str(OUT),'observer_pid':os.getpid(),'ready_wall':ready_wall,'ready_monotonic':ready_mono,'ready_utc':datetime.datetime.fromtimestamp(ready_wall,datetime.timezone.utc).isoformat(),'duration_sec':args.duration,'deadline_monotonic':deadline,'event_file':str(OUT/'root_event.json'),'root_event_format':{'events':[{'event':'perception_reload_begin_or_end','wall_time':0,'monotonic':0,'note':'root supplies actual timestamps'}]},'camera_pid':CAMERA_PID,'camera_tids':[CAMERA_PID]+sorted(camera_candidates),'camera_identity':camera_identity,'driver_pid':DRIVER_PID,'driver_start_ticks':expected_process_start_ticks[DRIVER_PID],'native_thread_selection':'Retain every thread observed in do_select; discover every 500 ms and pin its start_ticks; includes main thread. Heuristic candidates, not symbol-verified.','user_may_manually_move_arm':True,'hz_clock_ticks':os.sysconf('SC_CLK_TCK'),'scope':'Passive subscriptions + proc/sys reads; no publishers/services/parameter mutations/serial/debugger.'}
(OUT/'ready.json').write_text(json.dumps(ready,indent=2)+'\n');print(json.dumps(ready),flush=True)
period=.05;next_sample=ready_mono;next_resources=ready_mono;last_sample=None;costs=[];last_tids=None
while time.monotonic()<deadline:
 wait=next_sample-time.monotonic()
 if wait>0:time.sleep(min(wait,max(0,deadline-time.monotonic())))
 now=time.monotonic()
 if now>=deadline:break
 wall=time.time();tasks,discovery_error=proc_threads();after=time.monotonic();cost=after-now;costs.append(cost)
 tids=[r['tid'] for r in tasks if r['pid']==8750]
 if tids!=last_tids:
  changes.append({'monotonic':now,'wall':wall,'event':'driver_tid_set','tids':tids,'discovery_error':discovery_error});last_tids=tids
 thread_samples.append({'monotonic':now,'wall':wall,'actual_interval_sec':None if last_sample is None else now-last_sample,'scheduled_monotonic':next_sample,'late_sec':now-next_sample,'read_cost_sec':cost,'tasks':tasks,'process_discovery_error':discovery_error})
 last_sample=now
 if now>=next_resources:
  resource_start=time.monotonic(); rr=resources();rr.update({'monotonic':resource_start,'wall':time.time(),'read_cost_sec':time.monotonic()-resource_start});resource_samples.append(rr);next_resources=now+.5
 if period==.05 and len(costs)>=20 and statistics.mean(costs[-20:])>.010:
  changes.append({'monotonic':time.monotonic(),'wall':time.time(),'event':'sampling_period_increased','old_s':period,'new_s':.1,'reason':'last20 thread proc reads mean exceeded 10 ms','mean_read_cost_sec':statistics.mean(costs[-20:])});period=.1
 # Do not fabricate missed observations or catch up with synthetic samples.
 next_sample=now+period
ended_mono=time.monotonic();ended_wall=time.time()
for s in subs:s.unregister()
with lock:
 captured={k:list(v) for k,v in rows.items()};captured_logs=list(logs)

def percentile(xs,q):
 if not xs:return None
 ss=sorted(xs);return ss[min(len(ss)-1,int((len(ss)-1)*q))]

def topic_summary(rs):
 if not rs:return {'count':0}
 src=[b['source_stamp']-a['source_stamp'] for a,b in zip(rs,rs[1:])];arr=[b['monotonic']-a['monotonic'] for a,b in zip(rs,rs[1:])]
 delay=[r['arrival_wall']-r['source_stamp'] for r in rs]
 def gaprows(field,threshold):
  found=[]
  for a,b in zip(rs,rs[1:]):
   dt=b[field]-a[field]
   if dt>=threshold:found.append({'gap_sec':dt,'from_source':a['source_stamp'],'to_source':b['source_stamp'],'from_arrival_wall':a['arrival_wall'],'to_arrival_wall':b['arrival_wall'],'from_mono':a['monotonic'],'to_mono':b['monotonic'],'from_seq':a['seq'],'to_seq':b['seq'],'seq_delta':b['seq']-a['seq']})
  return sorted(found,key=lambda r:r['gap_sec'],reverse=True)[:50]
 return {'count':len(rs),'callerids':sorted(set(r['callerid'] for r in rs)),'source_gap_median_sec':statistics.median(src) if src else None,'source_gap_max_sec':max(src) if src else None,'receipt_gap_max_sec':max(arr) if arr else None,'arrival_minus_source_median_sec':statistics.median(delay),'arrival_minus_source_p99_sec':percentile(delay,.99),'arrival_minus_source_max_sec':max(delay),'source_gaps_ge_250ms':gaprows('source_stamp',.25),'receipt_gaps_ge_250ms':gaprows('monotonic',.25),'seq_discontinuities':sum(b['seq']!=a['seq']+1 for a,b in zip(rs,rs[1:]))}
summary={'ready':ready,'ended_wall':ended_wall,'ended_monotonic':ended_mono,'observed_duration_sec':ended_mono-ready_mono,'topics':{k:topic_summary(v) for k,v in captured.items()},'camera_slow_log_count':sum('Camera slow cycle' in r['message'] for r in captured_logs),'pair_unavailable_log_count_lower_bound':sum('Camera RGB-D pair unavailable' in r['message'] for r in captured_logs),'pair_log_count_semantics':'Throttled pair_unavailable logs are a lower bound, not total rejected pairs','thread_sample_count':len(thread_samples),'resource_sample_count':len(resource_samples),'changes':changes,'callback_errors':callback_errors}
intervals=[r['actual_interval_sec'] for r in thread_samples if r['actual_interval_sec'] is not None]
summary['sampling']={'initial_period_sec':.05,'final_period_sec':period,'actual_interval_median_sec':statistics.median(intervals) if intervals else None,'actual_interval_p99_sec':percentile(intervals,.99),'actual_interval_max_sec':max(intervals) if intervals else None,'read_cost_mean_sec':statistics.mean(costs),'read_cost_max_sec':max(costs),'missed_observations_are_not_interpolated':True}
event=readtext(OUT/'root_event.json')
if isinstance(event,str):
 try:summary['root_events']=json.loads(event)
 except ValueError:summary['root_events']={'unparsed':event}
else:summary['root_events']=event
(OUT/'topics.json').write_text(json.dumps(captured,separators=(',',':'))+'\n')
(OUT/'camera_logs.json').write_text(json.dumps(captured_logs,indent=2)+'\n')
(OUT/'thread_samples.json').write_text(json.dumps(thread_samples,separators=(',',':'))+'\n')
(OUT/'resource_samples.json').write_text(json.dumps(resource_samples,separators=(',',':'))+'\n')
(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({'completed':True,'output_dir':str(OUT),'summary':str(OUT/'summary.json'),'observed_duration_sec':summary['observed_duration_sec'],'topic_counts':{k:v['count'] for k,v in summary['topics'].items()},'sampling':summary['sampling']},ensure_ascii=False),flush=True)
