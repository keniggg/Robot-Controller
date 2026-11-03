"""Stationary camera A/B capture load observation; never commands hardware."""
import json,time,os,threading,queue,subprocess
from pathlib import Path
from datetime import datetime,timezone
base=Path('/media/zhuyupei/SanStand');assert base.is_mount()
out=base/'实验记录/Robot-Controller/ros_log'/('camera_recording_ab_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'));out.mkdir();(out/'ros').mkdir();os.environ['ROS_LOG_DIR']=str(out/'ros')
import rospy,rosbag
from sensor_msgs.msg import Image,JointState
from alicia_flexible_grasp_supervisor.msg import GraspState
rospy.init_node('stationary_camera_recording_ab',anonymous=True,disable_signals=True)
assert not rospy.wait_for_message('/grasp/state',GraspState,timeout=4).active
lock=threading.Lock();jobs=queue.Queue(maxsize=8);state={'phase':'initial','last_written':0,'q_count':0,'task_active':False};frames=[];joints=[];resources=[];phase_rows=[];colors={};depths={};write_errors=[]
def image_cb(m,kind):
 now=time.time_ns()
 with lock:
  ns=m.header.stamp.to_nsec()
  if kind=='color':frames.append(dict(phase=state['phase'],source_ns=ns,received_ns=now,seq=m.header.seq))
  table=colors if kind=='color' else depths;table[ns]=m
  for key in sorted(table)[:-6]:table.pop(key,None)
  if state['phase']=='dense_5hz' and ns in colors and ns in depths and ns-state['last_written']>=200000000:
   try:jobs.put_nowait((colors[ns],depths[ns]));state['last_written']=ns
   except queue.Full:write_errors.append(dict(time_ns=now,error='bounded_recorder_queue_full'))
def joint_cb(m):
 with lock:joints.append(dict(phase=state['phase'],source_ns=m.header.stamp.to_nsec(),received_ns=time.time_ns(),q=list(m.position[:6])))
def task_cb(m):state['task_active']=bool(m.active)
rospy.Subscriber('/supervisor/camera/color/image_raw',Image,lambda m:image_cb(m,'color'),queue_size=2,buff_size=2**24)
rospy.Subscriber('/supervisor/camera/depth/image_raw',Image,lambda m:image_cb(m,'depth'),queue_size=2,buff_size=2**24)
rospy.Subscriber('/alicia_d/accepted_joint_states',JointState,joint_cb,queue_size=100)
rospy.Subscriber('/grasp/state',GraspState,task_cb,queue_size=5)
writer_done=threading.Event();writes=[]
def writer():
 with rosbag.Bag(str(out/'dense_5hz_rgbd.bag'),'w',compression='lz4') as bag:
  while not writer_done.is_set() or not jobs.empty():
   try:rgb,depth=jobs.get(timeout=.2)
   except queue.Empty:continue
   start=time.monotonic()
   for topic,m in [('/supervisor/camera/color/image_raw',rgb),('/supervisor/camera/depth/image_raw',depth)]:bag.write(topic,m,m.header.stamp)
   writes.append(dict(source_ns=rgb.header.stamp.to_nsec(),wall_ns=time.time_ns(),duration_sec=time.monotonic()-start));jobs.task_done()
thread=threading.Thread(target=writer,daemon=True);thread.start()
def read(p):
 try:return Path(p).read_text().strip()
 except OSError as e:return str(e)
try:
 # All phases remain stationary. The second phase keeps metadata/resource logs
 # but adds no dense RGB-D disk stream. Existing live logs continue unchanged.
 for name in ['dense_5hz','metadata_only']:
  assert not state['task_active']
  with lock:state['phase']=name
  started=time.time_ns();deadline=time.monotonic()+45
  print('PHASE',name,started,flush=True)
  while time.monotonic()<deadline and not rospy.is_shutdown():
   if state['task_active']:raise RuntimeError('another task became active; no concurrent experiment')
   resources.append(dict(phase=name,wall_ns=time.time_ns(),monotonic_ns=time.monotonic_ns(),pressure={k:read('/proc/pressure/'+k) for k in ['cpu','io','memory']},cpu=read('/proc/stat').splitlines()[0],diskstats=read('/proc/diskstats'),usb_camera_power=read('/sys/bus/usb/devices/4-1/power/runtime_status')))
   time.sleep(.5)
  phase_rows.append(dict(name=name,started_ns=started,ended_ns=time.time_ns()))
  # Drain only our bounded writer between phases, before the next timed window.
  if name=='dense_5hz':
   with lock:state['phase']='drain'
   jobs.join()
finally:
 with lock:state['phase']='done'
 writer_done.set();thread.join(timeout=15)
 result=dict(directory=str(out),phases=phase_rows,frames=frames,joints=joints,resources=resources,writes=writes,write_errors=write_errors,hardware_commands_sent=0,writer_closed=not thread.is_alive())
 (out/'raw.json').write_text(json.dumps(result,indent=2)+'\n');Path('.runtime/latest_camera_recording_ab.json').write_text(json.dumps({'directory':str(out)},ensure_ascii=False)+'\n')
 summary=[]
 for ph in phase_rows:
  name=ph['name'];rows=[r for r in frames if r['phase']==name];qr=[r for r in joints if r['phase']==name]
  gaps=[(b['source_ns']-a['source_ns'])*1e-9 for a,b in zip(rows,rows[1:])]
  recv=[(b['received_ns']-a['received_ns'])*1e-9 for a,b in zip(rows,rows[1:])]
  summary.append(dict(phase=name,frames=len(rows),max_source_gap_sec=max(gaps,default=0),source_gaps_over_800ms=sum(v>.8 for v in gaps),source_gaps_over_200ms=sum(v>.2 for v in gaps),max_receipt_gap_sec=max(recv,default=0),max_source_age_sec=max(((r['received_ns']-r['source_ns'])*1e-9 for r in rows),default=0),joint_samples=len(qr),max_joint_excursion_rad=max((max(r['q'][i] for r in qr)-min(r['q'][i] for r in qr) for i in range(6)),default=0)))
 report=dict(phases=summary,write_errors=write_errors,hardware_commands_sent=0,limitations=['One short A/B window cannot prove causation or exclude an intermittent host/USB fault.','Existing logs continue in both phases; only the additional dense RGB-D stream is changed.'])
 (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True)
