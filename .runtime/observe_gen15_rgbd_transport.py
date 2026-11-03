"""Bounded read-only RGB-D header observer plus passive process/socket evidence."""
import argparse,collections,hashlib,json,os,shutil,subprocess,threading,time
from pathlib import Path
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

ROOT=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')

def proc(pid):
 p=Path('/proc')/str(pid);result={'pid':pid,'threads':{}}
 try:
  result['start_ticks']=int((p/'stat').read_text().rsplit(')',1)[1].split()[19])
  result['status']={l.split(':')[0]:l.split(':',1)[1].strip() for l in (p/'status').read_text().splitlines() if l.startswith(('Threads:','VmRSS:','State:'))}
  for t in (p/'task').iterdir():
   try: result['threads'][t.name]={'name':(t/'comm').read_text().strip(),'wchan':(t/'wchan').read_text().strip(),'schedstat':[int(x)for x in (t/'schedstat').read_text().split()]}
   except OSError:pass
 except OSError as e:result['error']=str(e)
 return result

def sockets(pids):
 try:
  done=subprocess.run(['ss','-tinp'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,timeout=.7)
  selected=[];lines=done.stdout.splitlines()
  for i,l in enumerate(lines):
   if any(('pid=%d,'%p) in l for p in pids):selected.append(l+('\n'+lines[i+1] if i+1<len(lines) and lines[i+1].startswith((' ','\t')) else ''))
  return {'returncode':done.returncode,'connections':selected,'stderr':done.stderr[:1000]}
 except subprocess.TimeoutExpired:return {'error':'ss_timeout'}

def main():
 p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('--seconds',type=float,default=60);a=p.parse_args()
 assert 0<a.seconds<=60
 out=Path(a.out);assert Path('/media/zhuyupei/SanStand').is_mount() and EXT in out.parents;out.mkdir(exist_ok=False,parents=True)
 os.environ['ROS_LOG_DIR']=str(out/'ros')
 rospy.init_node('rgbd_transport_header_observer',anonymous=True,disable_signals=True)
 rows=[];lock=threading.Lock();open_=True;pids=[13484,118128,os.getpid()]
 def append(row):
  with lock:
   if open_:rows.append(row)
 def raw(stream,msg):
  wall=time.time_ns();mono=time.monotonic();source=msg.header.stamp.to_nsec()
  append(dict(kind='image_header',stream=stream,source_stamp_ns=source,seq=msg.header.seq,receive_wall_ns=wall,receive_monotonic=mono,source_age_sec=(wall-source)*1e-9,data_bytes=len(msg.data)))
 def status(msg):
  wall=time.time_ns();mono=time.monotonic()
  try:s=json.loads(msg.data)
  except ValueError:return
  append(dict(kind='unknown_status',receive_wall_ns=wall,receive_monotonic=mono,**{k:s[k] for k in ('generation','source_stamp_ns','detail','state','target_lost','frame_timing','rgbd_ingress_timing') if k in s}))
 subs=[rospy.Subscriber('/supervisor/camera/'+s+'/image_raw',Image,lambda m,s=s:raw(s,m),queue_size=2,buff_size=2**24)for s in ('color','depth')]
 subs.append(rospy.Subscriber('/perception/unknown/detector_status',String,status,queue_size=4))
 start=time.monotonic();startwall=time.time_ns();i=0;passive=[]
 while time.monotonic()-start<a.seconds and not rospy.is_shutdown():
  tick=time.monotonic();sample=dict(wall_ns=time.time_ns(),monotonic=tick,processes=[proc(pid)for pid in pids],psi={k:Path('/proc/pressure',k).read_text()for k in ('cpu','memory','io')},loadavg=Path('/proc/loadavg').read_text().strip())
  if i%2==0:sample['sockets']=sockets(pids)
  passive.append(sample);i+=1
  wait=min(start+a.seconds,time.monotonic()+max(0,1-(time.monotonic()-tick)))-time.monotonic()
  if wait>0:time.sleep(wait)
 with lock:open_=False
 for s in subs:s.unregister()
 header={'schema':1,'started_wall_ns':startwall,'elapsed_sec':time.monotonic()-start,'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'observer_pid':os.getpid(),'observed_processes':pids,'scope':'Two additional full RGB-D subscriptions queue2/buff16MiB; retains only numeric headers, not image pixels; additional transport/serialization load may influence timing. Passive proc and sockets are contemporaneous only. No ROS service calls, actions, runtime parameter writes or temperature reads.','py_spy_available':shutil.which('py-spy'),'counts':dict(collections.Counter(r['kind']for r in rows))}
 (out/'manifest.json').write_text(json.dumps(header,indent=2)+'\n')
 with (out/'events.jsonl').open('w')as f:
  for r in rows:f.write(json.dumps(r,sort_keys=True,allow_nan=False)+'\n')
 with (out/'passive.jsonl').open('w')as f:
  for r in passive:f.write(json.dumps(r,sort_keys=True,allow_nan=False)+'\n')
 summary={}
 for s in ('color','depth'):
  rr=sorted([r for r in rows if r['kind']=='image_header' and r['stream']==s],key=lambda r:r['receive_monotonic']);ages=sorted(r['source_age_sec']for r in rr);gaps=[rr[j]['receive_monotonic']-rr[j-1]['receive_monotonic']for j in range(1,len(rr))]
  summary[s]=dict(count=len(rr),maximum_age_sec=max(ages)if ages else None,p95_age_sec=ages[min(len(ages)-1,int(len(ages)*.95))]if ages else None,maximum_receive_gap_sec=max(gaps)if gaps else None)
 header['summary']=summary;(out/'summary.json').write_text(json.dumps(header,indent=2)+'\n');print(json.dumps(dict(directory=str(out),**header)),flush=True)
if __name__=='__main__':main()
