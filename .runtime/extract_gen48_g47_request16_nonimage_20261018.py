"""Bounded original G47 SDK/error topic extraction; no image deserialization."""
import hashlib,json,os,sys,time,traceback,platform,struct,io
from pathlib import Path
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller');V=E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
G=E/'grasp_records/20261018T085552.871836Z_7d7d31b2570b433b8ee05398edc47d73'
OUT=V/'gen48_g47_request16_nonimage_inputs_r1';BOUNDARY=[]
STAMP=1792313803719039440
def sha(path):
 h=hashlib.sha256()
 with Path(path).open('rb')as f:
  for block in iter(lambda:f.read(4*1024*1024),b''):h.update(block)
 return h.hexdigest()

def put(path,value):
 raw=(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
 with path.open('xb')as f:f.write(raw)
 assert path.read_bytes()==raw
 return dict(path=str(path),sha256=sha(path))

def deny(kind,detail):
 BOUNDARY.append(dict(kind=kind,detail=str(detail)));raise RuntimeError('offline boundary: '+kind)

def audit_event(event,args):
 if event.startswith('socket.'):deny(event,args[:1])
 if event in ('subprocess.Popen','os.system','os.exec','os.posix_spawn','os.fork','os.forkpty'):deny(event,'process creation')
 if event=='open':
  path=args[0]
  if isinstance(path,(str,bytes,os.PathLike)):
   p=os.fsdecode(path)
   if p.startswith('/dev/') and p not in ('/dev/null','/dev/urandom','/dev/random'):deny('device_open',p)
   if p.startswith('/sys/class/thermal')or '/hwmon/'in p:deny('temperature_file',p)
   flags=args[2]if len(args)>2 and isinstance(args[2],int)else 0
   mode=args[1]if len(args)>1 and isinstance(args[1],str)else ''
   if p!='/dev/null' and (flags & (os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC|os.O_APPEND) or any(x in mode for x in 'wax+')):
    try:Path(p).resolve().relative_to(OUT.resolve())
    except ValueError:deny('write_outside_probe',p)

def main():
 assert E.parent.parent.is_mount() and not OUT.exists();OUT.mkdir();(OUT/'tmp').mkdir();(OUT/'unused_ros').mkdir()
 os.environ.update(PYTHONDONTWRITEBYTECODE='1',ROS_MASTER_URI='http://127.0.0.1:1',ROS_LOG_DIR=str(OUT/'unused_ros'),ROS_HOME=str(OUT/'unused_ros'),TMPDIR=str(OUT/'tmp'))
 sys.dont_write_bytecode=True;sys.addaudithook(audit_event)
 native=os.uname();platform.uname=lambda:platform.uname_result(*(tuple(native)+(native.machine,)));platform.architecture=lambda *a,**k:(str(8*struct.calcsize('P'))+'bit','ELF')
 result=dict(kind='G47_R16_original_nonimage_SDK_error_extraction',completed=False,source_stamp_ns=STAMP,ROS_init_called=False,hardware_contacted=False,temperature_read=False,motion_commands_sent=False,image_topics_read=False,boundary_attempts=BOUNDARY)
 try:
  import rosbag,rospy
  for name in ('init_node','Publisher','Subscriber','Service','ServiceProxy','wait_for_service','get_param','set_param','delete_param'):setattr(rospy,name,lambda *a,_name=name,**k:deny('ROS_'+_name,'forbidden'))
  manifest=json.loads((G/'manifest.json').read_text());assert manifest['recorder_closed'] and manifest['result']=='failure'
  ref=next(r for r in manifest['files'] if r['path']=='keyframes.bag');bagpath=G/'keyframes.bag';assert bagpath.stat().st_size==ref['size_bytes'] and sha(bagpath)==ref['sha256']
  topics=['/alicia_d/accepted_joint_states','/experiment/runtime_execution_error_snapshot'];joints=[];errors=[];counts={t:0 for t in topics}
  with rosbag.Bag(str(bagpath),'r') as bag:
   for topic,msg,received in bag.read_messages(topics=topics):
    counts[topic]+=1;wire=io.BytesIO();msg.serialize(wire);common=dict(topic=topic,bag_receive_ns=received.to_nsec(),wire_sha256=hashlib.sha256(wire.getvalue()).hexdigest())
    if topic==topics[0]:
     stamp=msg.header.stamp.to_nsec()
     if STAMP-2_000_000_000<=stamp<=STAMP+3_000_000_000:joints.append(dict(common,header_stamp_ns=stamp,frame_id=msg.header.frame_id,names=list(msg.name),positions=list(msg.position)))
    else:
     if STAMP-5_000_000_000<=received.to_nsec()<=STAMP+3_000_000_000:errors.append(dict(common,data=json.loads(msg.data)))
  nearest=max((r for r in joints if r['header_stamp_ns']<=STAMP and r['frame_id']=='sdk_measured'),key=lambda r:r['header_stamp_ns'],default=None)
  eligible=[r for r in errors if r['bag_receive_ns']<=STAMP and isinstance(r['data'],dict) and r['data']]
  latest=max(eligible,key=lambda r:r['bag_receive_ns'],default=None)
  assert not BOUNDARY and sha(bagpath)==ref['sha256']
  result.update(completed=True,bag_sha256=ref['sha256'],manifest_sha256=sha(G/'manifest.json'),topic_message_counts=counts,read_topics=topics,joint_sample_window=[STAMP-2_000_000_000,STAMP+3_000_000_000],joint_samples=joints,runtime_error_snapshots=errors,nearest_sdk_at_or_before_snapshot=nearest,nearest_sdk_source_age_sec=(STAMP-nearest['header_stamp_ns'])/1e9 if nearest else None,latest_error_snapshot_received_at_or_before_source=latest,temporal_scope='Nearest measured source<=final image stamp, not proven original prepare callback latest state. No current/fresh execution authority.',helper_sha256=sha(__file__))
 except BaseException as e:result.update(error=repr(e),traceback=traceback.format_exc(),helper_sha256=sha(__file__))
 rr=put(OUT/'result.json',result);seal=put(OUT/'receipt.json',dict(result=rr,boundary_attempts=BOUNDARY,completed=result['completed'],image_topics_read=False,temperature_read=False,ROS_init_called=False));print(json.dumps(dict(result=rr,receipt=seal,counts=result.get('topic_message_counts'),sdk_age=result.get('nearest_sdk_source_age_sec'),error=result.get('error')),ensure_ascii=False),flush=True)
 return 0 if result['completed'] and not BOUNDARY else 1
if __name__=='__main__':raise SystemExit(main())
