"""Extract explicitly allowed nonimage/non-temperature messages from sealed G48."""
import base64
import hashlib
import io
import json
import math
import os
import platform
import struct
from pathlib import Path
import socket
import sys
import time
W=Path(__file__).resolve().parents[1]
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N=E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
V=N/'verification'
G=E/'grasp_records/20261018T095014.238124Z_ea3e1758e151468c9c945b7e94a07b1d'
O=V/'gen49_g48_terminal_messages_r2'
TOPICS=(
 '/gui/joint_direct_mode','/grasp/state','/grasp/task_event','/grasp/near_field_phase',
 '/grasp_mode/observe_return_session','/grasp_mode/observe_return_ack',
 '/grasp_mode/observe_return_measurement_request','/grasp_mode/observe_return_measurement_ack',
 '/grasp_mode/contact_station_retirement_request','/grasp_mode/contact_station_retirement_ack',
 '/grasp_6d/status','/perception/unknown/detector_status',
 '/grasp_6d/plan_enriched','/grasp_6d/preview_plan_enriched','/grasp_6d/frozen_contact_plan',
 '/grasp_6d/object_geometry','/perception/object')
sys.path[:0]=[str(W/'devel/lib/python3/dist-packages'),'/opt/ros/noetic/lib/python3/dist-packages']
attempts=[]
class OfflineSocket(socket.socket):
 def __new__(cls,*args,**kwargs):
  attempts.append('socket');raise RuntimeError('offline sealed-bag extraction forbids sockets')
socket.socket=OfflineSocket
sys.dont_write_bytecode=True
native_uname=os.uname()
platform.uname=lambda:platform.uname_result(*(tuple(native_uname)+(native_uname.machine,)))
platform.architecture=lambda *a,**k:(str(8*struct.calcsize('P'))+'bit','ELF')
def audit_event(event,args):
 if event.startswith('socket.') or event in ('subprocess.Popen','os.system','os.exec','os.posix_spawn','os.fork','os.forkpty'):
  attempts.append(event);raise RuntimeError('offline process/network boundary: '+event)
 if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)):
  name=os.fsdecode(args[0]);flags=args[2]if len(args)>2 and isinstance(args[2],int)else 0
  mode=args[1]if len(args)>1 and isinstance(args[1],str)else ''
  if (name.startswith('/dev/')and name not in ('/dev/null','/dev/urandom','/dev/random'))or '/hwmon/'in name or name.startswith('/sys/class/thermal'):
   attempts.append('device:'+name);raise RuntimeError('offline device boundary')
  if name!='/dev/null'and(flags&(os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC|os.O_APPEND)or any(k in mode for k in 'wax+')):
   try:Path(name).resolve().relative_to(O.resolve())
   except ValueError:attempts.append('write:'+name);raise RuntimeError('offline write boundary')
sys.addaudithook(audit_event)
import rosbag
import rospy
def blocked(*args,**kwargs):
 attempts.append('ROS_API');raise RuntimeError('offline ROS API boundary')
for api in ('init_node','Publisher','Subscriber','Service','ServiceProxy','wait_for_service','get_param','set_param','delete_param'):
 setattr(rospy,api,blocked)

def sha(path):
 h=hashlib.sha256()
 with Path(path).open('rb')as f:
  for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
 return h.hexdigest()

def payload(v):
 if hasattr(v,'__slots__'):return {k:payload(getattr(v,k))for k in v.__slots__}
 if isinstance(v,(tuple,list)):return [payload(x)for x in v]
 if isinstance(v,bytes):return {'bytes_base64':base64.b64encode(v).decode()}
 if isinstance(v,float)and not math.isfinite(v):return {'original_nonfinite_repr':repr(v)}
 return v

def save(path,value):
 path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
 assert json.loads(path.read_text())==value

def main():
 assert Path('/media/zhuyupei/SanStand').is_mount()and not O.exists()
 baseline=W/'.runtime/gen49_baseline_source/manifest.json'
 assert sha(baseline)=='21f6c6a1b0b8072d78332a081af1a425c570c9e5dd1983b2f982960e3911e935'
 m=json.loads((G/'manifest.json').read_text())
 assert m['state']=='complete'and m['recorder_closed']is True and m['result']=='failure'
 brow=next(x for x in m['files']if x['path']=='keyframes.bag')
 assert (G/'keyframes.bag').stat().st_size==brow['size_bytes']and sha(G/'keyframes.bag')==brow['sha256']
 O.mkdir();(O/'tmp').mkdir();os.environ['TMPDIR']=str(O/'tmp')
 import tempfile
 tempfile.tempdir=str(O/'tmp')
 rows={t:[]for t in TOPICS};n=0
 with rosbag.Bag(str(G/'keyframes.bag'),'r')as bag:
  available=bag.get_type_and_topic_info().topics
  selected={t:dict(type=available[t].msg_type,message_count=available[t].message_count)for t in TOPICS if t in available}
  assert not any('Image'in v['type']or'temperature'in t.lower()for t,v in selected.items())
  for topic,msg,stamp in bag.read_messages(topics=list(TOPICS)):
   assert topic in TOPICS and msg._type not in ('sensor_msgs/Image','sensor_msgs/CompressedImage')
   stream=io.BytesIO();msg.serialize(stream);wire=stream.getvalue()
   body=payload(msg);parsed=None
   if msg._type=='std_msgs/String':
    try:parsed=json.loads(msg.data)
    except (ValueError,TypeError):pass
   rows[topic].append(dict(received_ns=stamp.to_nsec(),message_type=msg._type,md5sum=msg._md5sum,
    wire_sha256=hashlib.sha256(wire).hexdigest(),wire_base64=base64.b64encode(wire).decode(),payload=body,parsed_json=parsed))
   n+=1
 files={}
 for i,topic in enumerate(TOPICS):
  p=O/('%02d_%s.json'%(i,topic.strip('/').replace('/','__')))
  save(p,dict(topic=topic,messages=rows[topic]));files[topic]=dict(path=str(p),sha256=sha(p),messages=len(rows[topic]))
 assert not attempts and sha(G/'keyframes.bag')==brow['sha256']
 result=dict(passed=True,kind='sealed_G48_exact_nonimage_owner_messages',created_ns=time.time_ns(),record=str(G),
  manifest_sha256=sha(G/'manifest.json'),bag_sha256=brow['sha256'],bag_size_bytes=brow['size_bytes'],
  bag_before_after_same=True,explicit_topic_whitelist=list(TOPICS),recorded_selected_topic_metadata=selected,
  extracted_count=n,files=files,helper_sha256=sha(__file__),external_boundary_attempts=attempts,
  ROS_initialized=False,hardware_contacted=False,image_messages_read=False,temperature_messages_read=False,
  production_or_tests_or_record_modified=False)
 save(O/'result.json',result);(O/'helper_source.py').write_bytes(Path(__file__).read_bytes())
 print(json.dumps(dict(result=str(O/'result.json'),sha256=sha(O/'result.json'),extracted_count=n,
  topic_counts={k:len(v)for k,v in rows.items()}),ensure_ascii=False))
if __name__=='__main__':main()
