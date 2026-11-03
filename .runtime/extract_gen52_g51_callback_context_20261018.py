"""Exact camera configuration and accepted SDK feedback for station callback replay."""
import base64, hashlib, io, json, os, platform, struct, sys, time
import extract_gen52_g51_station_wire_20261018 as p
import extract_gen50_retry_r30_rgb_20261018 as boundary
OUT=p.OUT/'callback_context_r1'
TOPICS=('/alicia_d/accepted_joint_states','/grasp_record/camera_runtime_metadata','/alicia_d/following_epoch')

def main():
 assert p.E.parent.parent.is_mount() and not OUT.exists(); OUT.mkdir()
 os.environ.update(PYTHONDONTWRITEBYTECODE='1',ROS_MASTER_URI='http://127.0.0.1:1',ROS_LOG_DIR=str(OUT),ROS_HOME=str(OUT),TMPDIR=str(OUT))
 sys.dont_write_bytecode=True
 sys.path[:0]=[str(p.W/'devel/lib/python3/dist-packages'),'/opt/ros/noetic/lib/python3/dist-packages']
 n=os.uname();platform.uname=lambda:platform.uname_result(*(tuple(n)+(n.machine,)));platform.architecture=lambda *a,**k:(str(8*struct.calcsize('P'))+'bit','ELF')
 boundary.OUT=OUT;sys.addaudithook(boundary.audit)
 import rosbag
 m=json.loads((p.G/'manifest.json').read_text());b=next(x for x in m['files']if x['path']=='keyframes.bag');bp=p.G/'keyframes.bag';assert boundary.sha(bp)==b['sha256'] and m['recorder_closed']
 rows={t:[]for t in TOPICS}
 with rosbag.Bag(str(bp),'r')as bag:
  info=bag.get_type_and_topic_info().topics
  for topic,msg,recv in bag.read_messages(topics=[t for t in TOPICS if t in info]):
   if topic==TOPICS[0] and not 1792329984000000000<=recv.to_nsec()<=1792330055200000000:continue
   raw=io.BytesIO();msg.serialize(raw);parsed=None
   if msg._type=='std_msgs/String':
    try:parsed=json.loads(msg.data)
    except ValueError:pass
   rows[topic].append(dict(received_ns=recv.to_nsec(),message_type=msg._type,md5sum=msg._md5sum,wire_sha256=hashlib.sha256(raw.getvalue()).hexdigest(),wire_base64=base64.b64encode(raw.getvalue()).decode(),payload=p.plain(msg),parsed_json=parsed))
 files={}
 for i,(topic,rr) in enumerate(rows.items()):
  path=OUT/('%d.json'%i);p.put(path,dict(topic=topic,messages=rr));files[topic]=dict(path=str(path),sha256=boundary.sha(path),count=len(rr))
 assert not boundary.BOUNDARY and boundary.sha(bp)==b['sha256']
 result=dict(passed=True,files=files,topics_read=list(TOPICS),bag_sha256=b['sha256'],bag_before_after_same=True,external_boundary_attempts=boundary.BOUNDARY,temperature_read=False,ROS_initialized=False,hardware_contacted=False,helper_sha256=boundary.sha(__file__))
 p.put(OUT/'result.json',result);print(json.dumps(dict(result=str(OUT/'result.json'),sha256=boundary.sha(OUT/'result.json'),counts={t:len(v)for t,v in rows.items()}),ensure_ascii=False))
if __name__=='__main__':main()
