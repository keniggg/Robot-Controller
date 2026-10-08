#!/usr/bin/env python3
import json,time,threading,importlib.util,collections,sys
from pathlib import Path
import rospy,rosgraph,rosbag
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
root=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
out=Path(json.loads((root/'.runtime/current_grasp_repair_resume.json').read_text())['directory'])
spec=importlib.util.spec_from_file_location('cam',root/'src/real-arm/alicia_d_calibration/scripts/charuco_tracker.py');cam=importlib.util.module_from_spec(spec);spec.loader.exec_module(cam)
rospy.init_node('readonly_board_pair_latency_review',anonymous=True,disable_signals=True)
lock=threading.RLock();rgb=collections.OrderedDict();board={};quality={};rows=[];calls=[]
def recv_rgb(m):
 with lock:
  ns=m.header.stamp.to_nsec();rgb[ns]=time.time_ns();
  while len(rgb)>12:rgb.popitem(last=False)
def recv_tf(m):
 with lock:
  for t in m.transforms:
   if t.header.frame_id=='camera_link':board[(t.header.stamp.to_nsec(),t.child_frame_id)]=time.time_ns()
def recv_q(m):
 q=json.loads(m.data);ns=q.get('source_stamp_ns');now=rospy.Time.now().to_nsec()
 with lock:
  quality[ns]=q
  rows.append(dict(source_stamp_ns=ns,receipt_stamp_ns=now,receipt_age_ms=(now-ns)/1e6,pose_ok=q['pose_ok'],has_rgb_12=ns in rgb,has_tf=(ns,q['board_frame']) in board,rgb_oldest_age_ms=(now-next(iter(rgb)))/1e6 if rgb else None,quality=q))
subs=[rospy.Subscriber('/supervisor/camera/color/image_raw',Image,recv_rgb,queue_size=4,buff_size=2**24),rospy.Subscriber('/charuco/quality',String,recv_q,queue_size=50),rospy.Subscriber('/tf',TFMessage,recv_tf,queue_size=200)]
end=time.monotonic()+8
while time.monotonic()<end:
 t0=time.monotonic();parameters=cam.read_runtime_camera_parameters('/camera',rospy.get_name(),rosgraph.get_master_uri());t1=time.monotonic();e=cam.runtime_color_model_evidence(parameters,'camera_link','/supervisor/camera/color/image_raw');t2=time.monotonic();model=cam.RuntimeColorModel(e);t3=time.monotonic();now=rospy.Time.now().to_nsec()
 with lock:
  opts=[]
  for ns,q in quality.items():
   if q.get('pose_ok') and ns in rgb and (ns,q['board_frame']) in board and 0<=now-ns<=350000000 and q['camera_model_fingerprint']==model.fingerprint:opts.append(ns)
  calls.append(dict(param_ms=(t1-t0)*1000,evidence_ms=(t2-t1)*1000,model_ms=(t3-t2)*1000,candidate_count=len(opts),newest_quality_age_ms=(now-max(quality))/1e6 if quality else None,fingerprint=model.fingerprint))
 time.sleep(.03)
for s in subs:s.unregister()
bagpath=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/bounded_joint_response_20260928T104443Z_12672/evidence.bag')
bq=[];bd=set();bt=set()
with rosbag.Bag(str(bagpath)) as bag:
 for topic,m,t in bag.read_messages(topics=['/charuco/quality','/tf','/supervisor/camera/depth/image_raw']):
  if topic=='/charuco/quality':q=json.loads(m.data);bq.append(dict(source_stamp_ns=q['source_stamp_ns'],receipt_age_ms=(t.to_nsec()-q['source_stamp_ns'])/1e6,quality=q))
  elif topic=='/tf':
   for tr in m.transforms:
    if tr.header.frame_id=='camera_link':bt.add((tr.header.stamp.to_nsec(),tr.child_frame_id))
  else:bd.add(m.header.stamp.to_nsec())
for r in bq:
 q=r['quality'];r['has_exact_depth_stamp']=q['source_stamp_ns'] in bd;r['has_exact_tf_stamp']=(q['source_stamp_ns'],q['board_frame']) in bt
payload=dict(live_quality_rows=rows,live_model_calls=calls,bag_quality_rows=bq,readonly=True)
(out/'board_pair_latency_review.json').write_text(json.dumps(payload,ensure_ascii=False,indent=2))
def stats(xs):
 return dict(n=len(xs),minimum=min(xs) if xs else None,maximum=max(xs) if xs else None,mean=sum(xs)/len(xs) if xs else None)
print(json.dumps(dict(output=str(out/'board_pair_latency_review.json'),quality_count=len(rows),valid_quality=sum(r['pose_ok'] for r in rows),quality_age_ms=stats([r['receipt_age_ms'] for r in rows]),valid_missing_rgb=sum(r['pose_ok'] and not r['has_rgb_12'] for r in rows),valid_missing_tf_at_quality_receive=sum(r['pose_ok'] and not r['has_tf'] for r in rows),param_ms=stats([r['param_ms'] for r in calls]),model_ms=stats([r['model_ms'] for r in calls]),candidate_checks_pass=sum(r['candidate_count']>0 for r in calls),candidate_checks=len(calls),fingerprints=list(set(r['fingerprint'] for r in calls)),quality_fingerprints=list(set(r['quality']['camera_model_fingerprint'] for r in rows)),bag=[dict([(k,v) for k,v in r.items() if k!='quality']+[('pose_ok',r['quality']['pose_ok'])]) for r in bq]),ensure_ascii=False,indent=2))
