#!/usr/bin/env python3
import json, time, threading, importlib.util, collections
from pathlib import Path
import rospy, cv2, numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
root=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
out=Path(json.loads((root/'.runtime/current_grasp_repair_resume.json').read_text())['directory'])
rospy.init_node('readonly_tracker_restart_profile',anonymous=True,disable_signals=True)
lock=threading.Lock(); rows=[]; last=[None]; bridge=CvBridge()
def rgb(m):
 n=rospy.Time.now().to_nsec()
 with lock: rows.append(dict(kind='rgb',stamp=m.header.stamp.to_nsec(),age_ms=(n-m.header.stamp.to_nsec())/1e6));last[0]=m
def quality(m):
 q=json.loads(m.data);n=rospy.Time.now().to_nsec()
 with lock: rows.append(dict(kind='quality',stamp=q['source_stamp_ns'],age_ms=(n-q['source_stamp_ns'])/1e6,pose_ok=q.get('pose_ok'),corners=q.get('charuco_corners'),model_error=q.get('runtime_model_error')))
subs=[rospy.Subscriber('/supervisor/camera/color/image_raw',Image,rgb,queue_size=1,buff_size=2**24,tcp_nodelay=True),rospy.Subscriber('/charuco/quality',String,quality,queue_size=30)]
time.sleep(8)
for s in subs:s.unregister()
(out/'tracker_restart_source_latency.json').write_text(json.dumps(rows,indent=2))
image=bridge.imgmsg_to_cv2(last[0],'bgr8'); cv2.imwrite(str(out/'tracker_restart_source_rgb.png'),image)
spec=importlib.util.spec_from_file_location('tracker',root/'src/real-arm/alicia_d_calibration/scripts/charuco_tracker.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
def make():
 tr=mod.CharucoTracker.__new__(mod.CharucoTracker);tr.dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100);tr.board=cv2.aruco.CharucoBoard((12,9),.015,.01125,tr.dictionary);tr.board.setLegacyPattern(True);tr.detection_scale=1.5;tr.detection_roi_padding_px=45.;tr.detection_roi=None
 return tr
profile=[]
original=cv2.getNumThreads()
for threads in [original,1,2,original,1]:
 cv2.setNumThreads(threads);tr=make();samples=[]
 for i in range(12):
  st=time.perf_counter();c,ids=tr.detect_charuco_board(image);samples.append(dict(ms=(time.perf_counter()-st)*1000,corners=0 if ids is None else len(ids),roi=tr.detection_roi))
 profile.append(dict(threads=threads,samples=samples))
cv2.setNumThreads(original)
def stats(v):return dict(n=len(v),min=min(v),median=float(np.median(v)),p95=float(np.percentile(v,95)),max=max(v)) if v else None
report=dict(opencv=cv2.__version__,opencv_default_threads=original,latency={k:stats([r['age_ms'] for r in rows if r['kind']==k]) for k in ['rgb','quality']},quality_pose_ok=sum(r.get('pose_ok',False) for r in rows),profile=profile)
(out/'tracker_restart_profile.json').write_text(json.dumps(report,indent=2));print(json.dumps({**{k:v for k,v in report.items() if k!='profile'},'profile_summary':[dict(threads=p['threads'],all_ms=stats([x['ms'] for x in p['samples']]),warm_ms=stats([x['ms'] for x in p['samples'][1:]]),corners=list(set(x['corners'] for x in p['samples']))) for p in profile]},indent=2))
