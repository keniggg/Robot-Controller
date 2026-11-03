"""Compare support estimates on frozen depth, without changing runtime."""
import json,pathlib,rosbag,numpy as np
from cv_bridge import CvBridge
from types import SimpleNamespace as NS
from alicia_flexible_grasp.vision.object_geometry import _expanded_bbox_mask,deproject_depth,_fit_support_plane
p=pathlib.Path(json.load(open('.runtime/current_handeye_deployment.json'))['directory']);bridge=CvBridge();results=[]
cases=[('origin','second_grasp_planning_audit.json','20260927T114531.914586Z_2ec803d779064ce1ac4b4a3be510a089'),('observation','current_observation_direct_audit.json','20260927T115956.474759Z_6ae3f12483654949a1e5b2c72f6d831b')]
for label,afile,record in cases:
 a=json.load(open(p/afile));stamp=a['snapshot_stamp_ns'];best=None;bestdelta=10**40
 path=pathlib.Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records')/record/'keyframes.bag'
 with rosbag.Bag(str(path)) as bag:
  for topic,msg,t in bag.read_messages(topics=['/supervisor/camera/depth/image_raw']):
   delta=abs(msg.header.stamp.to_nsec()-stamp)
   if delta<bestdelta:best=msg;bestdelta=delta
 d=bridge.imgmsg_to_cv2(best,'passthrough');np.save(p/(label+'_support_replay_depth.npy'),d)
 K=NS(width=d.shape[1],height=d.shape[0],fx=395.7682800292969,fy=395.6027526855469,cx=317.9961242675781,cy=244.833984375)
 T=np.array(a['snapshot_transform']['T_base_optical']);bbox=a['graspnet_input']['detected_bbox_xywh'];original=np.array(a['replay_geometry']['support_normal_base']);center=np.array(a['target_cloud_center_base'])
 row=dict(label=label,audit_stamp_ns=stamp,frame_stamp_ns=best.header.stamp.to_nsec(),same_stationary_pose=True,time_delta_sec=bestdelta/1e9,bbox=bbox,original_normal=original.tolist(),fits=[])
 for ratio in (.3,.6,1.,1.5,2.5,4.):
  expanded,bm=_expanded_bbox_mask(d.shape,bbox,ratio);pts,_=deproject_depth(d,expanded & ~bm,K,9.999999747378752e-05);pts=pts@T[:3,:3].T+T[:3,3]
  n,off,frac=_fit_support_plane(pts,.0035,100)
  if n[2]<0:n=-n;off=-off
  resid=pts@n+off;err=np.abs(resid);inlier=err<=.0035
  fit=dict(expand_ratio=ratio,points=len(pts),inlier_ratio=frac,normal=n.tolist(),offset_m=off,normal_vs_original_deg=np.degrees(np.arccos(np.clip(n@original,-1,1))),signed_target_center_height_m=float(center@n+off),inlier_rms_m=float(np.sqrt(np.mean(resid[inlier]**2))))
  row['fits'].append(fit)
  print(label,ratio,'n',np.round(n,5),'angle',round(fit['normal_vs_original_deg'],3),'height',round(fit['signed_target_center_height_m']*1000,3),'rms',round(fit['inlier_rms_m']*1000,3),flush=True)
 results.append(row)
(p/'support_context_replay.json').write_text(json.dumps(results,indent=2)+'\n')
