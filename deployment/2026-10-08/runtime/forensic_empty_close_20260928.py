"""Offline recorded RGB-D/accepted encoder consistency; no ROS I/O."""
import json,pathlib,hashlib,bisect
from types import SimpleNamespace as NS
import numpy as np,rosbag,cv2
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.vision.object_geometry import deproject_depth,_fit_support_plane
root=pathlib.Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20260928T032911.480689Z_b748c67fddc14632870c1c70fa5b97f9')
out=pathlib.Path(json.load(open('.runtime/current_resumed_grasp_session.json'))['directory'])/'empty_close_forensics';out.mkdir(exist_ok=True)
audit=root/'keyframes_audits/632a73f35a63335c073ea9f16b55e85e44acc1d9b7aebaf3eb0737510aa6326e.json';a=json.loads(audit.read_text())
f=json.load(open('src/alicia_flexible_grasp_supervisor/tests/fixtures/measured_contact_20260926.json'));fk=SerialUrdfFk(f['robot_description'],['Joint%d'%i for i in range(1,7)])
assert f['model_sha256']=='890592a24b8ad818d995e69beb670ea4254bf84f589615bfac3c8562f62449b7'
he=json.load(open('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/handeye_auto_20260927T102700Z/deployment_20260927T111819Z/deployment.json'))['new_runtime_handeye']
H=quaternion_matrix(he['rotation_xyzw']);H[:3,3]=he['translation_xyz'];O=np.eye(4);O[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
ref=a['replay_geometry'];n0=np.array(ref['support_normal_base']);d0=ref['support_offset_m'];C=np.array(ref['obb_center_base_m']);plane_anchor=C-(C@n0+d0)*n0
K=NS(width=640,height=480,fx=395.7682800292969,fy=395.6027526855469,cx=317.9961242675781,cy=244.833984375);scale=9.999999747378752e-05
wanted=np.array([1790566160.674229621,1790566201.,1790566208.,1790566215.,1790566223.,1790566230.,1790566246.,1790566278.,1790566309.,1790566318.5])
image_topics=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/perception/object_mask']
chosen={t:[None]*len(wanted) for t in image_topics};dist={t:np.full(len(wanted),np.inf) for t in image_topics};joints=[];objects=[];geometry=[];bridge=CvBridge()
with rosbag.Bag(str(root/'keyframes.bag')) as bag:
 for topic,m,s in bag.read_messages(topics=image_topics+['/alicia_d/accepted_joint_states','/perception/object','/grasp_6d/object_geometry']):
  stamp=m.header.stamp.to_sec()
  if topic in chosen:
   ds=abs(wanted-stamp)
   for i in np.flatnonzero(ds<dist[topic]):chosen[topic][i]=m;dist[topic][i]=ds[i]
  elif topic=='/alicia_d/accepted_joint_states':
   v=dict(zip(m.name,m.position));joints.append((stamp,[v['Joint%d'%i] for i in range(1,7)]))
  elif topic=='/perception/object' and m.detected:
   p=m.pose_base.pose.position;objects.append(dict(stamp=stamp,xyz=[p.x,p.y,p.z],uv=[m.u,m.v],depth=m.depth_m,bbox=[m.bbox_x,m.bbox_y,m.bbox_width,m.bbox_height]))
  elif topic=='/grasp_6d/object_geometry' and m.valid:
   p=m.pose_base.position;n=m.support_normal_base;geometry.append(dict(stamp=stamp,center=[p.x,p.y,p.z],normal=[n.x,n.y,n.z],offset=m.support_offset_m))
print('EXTRACTED',len(joints),'feedback',len(objects),'objects',flush=True)
joints.sort();times=np.array([r[0] for r in joints]);values=np.array([r[1] for r in joints]);reports=[]
for i,w in enumerate(wanted):
 depth=chosen[image_topics[1]][i];stamp=depth.header.stamp.to_sec();q=np.array([np.interp(stamp,times,values[:,j]) for j in range(6)]);T=fk(q)@H@O
 nearest=int(np.argmin(abs(times-stamp)));D=bridge.imgmsg_to_cv2(depth,'passthrough');rgb=bridge.imgmsg_to_cv2(chosen[image_topics[0]][i],'bgr8');M=bridge.imgmsg_to_cv2(chosen[image_topics[2]][i],'mono8')
 tag='%02d'%i;cv2.imwrite(str(out/(tag+'_rgb.jpg')),rgb);np.savez_compressed(out/(tag+'_rgbd.npz'),depth=D,mask=M,joints=q,T_base_optical=T)
 Pc=(C-T[:3,3])@T[:3,:3];uv=[K.fx*Pc[0]/Pc[2]+K.cx,K.fy*Pc[1]/Pc[2]+K.cy]
 r=dict(index=i,image=str(out/(tag+'_rgb.jpg')),stamp=stamp,seconds_since_snapshot=stamp-a['snapshot_stamp_ns']/1e9,depth_stamp_ns=depth.header.stamp.to_nsec(),color_stamp_ns=chosen[image_topics[0]][i].header.stamp.to_nsec(),feedback_nearest_age_s=abs(times[nearest]-stamp),q=q.tolist(),T_base_optical=T.tolist(),frozen_target_projected_uv=uv,fits=[])
 if i==0:r['snapshot_transform_max_difference']=float(np.max(abs(T-np.array(a['snapshot_transform']['T_base_optical']))))
 for kind in ['central_visible_table','near_frozen_target_projection']:
  sm=np.zeros(D.shape,bool)
  if kind=='central_visible_table':sm[60:420:4,80:560:4]=True
  else:
   u,v=np.round(uv).astype(int);x0,x1=max(10,u-75),min(630,u+75);y0,y1=max(10,v-75),min(470,v+75)
   if x1-x0<50 or y1-y0<50:continue
   sm[y0:y1:2,x0:x1:2]=True
  sm &= (M==0)
  try:
   P,_=deproject_depth(D,sm,K,scale);P=P@T[:3,:3].T+T[:3,3];n,d,frac=_fit_support_plane(P,.0035,200)
   if n@n0<0:n=-n;d=-d
   residual=P@n+d;inside=abs(residual)<=.0035
   fit=dict(kind=kind,n=n.tolist(),d=d,inlier_ratio=frac,points=len(P),inlier_rms_m=float(np.sqrt(np.mean(residual[inside]**2))),angle_from_frozen_deg=float(np.degrees(np.arccos(np.clip(n@n0,-1,1)))),offset_at_frozen_plane_anchor_m=float(plane_anchor@n+d),frozen_target_height_above_current_plane_m=float(C@n+d))
  except Exception as exc:fit=dict(kind=kind,error=str(exc))
  r['fits'].append(fit)
 nearobj=min(objects,key=lambda x:abs(x['stamp']-stamp))
 if abs(nearobj['stamp']-stamp)<.7:r['detected_object']=nearobj;r['detected_vs_frozen_center_m']=(np.array(nearobj['xyz'])-C).tolist()
 reports.append(r)
 print('FRAME',i,'t',round(r['seconds_since_snapshot'],2),'q',np.round(q,3).tolist(),'fits',json.dumps(r['fits']),flush=True)
report=dict(scope='offline_recorded_RGBD_encoder_consistency_diagnostic_only',operator_observation='没有夹住，仍在桌面上；随后照片显示两指未包住电机两侧、闭合位置偏高',source_record=str(root),source_audit_sha256=hashlib.sha256(audit.read_bytes()).hexdigest(),model_sha256=f['model_sha256'],frozen_support_normal=n0.tolist(),frozen_support_offset_m=d0,plane_anchor=plane_anchor.tolist(),samples=reports)
(out/'recorded_plane_consistency.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
(out/'object_motion_series.json').write_text(json.dumps(dict(objects=objects,geometry=geometry),indent=2)+'\n')
print('REPORT',out/'recorded_plane_consistency.json',flush=True)
