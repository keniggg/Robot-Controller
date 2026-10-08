#!/usr/bin/env python3
"""Offline same-source RGBD / accepted FK consistency at actual grasp stages."""
import json,hashlib,bisect,importlib.util
from pathlib import Path
import numpy as np,cv2,rosbag
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.vision.object_geometry import _fit_support_plane
p=Path(json.load(open('.runtime/current_grasp_diagnostic.json'))['directory']);root=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20260928T084603.735476Z_0270fad562cf46a9ae14b8c203da5bc1');audit=root/'keyframes_audits/25c56fe78f206f38bacc6206d71c852c09ba3a962681ff5c4402dfb1e89de1ec.json';a=json.loads(audit.read_text());context=json.loads((p/'initial_context.json').read_text());camera=context['camera'];he=context['handeye'];urdf=(p/'robot_description.urdf').read_text();fk=SerialUrdfFk(urdf,['Joint%d'%i for i in range(1,7)])
spec=importlib.util.spec_from_file_location('cam','src/real-arm/alicia_d_calibration/scripts/charuco_tracker.py');cm=importlib.util.module_from_spec(spec);spec.loader.exec_module(cm);model=cm.RuntimeColorModel(cm.runtime_color_model_evidence(camera,'camera_link','/supervisor/camera/color/image_raw'))
H=quaternion_matrix(he['rotation_xyzw']);H[:3,3]=he['translation_xyz'];O=np.eye(4);O[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
C=np.array(a['replay_geometry']['obb_center_base_m']);n0=np.array(a['replay_geometry']['support_normal_base']);d0=a['replay_geometry']['support_offset_m'];anchor=C-(C@n0+d0)*n0
wanted={'initial':a['snapshot_stamp_ns']/1e9,'pregrasp_mid':1790585261.,'pregrasp_settled':1790585284.5,'pregrasp_compensated':1790585316.,'approach_settled':1790585347.,'before_close':1790585354.35,'after_close':1790585355.3,'after_lift':1790585373.,'terminal':1790585377.5}
RGB='/supervisor/camera/color/image_raw';DEPTH='/supervisor/camera/depth/image_raw';best={};joints=[];sdk=[];epochs=[]
with rosbag.Bag(str(p/'scene_diagnostic.bag')) as bag:
 for topic,m,t in bag.read_messages(topics=[RGB,DEPTH,'/alicia_d/accepted_joint_states','/alicia_d/control_reference','/alicia_d/control_reference_epoch']):
  if topic in (RGB,DEPTH):
   ts=m.header.stamp.to_nsec()
   for label,goal in wanted.items():
    dist=abs(ts-round(goal*1e9));key=(label,topic)
    if key not in best or dist<best[key][0]:best[key]=(dist,m)
  elif topic.endswith('epoch'):epochs.append((t.to_sec(),m.stamp.to_nsec()))
  else:
   v=dict(zip(m.name,m.position));row=(m.header.stamp.to_sec(),np.array([v['Joint%d'%i] for i in range(1,7)]),float(v['right_finger']));(joints if topic.endswith('joint_states') else sdk).append(row)
joints.sort(key=lambda x:x[0]);sdk.sort(key=lambda x:x[0]);jt=np.array([r[0] for r in joints]);jq=np.array([r[1] for r in joints]);st=np.array([r[0] for r in sdk]);bridge=CvBridge();rows=[];clouds={};out=p/'geometry_forensics';out.mkdir(exist_ok=True)
for label,goal in wanted.items():
 color=best[(label,RGB)][1];depth=best[(label,DEPTH)][1];assert color.header.stamp==depth.header.stamp
 ns=depth.header.stamp.to_nsec();ts=ns/1e9;ii=int(np.argmin(abs(jt-ts)));lo=max(0,bisect.bisect_right(jt,ts)-1);hi=min(len(jt)-1,lo+1)
 if abs(jt[ii]-ts)>.12 or jt[hi]-jt[lo]>.25:raise RuntimeError('joint association unavailable')
 q=np.array([np.interp(ts,jt,jq[:,k]) for k in range(6)]);T=fk(q)@H@O;j=int(np.argmin(abs(st-ts)));S=fk(sdk[j][1]);F=fk(q)
 im=bridge.imgmsg_to_cv2(color,'bgr8');D=bridge.imgmsg_to_cv2(depth,'passthrough');cv2.imwrite(str(out/(label+'_rgb.png')),im)
 hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV);z=D*camera['depth_scale'];mask=(hsv[:,:,1]>100)&(hsv[:,:,2]>60)&((hsv[:,:,0]<35)|(hsv[:,:,0]>165))&(z>.03)&(z<.8)
 yy,xx=np.where(mask);take=np.arange(0,len(xx),max(1,len(xx)//6000));xx=xx[take];yy=yy[take];zz=z[yy,xx]
 rays=np.column_stack([model.normalized_points(np.column_stack([xx,yy])).reshape(-1,2),np.ones(len(xx))]);P=(rays*zz[:,None])@T[:3,:3].T+T[:3,3];clouds[label]=P
 local=jq[abs(jt-ts)<=.3];span=np.ptp(local,axis=0) if len(local)>1 else None
 optical=(C-T[:3,3])@T[:3,:3];uv=model.project([optical],[0.,0.,0.],[0.,0.,0.])[0]
 row=dict(label=label,goal_stamp=goal,source_stamp_ns=ns,rgb_depth_exact=True,source_minus_goal_sec=ts-goal,accepted_nearest_delta_sec=jt[ii]-ts,accepted_bracket_delta_sec=[jt[lo]-ts,jt[hi]-ts],joint_span_in_halfsecond_rad=span.tolist() if span is not None else None,stationary_around_image=bool(span is not None and max(span)<=.0025),q_rad=q.tolist(),sdk_q_rad=sdk[j][1].tolist(),sdk_source_stamp_sec=st[j],sdk_to_actual_tool_xyz_mm=((F[:3,3]-S[:3,3])*1000).tolist(),sdk_to_actual_tool_normal_mm=float((F[:3,3]-S[:3,3])@n0*1000),T_base_optical=T.tolist(),frozen_target_uv=uv.tolist(),target_inside_image=bool(0<=uv[0]<640 and 0<=uv[1]<480),red_points=len(P),fits=[])
 for name,points in [('all_visible_red',P),('near_frozen_target_xy_100mm',P[np.linalg.norm(P[:,:2]-anchor[:2],axis=1)<.1])]:
  try:
   n,d,frac=_fit_support_plane(points,.002,180)
   if n[2]<0:n=-n;d=-d
   residual=points@n+d;inlier=abs(residual)<=.002
   row['fits'].append(dict(kind=name,points=len(points),n=n.tolist(),d=float(d),inlier_fraction=float(frac),rms_mm=float(np.sqrt(np.mean(residual[inlier]**2))*1000),table_z_at_frozen_anchor_mm=float(-(d+n[:2]@anchor[:2])/n[2]*1000),normal_distance_at_frozen_anchor_mm=float((n@anchor+d)*1000),angle_from_frozen_deg=float(np.degrees(np.arccos(np.clip(n@n0,-1,1))))))
  except Exception as e:row['fits'].append(dict(kind=name,error=str(e),points=len(points)))
 rows.append(row);print(json.dumps(dict(label=label,stationary=row['stationary_around_image'],sdk_to_actual_normal_mm=row['sdk_to_actual_tool_normal_mm'],fits=row['fits']),ensure_ascii=False),flush=True)
# Compare only a common observed XY patch, centered on the near-contact visible table.
patches=[]
for label in ['pregrasp_settled','pregrasp_compensated','approach_settled','before_close','after_close','after_lift']:
 late=clouds[label];initial=clouds['initial'];center=np.median(late[:,:2],axis=0)
 for radius in [.05,.08,.10]:
  patch=dict(stage=label,center_xy_base_m=center.tolist(),radius_m=radius,fits={})
  for key,xyz in [('initial',initial),(label,late)]:
   chosen=xyz[np.linalg.norm(xyz[:,:2]-center,axis=1)<radius]
   try:
    n,d,frac=_fit_support_plane(chosen,.002,150)
    if n[2]<0:n=-n;d=-d
    residual=chosen@n+d;ins=abs(residual)<=.002
    patch['fits'][key]=dict(points=len(chosen),n=n.tolist(),d=float(d),inlier_fraction=float(frac),rms_mm=float(np.sqrt(np.mean(residual[ins]**2))*1000),z_at_common_center_mm=float(-(d+n[:2]@center)/n[2]*1000))
   except Exception as e:patch['fits'][key]=dict(points=len(chosen),error=str(e))
  if all('z_at_common_center_mm' in v for v in patch['fits'].values()):patch['later_minus_initial_z_mm']=patch['fits'][label]['z_at_common_center_mm']-patch['fits']['initial']['z_at_common_center_mm']
  patches.append(patch)
report=dict(schema=1,scope='offline_diagnostic_no_motion_no_calibration_change',record=str(root),audit_sha256=hashlib.sha256(audit.read_bytes()).hexdigest(),urdf_sha256=hashlib.sha256(urdf.encode()).hexdigest(),camera_fingerprint=model.fingerprint,operator_result='小电机仍在桌面，没有夹住',frozen_center=C.tolist(),frozen_support=dict(n=n0.tolist(),d=d0,anchor=anchor.tolist(),z_at_anchor_mm=float(anchor[2]*1000)),epochs=epochs,frames=rows,common_patch_comparisons=patches,limits=['Base reconstruction retains current handeye and measured FK; not independent physical metrology.','Common patches refer to overlapping visible red tabletop XY under the same model, not identified individual material points.','Moving frames are diagnostic only; causality conclusions require stationary windows.','Plane heights at off-image frozen target are extrapolation; common visible patch comparisons provided separately.'])
(out/'geometry_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print('REPORT '+str(out/'geometry_report.json'),flush=True)
