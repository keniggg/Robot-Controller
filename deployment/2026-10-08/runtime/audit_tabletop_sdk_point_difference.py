"""Closed-file/tabletop-vs-SDK ray audit. Never creates a camera or ROS node."""
import hashlib,json,math,os
from pathlib import Path
import numpy as np
import pyrealsense2 as rs
import rosbag,rospy

ROOT=Path(__file__).resolve().parents[1]
ARCHIVE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')
OUT=ARCHIVE/'grasp_repair_resume_20260928T120606Z/tabletop_sdk_ray_audit'
O=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])


def sdk_intrinsics(p):
 i=rs.intrinsics()
 for k in ('width','height','fx','fy'):setattr(i,k,p[k])
 i.ppx,i.ppy=p['cx'],p['cy'];i.model=getattr(rs.distortion,p['model']);i.coeffs=p['coeffs'];return i


def point_comparison(pixel,z,p,T):
 u,v=map(float,pixel)
 if not np.isfinite(z) or z<=0:return dict(pixel_uv=[u,v],valid=False,depth_m=float(z))
 pinhole=np.array([(u-p['cx'])*z/p['fx'],(v-p['cy'])*z/p['fy'],z])
 sdk=np.array(rs.rs2_deproject_pixel_to_point(sdk_intrinsics(p),[u,v],float(z)))
 dc=O@(sdk-pinhole);db=T[:3,:3]@dc
 return dict(pixel_uv=[u,v],valid=True,depth_m=float(z),pinhole_optical_m=pinhole.tolist(),sdk_optical_m=sdk.tolist(),
  sdk_minus_pinhole_optical_mm=((sdk-pinhole)*1000).tolist(),sdk_minus_pinhole_camera_link_mm=(dc*1000).tolist(),
  sdk_minus_pinhole_base_mm=(db*1000).tolist(),norm_mm=float(np.linalg.norm(dc)*1000),
  pinhole_base_m=(T[:3,:3]@(O@pinhole)+T[:3,3]).tolist(),sdk_base_m=(T[:3,:3]@(O@sdk)+T[:3,3]).tolist())


def exact_depth(bagfile,source):
 with rosbag.Bag(str(bagfile)) as bag:
  for _,msg,_ in bag.read_messages(topics=['/supervisor/camera/depth/image_raw'],start_time=rospy.Time.from_sec(source/1e9-1),end_time=rospy.Time.from_sec(source/1e9+1)):
   if msg.header.stamp.to_nsec()!=source:continue
   if msg.encoding not in ('16UC1','mono16'):raise ValueError('unexpected depth encoding')
   dtype=np.dtype('>u2' if msg.is_bigendian else '<u2')
   depth=np.frombuffer(msg.data,dtype=dtype).reshape(msg.height,msg.step//2)[:,:msg.width].copy()
   return depth,dict(topic='/supervisor/camera/depth/image_raw',source_stamp_ns=source,encoding=msg.encoding,step=msg.step,raw_message_bytes_sha256=hashlib.sha256(msg.data).hexdigest())
 raise ValueError('no exact source depth; cannot substitute nearby frame')


def main():
 if not os.path.ismount('/media/zhuyupei/SanStand'):raise RuntimeError('external archive absent')
 if rs.__version__!='2.56.5':raise RuntimeError('wrong SDK version')
 OUT.mkdir(exist_ok=True)
 session=ARCHIVE/'bounded_joint23_counterrotation_20260928T125945Z_146409'
 gfile=session/'baseline_geometry.json';g=json.loads(gfile.read_text());p=g['quality']['camera_model']['color_intrinsics']
 T=np.array(g['T_base_tool'])@np.array(g['T_tool_camera']);depth,source=exact_depth(session/'evidence.bag',g['source_stamp_ns'])
 scale=g['camera_parameters']['depth_scale'];u,v=267,155
 motor=point_comparison((u,v),float(depth[v,u])*scale,p,T)
 patch=[]
 for yy in range(v-2,v+3):
  for xx in range(u-2,u+3):
   c=point_comparison((xx,yy),float(depth[yy,xx])*scale,p,T)
   if c['valid']:patch.append(c)
 motor.update(pixel_role='operator-provided motor ROI center; 5x5 is an image neighborhood, not an object mask',
  geometry_source=str(gfile),geometry_sha256=hashlib.sha256(gfile.read_bytes()).hexdigest(),depth_source=source,
  neighborhood_valid_count=len(patch),neighborhood_norm_mm_min_median_max=np.percentile([x['norm_mm'] for x in patch],[0,50,100]).tolist())
 nearby=[]
 prior=ARCHIVE/'grasp_diagnostic_20260928T084245Z'
 params=json.loads((prior/'initial_context.json').read_text())['camera'];intr=params['runtime_profile']['geometry_intrinsics']
 for stage in ('initial','pregrasp_settled','before_close'):
  f=prior/'support_visibility_replay'/(stage+'.npz');a=np.load(f);D=a['depth_raw'];tr=a['transform'];scale=float(a['depth_scale']);anchor=a['anchor_position'];cam=np.linalg.solve(tr,np.r_[anchor,1.])[:3]
  uv=np.array([intr['cx']-intr['fx']*cam[1]/cam[0],intr['cy']-intr['fy']*cam[2]/cam[0]])
  roi=np.rint(uv).astype(int);item=dict(stage=stage,source=str(f),source_sha256=hashlib.sha256(f.read_bytes()).hexdigest(),frozen_anchor_projection_uv=uv.tolist(),pixel_role='nearest pixel to the frozen anchor projection; not a new object identification',fixed_pixel_control=point_comparison((u,v),float(D[v,u])*scale,intr,tr))
  if 0<=roi[0]<D.shape[1] and 0<=roi[1]<D.shape[0]:item['anchor_pixel_comparison']=point_comparison(roi,float(D[roi[1],roi[0]])*scale,intr,tr)
  else:item['anchor_pixel_comparison']=dict(valid=False,reason='frozen target projects outside image')
  nearby.append(item)
 priorfile=ARCHIVE/'grasp_repair_resume_20260928T104059Z/support_ray_model_sensitivity.json'
 priorreport=json.loads(priorfile.read_text())
 result=dict(scope='diagnostic only; no model/config/ROS changes',sdk_version=rs.__version__,sdk_module=rs.__file__,intrinsics=p,
  difference_convention='SDK minus current tabletop pinhole, same raw depth and same pixel',
  transform_chain='P_optical=(x*z,y*z,z); P_camera_link=(z,-x*z,-y*z); P_base=FK(accepted)*X*P_camera_link. Delta_base=R_base_camera*Delta_camera; translation cancels and rigid transform preserves norm.',
  actual_motor_roi=motor,near_field_replays=nearby,
  prior_same_support_pixels_plane_audit=dict(source=str(priorfile),sha256=hashlib.sha256(priorfile.read_bytes()).hexdigest(),frames=priorreport['frames'],conclusion=priorreport['conclusion']),
  limitations=['A single point difference is not a fitted-plane or grasp-pose correction.','SDK rays do not remove extrinsic/FK/calibration errors.','Near-field frozen-anchor pixels are controls, not proof the same motor is still visible.'])
 (OUT/'report.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');(OUT/'analysis_source.py').write_bytes(Path(__file__).read_bytes())
 print(json.dumps(dict(motor=motor,nearby=nearby),ensure_ascii=False))

if __name__=='__main__':main()
