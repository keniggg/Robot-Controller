from pathlib import Path
import json,hashlib,itertools
import numpy as np
import rosbag,rospy,tf2_ros
from tf.transformations import quaternion_matrix
from alicia_grasp_modes.observation_arrival_projection import capture_arrival_projection
ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller');NEW=ROOT/'ros_log/full_latest_restart_20261010T002347Z';OUT=NEW/'gen20_analysis/arrival_projection';REC=ROOT/'grasp_records/20261010T084609.133186Z_f832fa3e55eb45a59d41fecf2246af73'
audit=json.loads((REC/'keyframes_audits/f3c7dd20ffb7481a957173d2429e59d47a38bd0b42d0327b320fbdf2de92afed.json').read_text());diag=json.loads((OUT/'planned_actual_projection.json').read_text());s=audit['selected'];ns=audit['snapshot_stamp_ns'];old=s['observation_source_audit'];buf=tf2_ros.Buffer(cache_time=rospy.Duration(300),debug=False);arrays={}
with rosbag.Bag(str(REC/'keyframes.bag')) as bag:
 for topic,m,t in bag.read_messages(topics=['/tf','/tf_static','/supervisor/camera/depth/image_raw','/perception/object_mask']):
  if topic in ('/tf','/tf_static'):
   for x in m.transforms:(buf.set_transform_static if topic=='/tf_static' else buf.set_transform)(x,'sealed')
  elif m.header.stamp.to_nsec()==ns:
   dtype=np.uint16 if m.encoding=='16UC1' else np.uint8
   key='depth' if dtype==np.uint16 else 'mask';arrays[key]=np.frombuffer(m.data,dtype).reshape(m.height,m.step//np.dtype(dtype).itemsize)[:,:m.width].copy()
assert set(arrays)=={'mask','depth'},arrays.keys()
sha=lambda a:hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()
assert sha(arrays['depth'])==old['raw_depth_sha256'];assert sha(arrays['mask'])==old['raw_mask_sha256']
def matrix(parent,child):
 tr=buf.lookup_transform(parent,child,rospy.Time(ns//10**9,ns%10**9)).transform;T=quaternion_matrix([tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w]);T[:3,3]=[tr.translation.x,tr.translation.y,tr.translation.z];return T
cam=matrix('base_link','camera_link');mount=matrix('tool0','camera_link');c=diag['camera'];g=audit['replay_geometry'];normal=np.asarray(g['support_normal_base']);offset=g['support_offset_m'];center=np.asarray(g['obb_center_base_m']);R=np.asarray(g['R_base_obb']);size=np.asarray(g['obb_size_xyz_m']);yy,xx=np.nonzero((arrays['depth']>0)&(arrays['mask']==0));z=arrays['depth'][yy,xx].astype(float)*c['depth_scale'];points=np.column_stack((z,-(xx-c['cx'])*z/c['fx'],-(yy-c['cy'])*z/c['fy']))@cam[:3,:3].T+cam[:3,3];points=points[abs(points@normal+offset)<=.002];corners=center+(np.asarray(list(itertools.product((-.5,.5),repeat=3)))*size)@R.T
assert len(points)==old['support_point_count'],(len(points),old['support_point_count'])
# Floating multiplication grouping changes only the last bit relative to optical conversion.
source=dict(binding=s['observation_source_binding'],camera=c,tool_from_camera=mount.tolist(),view_limits=old['view_limits'],geometry=dict(support_normal_base=normal.tolist(),support_offset_m=offset,R_base_obb=R.tolist(),obb_size_xyz_m=size.tolist()),center_base=center.tolist())
search=s['observation_view']['proposal_search'];out=dict(source=source,current_joint_positions=search['current_joint_positions'],joint_error_bounds_rad=search['joint_error_bounds_rad'],gaps=audit['observe_return']['measured_missing_sides'],depth_uncertainty_m=search['endpoint_acceptance_uncertainty_m'],side_depth_uncertainty_m=search['optical_side_uncertainty_m'],source_status=diag['source_status'],selection=dict(mode='unknown',strategy='observe_return',generation=20,stamp_ns=1791621970669396162),model_sha256=search['model_sha256'],recorded_selected_q=s['observation_view']['proposal']['q'],provenance=dict(raw_depth_sha256=sha(arrays['depth']),raw_mask_sha256=sha(arrays['mask']),support_points_count=len(points),support_points_sha256=sha(points),original_support_points_sha256=old['support_points_sha256'],limitations=['base-camera multiplication grouping can differ in floating final bits from recorded optical-base path; pixel membership count is identical']))
np.savez_compressed(str(OUT/'view_source.npz'),points_base=points,corners_base=corners)
(OUT/'view_source.json').write_text(json.dumps(out,indent=2)+'\n');print('extracted',len(points),out['provenance'])
