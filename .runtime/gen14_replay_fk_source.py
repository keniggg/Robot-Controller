"""Reconstruct GEN14 exact recorded source and replay auxiliary FK; offline only."""
from pathlib import Path
import hashlib, json, itertools, time
import numpy as np
import rosbag, rospy, tf
from tf.transformations import quaternion_matrix
from alicia_grasp_modes.observation_arrival_projection import capture_arrival_projection
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
import alicia_grasp_modes.reachable_observation_views as module

EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW=EXT/'ros_log/full_latest_no_skin_20261011T000625Z'
REC=EXT/'grasp_records/20261011T070957.732968Z_f55879ef6b7140c3a5b3ebe7ec00e0d7'
AUD=REC/'keyframes_audits/2b08923465a42090a121778fb20d63c8537eb596b365f00d0ca77abd001e3719.json'
OUT=NEW/'gen14_analysis/fk_replay_inputs'
def sha(a):return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()
def matrix(t,q):
 q=np.asarray(q,dtype=float);q=q/np.linalg.norm(q)
 v=quaternion_matrix(q);v[:3,3]=t;return v
def main():
 assert Path('/media/zhuyupei/SanStand').is_mount()
 a=json.loads(AUD.read_text()); d=a['observation_failure_diagnostics']; ca=d['source']['captured_audit']; ns=a['snapshot_stamp_ns']
 assert hashlib.sha256(AUD.read_bytes()).hexdigest()==AUD.stem
 transformer=tf.Transformer(True,rospy.Duration(200))
 images={};meta=None;status=None;static=[]
 topics=['/tf','/tf_static','/grasp_record/camera_runtime_metadata','/perception/unknown/detector_status','/supervisor/camera/depth/image_raw','/perception/object_mask']
 with rosbag.Bag(str(REC/'keyframes.bag')) as bag:
  for topic,msg,received in bag.read_messages(topics=topics):
   if topic=='/tf':
    for tr in msg.transforms:transformer.setTransform(tr,'recorded')
   elif topic=='/tf_static':static.extend(msg.transforms)
   elif topic=='/grasp_record/camera_runtime_metadata':meta=json.loads(msg.data)['parameters']
   elif topic=='/perception/unknown/detector_status':
    s=json.loads(msg.data)
    if s.get('source_stamp_ns')==ns and s.get('state')=='ready':status=s
   elif msg.header.stamp.to_nsec()==ns:
    dtype=np.uint16 if msg.encoding=='16UC1' else np.uint8
    raw=np.frombuffer(msg.data,dtype=dtype).reshape(msg.height,msg.step//np.dtype(dtype).itemsize)
    images['depth' if 'depth' in topic else 'mask']=raw[:,:msg.width].copy()
 stamp=rospy.Time(ns//10**9,ns%10**9)
 for tr in static:
  tr.header.stamp=stamp;transformer.setTransform(tr,'recorded-static')
 mount=matrix(*transformer.lookupTransform('tool0','camera_link',stamp))
 bt,bq=transformer.lookupTransform('base_link','camera_link',stamp)
 basecam=quaternion_matrix([float(v) for v in bq]);basecam[:3,3]=bt
 c2o=np.eye(4);c2o[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
 optical=basecam@c2o
 raw,mask=images['depth'],images['mask'];k=ca['camera_parameters'];g=a['replay_geometry']
 normal=np.array(g['support_normal_base']);offset=g['support_offset_m'];center=np.array(g['obb_center_base_m']);axes=np.array(g['R_base_obb']);size=np.array(g['obb_size_xyz_m'])
 ys,xs=np.nonzero((raw>0)&(mask==0));z=raw[ys,xs].astype(float)*k['depth_scale']
 points=np.column_stack(((xs-k['cx'])*z/k['fx'],(ys-k['cy'])*z/k['fy'],z))@optical[:3,:3].T+optical[:3,3]
 points=points[np.abs(points@normal+offset)<=.002]
 corners=center+(np.asarray(list(itertools.product((-.5,.5),repeat=3)))*size)@axes.T
 hashes={key:dict(expected=ca[key],actual=sha(value),match=ca[key]==sha(value)) for key,value in [('raw_depth_sha256',raw),('raw_mask_sha256',mask),('tool_camera_sha256',mount),('base_optical_sha256',optical),('support_points_sha256',points),('conservative_target_corners_sha256',corners)]}
 print('SOURCE_HASHES',json.dumps(hashes),flush=True)
 assert all(v['match'] for v in hashes.values()),hashes
 assert status is not None and meta is not None
 arrival=capture_arrival_projection(status,a['grasp_mode_selection'],ns,meta)
 assert arrival['audit']==ca['arrival_projection'],(arrival['audit'],ca['arrival_projection'])
 source=dict(binding=d['source']['captured_binding'],audit=ca,points_base=points,corners_base=corners,center_base=center,tool_from_camera=mount,camera=k,geometry=g,view_limits=ca['view_limits'],arrival_projection=arrival)
 probe=json.loads((NEW/'gen14_analysis/req16_read_only_ik_probe_inputs.json').read_text());fk=SerialUrdfFk(Path(probe['model_path']).read_text(),['Joint%d'%i for i in range(1,7)])
 assert fk.model_sha256==probe['model_sha256']
 args=dict(source=source,current_joint_positions=d['joint_space_fk_search']['audit']['current_joint_positions'],fk=fk,gripper=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003),opening_width_m=.0498,gaps=a['observe_return']['measured_missing_sides'],depth_uncertainty_m=.003,side_depth_uncertainty_m=.003,joint_error_bounds_rad=d['joint_space_fk_search']['audit']['joint_error_bounds_rad'])
 OUT.mkdir(exist_ok=True)
 np.savez_compressed(str(OUT/'source.npz'),points_base=points,corners_base=corners,center_base=center,tool_from_camera=mount,prism_base=arrival['prism_base'],rgb_valid=arrival['rgb_valid'],invalid_integral=arrival['invalid_integral'])
 metaout=dict(source_audit=str(AUD),source_audit_sha256=AUD.stem,source_hashes=hashes,source={k:v for k,v in source.items() if k not in ('points_base','corners_base','center_base','tool_from_camera','arrival_projection')},arrival_audit=arrival['audit'],current_joint_positions=args['current_joint_positions'],gaps=args['gaps'],joint_error_bounds_rad=args['joint_error_bounds_rad'],model_path=probe['model_path'],model_sha256=fk.model_sha256,opening_width_m=.0498,physical_actions=False)
 (OUT/'source.json').write_text(json.dumps(metaout,indent=2)+'\n')
 rows,report=module.generate_reachable_observation_views(**args)
 (OUT/'original_fk_replay.json').write_text(json.dumps(dict(candidates=rows,audit=report),indent=2)+'\n')
 print('REPLAY',json.dumps(report),flush=True)
if __name__=='__main__':main()
