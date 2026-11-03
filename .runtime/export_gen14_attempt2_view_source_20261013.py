#!/usr/bin/env python3
"""Offline measured attempt 2 source/proposal diagnostics; no ROS master or motion."""
from pathlib import Path
import hashlib, json, itertools, math
import numpy as np
import rosbag, rospy, tf2_py
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix
from alicia_grasp_modes.observation_arrival_projection import capture_arrival_projection
from alicia_grasp_modes.observe_return_visual import missing_side_view_gain
from alicia_grasp_modes.observation_policy import observation_distance_preferences
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_grasp_modes.reachable_observation_views import generate_reachable_observation_views

N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
R=N.parents[1]/'grasp_records/20261013T025604.344486Z_af3ecb9373264cc9b19a98143f7b5df5'
F=N/'verification/gen14_arrival_independent'
OUT=F/'view_source'
A=R/'keyframes_audits/0e4b37a0810b68c784653ec71741d0a15c70bee954ea112c830e5a3c325e09cc.json'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def h(a):return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()
def plain(x):
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 if isinstance(x,dict):return {k:plain(v)for k,v in x.items()}
 if isinstance(x,(tuple,list)):return [plain(v)for v in x]
 return x
def matrix(msg):
 t,q=msg.transform.translation,msg.transform.rotation
 T=quaternion_matrix([q.x,q.y,q.z,q.w]);T[:3,3]=[t.x,t.y,t.z];return T
def angle(a,b):return float(np.degrees(np.arccos(np.clip(np.dot(a,b)/np.linalg.norm(a)/np.linalg.norm(b),-1,1))))
def main():
 assert Path('/media/zhuyupei/SanStand').is_mount() and not OUT.exists()
 audit=json.loads(A.read_text());assert sha(A)==A.stem
 selected=audit['selected'];ca=selected['observation_source_audit'];ns=ca['source_stamp_ns'];view=selected['observation_view'];search=view['proposal_search']
 camera=json.loads((F/'fixture.json').read_text())['camera'];bridge=CvBridge();buffer=tf2_py.BufferCore(rospy.Duration(600));raw={};status=None
 with rosbag.Bag(str(R/'keyframes.bag'))as bag:
  for topic,msg,t in bag.read_messages(topics=['/tf','/tf_static','/perception/object_mask','/perception/unknown/detector_status']):
   if topic=='/tf_static':
    for tr in msg.transforms:buffer.set_transform_static(tr,'sealed')
   elif topic=='/tf':
    for tr in msg.transforms:buffer.set_transform(tr,'sealed')
   elif topic.endswith('object_mask') and msg.header.stamp.to_nsec()==ns:raw['mask']=bridge.imgmsg_to_cv2(msg,'passthrough').copy()
   elif topic.endswith('detector_status'):
    d=json.loads(msg.data)
    if d.get('source_stamp_ns')==ns and d.get('state')=='ready':status=d
 with rosbag.Bag(str(N/'attempt_2_full_frames/full_frames.bag'))as bag:
  for topic,msg,t in bag.read_messages(topics=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw'],start_time=rospy.Time(ns//10**9,0),end_time=rospy.Time(ns//10**9+1,0)):
   if msg.header.stamp.to_nsec()==ns:raw['rgb'if'/color/'in topic else'depth']=bridge.imgmsg_to_cv2(msg,'bgr8'if'/color/'in topic else'passthrough').copy()
 assert set(raw)=={'mask','rgb','depth'} and status
 stamp=rospy.Time(ns//10**9,ns%10**9);mount=matrix(buffer.lookup_transform_core('tool0','camera_link',stamp));basecam=matrix(buffer.lookup_transform_core('base_link','camera_link',stamp));c2o=np.eye(4);c2o[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]];optical=basecam@c2o
 g=audit['replay_geometry'];normal=np.array(g['support_normal_base']);offset=g['support_offset_m'];center=np.array(g['obb_center_base_m']);axes=np.array(g['R_base_obb']);size=np.array(g['obb_size_xyz_m']);k=ca['camera_parameters'];depth=raw['depth'];mask=raw['mask']
 yy,xx=np.nonzero((depth>0)&(mask==0));z=depth[yy,xx].astype(float)*k['depth_scale'];points=np.c_[(xx-k['cx'])*z/k['fx'],(yy-k['cy'])*z/k['fy'],z]@optical[:3,:3].T+optical[:3,3];points=points[np.abs(points@normal+offset)<=.002]
 corners=center+(np.array(list(itertools.product((-.5,.5),repeat=3)))*size)@axes.T
 hashes={key:dict(expected=ca[key],actual=h(v),match=ca[key]==h(v))for key,v in [('raw_depth_sha256',depth),('raw_mask_sha256',mask),('tool_camera_sha256',mount),('base_optical_sha256',optical),('support_points_sha256',points),('conservative_target_corners_sha256',corners)]}
 assert all(hashes[key]['match']for key in ['raw_depth_sha256','raw_mask_sha256','tool_camera_sha256','conservative_target_corners_sha256']),hashes
 assert len(points)==ca['support_point_count']
 arrival=capture_arrival_projection(status,audit['grasp_mode_selection'],ns,camera);assert arrival['audit']==ca['arrival_projection']
 base_tool=basecam@np.linalg.inv(mount)
 source=dict(binding=selected['observation_source_binding'],audit=ca,points_base=points,corners_base=corners,center_base=center,base_from_tool=base_tool,tool_from_camera=mount,camera=k,geometry=dict(support_normal_base=normal,support_offset_m=offset,R_base_obb=axes,obb_size_xyz_m=size),view_limits=ca['view_limits'],arrival_projection=arrival)
 ctx=N/'readonly_full_frame_probe_002/context.json';description=json.loads(ctx.read_text())['parameters']['/robot_description'];fk=SerialUrdfFk(description,['Joint%d'%i for i in range(1,7)]);assert fk.model_sha256==search['model_sha256']
 gripper=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)
 args=dict(current_joint_positions=search['current_joint_positions'],opening_width_m=.0498,gaps=audit['observe_return']['measured_missing_sides'],depth_uncertainty_m=.003,side_depth_uncertainty_m=.003,joint_error_bounds_rad=search['joint_error_bounds_rad'],max_seconds=2.,max_directions=2,proposal_image_reserve_px=4.)
 OUT.mkdir();(OUT/'model.urdf').write_text(description)
 arrays={key:source[key]for key in ['points_base','corners_base','center_base','base_from_tool','tool_from_camera']};arrays.update(prism_base=arrival['prism_base'],rgb_valid=arrival['rgb_valid'],invalid_integral=arrival['invalid_integral'],base_from_camera=basecam,raw_depth=depth,raw_mask=mask,rgb=raw['rgb']);np.savez_compressed(OUT/'source.npz',**arrays)
 metadata=dict(source={k:v for k,v in source.items()if k not in arrays and k!='arrival_projection'},arrival_audit=arrival['audit'],arguments=args,gripper=dict(max_inner_gap_m=.05,jaw_clearance_each_side_m=.002,finger_size_xyz_m=[.0434,.0286,.06],palm_size_xyz_m=[.1175,.155,.0774],support_clearance_m=.003),source_hashes=hashes,source_npz_sha256=sha(OUT/'source.npz'),audit_path=str(A),audit_sha256=sha(A),model_path=str(OUT/'model.urdf'),model_sha256=fk.model_sha256,recorded_source_ns=ns,context_path=str(ctx),context_sha256=sha(ctx),script_sha256=sha(__file__),scope='Original sealed raw pixels/mask/gaps/source geometry and exact-time TF reconstruction for offline proposal checks; no historical or live execution authority',limitation='Reconstruction floating-point transform/support hashes are explicitly compared; mismatches are not claimed bit-identical. Original source gates, actual image arrival, full trajectory/collision/following checks remain independent.')
 (OUT/'source.json').write_text(json.dumps(plain(metadata),indent=2)+'\n')
 rows,report=generate_reachable_observation_views(source=source,fk=fk,gripper=gripper,**args)
 initial_origin=basecam[:3,3];initial_ray=initial_origin-center;initial_axis=basecam[:3,0]
 def orientation(T):
  ray=T[:3,3]-center;return dict(optical_incidence_deg=angle(T[:3,0],-normal),optical_turn_deg=angle(T[:3,0],initial_axis),target_ray_change_deg=angle(ray,initial_ray),camera_translation_m=float(np.linalg.norm(T[:3,3]-initial_origin)),target_distance_m=float(np.linalg.norm(ray)))
 variants=[]
 for row in rows:variants.append(dict(row,diagnostic_orientation=orientation(np.array(row['T_base_camera_link']))))
 distances=observation_distance_preferences(corners,center,basecam[:3,:3],initial_origin,[k[x]for x in('fx','fy','cx','cy')],k['width'],k['height'],**ca['view_limits']['effective_visibility_limits'])
 controls=[]
 for distance in distances:
  for mode in ['original_orientation_centered_lattice','original_target_ray_radial']:
   T=basecam.copy();T[:3,3]=center-(initial_axis if mode.endswith('lattice')else-initial_ray/np.linalg.norm(initial_ray))*distance
   controls.append(dict(mode=mode,distance_m=distance,gain=missing_side_view_gain(args['gaps'],T[:3,3],.003),diagnostic_orientation=orientation(T),pose=T))
 result=dict(source_hashes=hashes,original_fk_candidates=variants,original_fk_audit=report,controls=controls,recorded_selected_orientation=orientation(np.array(view['proposal']['T_base_camera_link'])),recorded_selected_view=view,recorded_search_summary={k:selected['observation_orientation_search'].get(k)for k in ['configured_variant_count','attempted_variant_count','reachable_variant_count','search_complete','check_order','selected_preference_index']},source_json_sha256=sha(OUT/'source.json'),motion_commands_sent=False,no_ros_master=True,complete_path_proof=False)
 (OUT/'original_proposals.json').write_text(json.dumps(plain(result),indent=2)+'\n');print(json.dumps(dict(output=str(OUT),source_hashes=hashes,candidates=len(rows),proposal_summary=[v['diagnostic_orientation']for v in variants],control_gains=[dict(mode=v['mode'],distance=v['distance_m'],ok=v['gain']['ok'])for v in controls])))
if __name__=='__main__':main()
