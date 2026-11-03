"""Rebuild one recorded observation source; replay the original solver offline."""
from pathlib import Path
from types import SimpleNamespace as NS
import hashlib, itertools, json, os, time
W=Path(__file__).resolve().parents[1]
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
R=E/'ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1/actual_gen63_baseline_validation_20261023T041149Z/high_accuracy_grasp_r1/arrival_margin_repair_r1'
OUT=R/'supplemental_view_r3/collision_diagnosis'
RECORD=E/'grasp_records/20261023T080014.824313Z_3e10cdc5160c4c0f879b83412aa4f27c'
os.environ['OFFLINE_BOUNDARY_LOG']=str(OUT/'view_search_boundary.jsonl')
os.environ['ROS_LOG_DIR']=str(OUT/'offline_ros')
exec((R/'guard.py').read_text())
import numpy as np
import rospy, rosbag, tf2_py
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.robot.observation_view_limits import effective_observation_view_limits, read_observation_view_parameters
from alicia_grasp_modes.silhouette_observation_view_source import measured_view_source
from alicia_grasp_modes.observation_arrival_projection import capture_arrival_projection
import alicia_grasp_modes.reachable_observation_views as solver

def sha(raw):return hashlib.sha256(raw).hexdigest()
def plain(x):
    if isinstance(x,np.ndarray):return x.tolist()
    if isinstance(x,np.generic):return x.item()
    if isinstance(x,dict):return {k:plain(v) for k,v in x.items()}
    if isinstance(x,(tuple,list)):return [plain(v) for v in x]
    return x
audit_path=RECORD/'keyframes_audits/f48f61f3157566b86c3d302f9534fbd4f3d1e8cd22ab730795c49b3204da2aa0.json'
assert sha(audit_path.read_bytes())==audit_path.stem
d=json.loads(audit_path.read_text());view=d['selected']['observation_view'];search=view['proposal_search']
binding=view['proposal']['binding'];ns=binding['snapshot_stamp_ns'];stamp=rospy.Time(ns//10**9,ns%10**9)
saved=json.loads((OUT/'camera_and_model.json').read_text())['values'];camera=saved['/camera']
params=json.loads((OUT/'readonly_parameters.json').read_text())['values'];rcfg=params['/grasp_6d']['remote']
static=[];dynamic=[];messages={};statuses=[]
topics=['/tf','/tf_static','/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/perception/object_mask','/perception/unknown/detector_status']
with rosbag.Bag(str(RECORD/'keyframes.bag')) as b:
    for topic,m,t in b.read_messages(topics=topics):
        if topic=='/tf':dynamic.extend(m.transforms)
        elif topic=='/tf_static':static.extend(m.transforms)
        elif topic.endswith('detector_status'):
            v=json.loads(m.data)
            if v.get('source_stamp_ns')==ns and v.get('state')=='ready':statuses.append(v)
        elif m.header.stamp.to_nsec()==ns:messages[topic]=m
buf=tf2_py.BufferCore(rospy.Duration(120))
for t in static:buf.set_transform_static(t,'sealed_static')
for t in sorted(dynamic,key=lambda t:t.header.stamp.to_nsec()):buf.set_transform(t,'sealed_dynamic')
def transform(parent,child):
    m=buf.lookup_transform_core(parent,child,stamp).transform
    a=quaternion_matrix([getattr(m.rotation,k) for k in 'xyzw']);a[:3,3]=[getattr(m.translation,k) for k in 'xyz'];return a
mount=transform('tool0','camera_link');T=transform('base_link','camera_link')
link_opt=np.eye(4);link_opt[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
bridge=CvBridge();rgb=bridge.imgmsg_to_cv2(messages['/supervisor/camera/color/image_raw'],'bgr8')
depth=bridge.imgmsg_to_cv2(messages['/supervisor/camera/depth/image_raw'],'passthrough')
mask=bridge.imgmsg_to_cv2(messages['/perception/object_mask'],'passthrough')
identity=TargetTrackIdentity(binding['target_epoch'],binding['target_track_id'])
sample=NS(stamp_ns=ns,target_identity=identity,target_epoch=identity.epoch,frame_id='camera_link',
    object_msg=NS(header=NS(stamp=stamp),detected=True),color_bgr=rgb,depth_raw=depth,object_mask=mask)
snapshot=NS(stamp_ns=ns,stamp_sec=stamp.to_sec(),sample_stamp_ns=tuple(d['pipeline_funnel']['snapshot_evidence']['source_stamp_ns']),
    target_identity=identity,target_epoch=identity.epoch,frame_id='camera_link',color_bgr=rgb)
ticket=NS(payload=(snapshot,),snapshot_stamp_sec=stamp.to_sec(),request_id=binding['request_id'],
    generation=binding['generation'],target_epoch=identity.epoch)
g=d['replay_geometry'];geometry=NS(ok=True,center_base=np.array(g['obb_center_base_m']),axes_base=np.array(g['R_base_obb']),
    size_xyz_m=np.array(g['obb_size_xyz_m']),support_normal_base=np.array(g['support_normal_base']),support_offset_m=g['support_offset_m'])
prepared=NS(snapshot=snapshot,ticket=ticket,geometry=geometry,pose_estimator=NS(_T_base_optical=T@link_opt))
source=measured_view_source(sample,prepared,NS(**{k:camera[k] for k in ['width','height','fx','fy','cx','cy','depth_scale']}),mount)
requested=read_observation_view_parameters(lambda name,default:rcfg.get(name.rsplit('/',1)[-1],default))
source['view_limits']=dict(requested_visibility_limits_snapshot=requested,effective_visibility_limits=effective_observation_view_limits(requested))
assert statuses,'recorded exact-source ready status required'
arrival=capture_arrival_projection(statuses[-1],d['grasp_mode_selection'],ns,camera)
expected=view['arrival_projection_endpoint']['source']
assert arrival['audit']==expected,(arrival['audit'],expected)
source['arrival_projection']=arrival
source['geometry']={k:g[k] for k in ['R_base_obb','obb_size_xyz_m','support_normal_base','support_offset_m']}
source['geometry']['obb_center_base_m']=g['obb_center_base_m']
fk=SerialUrdfFk(saved['/robot_description'],['Joint%d'%i for i in range(1,7)])
assert fk.model_sha256==search['model_sha256']
np.testing.assert_allclose(fk(view['proposal']['q']),view['proposal']['T_base_tool0'],atol=1e-12,rtol=0)
gg=rcfg['gripper_geometry'];gripper=GripperGeometry(gg['max_inner_gap_m'],gg['width_safety_margin_per_side_m'],
    np.array(gg['finger_box_xyz_m']),np.array(gg['palm_box_xyz_m']),gg.get('support_clearance_m',.003))
args=dict(source=source,current_joint_positions=search['current_joint_positions'],fk=fk,gripper=gripper,
    opening_width_m=.05,gaps=d['observe_return']['measured_missing_sides'],depth_uncertainty_m=search['endpoint_acceptance_uncertainty_m'],
    side_depth_uncertainty_m=search['optical_side_uncertainty_m'],joint_error_bounds_rad=search['joint_error_bounds_rad'],
    max_seconds=search['max_seconds'],max_directions=search['selected_direction_count'],proposal_image_reserve_px=search['proposal_image_reserve_px'])
candidates,report=solver.generate_reachable_observation_views(**args)
out=dict(scope='original exact-source observation search, no motion or new camera capture',
    audit_sha256=audit_path.stem,source_audit=source['audit'],original_search=search,report=report,candidates=candidates,
    original_support_point_count=view['check_scheduling_metrics']['measured_support_schedule']['measured_source_points'],
    reproduced_support_point_count=len(source['points_base']),arrival_audit_exact=True,
    execution_authorized=False)
(OUT/'view_search_replay.json').write_text(json.dumps(plain(out),ensure_ascii=False,indent=2)+'\n')
np.savez_compressed(OUT/'view_search_source.npz',points_base=source['points_base'],corners_base=source['corners_base'],
    tool_from_camera=mount,prism_base=arrival['prism_base'],rgb_valid=arrival['rgb_valid'],invalid_integral=arrival['invalid_integral'])
small={k:v for k,v in source.items() if k not in ('points_base','corners_base','tool_from_camera','arrival_projection')}
small['arrival_projection_audit']=arrival['audit'];small['arguments']={k:v for k,v in args.items() if k not in ('source','fk','gripper')}
small['robot_description_sha256']=fk.model_sha256
(OUT/'view_search_source.json').write_text(json.dumps(plain(small),ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(candidates=len(candidates),support_points=len(source['points_base']),elapsed=report['elapsed_sec'],status=report['status'],
    directions=[{k:r[k] for k in ('gap_index','exposure_target_noise_multiplier','accepted','reason') if k in r} for r in report['directions']])),flush=True)
