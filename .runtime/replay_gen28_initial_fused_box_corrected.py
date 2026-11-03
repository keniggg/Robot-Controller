"""Corrected bbox reconstruction; preserves earlier proxy evidence unchanged."""
import hashlib,json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import rosbag,rospy,yaml
from cv_bridge import CvBridge
from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample,fuse_stable_samples
from alicia_flexible_grasp.vision.object_geometry import estimate_object_geometry
from alicia_flexible_grasp.vision.regular_box_inference import qualify_regular_box_top
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
R=E/'grasp_records/20261015T075503.791411Z_24849e8304a44341adebca3867d29fdd'
O=E/'ros_log/full_latest_gen22_restart_20261015T003700Z/verification/gen27_attempt3_initial_inference_review'
assert Path('/media/zhuyupei/SanStand').is_mount()
assert json.loads((R/'manifest.json').read_text())['recorder_closed']
ids=[1792050906773036718,1792050906972096443,1792050907106353521]
f=np.load(O/('source_%d.npz'%ids[-1]));intr=f['intrinsics'];scale=float(f['depth_scale_m']);T=f['T_base_optical']
topics=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/perception/object_mask','/perception/object','/joint_states']
frames={ns:{} for ns in ids};joints=[]
with rosbag.Bag(str(R/'keyframes.bag')) as b:
    for topic,msg,rec in b.read_messages(topics=topics,start_time=rospy.Time(1792050906,300000000),end_time=rospy.Time(1792050907,900000000)):
        ns=msg.header.stamp.to_nsec()
        if topic=='/joint_states':joints.append((ns,msg))
        elif ns in frames:frames[ns][topic]=msg
bridge=CvBridge();identity=TargetTrackIdentity.from_stream(11,12);samples=[]
for ns in ids:
    row=frames[ns];obj=row['/perception/object'];assert obj.detected
    state=min(joints,key=lambda x:abs(x[0]-ns))[1];positions=dict(zip(state.name,state.position))
    samples.append(RgbdSample(bridge.imgmsg_to_cv2(row[topics[0]],'bgr8'),bridge.imgmsg_to_cv2(row[topics[1]],'passthrough'),
        bridge.imgmsg_to_cv2(row[topics[2]],'mono8'),(obj.bbox_x,obj.bbox_y,obj.bbox_width,obj.bbox_height),
        obj,ns/1e9,row[topics[1]].header.frame_id,np.array([positions['Joint%d'%i] for i in range(1,7)]),ns,12,identity))
cfgpath=Path('src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml')
cfg=yaml.safe_load(cfgpath.read_text())['grasp_6d']['remote']
sk=dict(require_mask=True,min_mask_iou=cfg['planning_mask_min_iou'],max_centroid_shift_px=cfg['planning_mask_max_centroid_shift_px'],
    max_joint_delta_rad=cfg['planning_max_joint_delta_rad'],erosion_px=cfg['mask_erosion_px'],depth_scale=scale,depth_min_m=.03,depth_max_m=2.,
    mad_scale=cfg['depth_mad_scale'],mad_absolute_floor_m=cfg['depth_mad_absolute_floor_m'],internal_hole_max_area_px=cfg['mask_internal_hole_max_area_px'])
s=fuse_stable_samples(samples,**sk);assert s.ok,(s.failure_code,s.failure_reason)
gk=dict(support_bbox_expand_ratio=cfg.get('support_bbox_expand_ratio',.30),support_distance_threshold_m=cfg.get('support_distance_threshold_m',cfg.get('target_cloud_support_plane_inlier_distance_m',.004)),
    voxel_size_m=cfg['target_cloud_voxel_size_m'],min_support_points=cfg.get('geometry_min_support_points',200),min_object_points=cfg.get('geometry_min_object_points',cfg.get('target_cloud_min_points',120)),
    min_size_m=cfg.get('geometry_min_size_m',.005),max_size_m=cfg.get('geometry_max_size_m',.600),max_height_m=cfg.get('geometry_max_height_m',.500),previous_axes_base=None,
    outlier_neighbors=cfg['target_cloud_outlier_neighbors'],outlier_std_ratio=cfg['target_cloud_outlier_std_ratio'],support_minimum_span_m=max(.0434,.05+2*.0286)+2*.003)
camera=SimpleNamespace(fx=intr[0],fy=intr[1],cx=intr[2],cy=intr[3],width=s.depth_raw.shape[1],height=s.depth_raw.shape[0],depth_scale=scale)
g=estimate_object_geometry(s.depth_raw,s.target_depth_raw,s.object_mask,s.bbox,camera,scale,T,s.source_mode,**gk)
assert g.ok,(g.failure_code,g.failure_reason)
result=qualify_regular_box_top(s.depth_raw,s.object_mask,intr,scale,T,g,identity,s.stamp_ns,color_bgr=s.color_bgr)
raw_result=qualify_regular_box_top(f['depth_raw'],f['object_mask'],intr,scale,T,g,identity,s.stamp_ns,color_bgr=f['color'])
O=O.parent/'gen28_initial_corrected_bbox_replay'
assert not O.exists();O.mkdir()
gp=dict(obb_center_base_m=g.center_base.tolist(),R_base_obb=g.axes_base.tolist(),obb_size_xyz_m=g.size_xyz_m.tolist(),support_normal_base=g.support_normal_base.tolist(),support_offset_m=g.support_offset_m,
    support_inlier_ratio=g.support_inlier_ratio,object_points_base_m=g.object_points_base.tolist(),source_mode=g.source_mode)
npz=O/'initial_three_frame_reconstruction.npz';assert not npz.exists()
np.savez_compressed(str(npz),color=s.color_bgr,depth_raw=s.depth_raw,target_depth_raw=s.target_depth_raw,object_mask=s.object_mask,T_base_optical=T,intrinsics=intr,depth_scale_m=np.array(scale),source_stamp_ns=np.array(ids[-1]))
p=O/'initial_three_frame_geometry.json';assert not p.exists();p.write_text(json.dumps(gp,indent=2)+'\n')
report=dict(classification='offline_reconstruction_not_original_retained_qualification_audit',source_stamp_ns=ids[-1],sample_stamp_ns=ids,
    limitations=['Production YAML plus source defaults; full original effective geometry parameter capture unavailable.',
      'Original previous_axes_base and exact buffer admission/joint pairing not retained; using nearest sealed joint stamp and no previous axes.',
      'Three sources are all detected target messages from selection acquisition through trigger; original sample membership not explicitly retained in trigger audit.'],
    snapshot_arguments=sk,geometry_arguments=gk,config_sha256=hashlib.sha256(cfgpath.read_bytes()).hexdigest(),npz=str(npz),geometry=str(p),
    center_base=g.center_base.tolist(),size_xyz_m=g.size_xyz_m.tolist(),support_normal_base=g.support_normal_base.tolist(),support_offset_m=g.support_offset_m,
    qualification=dict(ok=result.ok,code=result.code,audit=dict(result.audit)),raw_full_qualification_with_corrected_fused_geometry=dict(ok=raw_result.ok,code=raw_result.code,audit=dict(raw_result.audit),model=raw_result.model.to_evidence_dict() if raw_result.ok else None),bbox_convention='x_y_width_height',prior_proxy_correction='Old replay used x_y_xmax_ymax incorrectly for production bbox API; original old artifacts preserved unchanged. Fusion inputs remain identical but support estimation context corrected.',no_ros_calls=True,no_hardware_access=True,no_temperature_reads=True)
if result.ok:report['model']=result.model.to_evidence_dict()
p=O/'initial_three_frame_replay.json';assert not p.exists();p.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),size_xyz_m=report['size_xyz_m'],qualification=report['qualification'],raw_full_qualification=report['raw_full_qualification_with_corrected_fused_geometry']),ensure_ascii=False))
