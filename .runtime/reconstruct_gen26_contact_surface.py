"""Original seven SurfaceViews from exported sealed RGB-D. No ROS IO or motion."""
import dataclasses,hashlib,json,time
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import yaml
from tf.transformations import quaternion_matrix
from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample,fuse_stable_samples
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.vision.object_geometry import estimate_object_geometry
from alicia_flexible_grasp.vision.fused_scene_geometry import enclose_registered_surface
from alicia_flexible_grasp.vision import multiview_surface as mv

BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z/gen26_contact_analysis')
INPUT=BASE/'source_inputs/manifest.json';M=json.loads(INPUT.read_text());R=Path(M['record'])
OUT=BASE/'fused_reconstruction_final';OUT.mkdir(exist_ok=True)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def convert(v):
    if isinstance(v,np.ndarray):return v.tolist()
    if isinstance(v,np.generic):return v.item()
    if dataclasses.is_dataclass(v):return {f.name:convert(getattr(v,f.name)) for f in dataclasses.fields(v)}
    if isinstance(v,dict):return {str(k):convert(x) for k,x in v.items()}
    if isinstance(v,(list,tuple)):return [convert(x) for x in v]
    return v
def namespace(x):return NS(**{k:namespace(v) for k,v in x.items()}) if isinstance(x,dict) else x
def msgstamp(m):return m['header']['stamp']['secs']*10**9+m['header']['stamp']['nsecs']
frames={x['source_stamp_ns']:x for x in M['frames']};arrays={ns:np.load(x['npz'],allow_pickle=False) for ns,x in frames.items()}
audits={}
for p in (R/'keyframes_audits').glob('*.json'):
    d=json.loads(p.read_text())
    if d['request_id'] in (9,19):audits[d['snapshot_stamp_ns']]=(p,d)
cfg_path=Path('src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml')
cfg=yaml.safe_load(cfg_path.read_text())['grasp_6d']['remote']
g=cfg.get('gripper_geometry',{});fingers=g.get('finger_box_xyz_m',[.0434,.0286,.0600])
geom=dict(support_bbox_expand_ratio=cfg.get('support_bbox_expand_ratio',.30),support_minimum_span_m=max(fingers[0],g.get('max_inner_gap_m',.05)+2*fingers[1])+2*g.get('support_clearance_m',.003),support_distance_threshold_m=cfg.get('support_distance_threshold_m',cfg.get('target_cloud_support_plane_inlier_distance_m',.004)),voxel_size_m=cfg.get('target_cloud_voxel_size_m',.0025),min_support_points=cfg.get('geometry_min_support_points',200),min_object_points=cfg.get('geometry_min_object_points',cfg.get('target_cloud_min_points',120)),min_size_m=cfg.get('geometry_min_size_m',.005),max_size_m=cfg.get('geometry_max_size_m',.6),max_height_m=cfg.get('geometry_max_height_m',.5),outlier_neighbors=cfg.get('target_cloud_outlier_neighbors',16),outlier_std_ratio=cfg.get('target_cloud_outlier_std_ratio',2.),previous_axes_base=None)
identity=TargetTrackIdentity(M['target_epoch'],M['target_track_id'])
opt=np.eye(4);opt[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
views=[];checks=[];estimates={};snapshots={};enclosures=[];start=time.monotonic()
def view(ns):
    stamp_list=M['direct_snapshot_sources'].get(str(ns),[ns]); samples=[]
    for sn in stamp_list:
        f=frames[sn];a=arrays[sn];obj=namespace(f['object']);box=tuple(f['object'][k] for k in ('bbox_x','bbox_y','bbox_width','bbox_height'))
        samples.append(RgbdSample(a['rgb'],a['depth_raw'],a['raw_mask'],box,obj,sn/1e9,'camera_link',a['q'][:6],sn,M['target_epoch'],identity))
    camera=frames[ns]['camera'];scale=camera['depth_scale'];K=NS(**{k:camera[k] for k in ('fx','fy','cx','cy','width','height')})
    fusion=dict(require_mask=True,min_mask_iou=cfg.get('planning_mask_min_iou',.85),max_centroid_shift_px=cfg.get('planning_mask_max_centroid_shift_px',5.),max_joint_delta_rad=cfg.get('planning_max_joint_delta_rad',.01),erosion_px=cfg.get('mask_erosion_px',2),depth_scale=scale,depth_min_m=cfg.get('depth_min_m',.1),depth_max_m=cfg.get('depth_max_m',2.),mad_scale=cfg.get('depth_mad_scale',3.5),mad_absolute_floor_m=cfg.get('depth_mad_absolute_floor_m',.002),internal_hole_max_area_px=cfg.get('mask_internal_hole_max_area_px',25))
    snap=fuse_stable_samples(samples,**fusion);assert snap.ok,(ns,snap.failure_reason)
    est=estimate_object_geometry(snap.depth_raw,snap.target_depth_raw,snap.object_mask,snap.bbox,K,scale,arrays[ns]['T_base_camera']@opt,snap.source_mode,**geom);assert est.ok,(ns,est.failure_reason)
    estimates[ns]=est;snapshots[ns]=snap
    x,y,w,h=snap.bbox;clearance=min(x,y,K.width-x-w,K.height-y-h)
    row=dict(source_stamp_ns=ns,sample_source_stamps_ns=stamp_list,object_points=len(est.object_points_base),center_base=est.center_base,axes_base=est.axes_base,size_xyz_m=est.size_xyz_m,support_normal_base=est.support_normal_base,support_offset_m=est.support_offset_m,edge_clearance_px=clearance,fusion_parameters=fusion,snapshot_quality=dataclasses.asdict(snap.quality))
    expected=next((x['message'] for x in M['geometry'] if msgstamp(x['message'])==ns),None)
    if expected is not None:
        center=np.array([expected['pose_base']['position'][k] for k in ('x','y','z')]);size=np.array([expected['size_xyz_m'][k] for k in ('x','y','z')]);quat=[expected['pose_base']['orientation'][k] for k in ('x','y','z','w')];axes=quaternion_matrix(quat)[:3,:3]
        check=dict(source_stamp_ns=ns,online_point_count=expected['object_point_count'],reconstructed_point_count=len(est.object_points_base),center_max_abs_error_m=float(np.max(abs(est.center_base-center))),size_max_abs_error_m=float(np.max(abs(est.size_xyz_m-size))),axes_max_abs_error=float(np.max(abs(est.axes_base-axes))),axes_absolute_alignment=(est.axes_base.T@axes).tolist(),support_normal_max_abs_error=float(np.max(abs(est.support_normal_base-np.array([expected['support_normal_base'][k] for k in ('x','y','z')])))),support_offset_abs_error_m=float(abs(est.support_offset_m-expected['support_offset_m'])))
        check['scope']='raw estimate versus published fused enclosure; point count and size are expected to differ here'
        checks.append(check);row['raw_estimate_vs_online_enclosure']=check
    np.savez_compressed(str(OUT/(str(ns)+'_geometry.npz')),object_points_base=est.object_points_base,center_base=est.center_base,axes_base=est.axes_base,size_xyz_m=est.size_xyz_m,support_normal_base=est.support_normal_base,support_offset_m=est.support_offset_m,mask=snap.object_mask,target_depth_raw=snap.target_depth_raw,depth_raw=snap.depth_raw,T_base_camera=arrays[ns]['T_base_camera'])
    views.append(row)
    return mv.SurfaceView(identity,ns,est.object_points_base,est.support_normal_base,est.support_offset_m,clearance)
stamps=M['surface_source_stamps_ns'];reference=view(stamps[0]);surface=mv.fused_surface_from_view(reference,geom['voxel_size_m']);registrations=[];audit_checks=[]
config=mv.RegistrationConfig(**cfg['multiview'])
for ns in stamps[1:]:
    v=view(ns);reg,surface=mv.append_registered_view(surface,reference,v,config,geom['voxel_size_m']);assert reg.ok,convert(reg)
    registrations.append(dict(source_stamp_ns=ns,registration=convert(reg)))
    counts={str(int(k)):int(v) for k,v in zip(*np.unique(surface.view_indices,return_counts=True))}
    if ns in audits:
        p,a=audits[ns];expected=a['multiview_surface'];oldT=np.array(expected['latest_registration']['snapshot_to_reference_transform'])
        audit_checks.append(dict(request_id=a['request_id'],audit_path=str(p),audit_sha256=sha(p),expected_point_count=expected['point_count'],actual_point_count=len(surface.points_base),expected_view_point_counts=expected['view_point_counts'],actual_view_point_counts=counts,registration_transform_max_abs_error=float(np.max(abs(reg.transform_base-oldT)))))
    inv=np.linalg.inv(reg.transform_base)
    np.savez_compressed(str(OUT/(str(ns)+'_surface.npz')),reference_points=surface.points_base,view_indices=surface.view_indices,last_transform=reg.transform_base,current_points=surface.points_base@inv[:3,:3].T+inv[:3,3],support_normal=reference.support_normal_base,support_offset=reference.support_offset_m)
    if str(ns) in M['direct_snapshot_sources']:
        current_surface=dataclasses.replace(surface,points_base=surface.points_base@inv[:3,:3].T+inv[:3,3])
        enclosed,scene_audit=enclose_registered_surface(estimates[ns],current_surface,expected_identity=identity,max_size_m=geom['max_size_m'],max_height_m=geom['max_height_m'])
        expected=next(x['message'] for x in M['geometry'] if msgstamp(x['message'])==ns)
        expected_size=np.array([expected['size_xyz_m'][k] for k in ('x','y','z')])
        enclosure=dict(source_stamp_ns=ns,online_point_count=expected['object_point_count'],reconstructed_point_count=len(enclosed.object_points_base),online_size_xyz_m=expected_size.tolist(),reconstructed_size_xyz_m=enclosed.size_xyz_m.tolist(),size_max_abs_error_m=float(np.max(abs(enclosed.size_xyz_m-expected_size))),scene_audit=scene_audit,snapshot_quality=dataclasses.asdict(snapshots[ns].quality),view_stamps_ns=list(surface.view_stamps_ns),view_sources=surface.view_sources)
        enclosures.append(enclosure)
        np.savez_compressed(str(OUT/(str(ns)+'_contact_inputs.npz')),reference_points=surface.points_base,current_points=current_surface.points_base,view_indices=surface.view_indices,source_stamps_ns=np.array(surface.view_stamps_ns,dtype=np.int64),last_transform=reg.transform_base,geometry_points=enclosed.object_points_base,geometry_center=enclosed.center_base,geometry_axes=enclosed.axes_base,geometry_size=enclosed.size_xyz_m,geometry_support_normal=enclosed.support_normal_base,geometry_support_offset=enclosed.support_offset_m,reference_support_normal=reference.support_normal_base,reference_support_offset=reference.support_offset_m)
out=dict(record=str(R),request_id=28,source_manifest=str(INPUT),source_manifest_sha256=sha(INPUT),config_path=str(cfg_path.resolve()),config_sha256=sha(cfg_path),geometry_parameters=geom,registration_parameters=dataclasses.asdict(config),surface_source_stamps_ns=stamps,views=views,registrations=registrations,prior_audit_checks=audit_checks,raw_estimate_vs_online_enclosure_checks=checks,online_fused_geometry_checks=enclosures,final_point_count=len(surface.points_base),wall_sec=time.monotonic()-start,script_sha256=sha(__file__),limitations=['Offline reconstruction from exact saved measured sources; not the canceled runtime candidate audit or a failed trajectory.', 'The frozen deployed configuration file is used; no claim it is dynamically recovered ROS parameters.', 'Previous-axis state is unavailable, so raw geometry is first estimated with previous_axes=None and compared against recorded ObjectGeometry; any sign difference is explicitly reported.','No ROS node/services, action, live IO, replacement pixels or lowered threshold.'])
p=OUT/'result.json';p.write_text(json.dumps(convert(out),ensure_ascii=False,indent=2)+'\n')
print(json.dumps(convert(dict(path=str(p),sha256=sha(p),wall_sec=out['wall_sec'],prior_audit_checks=audit_checks,online_fused_geometry_checks=[{k:v for k,v in x.items() if k not in ('scene_audit','view_sources')} for x in enclosures],final_point_count=len(surface.points_base))),ensure_ascii=False,indent=2))
