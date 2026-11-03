#!/usr/bin/env python3
"""One new unselected source through existing pure production vision methods."""
from pathlib import Path
from dataclasses import asdict, fields
from types import SimpleNamespace as NS
import hashlib, json, os, socket, sys, time, traceback
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
C=E/'ros_log/full_latest_gen29_restart_20261016T000728Z'
P=C/'gen35_current_pose_analysis_scene_r4'
O=C/'verification/gen35_current_pose_geometry_r1'
assert Path('/media/zhuyupei/SanStand').is_mount()
O.mkdir(exist_ok=False);(O/'tmp').mkdir()
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:1',ROS_LOG_DIR=str(O/'ros'),TMPDIR=str(O/'tmp'),PYTHONDONTWRITEBYTECODE='1')
class BoundaryViolation(BaseException):pass
attempts=[]
def forbidden(*a,**k):attempts.append(repr(a)[:200]);raise BoundaryViolation('offline only')
socket.socket.connect=forbidden;socket.socket.connect_ex=forbidden;socket.socket.sendto=forbidden
socket.create_connection=forbidden;socket.getaddrinfo=forbidden
for p in ('src/alicia_grasp_modes/src','src/alicia_flexible_grasp_supervisor/src','devel/lib/python3/dist-packages'):
    sys.path.insert(0,str(W/p))
sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def serial(v):
    if isinstance(v,np.ndarray):return v.tolist()
    if isinstance(v,np.generic):return v.item()
    raise TypeError(type(v).__name__)
report=dict(kind='unselected_single_current_pose_production_vision_diagnosis',passed=False,
    execution_authorized=False,runtime_target_identity_used=False,manual_roi_used=False,
    source_admission_completed=False,live_ros_contacted=False,motion_commands_sent=0,production_modified=False)
try:
    import numpy as np,cv2,rospy
    for name in ('init_node','Publisher','Subscriber','Service','ServiceProxy','get_master','get_param','get_param_cached','set_param','delete_param','wait_for_service'):
        setattr(rospy,name,forbidden)
    from tf.transformations import quaternion_matrix
    from alicia_grasp_modes import tabletop as table,rgb_instance as rgb
    from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask
    from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample,fuse_stable_samples
    from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
    from alicia_flexible_grasp.vision.object_geometry import estimate_object_geometry
    from alicia_flexible_grasp.vision import regular_box_inference as box
    from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
    from alicia_flexible_grasp.grasp.tabletop_geometry_candidates import TabletopGeometryConfig,generate_tabletop_proposals
    receipt=read(P/'result.json');params=read(P/'configuration.json');afterparams=read(P/'configuration_after.json')
    inputs={str(P/'result.json'):sha(P/'result.json')}
    for name,digest in receipt['files_sha256'].items():
        assert sha(P/name)==digest;inputs[str(P/name)]=digest
    assert receipt['planning_configuration_unchanged'] and receipt['maximum_joint_delta_rad']==0
    assert not receipt['execution_authorized'] and not receipt['target_identity_assigned']
    for data in (params,afterparams):data['/camera']['runtime_profile']['stationary_temporal'].pop('regions',None)
    assert params==afterparams
    camera=params['/camera'];remote=params['/grasp_6d/remote'];perception=params['/perception'];unknown=params['/unknown_perception']
    color=cv2.imread(str(P/'current_scene.png'));z=np.load(P/'current_depth.npz');raw=z['depth'];ns=int(z['stamp_ns'])
    assert ns==receipt['image_stamp_ns'] and color.shape[:2]==raw.shape
    scale=float(camera['depth_scale']);depth=raw.astype(np.float32)*scale
    K=np.array([camera[k] for k in ('fx','fy','cx','cy')]);intrinsics=NS(**{k:camera[k] for k in ('fx','fy','cx','cy','width','height')})
    tr=next(x for x in receipt['transforms'] if x['parent']=='base_link' and x['child']=='camera_link')
    assert tr['stamp_ns']==ns;T=quaternion_matrix(tr['rotation_xyzw']);T[:3,3]=tr['translation']
    optical=np.eye(4);optical[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]];Topt=T@optical
    cfg=table.Config(**{f.name:unknown[f.name] for f in fields(table.Config) if f.name in unknown})
    sources={str(Path(m.__file__).resolve()):sha(m.__file__) for m in list(sys.modules.values())
        if getattr(m,'__file__',None) and str(Path(m.__file__).resolve()).startswith(str(W/'src')) and str(m.__file__).endswith('.py')}
    release=read(E/'ros_log/full_latest_gen22_restart_20261015T003700Z/verification/gen35_release_manifest.json')
    assert all(release['source_sha256'][str(Path(p).relative_to(W))]==h for p,h in sources.items())
    started=time.monotonic();cpu=time.process_time()
    valid=rectification_rgb_valid_mask(camera,depth.shape)
    segmented=table.segment(depth,K,T,cfg)
    rows=[dict(index=i,bbox=c.bbox,center_uv=c.center_uv,position_base=c.position_base,
        extent_m=c.extent_m,quality=c.quality,mask_pixels=int(np.count_nonzero(c.mask)),metrics=c.metrics) for i,c in enumerate(segmented.candidates)]
    initial,reason=table.TargetTracker(cfg).choose(segmented,depth.shape)
    chosen_index=next((i for i,c in enumerate(segmented.candidates) if c is initial),None)
    report.update(input_sha256=inputs,source_before=sources,source_stamp_ns=ns,T_base_camera_link=T,T_base_optical=Topt,
        effective_tabletop_config=asdict(cfg),segmentation=dict(metrics=segmented.metrics,plane_base=segmented.plane_base,candidates=rows),
        production_center_choice=dict(reason=reason,candidate_index=chosen_index),snapshot_required_frames=remote['planning_snapshot_frames'],
        near_field_snapshot_required_frames=remote['near_field_planning_snapshot_frames'])
    if initial is not None and initial.quality>=float(unknown.get('minimum_geometry_quality',.5)):
        refined=rgb.refine_candidate(color,depth,K,T,initial,cfg,rgb_valid=valid) if unknown.get('rgb_instance_refinement',True) else initial
        cv2.imwrite(str(O/'production_refined_mask.png'),refined.mask)
        report['refined']=dict(bbox=refined.bbox,quality=refined.quality,metrics=refined.metrics,mask_pixels=int(np.count_nonzero(refined.mask)),
            mask_sha256=hashlib.sha256(np.ascontiguousarray(refined.mask).tobytes()).hexdigest())
        identity=TargetTrackIdentity(1,'offline-unselected-gen35-current-pose')
        q=np.array(receipt['sdk']['positions'][:6])
        sample=RgbdSample(color,raw,refined.mask,refined.bbox,None,ns/1e9,'camera_link',q,ns,identity.epoch,identity)
        sk=dict(require_mask=True,min_mask_iou=remote['planning_mask_min_iou'],max_centroid_shift_px=remote['planning_mask_max_centroid_shift_px'],
            max_joint_delta_rad=remote['planning_max_joint_delta_rad'],erosion_px=remote['mask_erosion_px'],depth_scale=scale,
            depth_min_m=perception['depth_min_m'],depth_max_m=perception['depth_max_m'],mad_scale=remote['depth_mad_scale'],
            mad_absolute_floor_m=remote['depth_mad_absolute_floor_m'],internal_hole_max_area_px=0)
        snapshot=fuse_stable_samples([sample],**sk)
        report['single_source_snapshot']=dict(ok=snapshot.ok,failure_code=snapshot.failure_code,failure_reason=snapshot.failure_reason,
            fused_frames=1,diagnostic_only=True,temporal_three_source_admission_not_proven=True,arguments=sk)
        if snapshot.ok:
            gg=remote['gripper_geometry']
            gk=dict(support_bbox_expand_ratio=remote.get('support_bbox_expand_ratio',.30),
                support_distance_threshold_m=remote.get('support_distance_threshold_m',remote.get('target_cloud_support_plane_inlier_distance_m',.004)),
                voxel_size_m=remote['target_cloud_voxel_size_m'],min_support_points=remote.get('geometry_min_support_points',200),
                min_object_points=remote.get('geometry_min_object_points',remote.get('target_cloud_min_points',120)),
                min_size_m=remote.get('geometry_min_size_m',.005),max_size_m=remote.get('geometry_max_size_m',.600),
                max_height_m=remote.get('geometry_max_height_m',.500),previous_axes_base=None,
                outlier_neighbors=remote['target_cloud_outlier_neighbors'],outlier_std_ratio=remote['target_cloud_outlier_std_ratio'],
                support_minimum_span_m=max(gg['finger_box_xyz_m'][0],gg['max_inner_gap_m']+2*gg['finger_box_xyz_m'][1])+2*gg.get('support_clearance_m',.003))
            geo=estimate_object_geometry(snapshot.depth_raw,snapshot.target_depth_raw,snapshot.object_mask,snapshot.bbox,intrinsics,scale,Topt,snapshot.source_mode,**gk)
            report['geometry']=dict(ok=geo.ok,code=geo.failure_code,reason=geo.failure_reason,arguments=gk,
                center_base=geo.center_base,axes_base=geo.axes_base,size_xyz_m=geo.size_xyz_m,measured_points=len(geo.object_points_base))
            if geo.ok:
                qualified=box.qualify_regular_box_snapshot(snapshot,intrinsics,scale,Topt,geo,identity,ns)
                report['box']=dict(ok=qualified.ok,code=qualified.code,audit=dict(qualified.audit),model=qualified.model.to_evidence_dict() if qualified.model else None)
                if qualified.model:
                    planning,enclosure=box.enclose_inferred_box_geometry(geo,qualified.model)
                    gripper=GripperGeometry(gg['max_inner_gap_m'],gg['width_safety_margin_per_side_m'],gg['finger_box_xyz_m'],gg['palm_box_xyz_m'],gg.get('support_clearance_m',.003))
                    tc=remote['tabletop_geometry_candidates'];values={f.name:tc[f.name] for f in fields(TabletopGeometryConfig) if f.name in tc}
                    values.update(max_inner_gap_m=gg['max_inner_gap_m'],opening_fit_clearance_each_side_m=gg['opening_fit_clearance_each_side_m'])
                    config=TabletopGeometryConfig(**values)
                    proposal=generate_tabletop_proposals(planning.object_points_base,planning.center_base,planning.axes_base,planning.size_xyz_m,
                        -planning.support_offset_m*planning.support_normal_base,planning.support_normal_base,config,inferred_box=qualified.model,finger_geometry=gripper)
                    report['tabletop_proposals']=dict(ok=proposal.ok,count=len(proposal.proposals),failure_code=proposal.failure_code,
                        failure_reason=proposal.failure_reason,surface_evidence_rejections=proposal.surface_evidence_rejections,
                        enclosure=enclosure,config=asdict(config),full_remote_contact_gate_not_run=True)
    report.update(passed=True,source_after={p:sha(p) for p in sources},source_unchanged=all(sha(p)==h for p,h in sources.items()),
        input_unchanged=all(sha(p)==h for p,h in inputs.items()),wall_sec=time.monotonic()-started,cpu_sec=time.process_time()-cpu,
        external_boundary_attempts=attempts,script_sha256=sha(__file__),
        limitations=['Single unselected source: no three-frame acquisition, current source admission, historical registration or live target identity is asserted.',
            'Only original production center selection/refinement is used; no manual ROI, frozen old mask, or old lock is reused.',
            'Any generated proposal is a pure geometric diagnostic. Full remote materialization, collision, IK, strict MoveIt and execution are not run.',
            'This image includes a forearm; it is not a fresh clear-workspace or motion authorization receipt.'])
    assert report['source_unchanged'] and report['input_unchanged'] and not attempts
except BaseException as exc:
    report.update(error_type=type(exc).__name__,error=str(exc),traceback=traceback.format_exc(),external_boundary_attempts=attempts)
(O/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,default=serial,allow_nan=False)+'\n')
print(json.dumps(dict(output=str(O),sha256=sha(O/'result.json'),passed=report['passed'],choice=report.get('production_center_choice'),
    box=report.get('box',{}).get('code'),error=report.get('error')),ensure_ascii=False))
if not report['passed']:sys.exit(1)
