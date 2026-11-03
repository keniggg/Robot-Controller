"""Sealed initial source -> actual GEN28 remote entry and candidate generator.

No cloned qualifier and no ROS node/device connection. Original admission
scheduling is not replayed; the immutable request is rebuilt from three actual
samples, corrected xywh geometry, and the pinned original camera transform.
"""
import hashlib,json,os
from pathlib import Path
import sys,time
from dataclasses import asdict,replace

W=Path(__file__).resolve().parents[1]
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N=E/'ros_log/full_latest_gen22_restart_20261015T003700Z'
R=E/'grasp_records/20261015T075503.791411Z_24849e8304a44341adebca3867d29fdd'
V=N/'verification';C=V/'gen28_initial_corrected_bbox_replay';I=V/'gen27_attempt3_initial_inference_review'
O=V/'gen28_actual_raw_box_entry_r3'
assert Path('/media/zhuyupei/SanStand').is_mount()
assert not O.exists();O.mkdir();(O/'tmp').mkdir();(O/'ros').mkdir()
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:1',PYTHONDONTWRITEBYTECODE='1',
    ROS_LOG_DIR=str(O/'ros'),TMPDIR=str(O/'tmp'),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
for p in ('src/alicia_flexible_grasp_supervisor/tests','src/alicia_grasp_modes/tests',
          'src/alicia_flexible_grasp_supervisor/src','src/alicia_grasp_modes/src','devel/lib/python3/dist-packages'):
    sys.path.insert(0,str(W/p))
import numpy as np
import rosbag,rospy,pytest
from _pytest.monkeypatch import MonkeyPatch
from cv_bridge import CvBridge
from types import SimpleNamespace as NS
from test_gen26_inferred_box_remote_pipeline import pipeline
from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample,fuse_stable_samples
from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
from alicia_flexible_grasp.vision.target_observation import TargetObservation,TargetTrackIdentity
from alicia_flexible_grasp.vision.multiview_surface import SurfaceView,fused_surface_from_view
from alicia_flexible_grasp.vision.regular_box_inference import qualify_regular_box_top

def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()
def arrsha(a):return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()
source_paths=[
 'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',
 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/regular_box_inference.py',
 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/rgbd_snapshot.py',
 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/object_geometry.py',
 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/target_observation.py',
 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/multiview_surface.py',
 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/gripper_geometry.py',
 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/tabletop_geometry_candidates.py',
 'src/alicia_flexible_grasp_supervisor/tests/test_gen26_inferred_box_remote_pipeline.py',
 'src/alicia_flexible_grasp_supervisor/tests/test_gen26_regular_box_inference_independent.py',
 'src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py',
 '.runtime/replay_gen28_actual_raw_box_entry.py']
sources={p:sha(W/p) for p in source_paths}
input_paths=[C/'initial_three_frame_reconstruction.npz',C/'initial_three_frame_geometry.json',
 C/'initial_three_frame_replay.json',C/'correction_receipt.json',I/'source_1792050907106353521.npz',
 R/'manifest.json',R/'keyframes.bag',R/'keyframes_audits/17d4f9d7ee2d80d84edb4031f085f42bf0fea965f9b6ec1802bd7054e890f14c.json']
inputs={str(p):sha(p) for p in input_paths}
manifest=json.loads((R/'manifest.json').read_text());assert manifest['recorder_closed'] is True
bag_entry=next(x for x in manifest['files'] if x['path']=='keyframes.bag')
assert inputs[str(R/'keyframes.bag')]==bag_entry['sha256']
report=json.loads((C/'initial_three_frame_replay.json').read_text())
assert report['bbox_convention']=='x_y_width_height'
raw=np.load(I/'source_1792050907106353521.npz',allow_pickle=False)
fused=np.load(C/'initial_three_frame_reconstruction.npz',allow_pickle=False)
ids=report['sample_stamp_ns'];identity=TargetTrackIdentity.from_stream(11,12)
topics=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/perception/object_mask','/perception/object','/joint_states']
frames={ns:{} for ns in ids};joints=[]
with rosbag.Bag(str(R/'keyframes.bag')) as bag:
    for topic,msg,received in bag.read_messages(topics=topics,start_time=rospy.Time(1792050906,300000000),end_time=rospy.Time(1792050907,900000000)):
        ns=msg.header.stamp.to_nsec()
        if topic=='/joint_states':joints.append((ns,msg))
        elif ns in frames:frames[ns][topic]=msg
bridge=CvBridge();samples=[]
for ns in ids:
    row=frames[ns];obj=row['/perception/object'];assert obj.detected
    q=min(joints,key=lambda x:abs(x[0]-ns))[1];joint=dict(zip(q.name,q.position))
    bbox=(obj.bbox_x,obj.bbox_y,obj.bbox_width,obj.bbox_height)
    samples.append(RgbdSample(bridge.imgmsg_to_cv2(row[topics[0]],'bgr8'),bridge.imgmsg_to_cv2(row[topics[1]],'passthrough'),
        bridge.imgmsg_to_cv2(row[topics[2]],'mono8'),bbox,obj,ns/1e9,row[topics[1]].header.frame_id,
        np.asarray([joint['Joint%d'%i] for i in range(1,7)]),ns,12,identity))
snapshot=fuse_stable_samples(samples,**report['snapshot_arguments']);assert snapshot.ok,snapshot.failure_reason
for name in ('depth_raw','target_depth_raw','object_mask'):
    np.testing.assert_array_equal(getattr(snapshot,name),fused[name])
np.testing.assert_array_equal(snapshot.color_bgr,fused['color'])
for name,npz_name in [('depth_raw','depth_raw'),('object_mask','object_mask'),('color_bgr','color')]:
    np.testing.assert_array_equal(getattr(snapshot.original_sample,name),raw[npz_name])
assert snapshot.bbox==(264,202,111,74)
g=json.loads((C/'initial_three_frame_geometry.json').read_text())
geometry=GeometryEstimate(True,'','',g['obb_center_base_m'],g['R_base_obb'],g['obb_size_xyz_m'],g['support_normal_base'],
    g['support_offset_m'],g['support_inlier_ratio'],g['object_points_base_m'],g['source_mode'])
K=NS(fx=raw['intrinsics'][0],fy=raw['intrinsics'][1],cx=raw['intrinsics'][2],cy=raw['intrinsics'][3],
    width=snapshot.depth_raw.shape[1],height=snapshot.depth_raw.shape[0],depth_scale=float(raw['depth_scale_m']))
T=raw['T_base_optical'];bbox=snapshot.bbox
edge=min(bbox[0],bbox[1],K.width-bbox[0]-bbox[2],K.height-bbox[1]-bbox[3])
observed=TargetObservation(identity,snapshot.stamp_ns,'base_link',geometry.object_points_base,geometry.support_normal_base,
    geometry.support_offset_m,bbox,snapshot.depth_raw.shape,edge,'instance_mask','unknown_small')
view=SurfaceView(identity,snapshot.stamp_ns,observed.points_base,observed.support_normal_base,observed.support_offset_m,edge)
surface=fused_surface_from_view(view)
baseline=qualify_regular_box_top(snapshot.depth_raw,snapshot.object_mask,K,K.depth_scale,T,geometry,identity,snapshot.stamp_ns,color_bgr=snapshot.color_bgr)
assert baseline.ok is False and baseline.code=='BOX_TOP_NOT_RECTANGULAR'
before_points=geometry.object_points_base.copy()
before_pixels={k:getattr(snapshot,k).copy() for k in ('depth_raw','target_depth_raw','object_mask','color_bgr')}
monkeypatch=MonkeyPatch();started=time.monotonic()
try:
    node,_scene,_prepared,_params,_surface=pipeline(monkeypatch)
    node._current_stream_target_identity=lambda:identity
    node._latest_target_observation=observed
    node._near_field_reference_view=view
    node._active_multiview_surface=lambda:(surface,view)
    node._camera_intrinsics=lambda:K
    actual_audit_path=R/'keyframes_audits/17d4f9d7ee2d80d84edb4031f085f42bf0fea965f9b6ec1802bd7054e890f14c.json'
    trigger=json.loads(actual_audit_path.read_text())['observe_return']['trigger_evidence']['missing_surface_direction']
    fit=float(trigger['opening_fit_clearance_each_side_m'])
    assert abs(trigger['required_open_width_m']-trigger['whole_obb_width_m']-2*fit)<1e-12
    assert float(trigger['max_inner_gap_m'])==node.gripper_physical_open_width_m==node.gripper_geometry.max_inner_gap_m
    node.tabletop_geometry_config=replace(node.tabletop_geometry_config,opening_fit_clearance_each_side_m=fit)
    node.opening_fit_clearance_each_side_m=fit
    planning=node._prepare_regular_box_snapshot(snapshot,geometry,T)
    captured=node._regular_box_snapshot;result=captured['result'];model=node._current_inferred_box()
    assert result.ok is True and model is not None,result.audit
    assert model.source['stamp_ns']==snapshot.stamp_ns
    assert model.source['target_identity']==dict(epoch=12,track_id='g11-t12')
    expected_hashes={name:arrsha(raw[key]) for name,key in [('depth_sha256','depth_raw'),('mask_sha256','object_mask'),('color_sha256','color')]}
    assert all(model.source[k]==v for k,v in expected_hashes.items())
    assert model.model_sha256==report['raw_full_qualification_with_corrected_fused_geometry']['model']['model_sha256']
    np.testing.assert_array_equal(geometry.object_points_base,before_points)
    np.testing.assert_array_equal(planning.object_points_base,before_points)
    assert captured['geometry'] is geometry
    for name,value in before_pixels.items():np.testing.assert_array_equal(getattr(snapshot,name),value)
    candidates,diagnostics=node._generate_tabletop_candidates(planning,snapshot)
    assert not candidates,diagnostics
    assert diagnostics['regular_box_inference']['ok'] is True
    assert diagnostics['failure_code']=='BOX_INFERENCE_PRECISION_VIEW_REQUIRED',diagnostics
    assert all(p==sha(W/name) for name,p in sources.items()),'production/source changed during replay'
    # Recheck small inputs; immutable closed bag identity is rechecked below.
    for path,digest in inputs.items():
        if path!=str(R/'keyframes.bag'):assert sha(path)==digest
    assert (R/'keyframes.bag').stat().st_size==bag_entry['size_bytes']
    output=dict(actual_production_entry_called=True,corrected_bbox_xywh=True,raw_model_qualified=True,
        fused_pixel_baseline_qualified=False,fused_geometry_preserved=True,raw_source_hashes_match=True,
        physical_grasp_verified=False,actual_production_generator_called=True,
        candidate_count=len(candidates),candidate_generation_failure_code=diagnostics['failure_code'],
        source_sha256=sources,input_sha256=inputs,source_unchanged=True,source_stamp_ns=snapshot.stamp_ns,
        original_sample_stamp_ns=snapshot.original_sample.stamp_ns,original_bbox_xywh=list(snapshot.bbox),sample_stamp_ns=list(snapshot.sample_stamp_ns),
        snapshot_reconstructed_with_actual_fuser=True,fusion_arrays_match_corrected_replay=True,
        baseline=dict(ok=baseline.ok,code=baseline.code,audit=dict(baseline.audit)),raw_result=dict(ok=result.ok,code=result.code,audit=dict(result.audit)),
        model=model.to_evidence_dict(),tabletop_diagnostics=diagnostics,
        effective_tabletop_geometry_config=asdict(node.tabletop_geometry_config),
        candidate_configuration_provenance=dict(opening_fit_clearance_each_side_m=dict(value=fit,source=str(actual_audit_path),source_sha256=inputs[str(actual_audit_path)],
            json_path='observe_return.trigger_evidence.missing_surface_direction.opening_fit_clearance_each_side_m',original_width_plus_two_clearances_verified=True),
            physical_opening_m=dict(value=node.gripper_physical_open_width_m,source=str(actual_audit_path),source_sha256=inputs[str(actual_audit_path)],json_path='observe_return.trigger_evidence.missing_surface_direction.max_inner_gap_m'),
            remaining_config_source='test_gen26_inferred_box_remote_pipeline.pipeline fixture; SHA pinned in source_sha256; full original effective candidate parameter capture unavailable'),
        measured_points_sha256_before=arrsha(before_points),measured_points_sha256_after=arrsha(planning.object_points_base),
        elapsed_sec=time.monotonic()-started,no_ros_calls=True,no_hardware_access=True,no_temperature_reads=True,
        limitations=['Offline immutable request reconstructed from the three exact original samples; asynchronous ROS admission scheduling is not recreated.',
          'Corrected production-geometry reconstruction uses documented YAML/default parameter and nearest-joint pairing assumptions; recorded trigger center/width/height match to floating precision.',
          'Actual remote functions use a minimal offline node fixture. Opening-fit clearance and maximum opening are bound to original trigger audit; remaining standard tabletop/gripper profile is a fixture, not claimed to be the complete original effective parameter set. No MoveIt, robot service, controller or task execution is invoked.',
          'Qualification does not authorize grasping: actual candidate generator rejects aperture/precision with original uncertainty retained.'])
    p=O/'result.json';p.write_text(json.dumps(output,ensure_ascii=False,indent=2,default=lambda x:x.tolist() if isinstance(x,np.ndarray) else str(x))+'\n')
    print(json.dumps(dict(path=str(p),sha256=sha(p),raw_qualified=True,baseline_qualified=False,generator_failure=diagnostics['failure_code'],candidate_count=len(candidates)),ensure_ascii=False))
finally:monkeypatch.undo()
