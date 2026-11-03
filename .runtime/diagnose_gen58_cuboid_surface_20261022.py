"""Finite pure replay of the exact sealed GEN58 frame and measured surface."""
import os
os.environ.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
from pathlib import Path
import sys, json, hashlib, socket, gzip, time
from types import SimpleNamespace
W = Path(__file__).resolve().parents[1]
B = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N = B/'ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1'
O = N/'actual_attempt_r1_geometry/cuboid_exact_frame_r1'
assert Path('/media/zhuyupei/SanStand').is_mount()
attempts = []
class OfflineSocket(socket.socket):
    def __new__(cls, *a, **kw):
        attempts.append('socket'); raise RuntimeError('offline sockets prohibited')
socket.socket = OfflineSocket
sys.path.insert(0, str(W/'src/alicia_flexible_grasp_supervisor/src'))
import numpy as np
import cv2
cv2.setNumThreads(1)
from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.vision.regular_box_inference import qualify_regular_box_top
from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry, _bilateral_surface_measurement_details
from alicia_flexible_grasp.grasp.tabletop_geometry_candidates import _plane_basis_x
sha = lambda b: hashlib.sha256(b).hexdigest()
def plain(v):
    if isinstance(v,np.ndarray): return v.tolist()
    if isinstance(v,np.generic): return v.item()
    if isinstance(v,dict): return {k:plain(x) for k,x in v.items()}
    if isinstance(v,(tuple,list)): return [plain(x) for x in v]
    return v
def save(name,v):
    (O/name).write_text(json.dumps(plain(v),indent=2,ensure_ascii=False,allow_nan=False)+'\n')
audit_path=N/'actual_attempt_r1_geometry/terminal_gate_audit.json'
d=json.loads(audit_path.read_text()); g=d['replay_geometry']
cpath=N/'full_restart_r1/scene_start_r1/camera_configuration.json'
c=json.loads(cpath.read_text()); K=SimpleNamespace(**{k:c[k] for k in ['fx','fy','cx','cy','height','width']})
a=np.load(O/'exact_rgbd_mask.npz',allow_pickle=False)
expected=d['remote_diagnostics']['tabletop_geometry']['regular_box_inference']
for k in ('color','depth','mask'):
    assert sha(np.ascontiguousarray(a[k]).tobytes())==expected['retained_pixel_sha256'][k+'_sha256']
geometry=GeometryEstimate(True,'','',np.array(g['obb_center_base_m']),np.array(g['R_base_obb']),np.array(g['obb_size_xyz_m']),np.array(g['support_normal_base']),g['support_offset_m'],g['support_inlier_ratio'],np.array(g['object_points_base_m']),g['source_mode'])
identity=TargetTrackIdentity(epoch=4,track_id='g1-t4')
captures=[]
def tracer(frame,event,arg):
    if frame.f_code.co_name=='infer_cuboid_rgb_interval' and event=='line' and frame.f_lineno==751:
        names=('edge','yaw','axis','hu','top_height','result','groups','seed_groups','fitted','proposal_window','origin','tn','pixel_size','rms','slab','vertical','nominal_lo','nominal_hi','nominal_A','A','lo','hi','model')
        captures.append({k:plain(frame.f_locals[k]) for k in names if k in frame.f_locals})
    return tracer
start=time.monotonic(); sys.settrace(tracer)
try:
    result=qualify_regular_box_top(a['depth'],a['mask'],K,c['depth_scale'],np.array(d['snapshot_transform']['T_base_optical']),geometry,identity,d['snapshot_stamp_ns'],color_bgr=a['color'])
finally: sys.settrace(None)
elapsed=time.monotonic()-start
actual=plain(dict(result.audit))
checks={k:actual.get(k)==v for k,v in expected.items() if k in actual}
save('qualification_replay.json',dict(ok=result.ok,code=result.code,reason=actual.get('reason'),audit=actual,shared_original_audit_fields_equal=checks,elapsed_sec=elapsed))
assert result.code==expected['code']
assert actual['cuboid_qualification_reason']==expected['cuboid_qualification_reason']
assert actual['cuboid_rgb_interval']['top_mask_coverage']==expected['cuboid_rgb_interval']['top_mask_coverage']
assert len(captures)==1
cap=captures[0]
save('failure_locals.json',cap)
metadata_path=N/'actual_attempt_r1_handoff/camera_metadata_r1/camera_runtime_metadata.json'
metadata=json.loads(metadata_path.read_text())
keys=['fx','fy','cx','cy','depth_scale','width','height']
assert all(all(row['data']['parameters'][k]==c[k] for k in keys) for row in metadata['rows'])
save('recorded_camera_binding.json',dict(path=str(metadata_path),sha256=sha(metadata_path.read_bytes()),rows_checked=len(metadata['rows']),all_K_scale_dimensions_same=True,image_time_association='none',camera_parameters={k:c[k] for k in keys}))

ref=d['multiview_surface']['fused_scene_geometry']['surface_replay']['references']['snapshot']
path=Path(ref['store_root'])/ref['relative_path']; raw=path.read_bytes(); assert sha(raw)==ref['gzip_sha256']
blob=gzip.decompress(raw); assert sha(blob)==ref['object_sha256']; f=json.loads(blob)
surface=FusedTargetSurface(identity,np.array(f['points_base']),np.array(f['view_indices']),tuple(f['view_stamps_ns']),tuple(tuple(x) for x in f['view_sources']))
gripper=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)
n=geometry.support_normal_base; bx=_plane_basis_x(n); by=np.cross(n,bx)
heights=(surface.points_base-geometry.center_base)@n
def side_info(mask):
    count=int(np.count_nonzero(mask)); q=np.quantile(heights[mask],[.05,.95]) if count else None
    return dict(points=count,support_height_5_95_m=q,span_m=None if q is None else float(q[1]-q[0]),
        per_source=[dict(source_index=i,source_stamp_ns=st,supported_count=sum(bool(mask[j]) and i in vs for j,vs in enumerate(surface.view_sources)),exclusive_count=sum(bool(mask[j]) and tuple(vs)==(i,) for j,vs in enumerate(surface.view_sources)),representative_count=int(np.count_nonzero(mask & (surface.view_indices==i)))) for i,st in enumerate(surface.view_stamps_ns)])
rows=[]
for angle in (45.,5.,0.,105.,51.83821585740807,53.41910792870405):
    jaw=bx*np.cos(np.deg2rad(angle))+by*np.sin(np.deg2rad(angle))
    counts,lo,hi,common,independent=_bilateral_surface_measurement_details(surface,geometry.center_base,jaw,-n,gripper,12,n)
    rows.append(dict(angle_deg=angle,counts=counts,common_insertion_bounds_m=None if lo is None else [lo,hi],
        common_band_thickness_m=None if lo is None else hi-lo,
        independent_sides=[side_info(x) for x in independent] if independent is not None else None,
        common_sides=[side_info(x) for x in common] if isinstance(common,tuple) else None))
save('measured_side_source_contributions.json',dict(rows=rows,gripper_geometry_source='grasp_params.yaml current source; exact geometry matches four original gate profiles below',minimum_points_per_side=12,source_coordinate_frame=f['coordinate_frame'],view_sources_semantics='Voxel support provenance; does not certify distinct camera viewpoints or original exact point coordinates.',raw_surface_gzip_path=str(path),raw_surface_gzip_sha256=ref['gzip_sha256'],raw_surface_object_sha256=ref['object_sha256']))
source_paths=[Path(v.__file__) for k,v in sorted(sys.modules.items()) if k.startswith('alicia_flexible_grasp') and getattr(v,'__file__',None)]
provenance=dict(source_stamp_ns=d['snapshot_stamp_ns'],terminal_audit_sha256=sha(audit_path.read_bytes()),camera_configuration_path=str(cpath),camera_configuration_sha256=sha(cpath.read_bytes()),image_archive_sha256=sha((O/'exact_rgbd_mask.npz').read_bytes()),script_sha256=sha(Path(__file__).read_bytes()),sources=[dict(path=str(p),sha256=sha(p.read_bytes())) for p in source_paths],production_modified=False,thresholds_modified=False,socket_attempts=attempts,ROS_initialized=False,execution_authorized=False)
save('provenance.json',provenance)
print(json.dumps(dict(code=result.code,reason=actual.get('reason'),precision={k:cap[k] for k in ('edge','yaw','axis','hu','top_height')},all_shared_original_fields_equal=all(checks.values()),band_thicknesses=[(r['angle_deg'],r['common_band_thickness_m']) for r in rows],elapsed_sec=elapsed),indent=2))
