"""Source-bound TF surrogate comparison; endpoint diagnostics, no ROS or motion."""
import os
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
from pathlib import Path
import sys,argparse,json,hashlib,socket,time,resource,copy
W=Path(__file__).resolve().parents[1];B=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller');N=B/'ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1'
p=argparse.ArgumentParser();p.add_argument('--variant',choices=['baseline','candidate_r2'],required=True);args=p.parse_args()
O=N/'observation_pose_fix/offline_selection_surrogate_r1'/args.variant/'run_r4';I=N/'observation_pose_fix/offline_selection_r1/source_attempt2';I1=I.parent/'source_attempt1'
assert Path('/media/zhuyupei/SanStand').is_mount();O.mkdir(parents=True,exist_ok=False)
C=W if args.variant=='baseline' else W/'.runtime/gen59_observation_pose_candidate_r2'
mp=N/'observation_pose_fix/verification/freeze_r2/manifest_uncollected.json';sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert sha(mp)=='7c252a59c7ee908c58d90b6fbcd7a61569d8bd7e99d84e755f7d7185db1c2e1c';freeze=json.loads(mp.read_text());pin=freeze['baseline_live_source_sha256'] if args.variant=='baseline' else freeze['copied_input_sha256']
blocked=[]
class NoSocket(socket.socket):
 def __new__(cls,*a,**k):blocked.append('socket');raise RuntimeError('offline sockets prohibited')
socket.socket=NoSocket
sys.path[:0]=[str(C/'src/alicia_grasp_modes/src'),str(C/'src/alicia_flexible_grasp_supervisor/src')]
import numpy as np
from alicia_grasp_modes import reachable_observation_views as views
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk

def module_sources():
 out={}
 for name,m in list(sys.modules.items()):
  f=getattr(m,'__file__',None)
  if not f or not name.startswith(('alicia_grasp_modes','alicia_flexible_grasp')):continue
  fp=Path(f).resolve()
  if fp.suffix=='.py' and str(fp).startswith(str(C)+'/'):
   rel=str(fp.relative_to(C));assert rel in pin,(name,rel);actual=sha(fp);assert actual==pin[rel],(rel,actual,pin[rel]);out[rel]=actual
 return out
before=module_sources();meta=json.loads((I/'source.json').read_text());r2=json.loads((I/'reconstruction.json').read_text());r1=json.loads((I1/'reconstruction.json').read_text())
assert sha(I/'source.npz')==r2['source_npz_sha256']=='8ff71fe936f2e9d2ce9bac2cbe0c33b250c265931084b11a3c552627e722172f'
with np.load(I/'source.npz') as z:arrays={k:z[k].copy() for k in z.files}
source=copy.deepcopy(meta['source']);source.update({k:arrays[k] for k in ['points_base','corners_base','center_base','tool_from_camera','base_from_tool']});source['arrival_projection']=dict(audit=meta['arrival_audit'],**{k:arrays[k] for k in ['prism_base','rgb_valid','invalid_integral']})
# Keep original audit for mismatch disclosure only; generate_reachable_observation_views
# consumes explicit arrays, geometry and binding, never a fake positive source qualification.
model=Path(meta['model_path']);assert sha(model)==meta['model_sha256'];fk=SerialUrdfFk(model.read_text(),['Joint%d'%i for i in range(1,7)])
kwargs=dict(source=source,current_joint_positions=meta['current_joint_positions'],fk=fk,gripper=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003),opening_width_m=meta['opening_width_m'],gaps=meta['gaps'],depth_uncertainty_m=.003,side_depth_uncertainty_m=meta['original_search_audit']['optical_side_uncertainty_m'],joint_error_bounds_rad=meta['joint_error_bounds_rad'],max_directions=2,max_iterations=96,max_seconds=2.,proposal_image_reserve_px=4.)
start_cpu=time.process_time();start=time.monotonic();rows,audit=views.generate_reachable_observation_views(**kwargs);wall=time.monotonic()-start;cpu=time.process_time()-start_cpu
for row in rows:
 T=np.asarray(row['T_base_camera_link']);row['surrogate_geometry_diagnostics']=dict(camera_to_current_obb_center_distance_m=float(np.linalg.norm(T[:3,3]-arrays['center_base'])),nominal_18_to_22_cm_distance=float(np.linalg.norm(T[:3,3]-arrays['center_base']))>=.18 and float(np.linalg.norm(T[:3,3]-arrays['center_base']))<=.22,FK_matches_joint_endpoint=bool(np.allclose(fk(row['q']),row['T_base_tool0'],atol=1e-12,rtol=0)))
ros=sys.modules.get('rospy');ros_initialized=False if ros is None else ros.core.is_initialized()
assert not blocked and not ros_initialized and 'rosbag' not in sys.modules
now=module_sources();assert all(now[k]==v for k,v in before.items())

def plain(v):
 if isinstance(v,dict):return {str(k):plain(x) for k,x in v.items()}
 if isinstance(v,(tuple,list)):return [plain(x) for x in v]
 if isinstance(v,np.ndarray):return v.tolist()
 if isinstance(v,np.generic):return v.item()
 return v
report=dict(schema=1,scope='source_bound_TF_reconstruction_surrogate_endpoint_diagnostic_only',variant=args.variant,exact_original_scene_replay=False,source_stamp_ns=r1['source_stamp_ns'],immutable_input_npz_sha256=sha(I/'source.npz'),input_metadata_sha256=sha(I/'source.json'),source_reconstruction_report_sha256=sha(I/'reconstruction.json'),original_transform_hash_match=False,source_hash_checks=r1['checks'],tf2_source_hash_checks=r2['checks'],candidate_freeze_manifest_sha256=sha(mp),source_inputs=dict(model_sha256=sha(model),source_audit_sha256=meta['source_audit_sha256'],current_joint_positions=meta['current_joint_positions'],joint_error_bounds_rad=meta['joint_error_bounds_rad'],opening_width_m=meta['opening_width_m'],depth_uncertainty_m=.003,side_depth_uncertainty_m=kwargs['side_depth_uncertainty_m'],max_directions=2,max_iterations=96,max_seconds=2.,proposal_image_reserve_px=4.),candidates=rows,audit=audit,python_environment=dict(executable=sys.executable,numpy_version=np.__version__,numpy_path=np.__file__,no_user_site=sys.flags.no_user_site),resource=dict(wall_sec=wall,cpu_sec=cpu,max_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,numerical_threads=1),clock_policy='original 2 second monotonic wall budget; pure source geometry functions have no live freshness gate; no clock extension or deadline bypass',ROS_initialized=ros_initialized,ROS_imported=ros is not None,socket_attempts=blocked,complete_path_checked=False,whole_arm_collision_checked=False,execution_authority=False,limitations=['Original base-optical/support/reference hashes are not reproduced; these are identical surrogate arrays for both algorithm versions, not exact original scene replay.','No service, planning scene, whole-arm collision, continuous path, live freshness, or motion authority was used or established.','Missing-side exposure and optical evidence are geometry predictions; actual side depth and a common contact band remain to be acquired.','18–22 cm is reported as a diagnostic center distance. This pure FK API enforces original 0.07–0.5m optical-depth/image gates; it does not have an independent 18–22cm range input.'],imported_source_sha256=now,helper_sha256=sha(Path(__file__)))
(O/'result.json').write_text(json.dumps(plain(report),indent=2,allow_nan=False)+'\n');(O/'helper_source.py').write_bytes(Path(__file__).read_bytes())
print(json.dumps(dict(variant=args.variant,status=audit['status'],count=len(rows),wall_sec=wall,cpu_sec=cpu,result_sha256=sha(O/'result.json'))))
