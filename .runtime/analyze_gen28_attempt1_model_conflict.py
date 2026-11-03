"""Classify the bounded RGB prototype's real measured-point conflict."""
import hashlib,importlib.util,inspect,json,os,sys
from pathlib import Path
W=Path(__file__).resolve().parents[1]
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z')
I=N/'verification/gen28_attempt1_box_edges_r1';O=N/'verification/gen28_attempt1_local_rgb_edges_prototype_r1'
assert Path('/media/zhuyupei/SanStand').is_mount() and O.is_dir()
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:1',TMPDIR=str(O/'tmp'),PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1')
sys.path.insert(0,str(W/'src/alicia_flexible_grasp_supervisor/src'))
import numpy as np,cv2
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.vision.fused_scene_geometry import _array_digest
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
spec=importlib.util.spec_from_file_location('rgb_direction_prototype',W/'.runtime/prototype_gen27_rgb_direction_box.py');p=importlib.util.module_from_spec(spec);spec.loader.exec_module(p)
r=json.loads((O/'radius_3.json').read_text());candidate=r['candidates'][6];assert candidate['code']=='BOX_MEASURED_GEOMETRY_CONFLICT' and candidate['visual_rectangle_found']
a=json.loads((I/'result.json').read_text());live=json.loads(Path(a['audit_path']).read_text());scene=live['multiview_surface']['fused_scene_geometry']
raw=np.load(a['source_npz'],allow_pickle=False);geometry=p.load_geometry(I/'geometry.json');points=np.asarray(geometry.object_points_base)
n=scene['current_raw_point_count'];assert _array_digest(points[:n])==scene['current_raw_points_sha256']
assert _array_digest(points)==scene['enclosed_points_sha256']
identity=TargetTrackIdentity(**a['actual_live_box']['source_target_identity'])
original_visual=p.box._visual_rectangle
source=inspect.getsource(original_visual).replace('np.ones((5,5), np.uint8)','np.ones((7,7), np.uint8)');vn=dict(original_visual.__globals__)
exec(compile(source,'<candidate_domain_only_radius3>','exec'),vn);p.box._visual_rectangle=vn['_visual_rectangle']
namespace,context,qualifier=p.make_qualifier();namespace['_prototype_axes']=np.array(candidate['axes_base']);captured={}
def trace(f,event,arg):
 if f.f_code is qualifier.__code__ and event=='return':
  for k in ('center','axes','size','edge_uncertainty','height_uncertainty','yaw_uncertainty','excess','origin','tn','corners2','n','top_height','d'):
   if k in f.f_locals:captured[k]=f.f_locals[k]
 return trace
sys.settrace(trace)
try:result=qualifier(raw['depth_raw'],raw['object_mask'],raw['intrinsics'],float(raw['depth_scale_m']),raw['T_base_optical'],geometry,identity,int(raw['source_stamp_ns']),color_bgr=raw['color'])
finally:sys.settrace(None);p.box._visual_rectangle=original_visual
assert result.code=='BOX_MEASURED_GEOMETRY_CONFLICT' and abs(captured['excess']-candidate['audit']['maximum_observed_model_excess_m'])<1e-12
local=(points-captured['center'])@captured['axes'];excess=np.abs(local)-captured['size']/2-np.array([captured['edge_uncertainty']]*2+[captured['height_uncertainty']]);bad=np.any(excess>.001,axis=1)
rows=[]
for name,sl in [('current_fused_snapshot_raw_points',slice(0,n)),('additional_registered_fused_history_points',slice(n,None))]:
 x=excess[sl];q=points[sl];sel=bad[sl];rows.append(dict(source=name,count=len(x),conflicting_count=int(np.count_nonzero(sel)),maximum_excess_m=float(max(0,np.max(x))),
  per_axis_maximum_excess_m=np.maximum(0,np.max(x,axis=0)).tolist(),per_axis_conflicting_count=np.count_nonzero(x>.001,axis=0).tolist(),
  conflicting_base_aabb_min=q[sel].min(axis=0).tolist() if np.any(sel) else None,conflicting_base_aabb_max=q[sel].max(axis=0).tolist() if np.any(sel) else None))
indices=np.where(bad)[0];worst=indices[np.argsort(np.max(excess[indices],axis=1))[::-1]][:12]
overlay=raw['color'].copy();project=p.box._project(points,raw['T_base_optical'],raw['intrinsics']);inside=(project[:,0]>=0)&(project[:,0]<overlay.shape[1])&(project[:,1]>=0)&(project[:,1]<overlay.shape[0])
for index in indices[inside[indices]]:
 cv2.circle(overlay,tuple(np.rint(project[index]).astype(int)),1,(0,0,255) if index<n else (0,128,255),-1)
op=O/'measured_conflict_overlay.png';assert not op.exists();cv2.imwrite(str(op),overlay)
report=dict(prototype_candidate=6,prototype_radius_px=3,prototype_actual_rgb_direction_deg=candidate['relative_angle_deg'],candidate_report_sha256=sha(O/'radius_3.json'),
 original_actual_nearfield_audit=a['audit_path'],audit_sha256=sha(a['audit_path']),current_raw_prefix_count=n,current_raw_prefix_hash_matches=True,
 prefix_provenance_contract='enclose_registered_surface exact XYZ union keeps current raw first; prefix and entire union independently match recorded typed-array hashes',
 model_dimensions_m=captured['size'].tolist(),empirical_edge_uncertainty_m=captured['edge_uncertainty'],empirical_height_uncertainty_m=captured['height_uncertainty'],empirical_yaw_uncertainty_rad=captured['yaw_uncertainty'],
 groups=rows,worst_points=[dict(index=int(i),origin='current_raw' if i<n else 'additional_registered',point_base=points[i].tolist(),projected_uv=project[i].tolist(),excess_xyz_m=excess[i].tolist()) for i in worst],
 scene_enclosure=dict(old_size_xyz_m=scene['old_size_xyz_m'],new_size_xyz_m=scene['new_size_xyz_m'],view_stamps_ns=scene['view_stamps_ns']),
 overlay=dict(path=str(op),sha256=sha(op),legend='red=current raw conflict, orange=additional registered-history conflict'),
 interpretation='Pixel-domain expansion can recover an independently evidenced RGB rectangle, but the unchanged model-consistency gate must still reconcile registered historical measurements. No points are removed and no model is admitted here.',
 no_ros_calls=True,no_hardware_access=True,no_temperature_reads=True,production_modified=False,physical_grasp_verified=False)
out=O/'measured_conflict_analysis.json';assert not out.exists();out.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(dict(path=str(out),sha256=sha(out),groups=rows),ensure_ascii=False),flush=True)
