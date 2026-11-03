"""Bounded candidate-search support experiment; original validation unchanged."""
import hashlib,importlib.util,inspect,json,os,sys
from pathlib import Path
from types import SimpleNamespace as NS
W=Path(__file__).resolve().parents[1]
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z')
I=N/'verification/gen28_attempt1_box_edges_r1'
O=N/'verification/gen28_attempt1_local_rgb_edges_prototype_r1'
assert Path('/media/zhuyupei/SanStand').is_mount() and not O.exists();O.mkdir();(O/'tmp').mkdir()
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:1',TMPDIR=str(O/'tmp'),OPENBLAS_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
sys.path.insert(0,str(W/'src/alicia_flexible_grasp_supervisor/src'))
import numpy as np
from alicia_flexible_grasp.vision.regular_box_inference import RegularBoxInference
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry,evaluate_inferred_box_contact
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(name,v):
 p=O/name;assert not p.exists();p.write_text(json.dumps(v,ensure_ascii=False,indent=2,default=lambda x:x.tolist() if isinstance(x,np.ndarray) else str(x))+'\n');return dict(path=str(p),sha256=sha(p))
p=W/'.runtime/prototype_gen27_rgb_direction_box.py';spec=importlib.util.spec_from_file_location('bounded_existing_rgb_prototype',p);proto=importlib.util.module_from_spec(spec);spec.loader.exec_module(proto)
audit=json.loads((I/'result.json').read_text());identity=TargetTrackIdentity(**audit['actual_live_box']['source_target_identity']);proto.TargetTrackIdentity=NS(from_stream=lambda *_:identity)
raw=Path(audit['source_npz']);geometry=I/'geometry.json';assert sha(raw)==audit['source_npz_sha256']
R=Path(audit['record']);record_audit=json.loads(Path(audit['audit_path']).read_text());trigger=record_audit['observe_return']['trigger_evidence']['missing_surface_direction']
parameters=N/'gen28_repositioned_initial_readonly_inputs/parameters.json';values=json.loads(parameters.read_text())['before']['parameters'];cfg=values['/grasp_6d/remote']['gripper_geometry']
assert abs(trigger['opening_fit_clearance_each_side_m']-cfg['opening_fit_clearance_each_side_m'])<1e-12
assert trigger['max_inner_gap_m']==cfg['max_inner_gap_m']==.05
gripper=GripperGeometry(max_inner_gap_m=cfg['max_inner_gap_m'],jaw_clearance_each_side_m=cfg['width_safety_margin_per_side_m'],finger_size_xyz_m=np.array(cfg['finger_box_xyz_m']),palm_size_xyz_m=np.array(cfg['palm_box_xyz_m']),support_clearance_m=cfg.get('support_clearance_m',.003))
fit=cfg['opening_fit_clearance_each_side_m'];physical=.05
original_visual=proto.box._visual_rectangle;original_hough=proto.actual_hough_segments
visual_source=inspect.getsource(original_visual);hough_source=inspect.getsource(original_hough)
assert visual_source.count('np.ones((5,5), np.uint8)')==1
assert hough_source.count('np.ones((5, 5), np.uint8)')==1
dependencies=[p,Path(proto.box.__file__),raw,geometry,I/'result.json',Path(audit['audit_path']),parameters,W/'.runtime/prototype_gen28_attempt1_local_rgb_edges.py',W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/gripper_geometry.py']
before={str(p):sha(p) for p in dependencies};reports=[]
try:
 for radius in (2,3,4,6):
  kernel=2*radius+1
  if radius==2:proto.box._visual_rectangle=original_visual;proto.actual_hough_segments=original_hough
  else:
   vs=visual_source.replace('np.ones((5,5), np.uint8)','np.ones((%d,%d), np.uint8)'%(kernel,kernel))
   vn=dict(original_visual.__globals__);exec(compile(vs,'<candidate_domain_only_radius_%d>'%radius,'exec'),vn);proto.box._visual_rectangle=vn['_visual_rectangle']
   hs=hough_source.replace('np.ones((5, 5), np.uint8)','np.ones((%d, %d), np.uint8)'%(kernel,kernel)).replace('mask_dilation_size_px=5','mask_dilation_size_px=%d'%kernel)
   hn=dict(original_hough.__globals__);exec(compile(hs,'<actual_hough_domain_radius_%d>'%radius,'exec'),hn);proto.actual_hough_segments=hn['actual_hough_segments']
  report=proto.qualify_source(raw,geometry,'exact_nearfield_source_with_actual_audit_geometry',[
   'Only Canny candidate-search mask dilation and matching Hough-direction domain differ for radius>2. Line support dilation remains original5x5; all depth/contour/uncertainty gates unchanged.',
   'All directions come from real RGB Hough segments, at most12 plus original axis, half-degree dedup. No arbitrary angle grid.',
   'Historical source identity preserved for offline provenance only. Prototype is not deployed or executable motion authority.'])
  report['target_identity']=dict(epoch=identity.epoch,track_id=identity.track_id);report['candidate_domain_radius_px']=radius
  report['candidate_domain_radius_nominal_m']=radius*audit['replay']['audit']['metric_pixel_size_m']
  for row in report['candidates']:
   if not row['ok']:continue
   model=RegularBoxInference.from_evidence_dict(row['model']);assert model.source['target_identity']==report['target_identity']
   budgets=[]
   for axis in (0,1):
    e=evaluate_inferred_box_contact(model,model.center_base,model.axes_base[:,axis],-model.support_normal_base,gripper,support_normal_base=model.support_normal_base,opening_fit_clearance_each_side_m=fit)
    required=e.inferred_width_m+e.width_uncertainty_m+2*fit;reserve=max(gripper.jaw_clearance_each_side_m,e.width_uncertainty_m)
    budgets.append(dict(axis=axis,evidence_ok=e.ok,evidence_code=e.code,inferred_width_m=e.inferred_width_m,width_uncertainty_m=e.width_uncertainty_m,required_open_width_m=required,
     physical_opening_m=physical,fit_each_side_m=fit,aperture_margin_m=physical-required,required_precision_reserve_m=reserve,
     aperture_and_precision_pass=e.ok and required<=physical+1e-12 and physical-required>=reserve,targeted_view=dict(e.targeted_view)))
   row['opening_budgets']=budgets
  receipt=save('radius_%d.json'%radius,report);reports.append(dict(**receipt,radius=radius,candidate_count=report['candidate_count'],qualified_count=report['qualified_rgb_candidate_count']))
  print(json.dumps(reports[-1],ensure_ascii=False),flush=True)
finally:proto.box._visual_rectangle=original_visual;proto.actual_hough_segments=original_hough
assert before=={str(p):sha(p) for p in dependencies}
print(json.dumps(save('receipt.json',dict(experiments=reports,source_and_input_sha256=before,source_and_input_unchanged=True,production_files_edited=False,
 candidate_domain_only_changes=True,independent_qualification_and_uncertainty_unchanged=True,no_arbitrary_angle_scan=True,no_ros_calls=True,no_hardware_access=True,no_temperature_reads=True,
 configuration_binding='Pre-attempt passive parameter capture; fit clearance and50mm opening independently match actual initial trigger within this same sealed task.',
 physical_grasp_verified=False,not_deployed=True)),ensure_ascii=False),flush=True)
