"""Bounded reuse of the existing actual-Hough direction prototype; offline only."""
import hashlib,importlib.util,json,os,sys
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace as NS
W=Path(__file__).resolve().parents[1]
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z')
I=N/'gen28_repositioned_initial_readonly_inputs'
R=N/'verification/gen28_repositioned_initial_replay_r1'
O=N/'verification/gen28_repositioned_rgb_direction_prototype_r1'
assert Path('/media/zhuyupei/SanStand').is_mount() and not O.exists();O.mkdir();(O/'tmp').mkdir()
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:1',TMPDIR=str(O/'tmp'),PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1')
sys.path.insert(0,str(W/'src/alicia_flexible_grasp_supervisor/src'))
import numpy as np
from alicia_flexible_grasp.vision.regular_box_inference import RegularBoxInference
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry,evaluate_inferred_box_contact
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(name,data):
 p=O/name;assert not p.exists();p.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False,default=lambda v:v.tolist() if isinstance(v,np.ndarray) else str(v))+'\n');return dict(path=str(p),sha256=sha(p))
prototype_path=W/'.runtime/prototype_gen27_rgb_direction_box.py'
spec=importlib.util.spec_from_file_location('existing_rgb_direction_prototype',prototype_path);prototype=importlib.util.module_from_spec(spec);spec.loader.exec_module(prototype)
identity=TargetTrackIdentity(0,'diagnostic-gen28-repositioned-initial-only')
# The historical helper embeds g11-t12. Override only its diagnostic identity
# constructor; the actual qualifier validator/model retain the real class.
prototype.TargetTrackIdentity=NS(from_stream=lambda *_:identity)
capture=json.loads((I/'result.json').read_text());assert capture['usable_detected_target_triples'] is True
frame=capture['frames'][-1];npz=Path(frame['npz']);assert sha(npz)==frame['sha256']
g=json.loads((R/'measured_geometry.json').read_text());gp=O/'same_fused_geometry_compatibility.json'
save(gp.name,dict(obb_center_base_m=g['center_base'],R_base_obb=g['axes_base'],obb_size_xyz_m=g['size_xyz_m'],
 support_normal_base=g['support_normal_base'],support_offset_m=g['support_offset_m'],support_inlier_ratio=g['support_inlier_ratio'],object_points_base_m=g['object_points_base'],source_mode=g['source_mode'],
 provenance=dict(path=str(R/'measured_geometry.json'),sha256=sha(R/'measured_geometry.json'),mapping_only_no_geometry_changes=True)))
dependency=[prototype_path,W/'.runtime/prototype_gen28_repositioned_rgb_direction.py',Path(prototype.box.__file__),
 W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/gripper_geometry.py',npz,I/'result.json',I/'parameters.json',R/'result.json',R/'measured_geometry.json',gp]
before={str(p):sha(p) for p in dependency}
report=prototype.qualify_source(npz,gp,'repositioned_live_capture_raw_triad_with_same_fused_measured_geometry',[
 'Offline prototype only: actual RGB Hough directions override axes in memory before the original qualifier calculations; all original qualification/uncertainty gates remain.',
 'Diagnostic identity only. No new production source, model, plan or motion authority is created.',
 'Geometry is unchanged production-fuser measured geometry from the same three captured samples; only compatibility field names are mapped.'])
report['target_identity']=asdict(identity)
report['identity_override']='Historical helper constructor overridden with explicitly diagnostic TargetTrackIdentity; model source checked below.'
cfg=json.loads((I/'parameters.json').read_text())['before']['parameters']['/grasp_6d/remote']['gripper_geometry']
gripper=GripperGeometry(max_inner_gap_m=cfg['max_inner_gap_m'],jaw_clearance_each_side_m=cfg['width_safety_margin_per_side_m'],
 finger_size_xyz_m=np.array(cfg['finger_box_xyz_m']),palm_size_xyz_m=np.array(cfg['palm_box_xyz_m']),support_clearance_m=cfg.get('support_clearance_m',.003))
fit=float(cfg['opening_fit_clearance_each_side_m']);physical=min(.05,gripper.max_inner_gap_m)
for row in report['candidates']:
 if not row['ok']:continue
 model=RegularBoxInference.from_evidence_dict(row['model']);assert model.source['target_identity']==asdict(identity)
 budgets=[]
 for axis in (0,1):
  evidence=evaluate_inferred_box_contact(model,model.center_base,model.axes_base[:,axis],-model.support_normal_base,gripper,
   support_normal_base=model.support_normal_base,opening_fit_clearance_each_side_m=fit)
  required=evidence.inferred_width_m+evidence.width_uncertainty_m+2*fit
  reserve=max(gripper.jaw_clearance_each_side_m,evidence.width_uncertainty_m)
  budgets.append(dict(axis=axis,evidence_ok=evidence.ok,evidence_code=evidence.code,inferred_width_m=evidence.inferred_width_m,width_uncertainty_m=evidence.width_uncertainty_m,
   opening_fit_clearance_each_side_m=fit,required_open_width_m=required,physical_opening_m=physical,aperture_margin_m=physical-required,
   required_precision_reserve_m=reserve,aperture_and_precision_pass=evidence.ok and required<=physical+1e-12 and physical-required>=reserve,
   targeted_view=dict(evidence.targeted_view)))
 row['actual_contact_contract_opening_budgets']=budgets
report.update(no_ros_calls=True,no_hardware_access=True,no_temperature_reads=True,physical_grasp_verified=False,production_files_edited=False,production_thresholds_and_uncertainty_unchanged=True,
 arbitrary_angle_scan=False,maximum_actual_rgb_directions=12,source_and_input_sha256=before,source_and_input_unchanged=before=={str(p):sha(p) for p in dependency},
 opening_configuration=dict(live_parameters_path=str(I/'parameters.json'),sha256=sha(I/'parameters.json'),gripper_geometry=cfg),
 prototype_limit='Direction prototype is not deployed. Even a qualified model and passing opening do not certify materialized trajectory collision/IK feasibility.')
assert report['source_and_input_unchanged']
receipt=save('result.json',report);print(json.dumps(receipt,ensure_ascii=False),flush=True)
for row in report['candidates']:
 print(json.dumps(dict(index=row['candidate_index'],angle_deg=row['relative_angle_deg'],visual=row['visual_rectangle_found'],ok=row['ok'],code=row['code'],reason=row['audit'].get('reason'),opening=row.get('actual_contact_contract_opening_budgets')),ensure_ascii=False),flush=True)
