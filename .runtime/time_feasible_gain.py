"""Frozen same-budget proposal experiment; does not alter running source."""
from pathlib import Path
from copy import deepcopy
import hashlib,json,os,types
W=Path(__file__).resolve().parents[1]
R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1/actual_gen63_baseline_validation_20261023T041149Z/high_accuracy_grasp_r1/arrival_margin_repair_r1')
OUT=R/'view_boundary_repair_r1/timed_replay';assert Path('/media/zhuyupei/SanStand').is_mount();OUT.mkdir(parents=True)
os.environ['OFFLINE_BOUNDARY_LOG']=str(OUT/'boundary.jsonl');exec((R/'guard.py').read_bytes())
import numpy as np
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
import alicia_grasp_modes.reachable_observation_views as original
before_path=R/'view_boundary_repair_r1/reachable_observation_views.before.py'
assert hashlib.sha256(before_path.read_bytes()).hexdigest()=='40c08e231582046b59c271835874adb211668affc6fb381d813e7c50a06abef4'
proposal=original
original=types.ModuleType('alicia_grasp_modes.before_feasible_gain');original.__package__='alicia_grasp_modes';original.__file__=str(before_path)
exec(compile(before_path.read_bytes(),str(before_path),'exec'),original.__dict__)
def plain(x):
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 if isinstance(x,dict):return {k:plain(v) for k,v in x.items()}
 if isinstance(x,(tuple,list)):return [plain(v) for v in x]
 return x
result=dict(scope='frozen endpoint proposals only; no path, arrival or physical grasp proof',runs={})
for scene in ['supplemental_view_r6','supplemental_view_r3']:
 d=R/scene/'collision_diagnosis';metadata=json.loads((d/'view_search_source.json').read_text())
 args=deepcopy(metadata['arguments']);source={k:deepcopy(v) for k,v in metadata.items() if k not in ('arguments','arrival_projection_audit','robot_description_sha256')}
 with np.load(d/'view_search_source.npz') as a:
  source.update({k:a[k].copy() for k in ('points_base','corners_base','tool_from_camera')})
  source['arrival_projection']={k:a[k].copy() for k in ('prism_base','rgb_valid','invalid_integral')}
 source['arrival_projection']['audit']=deepcopy(metadata['arrival_projection_audit'])
 saved=json.loads((d/'camera_and_model.json').read_text())['values'];params=json.loads((d/'readonly_parameters.json').read_text())['values'];gg=params['/grasp_6d']['remote']['gripper_geometry']
 fk=SerialUrdfFk(saved['/robot_description'],['Joint%d'%i for i in range(1,7)]);assert fk.model_sha256==metadata['robot_description_sha256']
 args.update(source=source,fk=fk,gripper=GripperGeometry(gg['max_inner_gap_m'],gg['width_safety_margin_per_side_m'],np.array(gg['finger_box_xyz_m']),np.array(gg['palm_box_xyz_m']),gg.get('support_clearance_m',.003)))
 for label,mod in [('before',original),('proposal',proposal)]:
  rows,report=mod.generate_reachable_observation_views(**deepcopy(args));key=scene+'_'+label
  result['runs'][key]=dict(report=report,candidates=rows)
  print(json.dumps(dict(scene=scene,label=label,elapsed=report['elapsed_sec'],status=report['status'],gains_mm=[1000*x['endpoint_checks']['missing_side_view_gain']['predicted_new_side_gain_m'] for x in rows])),flush=True)
result['source_sha256']={'src/alicia_grasp_modes/src/alicia_grasp_modes/reachable_observation_views.py':hashlib.sha256(Path(proposal.__file__).read_bytes()).hexdigest()}
result['passed']=all(v['report']['status']=='complete' and v['report']['elapsed_sec']<=2. and len(v['candidates'])<=4 for v in result['runs'].values()) and max(x['endpoint_checks']['missing_side_view_gain']['predicted_new_side_gain_m'] for x in result['runs']['supplemental_view_r6_proposal']['candidates'])>.005 and not (OUT/'boundary.jsonl').exists()
(OUT/'result.json').write_text(json.dumps(plain(result),ensure_ascii=False,indent=2)+'\n')
assert result['passed']
