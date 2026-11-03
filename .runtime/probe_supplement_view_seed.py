"""Offline numerical-seed comparison on the exact rebuilt frozen source."""
from pathlib import Path
import inspect,json,os,time
W=Path(__file__).resolve().parents[1]
R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1/actual_gen63_baseline_validation_20261023T041149Z/high_accuracy_grasp_r1/arrival_margin_repair_r1')
OUT=R/'supplemental_view_r3/collision_diagnosis'
os.environ['OFFLINE_BOUNDARY_LOG']=str(OUT/'seed_probe_boundary.jsonl');exec((R/'guard.py').read_text())
import numpy as np
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
import alicia_grasp_modes.reachable_observation_views as module
def plain(x):
    if isinstance(x,np.ndarray):return x.tolist()
    if isinstance(x,np.generic):return x.item()
    if isinstance(x,dict):return {k:plain(v) for k,v in x.items()}
    if isinstance(x,(tuple,list)):return [plain(v) for v in x]
    return x
source=json.loads((OUT/'view_search_source.json').read_text());args=source.pop('arguments')
saved=json.loads((OUT/'camera_and_model.json').read_text())['values']
fk=SerialUrdfFk(saved['/robot_description'],['Joint%d'%i for i in range(1,7)])
assert source.pop('robot_description_sha256')==fk.model_sha256
with np.load(OUT/'view_search_source.npz') as a:
    source.update({k:a[k] for k in ('points_base','corners_base','tool_from_camera')})
    source['arrival_projection']={k:a[k] for k in ('prism_base','rgb_valid','invalid_integral')}
source['arrival_projection']['audit']=source.pop('arrival_projection_audit')
gg=json.loads((OUT/'readonly_parameters.json').read_text())['values']['/grasp_6d']['remote']['gripper_geometry']
gripper=GripperGeometry(gg['max_inner_gap_m'],gg['width_safety_margin_per_side_m'],np.array(gg['finger_box_xyz_m']),np.array(gg['palm_box_xyz_m']),gg.get('support_clearance_m',.003))
args.update(source=source,fk=fk,gripper=gripper)
text=inspect.getsource(module.generate_reachable_observation_views)
for old in ('    candidates = []\n','result = minimize(objective, q0.copy(), method=',"                row_audit['accepted'] = True\n"):
    assert text.count(old)==1,old
text=text.replace('    candidates = []\n','    candidates = []\n    warm = {}\n')
text=text.replace('result = minimize(objective, q0.copy(), method=',
    'result = minimize(objective, warm.get((gap_index,sign),q0).copy(), method=')
text=text.replace("                row_audit['accepted'] = True\n", "                row_audit['accepted'] = True\n                warm[(gap_index,sign)] = q.copy()\n")
namespace=dict(module.__dict__);exec(compile(text,'<offline_same_direction_seed>','exec'),namespace)
rows,report=namespace['generate_reachable_observation_views'](**args)
out=dict(scope='counterfactual request-local same-direction qualified endpoint seed; measured start, original objective, budgets and all endpoint checks unchanged',
    report=report,candidates=rows,execution_authorized=False,complete_path_checked=False)
(OUT/'warm_seed_probe.json').write_text(json.dumps(plain(out),ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(candidates=len(rows),elapsed=report['elapsed_sec'],
    accepted=[dict(direction=r['direction_gap_index'],tier=r['exposure_target_noise_multiplier'],actual_gain_m=r['endpoint_checks']['missing_side_view_gain']['predicted_new_side_gain_m']) for r in rows],
    directions=[{k:r[k] for k in ('gap_index','exposure_target_noise_multiplier','accepted','reason') if k in r} for r in report['directions']])),flush=True)
