"""Frozen comparison, using the real before/after implementations and live budget."""
from pathlib import Path
from copy import deepcopy
import hashlib, json, os, time, types
W=Path(__file__).resolve().parents[1]
R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1/actual_gen63_baseline_validation_20261023T041149Z/high_accuracy_grasp_r1/arrival_margin_repair_r1')
V=R/'view_search_repair_r1'
assert Path('/media/zhuyupei/SanStand').is_mount()
OUT=V/'timed_replay';OUT.mkdir()
os.environ['OFFLINE_BOUNDARY_LOG']=str(OUT/'boundary.jsonl')
exec((R/'guard.py').read_bytes())
import numpy as np
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
import alicia_grasp_modes.reachable_observation_views as repaired
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def plain(x):
    if isinstance(x,np.ndarray):return x.tolist()
    if isinstance(x,np.generic):return x.item()
    if isinstance(x,dict):return {k:plain(v) for k,v in x.items()}
    if isinstance(x,(list,tuple)):return [plain(v) for v in x]
    return x
fixtures=W/'src/alicia_grasp_modes/tests/fixtures'
metadata=json.loads((fixtures/'supplement_view_seed_20261023.json').read_text())
assert sha(fixtures/'supplement_view_seed_20261023.npz')==metadata['arrays_sha256']
args=deepcopy(metadata['arguments'])
source={k:deepcopy(v) for k,v in metadata.items() if k not in ('arguments','arrays_sha256','model_relative_path','robot_description_sha256','provenance','arrival_projection_audit')}
with np.load(fixtures/'supplement_view_seed_20261023.npz') as a:
    source.update({k:a[k].copy() for k in ('points_base','corners_base','tool_from_camera')})
    source['arrival_projection']={k:a[k].copy() for k in ('prism_base','rgb_valid','invalid_integral')}
source['arrival_projection']['audit']=deepcopy(metadata['arrival_projection_audit'])
fk=SerialUrdfFk((W/metadata['model_relative_path']).read_text(),['Joint%d'%i for i in range(1,7)])
assert fk.model_sha256==metadata['robot_description_sha256']
args.update(source=source,fk=fk,gripper=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003))
assert args['max_seconds']==2.
old=types.ModuleType('alicia_grasp_modes.before_view_seed')
old.__package__='alicia_grasp_modes'
old.__file__=str(V/'reachable_observation_views.before.py')
assert sha(old.__file__)=='7736b592f243ad709afe8ccd2f667c35a5c93c14037f5218769b51c61d631c2e'
exec(compile(Path(old.__file__).read_bytes(),old.__file__,'exec'),old.__dict__)
result=dict(scope='identical frozen failed scene; original two-second budget and endpoint checks; no path or physical grasp claim',passed=False,host_load=os.getloadavg(),runs={})
for label,mod in [('before',old),('after',repaired)]:
    rows,report=mod.generate_reachable_observation_views(**deepcopy(args))
    result['runs'][label]=dict(report=report,candidates=rows,source_sha256=sha(mod.__file__))
result['passed']=(result['runs']['after']['report']['status']=='complete' and len(result['runs']['after']['candidates'])>len(result['runs']['before']['candidates']) and max(r['endpoint_checks']['missing_side_view_gain']['predicted_new_side_gain_m'] for r in result['runs']['after']['candidates'])>.006 and not (OUT/'boundary.jsonl').exists())
result['source_sha256']={'src/alicia_grasp_modes/src/alicia_grasp_modes/reachable_observation_views.py':sha(repaired.__file__)}
(OUT/'result.json').write_text(json.dumps(plain(result),ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(passed=result['passed'],runs={k:dict(elapsed=v['report']['elapsed_sec'],status=v['report']['status'],gains_m=[r['endpoint_checks']['missing_side_view_gain']['predicted_new_side_gain_m'] for r in v['candidates']]) for k,v in result['runs'].items()})))
assert result['passed']
