"""GEN58 exact source under unchanged legacy .002-per-inequality test clock."""
import os
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
from pathlib import Path
import sys,json,hashlib,socket,time
W=Path(__file__).resolve().parents[1];C=W/'.runtime/gen59_observation_pose_candidate_r2';N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1');I=N/'observation_pose_fix/offline_selection_r1/source_attempt3';O=N/'observation_pose_fix/full_r2_failure_review/gen59_workclock';assert Path('/media/zhuyupei/SanStand').is_mount();O.mkdir(exist_ok=False)
sys.path[:0]=[str(C/'src/alicia_grasp_modes/src'),str(C/'src/alicia_flexible_grasp_supervisor/src')];blocked=[]
class NoSocket(socket.socket):
 def __new__(cls,*a,**k):blocked.append('socket');raise RuntimeError('offline socket denied')
socket.socket=NoSocket
import numpy as np
from scipy.optimize import minimize
from alicia_grasp_modes import reachable_observation_views as module
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();m=json.loads((I/'source.json').read_text());z=np.load(I/'source.npz');source=m['source'];source.update({k:z[k].copy() for k in ['points_base','corners_base','center_base','tool_from_camera','base_from_tool']});source['arrival_projection']=dict(audit=m['arrival_audit'],**{k:z[k].copy() for k in ['prism_base','rgb_valid','invalid_integral']})
args=dict(source=source,current_joint_positions=m['current_joint_positions'],fk=SerialUrdfFk(Path(m['model_path']).read_text(),['Joint%d'%i for i in range(1,7)]),gripper=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003),opening_width_m=.05,gaps=m['gaps'],depth_uncertainty_m=.003,side_depth_uncertainty_m=.003,joint_error_bounds_rad=m['joint_error_bounds_rad'])
e=dict(now=0.,calls=0,solver_entries=0,first_qualified=None);start=time.monotonic();original=module._backend

def backend(fun,q,**kw):
 e['solver_entries']+=1;f=kw['constraints'][0]['fun']
 def measured(q):
  e['calls']+=1;e['now']+=.002
  try:return f(q)
  except module._EndpointQualified as x:
   if e['first_qualified'] is None:e['first_qualified']=dict(calls=e['calls'],clock=e['now'],q=x.endpoint[0].tolist(),actual_wall_sec=time.monotonic()-start,endpoint_checks=x.endpoint[3])
   raise
 kw['constraints']=[dict(type='ineq',fun=measured)];return minimize(fun,q,**kw)
module._backend=lambda:backend
rows,audit=module.generate_reachable_observation_views(**args,clock=lambda:e['now']);module._backend=original
r=dict(input_npz_sha256=sha(I/'source.npz'),helper_sha256=sha(Path(__file__)),test_clock_step_sec=.002,clock_budget_sec=2.,wall_sec=time.monotonic()-start,counters=e,audit=audit,candidates=rows,socket_attempts=blocked,execution_authority=False)
def plain(x):
 if isinstance(x,dict):return {str(k):plain(v) for k,v in x.items()}
 if isinstance(x,(list,tuple)):return [plain(v) for v in x]
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 return x
(O/'result.json').write_text(json.dumps(plain(r),indent=2)+'\n');(O/'helper_source.py').write_bytes(Path(__file__).read_bytes());print(json.dumps(dict(status=audit['status'],rows=len(rows),calls=e['calls'],first_qualified_calls=None if e['first_qualified'] is None else e['first_qualified']['calls'],first_qualified_q=None if e['first_qualified'] is None else e['first_qualified']['q'])))
