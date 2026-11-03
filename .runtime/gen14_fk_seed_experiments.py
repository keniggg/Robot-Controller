"""GEN14 source-bound FK numerical experiments; no ROS, motion, or source edits."""
from pathlib import Path
from types import SimpleNamespace
import hashlib,json,time,sys
import numpy as np
from scipy.optimize import minimize
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
import alicia_grasp_modes.reachable_observation_views as module
NEW=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_no_skin_20261011T000625Z')
INPUT=NEW/'gen14_analysis/fk_replay_inputs'
def load():
 m=json.loads((INPUT/'source.json').read_text());z=np.load(str(INPUT/'source.npz'))
 source=dict(m['source']);source.update({k:z[k].copy() for k in ('points_base','corners_base','center_base','tool_from_camera')})
 source['arrival_projection']={k:z[k].copy() for k in ('prism_base','rgb_valid','invalid_integral')};source['arrival_projection']['audit']=m['arrival_audit']
 fk=SerialUrdfFk(Path(m['model_path']).read_text(),['Joint%d'%i for i in range(1,7)])
 assert fk.model_sha256==m['model_sha256']
 return dict(source=source,current_joint_positions=m['current_joint_positions'],fk=fk,gripper=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003),opening_width_m=.05,gaps=m['gaps'],depth_uncertainty_m=.003,side_depth_uncertainty_m=.003,joint_error_bounds_rad=m['joint_error_bounds_rad'])
class Feasible(Exception):
 def __init__(self,q):self.q=np.array(q,copy=True)
def experiment(name,delta=None,early=False):
 args=load();events=[];began=time.monotonic();original_backend=module._backend
 def solver(fun,seed,**kw):
  run=dict(seed=seed.tolist(),constraint_calls=0,iterations=0,first_feasible=None,initial_minimum=None,best_minimum=-1e100,iterates=[]);events.append(run)
  constraint=kw['constraints'][0]['fun']
  if delta is not None:seed=np.clip(seed+delta,args['fk'].limits[:,0],args['fk'].limits[:,1]);run['seed']=seed.tolist()
  def checked(q):
   v=constraint(q);run['constraint_calls']+=1;mn=float(np.min(v));elapsed=time.monotonic()-began
   if run['initial_minimum'] is None:run['initial_minimum']=mn
   run['best_minimum']=max(run['best_minimum'],mn)
   if mn>=0. and run['first_feasible'] is None:run['first_feasible']=dict(elapsed_sec=elapsed,q=np.asarray(q).tolist(),min_constraint=mn)
   if early=='probe' and mn>=0.:raise Feasible(q)
   return v
  def callback(q):
   run['iterations']+=1;mn=float(np.min(checked(q)))
   if len(run['iterates'])<100:run['iterates'].append(dict(iteration=run['iterations'],elapsed_sec=time.monotonic()-began,minimum_constraint=mn))
   if early is True and mn>=0.:raise Feasible(q)
  kw['constraints']=[dict(type='ineq',fun=checked)];kw['callback']=callback
  try:return minimize(fun,seed,**kw)
  except Feasible as e:return SimpleNamespace(x=e.q,nit=run['iterations'],success=False,message='offline first feasible iterate; original endpoint gates still required')
 module._backend=lambda:solver
 try:rows,report=module.generate_reachable_observation_views(**args)
 finally:module._backend=original_backend
 result=dict(name=name,opening_width_m=.05,arguments_exact_except_described_proposal_heuristic=True,seed_delta_rad=None if delta is None else np.asarray(delta).tolist(),first_feasible_early_stop=early,events=events,candidates=rows,audit=report,physical_actions=False,full_path_checked=False)
 out=NEW/'gen14_analysis'/('fk_'+name+'.json');out.write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps(dict(name=name,path=str(out),sha256=hashlib.sha256(out.read_bytes()).hexdigest(),candidate_count=len(rows),status=report['status'],first_feasible=[r['first_feasible'] for r in events],iterations=[r['iterations'] for r in events])),flush=True)
 return result
def main():
 assert Path('/media/zhuyupei/SanStand').is_mount()
 mode=sys.argv[1] if len(sys.argv)>1 else 'baseline'
 if mode=='baseline':experiment('exact50_original_profile')
 elif mode=='early':experiment('exact50_first_feasible',early='probe')
 elif mode=='seeds':
  for i,sign in [(1,-1),(1,1),(2,-1),(2,1),(3,-1),(3,1),(4,1),(5,-1),(5,1)]:
   delta=np.zeros(6);delta[i]=sign*.15
   experiment('seed_joint%d_%s'%(i+1,'plus' if sign>0 else 'minus'),delta,True)
if __name__=='__main__':main()
