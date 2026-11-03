"""Bounded counterfactual proposal optimization only; no ROS/path/identity authority."""
from pathlib import Path
import inspect,json,time,hashlib,copy,math,sys
import numpy as np
import alicia_grasp_modes.reachable_observation_views as original
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry

N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
F=N/'verification/gen14_arrival_independent/view_source'
def plain(v):
 if isinstance(v,np.ndarray):return v.tolist()
 if isinstance(v,np.generic):return v.item()
 if isinstance(v,dict):return {k:plain(x)for k,x in v.items()}
 if isinstance(v,(tuple,list)):return [plain(x)for x in v]
 return v

def main():
 out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=False)
 d=json.load(open(F/'source.json'));a=np.load(F/'source.npz');source=copy.deepcopy(d['source'])
 for key in ('points_base','corners_base','center_base','base_from_tool','tool_from_camera'):source[key]=a[key].copy()
 source['arrival_projection']={key:a[key].copy()for key in ('prism_base','rgb_valid','invalid_integral')};source['arrival_projection']['audit']=d['arrival_audit']
 fk=SerialUrdfFk((F/'model.urdf').read_text(),['Joint%d'%i for i in range(1,7)])
 args=dict(d['arguments'],source=source,fk=fk,gripper=GripperGeometry(**d['gripper']),max_iterations=96)
 initial=fk(args['current_joint_positions'])@source['tool_from_camera'];center=np.asarray(source['center_base']);baseline=(initial[:3,3]-center);baseline/=np.linalg.norm(baseline)
 normal=np.array(source['geometry']['support_normal_base']); src=inspect.getsource(original.generate_reachable_observation_views)
 prod=Path(original.__file__);before=hashlib.sha256(prod.read_bytes()).hexdigest();records=[]
 def ray_cost(cam):
  unit=cam[:3,3]-center;unit/=np.linalg.norm(unit);angle=math.acos(float(np.clip(unit@baseline,-1,1)));return angle*angle
 def describe(endpoint):
  q,tool,cam,checks,accepted=endpoint
  return dict(q=q,T_base_tool0=tool,T_base_camera_link=cam,endpoint_checks=checks,accepted=accepted,
    ray_change_deg=math.degrees(math.sqrt(ray_cost(cam))),optical_incidence_deg=math.degrees(math.acos(float(np.clip(cam[:3,0]@(-normal),-1,1)))),
    original_objective=float(np.sum((q-args['current_joint_positions'])**2))+original._arrival_center_preference(source['arrival_projection']['prism_base'],cam,np.array([source['camera'][k]for k in ('fx','fy','cx','cy')]),(source['camera']['width'],source['camera']['height']))['cost'])
 for kind in ('continue_original_objective','continue_original_plus_ray_angle_squared'):
  admitted=[];start=time.monotonic()
  def capture(endpoint,tier,gap_index,sign):
   value=describe(copy.deepcopy(endpoint));value.update(completed_elapsed_sec=time.monotonic()-start,tier=tier,gap_index=gap_index,side_sign=sign);admitted.append(value)
  code=src.replace('raise _EndpointQualified(endpoint)','_capture(endpoint, tier, gap_index, sign)')
  # Complete endpoint already performs remaining() after all original gates.
  code=code.replace('q, tool, cam, checks, accepted = endpoint\n                if not accepted:', 'q, tool, cam, checks, accepted = endpoint\n                if accepted:\n                    _capture(endpoint, tier, gap_index, sign)\n                if not accepted:')
  if kind.endswith('ray_angle_squared'):
   code=code.replace("prism, cam, k, (width, height))['cost']\n\n                try:","prism, cam, k, (width, height))['cost'] + _ray_cost(cam)\n\n                try:")
   if '+ _ray_cost(cam)'not in code:raise RuntimeError('objective transformation missing')
  namespace=dict(original.__dict__,_capture=capture,_ray_cost=ray_cost);exec(code,namespace)
  candidates,report=namespace['generate_reachable_observation_views'](**copy.deepcopy(args));elapsed=time.monotonic()-start
  for row in admitted:row['diagnostic_objective']=row['original_objective']+(ray_cost(np.array(row['T_base_camera_link']))if kind.endswith('ray_angle_squared') else 0.)
  groups={}
  for row in admitted:
   key=(row['tier'],row['gap_index'],row['side_sign']);old=groups.get(key)
   if old is None or row['diagnostic_objective']<old['diagnostic_objective']:groups[key]=row
  records.append(dict(kind=kind,actual_elapsed_sec=elapsed,original_function_report=report,original_final_return_count=len(candidates),complete_endpoint_receipts=admitted,retained_diagnostic_incumbents=list(groups.values()),execution_authority=False,whole_path_checked=False,arrival_identity_claimed=False,max_seconds=2.,max_iterations=96,maximum_early_endpoint_checks_per_direction_tier=4))
  print(kind,'elapsed',elapsed,'status',report['status'],'receipts',len(admitted),'incumbents',[(round(x['ray_change_deg'],3),round(x['optical_incidence_deg'],3),x['completed_elapsed_sec'])for x in groups.values()],flush=True)
 result=dict(scope='Two fixed counterfactual proposal objectives, same original 2-second/96-iteration/max4-early-audit constraints; no original acceptance gates changed. No new endpoint has measured appearance or full-path proof.',records=records,production_sha256_before=before,production_sha256_after=hashlib.sha256(prod.read_bytes()).hexdigest(),fixture_sha256=hashlib.sha256((F/'source.json').read_bytes()).hexdigest(),script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),no_ros_master=True,motion_commands_sent=0)
 (out/'result.json').write_text(json.dumps(plain(result),indent=2)+'\n')
if __name__=='__main__':main()
