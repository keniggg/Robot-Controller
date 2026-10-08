"""Closed-record geometry only; imports no ROS node and sends no commands."""
import importlib.util,itertools,json,math,sys,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'.runtime/realsense-2.56.5'),str(ROOT/'.runtime'),str(ROOT/'src/alicia_flexible_grasp_supervisor/src')]
import bounded_joint23_counterrotation as cr
from bootstrap_mixed_geometry import certify_local_cell_geometry,board_camera_taylor_box
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
spec=importlib.util.spec_from_file_location('camera',ROOT/'src/real-arm/alicia_d_calibration/scripts/charuco_tracker.py')
cm=importlib.util.module_from_spec(spec);spec.loader.exec_module(cm)
SOURCE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/bounded_joint23_counterrotation_20260928T131913Z_197737')
OUT=SOURCE.parent/'grasp_repair_resume_20260928T120606Z/counterrotation_return_geometry_diagnosis.json'
p=json.loads((SOURCE/'plan.json').read_text());g=json.loads((SOURCE/'return_after_positive_geometry.json').read_text())
q=np.array(g['accepted_positions_rad']);original=np.array(p['initial_measured_rad']);target=np.array(p['starting_sdk_rad']);sdk=np.array(p['logical_legs'][0]['target_rad'])
fk=SerialUrdfFk((SOURCE/'robot_description.urdf').read_text(),['Joint%d'%i for i in range(1,7)])
G=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)
x=np.array(g['T_tool_camera']);y=np.array(g['T_base_board']);model=cm.RuntimeColorModel(g['quality']['camera_model'])
geometry=dict(X=x,Y=y,model=model);lo,hi,_=cr.segment_rectangle(q,target,sdk,original)
axes=cr.rectangle_grid(lo,hi);ci=model.evidence['color_intrinsics'];board=np.array([[u,v,0.,1.] for u in np.linspace(0,.18,11) for v in np.linspace(0,.135,9)])
def exact(qq):
 camera=(np.linalg.inv(fk(qq)@x)@y@board.T).T[:,:3]
 uv=np.array([model.sdk.rs2_project_point_to_pixel(model.intrinsics,[-v[1],-v[2],v[0]]) for v in camera])
 b=[float(uv[:,0].min()),float(uv[:,0].max()),float(uv[:,1].min()),float(uv[:,1].max())]
 return dict(edge_px=min(b[0],ci['width']-1-b[1],b[2],ci['height']-1-b[3]),bbox_px=b)
def proof(lower,upper):
 return certify_local_cell_geometry(fk,G,np.r_[.5*(lower+upper),q[6]],geometry,fk.limits[:,0],fk.limits[:,1],math.degrees(.5*np.sum(upper-lower)))
started=time.monotonic();result=dict(source=str(SOURCE),execution_authorized=False)
for index,indices in enumerate(itertools.product(*(range(len(a)-1) for a in axes))):
 lower=np.array([a[j] for a,j in zip(axes,indices)]);upper=np.array([a[j+1] for a,j in zip(axes,indices)]);center=.5*(lower+upper)
 try:proof(lower,upper)
 except RuntimeError as e:
  t=board_camera_taylor_box(fk,center,x,y,.5*np.sum(upper-lower))
  result.update(first_failure_index=index,reason=str(e),lower_rad=lower.tolist(),upper_rad=upper.tolist(),center_rad=center.tolist(),center_exact=exact(center),
    corner_coordinate_width_m=(np.asarray(t['corner_upper_xyz_m'])-np.asarray(t['corner_lower_xyz_m'])).tolist(),
    maximum_corner_coordinate_width_m=float(np.max(np.asarray(t['corner_upper_xyz_m'])-np.asarray(t['corner_lower_xyz_m']))))
  corner_samples=[exact(np.array(v)) for v in itertools.product(*zip(lower,upper))]
  result['minimum_actual_cell_corner_sample_px']=min(v['edge_px'] for v in corner_samples)
  if result['center_exact']['edge_px']<20 or result['minimum_actual_cell_corner_sample_px']<20:
   result['subdivision']='not attempted; exact sampled physical posture outside original 20 px'
  else:
   stack=[(lower,upper,0)];checks=0;leaves=[];failures=[]
   while stack and checks<64:
    a,b,depth=stack.pop();checks+=1
    try:leaves.append(proof(a,b)['minimum_full_board_edge_lower_bound_px'])
    except RuntimeError as er:
     if depth>=4:failures.append(str(er));continue
     j=int(np.argmax(b-a));mid=(a[j]+b[j])/2;upper1=b.copy();upper1[j]=mid;lower2=a.copy();lower2[j]=mid
     stack.extend([(a,upper1,depth+1),(lower2,b,depth+1)])
   result['bounded_subdivision']=dict(max_depth=4,max_checks=64,checks=checks,passed_leaves=len(leaves),minimum_leaf_edge_px=min(leaves) if leaves else None,failures=failures,unvisited=len(stack))
  break
result['full_physical_rectangle_corner_samples_minimum_px']=min(exact(np.array(v))['edge_px'] for v in itertools.product(*zip(lo,hi)))
result['seconds']=time.monotonic()-started
OUT.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
