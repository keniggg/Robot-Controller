"""Offline direct samples of rejected cells and optional view changes; no ROS controls."""
import importlib.util,json,sys,itertools,math
from pathlib import Path
import numpy as np,rosbag
ROOT=Path.cwd();sys.path.insert(0,str(ROOT/'.runtime'));sys.path.insert(0,str(ROOT/'src/alicia_flexible_grasp_supervisor/src'))
import bounded_joint23_counterrotation as cr
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry,_stage_boxes,_box_corners
spec=importlib.util.spec_from_file_location('cm',ROOT/'src/real-arm/alicia_d_calibration/scripts/charuco_tracker.py');cm=importlib.util.module_from_spec(spec);spec.loader.exec_module(cm)
SOURCE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/bounded_joint23_counterrotation_20260928T123521Z_77346')
OUT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/grasp_repair_resume_20260928T120606Z/counterrotation_view_diagnosis.json')
g=json.loads((SOURCE/'baseline_geometry.json').read_text());q=np.array(g['accepted_positions_rad']);x=np.array(g['T_tool_camera']);y=np.array(g['T_base_board']);model=cm.RuntimeColorModel(g['quality']['camera_model']);fk=SerialUrdfFk((SOURCE/'robot_description.urdf').read_text(),['Joint%d'%i for i in range(1,7)])
with rosbag.Bag(str(SOURCE/'evidence.bag')) as bag:
 for _,m,_ in bag.read_messages(topics=['/alicia_d/control_reference']):
  ref=np.array([dict(zip(m.name,m.position))[n] for n in ['Joint%d'%i for i in range(1,7)]+['right_finger']]);break
G=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)
board=np.array([[u,v,0.,1.] for u in np.linspace(0,.18,11) for v in np.linspace(0,.135,9)])
n=y[:3,2].copy()
if n[2]<0:n=-n
offset=-n@y[:3,3]
ci=model.evidence['color_intrinsics']
def sample(qq):
 tool=fk(qq[:6]);camera=(np.linalg.inv(tool@x)@y@board.T).T[:,:3]
 optical=np.c_[-camera[:,1],-camera[:,2],camera[:,0]]
 uv=np.array([model.sdk.rs2_project_point_to_pixel(model.intrinsics,p.tolist()) for p in optical])
 bounds=[float(uv[:,0].min()),float(uv[:,0].max()),float(uv[:,1].min()),float(uv[:,1].max())]
 edge=min(bounds[0],ci['width']-1-bounds[1],bounds[2],ci['height']-1-bounds[3])
 clearance=min(float(np.min(_box_corners(c,tool[:3,:3],size)@n+offset)) for _,c,size in _stage_boxes(tool,G,float(qq[6]),'y','z'))
 return dict(q_rad=qq.tolist(),board_bbox_xmin_xmax_ymin_ymax_px=bounds,board_minimum_edge_px=float(edge),minimum_board_depth_m=float(camera[:,0].min()),minimum_plane_clearance_m=clearance)
def bounds_report(origin,reference):
 c=cr.CounterrotationContract(origin,reference,100,2.);lo,hi,_=cr.segment_rectangle(origin,c.targets[0],reference,origin);axes=cr.rectangle_grid(lo,hi)
 mid,rho=next(cr.rectangle_cells(axes));first_lo=np.array([a[0] for a in axes]);first_hi=np.array([a[1] for a in axes])
 first=[sample(np.r_[v,origin[6]]) for v in itertools.product(*zip(first_lo,first_hi))]
 full=[sample(np.r_[v,origin[6]]) for v in itertools.product(*zip(lo,hi))]
 return dict(first_cell_center=sample(np.r_[mid,origin[6]]),first_cell_radius_deg=math.degrees(rho),
  first_cell_worst_corner=min(first,key=lambda r:r['board_minimum_edge_px']),
  full_physical_rectangle_worst_sample=min(full,key=lambda r:r['board_minimum_edge_px']),
  full_box_board_uv_envelope=[min(r['board_bbox_xmin_xmax_ymin_ymax_px'][0] for r in full),max(r['board_bbox_xmin_xmax_ymin_ymax_px'][1] for r in full),min(r['board_bbox_xmin_xmax_ymin_ymax_px'][2] for r in full),max(r['board_bbox_xmin_xmax_ymin_ymax_px'][3] for r in full)])
result=dict(source=str(SOURCE),execution_authorized=False,source_stamp_ns=g['source_stamp_ns'],actual_q=q.tolist(),sdk_q=ref.tolist(),baseline=sample(q),counterrotation=bounds_report(q,ref),view_candidates=[])
for joint in (4,5):
 for direction in (-1,1):
  step=22*cr.QUANTUM;candidate=q.copy();candidate[joint]+=direction*step;candidate_ref=ref.copy();candidate_ref[joint]+=direction*step
  try:record=bounds_report(candidate,candidate_ref);record.update(joint=joint+1,nominal_actual_change_deg=direction*math.degrees(step),new_view=sample(candidate),current_single_axis_tool_supported=(joint==4))
  except Exception as e:record=dict(joint=joint+1,nominal_actual_change_deg=direction*math.degrees(step),error=str(e))
  result['view_candidates'].append(record)
from bootstrap_mixed_geometry import certify_local_cell_geometry
c=cr.CounterrotationContract(q,ref,100,2.);lo,hi,_=cr.segment_rectangle(q,c.targets[0],ref,q);axes=cr.rectangle_grid(lo,hi)
for index,(mid,radius) in enumerate(cr.rectangle_cells(axes)):
 try:
  proof=certify_local_cell_geometry(fk,G,np.r_[mid,q[6]],dict(X=x,Y=y,model=model),fk.limits[:,0],fk.limits[:,1],math.degrees(radius))
 except Exception as e:
  indices=list(itertools.product(*(range(len(axis)-1) for axis in axes)))[index]
  lower=np.array([axis[j] for axis,j in zip(axes,indices)]);upper=np.array([axis[j+1] for axis,j in zip(axes,indices)])
  corners=[sample(np.r_[v,q[6]]) for v in itertools.product(*zip(lower,upper))]
  result['first_failed_continuous_cell']=dict(index=index,radius_deg=math.degrees(radius),reason=str(e),
    midpoint=sample(np.r_[mid,q[6]]),worst_corner=min(corners,key=lambda r:r['board_minimum_edge_px']))
  break
result['limitations']=['99 points on board per posture and box corner samples are rejection diagnostics, never continuous proof or execution approval.','View candidates assume modeled actual joint change, not guaranteed response to SDK words.','J6 is not available in existing bounded single-axis execution tool.']
OUT.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(report=str(OUT),baseline=result['baseline'],counterrotation=result['counterrotation'],first_failed_continuous_cell=result.get('first_failed_continuous_cell'),candidates=[dict(joint=r['joint'],step_deg=r['nominal_actual_change_deg'],new_view=r.get('new_view'),worst=r.get('full_physical_rectangle_worst_sample'),error=r.get('error')) for r in result['view_candidates']]),ensure_ascii=False))
