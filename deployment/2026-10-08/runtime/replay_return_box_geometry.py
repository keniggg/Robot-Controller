"""Offline full return cell proof; no node, publishers, services or MoveIt calls."""
import json,hashlib,math,os,time
from pathlib import Path
import numpy as np
import bounded_joint_response_diagnostic as base
import bounded_joint23_fresh_return as ret
import bootstrap_mixed_geometry as geo
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')
SOURCE=ROOT/'bounded_joint23_fresh_return_20260928T133453Z_239528'
OUT=ROOT/'grasp_repair_resume_20260928T120606Z/explicit_joint_box_return_replay'

def main():
 if not os.path.ismount('/media/zhuyupei/SanStand'):raise RuntimeError('external disk missing')
 g=json.loads((SOURCE/'fresh_return_baseline_geometry.json').read_text())
 parent=ret.load_parent(ROOT/'bounded_joint23_counterrotation_20260928T131913Z_197737')
 fk=SerialUrdfFk((SOURCE/'robot_description.urdf').read_text(),base.NAMES[:6])
 gr=base.GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)
 q=np.array(g['accepted_positions_rad']);reference=parent['expected_current_sdk'];target=parent['original_sdk']
 geometry=dict(X=np.array(g['T_tool_camera']),Y=np.array(g['T_base_board']),model=base.CAMERA.RuntimeColorModel(g['quality']['camera_model']))
 lo,hi,directions=ret.cr.segment_rectangle(q,target,reference,parent['origin'])
 endpoints=np.vstack((q[:6],reference[:6],target[:6]));blocks=[('physical',lo,hi),('nominal',endpoints.min(0),endpoints.max(0))]
 openings=sorted(set([parent['origin'][6],parent['original_sdk'][6]]));report=[];lowers=[];uppers=[];start=time.monotonic()
 for label,lower,upper in blocks:
  axes=ret.cr.rectangle_grid(lower,upper);proofs=[];failures=[];old_failures=[];worst=None
  for indices in __import__('itertools').product(*(range(len(a)-1) for a in axes)):
   a=np.array([axis[i] for axis,i in zip(axes,indices)]);b=np.array([axis[i+1] for axis,i in zip(axes,indices)]);center=.5*(a+b);half=.5*(b-a)
   for opening in openings:
    try:
     proof=geo.certify_joint_box_geometry(fk,gr,np.r_[center,opening],geometry,fk.limits[:,0],fk.limits[:,1],half)
    except RuntimeError as e:
     failures.append(dict(indices=list(indices),lower_rad=a.tolist(),upper_rad=b.tolist(),reason=str(e)));continue
    proofs.append(proof);t=proof['camera_coordinate_taylor_bound'];lowers.append(t['corner_lower_xyz_m']);uppers.append(t['corner_upper_xyz_m'])
    if worst is None or proof['minimum_full_board_edge_lower_bound_px']<worst['minimum_full_board_edge_lower_bound_px']:
     worst=dict(indices=list(indices),lower_rad=a.tolist(),upper_rad=b.tolist(),**{k:proof[k] for k in ['minimum_full_board_edge_lower_bound_px','minimum_plane_clearance_lower_bound_m','minimum_board_depth_lower_bound_m']})
  report.append(dict(block=label,lower_rad=lower.tolist(),upper_rad=upper.tolist(),number_cells=int(np.prod([len(a)-1 for a in axes])),proofs=len(proofs),failures=failures,worst=worst,
   minimum_plane_m=min([p['minimum_plane_clearance_lower_bound_m'] for p in proofs],default=None),minimum_board_depth_m=min([p['minimum_board_depth_lower_bound_m'] for p in proofs],default=None)))
 batch=geo.projection_corner_box_batch(geometry['model'],np.asarray(lowers),np.asarray(uppers));ci=geometry['model'].evidence['color_intrinsics']
 edge=float(min(batch[0,0],ci['width']-1-batch[0,1],batch[1,0],ci['height']-1-batch[1,1]))
 # Re-evaluate the archived cell that originally required repeated subdivision.
 prior=ROOT/'grasp_repair_resume_20260928T120606Z/counterrotation_return_geometry_diagnosis.json'
 bad=json.loads(prior.read_text());a=np.array(bad['lower_rad']);b=np.array(bad['upper_rad']);center=.5*(a+b);half=.5*(b-a)
 compare={}
 for kind,fn in [('old_L1',lambda:geo.certify_local_cell_geometry(fk,gr,np.r_[center,q[6]],geometry,fk.limits[:,0],fk.limits[:,1],math.degrees(sum(half)))),('explicit_box',lambda:geo.certify_joint_box_geometry(fk,gr,np.r_[center,q[6]],geometry,fk.limits[:,0],fk.limits[:,1],half))]:
  try:
   p=fn();compare[kind]={k:p[k] for k in ['minimum_plane_clearance_lower_bound_m','minimum_full_board_edge_lower_bound_px','minimum_board_depth_lower_bound_m']}
  except RuntimeError as e:compare[kind]=dict(failure=str(e))
 result=dict(source_geometry=str(SOURCE/'fresh_return_baseline_geometry.json'),source_sha256=hashlib.sha256((SOURCE/'fresh_return_baseline_geometry.json').read_bytes()).hexdigest(),
  parent_digest=parent['digest'],source_stamp_ns=g['source_stamp_ns'],old_failed_cell_replayed_with_this_scene=compare,blocks=report,
  entire_box_union_projection_edge_lower_bound_px=edge,entire_box_union_pixel_rectangle_uv=batch.tolist(),seconds=time.monotonic()-start,
  all_requested_original_cells_pass_without_subdivision=not any(p['failures'] for p in report),no_MoveIt_called=True,no_ROS_node=True,no_execution_authority=True)
 OUT.mkdir(exist_ok=True);(OUT/'report.json').write_text(json.dumps(result,indent=2)+'\n');(OUT/'analysis_source.py').write_bytes(Path(__file__).read_bytes());np.savez(OUT/'proved_cell_corner_boxes.npz',lower=lowers,upper=uppers)
 print(json.dumps(result))
if __name__=='__main__':main()
