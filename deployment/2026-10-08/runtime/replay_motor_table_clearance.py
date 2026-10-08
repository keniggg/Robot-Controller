#!/usr/bin/env python3
"""Read closed centering session; replay same continuous CAD box, no ROS calls."""
import argparse,json,sys,math,hashlib
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;ROOT=HERE.parent
sys.path.insert(0,str(HERE));sys.path.insert(0,str(ROOT/'src/alicia_flexible_grasp_supervisor/src'))
import bounded_motor_centering as M
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry,_stage_boxes,_box_corners

def main():
 p=argparse.ArgumentParser();p.add_argument('session',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 plan=json.loads((a.session/'plan.json').read_text());geometry=json.loads((a.session/'center_J1_geometry.json').read_text())
 original=np.array(plan['initial_measured_rad']);reference=np.array(plan['starting_sdk_rad']);measured=np.array(geometry['accepted_positions_rad'])
 increments=np.zeros(3,dtype=int)
 for c in plan['commands']:
  j=c['joint']-1;increments[j]=M.words(c['target_rad'])[j]-M.words(reference)[j]
 contract=M.CenteringContract(original,reference,plan['frozen_epoch_ns'],increments)
 target=np.array(plan['commands'][0]['target_rad']);index=plan['commands'][0]['joint']-1
 lo,hi,_=contract.rectangle(measured,target,reference,index)
 endpoints=np.vstack((measured[:6],reference[:6],target[:6]));bl,bh=endpoints.min(0),endpoints.max(0)
 fk=SerialUrdfFk((a.session/'robot_description.urdf').read_text(),['Joint%d'%i for i in range(1,7)])
 gr=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)
 corners=[];radii=[];counts=[]
 for lower,upper in ((lo,hi),(bl,bh)):
  count=0
  for center,half in M.joint_cells(lower,upper):
   count+=1
   for opening in sorted(set(contract.gripper_bounds)):
    local=np.concatenate([_box_corners(c,np.eye(3),size) for _,c,size in _stage_boxes(np.eye(4),gr,opening,'y','z')])
    radius=np.array([np.dot(fk.corner_motion_radii(p),half) for p in local]);T=fk(center)
    world=local@T[:3,:3].T+T[:3,3];corners.append(world);radii.append(radius)
  counts.append(count)
 corners=np.concatenate(corners);radii=np.concatenate(radii)
 reports=[]
 for name in ('motor_000','motor_001','motor_002'):
  r=json.loads((a.session/(name+'.json')).read_text());arr=np.load(a.session/(name+'.npz'))
  p=np.array(r['table_plane_camera']);top=float(np.max(arr['points_camera'][arr['motor_candidate']]@p[:3]+p[3]))
  x=M.table_clearance_from_cad_enclosure(corners,radii,r['table_plane_base'],top)
  x.update(frame=name,source_stamp_ns=r['source_stamp_ns']);reports.append(x)
 first,last=reports[1:];old_base_radius=float(np.max(np.linalg.norm(corners,axis=1)+radii))
 before=np.array(first['table_plane_base']);after=np.array(last['table_plane_base'])
 old_normal=float(np.linalg.norm(after[:3]-before[:3]))*old_base_radius
 old_offset=abs(float(after[3]-before[3]));old_top=max(0,last['visible_motor_top_m']-first['visible_motor_top_m'])
 old_after=first['minimum_adjacent_table_plus_visible_motor_clearance_m']-old_normal-old_offset-old_top
 report=dict(session=str(a.session),diagnostic_only=True,motion_authorized=False,
  physical_cells=counts[0],nominal_sdk_bridge_cells=counts[1],cad_corner_enclosures=len(corners),
  original_feedback_box=dict(lower_rad=lo.tolist(),upper_rad=hi.tolist()),nominal_sdk_bridge_box=dict(lower_rad=bl.tolist(),upper_rad=bh.tolist()),
  old_bound=dict(maximum_base_distance_m=old_base_radius,normal_debit_m=old_normal,offset_debit_m=old_offset,visible_top_debit_m=old_top,after_debit_m=old_after),
  fresh_direct_reports=reports,implementation_sha256=hashlib.sha256((HERE/'bounded_motor_centering.py').read_bytes()).hexdigest())
 if M.MOUNT not in a.output.resolve().parents:raise RuntimeError('external archive output only')
 a.output.mkdir(parents=True,exist_ok=True);(a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n');np.savez_compressed(a.output/'cad_enclosures.npz',corners_base=corners,motion_radius_m=radii)
 print(json.dumps(report,indent=2))
if __name__=='__main__':main()
