#!/usr/bin/env python3
"""Read-only replay of recorded requested poses, not recorded MoveIt curves."""
import argparse, json, hashlib
from pathlib import Path
import numpy as np
from tf.transformations import quaternion_matrix
from alicia_flexible_grasp.robot.contact_sequence_precheck import contact_precheck_gripper
from alicia_flexible_grasp.grasp.gripper_geometry import _stage_boxes,_box_corners,_obb_overlap
from alicia_flexible_grasp.grasp.contact_height_margin import evaluate_contact_height_margin
from alicia_flexible_grasp.grasp.contact_height_placement import contact_height_window_proposals

P=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/initial_preflight_deployment/consumers.log')

def run():
 rows=[]
 for line in P.read_text().splitlines():
  if 'remote 6D rejected contact evidence: ' not in line:continue
  row=json.loads(line.split('remote 6D rejected contact evidence: ',1)[1])
  if row.get('request',{}).get('request_id')==41 and row.get('target_track_id')=='g5-t16':rows.append(row)
 gripper=contact_precheck_gripper({});results=[]
 for row in rows:
  obj=row['object_geometry'];e=json.loads(row['contact_geometry_evidence']);opening=row['precheck'].get('opening_m',.04975)
  T=[]
  for pose in row['poses_xyz_xyzw']:
   transform=quaternion_matrix(pose[3:]);transform[:3,3]=pose[:3];T.append(transform)
  G=quaternion_matrix(obj['pose_xyz_xyzw'][3:]);G[:3,3]=obj['pose_xyz_xyzw'][:3];n=np.array(obj['support_normal_base']);size=np.array(obj['size_xyz_m'])
  def check(transform):
   boxes=[]
   for name,c,s in _stage_boxes(transform,gripper,opening,'y','z'):
    corners=_box_corners(c,transform[:3,:3],s)
    boxes.append(dict(body=name,overlap=bool(_obb_overlap(c,transform[:3,:3],s,G[:3,3],G[:3,:3],size)),support_clearance_m=float(np.min(corners@n+obj['support_offset_m']))))
   return boxes
  geometry=dict(gripper=gripper,actual_T_base_tool0=T[2],planned_T_base_tool0=T[2],
   contact_center_base=e['contact_center_base'],contact_height_axis_base=e['contact_height_axis_base'],contact_height_bounds_m=e['contact_height_bounds_m'],
   minimum_contact_overlap_m=e['minimum_contact_overlap_m'],support_normal_base=n,support_offset_m=obj['support_offset_m'],opening_width_m=opening)
  limit=.002676768
  margin=evaluate_contact_height_margin(translation_evidence_limit_m=limit,**geometry)
  options=contact_height_window_proposals(translation_evidence_limit_m=limit,**geometry)
  exact_options=[]
  for transform,audit in options:
   exact_options.append(dict(delta_m=audit['normal_translation_m'],bodies=check(transform),margin=audit['after']))
  sample=[]
  for delta in np.linspace(-.015,.030,181):
   t=T[2].copy();t[:3,3]+=delta*n;boxes=check(t)
   m=evaluate_contact_height_margin(translation_evidence_limit_m=limit,**dict(geometry,actual_T_base_tool0=t,planned_T_base_tool0=t))
   if all(not b['overlap'] and b['support_clearance_m']>=.003-1e-9 for b in boxes):
    sample.append(dict(delta_m=float(delta),contact_margin_ok=m['ok'],reason=m['reason']))
  projected=sum(abs(T[2][:3,1]@G[:3,i])*size[i] for i in range(3))
  results.append(dict(plan_id=row['plan_id'],recorded_failure=row['precheck'].get('reason'),opening_m=opening,geometry=obj,
   contact=e,poses=row['poses_xyz_xyzw'],nominal_stage_boxes=[check(t) for t in T],height_margin=margin,
   original_height_proposals=exact_options,normal_shift_free_samples=sample,obb_projection_on_jaw_m=float(projected),source_record=row))
 return dict(schema=1,input_log=str(P),results=results,physical_actions=0,motion_authorized=False,
  limitations=['Requested final poses only; recorded failing MoveIt curve was not persisted.',
  'Nominal endpoint checks use unchanged physical model and frozen OBB; free endpoint does not authorize a path.',
  'Height evidence limit 0.002676768 m is rounded to nine decimal places from current log; no threshold is reduced.'])
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();d=run();a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(d,indent=2)+'\n')
 for row in d['results']:
  print(json.dumps({k:row[k] for k in ('plan_id','recorded_failure','opening_m','obb_projection_on_jaw_m','height_margin','original_height_proposals','normal_shift_free_samples')}))
