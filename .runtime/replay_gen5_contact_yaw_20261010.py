#!/usr/bin/env python3
"""Counterfactual yaw from recorded current-view points; no motion authority."""
import json,argparse
from pathlib import Path
import numpy as np
from tf.transformations import quaternion_matrix,rotation_matrix
from alicia_flexible_grasp.robot.contact_sequence_precheck import contact_precheck_gripper
from alicia_flexible_grasp.grasp.gripper_geometry import _stage_boxes,_box_corners,_obb_overlap,evaluate_bilateral_surface_evidence_and_bounds
from alicia_flexible_grasp.grasp.contact_height_placement import place_contact_height
from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface,_voxel_deduplicate
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
REC=BASE/'grasp_records/20261010T020909.515250Z_ec46b035f0d74e2688bb217beb3da64a'
AN=BASE/'ros_log/full_latest_restart_20261010T002347Z/gen5_contact_analysis'

def overlap_interval(c,R,s,oc,oR,os,n):
 axes=[R[:,i] for i in range(3)]+[oR[:,i] for i in range(3)]
 for a in R.T:
  for b in oR.T:
   cross=np.cross(a,b);length=np.linalg.norm(cross)
   if length>1e-9:axes.append(cross/length)
 lo,hi=-float('inf'),float('inf')
 for ax in axes:
  d=float((oc-c)@ax);v=float(n@ax);rad=float((s/2)@abs(R.T@ax)+(os/2)@abs(oR.T@ax))+1e-9
  if abs(v)<1e-14:
   if abs(d)>rad:return None
  else:
   ends=sorted(((d-rad)/v,(d+rad)/v));lo=max(lo,ends[0]);hi=min(hi,ends[1])
 if lo>hi:return None
 return [lo,hi]

def run():
 audit=json.loads((REC/'keyframes_audits/bb75081aa6bcc160e641481385310be3b02152af763da52f964c570d7afcb42b.json').read_text())
 original=json.loads((AN/'request41_endpoint_replay.json').read_text())['results'][1]
 obj=original['geometry'];e=original['contact'];g=contact_precheck_gripper({});opening=original['opening_m'];limit=.0026767676132463574
 G=quaternion_matrix(obj['pose_xyz_xyzw'][3:]);G[:3,3]=obj['pose_xyz_xyzw'][:3];n=np.array(obj['support_normal_base']);sz=np.array(obj['size_xyz_m']);center=np.array(e['contact_center_base'])
 pose=original['poses'][2];T=quaternion_matrix(pose[3:]);T[:3,3]=pose[:3]
 def boxes(t):
  return [dict(body=name,overlap=bool(_obb_overlap(c,t[:3,:3],s,G[:3,3],G[:3,:3],sz)),support_clearance_m=float(np.min(_box_corners(c,t[:3,:3],s)@n+obj['support_offset_m']))) for name,c,s in _stage_boxes(t,g,opening,'y','z')]
 intervals=[dict(body=name,overlap_normal_translation_interval_m=overlap_interval(c,T[:3,:3],s,G[:3,3],G[:3,:3],sz,n)) for name,c,s in _stage_boxes(T,g,opening,'y','z')]
 correction=np.array(audit['multiview_surface']['latest_registration']['snapshot_to_reference_transform'])
 raw=np.array(audit['replay_geometry']['object_points_base_m']);points=raw@correction[:3,:3].T+correction[:3,3]
 reconstructed=BASE/'ros_log/full_latest_restart_20261010T002347Z/gen5_handoff_analysis/contact_height_placement/exact_snapshot_reconstruction/fused_reconstruction.npz'
 source=np.load(reconstructed);points=source['reference_points'];indices=source['view_indices']
 surface=FusedTargetSurface(TargetTrackIdentity(16,'g5-t16'),points,indices,tuple(audit['multiview_surface']['source_stamps_ns']))
 rows=[]
 for deg in np.arange(-7.,7.01,.5):
  rot=rotation_matrix(np.deg2rad(deg),n)[:3,:3];t=T.copy();t[:3,:3]=rot@T[:3,:3];t[:3,3]=center+rot@(T[:3,3]-center)
  ev,bounds=evaluate_bilateral_surface_evidence_and_bounds(surface,center,t[:3,1],t[:3,2],g,support_normal_base=n)
  row=dict(yaw_delta_deg=float(deg),measured_bilateral_ok=ev.ok,counts=[ev.negative_jaw_points,ev.positive_jaw_points],measured_bounds_m=bounds,raw_open_boxes=boxes(t),placed=None)
  if ev.ok:
   geometry=dict(gripper=g,actual_T_base_tool0=t,planned_T_base_tool0=t,contact_center_base=center,contact_height_axis_base=t[:3,2],contact_height_bounds_m=bounds,minimum_contact_overlap_m=.002,support_normal_base=n,support_offset_m=obj['support_offset_m'],opening_width_m=opening)
   placed,report=place_contact_height(translation_evidence_limit_m=limit,**geometry)
   row.update(height_reason=report['reason'],height_ok=placed is not None)
   if placed is not None:row['placed']=dict(T_base_tool=placed.tolist(),normal_translation_m=report['normal_translation_m'],boxes=boxes(placed),margin_m=report['after']['maximum_symmetric_normal_translation_m'])
  rows.append(row)
 return dict(schema=1,scope='exact_reconstructed_fused_measurements_counterfactual_yaw',motion_authorized=False,physical_actions=0,original_plan_id=original['plan_id'],original_height_margin=original['height_margin'],same_rotation_overlap_intervals=intervals,current_snapshot_points_before_voxel=len(raw),current_snapshot_points_after_voxel=len(points),yaw_rows=rows,
  limitations=['Original 1848-point fused XYZ and representative view indices were reconstructed from sealed exact RGBD and original registration; all three source stamps and per-view counts match.', 'New yaw gets independently remeasured bilateral bounds on the reconstructed registered phase-reference surface.', 'No IK, full motion curve, or future SDK following validation; endpoint pass alone cannot authorize grasp.'])
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();d=run();a.output.write_text(json.dumps(d,indent=2)+'\n')
 print(json.dumps(dict(intervals=d['same_rotation_overlap_intervals'],rows=[dict(yaw=x['yaw_delta_deg'],bilateral=x['measured_bilateral_ok'],height=x.get('height_ok'),raw_collision=[b['body'] for b in x['raw_open_boxes'] if b['overlap']],placed_collision=[b['body'] for b in (x['placed'] or {}).get('boxes',[]) if b['overlap']]) for x in d['yaw_rows']])))
