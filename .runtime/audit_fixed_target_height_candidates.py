"""Read-only replay of recorded contact candidates; no control publishers."""
import json, pathlib, os
from dataclasses import asdict
import numpy as np
from tf.transformations import quaternion_matrix
from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface,TargetTrackIdentity
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry,evaluate_bilateral_surface_evidence_and_bounds
from alicia_flexible_grasp.grasp.contact_height_margin import evaluate_contact_height_margin
p=pathlib.Path(json.load(open('.runtime/current_handeye_deployment.json'))['directory']);source=os.environ.get('HEIGHT_AUDIT_SOURCE','second_grasp_planning_audit.json');a=json.load(open(p/source))
g=a['replay_geometry'];pts=np.array(g['object_points_base_m']);n=np.array(g['support_normal_base']);center=np.array(a['target_cloud_center_base'])
identity=TargetTrackIdentity(a['target_epoch'],a['multiview_surface']['target_track_id'])
s=FusedTargetSurface(identity,pts,np.zeros(len(pts),dtype=np.int64),(a['snapshot_stamp_ns'],))
f=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)
rows=[]
for i,e in enumerate(a['stable_evaluations']):
 st=next(x for x in e['execution_sequence']['stages'] if x['stage']=='grasp');T=quaternion_matrix(st['quaternion_xyzw']);T[:3,3]=st['position_m']
 ev,bounds=evaluate_bilateral_surface_evidence_and_bounds(s,center,T[:3,1],T[:3,2],f,support_normal_base=n)
 row=dict(index=i,source_index=e['source_index'],variant=e['variant_index'],evidence=asdict(ev),bounds=bounds,endpoint_hint=e.get('endpoint_ik_search_probe',{}).get('status'))
 if ev.ok:
  args=dict(gripper=f,actual_T_base_tool0=T,planned_T_base_tool0=T,contact_center_base=center,contact_height_axis_base=T[:3,2],contact_height_bounds_m=bounds,minimum_contact_overlap_m=.002,support_normal_base=n,support_offset_m=g['support_offset_m'],opening_width_m=.05)
  rep=evaluate_contact_height_margin(translation_evidence_limit_m=.0026767676132463574,**args)
  row['height_report']=rep
  intervals=rep.get('allowed_normal_translation_intervals_m',[])
  row['best_possible_normal_margin_m']=max(((b-a)/2 for a,b in intervals if a is not None and b is not None),default=0.)
 rows.append(row)
(p/('height_intervals_'+source)).write_text(json.dumps(dict(fixed_target=True,source_audit=source,rows=rows),indent=2)+'\n')
for r in sorted(rows,key=lambda x:-x.get('best_possible_normal_margin_m',0)):
 print(r['index'],r['source_index'],r['variant'],'hint',r['endpoint_hint'],'best_mm',round(r.get('best_possible_normal_margin_m',0)*1000,3),'span_mm',round((r['bounds'][1]-r['bounds'][0])*1000,3) if r['bounds'] else None,'intervals',r.get('height_report',{}).get('allowed_normal_translation_intervals_m'))
