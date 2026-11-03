"""Offline measured-plane alternatives, not execution authorization."""
import json,pathlib,numpy as np
from dataclasses import asdict
from alicia_flexible_grasp.grasp.tabletop_geometry_candidates import *
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry,evaluate_bilateral_surface_evidence_and_bounds
from alicia_flexible_grasp.grasp.contact_height_margin import evaluate_contact_height_margin
from alicia_flexible_grasp.vision.object_geometry import _fit_obb
from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface,TargetTrackIdentity
p=pathlib.Path(json.load(open('.runtime/current_handeye_deployment.json'))['directory']);contexts=json.load(open(p/'support_context_replay.json'));out=[]
f=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003);limit=.0026767676132463574
for ix,afile in enumerate(('second_grasp_planning_audit.json','current_observation_direct_audit.json')):
 a=json.load(open(p/afile));pts=np.array(a['replay_geometry']['object_points_base_m']);fit=contexts[ix]['fits'][3];n=np.array(fit['normal']);d=fit['offset_m'];center,R,size=_fit_obb(pts,n,d,.005,.6,.5,None)
 surface=FusedTargetSurface(TargetTrackIdentity(a['target_epoch'],a['multiview_surface']['target_track_id']),pts,np.zeros(len(pts),np.int64),(a['snapshot_stamp_ns'],))
 gen=generate_tabletop_proposals(pts,center,R,size,-d*n,n,TabletopGeometryConfig(max_candidates=8),surface,f)
 row=dict(source_audit=afile,plane_context_replay=contexts[ix],geometry=dict(center=center.tolist(),R=R.tolist(),size=size.tolist(),normal=n.tolist(),offset=d),proposals=len(gen.proposals),results=[])
 for proposal in gen.proposals:
  for c in materialize_tabletop_candidates(proposal,-d*n,n,f,approach_tilt_degrees=(2.,4.,6.,8.,10.,15.,20.,30.)):
   T=np.array(c.T_base_tool0);e,b=evaluate_bilateral_surface_evidence_and_bounds(surface,center,T[:3,1],T[:3,2],f,support_normal_base=n)
   if not e.ok:continue
   args=dict(gripper=f,actual_T_base_tool0=T,planned_T_base_tool0=T,contact_center_base=center,contact_height_axis_base=T[:3,2],contact_height_bounds_m=b,minimum_contact_overlap_m=.002,support_normal_base=n,support_offset_m=d,opening_width_m=.05)
   r=evaluate_contact_height_margin(translation_evidence_limit_m=limit,**args)
   ints=r.get('allowed_normal_translation_intervals_m',[])
   choices=[(u-l,(u+l)/2) for l,u in ints if l is not None and u is not None and l<=0<=u and u-l>=2*limit]
   if not choices:continue
   width,shift=max(choices);U=T.copy();U[:3,3]+=shift*n
   check=evaluate_contact_height_margin(translation_evidence_limit_m=limit,**dict(args,actual_T_base_tool0=U,planned_T_base_tool0=U))
   row['results'].append(dict(source_index=c.source_index,variant=c.variant_index,tilt=c.audit['approach_tilt_deg'],polarity=c.audit['approach_tilt_polarity'],margin_m=width/2,normal_displacement_m=shift,T=U.tolist(),contact_bounds=b,required_open_width_m=c.required_open_width_m,balanced_report=check))
 out.append(row);print(afile,'proposals',len(gen.proposals),'robust_candidates',len(row['results']),'best_margin_mm',max((r['margin_m']*1000 for r in row['results']),default=None),flush=True)
 for r in row['results'][:10]:print({k:r[k] for k in ('source_index','variant','tilt','polarity','margin_m','normal_displacement_m')},flush=True)
(p/'wider_support_candidate_replay.json').write_text(json.dumps(out,indent=2)+'\n')
