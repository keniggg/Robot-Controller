"""Frozen production candidate generation; no ROS services or robot commands."""
import json,pathlib,sys,numpy as np
from types import SimpleNamespace as NS
from _pytest.monkeypatch import MonkeyPatch
sys.path[:0]=['src/alicia_grasp_modes/tests','src/alicia_flexible_grasp_supervisor/tests']
from test_remote_modes import configured
from test_contact_tilt_replay import configured_replay
from alicia_flexible_grasp.vision.object_geometry import _support_context_mask,deproject_depth,_fit_support_plane,_fit_obb
from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface,SurfaceView
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.grasp.contact_height_placement import place_contact_height
p=pathlib.Path(json.load(open('.runtime/current_handeye_deployment.json'))['directory'])
a=json.load(open(p/'second_grasp_planning_audit.json'));D=np.load(p/'origin_support_replay_depth.npy');T=np.array(a['snapshot_transform']['T_base_optical']);bbox=a['graspnet_input']['detected_bbox_xywh']
mp=MonkeyPatch();node,geometry,snapshot,_,_=configured_replay(configured.__wrapped__(mp),'gen3_contact_tilt_20260926.json')
f=node.gripper_geometry;span=max(f.finger_size_xyz_m[0],f.max_inner_gap_m+2*f.finger_size_xyz_m[1])+2*f.support_clearance_m
K=NS(width=D.shape[1],height=D.shape[0],fx=395.7682800292969,fy=395.6027526855469,cx=317.9961242675781,cy=244.833984375)
mask=_support_context_mask(D,np.zeros(D.shape,bool),bbox,K,9.999999747378752e-05,.3,span)
pts,_=deproject_depth(D,mask,K,9.999999747378752e-05);pts=pts@T[:3,:3].T+T[:3,3]
n,d,ratio=_fit_support_plane(pts,.0035,100)
if n[2]<0:n=-n;d=-d
P=np.array(a['replay_geometry']['object_points_base_m']);center,R,size=_fit_obb(P,n,d,.005,.6,.5,None)
geometry=NS(object_points_base=P,center_base=center,axes_base=R,size_xyz_m=size,support_normal_base=n,support_offset_m=d)
stamp=a['snapshot_stamp_ns'];identity=TargetTrackIdentity(a['target_epoch'],a['multiview_surface']['target_track_id'])
surface=FusedTargetSurface(identity,P,np.zeros(len(P),int),(stamp,));ref=SurfaceView(identity,stamp,P,n,d,1);frozen=(surface,ref);node._snapshot_multiview_surface=lambda:frozen
node.gripper_physical_open_width_m=.05
cs,audit=node._generate_tabletop_candidates(geometry,snapshot)
rows=[]
for c in cs:
 _,e,b,axis=node._current_bilateral_surface_measurement(c.contact_center_base,c.T_base_tool0[:3,:3],surface_snapshot=frozen)
 U,report=place_contact_height(translation_evidence_limit_m=.0026767676132463574,**node._contact_height_geometry(c.T_base_tool0,geometry,c.contact_center_base,b,axis,.002))
 rows.append(dict(index=c.source_index,variant=c.variant_index,tilt=c.audit['approach_tilt_deg'],polarity=c.audit['approach_tilt_polarity'],feasible=U is not None,height_placement=report))
out=dict(scope='offline_frozen_production_generation_no_execution_authority',physical_support_span_m=span,support_normal=n.tolist(),support_offset_m=d,support_inlier_ratio=ratio,generator_audit=audit,candidates=rows)
(p/'production_support_height_replay.json').write_text(json.dumps(out,indent=2,default=lambda x:x.tolist() if hasattr(x,'tolist') else str(x))+'\n')
print('production_candidates',len(cs),'height_robust',sum(r['feasible'] for r in rows),'support_normal',n,flush=True)
print('robust_tilts',[r['tilt'] for r in rows if r['feasible']],flush=True)
mp.undo()
