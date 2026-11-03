"""Exact GEN7 fused-surface contact-band diagnostics; no actions or edits."""
from pathlib import Path
import dataclasses,hashlib,json
import numpy as np
from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry,evaluate_bilateral_surface_evidence_and_bounds,_linear_quantiles
from alicia_flexible_grasp.grasp.tabletop_geometry_candidates import TabletopGeometryConfig,generate_tabletop_proposals

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
REC=ROOT/'grasp_records/20261010T030141.809371Z_7724ffb5910c4565bf751bd9c281954c'
OUT=ROOT/'ros_log/full_latest_restart_20261010T002347Z/gen7_handoff_analysis/contact_analysis'
A=json.loads((REC/'keyframes_audits/960850384e1956c0f1d30c3c3d72dafa23240b9a26ea9eff056b3df416b28da9.json').read_text())
B=json.loads((REC/'keyframes_audits/e2a91743ab652cb5cd6cefafd051afa60ab851fe0255a22855f1569e15cf0f9f.json').read_text())
data=np.load(OUT/'exact_surface/fused_reconstruction.npz')
assert json.loads((OUT/'exact_surface/reconstruction.json').read_text())['exact_reconstruction']
g=A['replay_geometry'];points=np.array(g['object_points_base_m']);center=np.array(g['obb_center_base_m']);axes=np.array(g['R_base_obb']);normal=np.array(g['support_normal_base'])
surface=FusedTargetSurface(TargetTrackIdentity(4,'g1-t4'),data['current_points'],data['view_indices'],tuple(A['multiview_surface']['source_stamps_ns']))
gripper=GripperGeometry(.05,.002,np.array([.0434,.0286,.060]),np.array([.1175,.155,.0774]),.003)
cfg=TabletopGeometryConfig(angle_step_deg=5.,angle_dedup_deg=1.,opening_fit_clearance_each_side_m=.0005,width_projection_trim_fraction=.01,max_candidates=32)
gen=generate_tabletop_proposals(object_points_base=points,obb_center_base=center,R_base_obb=axes,obb_size_xyz_m=g['obb_size_xyz_m'],support_point_base=-normal*g['support_offset_m'],support_normal_base=normal,config=cfg,fused_surface=surface,finger_geometry=gripper,refine_obb_neighbor_angles=True)
assert len(gen.proposals)==A['remote_diagnostics']['tabletop_geometry']['proposal_count']
assert np.max(np.abs(np.array(gen.sampled_angles_deg)-np.array(A['remote_diagnostics']['tabletop_geometry']['sampled_angles_deg'])))<1e-10

def breakdown(jaw):
 jaw=np.array(jaw);insertion=-normal;cross=np.cross(jaw,insertion);cross/=np.linalg.norm(cross)
 delta=surface.points_base-center;jp=delta@jaw;ip=delta@insertion;cp=delta@cross;hp=delta@normal
 footprint=(abs(cp)<=.0434/2)&(abs(ip)<=.060/2)&(abs(jp)<=.025+1e-9)
 lo,hi=_linear_quantiles(jp[footprint],[.05,.95]);mid=(lo+hi)/2
 neg=footprint&(abs(jp-lo)<=.002)&(jp<mid);pos=footprint&(abs(jp-hi)<=.002)&(jp>mid)
 original=[neg.copy(),pos.copy()]
 heights=[_linear_quantiles(hp[mask],[.05,.95]) for mask in original]
 common=[max(x[0] for x in heights),min(x[1] for x in heights)]
 keep=(hp>=common[0])&(hp<=common[1]);masks=[mask&keep for mask in original]
 ev,bounds=evaluate_bilateral_surface_evidence_and_bounds(surface,center,jaw,insertion,gripper,support_normal_base=normal)
 perview=[]
 for view in range(4):
  v=surface.view_indices==view
  perview.append(dict(view_index=view,before_common_counts=[int(np.count_nonzero(mask&v)) for mask in original],after_common_counts=[int(np.count_nonzero(mask&v)) for mask in masks]))
 return dict(evidence=dataclasses.asdict(ev),bounds_m=bounds,footprint_points=int(footprint.sum()),raw_side_point_counts=[int(mask.sum()) for mask in original],robust_support_height_bounds_m=[x.tolist() for x in heights],common_support_height_bounds_m=common,common_support_height_span_m=common[1]-common[0],after_common_point_counts=[int(mask.sum()) for mask in masks],per_view=perview)

rows=[]
for side in B['observe_return']['measured_missing_sides']:
 row=breakdown(side['jaw_axis_base']);row['angle_deg']=side['angle_deg'];row['recorded_counts']=[side['negative_measured_points'],side['positive_measured_points']]
 row['counts_reproduced']=[row['evidence']['negative_jaw_points'],row['evidence']['positive_jaw_points']]==row['recorded_counts'];rows.append(row)
assert all(row['counts_reproduced'] for row in rows)
local=(points-center)@axes;rawlocal=(data['raw_points']-center)@axes
summary=dict(schema=1,script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
 source_audit_sha256='960850384e1956c0f1d30c3c3d72dafa23240b9a26ea9eff056b3df416b28da9',
 proposal_count=len(gen.proposals),sampled_angles_count=len(gen.sampled_angles_deg),
 proposal_angles=[dict(index=p.source_index,angle_deg=p.angle_deg,negative_count=p.negative_contact_count,positive_count=p.positive_contact_count,contact_bounds=p.audit.get('contact_height_bounds_m')) for p in gen.proposals],
 side_bands=rows,
 obb=dict(recorded_size_m=g['obb_size_xyz_m'],union_actual_axis_bounds_m=[local.min(0).tolist(),local.max(0).tolist()],union_actual_axis_span_m=np.ptp(local,axis=0).tolist(),union_axis_midpoint_offset_m=((local.max(0)+local.min(0))/2).tolist(),raw_actual_axis_span_m=np.ptp(rawlocal,axis=0).tolist()),
 interpretation='Current exact fused bands have many initial side points; enforcing their actual common support-normal height leaves the small recorded counts. This is a measured contact-band limitation, not missing entire object geometry or a midpoint-scope failure.',
 limitations=['Diagnostic replica retains all original thresholds and poses. No claim that changing center, geometry bounds or thresholds authorizes contact.', 'View indices identify retained source ownership, not a count of physically independent camera viewpoints.'])
with (OUT/'band_breakdown.json').open('x') as out:json.dump(summary,out,ensure_ascii=False,indent=2);out.write('\n')
print(json.dumps(dict(proposal_count=len(gen.proposals),side_bands=[{k:v for k,v in r.items() if k not in ['per_view','evidence']} for r in rows],obb=summary['obb']),ensure_ascii=False,indent=2))
