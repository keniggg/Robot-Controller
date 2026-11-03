"""Offline exact-source reconstructed surface: original gates, angle-only counterfactual."""
from pathlib import Path
from types import SimpleNamespace as NS
from dataclasses import asdict,is_dataclass
import json,hashlib,math,time
import numpy as np
import yaml
from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface,SurfaceView
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.grasp import tabletop_geometry_candidates as tg
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry,_obb_line_interval,evaluate_bilateral_surface_evidence_and_bounds,_bilateral_surface_measurement
import test_remote_grasp6d_streaming as fixtures
remote=fixtures.remote_node
ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z/gen29_contact_analysis')
STAMP=1791549928519117355
meta=json.loads((ROOT/'source_inputs/manifest.json').read_text()); reconstruction=json.loads((ROOT/'fused_reconstruction_final/result.json').read_text())
audit_path=Path(meta['record'])/'keyframes_audits/f2bfb1e7c5c0ccb1b7e2b5043f75fbea233170930bfa077499fdabc36f495bc3.json';audit=json.loads(audit_path.read_text())
z=np.load(ROOT/'fused_reconstruction_final'/('%d_contact_inputs.npz'%STAMP)); cfg=yaml.safe_load(Path('src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml').read_text());rcfg=cfg['grasp_6d']['remote'];gg=rcfg['gripper_geometry']
node=remote.RemoteGrasp6DNode.__new__(remote.RemoteGrasp6DNode)
node.gripper_geometry=GripperGeometry(gg['max_inner_gap_m'],gg['width_safety_margin_per_side_m'],np.array(gg['finger_box_xyz_m']),np.array(gg['palm_box_xyz_m']),gg.get('support_clearance_m',.003))
node.gripper_tool_jaw_axis=gg.get('tool_jaw_axis','y');node.gripper_tool_finger_length_axis=gg.get('tool_finger_length_axis','z')
node.opening_fit_clearance_each_side_m=remote.load_opening_fit_clearance_config(gg,node.gripper_geometry)
node.tabletop_geometry_enabled,node.tabletop_geometry_config,node.hybrid_merge_config=remote.load_tabletop_geometry_config(rcfg,node.gripper_geometry,node.opening_fit_clearance_each_side_m)
node.adaptive_stage_limits=remote.load_adaptive_stage_config(rcfg);node.candidate_min_downward_approach_cos=rcfg.get('candidate_min_downward_approach_cos',.65);node.candidate_max_final_approach_lateral_m=rcfg.get('candidate_max_final_approach_lateral_m',.02)
node.grasp_config=cfg['grasp'];node.gripper_physical_open_width_m=.05
node._runtime_execution_position_error_m=lambda:audit['remote_diagnostics']['tabletop_geometry']['adaptive_stage_profiles'][0]['execution_position_error_m']
identity=TargetTrackIdentity(meta['target_epoch'],meta['target_track_id'])
surface=FusedTargetSurface(identity,z['current_points'],z['view_indices'],tuple(meta['surface_source_stamps_ns']),view_sources=tuple(tuple(x) for x in reconstruction['online_fused_geometry_checks'][-1]['view_sources']))
REG=z['last_transform'];inverse=np.linalg.inv(REG)
rp=np.load(ROOT/'fused_reconstruction_final'/('%d_geometry.npz'%meta['surface_source_stamps_ns'][0]))['object_points_base']
reference=SurfaceView(identity,meta['surface_source_stamps_ns'][0],rp@inverse[:3,:3].T+inverse[:3,3],inverse[:3,:3]@z['reference_support_normal'],float(z['reference_support_offset']+z['reference_support_normal']@REG[:3,3]),reconstruction['views'][0]['edge_clearance_px'])
node._snapshot_multiview_surface=lambda:(surface,reference)
g=audit['replay_geometry']; geometry=NS(object_points_base=np.array(g['object_points_base_m']),center_base=np.array(g['obb_center_base_m']),axes_base=np.array(g['R_base_obb']),size_xyz_m=np.array(g['obb_size_xyz_m']),support_normal_base=np.array(g['support_normal_base']),support_offset_m=g['support_offset_m'])
quality=reconstruction['online_fused_geometry_checks'][-1]['snapshot_quality'];snapshot=NS(quality=NS(**quality))
normal=geometry.support_normal_base;bx=tg._plane_basis_x(normal);by=np.cross(normal,bx)
def axis(angle):return bx*np.cos(np.deg2rad(angle))+by*np.sin(np.deg2rad(angle))
def cv(v):
 if isinstance(v,np.ndarray):return v.tolist()
 if isinstance(v,np.generic):return v.item()
 if is_dataclass(v):return {k:cv(x) for k,x in asdict(v).items()}
 if isinstance(v,dict):return {str(k):cv(x) for k,x in v.items()}
 if isinstance(v,(tuple,list)):return [cv(x) for x in v]
 return v
OUT=ROOT/'angle_window';OUT.mkdir(exist_ok=True)
def save(name,obj):(OUT/name).write_text(json.dumps(cv(obj),indent=2)+'\n')
start=time.monotonic();rows=[]
for a in sorted(set(np.round(np.arange(40.,45.000001,.01),2)) | set(np.round(np.arange(45.,70.000001,.25),2))):
 jaw=axis(a);e,b=evaluate_bilateral_surface_evidence_and_bounds(surface,geometry.center_base,jaw,-normal,node.gripper_geometry,minimum_points_per_side=max(12,node.tabletop_geometry_config.min_contact_band_points),support_normal_base=normal)
 interval=_obb_line_interval(geometry.center_base,jaw,geometry.center_base,geometry.axes_base,geometry.size_xyz_m)
 reach=max(-interval[0],interval[1])+node.opening_fit_clearance_each_side_m
 rows.append(dict(angle_deg=float(a),evidence=asdict(e),bounds_m=b,common_interval_span_m=None if b is None else b[1]-b[0],required_open_width_m=e.measured_width_m+2*node.opening_fit_clearance_each_side_m,obb_negative_reach_m=-interval[0]+node.opening_fit_clearance_each_side_m,obb_positive_reach_m=interval[1]+node.opening_fit_clearance_each_side_m,reach_ok=reach<=.025+1.001e-9,proposal_ok=e.ok and e.measured_width_m+2*node.opening_fit_clearance_each_side_m<=.05))
survivors=[x for x in rows if x['reach_ok'] and x['proposal_ok']]
save('fast_gate_result.json',dict(rows=rows,intersection_count=len(survivors),intersection_angles=[x['angle_deg'] for x in survivors],wall_sec=time.monotonic()-start,reconstruction_checks=reconstruction['prior_audit_checks'],scope='RECONSTRUCTION_COUNTERFACTUAL_NO_EXECUTION'))
print('fast intersection',len(survivors),'range',([survivors[0]['angle_deg'],survivors[-1]['angle_deg']] if survivors else []),flush=True)
# Original full factory with the original 5deg grid first, then one angle at a time.
base_candidates,base_diag=node._generate_tabletop_candidates(geometry,snapshot=snapshot,contact_execution_phase=True)
save('original_grid_replay.json',dict(candidate_count=len(base_candidates),diagnostics=base_diag,online_diagnostics=audit['remote_diagnostics']['tabletop_geometry']))
chosen=[x for x in survivors if abs(x['angle_deg']*10-round(x['angle_deg']*10))<1e-7]
if survivors:
 for x in (survivors[0],survivors[-1]):
  if x not in chosen:chosen.append(x)
chosen.sort(key=lambda x:x['angle_deg']); original=tg._sample_angles;full=[]
try:
 for x in chosen:
  a=x['angle_deg'];tg._sample_angles=lambda rotation,normal,config,a=a:((a,axis(a)),)
  t=time.monotonic();cs,diag=node._generate_tabletop_candidates(geometry,snapshot=snapshot,contact_execution_phase=True)
  full.append(dict(angle_deg=a,candidate_count=len(cs),diagnostics=diag,candidates=[dict(source_index=c.source_index,variant_index=c.variant_index,audit=dict(c.audit),required_open_width_m=c.required_open_width_m,T_base_tool0=c.T_base_tool0,contact_center_base=c.contact_center_base) for c in cs],wall_sec=time.monotonic()-t))
  save('full_factory_result.json',dict(rows=full,wall_sec=time.monotonic()-start,scope='ORIGINAL_FACTORY_AND_ALL_ANALYTIC_CONTACT_GATES_NO_MOVEIT_NO_EXECUTION',limitations=['Only the angle sampler is replaced for offline exploration.','Reconstructed cloud is compared with audit; not claimed equal unless all validation matches.','Passing a factory candidate is not full MoveIt/path/contact-sequence qualification.']))
  print('angle',a,'materialized',len(cs),'reason',diag.get('rejection_counts'),flush=True)
finally:tg._sample_angles=original
# Per measured-view contributions, in the final current frame, before vs after original accepted registration.
regs={x['source_stamp_ns']:np.array(x['registration']['transform_base']) for x in reconstruction['registrations']}; contributions=[]
for i,ns in enumerate(meta['surface_source_stamps_ns']):
 p=np.load(ROOT/'fused_reconstruction_final'/('%d_geometry.npz'%ns))['object_points_base'];T=regs.get(ns,np.eye(4));registered=p@T[:3,:3].T+T[:3,3];cur=registered@inverse[:3,:3].T+inverse[:3,3]
 retained=np.array([i in x for x in surface.view_sources],dtype=bool)
 vals=[]
 for a in [40.,41.,42.,43.,44.,45.,53.843872185121214]:
  jaw=axis(a);proj=(cur-geometry.center_base)@jaw;native=(p-geometry.center_base)@jaw;fp=(surface.points_base[retained]-geometry.center_base)@jaw
  vals.append(dict(angle_deg=a,registered_projection_min_max_m=[proj.min(),proj.max()],native_projection_min_max_m=[native.min(),native.max()],retained_fused_projection_min_max_m=[fp.min(),fp.max()] if len(fp) else None,registered_projection_q01_q99_m=np.quantile(proj,[.01,.99]),native_to_registered_max_abs_jaw_shift_m=float(np.max(abs((cur-p)@jaw)))))
 contributions.append(dict(view_index=i,source_stamp_ns=ns,native_point_count=len(p),retained_shared_ownership_count=int(retained.sum()),angles=vals,registration=T,registered_support_height_quantiles_m=np.quantile(cur@normal+geometry.support_offset_m,[0,.01,.5,.99,1])))
save('view_width_contributions.json',dict(rows=contributions,scope='Native measured-view points and registered contributions; extremes are not all contact evidence. No pruning.'))
print('complete',str(OUT),flush=True)
