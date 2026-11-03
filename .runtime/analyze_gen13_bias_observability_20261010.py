"""Diagnostic fits of sealed plane data only. Never produces runtime calibration."""
from pathlib import Path
import json,hashlib,math
import numpy as np
ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
OUT=ROOT/'ros_log/full_latest_restart_20261010T002347Z/gen13_handoff_analysis/support_plane_replay'
r=json.loads((OUT/'report.json').read_text());s=json.loads((OUT/'coordinate_sensitivity_expanded_metric.json').read_text())
frames=r['frames'];sens=s['rows'];anchor=np.asarray(r['reference']['anchor_base'])
initial=frames[0];train=[1,2];held=[3,4]
h=np.asarray([f['reference_delta']['anchor_signed_delta_mm']*.001 for f in frames])

def scalar_fit(key):
    g=np.asarray([row['sensitivity'][key] for row in sens]);step=-float(g[train]@h[train])/float(g[train]@g[train])
    residual=h+step*g
    return dict(parameter_step=step,training_residual_mm=(residual[train]*1000).tolist(),
        same_station_heldout_residual_mm=(residual[held]*1000).tolist(),
        training_rmse_mm=float(np.sqrt(np.mean(residual[train]**2)))*1000,
        per_station_individually_required_step=[-float(h[i]/g[i]) for i in train])

grad=np.asarray([q['sensitivity']['camera_fixed_translation_to_local_delta_gradient'] for q in sens])
t,unused,rank,singular=np.linalg.lstsq(grad[train],-h[train],rcond=None)
_,sv,vh=np.linalg.svd(grad[train],full_matrices=True)
residual=h+grad@t

def skew(v):
    x,y,z=v;return np.asarray([[0,-z,y],[z,0,-x],[-y,x,0]])
def rotation(v):
    a=float(np.linalg.norm(v))
    if a<1e-15:return np.eye(3)
    k=skew(v/a);return np.eye(3)+math.sin(a)*k+(1-math.cos(a))*(k@k)
def plane_with_mount_rotation(frame,delta):
    T=np.asarray(frame['T_base_camera']).copy();T[:3,:3]=T[:3,:3]@rotation(delta)
    return np.linalg.solve(T.T,np.asarray(frame['plane_camera']))
def normal_residual(delta,indices):
    base=plane_with_mount_rotation(frames[0],delta)[:3]
    return np.concatenate([plane_with_mount_rotation(frames[i],delta)[:3]-base for i in indices])

rot=np.zeros(3)
for _ in range(6):
    base=normal_residual(rot,train)
    J=np.column_stack([(normal_residual(rot+np.eye(3)[i]*1e-6,train)-normal_residual(rot-np.eye(3)[i]*1e-6,train))/2e-6 for i in range(3)])
    step=np.linalg.lstsq(J,-base,rcond=None)[0];rot+=step
    if np.linalg.norm(step)<1e-12:break
refcorrected=plane_with_mount_rotation(frames[0],rot)
normal_errors=[];rot_h=[];rot_grad=[]
for frame in frames:
    p=plane_with_mount_rotation(frame,rot)
    normal_errors.append(float(np.rad2deg(np.arccos(np.clip(p[:3]@refcorrected[:3],-1,1)))))
    rot_h.append(float(np.r_[anchor,1.]@(p-refcorrected)))
    rot_grad.append(refcorrected[:3]@np.asarray(frames[0]['T_base_camera'])[:3,:3]-p[:3]@np.asarray(frame['T_base_camera'])[:3,:3])
rot_h=np.asarray(rot_h);rot_grad=np.asarray(rot_grad)
rot_t,_,rot_rank,rot_singular=np.linalg.lstsq(rot_grad[train],-rot_h[train],rcond=None)
rot_residual=rot_h+rot_grad@rot_t

viewpoints=[]
for frame in frames:
    T=np.asarray(frame['T_base_camera']);cam=np.linalg.solve(T,np.r_[anchor,1.])[:3]
    viewpoints.append(dict(source_stamp_ns=frame['source_stamp_ns'],q_rad=sens[len(viewpoints)]['source_interpolated_q_rad'],
        camera_position_base_m=T[:3,3].tolist(),optical_axis_base=T[:3,0].tolist(),
        optical_axis_downward_cosine=float(-T[2,0]),anchor_camera_link_m=cam.tolist(),
        anchor_range_m=float(np.linalg.norm(cam)),anchor_optical_depth_m=float(cam[0]),
        plane_camera=frame['plane_camera'],plane_base=frame['plane_base']))

report=dict(provenance=dict(input_report=str(OUT/'report.json'),input_sha256=hashlib.sha256((OUT/'report.json').read_bytes()).hexdigest(),
    coordinate_report=str(OUT/'coordinate_sensitivity_expanded_metric.json'),coordinate_sha256=hashlib.sha256((OUT/'coordinate_sensitivity_expanded_metric.json').read_bytes()).hexdigest()),
    training_source_stamps_ns=[frames[i]['source_stamp_ns'] for i in train],reference_source_stamp_ns=frames[0]['source_stamp_ns'],
    same_station_heldout_source_stamps_ns=[frames[i]['source_stamp_ns'] for i in held],
    observed_anchor_local_delta_mm=(h*1000).tolist(),viewpoints=viewpoints,
    common_additive_depth_bias_fit=dict(unit='meters',**scalar_fit('depth_constant_bias_to_local_delta_gain')),
    common_depth_scale_error_fit=dict(unit='fraction_of_scale',**scalar_fit('depth_scale_error_to_local_delta_m_per_unit_scale')),
    fixed_mount_translation_only_fit=dict(camera_axes_minimum_norm_translation_m=t.tolist(),minimum_norm_mm=float(np.linalg.norm(t))*1000,
        constraint_rank=int(rank),unknown_count=3,nullspace_direction=vh[-1].tolist(),singular_values=singular.tolist(),
        training_residual_mm=(residual[train]*1000).tolist(),same_station_heldout_residual_mm=(residual[held]*1000).tolist(),
        unresolved_normal_errors_deg=[f['reference_delta']['normal_delta_deg'] for f in frames]),
    fixed_mount_rotation_and_translation_fit=dict(camera_axes_rotation_vector_rad=rot.tolist(),rotation_magnitude_deg=float(np.rad2deg(np.linalg.norm(rot))),
        postfit_normal_errors_deg=normal_errors,camera_axes_minimum_norm_translation_m=rot_t.tolist(),translation_norm_mm=float(np.linalg.norm(rot_t))*1000,
        translation_constraint_rank=int(rot_rank),translation_unknown_count=3,training_residual_mm=(rot_residual[train]*1000).tolist(),
        same_station_heldout_residual_mm=(rot_residual[held]*1000).tolist()),
    interpretation=[
        'A single common additive depth bias requires opposite signed per-view corrections, so it cannot explain both measured station offsets by itself.',
        'A single common depth scale error likewise cannot explain both station offsets by itself; its fitted fraction is diagnostic and must not be applied.',
        'Fixed mount translation can fit the two station-to-reference offsets but has rank 2 for 3 unknowns; there is an unobservable direction. It does not correct measured normal changes.',
        'A joint rotation/translation fit is also underconstrained in translation. Its near-stationary heldout frames test repeatability only, not cross-pose calibration accuracy.',
        'The exact expanded-metric FK and mount reproduce source TF at all five stamps. Software TF freshness/model-version mismatch is not the measured cause.',
        'Physical hand-eye error, robot geometric/compliance error, view-dependent depth bias and surface shape remain confounded with three independent camera poses.',
        'Keep 4 mm support and capture limits unchanged. Candidate actions are calibration with independent diverse poses/reference geometry, or observation planning constrained by measured calibration uncertainty. No fit in this file authorizes either a runtime correction or a grasp.',
    ],runtime_calibration_produced=False,device_commands=False,production_changes=False)
path=OUT/'bias_observability.json'
with path.open('x') as f:json.dump(report,f,ensure_ascii=False,indent=2);f.write('\n')
brief={k:v for k,v in report.items() if k in ['common_additive_depth_bias_fit','common_depth_scale_error_fit','fixed_mount_translation_only_fit','fixed_mount_rotation_and_translation_fit']}
print(json.dumps(dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),fits=brief)))
