"""Read-only analytic projection of the frozen current r1 diagnostic model.

No qualifier rerun, new fit, yaw reduction, ROS access or production model.
All extrema are interval endpoints or exact trigonometric stationary points.
"""
import hashlib
import json
import math
import os
from itertools import product
from pathlib import Path
import numpy as np
import yaml

W=Path(__file__).resolve().parents[1]
M=Path('/media/zhuyupei/SanStand')
C=M/'实验记录/Robot-Controller/ros_log/full_latest_gen29_restart_20261016T000728Z'
V=C/'verification'
O=V/'gen30_current_anisotropic_envelope_r1'
assert M.is_mount() and not O.exists()
inputs={}
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p,digest=None):
    inputs[str(p)]=sha(p)
    if digest:assert inputs[str(p)]==digest
    return json.loads(Path(p).read_text())
contract=read(V/'gen30_current_interval_box_r1/result.json','1b2e189a53131b41c5dc06c8d651e377b381ea184ff2635f8d31155e59636ef9')
passive=read(V/'gen30_current_passive_initial_r1/result.json')
source=contract['rows'][-1]
assert source['source_stamp_ns']==1792111337386243343 and source['individually_budget_qualified']==1
surviving=[r for r in source['candidates'] if r.get('downstream',{}).get('ok')]
assert len(surviving)==1
assert all(c.get('downstream',{}).get('ok') or any(reason.startswith(('measured_inner_not_top','top_continues_beyond_localization_interval','measured_occluder')) for reason in c['rejections']) for c in source['candidates'])
cfg_path=W/'src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml'
inputs[str(cfg_path)]=sha(cfg_path)
cfg=yaml.safe_load(cfg_path.read_text())['grasp_6d']['remote']['gripper_geometry']
physical=cfg['max_inner_gap_m'];fit=cfg['opening_fit_clearance_each_side_m'];clearance=cfg['width_safety_margin_per_side_m']
assert physical==.05 and fit==.0005 and clearance==.002
for path in (W/'.runtime/probe_gen30_current_interval_box.py',W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/gripper_geometry.py',W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/regular_box_inference.py'):
    inputs[str(path)]=sha(path)

def scalar_extrema(a,b,spread):
    angles=[-spread,spread]
    base=math.atan2(b,a)
    angles.extend(base+k*math.pi for k in range(-3,4) if -spread<=base+k*math.pi<=spread)
    values=[a*math.cos(t)+b*math.sin(t) for t in angles]
    return min(values),max(values),angles

models=[]
for row in surviving:
    m=row['downstream']['model']
    floor=row['plane_error_budget_m']+.5*m['qualification']['metric_pixel_size_m']
    # Keep the same depth/pixel floor independently for every edge. Unknown
    # silhouette extent is allocated to the edge that actually sees it.
    u=np.array([max(e['normal_interval_m'],floor+e['original_mask_unobserved_extension_m']) for e in row['edges']])
    size=np.array(m['size_xyz_m']);center=np.array(m['center_base']);axes=np.array(m['axes_base'])
    lo=-size[:2]/2-u[[3,0]];hi=size[:2]/2+u[[1,2]]
    models.append(dict(candidate_index=row['index'],model=m,size=size,center=center,axes=axes,
        lo=lo,hi=hi,u=u,yaw=m['empirical_yaw_uncertainty_rad']))
reference=models[0];A=reference['axes'];c0=reference['center'];nominal=float(reference['size'][1])

def union_projection(jaw,origin,yaw_override=None):
    """Exact projection of every uncertain box, including its center shift."""
    lower=[];upper=[];records=[]
    for m in models:
        j=m['axes'].T@jaw;offset=float((m['center']-origin)@jaw)
        spread=m['yaw'] if yaw_override is None else yaw_override
        for x,y in product((m['lo'][0],m['hi'][0]),(m['lo'][1],m['hi'][1])):
            lo,hi,angles=scalar_extrema(j[0]*x+j[1]*y,-j[0]*y+j[1]*x,spread)
            zlo=-m['size'][2]/2;zhi=m['size'][2]/2+m['model']['empirical_height_uncertainty_m']
            zz=[j[2]*z for z in (zlo,zhi)]
            lower.append(lo+offset+min(zz));upper.append(hi+offset+max(zz))
            records.append(dict(candidate=m['candidate_index'],vertex_xy=[float(x),float(y)],
                extremum_angles_rad=angles,local_projection_range_m=[lo,hi]))
    return float(min(lower)),float(max(upper)),records

def opening_report(name,origin):
    low,high,extrema=union_projection(A[:,1],origin)
    body=2*max(abs(low),abs(high))
    uncertainty=max(0.,body-nominal)
    required=body+2*fit;margin=physical-required;reserve=max(clearance,uncertainty)
    return dict(name=name,commanded_center_base=np.asarray(origin).tolist(),
        commanded_center_jaw_offset_from_nominal_m=float((origin-c0)@A[:,1]),
        exact_projection_range_about_command_m=[low,high],worst_symmetric_body_aperture_m=body,
        nominal_body_projection_m=nominal,width_and_center_uncertainty_m=uncertainty,
        required_opening_m=required,physical_opening_m=physical,opening_margin_m=margin,
        original_precision_reserve_m=reserve,reserve_surplus_m=margin-reserve,
        physical_aperture_pass=required<=physical+1e-12,
        original_precision_reserve_pass=margin>=reserve-1e-12,
        interval_extrema=extrema)

low,high,_=union_projection(A[:,1],c0)
best_center=c0+A[:,1]*(low+high)/2
measured=passive['downstream']
old_center=np.array(measured['geometry_center_base']);old_axes=np.array(measured['geometry_axes_base']);old_size=np.array(measured['geometry_size_m'])
openings=[opening_report('nominal_inferred_center',c0),
    opening_report('best_fixed_center_for_entire_unchanged_yaw_family',best_center),
    opening_report('unchanged_fused_measured_geometry_center_comparison',old_center)]

# Collision uses exactly the same uncertainty family; additionally contain the
# original measured OBB, matching production's measured+inferred union policy.
inferred_lo=[];inferred_hi=[]
for axis in A.T:
    a,b,_=union_projection(axis,np.zeros(3));inferred_lo.append(a);inferred_hi.append(b)
inferred_lo=np.array(inferred_lo);inferred_hi=np.array(inferred_hi)
old_vertices=np.array([old_center+old_axes@(np.array(sign)*old_size/2) for sign in product((-1.,1.),repeat=3)])
old_local=old_vertices@A
collision_lo=np.minimum(inferred_lo,old_local.min(axis=0));collision_hi=np.maximum(inferred_hi,old_local.max(axis=0))
vertices=np.array([A@np.array([collision_lo[i] if sign[i]<0 else collision_hi[i] for i in range(3)]) for sign in product((-1.,1.),repeat=3)])
assert np.all(old_local>=collision_lo-1e-12) and np.all(old_local<=collision_hi+1e-12)

# Pure diagnostic target: retain all edge budgets and find the yaw upper bound
# at which the existing reserve would first pass. This is not an approved
# smaller yaw and is never used above. Monotonicity follows because projection
# over [-yaw,yaw] is a supremum over nested sets.
def best_center_reserve_surplus(yaw):
    lo,hi,_=union_projection(A[:,1],c0,yaw_override=yaw)
    body=hi-lo;unc=max(0.,body-nominal)
    return physical-(body+2*fit)-max(clearance,unc)
target=None
if best_center_reserve_surplus(0.)>=0 and best_center_reserve_surplus(reference['yaw'])<0:
    low_yaw=0.;high_yaw=reference['yaw']
    for _ in range(60):
        middle=(low_yaw+high_yaw)/2
        if best_center_reserve_surplus(middle)>=0:low_yaw=middle
        else:high_yaw=middle
    target=dict(diagnostic_only_yaw_upper_rad=low_yaw,diagnostic_only_yaw_upper_deg=math.degrees(low_yaw),
        existing_edge_budgets_unchanged=True,assumes_best_fixed_center=True,
        not_a_reestimated_yaw=True,not_used_for_qualification=True)
assert all(sha(p)==digest for p,digest in inputs.items())
O.mkdir()
report=dict(classification='frozen_single_qualified_hypothesis_anisotropic_analytic_projection_only',
    source_stamp_ns=source['source_stamp_ns'],input_sha256=inputs,script_sha256=sha(__file__),
    candidate_audit=dict(examined=len(source['candidates']),included=[m['candidate_index'] for m in models],
        excluded=[dict(index=r['index'],reasons=r['rejections']) for r in source['candidates'] if not r.get('downstream',{}).get('ok')],
        every_excluded_hypothesis_has_positive_measured_conflict=True,
        failed_first_and_second_source_qualification_unchanged=True),
    per_model=[dict(candidate_index=m['candidate_index'],nominal_size_xyz_m=m['size'].tolist(),
        edge_order=['lower_y','upper_x','upper_y','lower_x'],edge_uncertainty_m=m['u'].tolist(),
        asymmetric_local_lower_xy_m=m['lo'].tolist(),asymmetric_local_upper_xy_m=m['hi'].tolist(),
        unchanged_yaw_rad=m['yaw'],unchanged_yaw_deg=math.degrees(m['yaw'])) for m in models],
    opening_comparisons=openings,
    collision=dict(policy='same_anisotropic_yaw_family_union_unchanged_measured_obb',axes_base=A.tolist(),
        center_base=(A@((collision_hi+collision_lo)/2)).tolist(),size_xyz_m=(collision_hi-collision_lo).tolist(),
        vertices_base_m=vertices.tolist(),measured_obb_contained=True,
        inferred_only_size_xyz_m=(inferred_hi-inferred_lo).tolist(),
        note='Conservative geometry envelope only; no scene path collision or contact planner was run.'),
    diagnostic_precision_target=target,
    outcome='PRECISION_VIEW_REQUIRED_UNDER_ORIGINAL_RESERVE',
    plane_correlation_note='The original 4.7355 degree yaw interval is retained. Its endpoint uncertainty includes a shared top-plane displacement and may overcount independent endpoint variation. A future correlated model must propagate that same variable jointly, not remove the error or choose opposite plane displacements without evidence.',
    limitations=['Not a production schema, requalification, trajectory, motion authorization or execution result.',
        'All interval budgets remain empirical; strict maxima are mathematical relative to these retained input intervals, not certified sensor error bounds.',
        'Only the current third source reached the diagnostic full-model gates. The failed first/second and old three sources are unchanged.',
        'Compared jaws are parallel to the inferred narrow faces; no robot pose/IK or alternative insertion direction is being certified.'],
    source_unchanged=True,production_modified=False,no_ros_calls=True,no_hardware_access=True)
(O/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
print(json.dumps(dict(result=str(O/'result.json'),sha256=sha(O/'result.json'),
    opening_comparisons=[{k:v for k,v in r.items() if k!='interval_extrema'} for r in openings],
    collision_size_m=report['collision']['size_xyz_m'],diagnostic_precision_target=target),ensure_ascii=False))
