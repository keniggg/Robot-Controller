"""Conservative finite r2 union; nominal rejection never excludes a family.

One frozen frame, all 24 original line tuples. No qualifier, source edits,
production model, RGB resampling claim, hardware access, or yaw reduction.
"""
import ast
import hashlib
import json
import math
from itertools import product
from pathlib import Path
import numpy as np

W=Path(__file__).resolve().parents[1]
M=Path('/media/zhuyupei/SanStand');assert M.is_mount()
V=M/'实验记录/Robot-Controller/ros_log/full_latest_gen29_restart_20261016T000728Z/verification'
O=V/'gen30_current_interval_family_r2';assert not O.exists()
inputs={}
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p,digest=None):
    inputs[str(p)]=sha(p)
    if digest:assert inputs[str(p)]==digest
    return json.loads(Path(p).read_text())
old=read(V/'gen30_current_interval_box_r1/result.json','1b2e189a53131b41c5dc06c8d651e377b381ea184ff2635f8d31155e59636ef9')['rows'][-1]
trace=read(V/'gen30_current_visual_trace_r1/result.json','6e914a1f4c7152fa8cb07de1c86263f007072ec9d4c478a89f11e6ce7abf9e31')['rows'][-1]
passive=read(V/'gen30_current_passive_initial_r1/result.json')
review=read(V/'gen30_current_interval_independent_review_r1/common_plane_endpoint_checks.json')
assert old['source_stamp_ns']==trace['source_stamp_ns']==1792111337386243343
npz=V/'gen30_current_passive_initial_r1/1792111337386243343_arrays.npz';inputs[str(npz)]=sha(npz)
with np.load(npz,allow_pickle=False) as f:a={k:f[k].copy() for k in f.files}
p=W/'.runtime/replay_gen30_initial_visual_direction_full.py';inputs[str(p)]=sha(p)
s=p.read_text();node=next(n for n in ast.parse(s).body if isinstance(n,ast.FunctionDef) and n.name=='fitted_rectangle')
exec(compile(ast.get_source_segment(s,node),'<frozen_pure_endpoint_ls>','exec'),globals())
d=trace['qualification']['return_local_diagnostics'];origin=np.array(d['origin']);tn=np.array(d['tn'])
base_axes=np.array(d['axes']);K=a['intrinsics'];scale=float(a['depth_scale_m'])
link=np.eye(4);link[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]];T=a['T_base_camera_link']@link
camera=T[:3,3];h=d['top_height'];n=base_axes[:,2];support_d=passive['downstream']['geometry_support_offset_m'] if 'geometry_support_offset_m' in passive['downstream'] else None
refmodel=old['candidates'][3]['downstream']['model'];assert refmodel is not None
support_d=refmodel['support_offset_m'];hu=refmodel['empirical_height_uncertainty_m']
plane_budget=old['candidates'][3]['plane_error_budget_m']

def plane_points(pixels,normal,point):
    ray=np.column_stack(((pixels[:,0]-K[2])/K[0],(pixels[:,1]-K[3])/K[1],np.ones(len(pixels))))@T[:3,:3].T
    denom=ray@normal;distance=((point-camera)@normal)/denom
    if np.any(~np.isfinite(distance)) or np.any(distance<=0):raise ValueError('unbounded ray slab')
    return camera+ray*distance[:,None]
def top(p):return plane_points(p,tn,origin)
def cells(p):return np.array([q+[dx,dy] for q in p for dx,dy in product((-.5,.5),repeat=2)])
def endpoint_support(p):
    pixel=cells(np.array([p]))
    return np.vstack([plane_points(pixel,tn,origin+shift*tn) for shift in (-plane_budget,plane_budget)])

def projection(points,j,spread):
    """Every point's exact trig range; no angular grid."""
    aa=points[:,0]*j[0]+points[:,1]*j[1]
    bb=-points[:,1]*j[0]+points[:,0]*j[1]
    values=np.stack([aa*np.cos(t)+bb*np.sin(t) for t in (-spread,spread)])
    low=values.min(axis=0);high=values.max(axis=0)
    phases=np.arctan2(bb,aa)
    for k in range(-2,3):
        angle=phases+k*math.pi;within=(angle>=-spread)&(angle<=spread)
        v=aa*np.cos(angle)+bb*np.sin(angle)
        low=np.where(within,np.minimum(low,v),low);high=np.where(within,np.maximum(high,v),high)
    return float(low.min()),float(high.max())

# Include every original masked measured point, not the fused/eroded subset.
# Unknown masked pixels retain their complete ideal-cell rays between the
# verified support and top+height budget. This is deliberately an outer bound,
# not a claim that unknown pixels are measured at either endpoint.
mask=a['object_mask']>0;depth=a['depth_raw'];valid=np.isfinite(depth)&(depth>0)
ys,xs=np.nonzero(mask&valid);z=depth[ys,xs].astype(float)*scale
measured=np.column_stack(((xs-K[2])*z/K[0],(ys-K[3])*z/K[1],z))@T[:3,:3].T+camera
uy,ux=np.nonzero(mask&~valid);ucells=cells(np.column_stack((ux,uy)))
unknown=np.vstack([plane_points(ucells,n,-support_d*n),plane_points(ucells,n,(h+hu-support_d)*n)])
raw_bounds=np.vstack((measured,unknown))
choices=d['visual_rectangle_trace']['choices_before_top3']
choices=[[(r['offset_m'],r['overlap_m'],np.array(r['pixels']),r['slope']) for r in group] for group in choices]
families=[]
for index,selected in enumerate(product(*choices),1):
    previous=old['candidates'][index-1]
    axes,lo,hi,slope,residual,fit=fitted_rectangle(selected,top,base_axes)
    size=np.r_[hi-lo,h];center=axes[:,:2]@((lo+hi)/2)+n*(h/2-support_d)
    supports=[];angle_bounds=[]
    for i,row in enumerate(selected):
        e0=(endpoint_support(row[2][0])-center)@axes[:,:2]
        e1=(endpoint_support(row[2][1])-center)@axes[:,:2]
        across=1 if i%2==0 else 0;along=1-across
        if np.mean(e1[:,along])<np.mean(e0[:,along]):e0,e1=e1,e0
        du_min=float(e1[:,along].min()-e0[:,along].max())
        dv_low=float(e1[:,across].min()-e0[:,across].max())
        dv_high=float(e1[:,across].max()-e0[:,across].min())
        if du_min<=0:raise ValueError('line direction interval unbounded')
        angle_bounds.append(math.atan2(max(abs(dv_low),abs(dv_high)),du_min))
        supports.append(np.vstack((e0,e1)))
    # Common-plane correlation is deliberately relaxed to a superset; the
    # result cannot undercut r1 yaw and does not claim joint feasibility.
    yaw=max(previous['yaw_budget_rad'],*angle_bounds)
    local_lo=[];local_hi=[]
    offsets=[]
    for i,points in enumerate(supports):
        across=1 if i%2==0 else 0;j=[0.,0.];j[across]=1.
        low,high=projection(points,j,yaw)
        offsets.append([low,high])
    # In the rotated family coordinates each edge offset is constant along
    # the full infinite line. No short-segment interval is reused outside its
    # observed endpoint span as if it covered a full nominal line.
    flo=np.array([offsets[3][0],offsets[0][0]])
    fhi=np.array([offsets[1][1],offsets[2][1]])
    floor=previous['plane_error_budget_m']+.5*d['pixel_size']
    old_u=np.array([max(e['normal_interval_m'],floor+e['original_mask_unobserved_extension_m']) for e in previous['edges']])
    flo=np.minimum(flo,-size[:2]/2-old_u[[3,0]])
    fhi=np.maximum(fhi,size[:2]/2+old_u[[1,2]])
    raw=(raw_bounds-center)@axes[:,:2]
    for j in range(2):
        direction=[0.,0.];direction[j]=1.
        low,high=projection(raw,direction,yaw)
        flo[j]=min(flo[j],low);fhi[j]=max(fhi[j],high)
    families.append(dict(index=index,axes=axes,center=center,size=size,lo=flo,hi=fhi,yaw=yaw,
        original_yaw=previous['yaw_budget_rad'],offsets=offsets,
        nominal_rejections=previous['rejections'],r1_downstream=previous.get('downstream',{}).get('code')))
assert len(families)==24 and {r['index'] for r in families}==set(range(1,25))

reference=next(r for r in families if r['index']==4);A=reference['axes'];c0=reference['center']
def union_interval(jaw,origin):
    bounds=[]
    for f in families:
        points=np.array(list(product((f['lo'][0],f['hi'][0]),(f['lo'][1],f['hi'][1]))))
        localjaw=f['axes'].T@jaw
        low,high=projection(points,localjaw[:2],f['yaw'])
        z=[-f['size'][2]/2,f['size'][2]/2+hu];zz=[localjaw[2]*v for v in z]
        shift=float((f['center']-origin)@jaw)
        bounds.append((low+min(zz)+shift,high+max(zz)+shift,f['index']))
    return min(r[0] for r in bounds),max(r[1] for r in bounds),bounds
low,high,witnesses=union_interval(A[:,1],c0)
best_center=c0+A[:,1]*(low+high)/2
body=high-low;nominal=float(reference['size'][1]);uncertainty=max(0.,body-nominal)
required=body+.001;margin=.05-required;reserve=max(.002,uncertainty)
ilo=[];ihi=[]
for axis in A.T:
    low,high,_=union_interval(axis,np.zeros(3));ilo.append(low);ihi.append(high)
ilo=np.array(ilo);ihi=np.array(ihi)
g=passive['downstream'];mc=np.array(g['geometry_center_base']);ma=np.array(g['geometry_axes_base']);ms=np.array(g['geometry_size_m'])
mv=np.array([mc+ma@(np.array(sign)*ms/2) for sign in product((-1.,1.),repeat=3)])@A
clo=np.minimum(ilo,mv.min(axis=0));chi=np.maximum(ihi,mv.max(axis=0))
report=dict(classification='r2_all_nominal_families_retained_outer_approximation_diagnostic',
    source_stamp_ns=old['source_stamp_ns'],input_sha256=inputs,script_sha256=sha(__file__),
    retained_family_count=len(families),family_exclusions=[],
    measured_original_mask_point_count=len(measured),unknown_original_mask_pixel_count=len(ux),
    families=[dict(index=f['index'],nominal_rejections_are_not_family_exclusions=f['nominal_rejections'],
        r1_full_qualifier_code=f['r1_downstream'],center_base=f['center'].tolist(),axes_base=f['axes'].tolist(),
        nominal_size_xyz_m=f['size'].tolist(),edge_offset_intervals_over_whole_yaw_family_m=f['offsets'],
        local_lower_xy_m=f['lo'].tolist(),local_upper_xy_m=f['hi'].tolist(),
        yaw_rad=f['yaw'],original_yaw_rad=f['original_yaw'],yaw_not_reduced=f['yaw']>=f['original_yaw']) for f in families],
    opening=dict(jaw_axis_base=A[:,1].tolist(),best_fixed_center_base=best_center.tolist(),
        reference_center_base=c0.tolist(),reference_nominal_width_m=nominal,
        all_family_projection_range_at_reference_center_m=[min(r[0] for r in witnesses),max(r[1] for r in witnesses)],
        body_aperture_best_fixed_center_m=body,required_opening_m=required,physical_opening_m=.05,
        uncertainty_m=uncertainty,aperture_margin_m=margin,precision_reserve_m=reserve,
        precision_reserve_surplus_m=margin-reserve,all_family_projection_ranges=witnesses),
    collision=dict(axes_base=A.tolist(),center_base=(A@((clo+chi)/2)).tolist(),size_xyz_m=(chi-clo).tolist(),
        includes_all_24_outer_families=True,includes_original_measured_obb=True),
    outcome='PRECISION_REQUIRED_NOT_PRODUCTION_QUALIFIED',
    corrected_semantics=['A nominal external-depth conflict does not prove exclusion of the complete uncertain line family.',
        'All 24 families are retained; candidate 12 is not silently removed.',
        'Each rotated edge has an offset interval for the full line, so a short Hough segment is not asserted to bound an extrapolated nominal edge.',
        'Independent endpoint boxes deliberately overapproximate the common plane variable. No correlation-based budget reduction is used.',
        'Every original masked measured point and bounded unknown ideal-cell ray slab enters the positional outer approximation.'],
    limitations=['This is a conservative outer approximation, not proof that all 24 families are physically feasible.',
        'Original full-qualifier failures remain failures; this script never creates a qualified model or upgrades first/second source.',
        'Ideal half-pixel source support remains incomplete for real RGB resampling; therefore even a numerical fit could not authorize production.',
        'Raw unknown-ray height bounds rely on the declared regular-prism assumption and empirical height; no synthetic depth is created.',
        'All candidate enumeration is finite; no completeness proof beyond these recorded line groups is claimed.'],
    production_modified=False,no_ros_calls=True,no_hardware_access=True,motion_authorized=False)
assert all(sha(p)==digest for p,digest in inputs.items())
O.mkdir();(O/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
print(json.dumps(dict(result=str(O/'result.json'),sha256=sha(O/'result.json'),opening=report['opening'],collision=report['collision']),ensure_ascii=False))
