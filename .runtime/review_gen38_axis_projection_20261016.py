"""Independent axis/normal projection of an unchanged saved cuboid family.

No fit, source pixel, height interval or production function is modified.
The saved four raw groups are given a tighter analytical normal bound; the
original joint arithmetic is captured from source for independent replay.
This remains a numerical feasibility diagnostic, not a qualified model.
"""
import ast
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import sys

W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
D=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen36_restart_20261017T012037Z')
O=D/'verification/gen38_anisotropic_fixed_A_review_r1'
P=D/'verification/gen38_cuboid_silhouette_probe_r1/fixed_height_proposal_diagnostics.json'
assert Path('/media/zhuyupei/SanStand').is_mount() and 'sitecustomize' in sys.modules
assert os.environ['OFFLINE_BOUNDARY_LOG']==str(O/'boundary.jsonl')
sys.path.insert(0,str(W/'src/alicia_flexible_grasp_supervisor/src'))
import numpy as np
from alicia_flexible_grasp.vision.outer_rgb_rectangle import _trig_range
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
release=json.loads((D/'verification/gen37_release_manifest.json').read_text())
before={p:sha(W/p) for p in release['source_sha256']}
assert before==release['source_sha256']
d=json.loads(P.read_text())['cases'][0]['result'];v=d['audit']
A=np.array(v['nominal_axes_base']);n=np.array(v['support_normal_base'])
T=np.array(v['T_base_optical']);K=np.array(v['intrinsics']);h=v['nominal_height_m']
hu=v['empirical_height_uncertainty_m'];off=v['support_offset_m']
lo=np.array(v['nominal_lower']);hi=np.array(v['nominal_upper']);size=hi-lo
source_path=W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/cuboid_rgb_interval.py'
source=source_path.read_text();tree=ast.parse(source)
function=next(x for x in tree.body if isinstance(x,ast.FunctionDef) and x.name=='_joint')
namespace=dict(math=math,np=np,_trig_range=_trig_range)
exec(compile(ast.Module(body=[function],type_ignores=[]),'<captured-joint-numerics>','exec'),namespace)

def point(uv,height):
    rays=np.column_stack(((uv[:,0]-K[2])/K[0],(uv[:,1]-K[3])/K[1],np.ones(len(uv))))@T[:3,:3].T
    return T[:3,3]+rays*((height-off-T[:3,3]@n)/(rays@n))[:,None]

groups=copy.deepcopy(v['groups']);rows=[]
for i,row in enumerate(groups):
    bound=0.;components=np.zeros(2)
    if row['plane']=='top':
        uv=np.array(row['line_pixels']);nominal=point(uv,h)
        for delta_h in (-hu,hu):
            delta=(point(uv,h+delta_h)-nominal)@A[:,:2]
            components=np.maximum(components,np.max(np.abs(delta),axis=0))
            for dx,dy in delta:
                interval=(_trig_range(dx,dy,-math.radians(8.),math.radians(8.))
                          if i%2 else _trig_range(dy,-dx,-math.radians(8.),math.radians(8.)))
                bound=max(bound,max(abs(x) for x in interval))
    row['height_projection_bound_m']=bound
    row['strip_radius_m']=row['maximum_line_residual_m']+row['sampling_bound_m']+bound
    rows.append(dict(side=i,plane=row['plane'],old_euclidean_height_m=v['groups'][i]['height_projection_bound_m'],
        axis_height_components_m=components.tolist(),normal_height_bound_8deg_m=bound))

joint=namespace['_joint'](groups,fixed_theta=0.)
yaw=joint['yaw_uncertainty'];errors=[row['error_m'] for row in joint['position_intervals']]
body=np.array([max(errors[1],errors[3]),max(errors[0],errors[2])])
shift=np.abs((joint['lower']+joint['upper']-lo-hi)/2.)
half_difference=np.abs(joint['upper']-joint['lower']-(hi-lo))/2.
def rotate_bound(x,y):
    return max(_trig_range(x,y,0.,yaw))
fixed=np.array([rotate_bound(body[0],body[1]),rotate_bound(body[1],body[0])])
reference=shift+np.array([rotate_bound(half_difference[0],half_difference[1]),
                        rotate_bound(half_difference[1],half_difference[0])])
E=fixed+reference
narrow=int(np.argmin(size));other=1-narrow
turn=min(yaw,math.atan2(size[other],size[narrow]))
widest=size[narrow]*math.cos(turn)+size[other]*math.sin(turn)
uncertainty=widest-size[narrow]+2.*E[narrow]
required=widest+2.*E[narrow]+.001
result=dict(complete=True,diagnostic_only=True,qualified_model=None,motion_authorized=False,
    production_modified=False,source_count=len(before),source_unchanged=True,
    source_interval_sha256=sha(P),captured_joint_module_sha256=hashlib.sha256(source.encode()).hexdigest(),
    same_nominal_pose_and_fit=True,all_original_responses_retained=True,
    original_height_interval_m=v['original_common_height_interval_m'],
    preserved_complete_height_domain_m=[h-hu,h+hu],
    height_normal_orientation_domain_rad=[-math.radians(8.),math.radians(8.)],
    group_height_bounds=rows,per_face_position_error_m=errors,
    body_axis_position_error_m=body.tolist(),rotated_fixed_axis_error_m=fixed.tolist(),
    reference_center_shift_m=shift.tolist(),reference_half_size_difference_m=half_difference.tolist(),
    reference_axis_allowance_m=reference.tolist(),fixed_axis_error_m=E.tolist(),
    nominal_size_xy_m=size.tolist(),nominal_height_m=h,preserved_height_uncertainty_m=hu,
    common_yaw_rad=yaw,common_yaw_deg=math.degrees(yaw),
    narrow_axis=narrow,narrow_nominal_m=float(size[narrow]),
    narrow_width_uncertainty_m=float(uncertainty),fit_clearance_total_m=.001,
    required_opening_m=float(required),independent_reserve_m=.002,
    required_plus_reserve_m=float(required+.002),physical_opening_m=.05,
    numerical_aperture_with_reserve=bool(required+.002<=.05),
    old_scalar_required_opening_m=float(size[1]*math.cos(d['yaw_uncertainty'])+size[0]*math.sin(d['yaw_uncertainty'])+2*d['edge_uncertainty']+.001),
    formulas=dict(
        normal_height='H_i=max_{pixel,h_endpoint,theta in original +-8deg}|delta_xy dot n_i(theta)| using _trig_range; all heights remain.',
        body_axes='e_x=max(face1.error,face3.error), e_y=max(face0.error,face2.error)',
        fixed_axes='E_x=max_phi(e_x cos(phi)+e_y abs(sin(phi))) + reference_shift_x + max_phi(D_x cos(phi)+D_y abs(sin(phi))); swap x/y for E_y',
        arbitrary_jaw='U=(worst shared-yaw projected size - nominal projection)+2*(E_x*abs(jaw_x)+E_y*abs(jaw_y))+2*hu*abs(jaw_z)+2*commanded_center_jaw_offset',
        collision='same shared-yaw half-size extrema + [E_x,E_y] in model axes; original support/height and measured union remain'),
    limits=['One saved proposed family; no proof of all competing physical edge assignments.',
            'Numerical aperture feasibility is not a contact/path/actual-FK proof or permission to move.',
            'Must extend immutable source geometry and both collision/aperture consumers with exactly the same axis budgets.',
            'Do not replace the measured-source prior or select a favorable subset of heights, pixels or nominal fits.'])
assert {p:sha(W/p) for p in before}==before
boundary=O/'boundary.jsonl';attempts=boundary.read_text().splitlines() if boundary.exists() else []
assert not attempts
result.update(external_boundary_attempts=attempts,script_sha256=sha(__file__))
(O/'result.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
(O/'source_sha256_before.json').write_text(json.dumps(before,indent=2)+'\n')
(O/'source_sha256_after.json').write_text(json.dumps(before,indent=2)+'\n')
print(json.dumps(result))
