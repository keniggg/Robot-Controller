#!/usr/bin/env python3
"""Reconstruct only the current recorded box envelope arithmetic, offline."""
import hashlib
import itertools
import json
import math
from pathlib import Path
import numpy as np

E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
C=W/'.runtime/gen54_release_candidate_r1'
V=E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
OUT=V/'gen54_actual_pipeline_analysis_r1'
ap=E/'grasp_records/20261019T031844.566032Z_249db38c06c54219b4cc46bfc70fab51/keyframes_audits/57648911633e0f25eb53a1a8602d2c84d648004eaaf63240f024bb7ede2736fd.json'
ab=ap.read_bytes();assert hashlib.sha256(ab).hexdigest()==ap.stem
a=json.loads(ab);q=a['remote_diagnostics']['tabletop_geometry']['regular_box_inference'];c=q['cuboid_rgb_interval']
s=np.array(c['nominal_size_xyz_m']);edge=np.array(c['empirical_axis_uncertainty_m']);yaw=c['empirical_yaw_uncertainty_rad'];hu=c['empirical_height_uncertainty_m']
A=np.array(c['nominal_axes_base']);n=np.array(c['support_normal_base']);d=c['support_offset_m']
mid=(np.array(c['nominal_lower'])+np.array(c['nominal_upper']))/2.
center=A[:,:2].dot(mid)+n*(s[2]/2.-d)
turn=np.array([min(yaw,math.atan2(s[1-i],s[i])) for i in (0,1)])
body=np.array([s[i]*math.cos(turn[i])+s[1-i]*math.sin(turn[i]) for i in (0,1)])
family_size=np.r_[body+2*edge,s[2]+hu]
expanded_center=center+n*hu/2.
vertices=np.array([expanded_center+A.dot(np.array(sign)*family_size/2.) for sign in itertools.product((-1.,1.),repeat=3)])
points=np.array(a['replay_geometry']['object_points_base_m'])
union=np.vstack((points,vertices));local=(union-center).dot(A);lo=local.min(0);hi=local.max(0)
reconstructed_center=center+A.dot((lo+hi)/2.);reconstructed_size=hi-lo
actual_center=np.array(a['replay_geometry']['obb_center_base_m']);actual_size=np.array(a['replay_geometry']['obb_size_xyz_m'])
assert np.max(np.abs(reconstructed_size-actual_size))<1e-12
assert np.max(np.abs(reconstructed_center-actual_center))<1e-12
branch=[]
for b in c['visibility_family']['branches']:
    branch.append({k:b[k] for k in ('states','planes','retained','rejection','enclosed_yaw_interval_rad',
        'axis_face_error_m','axis_uncertainty','yaw_uncertainty') if k in b})
groups=[dict({k:g[k] for k in ('side','plane','maximum_line_residual_m','sampling_bound_m',
    'height_projection_bound_m','strip_radius_m')}, line_pixel_count=len(g['line_pixels'])) for g in c['groups']]
sources=[]
for rel in ('src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/regular_box_inference.py',
            'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/cuboid_rgb_interval.py'):
    p=C/rel;b=p.read_bytes();sources.append(dict(path=str(p),sha256=hashlib.sha256(b).hexdigest(),bytes=len(b)))
result=dict(scope='current_recorded_regular_box_uncertainty_envelope_arithmetic',audit_path=str(ap),audit_sha256=ap.stem,
    model_sha256=q['model_sha256'],source_stamp_ns=q['source_stamp_ns'],source_identity=q['source_target_identity'],
    nominal_size_xyz_m=s.tolist(),nominal_center_base_m=center.tolist(),nominal_axes_base=A.tolist(),
    yaw_uncertainty_rad=yaw,yaw_uncertainty_deg=math.degrees(yaw),common_yaw_interval_rad=c['interval']['common_yaw_interval_rad'],
    fixed_axis_single_face_uncertainty_m=edge.tolist(),height_top_allowance_m=hu,
    rotated_nominal_projection_size_xy_m=body.tolist(),yaw_only_extent_increase_xy_m=(body-s[:2]).tolist(),
    two_face_position_extent_increase_xy_m=(2*edge).tolist(),complete_family_enclosure_size_xyz_m=family_size.tolist(),
    measured_point_count=len(points),measured_extra_extent_xyz_m=(actual_size-family_size).tolist(),
    actual_planning_size_xyz_m=actual_size.tolist(),reconstructed_planning_size_xyz_m=reconstructed_size.tolist(),
    max_reconstruction_size_error_m=float(np.max(np.abs(reconstructed_size-actual_size))),
    max_reconstruction_center_error_m=float(np.max(np.abs(reconstructed_center-actual_center))),
    interval=c['interval'],line_strip_budgets=groups,visibility_branches=branch,
    arithmetic_findings=[
        'The 47.562099 mm width is 39.538345 mm nominal plus 2.983891 mm nominal-yaw projection spread plus 5.039864 mm for two opposite face position budgets.',
        'The factor two is the two opposite faces, not a second application of a whole width budget.',
        'Measured points extend the complete family by 0.336587 mm on the long axis only. No measured extra extent is added to the narrow axis or height.',
        'The producer uses the exact union of complete measured points and family vertices. Old fused OBB corners are explicitly not occupancy and are not added again.',
        'Face-position intervals themselves range over the shared yaw interval; their rotation into fixed axes and the nominal-body yaw projection serve different terms of an enclosing bound.',
        'Independent maximum yaw, face bounds and all plane-assignment bounds are then combined into a single OBB. This loses correlations and can be conservative, but the audited arithmetic is not evidence that a margin can be removed.',
        'The two retained visibility explanations are necessary-condition enclosures, not individually certified exact poses. A tighter correlated family needs source-backed interval containment proof, not a nominal-only replacement.'
    ],uncertainty_semantics=c['uncertainty_semantics'],certified_error_bound=c['certified_error_bound'],
    production_sources=sources,source_locations=dict(producer='regular_box_inference.py:403-475',
        family_vertices='regular_box_inference.py:478-513',reference_axis_bounds='cuboid_rgb_interval.py:309-322',
        all_yaw_face_position_intervals='cuboid_rgb_interval.py:543-605',visibility_families='cuboid_rgb_interval.py:392-507'),
    execution_authorized=False,source_modified=False)
p=OUT/'actual_box_envelope_decomposition.json';p.write_text(json.dumps(result,ensure_ascii=False,sort_keys=True,indent=2,allow_nan=False)+'\n')
print(json.dumps(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),
    size_error_m=result['max_reconstruction_size_error_m'],center_error_m=result['max_reconstruction_center_error_m'],
    nominal_mm=(1000*s).tolist(),family_mm=(1000*family_size).tolist(),planning_mm=(1000*actual_size).tolist()),indent=2))
