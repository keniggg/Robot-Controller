"""Pure geometry diagnostic on frozen, hash-verified GEN59 receipt objects."""
from pathlib import Path
import gzip
import hashlib
import json
import sys
import time

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
O = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/actual_attempt_r1/geometry_side_review_r1')
sys.path[:0] = [str(W/'src/alicia_flexible_grasp_supervisor/src'), '/opt/ros/noetic/lib/python3/dist-packages']
import numpy as np
from alicia_flexible_grasp.grasp.gripper_geometry import (
    _bilateral_surface_measurement_details, GripperGeometry)
from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity

started = time.monotonic()
initial = json.loads(next((O/'source_audits').glob('9*.json')).read_text())
final = json.loads((O/'source_audits/27ac190088e2c5fb1bbacbd1ead32f26f62f7322218682bdc26ca7edc554f822.json').read_text())
def receipt(digest):
    raw = gzip.decompress((O/'surface_receipts'/f'{digest}.json.gz').read_bytes())
    assert hashlib.sha256(raw).hexdigest() == digest
    return json.loads(raw)
ref = receipt('121642b392f8f8f3eab376ee012b06e2e1f118185175a913e3e8486ac955e394')
snap = receipt('918470c72f73961029dd2802237f7bb863a3144f5b64c0890ef90412282cbd1e')
collision = receipt('7fe7f604b37cf986bef565a14e8f9b13c325dcaacc6a600aa8cd5f7bf1c72a8e')
cfg = collision['config']['gripper_geometry']
finger = GripperGeometry(cfg['max_inner_gap_m'], cfg['width_safety_margin_per_side_m'],
    cfg['finger_box_xyz_m'], cfg['palm_box_xyz_m'], .003)
identity = TargetTrackIdentity(**ref['identity'])
def surface(data):
    return FusedTargetSurface(identity, np.asarray(data['points_base']),
        np.asarray(data['view_indices']), tuple(data['view_stamps_ns']),
        tuple(tuple(v) for v in data['view_sources']))
reference_surface = surface(ref)
snapshot_surface = surface(snap)
original = ref['reference']
initial_surface = FusedTargetSurface(identity, np.asarray(original['points_base']),
    np.zeros(len(original['points_base']), dtype=int), (original['stamp_ns'],))

def measure(s, center, jaw, normal, insertion=None):
    center, jaw, normal = map(lambda x: np.asarray(x, dtype=float), (center,jaw,normal))
    insertion = -normal if insertion is None else np.asarray(insertion)
    counts, lower, upper, contact, sides = _bilateral_surface_measurement_details(
        s, center, jaw, insertion, finger, 12, normal)
    h = (s.points_base-center) @ normal
    rows=[]
    for sign, mask, cmask in zip((-1,1), sides, contact):
        # The production helper returns un-intersected masks on early failure.
        # They must not be reported as a certified common contact band.
        returned_count = int(np.count_nonzero(cmask))
        if lower is None or upper is None:
            cmask = np.zeros(len(s.points_base), dtype=bool)
        bounds=np.quantile(h[mask],(.05,.95)) if np.any(mask) else None
        per=[]
        for v,stamp in enumerate(s.view_stamps_ns):
            representative=s.view_indices==v
            supported=np.asarray([v in sources for sources in s.view_sources])
            per.append(dict(source_stamp_ns=stamp,
                independent_representatives=int(np.count_nonzero(mask & representative)),
                common_representatives=int(np.count_nonzero(cmask & representative)),
                independent_supported_voxels=int(np.count_nonzero(mask & supported)),
                common_supported_voxels=int(np.count_nonzero(cmask & supported))))
        rows.append(dict(sign=sign,independent_count=int(np.count_nonzero(mask)),
            support_bounds_m=None if bounds is None else bounds.tolist(),
            support_span_m=None if bounds is None else float(bounds[1]-bounds[0]),
            common_count=int(np.count_nonzero(cmask)),returned_helper_mask_count=returned_count,
            common_contact_bounds_available=lower is not None and upper is not None,per_source=per,
            independent_xyz_mean_m=np.mean(s.points_base[mask],axis=0).tolist() if np.any(mask) else None))
    a,b=[r['support_bounds_m'] for r in rows]
    support_common=None if a is None or b is None else [max(a[0],b[0]), min(a[1],b[1])]
    return dict(center_base_m=center.tolist(),jaw_axis_base=jaw.tolist(),support_normal_base=normal.tolist(),
        insertion_axis_base=insertion.tolist(),measured_width_m=None if counts is None else counts[4],
        sides=rows,independent_support_interval_intersection_m=support_common,
        common_insertion_bounds_m=None if lower is None else [lower,upper],
        common_insertion_span_m=None if lower is None else upper-lower)

gaps=initial['observe_return']['measured_missing_sides']
comparisons=[]
normal=original['support_normal_base']
for g in gaps:
    kwargs=(g['target_center_base_m'],g['jaw_axis_base'],normal)
    before=measure(initial_surface,*kwargs)
    after=measure(reference_surface,*kwargs)
    for r,name in zip(before['sides'],('negative','positive')):
        assert r['independent_count']==g[name+'_measured_points']
        assert abs(r['support_span_m']-g[name+'_measured_height_m'])<1e-12
    assert before['common_insertion_span_m'] is None
    comparisons.append(dict(angle_label_deg=g['angle_deg'],basis='exact_original_center_normal_and_jaw_in_reference_frame',
        initial=before,fused_after_observation=after))

terminal_rows=[]
geo=final['replay_geometry']
for index,r in enumerate(final['rows']):
    if r.get('candidate_source')!='tabletop_geometry':continue
    if r['source_index'] not in (0,69):continue
    T=np.asarray(r['T_base_tool0'])
    m=measure(snapshot_surface,r['contact_center_base_m'],T[:3,1],geo['support_normal_base'],T[:3,2])
    terminal_rows.append(dict(row_index=index,source_index=r['source_index'],variant_index=r['variant_index'],
        approach_tilt_deg=r['approach_tilt_deg'],basis='actual_proposal_snapshot_coordinates',measurement=m))

T=np.asarray(final['snapshot_transform']['T_base_optical'])
current_camera=T[:3,3]
current_center=np.asarray(geo['obb_center_base_m'])
current_normal=np.asarray(geo['support_normal_base'])
ray=current_camera-current_center
ray/=np.linalg.norm(ray)
actual=dict(snapshot_stamp_ns=final['snapshot_stamp_ns'],T_base_optical=T.tolist(),
    target_center_base_m=current_center.tolist(),support_normal_base=current_normal.tolist(),
    optical_axis_incidence_to_support_normal_deg=float(np.degrees(np.arccos(np.clip(-T[:3,2]@current_normal,-1,1)))),
    target_ray_incidence_to_support_normal_deg=float(np.degrees(np.arccos(np.clip(ray@current_normal,-1,1)))))
# Compare target rays in one frozen original reference basis; apply actual
# snapshot registration to the current camera origin, never substitute FK.
R=np.asarray(ref['snapshot_to_reference'])
cam_ref=R[:3,:3]@current_camera+R[:3,3]
ray_rows=[]
for g in gaps:
    c=np.asarray(g['target_center_base_m']);j=np.asarray(g['jaw_axis_base']);n=np.asarray(normal)
    out=dict(angle_label_deg=g['angle_deg'])
    for name,cam in [('initial',np.asarray(g['initial_camera_origin_base_m'])),('current_registered_to_reference',cam_ref)]:
        v=cam-c; distance=float(np.linalg.norm(v));v/=distance
        signed=float(v@j)
        out[name]=dict(distance_m=distance,ray_incidence_deg=float(np.degrees(np.arccos(np.clip(v@n,-1,1)))),
            signed_jaw_cosine=signed,negative_face_grazing_deg=float(np.degrees(np.arcsin(np.clip(-signed,-1,1)))))
    ray_rows.append(out)

sources=[]
for rel in ['src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/gripper_geometry.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/multiview_surface.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_visual.py']:
    sources.append(dict(path=str(W/rel),sha256=hashlib.sha256((W/rel).read_bytes()).hexdigest()))
result=dict(scope='read_only_frozen_audits_and_receipts_no_bag_no_ROS_no_network_no_motion',
    initial_exact_gap_reproduction=True,original_source_stamp_ns=original['stamp_ns'],
    fused_source_stamps_ns=ref['view_stamps_ns'],same_original_jaw_comparisons=comparisons,
    terminal_proposal_measurements=terminal_rows,actual_snapshot_camera=actual,
    same_reference_actual_camera_ray_comparison=ray_rows,
    source_retention=final['multiview_surface']['voxel_source_evidence'],
    production_contact_profiles=final['remote_diagnostics']['tabletop_geometry']['contact_boundary_profiles'],
    terminal_outcome=final['outcome'],pipeline_funnel=final['pipeline_funnel'],
    collision=dict(binding=collision['binding'],failure=collision['failure'],
        first_tested_sample=collision['first_failed_sample'],scene=collision['scene'],
        contact_geometry_evidence=json.loads(collision['plan_data']['contact_geometry_evidence']),
        model_sha256=collision['model']['sha256']),
    limitations=['Jaw bands use robust spatial extremes and support-normal height; no surface-normal or complete-face reconstruction proves vertical planar face visibility.',
        'Per-source supported voxels are provenance of a representative voxel, not repeated exact independently measured points.',
        'Fixed-reference comparison preserves initial jaw/normal/center and may differ from production current-snapshot proposal geometry.',
        'Collision first sample is terminal endpoint screen, not first continuous curve contact; no execution occurred in this diagnostic.',
        'Image after terminal is not the same frozen snapshot; this diagnostic does not infer image coverage from labels.'],
    source_sha256=sources,probe_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    elapsed_sec=time.monotonic()-started)
(O/'side_band_result.json').write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
for x in comparisons:
    a=x['initial'];b=x['fused_after_observation']
    print(x['angle_label_deg'],'initial spans',[v['support_span_m'] for v in a['sides']],
        'fused spans',[v['support_span_m'] for v in b['sides']],
        'fused common',b['common_insertion_span_m'],'counts',[v['common_count'] for v in b['sides']])
print('actual camera',json.dumps(actual));print('ray',json.dumps(ray_rows))
print('terminal',json.dumps(terminal_rows))
print('result_sha256',hashlib.sha256((O/'side_band_result.json').read_bytes()).hexdigest())
