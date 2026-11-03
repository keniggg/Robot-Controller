"""Summarize existing finite offline results; never reads a bag or ROS."""
import os
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
from pathlib import Path
import json,hashlib,math,gzip
import numpy as np
import cv2
cv2.setNumThreads(1)
W=Path(__file__).resolve().parents[1]
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1')
O=N/'actual_attempt_r1_geometry/cuboid_exact_frame_r1'
assert Path('/media/zhuyupei/SanStand').is_mount()
sha=lambda b:hashlib.sha256(b).hexdigest()
save=lambda name,data:(O/name).write_text(json.dumps(data,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
d=json.loads((N/'actual_attempt_r1_geometry/terminal_gate_audit.json').read_text())
q=json.loads((O/'qualification_replay.json').read_text())
c=json.loads((O/'failure_locals.json').read_text())
s=json.loads((O/'measured_side_source_contributions.json').read_text())
expected=d['remote_diagnostics']['tabletop_geometry']['regular_box_inference']
diffs=[]
def diff(a,b,p=''):
    if isinstance(a,dict) and isinstance(b,dict):
        for k in sorted(a.keys()&b.keys()):diff(a[k],b[k],p+'/'+k)
    elif isinstance(a,list) and isinstance(b,list):
        assert len(a)==len(b)
        for i,(x,y) in enumerate(zip(a,b)):diff(x,y,p+'/'+str(i))
    elif a!=b:
        diffs.append(dict(path=p,live=a,replay=b,delta=b-a if isinstance(a,(int,float)) and isinstance(b,(int,float)) else None))
diff(expected,q['audit'])
save('live_replay_numeric_differences.json',dict(bitwise_all_fields_equal=False,differences=diffs,reason='Exact input arrays; original live numerical execution environment was not recreated. Tiny SVD/minAreaRect/refit differences are retained, not rounded away.',unchanged_codes=True,unchanged_mask_coverages=True))

arrays=np.load(O/'exact_rgbd_mask.npz',allow_pickle=False); mask=arrays['mask']>0
hull=np.asarray(c['model']['hull_pixels'])
poly=np.zeros(mask.shape,np.uint8);cv2.fillConvexPoly(poly,np.rint(hull).astype(np.int32),1)
counts=dict(mask_pixels=int(mask.sum()),fitted_raster_pixels=int(poly.sum()),intersection_pixels=int(np.count_nonzero(mask&(poly>0))),mask_pixels_outside_fitted_raster=int(np.count_nonzero(mask&(poly==0))))
counts['mask_recall_by_fitted_raster']=counts['intersection_pixels']/counts['mask_pixels']
counts['policy']='Diagnostic round-to-nearest filled hull, different from production inward-eroded coverage; no qualification or motion authority.'
seed_A=np.asarray(c['A']);nom_A=np.asarray(c['nominal_A']);seedlo=np.asarray(c['lo']);seedhi=np.asarray(c['hi']);nomlo=np.asarray(c['nominal_lo']);nomhi=np.asarray(c['nominal_hi'])
base_shift=nom_A[:,:2]@((nomlo+nomhi)/2)-seed_A[:,:2]@((seedlo+seedhi)/2)
ref=d['multiview_surface']['fused_scene_geometry']['surface_replay']['references']['snapshot']
blob=gzip.decompress((Path(ref['store_root'])/ref['relative_path']).read_bytes());assert sha(blob)==ref['object_sha256'];f=json.loads(blob)
points=np.asarray(f['points_base']);center=np.asarray(d['replay_geometry']['obb_center_base_m']);normal=np.asarray(d['replay_geometry']['support_normal_base']);z=(points-center)@normal
for row in s['rows']:
    row['fourth_return_masks_semantics']='Returned fourth field of production helper. On failure with no common support it can still contain independent masks; only non-null common_insertion_bounds_m proves an actual common band.'
save('measured_side_source_contributions.json',s)
groups=[]
for before,after in zip(c['seed_groups'],c['groups']):
    groups.append(dict(side=after['side'],plane=after['plane'],seed_count=len(before['line_pixels']),union_count=len(after['line_pixels']),seed_max_line_residual_m=before['maximum_line_residual_m'],union_max_line_residual_m=after['maximum_line_residual_m'],sampling_bound_m=after['sampling_bound_m'],height_projection_bound_m=after['height_projection_bound_m'],strip_radius_m=after['strip_radius_m']))
summary=dict(status='real_attempt_remains_failed; diagnostic_only',source_stamp_ns=d['snapshot_stamp_ns'],request_id=d['request_id'],generation=d['generation'],
    replay=dict(exact_pixel_arrays_match_original=True,all_camera_metadata_rows_match=True,recorded_camera_parameter_image_time_association='none',production_functions_unmodified=True,thresholds_unmodified=True,code=q['code'],reason=q['reason'],elapsed_sec=q['elapsed_sec'],numerical_differences_retained=True),
    cuboid=dict(failure='edge_uncertainty_only_exceeds_precision_gate',edge_uncertainty_m=c['edge'],edge_limit_m=.01,yaw_uncertainty_deg=math.degrees(c['yaw']),yaw_limit_deg=12.,height_uncertainty_m=c['hu'],height_m=c['top_height'],height_gate_required_m=2*c['hu']+.002,
        axis_uncertainty_m=c['axis'],reference_joint_components=c['result'],groups=groups,seed_size_xy_m=(seedhi-seedlo).tolist(),refitted_size_xy_m=(nomhi-nomlo).tolist(),seed_to_refitted_center_shift_base_m=base_shift.tolist(),fused_obb_size_xyz_m=d['replay_geometry']['obb_size_xyz_m'],
        production_top_coverage=q['audit']['cuboid_rgb_interval']['top_mask_coverage'],production_silhouette_coverage=q['audit']['cuboid_rgb_interval']['silhouette_mask_coverage'],opposite_direction_mask_coverage_diagnostic=counts,
        visual_interpretation='In exact_frame_diagnostic.png, side 0 seed responses lie near the upper external contour while many refit responses lie on the upper interior printed logo border. This visual interpretation is supported by the saved exact pixel coordinates, not a claim of calibrated ground-truth dimensions.',
        implication='High polygon-inside-mask coverage is insufficient to show the whole object is enclosed. Preserve seed evidence and precision gates; do not relax a 10 mm limit merely because excess is small.'),
    surface=dict(initial_data_retained=True,view_stamps_ns=f['view_stamps_ns'],initial_and_reached_sources_are_not_three_distinct_viewpoints=True,voxel_source_evidence=d['multiview_surface']['voxel_source_evidence'],measured_side_rows=s['rows'],
        interpretation='The added view contributes points near the deficient negative jaw-side band, but these may be top-face or edge points and do not certify visibility of a vertical face. There is still insufficient common support-normal height with the positive side. Camera pose/projection gain alone is not measured bilateral contact proof.',
        evidence_not_proven='This replay does not establish that an alternative pose will expose the missing surface or execute safely; no candidate, trajectory, remote prediction, or motion is authorized.'),
    local_candidate_population=dict(tabletop_proposals=20,tabletop_reach_rejections=16,remaining_band_directions=4,tabletop_materialized=0,remote_returned=12,locally_valid=0,moveit_checked=0,rejection_counts_nonexclusive=True),
    limitations=['Original live floating point execution environment not replicated bitwise; all raw pixel hashes match and differences are recorded.', 'Per-source counts describe support of fused voxels, not independent exact point measurements.', 'Initial/source masks are not manually labeled object ground truth.', 'No production edits, ROS initialization, network prediction, temperature inspection, or motion.'])
save('summary.json',summary)
report='''# GEN58 exact current-frame cuboid and contact evidence

The real attempt remains failed. This diagnostic replays the unchanged regular-box qualification on the exact final raw RGB/depth/mask; all three raw array SHA256 values match the terminal audit. The camera K/scale/dimensions match every one of the 40 recorded parameter snapshots (parameter content only, not image-clock proof). The original failure is reproduced, with small floating-point differences retained in a separate file.

The cuboid rejects only its edge uncertainty: 10.26343 mm exceeds the original 10 mm limit. Yaw uncertainty is 6.27449 degrees and height uncertainty is 2.02908 mm; both other precision predicates pass. Side 0 combines 54 seed responses near the external upper contour with the refitted response set near an interior printed border. Keeping that complete evidence raises its maximum line residual from 1.13714 to 7.69702 mm. The full intervals and raw response coordinates are in failure_locals.json. The fitted width is 16.72152 mm, versus a 24.87614 mm seed footprint and a 28.59660 mm fused enclosure; these are different estimates, not ground-truth dimensions. The production 99.6% coverage asks whether the proposed polygon is inside the mask. It does not establish that the polygon encloses the entire object. Do not remove seed evidence or relax a threshold to accept this fit.

The initial observation is used: it supports 1387/1864 fused voxels and is the sole source of 1053. Two later timestamps come from the reached viewing station and do not certify two further independent viewpoints. At the nominal narrow jaw direction 51.838 degrees, independent negative/positive sides contain 104/221 points, but their robust support-height ranges overlap by only 0.04754 mm. Restricting to the common height leaves 0/1 points. Initial evidence supports 71 negative-side and 207 positive-side voxels; the final view supports 35/11. Thus the new view adds points near the opposite jaw-side band, but these may be top-face or edge samples; the side-band helper does not classify a visible vertical face. It still does not prove a shared contact patch. Four other reachable directions yield common bands of 0.341, 0.347, 1.180 and 0.722 mm; the original materialization probes all fail the unchanged 2 mm continuous-overlap requirement.

Artifacts: summary.json, failure_locals.json, measured_side_source_contributions.json, qualification_replay.json, live_replay_numeric_differences.json, recorded_camera_binding.json, exact_frame_diagnostic.png, exact_rgb.png, exact_mask.png, extraction.json, provenance.json and artifact_manifest.json. No new execution or motion conclusion follows from this offline result.
'''
(O/'REPORT.md').write_text(report)
for src in ['extract_gen58_terminal_rgbd_20261022.py','diagnose_gen58_cuboid_surface_20261022.py','summarize_gen58_cuboid_surface_20261022.py']:
    (O/src).write_bytes((W/'.runtime'/src).read_bytes())
save('artifact_manifest.json',dict(files=[dict(path=str(p),size_bytes=p.stat().st_size,sha256=sha(p.read_bytes())) for p in sorted(O.iterdir()) if p.is_file() and p.name!='artifact_manifest.json'],original_audit_path=str(N/'actual_attempt_r1_geometry/terminal_gate_audit.json'),original_audit_sha256=sha((N/'actual_attempt_r1_geometry/terminal_gate_audit.json').read_bytes())))
print(json.dumps(dict(summary=str(O/'summary.json'),summary_sha256=sha((O/'summary.json').read_bytes()),mask_coverage_diagnostic=counts,base_shift=base_shift.tolist()),indent=2))
