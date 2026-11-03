"""GEN13 first observation contact rejection audit; no live ROS operations."""
from pathlib import Path
from collections import Counter
import hashlib
import json
import rosbag

WORK = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW = EXT / 'ros_log/full_latest_restart_20261010T002347Z'
REC = EXT / 'grasp_records/20261010T050849.339487Z_fa4a10a8358a4fa2b80214f1f626b5c7'
assert Path('/media/zhuyupei/SanStand').is_mount()
manifest = json.loads((REC/'manifest.json').read_text())
assert manifest['state'] == 'complete' and manifest['recorder_closed']
source_manifest = json.loads((REC/'source_manifest.json').read_text())
audit_files = {
    'contact_request_33': REC/'keyframes_audits/b4cf6e33d853a63d085600f12c67476ddb259780476907df9806ce954c845475.json',
    'supplemental_request_34': REC/'keyframes_audits/d166a7c80ca79ac65d9ffd22ce4327ead73c18ebe0191586f53b6f18b93bbe10.json',
}
contact, supplemental = [json.loads(p.read_text()) for p in audit_files.values()]
assert contact['request_id'] == 33 and supplemental['request_id'] == 34
tabletop = contact['remote_diagnostics']['tabletop_geometry']
metrics, rejection_samples, timeline = {}, [], []
with rosbag.Bag(str(REC/'keyframes.bag')) as bag:
    for _, message, _stamp in bag.read_messages(topics=['/rosout_agg']):
        if not 1791608976 <= message.header.stamp.to_sec() <= 1791609000:
            continue
        text = message.msg
        if text.startswith('remote 6D pipeline metrics: '):
            value = json.loads(text.partition(': ')[2])
            if value.get('request_id') in (33, 34) and value.get('event') == 'request_completed':
                metrics[value['request_id']] = value
        elif text.startswith('remote 6D GraspNet candidate rejection samples: '):
            rejection_samples = json.loads(text.partition(': ')[2])
        elif any(part in text for part in ('6D planning phase signal: near_field',
                'Observe-return measurement acknowledged:', 'remote 6D analytical gripper rejection:')):
            timeline.append({'source_ns': message.header.stamp.to_nsec(), 'node': message.name, 'text': text})
assert len(rejection_samples) == 8 and set(metrics) == {33, 34}
counts = Counter(contact['pipeline_funnel']['rejection_counts'])
assert tabletop['materialized_count'] == 0 and tabletop['failure_code'] == 'BILATERAL_SURFACE_EVIDENCE_MISSING'
# _evaluate_local_candidates adds one aggregate tabletop failure after the
# twelve per-learned-candidate failures; it is not a thirteenth candidate.
counts[tabletop['failure_code']] -= 1
assert dict(counts) == {'BILATERAL_SURFACE_EVIDENCE_MISSING': 5,
    'GRASPNET_STAGE_PROFILE_UNAVAILABLE': 6, 'GRIPPER_SWEEP_COLLISION': 1}
unlogged_counts = counts - Counter(row['code'] for row in rejection_samples)
assert sum(counts.values()) == 12 and sum(unlogged_counts.values()) == 4

source_paths = [
    'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/gripper_geometry.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/tabletop_geometry_candidates.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_visual.py',
]
source_checks = {}
for relative in source_paths:
    digest = hashlib.sha256((WORK/relative).read_bytes()).hexdigest()
    recorded = source_manifest['source_sha256'].get(relative)
    if recorded is not None:
        assert digest == recorded, relative
    source_checks[relative] = dict(sha256=digest, in_record_manifest=recorded is not None,
        matches_record_manifest=(digest == recorded) if recorded else None)

report = dict(record=str(REC), experiment_result='failure',
    final_failure_reason=manifest['failure_reason'],
    manifest_sha256=hashlib.sha256((REC/'manifest.json').read_bytes()).hexdigest(),
    audit_sources={name: {'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
                   for name, path in audit_files.items()},
    source_checks=source_checks,
    first_contact_request=dict(request_id=33, snapshot_stamp_ns=contact['snapshot_stamp_ns'],
        planning_origin=contact['planning_origin'], pipeline_funnel=contact['pipeline_funnel'],
        metrics=metrics[33], moveit_selection=contact['moveit_selection']),
    learned_candidates=dict(input_count=12, normalized_success_count=0,
        actual_per_candidate_rejection_counts=dict(counts),
        sampled_exact_rejections=rejection_samples,
        remaining_indices=[8, 9, 10, 11], remaining_aggregate_rejection_counts=dict(unlogged_counts),
        exact_remaining_index_to_reason_available=False,
        audit_row_count=24, audit_row_semantics='two diagnostic wrist variants per raw learned candidate; not twenty-four independently normalized candidates',
        funnel_extra_aggregate_failure={'source': 'tabletop_geometry', 'code': tabletop['failure_code'], 'count': 1}),
    tabletop_candidates=dict(proposal_count=tabletop['proposal_count'],
        materialized_count=tabletop['materialized_count'],
        proposal_reach_rejections=tabletop['proposal_reach_rejections'],
        surface_evidence_rejections=tabletop['surface_evidence_rejections'],
        stage_variant_rejection_counts=tabletop['rejection_counts'],
        rejection_count_semantics='16 proposal-level reach rejections plus 30 bilateral and 6 contact-overlap variant failures; not 52 independent physical grasp poses',
        contact_boundary_profiles=tabletop['contact_boundary_profiles'],
        adaptive_stage_profiles=tabletop['adaptive_stage_profiles'],
        required_contact_patch_overlap_m=tabletop['required_contact_patch_overlap_m'],
        contact_overlap_requirement_source=tabletop['contact_overlap_requirement_source']),
    measured_object=dict(user_narrow_width_m=.04,
        current_obb_size_xyz_m=contact['replay_geometry']['obb_size_xyz_m'],
        current_object_points=contact['replay_geometry']['object_points_count'],
        current_support_inlier_ratio=contact['replay_geometry']['support_inlier_ratio'],
        narrowest_aperture_fitting_gap=supplemental['observe_return']['measured_missing_sides'][0],
        other_aperture_fitting_gaps=supplemental['observe_return']['measured_missing_sides'][1:]),
    fusion=contact['multiview_surface'], previous_gain=supplemental['observe_return']['previous_gain'],
    supplemental_request=dict(request_id=34, plan_id=supplemental['plan_id'],
        planning_origin=supplemental['planning_origin'], metrics=metrics[34],
        contact_candidates_requested=supplemental['remote_diagnostics']['contact_candidates_requested'],
        trigger_evidence=supplemental['observe_return']['trigger_evidence']),
    focused_timeline=timeline,
    conclusions=[
        'The completed first observation did reach near-field contact evaluation. Request 33 finished in 1.2523 s end-to-end, with zero locally valid candidates and zero MoveIt contact checks. Request 34 spent 15.0556 s searching the next observation view; it was not fifteen seconds of failed contact path planning.',
        'All twelve learned candidates were rejected: five for missing measured bilateral support, six for an unavailable physically allowed stage profile, one for asymmetric single-side jaw travel. The published six bilateral failures include an additional aggregate zero-tabletop-candidate failure; sum thirteen is not thirteen learned candidates.',
        'There were eighteen generated tabletop proposals. Sixteen were rejected by an OBB jaw-line half-travel check before expensive materialization. Two reached tilt/materialization analysis: their recorded common contact-height intervals were about 1.4544 mm and 1.2271 mm wide, with maximum recorded CAD overlaps 1.4544 mm and 1.1772 mm, all below the unchanged 2 mm uncertainty-derived overlap requirement. Across their variants the audit counts thirty bilateral-evidence failures and six contact-patch misses; none materialized.',
        'The 40 mm user-measured narrow width is consistent with the current 41.0539 mm OBB narrow dimension. The narrowest observation gap requires 42.0539 mm including 1 mm total fit clearance, below the 50 mm gripper opening, and 21.0269 mm each side below the 25 mm half-travel limit. This direction was retained for missing contact evidence, not rejected as too wide.',
        'The isolated learned source-index-five finger_reach rejection 31.918/9.552 mm denotes distance along the jaw closing axis from a biased candidate midpoint to each OBB side plus clearance. It is not the object narrow-edge dimension and not finger insertion length. A total width that fits can still exceed one side of the centered 50 mm opening.',
        'New-view data were registered and used by the exact contact path: three retained source frames (initial frame plus two frames at the first supplemental station), 1550 fused voxels, latest registration overlap 0.80723 and RMSE 0.8083 mm. Both proposal generation and learned contact evaluation call the snapshot-bound fused surface, and the request-34 missing-side evidence names all three stamps. There is no recorded evidence that initial-view-only data replaced the new observations.',
        'The first observation passed its gain test with sixteen new occupied voxels beyond the 3 mm novelty distance, but this is not bilateral contact certification. At the narrowest axis the weak negative side has 109 points over 1.8149 mm height and the positive side 285 over 11.5545 mm; the actual common-height evaluator still fails. Point count alone therefore does not establish a usable pair of opposing contact patches.',
        'The support-normal common-height gate and CAD overlap gate are distinct. The weak-side 1.8149 mm diagnostic is not itself the two-millimeter gate: bilateral evaluation first requires at least twelve points on each actual jaw band with shared support-normal and insertion-height intervals. The separate two-millimeter requirement applies to the final continuous target/CAD contact overlap.',
        'Stored view indices 0,1,2 here are three source-frame provenance entries, not proof of three independent camera stations. The fusion audit explicitly denies independent-viewpoint and uncertainty-reduction certification.',
        'This evidence supports investigating how the first view samples the missing negative contact face and why the valid bilateral/CAD common intervals remain thin. It does not justify simply lowering contact gates, treating inference widths as real object width, or declaring the object ungraspable. The eventual second-station measurement ACK timeout is a later independent failure.',
    ],
    limitations=[
        'Only the first eight individual learned rejection samples are persisted. The remaining four comprise one bilateral and three stage-profile failures by exact aggregate subtraction; this report does not invent their individual index-to-reason mapping.',
        'The saved audit contains fused statistics and lineage but not every final fused point with provenance. It demonstrates registered new-frame use, not an independent proof that all potentially useful depth points survived segmentation/registration/footprint filtering.',
        'The missing-side report exposes counts and separate side spans, not all per-point masks/quantile bounds. It cannot uniquely separate no shared support-height interval from no surviving shared insertion-height interval for every failed direction.',
        'No source, threshold, controller state or physical command was changed. No temperature data were read.',
    ], physical_actions=False, production_edited=False, thresholds_changed=False)
out = NEW/'gen13_handoff_analysis/first_view_contact_rejections.json'
out.parent.mkdir(exist_ok=True)
assert not out.exists()
out.write_text(json.dumps(report, ensure_ascii=False, sort_keys=True, indent=2)+'\n')
print(json.dumps(dict(path=str(out),sha256=hashlib.sha256(out.read_bytes()).hexdigest(),
    learned_rejection_counts=dict(counts), tabletop_proposals=tabletop['proposal_count'],
    source_checks=source_checks),ensure_ascii=False))
