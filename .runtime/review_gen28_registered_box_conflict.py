"""Read-only source/evidence analysis; write one small report, never read live ROS."""
import hashlib
import json
import os
from pathlib import Path

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
N = M / '实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z'
O = N / 'verification/gen29_registered_box_conflict_review_r2'
assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY and not O.exists()

def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path): return json.loads(Path(path).read_text())

conflict_path = N / 'verification/gen28_attempt1_local_rgb_edges_prototype_r1/measured_conflict_analysis.json'
replay_path = N / 'verification/gen28_attempt1_box_edges_r1/result.json'
candidate_path = conflict_path.parent / 'radius_3.json'
release_path = N / 'verification/gen28_release_manifest.json'
conflict, replay, release = read(conflict_path), read(replay_path), read(release_path)
audit_path = Path(replay['audit_path'])
assert sha(audit_path) == replay['audit_sha256'] == conflict['audit_sha256']
assert sha(candidate_path) == conflict['candidate_report_sha256']
audit = read(audit_path)
multiview = audit['multiview_surface']
scene = multiview['fused_scene_geometry']
candidate = read(candidate_path)['candidates'][conflict['prototype_candidate']]
assert candidate['code'] == 'BOX_MEASURED_GEOMETRY_CONFLICT'
assert conflict['groups'][0]['conflicting_count'] == 0
assert conflict['groups'][1]['conflicting_count'] == len(conflict['worst_points']) == 6
raw_count = scene['current_raw_point_count']
removed = scene['exact_duplicates_removed']
view0 = scene['per_view_representative_point_counts'][0]
tail_indices = [point['index'] - raw_count for point in conflict['worst_points']]
assert raw_count == 993 and removed == 115 and view0 == 598
assert all(0 <= index < 1754 for index in tail_indices)
assert scene['enclosed_point_count'] - raw_count == 1754
paths = [
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/regular_box_inference.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/fused_scene_geometry.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/multiview_surface.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/gripper_geometry.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/contact_evidence.py',
    'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py']
sources = {p: sha(W / p) for p in paths}
assert all(release['source_sha256'][p] == digest for p, digest in sources.items())
result = dict(
    corrects_report=str(N / 'verification/gen29_registered_box_conflict_review/findings.json'),
    corrected_report_sha256=sha(N / 'verification/gen29_registered_box_conflict_review/findings.json'),
    correction='The prior view0 attribution was erroneous: np.lexsort uses the LAST key as primary. Final fused order is x/y/z with view index only a tie-breaker; aggregate counts cannot establish the six point sources. That attribution is withdrawn. The prior report is retained unchanged.',
    scope='Offline review of existing GEN28 measured conflict; no qualifier rerun, point deletion or threshold change.',
    prototype_only=True, qualified_model=False, physical_grasp_verified=False,
    evidence_sha256={str(p): sha(p) for p in (conflict_path, replay_path, candidate_path, audit_path, release_path)},
    reviewed_source_sha256=sources, sources_match_gen28_release=True,
    provenance=dict(
        current_raw_count=raw_count, additional_registered_union_count=1754,
        fused_count=scene['fused_point_count'], exact_union_duplicates_removed=removed,
        representative_counts=scene['per_view_representative_point_counts'],
        all_additional_points_are_strictly_historical=False,
        explanation='Fused surface includes 891 current-view representatives. The extra union tail is not itself an old-frame label.',
        conflicting_point_count=6, conflicting_tail_indices=tail_indices,
        representative_source_of_each_conflicting_point='not established from the retained per-point provenance',
        all_six_representatives_from_view0=None,
        derivation=['_voxel_deduplicate_with_sources sorts final representatives by x/y/z; view_indices is only a tie-breaker.',
                    '_snapshot_multiview_surface applies a rigid inverse without reordering points.',
                    'Exact XYZ union keeps current raw first and preserves retained fused order.',
                    'All six conflict indices are in the additional registered tail; this establishes their array role only, not the original source view.'],
        limits='Exact per-point representative indices, per-voxel source sets and per-point registration residuals are not reconstructed.'),
    metrics=dict(
        model_dimensions_m=conflict['model_dimensions_m'],
        current_raw_conflict_count=0, registered_conflict_count=6,
        maximum_excess_after_existing_model_budget_m=conflict['groups'][1]['maximum_excess_m'],
        model_edge_uncertainty_m=conflict['empirical_edge_uncertainty_m'],
        model_height_uncertainty_m=conflict['empirical_height_uncertainty_m'],
        model_yaw_uncertainty_rad=conflict['empirical_yaw_uncertainty_rad'],
        latest_registration=multiview['latest_registration'],
        measured_collision_enclosure=dict(old_size_xyz_m=scene['old_size_xyz_m'], new_size_xyz_m=scene['new_size_xyz_m'])),
    assessment=[
        'Registration transforms are already applied to measured points. The actual measured collision OBB encloses their resulting positions and the original OBB.',
        'This position enclosure has no explicit registration residual, covariance or bounded transform-error expansion. RMSE acceptance is not a per-point error bound.',
        'Regular-box edge/height/yaw uncertainty comes from current top fit, depth scale, pixel localization, contour extension and angular evidence; registration RMSE is not an input.',
        'The model-consistency check applies the current source model budgets to every geometry.object_points_base point, including the registered union tail. It does not use source-specific registration error.',
        'Coordinate roles are explicit and consistent: fusion is stored in phase-reference coordinates, then inverse-mapped to current snapshot coordinates for this model and scene.',
        'Evidence roles are flattened only at the consistency check: the geometry point array has no per-point uncertainty or source field, although the audit binds the union and provenance summaries.',
        'If a model were qualified, its collision enclosure would union the measured OBB and inferred edge/yaw/height envelope without removing points. This prototype was rejected before that branch.',
        'Rigid re-expression of a qualified model preserves its empirical budgets and hashes the registration transform; it does not estimate or add transform uncertainty.'
    ],
    minimum_next_evidence=[
        'Bind the six additional registered points to their original representative view indices, raw-frame pixels/depth and retained view source sets; distinguish true edge returns, segmentation spill and missing depth without deleting points.',
        'Record their nearest-correspondence residuals, normals/support-plane residuals and local consistency before and after the accepted transform, alongside all-inlier RMSE.',
        'Check whether the same edge discrepancy is repeated across independent original sources; shared voxels or repeated same-station stability alone cannot reduce systematic error.',
        'Keep current-model contradiction and uncertain historical placement as separately reported facts. Any future error model must propagate to consistency, jaw opening, collision and signed downstream evidence together.',
        'Do not subtract global RMSE from the 2.591 mm excess, treat the 6.343 mm correction as error, drop six points, or raise the 1 mm excess gate based on this report.'
    ],
    unresolved='Existing evidence cannot identify physical target change, raw-depth edge error or registration residual as the cause of these six conflicts. No production change is justified by classification alone.',
    no_live_ros=True, no_hardware_access=True, no_temperature_queries=True,
    script_sha256=sha(__file__))
O.mkdir()
path = O / 'findings.json'
path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
print(json.dumps(dict(path=str(path), sha256=sha(path), six_point_original_view_attribution='unresolved'), ensure_ascii=False))
