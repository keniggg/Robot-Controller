"""Freeze third-party offline input and endpoint replay evidence."""
import hashlib
import json
from pathlib import Path

WORK = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
STAGE = WORK / '.runtime/gen14_fk_early_qualified_20261011'
ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_no_skin_20261011T000625Z/gen14_analysis')
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
names = ('original_wall', 'original_work', 'early_wall', 'early_work')
paths = {n: ROOT / ('independent_fk50_' + n + '.json') for n in names}
data = {n: json.loads(p.read_text()) for n, p in paths.items()}
reference = data['early_wall']['input_verification']
keys = ('source_audit_sha256', 'fixture_arrays_sha256', 'checked_array_sha256',
        'exact_support_point_count', 'opening_width_m', 'current_joint_positions',
        'joint_error_bounds_rad', 'source_binding', 'source_geometry', 'model_sha256',
        'view_limits', 'all_endpoint_uncertainties_m', 'gripper')
assert all(all(row['input_verification'][k] == reference[k] for k in keys) for row in data.values())
assert all(row['max_seconds'] == 2. and row['audit']['max_iterations'] == 96 for row in data.values())
assert len(data['original_work']['candidates']) == 0
assert len(data['early_work']['candidates']) == 1
assert len(data['early_wall']['candidates']) == 2
assert len(data['original_wall']['candidates']) == 1
source = STAGE / 'src/alicia_grasp_modes/src/alicia_grasp_modes/reachable_observation_views.py'
assert sha(source) == data['early_wall']['source_module_sha256'] == data['early_work']['source_module_sha256']
assert data['original_wall']['source_module_sha256'] == data['original_work']['source_module_sha256']
for name in names:
    for row in data[name]['independent_candidate_rechecks']:
        assert row['all_original_endpoint_checks_passed'] and all(v['ok'] for v in row['checks'].values())
first = data['early_wall']['candidates'][0]['endpoint_checks']
report = dict(reviewed=True, blocking_findings=[], reviewer='gen6_contact_search',
    scope='Independent exact recorded source verification plus same-opening real SLSQP/endpoint replay; code review is separate.',
    source_sha256={'src/alicia_grasp_modes/src/alicia_grasp_modes/reachable_observation_views.py': sha(source)},
    baseline_source_sha256=data['original_wall']['source_module_sha256'],
    input_properties_compared=list(keys), source_input_properties_equal_across_all_four_runs=True,
    input_verification=reference,
    startup_parameter_note='The original-wall evidence was written before the startup log was located. Its missing-provenance caveat is superseded by exact startup open_position_m=.05/max_inner_gap_m=.05 lines included in early-wall and this review. No earlier evidence was overwritten.',
    comparisons=dict(
        actual_historical_live=reference['original_live_search'],
        current_offline_original_wall=dict(candidate_count=1, elapsed_sec=data['original_wall']['wall_elapsed_sec']),
        current_offline_early_wall=dict(candidate_count=2, elapsed_sec=data['early_wall']['wall_elapsed_sec']),
        controlled_computation_cost=dict(cost_per_real_inequality_call_sec=.002, original_candidates=0,
            early_candidates=1, maximum_budget_sec=2., real_solver_and_all_endpoint_checks=True,
            original_work=data['original_work']['deterministic_work_clock'],
            early_work=data['early_work']['deterministic_work_clock'], performance_benchmark=False)),
    first_early_endpoint_summary=dict(
        predicted_new_missing_side_gain_m=first['missing_side_view_gain']['predicted_new_side_gain_m'],
        minimum_working_volume_image_margin_px=first['working_volume']['minimum_image_edge_distance_px'],
        minimum_arrival_prism_image_margin_px=first['arrival_projection_endpoint']['minimum_image_edge_distance_px'],
        conditional_gripper_support_clearance_m=first['conditional_endpoint_support']['minimum_support_clearance_lower_bound_m'],
        optical_side_evidence_m=first['optical_side_evidence']['observation_projected_side_evidence_m'],
        measured_support_pixels=first['measured_support_projection']['support_pixels'],
        measured_support_quadrants=first['measured_support_projection']['qualifying_quadrants']),
    artifacts={name: dict(path=str(path), sha256=sha(path)) for name, path in paths.items()},
    independent_scripts={str(WORK / '.runtime' / name): sha(WORK / '.runtime' / name) for name in
        ('review_gen14_fk_exact50_replay.py', 'seal_gen14_fk_independent_inputs.py')},
    limitations=[
        'The same exact recorded source arrays and startup-backed 50 mm opening were used; no standalone full per-GEN14 runtime parameter snapshot is claimed.',
        'Original wall-clock replay returned one valid endpoint at this lighter offline load. Zero original candidates is the recorded live outcome and independently reproduced controlled-work outcome, not claimed for every wall-clock run.',
        'All endpoint geometry gates passed independently, but no full-arm MoveIt collision path, continuous visibility/following proof, fresh source authority, motion or arrival measurement was certified.',
        'No live ROS calls, production source modifications, temperature queries or robot commands.'])
out = ROOT / 'fk_exact50_independent_input_endpoint_review.json'
assert not out.exists()
out.write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(dict(path=str(out), sha256=sha(out), reviewed=True, blocking_findings=[])))
