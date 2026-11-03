"""Read-only scope and live-memo review evidence; never runs tests or ROS."""
from pathlib import Path
import ast
import hashlib
import json
import xml.etree.ElementTree as ET

W = Path(__file__).resolve().parent.parent
C = W / '.runtime/gen14_arrival_repair_20261013'
N = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
V = N / 'verification'
OUT = V / 'gen14_scope_live_memo_readonly_review_r1'

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

assert N.is_dir() and not OUT.exists()
prior_path = V / 'gen14_reporting_integrated_install/result.json'
prior = json.loads(prior_path.read_text())
old_path = V / 'gen14_measured_face_author_r3/result.json'
old = json.loads(old_path.read_text())
additions = ['test_measured_face_precompute_independent.py',
             'test_reference_texture_grouping.py', 'test_measurement_binding_memo.py',
             'test_measurement_live_binding_independent.py']
focused = sorted(set(old['test_scope']) | {
    'src/alicia_grasp_modes/tests/' + name for name in additions})
integrated = sorted(set(prior['test_scope']) | set(focused))
assert len(focused) == 35 and len(integrated) == 161
assert all((C / p).is_file() for p in focused)

changed = []
for path in sorted(C.rglob('*.py')):
    relative = str(path.relative_to(C))
    target = W / relative
    if not target.is_file() or sha(target) != sha(path):
        changed.append(dict(path=relative, candidate_sha256=sha(path),
                            base_sha256=sha(target) if target.is_file() else None))
assert len(changed) == 16
assert sum('/tests/' not in row['path'] for row in changed) == 6

evidence = []
nodeids = set()
duplicates = []
for name in ['gen14_measured_face_author_r3', 'gen14_precompute_independent_r2',
             'gen14_reference_texture_grouping_r1', 'gen14_live_binding_author_r1',
             'gen14_live_binding_independent_r2']:
    result_path = V / name / 'result.json'
    result = json.loads(result_path.read_text())
    xml = V / name / 'pytest.xml'
    assert result['passed'] is True
    if 'pytest_xml_sha256' in result:
        assert sha(xml) == result['pytest_xml_sha256']
    current = []
    for case in ET.parse(xml).getroot().iter('testcase'):
        # Tests copied from the independent work area preserve exact bytes.
        module = case.attrib['classname'].rsplit('.', 1)[-1]
        key = module + '::' + case.attrib['name']
        if key in nodeids:
            duplicates.append(key)
        nodeids.add(key)
        current.append(key)
    evidence.append(dict(path=str(result_path), sha256=sha(result_path),
                         pytest_xml_sha256=sha(xml), summary=result['summary'],
                         xml_bound_by_original_result='pytest_xml_sha256' in result,
                         historical_test_count=len(current),
                         current_six_production_match={row['path']:
                             result.get('source_sha256', {}).get(str(C / row['path']))
                             == row['candidate_sha256'] for row in changed
                             if '/tests/' not in row['path']}))
assert len(nodeids) == 1069 and len(duplicates) == 2
for name in ['test_measured_face_precompute_independent.py',
             'test_measurement_live_binding_independent.py']:
    assert sha(C / 'src/alicia_grasp_modes/tests' / name) == sha(
        W / '.runtime/gen14_measured_face_independent_20261013' / name)

unknown = C / 'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py'
frozen = W / '.runtime/gen14_profile_frozen_r2_20261013/src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py'
trees = [ast.parse(path.read_text()) for path in (frozen, unknown)]
def function(tree, name):
    return next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)

unchanged = ['_rgb_interest_joint_cb', '_rgb_bracketed_feedback',
             '_process_observation_measurement', '_observation_measurement_request_cb',
             '_prepare_locked_rgb_hold_reference']
assert all(ast.dump(function(trees[0], name), include_attributes=False) ==
           ast.dump(function(trees[1], name), include_attributes=False) for name in unchanged)

support = ['src/alicia_grasp_modes/src/alicia_grasp_modes/tabletop.py',
           'src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_observation_source.py',
           'src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_observation_handoff.py',
           'src/alicia_grasp_modes/src/alicia_grasp_modes/selection.py',
           'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_view_gateway.py']
report = dict(
    schema=1, kind='gen14_scope_live_memo_independent_readonly_review',
    passed=False, deployable=False, review_status='scoped_static_review_no_blocker_found_release_acceptance_pending',
    no_tests_executed=True, no_ros_used=True, no_motion_commands_sent=True,
    no_production_changes=True, no_temperature_queries=True,
    candidate_root=str(C), candidate_files=changed,
    pending_manifest=dict(path=str(C / 'delivery_manifest.pending.json'),
                          sha256=sha(C / 'delivery_manifest.pending.json'),
                          stale=True, expected_final_file_count=16),
    prior_install=dict(path=str(prior_path), sha256=sha(prior_path),
                       source_count=len(prior['source_sha256']), test_count=len(prior['test_scope'])),
    focused_scope=focused, focused_file_count=len(focused), integrated_scope=integrated,
    integrated_file_count=len(integrated), historical_deduplicated_test_count=1069,
    historical_duplicate_nodeids=duplicates, historical_evidence=evidence,
    current_combined_regression_passed=False, three_owner_replay_acceptance_not_reviewed=True,
    fingerprint_fields=['mask', 'position_camera', 'position_base', 'extent_m',
                        'support_footprint_base', 'bbox', 'center_uv', 'depth_m', 'quality',
                        'metrics.support_plane_base', 'metrics.valid_points'],
    scoped_findings=[
        'ExactValueMemo only caches deterministic original digest/fingerprint/camera-contract computations; cache hits require complete recursive values, Python types, array shape/dtype and raw-byte equality, not identity or TTL.',
        'Current Config has 17 flat scalar dataclass fields; fields/getattr to dict preserves the current asdict payload. Nested/custom values never gain cache-hit authority.',
        'Each measurement checkpoint retains direct owner/loss/plane/generation/selection/SDK epoch, GUI manual flag, original ROS deadline and monotonic deadline checks.',
        'Each source-context call still reads current cached ROS parameters and performs both source-stamp and latest tool-camera TF lookups before any memo hit.',
        'Every source-context nested parameter is part of the exact memo input. Context results are returned as owned copies, so callers cannot mutate the private memo result.',
        'Candidate memo inputs cover exactly the eleven fields of the old fingerprint, and misses call the unchanged original fingerprint on an owned Candidate copy.',
        'Unsupported/custom and nonfinite scalar values always execute the original pure validator. Miss computation rechecks the input snapshot before publishing a new memo entry.',
        'Five authority-sensitive callback/processing functions are AST-identical to the frozen pre-memo candidate; original checkpoint placement, source age gates, SDK out-and-back invalidation and final input lock are retained.',
        'Historical 929 tests predate four production performance changes. The union of previous and new suites is 35 files/1069 unique prior JUnit cases, not current combined acceptance.',
        'The old author runner omits arrival_train_only_proposal.py from its source binding; use run_gen14_train_regression_20261013.py, which binds all six production files, for any next combined run.'
    ],
    ast_unchanged_authority_functions=unchanged,
    reviewed_support_source_sha256={p: sha(W / p) for p in support},
    runner=dict(path=str(W / '.runtime/run_gen14_train_regression_20261013.py'),
                sha256=sha(W / '.runtime/run_gen14_train_regression_20261013.py')),
    limitations=['Static review is confined to memoization and release coverage; it does not assert real-time acceptance, producer READY handoff, deployment, planning feasibility or physical grasp success.',
                 'Full actual baseline source rehash deferred to release/install checker to avoid concurrent CPU work during timed replay.'])
OUT.mkdir()
(OUT / 'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
print(json.dumps(dict(path=str(OUT / 'result.json'), sha256=sha(OUT / 'result.json'),
    changed_files=len(changed), focused_files=len(focused), integrated_files=len(integrated),
    expected_unique_tests=len(nodeids), deployable=False)))
