"""Seal the independently reviewed five-file GEN9 patch after combined tests."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json

WORK = Path(__file__).resolve().parents[1]
NEW = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
STAGE = WORK / '.runtime/gen9_silhouette_coverage_staging_20261010'
assert Path('/media/zhuyupei/SanStand').is_mount()
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
read = lambda p: json.loads(p.read_text())
delivery = read(STAGE / 'delivery.json')
combined_path = NEW / 'verification/gen9_silhouette_coverage_combined_r2/result.json'
combined = read(combined_path)
assert combined['passed'] is True and combined['source_unchanged'] is True
assert combined['baseline147']['matched_before'] and combined['baseline147']['matched_after']
files = {row['path']: row['sha256'] for row in delivery['files']}
assert len(files) == 5 and files == combined['source_sha256']
assert all(sha(STAGE / p) == digest for p, digest in files.items())
expected_production = {
    'src/alicia_grasp_modes/src/alicia_grasp_modes/silhouette_observation_order.py': 'f7536c68ba3eaccdef28469a0e32ba5ac12d5beed1587c9682098455c731603e',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/silhouette_observation_planning.py': '39516ec401fad0344ae927bb8e653ccd697fdc96fc2639297e1f8acf8a33b31a',
    'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py': 'c5f867747edc4f25dc03e07d88e79ed662c68ec30d20109e43d0e78d525798aa',
}
assert all(files[p] == digest for p, digest in expected_production.items())
targeted_path = NEW / 'verification/gen9_silhouette_coverage_targeted_r4/result.json'
targeted = read(targeted_path)
assert targeted['passed'] is True
assert all(targeted['files'][p] == digest for p, digest in files.items()
    if not p.endswith('silhouette_observation_planning.py'))
assessment_path = NEW / 'gen9_handoff_analysis/observation_coverage/recorded_candidate_assessment.json'
report = dict(
    reviewer='/root/gen9_review', reviewed_at_utc=datetime.now(timezone.utc).isoformat(),
    baseline=147, stage=str(STAGE), passed=True,
    disposition='no_blocking_findings_in_frozen_patch', findings=[],
    scope='GEN9 diagnostic silhouette observation candidate scheduling before the first fully qualified path; three production files, one regression file and one exact recorded fixture.',
    files=[dict(path=p, sha256=digest) for p, digest in sorted(files.items())],
    production_edited_by_reviewer=False, deployed_by_reviewer=False, physical_actions=False,
    evidence=[dict(kind=kind, path=str(path), sha256=sha(path), **details)
        for kind, path, details in (
            ('recorded_candidate_assessment', assessment_path, {}),
            ('pre_singleton_compatibility_fix_targeted_regression', targeted_path, dict(tests=targeted['tests'])),
            ('combined_regression', combined_path, dict(tests=combined['unique_tests'])),
        )],
    independently_executed_checks=dict(
        sealed_bag_static_transform='tool0 -> camera_link through tf2 BufferCore, with no ROS service or live TF access',
        tool_camera_sha256='e9dfdb45ddebc391221c8dfdb052175092acc7e1ca95b75e665481a222b14732',
        exact_recorded_pose_count=47,
        coverage_is_complete_permutation=True,
        first_five_original_check_indices=[17, 24, 0, 10, 1],
        first_five_camera_distances_m=[.26982230422230874, .19618807360917923,
            .4626238527816416, .28500000000000003, .3738119263908207]),
    resolved_review_findings=[
        'First combined regression found that the new scheduler unnecessarily read the mount when only one candidate existed, adding an unrelated dependency to a legacy failure-audit path. Coverage now requires more than one candidate. The original failing test and its assertions remain unchanged and pass in final combined validation.',
        'An original URDF test resource was absent from the isolated staging directory. Combined validation copied the unmodified live resource and verified its SHA256; this is test-environment repair, not a deployed source change.',
    ],
    verified_invariants=[
        'Coverage dispatch is limited to diagnostic unknown observe_return observation_prefix with no positive endpoint IK hint. Initial direct contact, post-observation contact, carton and ordinary measured-information observation keep their existing paths.',
        'All 47 recorded geometry-qualified endpoint poses and existing per-candidate gates are retained. Negative or timed-out endpoint IK remains unknown and never removes a candidate. First four dispatched candidates cover four distinct distances; old untested near positions now receive a turn.',
        'Distance grouping and optical-axis direction grouping change only order. The optical axis uses frozen camera_link +X and the immutable tool-to-camera transform. No pose, contact dimension, support envelope or existing gate is altered.',
        'The first-incumbent signal is set only after the original strict path evaluation, required following/support evaluation, current-ticket validation and commit under the original stream condition. Failed following and a ticket invalidated during strict work do not set it or publish a valid plan.',
        'After the same prepared ticket earns a fully qualified incumbent, remaining unvisited candidates return to the original preference order exactly once. Final selection still compares only original qualified checked variants.',
        'Original three-second accumulated strict-work start budget, six-check ceiling, eight-second incumbent comparison window, source limits, identity checks and execution checks remain unchanged. The scheduler does not extend deadlines or claim optimality.',
        'The incumbent ticket remains private in the same request-local runtime dict. Audit builders select fields explicitly rather than serializing the entire runtime. Public scheduling evidence contains only JSON-compatible values; no deepcopy of the runtime breaks ticket identity.',
        'Combined validation verifies the 147-file live baseline before and after, unchanged stage sources and actual staged module and rospkg imports. The reviewer made no live source edits or robot actions.',
    ],
    validation_limits=[
        'Only four recorded endpoints have real strict path results, all failed. The other 43 are untested; neither the new schedule nor 20 ms IK -31 proves them reachable or unreachable.',
        'The reviewer independently read the sealed bag static transform and executed the pure ordering helper on recorded poses. Targeted and combined pytest execution was performed by the implementation and combined-test agents, with reports inspected here.',
        'Strict-loop regression uses controlled planning/following responses to verify scheduling, budget, promotion and cancellation plumbing. It does not establish new live IK, collision-free motion, successful observation arrival or physical object holding.',
        'Search order increases coverage within the unchanged bounded budget; it cannot guarantee a qualified first path will exist or finish before the original source deadlines.',
    ])
dest = NEW / 'verification/gen9_silhouette_coverage_independent_review.json'
assert not dest.exists(), 'existing sealed review must not be overwritten'
dest.write_text(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + '\n')
print(json.dumps(dict(path=str(dest), sha256=sha(dest), disposition=report['disposition'],
    files=len(files), combined_tests=combined['unique_tests']), ensure_ascii=False))
