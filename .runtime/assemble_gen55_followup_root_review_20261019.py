"""Bind the reviewed two-test followup to unchanged GEN55 production and focus.

Historical failed runs remain failed. This records root's source review; it
does not execute tests, contact ROS, install sources, or authorize motion.
"""
import argparse
import ast
import difflib
from gen55_idle_reload_common_20261019 import *
from install_and_seal_gen55_20261019 import check_delta

V = N/'verification'
PRIOR = V/'gen55_source_installation_r1/installed_candidate.json'
PRIOR_SHA = '38af05487563d1b19c539fa298bfba09b17b166c9c74dce7035419b7ad96deed'
OLD_REVIEW = V/'gen55_root_independent_review_r1/result.json'
OLD_REVIEW_SHA = '3e508b67d3ab7b57eed91422a500869bb4d752721b14d4824f7647d25d610f22'
FAILED = V/'gen55_full_parallel_r1/result.json'
FAILED_SHA = '0a7369d48ffff3caecdff12742fe6f0051b2c32ea6ec2e717d7c7b16f4c7d828'
PROBE = V/'gen55_legacy_0930_collision_probe_r1/result.json'
PROBE_SHA = '55185fbdeb1c7bec9fb657d5bd04d79171141f7ea7112459ebf9811ac99006b0'
T = 'src/alicia_flexible_grasp_supervisor/tests/'
FUNCTIONS = {
    T+'test_contact_sequence_precheck.py':
        'test_recorded_collision_is_rejected_during_candidate_check_without_motion',
    T+'test_gen38_contact_replay_staging.py':
        'test_actual_service_and_sequence_keep_failed_retimed_curve_after_exception_flattening',
}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('manifest', 'focus'):
        p.add_argument('--'+name, type=Path, required=True)
        p.add_argument('--'+name+'-sha256', required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    pins = {str(PRIOR):PRIOR_SHA, str(OLD_REVIEW):OLD_REVIEW_SHA,
            str(FAILED):FAILED_SHA, str(PROBE):PROBE_SHA,
            str(a.manifest):a.manifest_sha256, str(a.focus):a.focus_sha256}
    require(all(sha(path)==digest for path,digest in pins.items()), 'review input changed')
    runner = W/'.runtime/verify_gen55_parallel_offline_20261019.py'
    manifest, context, focus, evidence = validate_result(a.focus, a.manifest,
        a.manifest_sha256, runner, sha(runner), 'focus')
    prior, old_review, failed, probe = map(read, (PRIOR, OLD_REVIEW, FAILED, PROBE))
    source = manifest['source_sha256']; previous = prior['source_sha256']
    require(old_review['passed'] is True and old_review['source_sha256']==previous,
            'historical source review differs from previous candidate')
    require(len(source)==1082 and source.keys()==previous.keys(), 'source scope changed')
    require({path for path in source if source[path]!=previous[path]}==set(FUNCTIONS),
            'followup changed more than two reviewed tests')
    require(failed['source_sha256']==previous and failed['passed'] is False and
            failed['summary']==dict(tests=9568,failures=2,errors=0,skipped=0),
            'historical full failure changed')
    require(probe['passed'] is True and probe['active_reference_complete_union'] is True and
            probe['padding_each_tool_axis_each_side_m']==.0006 and
            probe['historical_failures_reclassified'] is False and
            len(probe['rows'])==2 and all(row['new_passed'] and
                row['downstream_rejection_propagated'] and row['unsupported_CAD_rejection_preserved']
                for row in probe['rows']), 'independent geometry evidence incomplete')
    require(focus['summary']==dict(tests=42,failures=0,errors=0,skipped=0) and
            set(manifest['focus_scope'])==set(FUNCTIONS), 'focused scope differs')
    require(len(context['scope'])==336 and len(context['case_identities'])==9568,
            'full scope changed')
    candidate = Path(manifest['workspace_root'])
    require(candidate==W/'.runtime/gen55_candidate_r1' and
            all(sha(candidate/path)==digest for path,digest in source.items()), 'candidate changed')
    baseline, changed, added = check_delta(manifest)
    require(all(sha(W/path)==digest for path,digest in baseline.items()), 'GEN54 changed before install')
    require(all(sha(W/path)==source[path]==previous[path] for path in added),
            'retained additional files changed')
    dependencies = dict(pins, **evidence)
    for row in manifest['approved_gen55_test_changes']:
        ref=row['review_evidence']; require(sha(ref['path'])==ref['sha256'], 'test review changed')
        review=read(ref['path'])
        require(review['passed'] and review['candidate_sha256']==source[row['path']],
                'test review not bound to final bytes')
        dependencies[ref['path']]=ref['sha256']
    # Review the runner delta as data: no runner function or execution guard changed.
    old_runner=V/'gen55_full_legacy_followup_r1/before_0_verify_gen55_parallel_offline_20261019.py'
    require(sha(old_runner)=='d095d6e7fe6432104bd2f2441d7284d79c145c738df347ff4634426f91123f2f',
            'original runner snapshot differs')
    def functions(path):
        return {node.name:ast.dump(node,include_attributes=False) for node in
                ast.parse(path.read_text()).body if isinstance(node,(ast.FunctionDef,ast.ClassDef))}
    require(functions(old_runner)==functions(runner), 'runner executable functions changed')
    dependencies[str(old_runner)]=sha(old_runner)
    preparation=V/'gen55_followup_pipeline_r1/static_preparation_review.json'
    require(read(preparation)['passed'] is True, 'runner/freeze static review failed')
    dependencies[str(preparation)]=sha(preparation)
    dependencies[str(Path(__file__).resolve())]=sha(__file__)
    output=a.output.resolve();storage(output);require(not output.exists(),'review output exists');output.mkdir()
    snapshots={}
    for path in sorted(changed):
        saved=output/'GEN54_source_before_install'/path;saved.parent.mkdir(parents=True,exist_ok=True)
        raw=(W/path).read_bytes();saved.write_bytes(raw)
        require(sha(saved)==baseline[path], 'baseline readback differs')
        snapshots[path]=dict(path=str(saved),sha256=sha(saved));dependencies[str(saved)]=sha(saved)
        diff=output/(Path(path).name+'.diff')
        diff.write_text(''.join(difflib.unified_diff(raw.decode().splitlines(True),
            (candidate/path).read_text().splitlines(True),fromfile='GEN54/'+path,tofile='GEN55/'+path)))
        dependencies[str(diff)]=sha(diff)
    require(all(sha(path)==digest for path,digest in dependencies.items()),'final evidence drift')
    result=dict(passed=True,reviewer='root',source_sha256=source,source_count=1082,
        dependency_sha256=dependencies,baseline_source_snapshots=snapshots,
        full_checks_retained=True,physical_grasp_verified=False,full_regression_passed=False,
        runtime_deployed=False,focused_case_count=42,full_expected_case_count=9568,
        historical_review=dict(path=str(OLD_REVIEW),sha256=OLD_REVIEW_SHA),
        unchanged_since_prior_review=1080,production_changed_since_prior_review=False,
        historical_full_failure_preserved=True,
        root_findings=[
            'Both old test identities retain their original finite-box rejection and receipt assertions in explicit unsupported-CAD mode.',
            'The actual supported CAD mode checks both complete continuous curves, reaches the whole-arm provider, and still propagates downstream rejection without execution authority.',
            'GEN38 preserves its two source-hash assertions and explicitly adds the complete CAD helper as the third replay dependency.',
            'Independent full-STL projections retain every active/reference vertex and each 0.6 mm tool-axis allowance; old FK and input wires remain unchanged.',
            'All 1080 other sources match the previously reviewed candidate; the runner function AST and execution guards are unchanged.',
            'The 9568-case full run must be executed anew after installation; previous failed and passed shards are not combined into a new full result.',
        ],limitations=[
            'Historical failed retimed wire was not recorded; these timing variants are explicitly reconstructed offline.',
            'Whole-arm positive provider is isolated; new live scene, feedback and full execution checks remain required.',
            'Historical physical failure is unchanged; there is no new physical grasp success.',
        ],motion_commands_sent=False,temperature_queried=False,recorded_ns=time.time_ns())
    write(output/'result.json',result)
    print(json.dumps(dict(passed=True,path=str(output/'result.json'),sha256=sha(output/'result.json'))))


if __name__=='__main__':
    main()
