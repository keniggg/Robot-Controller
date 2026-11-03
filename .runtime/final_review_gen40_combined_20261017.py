"""Independent offline source/evidence review; approve only a closed full pass."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
D=E/'ros_log/full_latest_gen36_restart_20261017T012037Z'
BASE=D/'verification/gen39_release_manifest.json'
BASE_SHA='47a2f5148d585dfa0d938868453e8293fffdefc35290f1b788bc013703257030'
COPY=W/'.runtime/gen40_r1_source'
COPY_SHA='fd8fab7a7fde24c98f17266a7de075758dfddfd868e8c10532bebc66ce72ee7b'
P='src/alicia_flexible_grasp_supervisor/'
PROD={P+'scripts/remote_grasp6d_node.py',P+'src/alicia_flexible_grasp/vision/regular_box_inference.py',
    *[P+'src/alicia_flexible_grasp/grasp/'+n for n in ('gripper_geometry.py','tabletop_geometry_candidates.py','measured_contact.py')]}
OLD_TESTS={P+'tests/'+n for n in ('test_gen26_contact_provenance_independent.py',
    'test_gen26_inferred_tabletop_candidates.py','test_gen31_box_interval_independent.py',
    'test_gen31_box_precision_budget.py','test_gen36_inferred_aperture_budget.py','test_gen37_measured_aperture.py')}
NEW_TESTS={P+'tests/test_gen40_source_bound_enclosure.py':27,P+'tests/test_gen40_scene_audit_capture.py':9,
    P+'tests/test_gen40_clearance_preference.py':41,'src/alicia_grasp_modes/tests/test_gen40_clearance_enclosure_chain.py':4}
HELPERS=['.runtime/'+n for n in ('verify_gen40_offline_tests_20261017.py',
    'seal_gen40_release_20261017.py','load_gen40_idle_compute_20261017.py',
    'read_gen40_deployment_preflight_20261017.py','finalize_gen40_idle_deployment_20261017.py')]


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--final',action='store_true')
    ap.add_argument('--combined',type=Path,default=D/'verification/gen40_combined_r1')
    ap.add_argument('--output',type=Path,default=D/'verification/gen40_combined_review_r1')
    args=ap.parse_args()
    assert Path('/media/zhuyupei/SanStand').is_mount()
    args.output.resolve().relative_to((D/'verification').resolve())
    args.output.mkdir(exist_ok=True)
    inputs={}
    def read(path,expected=None):
        path=Path(path).resolve(strict=True);raw=path.read_bytes();digest=hashlib.sha256(raw).hexdigest()
        assert inputs.setdefault(str(path),digest)==digest,path
        assert expected is None or digest==expected,path
        return raw
    def load(path,expected=None):return json.loads(read(path,expected))
    def write(name,value):
        path=args.output/name
        raw=(json.dumps(value,ensure_ascii=False,indent=2,sort_keys=True,allow_nan=False)+'\n').encode()
        with path.open('xb') as f:f.write(raw)
        assert path.read_bytes()==raw
        return hashlib.sha256(raw).hexdigest()
    baseline=load(BASE,BASE_SHA)
    assert baseline['source_count']==1025 and baseline['test_scope_files']==275
    assert baseline['test_summary']==dict(tests=7962,failures=0,errors=0,skipped=0)
    candidate=load(COPY/'manifest.json',COPY_SHA)
    assert candidate['source_count']==1029 and candidate['baseline_release_sha256']==BASE_SHA
    assert candidate['passed'] is False and candidate['regression_pending'] is True
    sources=candidate['source_sha256']
    assert set(sources)==set(baseline['source_sha256'])|set(NEW_TESTS)
    assert {n for n,h in baseline['source_sha256'].items() if sources[n]!=h}==PROD|OLD_TESTS
    expected_scope=set(baseline['test_scope'])|set(NEW_TESTS)
    scope=load(args.combined/'scope.json')
    assert scope['full'] is True and set(scope['tests'])==expected_scope and len(scope['tests'])==279
    assert scope['source_sha256']==sources
    assert scope['baseline_sha256']==BASE_SHA
    for n,h in sources.items():
        assert sha(W/n)==sha(COPY/n)==h,n
    reviewed={n:sources[n] for n in sorted(PROD|OLD_TESTS|set(NEW_TESTS))}
    for n in HELPERS:
        read(W/n);reviewed[n]=inputs[str((W/n).resolve())]
    generic=P+'src/alicia_flexible_grasp/vision/fused_scene_geometry.py'
    assert sources[generic]==baseline['source_sha256'][generic]

    # Verify the exact scope of changed executable functions independently.
    changed_defs={}
    def function_map(path):
        tree=ast.parse(Path(path).read_text());result={}
        def walk(rows,prefix=''):
            for n in rows:
                if isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef)):
                    result[prefix+n.name]=ast.dump(n,include_attributes=False)
                elif isinstance(n,ast.ClassDef):walk(n.body,prefix+n.name+'.')
        walk(tree.body);return result
    expected_defs={P+'scripts/remote_grasp6d_node.py':{'RemoteGrasp6DNode._prepare_and_predict'},
        P+'src/alicia_flexible_grasp/vision/regular_box_inference.py':{'enclose_inferred_box_geometry'},
        P+'src/alicia_flexible_grasp/grasp/gripper_geometry.py':{'evaluate_inferred_box_contact','inferred_box_precision_reserve'},
        P+'src/alicia_flexible_grasp/grasp/tabletop_geometry_candidates.py':{'_generate_inferred_proposals'},
        P+'src/alicia_flexible_grasp/grasp/measured_contact.py':{'evaluate_measured_aperture'}}
    for n in PROD:
        old=function_map(W/'.runtime/gen39_baseline_source'/n);new=function_map(W/n)
        changed={key for key in set(old)|set(new) if old.get(key)!=new.get(key)}
        assert changed==expected_defs[n],(n,changed)
        changed_defs[n]=sorted(changed)

    focus_names=['gen40_clearance_preference_focus_r2','gen40_source_bound_enclosure_focus_r2',
        'gen40_scene_audit_capture_focus_r1','gen40_clearance_enclosure_chain_focus_r1']
    focus_summaries={}
    for name in focus_names:
        directory=D/'verification'/name;r=load(directory/'result.json')
        assert r['passed'] is True and r['source_unchanged'] and r['scope_complete']
        assert r['returncode']==0 and not r['external_boundary_attempts']
        assert all(r['summary'][k]==0 for k in ('failures','errors','skipped'))
        read(directory/'pytest.xml',r['xml_sha256']);read(directory/'pytest.log',r['log_sha256'])
        read(directory/'scope.json');read(directory/'bootstrap/sitecustomize.py',r['bootstrap_sha256'])
        if (directory/'seal.json').exists():load(directory/'seal.json')
        focus_summaries[name]=r['summary']
        for n in set(NEW_TESTS)&set(r['test_scope']):
            assert r['source_sha256'][n]==sources[n] and r['test_case_counts_by_file'][n]==NEW_TESTS[n]
        # Earlier focused runs may bind pre-correction test fixtures, but all
        # tested production functions must match the final frozen production.
        assert all(r['source_sha256'][n]==sources[n] for n in PROD)

    a=load(D/'verification/gen40_policy_independent_source_review_r1/result.json',
        'ce6c8d53e4b599b5b365b151ebf0a038afad5cf8f915990cfbf414ed0f8e231c')
    c=load(D/'verification/gen40_enclosure_and_audit_independent_review_r1/result.json',
        '7dd632f20b45cd46708bc7f605e5a4c45ca5dbfc702ef0d3e9f2d1f7ae42c403')
    assert not a['blocking_findings'] and c['no_blocking_findings'] and c['review_complete_for_bound_hashes']
    for review in (a,c):
        assert all(sources[n]==h for n,h in review['source_sha256'].items())
    for directory in ('gen40_policy_independent_source_review_r1','gen40_enclosure_and_audit_independent_review_r1'):
        load(D/'verification'/directory/'seal.json')
    actual_report=Path(c['evidence']['actual_source_report_path'])
    actual=load(actual_report,c['evidence']['actual_source_report_sha256'])
    assert actual['retained_points']==2610 and actual['execution_authorized'] is False
    assert actual['original_device_path_available'] is False and actual['physical_grasp_verified'] is False
    assert actual['endpoint_IK_and_retimed_path'].startswith('not fabricated')

    helper_reviews=['gen40_offline_compute_helpers_review_r1','gen40_preflight_finalizer_helpers_review_r1','gen40_sealer_helper_review_r1']
    for name in helper_reviews:
        r=load(D/'verification'/name/'result.json')
        assert r['reviewed'] and not r['blocking_findings']
        for n,h in r['reviewed_source_sha256'].items():assert sha(W/n)==h,n
        load(D/'verification'/name/'seal.json')
    invocation=load(D/'verification/gen40_live_invocation_chain_review_r1/result.json',
        '54fcaaf2686346e4274bd999826d969d7bef0fe4a37634bab4e4e16d5c12d55d')
    assert invocation['no_blocking_interface_findings'] and invocation['helpers_unchanged_during_review']
    assert invocation['permissions_or_clearance_written'] is False
    assert invocation['motion_commands_sent'] is False and invocation['does_not_attest_future_live_readiness']
    for n,h in invocation['source_sha256'].items():
        assert sha(W/n)==h,n
        reviewed[n]=h
    load(D/'verification/gen40_live_invocation_chain_review_r1/seal.json')
    # Preserve all first-round failures. Do not turn fixture repairs into a
    # claim that the first source/test version had passed.
    red_history={}
    for name,expected in [('gen40_clearance_preference_focus_r1',6),('gen40_source_bound_enclosure_focus_r1',3)]:
        directory=D/'verification'/name;r=load(directory/'result.json')
        assert r['passed'] is False and r['summary']['failures']==expected
        assert r['source_unchanged'] and not r['external_boundary_attempts']
        read(directory/'pytest.xml',r['xml_sha256']);read(directory/'pytest.log',r['log_sha256'])
        read(directory/'scope.json');red_history[name]=r['summary']
    load(D/'verification/gen40_clearance_codec_fixture_fix_r1/result.json',
        '694e67c42e5f9736a7765a20db6304e2c2e121fc4b43ca372aaebd5a00427311')
    for name in ('before.py','after.py','changes.diff'):read(D/'verification/gen40_clearance_codec_fixture_fix_r1'/name)
    read(__file__)
    prepared=dict(schema_version=1,kind='gen40_independent_combined_review_preparation',
        recorded_ns=time.time_ns(),release_review_complete=False,full_result_not_yet_approved=True,
        reviewed_source_sha256=reviewed,evidence_sha256=dict(inputs),expected_summary=dict(tests=8043,failures=0,errors=0,skipped=0),
        copied_source_count=1029,scope_test_files=279,authorized_changed_original_files=sorted(PROD|OLD_TESTS),
        changed_executable_definitions=changed_defs,focus_summaries=focus_summaries,retained_first_round_failures=red_history)
    if not args.final:
        assert not (args.output/'prepared.json').exists()
        h=write('prepared.json',prepared)
        print(json.dumps(dict(prepared=True,release_review_complete=False,path=str(args.output/'prepared.json'),sha256=h)))
        return

    # The only approval path starts with an actually closed full pass.
    result_path=args.combined/'result.json'
    assert result_path.exists(),'full result not yet closed; review remains pending'
    r=load(result_path)
    assert r['passed'] is True,'full regression failed; do not approve'
    assert r['returncode']==0 and r['regression_scope']=='full_combined'
    assert r['summary']==dict(tests=8043,failures=0,errors=0,skipped=0)
    assert r['source_count']==1029 and r['source_sha256']==sources and r['source_unchanged'] is True
    assert r['scope_complete'] and r['baseline_file_test_counts_retained'] and not r['unassigned_testcases']
    assert r['test_scope_files']==279 and set(r['test_scope'])==expected_scope
    assert not r['external_boundary_attempts'] and r['live_ros_contacted'] is False
    assert r['motion_commands_sent'] is False and r['physical_grasp_verified'] is False
    counts=r['test_case_counts_by_file']
    assert set(counts)==expected_scope and sum(counts.values())==8043
    assert all(counts[n]==v for n,v in baseline['test_case_counts_by_file'].items())
    assert all(counts[n]==v for n,v in NEW_TESTS.items())
    xml=read(args.combined/'pytest.xml',r['xml_sha256']);root=ET.fromstring(xml)
    xml_summary={k:sum(int(s.get(k,0)) for s in root.iter('testsuite')) for k in ('tests','failures','errors','skipped')}
    assert xml_summary==r['summary']
    xml_counts=dict.fromkeys(expected_scope,0)
    for case in root.iter('testcase'):
        assert case.get('file') in xml_counts
        assert all(case.find(k) is None for k in ('failure','error','skipped'))
        xml_counts[case.get('file')]+=1
    assert xml_counts==counts and all(xml_counts.values())
    read(args.combined/'pytest.log',r['log_sha256'])
    read(args.combined/'bootstrap/sitecustomize.py',r['bootstrap_sha256'])
    assert sha(W/HELPERS[0])==r['runner_sha256']
    assert r['baseline_sha256']==BASE_SHA and r['baseline']==str(BASE)
    for n,h in sources.items():assert sha(W/n)==sha(COPY/n)==h,n
    for path,h in inputs.items():assert sha(path)==h,path
    review=dict(schema_version=1,kind='gen40_final_independent_combined_source_review',
        recorded_ns=time.time_ns(),reviewed=True,release_review_complete=True,blocking_findings=[],
        reviewed_source_sha256=reviewed,evidence_sha256=dict(inputs),source_count=1029,
        full_summary=r['summary'],full_test_files=279,full_regression=str(result_path.resolve()),
        source_and_candidate_copy_match_full=True,all_original_275_file_counts_exact=True,
        authorized_changed_production=sorted(PROD),authorized_modified_policy_contract_tests=sorted(OLD_TESTS),
        all_other_original_sources_byte_unchanged=True,all_other_269_original_tests_byte_unchanged=True,
        new_test_counts=NEW_TESTS,changed_executable_definitions=changed_defs,
        focused_evidence=focus_summaries,retained_first_round_failures=red_history,
        fixed_findings=[
            'Configured extra 2 mm is an explicit clearance preference across proposal, typed contact and measured aperture; real full-family U+fit and actual opening remain hard.',
            'Qualified planning representation uses all original measured points plus complete empirical family, exact same-source geometry digest and no point/uncertainty reduction. Generic measured-only fused v2 remains byte unchanged.',
            'Measured registration receipt is captured against its original geometry before planning representation changes; all original source/geometry equality and owner checks stay strict.',
            'First preference focus 6F were a new test decoding a schema3 compressed wire wrapper as bare JSON; corrected by real decoder only.',
            'First enclosure focus 3F comprise a malformed new invalid-object fixture and two old software-reserve boundary expectations; real physical-deficit boundaries and source-negative preconditions are retained, original failures immutable.',
        ],
        trust_boundary='Trusted producer verifies complete original P and F; existing wire consumer independently verifies source/support/frozen plan/F, not all measured points. Digest alone is not called all-point proof.',
        actual_record_replay='Closed GEN39 final source retains 2610 exact points and original model. New offline producer/typed/transport/aperture/CAD tests pass; final contact FK is explicitly nominal==actual simulation, and no historical contact IK/retimed trajectory is fabricated.',
        limitations=['Passing offline software regression does not prove a live path, a continuous following tube, fresh onsite target/robot state, or physical grip.',
            'U/fit are empirical model/geometric budgets, not a certified bound on full 3D handeye/tracking errors.',
            'Review author also owns the three policy modules; their production diff is independently reviewed by A, and enclosure/remote changes by C.',
            'The full Python socket/device guard is scoped; C SDK factories still depend on explicit inert fixtures.'],
        runtime_deployment_pending=True,deployed=False,physical_grasp_verified=False,motion_authorized=False,
        ros_contacted=False,hardware_contacted=False,production_or_tests_edited_during_final_review=False)
    h=write('review.json',review)
    seal=write('seal.json',dict(schema_version=1,review_completed=True,release_review_complete=True,
        artifacts_sha256={'review.json':h},reviewed_source_sha256=reviewed,full_summary=r['summary'],
        source_count=1029,blocking_findings=[],runtime_deployment_pending=True,motion_authorized=False))
    print(json.dumps(dict(review=str(args.output/'review.json'),review_sha256=h,seal_sha256=seal,blocking_findings=[])))


if __name__=='__main__':main()
