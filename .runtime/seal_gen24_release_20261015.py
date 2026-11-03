"""Seal a reviewed full GEN24 combined result; no ROS access or deployment."""
import argparse,json,time
import xml.etree.ElementTree as ET
from pathlib import Path
from verify_gen24_combined_20261015 import W,N,BASE,BASE_SHA,FULL,FULL_SHA,sha,read,guard,save,inventory,xml_summary

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    for key in ('combined','review','output'):ap.add_argument('--'+key,type=Path,required=True)
    ap.add_argument('--supplement',type=Path);ap.add_argument('--review-sha256',required=True);a=ap.parse_args();guard(a.output);assert not a.output.exists()
    evidence={}
    def bind(p,d=None):
        p=Path(p).resolve(strict=True);h=sha(p);assert d is None or h==d,str(p)
        assert str(p) not in evidence or evidence[str(p)]==h
        evidence[str(p)]=h;return p
    bind(BASE,BASE_SHA);bind(FULL,FULL_SHA)
    p=bind(a.combined);assert N/'verification' in p.parents
    r=read(p);assert r['source_unchanged'] is True and r['scope_complete'] is True
    assert r['passed'] is True or (a.supplement is not None and r['summary']==dict(tests=6450,failures=0,errors=0,skipped=1))
    assert r['returncode']==0 and r['production_worktree_tested'] is True and r['live_ros_contacted'] is False
    assert r['regression_scope']=='full_combined'
    assert r['baseline']==str(BASE) and r['baseline_sha256']==BASE_SHA
    assert r['full_scope_baseline']==str(FULL) and r['full_scope_baseline_sha256']==FULL_SHA
    patch_path=bind(r['patch'],r['patch_sha256']);patch=read(patch_path)
    spec=patch['spec'];bind(patch['spec_path'],patch['spec_sha256'])
    sources,scope=inventory(spec)
    assert sources==r['source_sha256'] and len(sources)==r['source_count']
    assert scope==r['test_scope'] and len(scope)==r['test_scope_files']
    assert set(patch['files'])==set(spec['production']+spec['changed_tests']+spec['new_tests']+spec.get('new_resources',[]))
    for name,row in patch['files'].items():
        assert row==dict(baseline_sha256=read(BASE)['source_sha256'].get(name),sha256=sources[name])
    bind(p.parent/'pytest.log',r['log_sha256']);bind(p.parent/'pytest.xml',r['xml_sha256'])
    assert xml_summary(p.parent/'pytest.xml')==r['summary']
    assert r['summary']['tests']>=read(FULL)['test_summary']['tests']
    summary=dict(r['summary']);supplement=None
    if a.supplement is None:
        assert all(summary[k]==0 for k in ('failures','errors','skipped'))
    else:
        from complete_gen24_recorded_fixture_20261015 import IDENTITY,SOURCE_SHA,DEPTH
        sp=bind(a.supplement);supplement=read(sp)
        assert N/'verification' in sp.parents
        assert supplement['passed'] is True and supplement['returncode']==0 and supplement['source_unchanged'] is True
        assert supplement['combined']==str(p) and supplement['combined_sha256']==sha(p)
        assert supplement['source_sha256']==sources and supplement['original_summary']==r['summary']
        assert supplement['live_ros_contacted'] is False
        bind(sp.parent/'pytest.log',supplement['log_sha256']);bind(sp.parent/'pytest.xml',supplement['xml_sha256'])
        bind(W/'.runtime/complete_gen24_recorded_fixture_20261015.py',supplement['runner_sha256'])
        skipped=[case for case in ET.parse(p.parent/'pytest.xml').getroot().iter('testcase') if case.find('skipped') is not None]
        actual=list(ET.parse(sp.parent/'pytest.xml').getroot().iter('testcase'))
        assert len(skipped)==len(actual)==1
        assert (skipped[0].get('classname'),skipped[0].get('name'))==(actual[0].get('classname'),actual[0].get('name'))==IDENTITY
        assert xml_summary(sp.parent/'pytest.xml')==supplement['summary']==dict(tests=1,failures=0,errors=0,skipped=0)
        assert r['summary']==dict(tests=6450,failures=0,errors=0,skipped=1)
        fixture=read(bind(supplement['fixture_manifest'],supplement['fixture_manifest_sha256']))
        assert fixture['source_retained_sha256']==SOURCE_SHA
        bind(fixture['recovered_source'],SOURCE_SHA)
        assert fixture['depth_sha256']=='e7837b324ac26385936007125fc8b270cb1794c931cc3c9f6a103d4aab60db36'
        bind(DEPTH,fixture['depth_sha256'])
        summary.update(skipped=0)
        assert supplement['unique_completed_summary']==summary

    assert set(r['test_case_counts_by_file'])==set(scope) and all(n>0 for n in r['test_case_counts_by_file'].values())
    assert sum(r['test_case_counts_by_file'].values())==r['summary']['tests'] and not r['unassigned_testcases']
    bind(W/'.runtime/verify_gen24_combined_20261015.py',r['runner_sha256'])
    review_path=bind(a.review,a.review_sha256);review=read(review_path)
    assert review['reviewed'] is True and not review['blocking_findings']
    assert set(patch['files'])<=set(review['reviewed_source_sha256'])
    for name,h in review['reviewed_source_sha256'].items():
        assert sha(W/name)==h,name
        if name in sources:assert sources[name]==h
    for name,h in review['evidence_sha256'].items():bind(name,h)
    for name in ('reload_gen19_compute_20261014.py','start_observation_measurement_consumers_20261008.sh','observation_measurement_consumers_20261008.launch','prepare_gen20_continuation_receipt_20261014.py','continuous_grasp_authorization_20261012.py','run_reboot_realigned_grasp_20261010.py','capture_continuation_scene_20261012.py','read_gen7_yield_readiness_20261012.py'):bind(W/'.runtime'/name)
    previous=bind(N/'gen23_deployment/result.json');dep=read(previous)
    assert dep['deployment_passed'] is True and dep['whole_patch_deployed'] is True and dep['source_manifest_sha256']==BASE_SHA
    assert len(dict(dep['after'],**dep['preserved_after']))==11
    bind(__file__)
    release=dict(schema_version=1,kind='gen24_current_support_recovery_and_fixed_identity_wait',sealed_ns=time.time_ns(),passed=True,reviewed=True,installed=True,installed_semantics='Reviewed source is installed; the full combined suite was rerun. Runtime three-compute reload is pending.',source_sha256=sources,source_count=len(sources),test_scope=scope,test_scope_files=len(scope),test_summary=summary,full_run_test_summary=r['summary'],supplemental_regression=None if a.supplement is None else str(a.supplement.resolve()),supplemental_test_summary=None if supplement is None else supplement['summary'],validation_semantics='Original full suite and exact skipped-case completion are retained separately; test_summary counts unique completed cases.',test_case_counts_by_file=r['test_case_counts_by_file'],regression_scope='full_combined',baseline_release=str(BASE),baseline_release_sha256=BASE_SHA,full_scope_baseline=str(FULL),full_scope_baseline_sha256=FULL_SHA,actual_regression=str(p),combined_review=str(review_path),evidence_sha256=evidence,previous_deployment=str(previous),runtime_deployment_pending=True,physical_grasp_verified=False,physical_grasp_success=False,motion_commands_sent=False,temperature_queried=False)
    save(a.output,release);print(json.dumps(dict(release=str(a.output),sha256=sha(a.output),summary=summary)),flush=True)
if __name__=='__main__':main()
