"""Record root source review and bind completed evidence. No ROS or motion."""
import argparse
import difflib
from gen55_idle_reload_common_20261019 import *
from install_and_seal_gen55_20261019 import check_delta

V = N/'verification'
PINS = {
    'gen55_cad_contract_resume_r1/module_source_review.json': 'c3aa7722c02a9ffd4068304d323081eda6412f781e698ea2a200348ac9d922e4',
    'gen55_cad_contract_resume_r1/versioned_expectation_review.json': 'ef5eb5fffc36fbc378b839d4217607ac96d1dcc5df8ee0df0ebd4317052edbdb',
    'gen55_pipeline_replay_resume_r2/review.json': '6609ed94aa83261bff9ae4976fe49f6b8c40523b69d6544dbd9b92a40bc17049',
    'gen55_held_pregrasp_reuse_r1/result.json': '72be42134825860b4748057ddfcf095534284d3b4c862b6a36b25b068a73145f',
    'gen55_g51_case2_cad_probe_r1/result.json': '17cb29345127daba4924d960bac90b5cfb985b14d1f4605a47691dec52ddf596',
    'gen55_integrated_focus_r1/result.json': 'b433eeb136de3018a6990cb26d6f2d4847b046267364b1a7eabec38e1f57ed8e',
    'gen55_runner_independent_review_r2/plan_review.json': '49ec9f3cf5a5517d89402e4c720028e623c352fc929abd73cb01b90f26156a44',
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('manifest', 'focus', 'runner_review', 'expectation_review'):
        parser.add_argument('--'+name.replace('_', '-'), type=Path, required=True)
        parser.add_argument('--'+name.replace('_', '-')+'-sha256', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    refs = {n:getattr(args,n).resolve(strict=True) for n in ('manifest','focus','runner_review','expectation_review')}
    dependencies = {str(V/p):h for p,h in PINS.items()}
    dependencies.update({str(p):getattr(args,n+'_sha256') for n,p in refs.items()})
    require(all(sha(p)==h for p,h in dependencies.items()), 'input evidence changed')
    runner = W/'.runtime/verify_gen55_parallel_offline_20261019.py'
    manifest, context, focus, evidence = validate_result(refs['focus'],refs['manifest'],args.manifest_sha256,runner,sha(runner),'focus')
    dependencies.update(evidence)
    baseline, changed, added = check_delta(manifest)
    source = manifest['source_sha256']; candidate = Path(manifest['workspace_root'])
    require(candidate == W/'.runtime/gen55_candidate_r1', 'review candidate differs')
    require(len(source)==1082 and all(sha(candidate/p)==h for p,h in source.items()), 'candidate source differs')
    require(all(sha(W/p)==h for p,h in baseline.items()), 'GEN54 baseline differs before install')
    cad = read(V/'gen55_cad_contract_resume_r1/module_source_review.json')
    replay = read(V/'gen55_pipeline_replay_resume_r2/review.json')
    for row in (cad,replay,read(V/'gen55_held_pregrasp_reuse_r1/result.json'),read(refs['runner_review']),read(refs['expectation_review'])):
        require(row['passed'] is True, 'independent evidence did not pass')
    for mapping in (cad['owned_source_sha256'],cad['reviewed_related_sources']):
        require(all(source[p]==h for p,h in mapping.items()), 'reviewed production/test bytes changed')
    require(replay['dual_continuous_pass_indices']==[4,7] and replay['whole_arm_service_is_simulated'] is True,
        'replay result/limitations differ')
    require(replay['all_original_command_headers_and_wire_sha_exact'] is True and
        replay['nominal_curve_is_original_measured'] is False, 'recorded/reconstructed distinction lost')
    require(len(context['scope'])==336 and len(context['case_identities'])==9568, 'full scope changed')
    xml = V/'gen54_full_parallel_r1/pytest.xml'
    import xml.etree.ElementTree as ET
    old_cases = [(c.get('file',''),c.get('classname',''),c.get('name','')) for c in ET.parse(xml).getroot().iter('testcase')]
    require(len(old_cases)==9255 and set(old_cases)<=set(map(tuple,context['case_identities'])), 'historical identities lost')
    dependencies[str(xml)] = sha(xml)
    old_failed = read(V/'gen55_integrated_focus_r1/result.json')
    require(old_failed['passed'] is False and old_failed['summary']['failures']==2, 'original focus history changed')
    for name in ('runner_review','expectation_review'):
        for p,h in read(refs[name]).get('dependency_sha256',{}).items():
            require(sha(p)==h, 'independent review dependency changed: '+p)
            dependencies[p]=h
    output=args.output.resolve();storage(output);require(not output.exists(),'review output exists');output.mkdir()
    baseline_snapshots={}
    for p in sorted(changed):
        saved=output/'GEN54_source_before_install'/p;saved.parent.mkdir(parents=True,exist_ok=True)
        raw=(W/p).read_bytes();saved.write_bytes(raw)
        require(sha(saved)==baseline[p], 'baseline snapshot read-back differs')
        baseline_snapshots[str(W/p)]=dict(path=str(saved),sha256=baseline[p])
        dependencies[str(saved)]=baseline[p]
        diff=output/(Path(p).name+'.diff')
        diff.write_text(''.join(difflib.unified_diff(raw.decode().splitlines(True),(candidate/p).read_text().splitlines(True),fromfile='GEN54/'+p,tofile='GEN55/'+p)))
        dependencies[str(diff)]=sha(diff)
    fk = read(V/'gen55_held_pregrasp_reuse_r1/result.json')
    for p,h in fk['dependencies'].items():
        current=Path(p) if Path(p).is_absolute() else W/p
        require(sha(current)==h, 'FK equivalence dependency changed')
        key=str(current)
        if key in baseline_snapshots:
            dependencies[baseline_snapshots[key]['path']]=h
        else:
            dependencies[key]=h
    require(all(sha(p)==h for p,h in dependencies.items()), 'final dependency check failed')
    result=dict(passed=True,reviewer='root',source_sha256=source,source_count=len(source),
        dependency_sha256=dependencies,baseline_source_snapshots=baseline_snapshots,
        reviewed_production_sources=dict(cad['reviewed_related_sources'],**{p:h for p,h in cad['owned_source_sha256'].items() if '/robot/' in p}),
        full_checks_retained=True,physical_grasp_verified=False,full_regression_passed=False,runtime_deployed=False,
        focused_case_count=focus['summary']['tests'],historical_case_count=9255,full_expected_case_count=9568,
        retained_prior_focus_evidence=manifest.get('retained_focus_evidence'),
        source_findings=cad['invariant_findings'],
        root_findings=[
            'Root read the complete new CAD support-model implementation and all three production diffs; exact active/reference model union, padding, enlarged broadphase and continuous motion radius agree.',
            'Endpoint screen and continuous validator call the same target-overlap function. Endpoint path_verified remains false; nominal, compensated-command, whole-arm and execution checks remain required.',
            'Root independently compared all three historical GEN51 terminal/curve outcomes: only case2 changes under CAD; original cases0/1 remain rejected. Original failed focus is preserved.',
            'Exact-vector per-invocation FK caching does not cache target qualification, collision checks, deadlines, feedback or final quantized verification.',
            'Scheduler evidence shows the first candidate fourth height was reached with31.729s remaining; no change to candidate ordering or deadline was needed.',
        ],
        limitations=[
            'Offline nominal is reconstructed from frozen bias; only the original command has exact recorded wire/header SHA.',
            'Offline whole-arm provider is simulated; fresh live collision checks and SDK/scene acceptance remain required.',
            '32 percent speedup is isolated FK endpoint function timing, not measured task latency.',
            'Final installed full9568 regression, release, deployment and physical grasp validation are pending.',
        ],motion_commands_sent=False,temperature_queried=False,recorded_ns=time.time_ns())
    write(output/'result.json',result)
    print(json.dumps(dict(passed=True,path=str(output/'result.json'),sha256=sha(output/'result.json'),dependencies=len(dependencies))))


if __name__=='__main__':
    main()
