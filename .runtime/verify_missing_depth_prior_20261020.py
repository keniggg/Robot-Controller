"""New immutable missing-depth freeze using hash-bound GEN57 isolation machinery.

The old runner file and all historical receipts remain unchanged. This wrapper
explicitly replaces only their generation-specific manifest/context entrypoints;
their worker guard, provenance, exact-case comparator, LPT execution and workload
monitor are reused as hash-bound infrastructure. No ROS/network/hardware access.
"""
import argparse
from collections import Counter
from copy import deepcopy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import sys
import time
import xml.etree.ElementTree as ET

W = Path(__file__).resolve().parents[1]
V = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
OLD = W/'.runtime/verify_gen57_parallel_offline_r2_20261020.py'
OLD_SHA = '0e217bc784b24c603afa1f4bca7c977f35f9036c03b6b263c9801f749d1c2727'
RELEASE = V/'gen57_release_manifest.json'
HISTORY = V/'gen57_full_parallel_r2/result.json'
BASE_PLAN = HISTORY.parent/'plan.json'
PRODUCTION = 'src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_instance.py'
NEW = 'src/alicia_grasp_modes/tests/test_missing_depth_rgb_prior.py'
CONTRACT = 'src/alicia_grasp_modes/tests/test_observation_arrival_probable_foreground.py'
CONTRACT_EVIDENCE = V.parent/'gen57_resume_r3/actual_attempt_r1_geometry/missing_depth_prior_fix_r1/test_contract_change.json'
OLD_CASE = 'test_only_current_valid_measured_height_adds_probable_not_hard_foreground'
NEW_CASE = 'test_current_depth_proposals_and_missing_prior_never_add_hard_foreground'
FOCUS = [NEW] + ['src/alicia_grasp_modes/tests/test_'+name+'.py' for name in (
    'rgb_instance','measured_mask_seeds','side_seed_evidence_scale','observation_arrival_rgb_retry',
    'observation_arrival_probable_foreground','arrival_fragment_support_guards','arrival_valid_rgb_crop_guards',
    'unknown_rgb_validity','arrival_spatial_support','gen43_top_mask_handoff',
    'gen57_source_component_measurement','gen57_source_component_pipeline','rgb_handoff_stream')]
MANIFEST = None
MANIFEST_SHA = None


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes())


def save(path, value):
    with Path(path).open('x') as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n'); f.flush(); os.fsync(f.fileno())


def load_old():
    assert sha(OLD) == OLD_SHA
    spec = importlib.util.spec_from_file_location('frozen_gen57_isolation_machinery', OLD)
    old = importlib.util.module_from_spec(spec); spec.loader.exec_module(old)
    return old


O = load_old()


def freeze(output, candidate):
    serial = O.load_serial(); serial.storage(output)
    assert not output.exists() and not candidate.exists()
    release = read(RELEASE); history = read(HISTORY); plan = read(BASE_PLAN)
    assert release['generation'] == 'GEN57' and release['passed'] and release['installed']
    assert sha(HISTORY) == release['actual_regression_sha256']
    assert sha(BASE_PLAN) == history['plan_sha256']
    assert history['passed'] and history['full_regression_passed']
    assert history['source_sha256'] == release['source_sha256'] == plan['context']['sources']
    assert sha(HISTORY.parent/'pytest.xml') == history['xml_sha256']
    assert sha(OLD) == release['runner_sha256']
    original = release['source_sha256']
    current = {p:sha(W/p) for p in original}
    changed = sorted(p for p in current if current[p] != original[p])
    assert changed == sorted([PRODUCTION,CONTRACT]), changed
    assert NEW not in current
    current[NEW] = sha(W/NEW)
    evidence = read(CONTRACT_EVIDENCE)
    assert evidence and (CONTRACT_EVIDENCE.parent/'test_observation_arrival_probable_foreground_before.py').exists()
    assert sha(CONTRACT_EVIDENCE.parent/'test_observation_arrival_probable_foreground_before.py') == original[CONTRACT]
    # Exactly the recorded function rename and two probable-label expectation
    # changes are allowed. No older fixture or other test byte may differ.
    before = (CONTRACT_EVIDENCE.parent/'test_observation_arrival_probable_foreground_before.py').read_text()
    after = (W/CONTRACT).read_text()
    assert OLD_CASE in before and NEW_CASE in after and OLD_CASE not in after
    _, cases, _ = O.inventory(HISTORY.parent/'pytest.xml')
    mappings = []
    for case in cases:
        if case[0] == CONTRACT and case[2].startswith(OLD_CASE+'['):
            mapped = [case[0],case[1],case[2].replace(OLD_CASE,NEW_CASE,1)]
            mappings.append(dict(before=case[:],after=mapped[:]))
            case[:] = mapped
    assert len(mappings) == 10 and len(cases) == 9766
    dependencies = dict(plan['context']['compatibility_dependencies'])
    dependencies.update({str(p):sha(p) for p in (OLD,RELEASE,HISTORY,BASE_PLAN,HISTORY.parent/'pytest.xml',CONTRACT_EVIDENCE)})
    # Copy the exact production/test tree plus finite read-only historical CAD
    # and oracle inputs required by the frozen test contracts.
    candidate.mkdir(); output.mkdir()
    copied = dict(current)
    for p in list(O.SUPPLEMENTAL_CAD_INPUTS)+[r['relative'] for r in plan['context']['supplemental_executable_inputs']]:
        copied[p] = sha(W/p)
    for p in ('.runtime/gen37_baseline_source/src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_path_guard.py',
              '.runtime/gen37_baseline_source/src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/contact_sequence_precheck.py'):
        copied[p] = sha(W/p)
    for relative,digest in copied.items():
        target=candidate/relative; target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(W/relative,target); assert sha(target)==digest
    context = deepcopy(plan['context'])
    context.update(scope=list(context['scope'])+[p for p in FOCUS if p not in context['scope']],sources=current,case_identities=cases,
        old_case_identities=cases,expected_counts=dict(Counter(c[0] for c in cases)),
        old_expected_counts=dict(Counter(c[0] for c in cases)),required_changed_sources=[PRODUCTION],
        required_imports_by_test_file={NEW:[PRODUCTION]},compatibility_dependencies=dependencies,
        full_scope_ready=False,compatibility=dict(variant='candidate',
            production_source_changes=[dict(path=PRODUCTION,baseline_sha256=original[PRODUCTION],candidate_sha256=current[PRODUCTION])],
            explicit_old_test_contract_change=dict(path=CONTRACT,baseline_sha256=original[CONTRACT],candidate_sha256=current[CONTRACT],
                evidence_path=str(CONTRACT_EVIDENCE),evidence_sha256=sha(CONTRACT_EVIDENCE),case_identity_mapping=mappings),
            other_historical_test_and_fixture_bytes_unchanged=True))
    assert all(p in context['scope'] for p in FOCUS), [p for p in FOCUS if p not in context['scope']]
    manifest=dict(kind='missing_depth_prior_frozen_offline_candidate',created_ns=time.time_ns(),workspace_root=str(candidate),
        runner_sha256=sha(__file__),infrastructure_path=str(OLD),infrastructure_sha256=OLD_SHA,
        source_sha256=current,copied_input_sha256=copied,context=context,focus_scope=FOCUS,
        live_source_sha256=current,collection_scope=[p for p in FOCUS if p not in plan['context']['scope']],baseline_case_count=9766,baseline_source_count=len(original),
        explicit_case_identity_mapping=mappings,new_test=NEW,full_scope_ready=False,
        full_regression_passed=False,production_deployed=False)
    save(output/'manifest_uncollected.json',manifest)
    print(str(output/'manifest_uncollected.json'))


def bind(path, expected):
    global MANIFEST,MANIFEST_SHA
    MANIFEST=Path(path).resolve(strict=True);MANIFEST_SHA=expected
    assert sha(MANIFEST)==expected
    manifest=read(MANIFEST)
    assert manifest['kind']=='missing_depth_prior_frozen_offline_candidate'
    assert manifest['runner_sha256']==sha(__file__) and manifest['infrastructure_sha256']==sha(OLD)==OLD_SHA
    O.W=Path(manifest['workspace_root']).resolve(strict=True)
    O.MANIFEST_PATH=MANIFEST;O.MANIFEST_SHA=MANIFEST_SHA
    return manifest


def context(serial):
    manifest=read(MANIFEST)
    assert sha(MANIFEST)==MANIFEST_SHA and manifest['runner_sha256']==sha(__file__)
    assert all(sha(O.W/p)==h for p,h in manifest['copied_input_sha256'].items())
    assert all(sha(p)==h for p,h in manifest['context']['compatibility_dependencies'].items())
    # Generated ROS message modules are immutable inputs, never a production
    # fallback. They must retain the previously audited exact SHA256 bytes.
    assert all(sha(p)==h for p,h in manifest['context']['generated_imports'].items())
    return deepcopy(manifest['context'])


def stable(serial, plan):
    return (sha(__file__)==plan['runner_sha256'] and sha(OLD)==OLD_SHA
        and sha(MANIFEST)==MANIFEST_SHA==plan['candidate_manifest_sha256']
        and all(sha(p)==h for p,h in plan.get('focus_evidence_sha256',{}).items())
        and context(serial)==plan['context'])


def close_manifest(output, collection):
    manifest=read(MANIFEST); data=read(collection)
    assert data['mode']=='collect' and data['passed'] and data['source_unchanged'] and data['scope_complete']
    assert data['candidate_manifest_sha256']==MANIFEST_SHA and data['runner_sha256']==sha(__file__)
    cases=data['collected_case_identities'];assert cases and len(cases)==len(set(map(tuple,cases)))
    assert set(c[0] for c in cases)==set(manifest['collection_scope'])
    updated=deepcopy(manifest); c=updated['context']; c['case_identities'] += cases
    c['expected_counts'].update(Counter(row[0] for row in cases));c['full_scope_ready']=True
    c['compatibility_dependencies'][str(Path(collection).resolve())]=sha(collection)
    c['compatibility_dependencies'][str(MANIFEST)]=MANIFEST_SHA
    updated.update(full_scope_ready=True,collection_path=str(collection),collection_sha256=sha(collection))
    save(output,updated);print(str(output))


def make_plan(output, mode, workers, focus):
    serial=O.load_serial();serial.storage(output);assert not output.exists()
    manifest=read(MANIFEST);ctx=context(serial);history=read(HISTORY)
    _,_,durations=O.inventory(HISTORY.parent/'pytest.xml')
    scope=manifest['collection_scope'] if mode=='collect' else (manifest['focus_scope'] if mode=='focus' else ctx['scope'])
    focus_refs={}
    if mode!='collect': assert manifest['full_scope_ready']
    if mode=='full':
        f=read(focus);assert f['passed'] and f['source_unchanged'] and f['scope_complete']
        assert f['mode']=='focus' and f['candidate_manifest_sha256']==MANIFEST_SHA
        assert f['source_sha256']==ctx['sources'] and f['runner_sha256']==sha(__file__)
        focus_refs[str(Path(focus).resolve())]=sha(focus)
        assert sha(Path(focus).parent/'pytest.xml')==f['xml_sha256']
        focus_refs[str(Path(focus).parent/'pytest.xml')]=f['xml_sha256']
    budget=O.resource_budget(argparse.Namespace(workers=workers,worker_cap=6,reserve_cpus=5,memory_per_worker_mib=768,reserve_memory_mib=2048))
    count=min(budget['selected_workers'],len(scope));budget['selected_workers']=count
    budget['budget_reason']='At most six one-thread workers; reserve five CPU slots and 2 GiB available RAM for online work. Historical XML LPT weights are estimates, not measured speedup.'
    assert budget['execution_resources_sufficient']
    expected=[] if mode=='collect' else [c for c in ctx['case_identities'] if c[0] in scope]
    counts=dict(Counter(c[0] for c in expected));assigned=[[] for _ in range(count)];loads=[0.]*count
    missing_weight=max(1.,sum(durations.values())/len(durations))
    for path in sorted(scope,key=lambda n:(-durations.get(n,missing_weight),n)):
        i=min(range(count),key=lambda j:(loads[j],j));assigned[i].append(path);loads[i]+=durations.get(path,missing_weight)
    shards=[dict(index=i,scope=names,test_files=len(names),required_changed_sources=([PRODUCTION] if NEW in names and mode!='collect' else []),
        expected_count=sum(counts.get(p,0) for p in names),historical_case_time_sec=loads[i],
        case_identities=[c for c in expected if c[0] in names]) for i,names in enumerate(assigned)]
    assert len(scope)==len(set(scope)) and sorted(p for s in shards for p in s['scope'])==sorted(scope)
    plan=dict(schema=1,kind='missing_depth_prior_'+mode+'_offline_plan',mode=mode,scope_kind=mode,
        candidate_manifest_path=str(MANIFEST),candidate_manifest_sha256=MANIFEST_SHA,required_changed_sources=[] if mode=='collect' else [PRODUCTION],
        focus_evidence_sha256=focus_refs,worker_count=count,resource_budget=budget,test_scope=scope,
        expected_file_count=len(scope),expected_case_count=len(expected),expected_counts_by_file=counts,
        full_regression_scope=mode=='full',runner_path=str(Path(__file__).resolve()),runner_sha256=sha(__file__),
        historical_result=str(HISTORY),historical_result_sha256=sha(HISTORY),historical_xml=str(HISTORY.parent/'pytest.xml'),
        historical_xml_sha256=sha(HISTORY.parent/'pytest.xml'),context=ctx,expected_case_identities=expected,
        shards=shards,output=str(output),maximum_workers=6,historical_result_elapsed_sec=history['elapsed_sec'],
        historical_worker_count=history['worker_count'],historical_scope_files=history['test_scope_files'],
        duration_estimate_scope='historical testcase execution time only',tests_executed=False,ROS_or_hardware_used=False)
    output.mkdir();save(output/'plan.json',plan);return plan


# This is a new explicit runner identity. The infrastructure file stays intact
# and separately hash-bound; child entrypoints always execute this wrapper.
O.__file__=str(Path(__file__).resolve())
O.current_context=context
O.stable=stable


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--freeze',action='store_true');p.add_argument('--candidate',type=Path)
    p.add_argument('--manifest',type=Path);p.add_argument('--manifest-sha256')
    p.add_argument('--close-manifest',action='store_true');p.add_argument('--collection',type=Path)
    p.add_argument('--output',type=Path);p.add_argument('--mode',choices=('collect','focus','full'),default='focus')
    p.add_argument('--workers',default='auto');p.add_argument('--focus-result',type=Path)
    p.add_argument('--execute',action='store_true');p.add_argument('--execute-plan',type=Path);p.add_argument('--plan-sha256')
    p.add_argument('--worker-plan',type=Path);p.add_argument('--worker-index',type=int)
    a=p.parse_args()
    if a.freeze:freeze(a.output.resolve(),a.candidate.resolve());return 0
    if a.worker_plan or a.execute_plan:
        path=(a.worker_plan or a.execute_plan).resolve();plan=read(path)
        if a.execute_plan:assert sha(path)==a.plan_sha256
        bind(plan['candidate_manifest_path'],plan['candidate_manifest_sha256'])
        if a.worker_plan:return O.worker(path,a.worker_index)
        return O.execute(O.load_serial(),path.parent,plan)
    bind(a.manifest,a.manifest_sha256)
    if a.close_manifest:close_manifest(a.output.resolve(),a.collection.resolve());return 0
    plan=make_plan(a.output.resolve(),a.mode,a.workers,a.focus_result)
    if a.execute:return O.execute(O.load_serial(),a.output.resolve(),plan)
    print(json.dumps(dict(plan=str(a.output/'plan.json'),sha256=sha(a.output/'plan.json'),case_count=plan['expected_case_count'],worker_count=plan['worker_count'])))
    return 0


if __name__=='__main__':raise SystemExit(main())
