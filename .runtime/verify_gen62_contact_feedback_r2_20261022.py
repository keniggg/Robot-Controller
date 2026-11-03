"""GEN62 frozen task-local contact feedback / bounded submission regression.

Retain all 10149 GEN61 case identities and all historical test/fixture bytes.
Only new tests are collected; reuse the immutable isolated executor and guards.
No installation, ROS initialization, hardware or network calls are authorized.
"""
import argparse
from collections import Counter
from copy import deepcopy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import time

W = Path(__file__).resolve().parents[1]
N = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z')
E = N/'gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1'
BASE = E
V = E/'v62'
CANDIDATE = W/'.runtime/gen62_contact_feedback_candidate_r1'
OLD = W/'.runtime/verify_gen57_parallel_offline_r2_20261020.py'
OLD_SHA = '0e217bc784b24c603afa1f4bca7c977f35f9036c03b6b263c9801f749d1c2727'
RELEASE = N/'verification/gen61_release_manifest.json'
HISTORY = BASE/'verification/full_r1/result.json'
BASE_PLAN = HISTORY.parent/'plan.json'
BASE_MANIFEST = BASE/'verification/freeze_r1/manifest_closed.json'
BASELINE = E/'contact_feedback_fix_r1/frozen_baseline_inputs_r1.json'
INVENTORY = E/'contact_feedback_verification_r1/baseline_inventory.json'
PILOT = E/'contact_feedback_verification_r1/pilot_r4_final_epoch/result.json'
P = 'src/alicia_grasp_modes/'
PRODUCTION = (P+'src/alicia_grasp_modes/contact_station_task.py',)
NEW_TESTS = (P+'tests/test_gen62_contact_feedback.py',)
NEW_FIXTURES = (P+'tests/fixtures/gen62_contact_feedback_recorded.json',)
FOCUS = list(NEW_TESTS) + [P+'tests/test_'+name+'.py' for name in (
    'contact_station_retirement','frozen_station_retirement_history',
    'gen11_contact_remote_receipt','gen11_contact_compute_admission_independent',
    'gen23_contact_station_param_types','gen35_minimal_chain_replay')]
REQUIRED_IMPORTS = {NEW_TESTS[0]:list(PRODUCTION)}
MANIFEST = None
MANIFEST_SHA = None
PINS = {Path(p): h for p, h in {'/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification/gen61_release_manifest.json': 'f233ba3a41dca9632a13e084b24c5a0c434c70ba0c28f38aeba369ecfb3c17e9', '/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/verification/full_r1/result.json': 'd1fc1834925993867cfca5a8c44b60e86c22fd4fd4223b7cdacfae366e0242e0', '/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/verification/full_r1/plan.json': '81303d3dff22b2afe11129392068185b0e4bc036d7b0e8e5bd9843f48ade5493', '/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/verification/full_r1/pytest.xml': '32c9703f21e133efffd2aac64217ada57b817c1e1b96a94ac727f8aa0d9a7c3f', '/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/verification/freeze_r1/manifest_closed.json': 'df01964c58063e6ee52b3d12c986fde49e57b4a7f0cf7c593596a2d453a68b02', '/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/verification/full_r1/final_process_receipt.json': '5958fa99e96a474396d3e7089f1f4c7c878da929c731bfbb8d931ee2debb3ef1', '/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade/.runtime/verify_gen57_parallel_offline_r2_20261020.py': '0e217bc784b24c603afa1f4bca7c977f35f9036c03b6b263c9801f749d1c2727', '/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade/.runtime/verify_gen61_seed_wait_20261022.py': '0eeb75cb557df3f0a7083252e3414460103bb715f45db79708ed05843c4c1b82', '/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/verification/full_r1/independent_verification.json': '9e8a73f36a8dbd2fe3c6ae7d9cb0e00ade400115150636acd0e8533858428ea9', '/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1/frozen_baseline_inputs_r1.json': 'a9ca68d367be4d60133c3f41f02d360cf632ac6978131f2af1c41cc466d5cba2', '/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_verification_r1/baseline_inventory.json': 'dcd88a7a0fb3b2a52c710d37c9e16e547a5020b232423242696ef31997281182', '/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_verification_r1/pilot_r4_final_epoch/result.json': '5d1b4988d1a473d2dfe90886b0cbcc2ee8bd5a3be2ef2e8a003d7c0d73618095'}.items()}

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes())


def save(path, value):
    with Path(path).open('x') as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n'); f.flush(); os.fsync(f.fileno())


assert sha(OLD) == OLD_SHA
spec = importlib.util.spec_from_file_location('frozen_gen61_isolation_infrastructure', OLD)
O = importlib.util.module_from_spec(spec)
spec.loader.exec_module(O)
original_load_serial = O.load_serial


def load_serial():
    serial = original_load_serial()
    # The original storage check is reused unchanged with a new finite output
    # root. All guard/provenance source and worker code bytes remain pinned.
    serial.V = V
    return serial


def freeze(output, candidate):
    assert candidate == CANDIDATE.resolve(strict=True) and candidate != W
    assert all(sha(p) == h for p,h in PINS.items()), 'Historical evidence changed.'
    release,history,plan,baseline,old_manifest = map(read,(RELEASE,HISTORY,BASE_PLAN,BASELINE,BASE_MANIFEST))
    assert release['generation'] == 'GEN61' and release['passed'] and release['installed']
    assert release['actual_regression_sha256'] == sha(HISTORY)
    assert release['candidate_manifest_sha256'] == sha(BASE_MANIFEST)
    assert history['passed'] and history['full_regression_passed'] and history['scope_complete']
    assert history['summary'] == dict(tests=10149,failures=0,errors=0,skipped=0)
    assert history['plan_sha256'] == sha(BASE_PLAN) and history['xml_sha256'] == sha(HISTORY.parent/'pytest.xml')
    original = release['source_sha256']
    assert len(original) == 1101 and history['test_scope_files'] == 354
    assert original == history['source_sha256'] == plan['context']['sources'] == old_manifest['source_sha256']
    assert baseline['candidate'] == str(candidate) and baseline['baseline'] == 'GEN61 accepted frozen candidate'
    assert {r['path']:r['sha256'] for r in baseline['files']} == old_manifest['copied_input_sha256']
    assert all(sha(W/p) == h for p,h in original.items()), 'Active GEN61 source changed.'
    current = {p:sha(candidate/p) for p in original}
    assert sorted(p for p in current if current[p] != original[p]) == sorted(PRODUCTION)
    assert all(p not in current for p in (*NEW_TESTS,*NEW_FIXTURES))
    current.update({p:sha(candidate/p) for p in (*NEW_TESTS,*NEW_FIXTURES)})
    assert len(current) == 1103
    summary,cases,_ = O.inventory(HISTORY.parent/'pytest.xml')
    assert summary == history['summary'] and len(cases) == len(set(map(tuple,cases))) == 10149
    assert sorted(map(tuple,cases)) == sorted(map(tuple,plan['expected_case_identities']))
    inventory = read(INVENTORY)
    assert sorted(map(tuple,cases)) == sorted(map(tuple,inventory['case_identities']))
    assert inventory['baseline_source_sha256'] == original
    copied = dict(current)
    for relative,digest in old_manifest['copied_input_sha256'].items():
        if relative not in original:
            assert sha(candidate/relative) == digest
            copied[relative] = digest
    actual = {str(p.relative_to(candidate)) for p in candidate.rglob('*') if p.is_file()}
    assert actual == set(copied), dict(unexpected=sorted(actual-set(copied)),missing=sorted(set(copied)-actual))
    pilot = read(PILOT)
    assert pilot['passed'] and pilot['source_unchanged'] and pilot['scope_complete']
    assert sha(pilot['manifest_path']) == pilot['manifest_sha256']
    assert read(pilot['manifest_path'])['copied_input_sha256'] == copied
    assert pilot['summary'] == dict(tests=pilot['new_case_count'],failures=0,errors=0,skipped=0)
    serial = load_serial()
    assert serial.E.parent.parent.is_mount()
    assert E.stat().st_dev == serial.E.parent.parent.stat().st_dev
    V.mkdir(exist_ok=True); serial.storage(output); assert not output.exists()
    dependencies = dict(plan['context']['compatibility_dependencies'])
    dependencies.update({str(p):h for p,h in PINS.items()})
    context = deepcopy(plan['context']); scope = list(context['scope'])+list(NEW_TESTS)
    assert len(scope) == len(set(scope)) == 355 and set(FOCUS) <= set(scope)
    context.update(scope=scope,sources=current,case_identities=cases,old_case_identities=deepcopy(cases),
        expected_counts=dict(Counter(c[0] for c in cases)),old_expected_counts=dict(Counter(c[0] for c in cases)),
        required_changed_sources=list(PRODUCTION),required_imports_by_test_file=REQUIRED_IMPORTS,
        new_fixture_sources=list(NEW_FIXTURES),original=original,baseline_sha256=sha(RELEASE),
        compatibility_dependencies=dependencies,full_scope_ready=False,
        compatibility=dict(variant='candidate',baseline_generation='GEN61',baseline_case_count=10149,
            production_source_changes=[dict(path=p,baseline_sha256=original[p],candidate_sha256=current[p]) for p in PRODUCTION],
            explicit_old_test_contract_changes=[],other_historical_test_and_fixture_bytes_unchanged=True,
            new_tests=list(NEW_TESTS),new_fixtures=list(NEW_FIXTURES)))
    manifest = dict(kind='GEN62_contact_feedback_frozen_offline_candidate',created_ns=time.time_ns(),workspace_root=str(candidate),
        runner_sha256=sha(__file__),infrastructure_path=str(OLD),infrastructure_sha256=OLD_SHA,
        source_sha256=current,copied_input_sha256=copied,context=context,focus_scope=FOCUS,
        revision=1,baseline_live_source_sha256=original,collection_scope=list(NEW_TESTS),
        baseline_case_count=10149,baseline_source_count=1101,explicit_case_identity_mapping=[],
        output_root=str(V),storage_scope_rebound_only=True,full_scope_ready=False,
        full_regression_passed=False,production_deployed=False,active_source_modified=False)
    output.mkdir(); save(output/'manifest_uncollected.json',manifest)
    print(str(output/'manifest_uncollected.json'))


def bind(path, expected):
    global MANIFEST, MANIFEST_SHA
    MANIFEST = Path(path).resolve(strict=True); MANIFEST_SHA = expected
    assert sha(MANIFEST) == expected
    manifest = read(MANIFEST)
    assert manifest['kind'] == 'GEN62_contact_feedback_frozen_offline_candidate'
    assert manifest['runner_sha256'] == sha(__file__) and manifest['infrastructure_sha256'] == sha(OLD) == OLD_SHA
    assert Path(manifest['workspace_root']).resolve(strict=True) == CANDIDATE.resolve(strict=True)
    O.W = CANDIDATE.resolve(strict=True); O.MANIFEST_PATH = MANIFEST; O.MANIFEST_SHA = MANIFEST_SHA
    return manifest


def context(serial):
    manifest = read(MANIFEST)
    assert sha(MANIFEST) == MANIFEST_SHA and manifest['runner_sha256'] == sha(__file__)
    assert all(sha(O.W/p) == h for p,h in manifest['copied_input_sha256'].items())
    assert all(sha(W/p) == h for p,h in manifest['baseline_live_source_sha256'].items())
    assert all(sha(p) == h for p,h in manifest['context']['compatibility_dependencies'].items())
    assert all(sha(p) == h for p,h in manifest['context']['generated_imports'].items())
    return deepcopy(manifest['context'])


def stable(serial, plan):
    return (sha(__file__) == plan['runner_sha256'] and sha(OLD) == OLD_SHA
        and sha(MANIFEST) == MANIFEST_SHA == plan['candidate_manifest_sha256']
        and all(sha(p) == h for p,h in plan.get('focus_evidence_sha256',{}).items())
        and context(serial) == plan['context'])


def close_manifest(output, collection):
    load_serial().storage(output)
    manifest, data = read(MANIFEST), read(collection)
    assert data['mode'] == 'collect' and data['passed'] and data['source_unchanged'] and data['scope_complete']
    assert data['candidate_manifest_sha256'] == MANIFEST_SHA and data['runner_sha256'] == sha(__file__)
    assert data['source_sha256'] == manifest['source_sha256']
    cases = data['collected_case_identities']
    assert cases and len(cases) == len(set(map(tuple,cases)))
    assert set(c[0] for c in cases) == set(manifest['collection_scope'])
    assert not set(map(tuple,cases)) & set(map(tuple,manifest['context']['case_identities']))
    assert sorted(map(tuple,cases)) == sorted(map(tuple,read(PILOT)['case_identities']))
    updated = deepcopy(manifest); c = updated['context']; c['case_identities'] += cases
    c['expected_counts'].update(Counter(row[0] for row in cases)); c['full_scope_ready'] = True
    assert len(c['case_identities']) == len(set(map(tuple,c['case_identities']))) == 10149 + len(cases)
    assert set(c['expected_counts']) == set(c['scope'])
    c['compatibility_dependencies'][str(Path(collection).resolve())] = sha(collection)
    c['compatibility_dependencies'][str(MANIFEST)] = MANIFEST_SHA
    updated.update(full_scope_ready=True,collection_path=str(collection),collection_sha256=sha(collection),
        total_case_count=len(c['case_identities']),new_case_count=len(cases))
    save(output,updated); print(str(output))


def make_plan(output, mode, workers, focus):
    serial = load_serial(); serial.storage(output); assert not output.exists()
    manifest = read(MANIFEST); ctx = context(serial); history = read(HISTORY)
    _,_,durations = O.inventory(HISTORY.parent/'pytest.xml')
    scope = manifest['collection_scope'] if mode == 'collect' else (manifest['focus_scope'] if mode == 'focus' else ctx['scope'])
    focus_refs = {}
    if mode != 'collect': assert manifest['full_scope_ready']
    if mode == 'full':
        f = read(focus)
        assert f['passed'] and f['source_unchanged'] and f['scope_complete'] and f['mode'] == 'focus'
        assert f['candidate_manifest_sha256'] == MANIFEST_SHA and f['source_sha256'] == ctx['sources'] and f['runner_sha256'] == sha(__file__)
        assert all(f['loaded_changed_sources_global'][p] for p in PRODUCTION)
        focus_refs[str(Path(focus).resolve())] = sha(focus)
        assert sha(Path(focus).parent/'pytest.xml') == f['xml_sha256']
        focus_refs[str(Path(focus).parent/'pytest.xml')] = f['xml_sha256']
    budget = O.resource_budget(argparse.Namespace(workers=workers,worker_cap=6,reserve_cpus=5,memory_per_worker_mib=768,reserve_memory_mib=2048))
    count = min(budget['selected_workers'],len(scope)); budget['selected_workers'] = count
    budget['budget_reason'] = 'At most six one-thread workers; reserve five CPU slots and 2 GiB available RAM. Historical XML LPT weights are estimates, not measured speedup.'
    assert budget['execution_resources_sufficient']
    expected = [] if mode == 'collect' else [c for c in ctx['case_identities'] if c[0] in scope]
    counts = dict(Counter(c[0] for c in expected)); assigned = [[] for _ in range(count)]; loads = [0.]*count
    missing_weight = max(1.,sum(durations.values())/len(durations))
    for path in sorted(scope,key=lambda n:(-durations.get(n,missing_weight),n)):
        i = min(range(count),key=lambda j:(loads[j],j)); assigned[i].append(path); loads[i] += durations.get(path,missing_weight)
    shards = [dict(index=i,scope=names,test_files=len(names),
        required_changed_sources=[] if mode == 'collect' else sorted({p for test in names for p in REQUIRED_IMPORTS.get(test,[])}),
        expected_count=sum(counts.get(p,0) for p in names),historical_case_time_sec=loads[i],
        case_identities=[c for c in expected if c[0] in names]) for i,names in enumerate(assigned)]
    assert len(scope) == len(set(scope)) and sorted(p for s in shards for p in s['scope']) == sorted(scope)
    plan = dict(schema=1,kind='GEN62_contact_feedback_'+mode+'_offline_plan',mode=mode,scope_kind=mode,
        candidate_manifest_path=str(MANIFEST),candidate_manifest_sha256=MANIFEST_SHA,
        required_changed_sources=[] if mode == 'collect' else list(PRODUCTION),
        focus_evidence_sha256=focus_refs,worker_count=count,resource_budget=budget,test_scope=scope,
        expected_file_count=len(scope),expected_case_count=len(expected),expected_counts_by_file=counts,
        full_regression_scope=mode == 'full',runner_path=str(Path(__file__).resolve()),runner_sha256=sha(__file__),
        historical_result=str(HISTORY),historical_result_sha256=sha(HISTORY),historical_xml=str(HISTORY.parent/'pytest.xml'),
        historical_xml_sha256=sha(HISTORY.parent/'pytest.xml'),context=ctx,expected_case_identities=expected,
        shards=shards,output=str(output),maximum_workers=6,historical_result_elapsed_sec=history['elapsed_sec'],
        historical_worker_count=history['worker_count'],historical_scope_files=history['test_scope_files'],
        duration_estimate_scope='historical testcase execution time only',tests_executed=False,ROS_or_hardware_used=False,
        result_kind_note='Reused immutable executor retains GEN57_parallel_* result kind; this GEN62 runner/manifest/source/10149+new scope supplies actual identity.')
    output.mkdir(); save(output/'plan.json',plan); return plan


O.__file__ = str(Path(__file__).resolve())
O.load_serial = load_serial
O.current_context = context
O.stable = stable


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--freeze',action='store_true'); p.add_argument('--candidate',type=Path,default=CANDIDATE)
    p.add_argument('--manifest',type=Path); p.add_argument('--manifest-sha256')
    p.add_argument('--close-manifest',action='store_true'); p.add_argument('--collection',type=Path)
    p.add_argument('--output',type=Path); p.add_argument('--mode',choices=('collect','focus','full'),default='focus')
    p.add_argument('--workers',default='auto'); p.add_argument('--focus-result',type=Path)
    p.add_argument('--execute',action='store_true'); p.add_argument('--execute-plan',type=Path); p.add_argument('--plan-sha256')
    p.add_argument('--worker-plan',type=Path); p.add_argument('--worker-index',type=int)
    a = p.parse_args()
    if a.freeze: freeze(a.output.resolve(),a.candidate.resolve(strict=True)); return 0
    if a.worker_plan or a.execute_plan:
        path = (a.worker_plan or a.execute_plan).resolve(); plan = read(path)
        if a.execute_plan: assert sha(path) == a.plan_sha256
        bind(plan['candidate_manifest_path'],plan['candidate_manifest_sha256'])
        if a.worker_plan: return O.worker(path,a.worker_index)
        return O.execute(load_serial(),path.parent,plan)
    bind(a.manifest,a.manifest_sha256)
    if a.close_manifest: close_manifest(a.output.resolve(),a.collection.resolve()); return 0
    plan = make_plan(a.output.resolve(),a.mode,a.workers,a.focus_result)
    if a.execute: return O.execute(load_serial(),a.output.resolve(),plan)
    print(json.dumps(dict(plan=str(a.output/'plan.json'),sha256=sha(a.output/'plan.json'),case_count=plan['expected_case_count'],worker_count=plan['worker_count'])))
    return 0


if __name__ == '__main__': raise SystemExit(main())
