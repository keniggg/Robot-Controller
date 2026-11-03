"""GEN57: manifest-bound configurable offline pytest; plan-only default.

This helper is outside the current release. It does not install/deploy anything
or run on import. Focus scope comes from the hash-bound candidate manifest.
Full mode requires its explicit full_scope_ready flag and closed case inventory.
Every child has separate temporary, ROS-log, Python guard, import-provenance and
XML files. The coordinator requires exact testcase identities, not just totals.
It preserves the frozen GEN56 scheduler/guard/provenance contracts. All 9658
GEN56 cases and every old test/fixture byte remain fixed; only explicit
production deltas and collected GEN57 additions are eligible.
"""
import argparse
from collections import Counter
from copy import deepcopy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import resource
import subprocess
import sys
import time
import traceback
import xml.etree.ElementTree as ET

PROJECT = Path(__file__).resolve().parents[1]
W = PROJECT
MANIFEST_PATH = None
MANIFEST_SHA = None
SERIAL = W/'.runtime/verify_gen43_offline_tests_20261018_r2.py'
SERIAL_SHA = 'de1b3727a78644ade82690dbfd748745c4e03969eda18556bd1192b54c55698f'
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
HISTORY = V/'gen56_full_parallel_r2/result.json'
HISTORY_SHA = 'e1877591d341b05af82102fbe46b27a8e3d3c71cb47886f988104bd6d5fc6b9b'
CALIBRATION = V/'gen43_two_process_pilot_monitor_r1/verification.json'
CALIBRATION_SHA = 'e8361d8a905351de2be64cd259008c3bc97193f4a0ec1cb16bf26306ceb3066b'
PRIOR_TWO_WORKER = W/'.runtime/verify_gen43_two_process_draft_20261018.py'
PRIOR_TWO_WORKER_SHA = '82c6efbf6947577c561ea7427d4dd8fd346b16f1a7ee0f5302d384751564be3b'
BASELINE = PROJECT/'.runtime/gen57_baseline_source/runner_manifest.json'
BASELINE_SHA = 'f570485f6f460648fc1bdc62c4804ed9ed69082e43b4b982273fcb3a57a1da3e'
BASELINE_RELEASE = V/'gen56_release_manifest.json'
BASELINE_RELEASE_SHA = '784af71745650269b1d114a5f9cbba32eaa792f8cf2029825eb5a56d52a8f255'
BASELINE_XML_SHA = 'd58c113a4c3d5bc75df9ffc004cad0fe54057965043bfe4508726fa569fa5116'
FROZEN_PREDECESSOR = PROJECT/'.runtime/verify_gen56_parallel_offline_20261020.py'
FROZEN_PREDECESSOR_SHA = 'bac8feee8fe84431b50598b5a40adf305e8542dabedc5b99c1b5cbec517cc4ee'
CANDIDATE = PROJECT/'.runtime/gen57_candidate_r1'
BALANCE_HISTORY = HISTORY
BALANCE_HISTORY_SHA = HISTORY_SHA
SUPPLEMENTAL_CAD_INPUTS = {
    'src/arm-mujoco/synriard/urdf/Alicia_D_v5_6/Alicia_D_v5_6_gripper_50mm.urdf': '7e08a01f1a6cf3b45ed78a26d3a9f5939e2bb6bc33f6bc01032db097882edaeb',
    'src/arm-mujoco/synriard/meshes/Alicia_D_v5_6/gripper_50mm/Link6.STL': 'cff46c5522d1a869b0397965adb885a7f4c62c5b137b409d8df605aab3cc3d5d',
    'src/arm-mujoco/synriard/meshes/Alicia_D_v5_6/gripper_50mm/Link7.STL': 'd546b7ab908c74281a41261412089a2f83f7519c2444739afe8ffff46450d19d',
    'src/arm-mujoco/synriard/meshes/Alicia_D_v5_6/gripper_50mm/Link8.STL': '5e5b548734269b731c1d60502c742e3d70d5a97a1e8316db523e2232b4c5043b',
    'src/real-arm/alicia_d_descriptions/urdf/alicia_duo_with_gripper.urdf': '890592a24b8ad818d995e69beb670ea4254bf84f589615bfac3c8562f62449b7',
    'src/real-arm/alicia_d_descriptions/meshes/Link7.STL': 'ff006151147d25c431d868fc6a0f46b96275690d30526d13c741ffe51ac6fcea',
    'src/real-arm/alicia_d_descriptions/meshes/Link8.STL': 'fd49b1a4c9ecf39854d0de1ee3645239aae2427cca6ce7a24b195bf041c6098e',
    'src/real-arm/alicia_d_moveit/config/alicia_d_descriptions.srdf': 'ac14d9a4de083d30444b3c8e762bc166a61382ec334ac455a93ac792ae6a42c2',
}
SUPPLEMENTAL_EXECUTABLE_PROOF = V/'gen56_dependency_repair_r1/supplemental_assets.json'
SUPPLEMENTAL_EXECUTABLE_PROOF_SHA = '695ddce04558920e2b939edb75db3e637eee5927832134eff29fbe1f1fd0928d'
SUPPLEMENTAL_EXECUTABLE_INPUTS = (
    dict(relative='.runtime/gen38_baseline_source/src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/multiview_surface.py',
         sha256='369a2947114c6c3953c439a7937cf26971e337c015e65abf96825720bc5c1ba8',
         label='alicia_flexible_grasp.vision._gen38_registration_test_baseline',
         test='src/alicia_flexible_grasp_supervisor/tests/test_gen39_registration_transform.py',
         test_sha256='fcd9fe03951fc4761f1faaf486b1ccf979583cb1cd07be39f9912845a5bd7c71',
         case_name='test_default_icp_result_matches_gen38_every_field_exactly[registered]'),
    dict(relative='tools/replay_observation_following_readonly.py',
         sha256='8a17a00d9439d818e00c4e332fd018be38111e2206f2ede00e66a5d58627503f',
         label='loader:following_replay',
         test='src/alicia_flexible_grasp_supervisor/tests/test_observation_following_support.py',
         test_sha256='9bb3a6d90e67c862468dc605938d520d4424060401069376f20e862109046c9d',
         case_name='test_closed_bag_replay_rejects_failed_branch_and_has_a_passing_alternative'),
)


def bind_manifest(path, expected_sha):
    global MANIFEST_PATH,MANIFEST_SHA,W
    MANIFEST_PATH=Path(path).resolve(strict=True);MANIFEST_SHA=expected_sha
    assert sha(MANIFEST_PATH)==MANIFEST_SHA
    manifest=read(MANIFEST_PATH)
    assert manifest['kind']=='GEN57_frozen_offline_candidate'
    W=Path(manifest['workspace_root']).resolve(strict=True)
    assert W in (PROJECT, CANDIDATE), 'Only the explicit GEN57 candidate or exact installed workspace is accepted.'
    assert manifest['runner_sha256'] == sha(__file__)
    return manifest


def reference(row):
    path=Path(row['path']).resolve(strict=True)
    assert sha(path)==row['sha256']
    return path


def resource_budget(args):
    assert 1<=args.worker_cap<=8 and args.reserve_cpus>=4
    assert args.memory_per_worker_mib>0 and args.reserve_memory_mib>=0
    affinity=sorted(os.sched_getaffinity(0))
    quota_cores=None;quota={}
    v2=Path('/sys/fs/cgroup/cpu.max')
    v1=Path('/sys/fs/cgroup/cpu/cpu.cfs_quota_us')
    if v2.exists():
        value,period=v2.read_text().split();quota=dict(path=str(v2),value=value,period=int(period))
        if value!='max':quota_cores=int(value)/int(period)
    elif v1.exists():
        value=int(v1.read_text());period=int(v1.with_name('cpu.cfs_period_us').read_text());quota=dict(path=str(v1),value=value,period=period)
        if value>0:quota_cores=value/period
    effective=len(affinity) if quota_cores is None else min(len(affinity),math.floor(quota_cores))
    cpu_workers=max(0,effective-args.reserve_cpus)
    mem={k:int(v.split()[0]) for k,v in (l.split(':',1) for l in Path('/proc/meminfo').read_text().splitlines())}
    available_mib=mem['MemAvailable']/1024.
    memory_workers=max(0,math.floor((available_mib-args.reserve_memory_mib)/args.memory_per_worker_mib))
    capacity=min(args.worker_cap,cpu_workers,memory_workers)
    selected=max(1,capacity) if args.workers=='auto' else int(args.workers)
    assert 1<=selected<=args.worker_cap
    calibration=read(CALIBRATION);assert sha(CALIBRATION)==CALIBRATION_SHA
    observed=max(float(p['peak_rss_kib'].split()[0])/1024. for key in ('first_sample','second_sample')
        for p in calibration['monitoring'][key]['processes'] if 'peak_rss_kib' in p)
    return dict(requested_workers=args.workers,selected_workers=selected,worker_cap=args.worker_cap,
        affinity_cpus=affinity,os_cpu_count=os.cpu_count(),cgroup_quota=quota,effective_cpu_count=effective,
        reserved_cpu_count=args.reserve_cpus,cpu_worker_capacity=cpu_workers,
        mem_total_mib=mem['MemTotal']/1024.,mem_available_mib=available_mib,
        reserved_memory_mib=args.reserve_memory_mib,memory_per_worker_mib=args.memory_per_worker_mib,
        memory_worker_capacity=memory_workers,resource_worker_capacity=capacity,
        execution_resources_sufficient=(selected<=capacity),load_average=list(os.getloadavg()),
        measured_pilot_peak_rss_mib=observed,calibration_path=str(CALIBRATION),calibration_sha256=CALIBRATION_SHA,
        budget_reason='One CPU per worker; leave four default CPU slots for online control. Per-worker 768MiB default exceeds observed pilot peak about315MiB; reserve an additional2GiB of current MemAvailable. These are planning margins, not a full-suite peak guarantee.')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes())


def save(path, value):
    with Path(path).open('x') as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n'); f.flush(); os.fsync(f.fileno())


def load_serial():
    assert sha(SERIAL) == SERIAL_SHA
    spec = importlib.util.spec_from_file_location('gen43_frozen_serial_contract', SERIAL)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)  # Definitions only: guarded main is not called.
    return module


def bootstrap_sources(serial):
    guard, audit = serial.bootstrap_sources()
    original = "'remote_grasp6d' in name or name.startswith('alicia_grasp_modes')"
    assert audit.count(original) == 1
    audit = audit.replace(original,
        "'remote_grasp6d' in name or Path(path).name in ('grasp_task_node.py', 'mode_aware_grasp_task.py', 'mode_aware_remote_grasp6d.py', 'unknown_tabletop_perception.py') or name.startswith('alicia_grasp_modes')")
    # This historical test keeps its loader-created module only in a local
    # variable. Retain actual successful loader evidence, never infer a load
    # from the expected asset path. All other imports retain the frozen audit.
    assert audit.count('    rows={}\n') == 1
    audit = audit.replace('    rows={}\n', '    rows=dict(_gen56_following_loads)\n')
    audit += '''
import importlib.machinery
_gen56_following_loads = {}
_gen56_original_exec_module = importlib.machinery.SourceFileLoader.exec_module
def _gen56_capture_following_loader(loader, module):
    result = _gen56_original_exec_module(loader, module)
    if getattr(module, '__name__', None) == 'following_replay' or loader.name == 'following_replay':
        path = Path(module.__file__).resolve()
        label = 'loader:following_replay'
        if _gen56_following_loads:
            label += '#' + str(len(_gen56_following_loads))
        _gen56_following_loads[label] = dict(path=str(path),
            sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            module_name=module.__name__, loader_name=loader.name,
            loader_path=str(Path(loader.path).resolve()))
    return result
importlib.machinery.SourceFileLoader.exec_module = _gen56_capture_following_loader
'''
    return guard, audit


def current_context(serial):
    assert MANIFEST_PATH is not None and sha(MANIFEST_PATH) == MANIFEST_SHA
    manifest = read(MANIFEST_PATH)
    assert manifest['runner_sha256'] == sha(__file__)
    assert sha(FROZEN_PREDECESSOR) == FROZEN_PREDECESSOR_SHA
    assert reference(manifest['baseline_manifest']) == BASELINE and sha(BASELINE) == BASELINE_SHA
    baseline = read(BASELINE); original = baseline['source_sha256']
    assert baseline['kind'] == 'GEN57_exact_GEN56_baseline_source'
    assert baseline['baseline_generation'] == 'GEN56'
    assert Path(baseline['workspace_root']).resolve(strict=True) == BASELINE.parent
    assert Path(baseline['live_workspace']).resolve(strict=True) == PROJECT
    assert Path(baseline['candidate_root']).resolve(strict=True) == CANDIDATE
    assert baseline['source_count'] == len(original) == 1085
    assert baseline['test_scope_files'] == 339 and baseline['all_source_bytes_exact'] is True
    assert Path(baseline['original_full_regression']).resolve(strict=True) == HISTORY
    assert baseline['original_full_regression_sha256'] == sha(HISTORY) == HISTORY_SHA
    assert Path(baseline['baseline_release']).resolve(strict=True) == BASELINE_RELEASE
    assert baseline['baseline_release_sha256'] == sha(BASELINE_RELEASE) == BASELINE_RELEASE_SHA
    assert all(sha(BASELINE.parent/p) == h for p,h in original.items())
    sealed = read(BASELINE_RELEASE); history = read(HISTORY); xml = HISTORY.parent/'pytest.xml'
    assert sealed['generation'] == 'GEN56' and sealed['passed'] is True
    assert sealed['source_sha256'] == history['source_sha256'] == original
    assert history['passed'] is True and history['full_regression_passed'] is True
    assert history['source_unchanged'] is True and history['scope_complete'] is True
    assert history['runner_sha256'] == FROZEN_PREDECESSOR_SHA
    assert sha(xml) == history['xml_sha256'] == BASELINE_XML_SHA
    summary, old_cases, _ = inventory(xml); counts = dict(Counter(c[0] for c in old_cases))
    assert summary == baseline['test_summary'] == dict(tests=9658,failures=0,errors=0,skipped=0)
    assert len(counts) == 339 and counts == history['test_case_counts_by_file']
    baseline_scope = list(history['test_case_counts_by_file'])
    assert set(baseline_scope) <= set(original)
    variant = manifest.get('variant','candidate'); assert variant in ('candidate','baseline')
    allowed = set(manifest['allowed_production_paths'])
    assert len(allowed) == len(manifest['allowed_production_paths'])
    assert (allowed or manifest.get('new_production_sources') or variant == 'baseline')
    assert all(p in original and '/tests/' not in p and p.startswith('src/') and p.endswith('.py') for p in allowed)
    expected = dict(original); changes = manifest['production_source_changes']; seen = set()
    for row in changes:
        path = row['path']; assert path in allowed and path not in seen
        seen.add(path); assert row['baseline_sha256'] == original[path]
        assert row['candidate_sha256'] != original[path] and valid_digest(row['candidate_sha256'])
        if variant == 'candidate': expected[path] = row['candidate_sha256']
    assert seen == allowed
    # No historical test or fixture override survives into this generation.
    for key in ('approved_fixture_changes','approved_old_test_changes','approved_gen55_test_changes','approved_gen56_test_changes','approved_gen57_test_changes','supplemental_existing_tests'):
        assert not manifest.get(key), 'GEN57 preserves every GEN56 test/fixture byte: '+key
    dependencies = {str(p):sha(p) for p in (BASELINE,BASELINE_RELEASE,HISTORY,xml,FROZEN_PREDECESSOR)}
    new_production = manifest.get('new_production_sources',[]); new_production_paths = set()
    for row in new_production:
        path = row['path']; parsed = Path(path)
        assert variant == 'candidate' and path.startswith('src/') and '/src/' in path and path.endswith('.py')
        assert '/tests/' not in path and not parsed.is_absolute() and '..' not in parsed.parts
        assert path not in original and path not in expected and path not in new_production_paths
        assert valid_digest(row['sha256'])
        evidence = reference(row['review_evidence']); review = read(evidence)
        assert review['passed'] is True and review['owned_source_sha256'][path] == row['sha256']
        dependencies[str(evidence)] = sha(evidence)
        new_production_paths.add(path); expected[path] = row['sha256']
    changed_or_new = allowed | new_production_paths
    new = manifest.get('new_tests',[]); new_paths = []; new_cases = []; expected_counts = dict(counts)
    for row in new:
        path = row['path']; parsed = Path(path)
        assert path not in original and path not in expected and path not in new_paths
        assert not parsed.is_absolute() and '..' not in parsed.parts
        assert any(path.startswith(root+'/test_gen57_') and path.endswith('.py') for root in serial.ROOTS)
        assert valid_digest(row['sha256'])
        new_paths.append(path); expected[path] = row['sha256']
        cases = row.get('case_identities')
        if cases is not None:
            assert type(row['expected_count']) is int and len(cases) == row['expected_count'] > 0
            assert all(len(c) == 3 and c[0] == path and all(isinstance(v,str) for v in c) for c in cases)
            assert len(cases) == len(set(map(tuple,cases)))
            expected_counts[path] = row['expected_count']; new_cases += cases
            evidence = reference(row['collection_evidence']); dependencies[str(evidence)] = sha(evidence)
            collection = read(evidence)
            assert collection['mode'] == 'collect' and collection['passed'] is True and collection['source_unchanged'] is True
            assert collection['runner_sha256'] == sha(__file__)
            assert collection['source_sha256'] == manifest['source_sha256']
            assert not collection['test_bodies_executed'] and not collection['full_regression_passed']
            assert not collection['external_boundary_attempts'] and not collection['foreign_or_inconsistent_imports']
            assert sorted(map(tuple,cases)) == sorted(tuple(c) for c in collection['collected_case_identities'] if c[0] == path)
    data_fixtures = manifest.get('new_fixture_sources',[])
    for row in data_fixtures:
        path = row['path']; parsed = Path(path)
        assert any(path.startswith(root+'/fixtures/') for root in serial.ROOTS)
        assert not parsed.is_absolute() and '..' not in parsed.parts
        assert parsed.suffix.lower() not in ('.py','.pyc','.pyo','.so','.sh','.bash')
        assert path not in original and path not in expected and valid_digest(row['sha256'])
        expected[path] = row['sha256']
    assert manifest['source_sha256'] == expected
    assert all(not Path(p).is_absolute() and '..' not in Path(p).parts for p in expected)
    sources = {p:sha(W/p) for p in expected}; assert sources == expected
    scope = baseline_scope+new_paths; assert len(scope) == len(set(scope))
    all_cases = old_cases+new_cases; assert len(all_cases) == len(set(map(tuple,all_cases)))
    if manifest.get('full_scope_ready'):
        assert type(manifest['full_scope_ready']) is bool and new
        assert all('case_identities' in row for row in new)
        assert set(map(tuple,old_cases)) <= set(map(tuple,all_cases))
    required = manifest.get('required_changed_sources',sorted(changed_or_new))
    assert len(required) == len(set(required)) and set(required) == changed_or_new
    per_file = manifest.get('required_imports_by_test_file',{})
    assert set(per_file) <= set(scope) and all(set(v) <= changed_or_new for v in per_file.values())
    for row in manifest.get('additional_review_evidence',[]):
        evidence = reference(row); dependencies[str(evidence)] = sha(evidence)
    # Baseline CAD tests and historical AST comparators are read-only data,
    # not a live-source import fallback. Pin project and candidate assets.
    for relative,digest in SUPPLEMENTAL_CAD_INPUTS.items():
        assert relative not in expected
        for root in (PROJECT,W):
            path = root/relative
            assert path.resolve(strict=True) == path and sha(path) == digest
            dependencies[str(path)] = digest
    provenance = V/'gen43_combined_r2/module_provenance.json'
    historical_provenance_result = V/'gen43_combined_r2/result.json'
    assert sha(historical_provenance_result) == 'ac2a76cc400ffc644c02ad6175557fa905da94f54d72abbe413e838f5a50e299'
    assert sha(provenance) == read(historical_provenance_result)['module_provenance_sha256']
    dependencies[str(historical_provenance_result)] = sha(historical_provenance_result)
    dependencies[str(provenance)] = sha(provenance)
    generated = {r['path']:r['sha256'] for r in read(provenance).values() if serial.inside(Path(r['path']),PROJECT/'devel')}
    historical_ast_inputs = {'.runtime/gen37_baseline_source/src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_path_guard.py': 'f3af52527035ad8211e785e3aeb14ecd596570bf236a989662547fd9ce73de9a', '.runtime/gen37_baseline_source/src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/contact_sequence_precheck.py': 'dba703313e5bde18ef74b300f01d15f69132bc0ccbae2a3855eaf51e7e976f88'}
    for relative,digest in historical_ast_inputs.items():
        path = W/relative; assert sha(path) == digest
        dependencies[str(path.resolve(strict=True))] = digest
    assert sha(SUPPLEMENTAL_EXECUTABLE_PROOF) == SUPPLEMENTAL_EXECUTABLE_PROOF_SHA
    dependencies[str(SUPPLEMENTAL_EXECUTABLE_PROOF)] = SUPPLEMENTAL_EXECUTABLE_PROOF_SHA
    executable_inputs = []
    for item in SUPPLEMENTAL_EXECUTABLE_INPUTS:
        assert item['relative'] not in expected
        assert original[item['test']] == sources[item['test']] == item['test_sha256']
        cases = [c for c in old_cases if c[0] == item['test'] and c[2] == item['case_name']]
        assert len(cases) == 1
        for root in (PROJECT,W):
            path = root/item['relative']
            assert path.resolve(strict=True) == path and sha(path) == item['sha256']
            dependencies[str(path)] = item['sha256']
        executable_inputs.append(dict(item,case_identity=cases[0]))
    guard,audit = bootstrap_sources(serial)
    return dict(scope=scope,sources=sources,expected_counts=expected_counts,
        case_identities=all_cases,old_case_identities=old_cases,old_expected_counts=counts,
        new_fixture_sources=data_fixtures,required_changed_sources=required,
        required_imports_by_test_file=per_file,generated_imports=generated,
        supplemental_executable_inputs=executable_inputs,
        compatibility=dict(variant=variant,production_source_changes=changes,new_production_sources=new_production,
            approved_old_test_changes={},approved_gen55_test_changes=[]),
        compatibility_dependencies=dependencies,full_scope_ready=manifest.get('full_scope_ready',False),
        original=original,baseline_sha256=BASELINE_SHA,
        guard_sha256=hashlib.sha256(guard.encode()).hexdigest(),audit_sha256=hashlib.sha256(audit.encode()).hexdigest())


def supplemental_import_requirements(context, shard):
    """Exact executable evidence required only in each owning complete-file shard."""
    required = {}
    for item in context['supplemental_executable_inputs']:
        if item['test'] not in shard['scope']:
            continue
        assert item['case_identity'] in shard['case_identities']
        assert context['sources'][item['test']] == sha(W/item['test']) == item['test_sha256']
        path = W/item['relative']
        assert path.resolve(strict=True) == path and sha(path) == item['sha256']
        row = dict(path=str(path),sha256=item['sha256'])
        if item['label'] == 'loader:following_replay':
            row.update(module_name='following_replay',loader_name='following_replay',loader_path=str(path))
        required[item['label']] = row
    return required


def supplemental_load_evidence(context, shard, provenance):
    required = supplemental_import_requirements(context,shard)
    return required, {label:provenance.get(label) == row for label,row in required.items()}


def valid_digest(value):
    return isinstance(value,str) and len(value) == 64 and all(c in '0123456789abcdef' for c in value)


def validate_focus_evidence(manifest,context):
    # A closed-inventory/full-ready manifest has a new metadata SHA. Its prior
    # focus manifest remains immutable and must bind the SAME source bytes.
    path = reference(manifest['focus_evidence']); result = read(path)
    assert result['mode'] == 'focus' and result['passed'] is True
    assert result['scope_complete'] is True and result['source_unchanged'] is True
    assert result['test_bodies_executed'] is True and not result['full_regression_passed']
    assert result['runner_sha256'] == sha(__file__) and result['source_sha256'] == context['sources']
    assert not result['external_boundary_attempts'] and not result['foreign_or_inconsistent_imports']
    assert not result['interrupted'] and not result['launcher_error']
    assert not result['offline_child_cleanup_required'] and not result['residual_offline_process_groups']
    assert not result['missing_worker_results'] and not result['duplicate_case_identities']
    assert not result['missing_case_identities'] and not result['unexpected_case_identities']
    assert result['worker_count'] > 0
    assert len(result['worker_returncodes']) == result['worker_count']
    assert all(code == 0 for code in result['worker_returncodes'])
    assert all(result['loaded_changed_sources_global'].get(p) is True for p in context['required_changed_sources'])
    focus_manifest_path = reference(dict(path=result['candidate_manifest_path'],sha256=result['candidate_manifest_sha256']))
    focus_manifest = read(focus_manifest_path)
    assert focus_manifest['kind'] == 'GEN57_frozen_offline_candidate'
    assert focus_manifest['source_sha256'] == manifest['source_sha256']
    assert focus_manifest['runner_sha256'] == manifest['runner_sha256']
    assert focus_manifest['focus_scope'] == manifest['focus_scope']
    assert focus_manifest['baseline_manifest'] == manifest['baseline_manifest']
    xml = path.parent/'pytest.xml'; assert sha(xml) == result['xml_sha256']
    summary,cases,_ = inventory(xml)
    expected = [c for c in context['case_identities'] if c[0] in manifest['focus_scope']]
    assert summary == result['summary'] == dict(tests=len(expected),failures=0,errors=0,skipped=0)
    assert sorted(map(tuple,cases)) == sorted(map(tuple,expected))
    assert result['test_case_counts_by_file'] == dict(Counter(c[0] for c in expected))
    plan_path = path.parent/'plan.json'; assert sha(plan_path) == result['plan_sha256']
    plan = read(plan_path)
    assert plan['mode'] == 'focus' and plan['runner_sha256'] == sha(__file__)
    assert plan['candidate_manifest_sha256'] == result['candidate_manifest_sha256']
    assert plan['context']['sources'] == context['sources']
    assert plan['test_scope'] == manifest['focus_scope']
    assert len(result['worker_result_sha256']) == result['worker_count'] == plan['worker_count']
    bindings = {str(p):sha(p) for p in (path,focus_manifest_path,xml,plan_path)}
    for worker_path,digest in result['worker_result_sha256'].items():
        assert sha(worker_path) == digest and read(worker_path)['passed'] is True
        child = read(worker_path); folder = Path(worker_path).parent
        assert child['plan_sha256'] == result['plan_sha256']
        assert child['source_unchanged'] and child['scope_complete']
        assert child['direct_child_reaped'] and not child['direct_child_cleanup_signals']
        assert not child['foreign_or_inconsistent_imports'] and not child['external_boundary_attempts']
        bindings[str(Path(worker_path))] = digest
        for name,key in (('pytest.xml','xml_sha256'),('pytest.log','log_sha256'),('module_provenance.json','module_provenance_sha256')):
            artifact = folder/name; assert sha(artifact) == child[key]
            bindings[str(artifact)] = child[key]
        shard = plan['shards'][child['worker_index']]
        assert child['test_scope'] == shard['scope']
        supplemental,loaded_supplemental=supplemental_load_evidence(
            context,shard,read(folder/'module_provenance.json'))
        assert child['required_supplemental_imports']==supplemental
        assert child['loaded_supplemental_imports']==loaded_supplemental and all(loaded_supplemental.values())
        for name,digest in child['generated_bootstrap_sha256'].items():
            artifact = folder/name; assert sha(artifact) == digest
            bindings[str(artifact)] = digest
        boundary = folder/'boundary_attempts.jsonl'
        assert not boundary.exists() or not boundary.read_text().splitlines()
    return bindings


def case_identity(case):
    return [case.get('file',''), case.get('classname',''), case.get('name','')]


def inventory(path):
    tree = ET.parse(path).getroot()
    summary = dict.fromkeys(('tests','failures','errors','skipped'),0)
    for suite in tree.iter('testsuite'):
        for key in summary: summary[key] += int(suite.get(key,0))
    cases = [case_identity(c) for c in tree.iter('testcase')]
    durations = {}
    for case in tree.iter('testcase'):
        seconds = float(case.get('time','0'))
        assert math.isfinite(seconds) and seconds >= 0
        durations[case.get('file','')] = durations.get(case.get('file',''),0.) + seconds
    assert summary['tests'] == len(cases)
    assert len(cases) == len({tuple(c) for c in cases}), 'duplicate testcase identities'
    for key,tag in (('failures','failure'),('errors','error'),('skipped','skipped')):
        assert summary[key] == sum(len(c.findall(tag)) for c in tree.iter('testcase')), 'XML child outcome mismatch: '+key
    return summary,cases,durations


def make_plan(serial, output, mode, resources):
    context = current_context(serial)
    manifest=read(MANIFEST_PATH);history=read(BALANCE_HISTORY);xml=BALANCE_HISTORY.parent/'pytest.xml'
    _,_,durations=inventory(xml)
    assert mode in ('focus','full','collect')
    focus_bindings={}
    if mode=='full':
        assert manifest.get('full_scope_ready') is True, 'GEN57 full scope is not yet frozen/approved.'
        assert manifest.get('new_tests') and all('case_identities' in r for r in manifest['new_tests']+manifest.get('supplemental_existing_tests',[]))
        actual_new={str(p.relative_to(W)) for root in serial.ROOTS for prefix in ('test_gen57_*.py',) for p in (W/root).glob(prefix)}
        assert actual_new=={r['path'] for r in manifest['new_tests']}
        focus_bindings=validate_focus_evidence(manifest,context)
        scope=context['scope']
    elif mode=='focus':
        scope=manifest['focus_scope']
        assert scope and len(scope)==len(set(scope)) and set(scope)<=set(context['scope'])
    else:
        scope=[r['path'] for r in manifest['new_tests']+manifest.get('supplemental_existing_tests',[])];assert scope
    assert mode=='collect' or all(p in context['expected_counts'] for p in scope), 'Guarded collection evidence is missing.'
    expected_counts = {} if mode=='collect' else {p:context['expected_counts'][p] for p in scope}
    scope_cases = [] if mode=='collect' else [c for c in context['case_identities'] if c[0] in expected_counts]
    assert len(scope_cases) == sum(expected_counts.values())
    # A three-file focus uses at most three workers; this never changes the
    # explicitly recorded 8-worker resource cap or full-scope semantics.
    worker_count=min(resources['selected_workers'],len(scope))
    resources=dict(resources,selected_workers=worker_count,
        execution_resources_sufficient=(worker_count<=resources['resource_worker_capacity']))
    # Every whole file is assigned once by historical LPT. Only shards owning a
    # changed-module contract test require that module to load. Every import in
    # every shard still has strict provenance, and the aggregate must load it.
    assigned = [[] for _ in range(worker_count)]
    loads = [0. for _ in range(worker_count)]
    rest = set(scope)
    missing_weight=max(1.,sum(durations.values())/len(durations))
    for name in sorted(rest,key=lambda n:(-durations.get(n,missing_weight),n)):
        index = min(range(worker_count),key=lambda i:(loads[i],i))
        assigned[index].append(name); loads[index] += durations.get(name,missing_weight)
    shards = []
    for index, names in enumerate(assigned):
        chosen = set(names)
        shard_scope = [n for n in scope if n in chosen]
        shards.append(dict(index=index,scope=shard_scope,test_files=len(shard_scope),
            required_changed_sources=[] if mode=='collect' else sorted({p for test in shard_scope for p in context['required_imports_by_test_file'].get(test,[])}),
            expected_count=0 if mode=='collect' else sum(expected_counts[p] for p in shard_scope),
            historical_case_time_sec=loads[index],
            case_identities=[c for c in scope_cases if c[0] in chosen]))
    all_scope=[p for s in shards for p in s['scope']]
    assert len(all_scope)==len(set(all_scope)) and set(all_scope)==set(scope)
    assert sum(s['expected_count'] for s in shards) == len(scope_cases)
    required=set(context['required_changed_sources']) if mode=='full' else {
        p for test in scope for p in context['required_imports_by_test_file'].get(test,[])}
    if not required:required=set(context['required_changed_sources'])
    if mode=='collect':required=set()
    return dict(schema=1,kind='GEN57_parallel_'+mode+'_offline_plan',
        mode=mode,candidate_manifest_path=str(MANIFEST_PATH),candidate_manifest_sha256=MANIFEST_SHA,
        required_changed_sources=sorted(required),focus_evidence_sha256=focus_bindings,
        worker_count=worker_count,resource_budget=resources,
        prior_verified_two_worker_runner=str(PRIOR_TWO_WORKER),prior_verified_two_worker_runner_sha256=PRIOR_TWO_WORKER_SHA,
        scope_kind=mode,test_scope=scope,expected_file_count=len(scope),
        expected_case_count=len(scope_cases),expected_counts_by_file=expected_counts,
        full_regression_scope=(mode=='full'),
        runner_path=str(Path(__file__).resolve()),runner_sha256=sha(__file__),
        frozen_serial_contract=str(SERIAL),frozen_serial_contract_sha256=SERIAL_SHA,
        historical_result=str(BALANCE_HISTORY),historical_result_sha256=BALANCE_HISTORY_SHA,
        historical_xml=str(xml),historical_xml_sha256=sha(xml),context=context,
        expected_case_identities=scope_cases,shards=shards,output=str(output),
        maximum_workers=resources['worker_cap'],historical_result_elapsed_sec=history['elapsed_sec'],
        historical_worker_count=history['worker_count'], historical_scope_files=history['test_scope_files'],
        duration_estimate_scope='historical testcase execution time only; not a measured speedup or wall-time promise',
        tests_executed=False,approved_for_current_release=False,ROS_or_hardware_used=False)


def group_exists(group):
    try: os.killpg(group,0); return True
    except ProcessLookupError: return False


def signal_group(group, sig, records):
    # The coordinator passes only the exact PGIDs it created using
    # start_new_session=True. Worker leader exit does not change that ownership.
    try:
        os.killpg(group,sig)
        records.append(dict(pgid=group,signal=signal.Signals(sig).name))
    except ProcessLookupError: pass


def cleanup_group(group, records, process):
    """Bounded cleanup, including children after their worker leader exits."""
    def exists():
        process.poll()  # Reap our leader even if only descendants still exist.
        return group_exists(group)
    if not exists(): return False
    signal_group(group,signal.SIGTERM,records)
    deadline=time.monotonic()+5.
    while exists() and time.monotonic()<deadline: time.sleep(.05)
    if exists():
        signal_group(group,signal.SIGKILL,records)
        deadline=time.monotonic()+2.
        while exists() and time.monotonic()<deadline: time.sleep(.05)
    return exists()


def process_stat(pid):
    """Read only a proc identity; never infer ownership from command text."""
    try:
        text = (Path('/proc')/str(pid)/'stat').read_text()
        tail = text[text.rfind(')')+2:].split()
        return dict(pid=int(pid), state=tail[0], ppid=int(tail[1]),
            pgid=int(tail[2]), session=int(tail[3]), start_ticks=int(tail[19]),
            user_ticks=int(tail[11]), system_ticks=int(tail[12]), rss_pages=int(tail[21]))
    except (FileNotFoundError, ProcessLookupError, PermissionError):
        return None


class WorkloadMonitor:
    """Measured offline process groups, kept separate from host-wide context."""
    def __init__(self, output):
        self.path = output/'workload_samples.jsonl'
        self.stream = self.path.open('x')
        self.clock_ticks = os.sysconf('SC_CLK_TCK')
        self.page_kib = os.sysconf('SC_PAGE_SIZE')/1024.
        self.next_sample = 0.
        self.samples = 0
        self.max_group_rss_kib = 0.
        self.peak_active_process_count = 0
        self.minimum_host_available_mib = None
        self.observed = {}

    def sample(self, processes, force=False):
        now = time.monotonic()
        if not force and now < self.next_sample:
            return
        self.next_sample = now+2.
        groups = {p.pid for p in processes if p.poll() is None}
        rows = []
        for entry in Path('/proc').iterdir():
            if not entry.name.isdecimal():
                continue
            row = process_stat(entry.name)
            if row is None or row['pgid'] not in groups:
                continue
            row['cpu_sec'] = (row['user_ticks']+row['system_ticks'])/self.clock_ticks
            row['rss_kib'] = row['rss_pages']*self.page_kib
            try:
                io = (entry/'io').read_text()
                row['io_bytes'] = {k:int(v) for k,v in (line.split(':',1) for line in io.splitlines())
                    if k in ('read_bytes','write_bytes','rchar','wchar')}
            except (FileNotFoundError, ProcessLookupError, PermissionError):
                row['io_bytes'] = None
            key = (row['pid'],row['start_ticks'])
            self.observed[key] = max(row['cpu_sec'],self.observed.get(key,0.))
            rows.append(row)
        memory = {k:int(v.split()[0]) for k,v in
            (line.split(':',1) for line in Path('/proc/meminfo').read_text().splitlines())}
        available = memory['MemAvailable']/1024.
        self.minimum_host_available_mib = available if self.minimum_host_available_mib is None else min(available,self.minimum_host_available_mib)
        rss = sum(row['rss_kib'] for row in rows)
        self.max_group_rss_kib = max(self.max_group_rss_kib,rss)
        self.peak_active_process_count = max(self.peak_active_process_count,len(rows))
        record = dict(monotonic_sec=now,at_ns=time.time_ns(),owned_worker_pgids=sorted(groups),
            offline_process_groups=dict(processes=rows,simultaneous_rss_kib=rss,
                sampled_accumulated_cpu_sec=sum(self.observed.values())),
            coordinator=process_stat(os.getpid()),
            host_context=dict(load_average=list(os.getloadavg()),mem_available_mib=available,
                mem_total_mib=memory['MemTotal']/1024.,cpu_stat=Path('/proc/stat').read_text().splitlines()[0]))
        self.stream.write(json.dumps(record,allow_nan=False)+'\n');self.stream.flush()
        self.samples += 1

    def close(self):
        self.stream.flush();os.fsync(self.stream.fileno());self.stream.close()
        return dict(path=str(self.path),sha256=sha(self.path),sample_count=self.samples,
            sample_interval_sec=2.,peak_sampled_group_rss_kib=self.max_group_rss_kib,
            peak_sampled_offline_process_count=self.peak_active_process_count,
            observed_process_cpu_sec=sum(self.observed.values()),
            minimum_host_mem_available_mib=self.minimum_host_available_mib,
            cpu_scope='Per-process CPU for exact worker PGIDs, accumulated by PID/start identity; sampled lower bound, separate from worker RUSAGE_CHILDREN totals.',
            rss_scope='Simultaneous sampled RSS sum for exact offline worker PGIDs; shared pages may be counted more than once.',
            host_scope='Host load/memory/CPU context includes online work; it is not attributed to offline tests.')


COLLECTION_PLUGIN = '''import json, os
from pathlib import Path
from _pytest.junitxml import mangle_test_address

def pytest_collection_finish(session):
    rows = []
    for item in session.items:
        names = mangle_test_address(item.nodeid)
        rows.append({"case_identity": [str(item.location[0]), ".".join(names[:-1]), names[-1]], "nodeid": item.nodeid})
    with Path(os.environ["GEN57_COLLECTED_ITEMS"]).open("x") as f:
        json.dump(rows, f, ensure_ascii=False, indent=2)
'''


def stable(serial, plan):
    return (sha(__file__) == plan['runner_sha256'] and sha(SERIAL) == SERIAL_SHA
        and sha(PRIOR_TWO_WORKER)==PRIOR_TWO_WORKER_SHA and sha(CALIBRATION)==CALIBRATION_SHA
        and sha(HISTORY) == HISTORY_SHA and sha(plan['historical_xml']) == plan['historical_xml_sha256']
        and sha(MANIFEST_PATH)==MANIFEST_SHA==plan['candidate_manifest_sha256']
        and all(sha(p)==h for p,h in plan.get('focus_evidence_sha256',{}).items())
        and (plan['mode']!='full' or validate_focus_evidence(read(MANIFEST_PATH),plan['context'])==plan['focus_evidence_sha256'])
        and current_context(serial) == plan['context'])


def worker(plan_path,index):
    serial = load_serial(); plan = read(plan_path); plan_sha = sha(plan_path)
    assert index in range(plan['worker_count']) and stable(serial,plan)
    assert os.getsid(0)==os.getpgrp()==os.getpid(), 'worker must own its coordinator-created session'
    assert Path(plan['output']).resolve() == plan_path.parent.resolve()
    shard = plan['shards'][index]; context = plan['context']
    output = plan_path.parent/('worker_%d'%index)
    serial.storage(output); assert not output.exists(); output.mkdir()
    for name in ('tmp','ros','ros_home','bootstrap','pycache','cache'): (output/name).mkdir()
    guard,audit = bootstrap_sources(serial)
    (output/'bootstrap/sitecustomize.py').write_text(guard)
    (output/'bootstrap/gen42_import_audit.py').write_text(audit)
    if plan['mode']=='collect':(output/'bootstrap/gen57_collect_audit.py').write_text(COLLECTION_PLUGIN)
    generated = {str(p.relative_to(output)):sha(p) for p in (output/'bootstrap').iterdir()}
    env = dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1',
        PYTHONPYCACHEPREFIX=str(output/'pycache'),XDG_CACHE_HOME=str(output/'cache'),
        PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',
        MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1',
        LD_LIBRARY_PATH='/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu',
        ROS_DISTRO='noetic',ROS_VERSION='1',ROS_PYTHON_VERSION='3',
        ROS_MASTER_URI='http://127.0.0.1:1',ROS_LOG_DIR=str(output/'ros'),
        ROS_HOME=str(output/'ros_home'),TMPDIR=str(output/'tmp'),TMP=str(output/'tmp'),TEMP=str(output/'tmp'),
        OFFLINE_BOUNDARY_LOG=str(output/'boundary_attempts.jsonl'),
        GEN57_COLLECTED_ITEMS=str(output/'collected_items.json'),
        GEN42_MODULE_PROVENANCE=str(output/'module_provenance.json'))
    for key in ('PYTEST_ADDOPTS','PYTEST_PLUGINS'): env.pop(key,None)
    paths = [output/'bootstrap']+[W/p for p in (*serial.ROOTS,
        'src/alicia_grasp_modes/src','src/alicia_flexible_grasp_supervisor/src')]
    paths += [PROJECT/'devel/lib/python3/dist-packages',Path('/opt/ros/noetic/lib/python3/dist-packages')]
    env['PYTHONPATH'] = ':'.join(map(str,paths))
    env['ROS_PACKAGE_PATH'] = str(W/'src')+':/opt/ros/noetic/share'
    command = [sys.executable,'-m','pytest','-q','--rootdir='+str(W),*shard['scope'],
        '-p','no:cacheprovider','-p','gen42_import_audit','--tb=short',
        '--basetemp='+str(output/'pytest_tmp'),'--junitxml='+str(output/'pytest.xml'),
        '-o','junit_family=legacy']
    if plan['mode']=='collect':command+=['--collect-only','-p','gen57_collect_audit']
    result = dict(passed=False,worker_index=index,plan_sha256=plan_sha,command=command,
        mode=plan['mode'],test_bodies_executed=(plan['mode']!='collect'),
        scope_kind=plan['scope_kind'],full_regression_scope=plan['full_regression_scope'],
        required_changed_sources=shard['required_changed_sources'],
        test_scope=shard['scope'],expected_count=shard['expected_count'],interrupted=False)
    started=time.monotonic(); cpu_before=resource.getrusage(resource.RUSAGE_CHILDREN); returncode=None; process=None; cleanup=[]
    try:
        with (output/'pytest.log').open('x') as log:
            process=subprocess.Popen(command,cwd=W,env=env,stdout=log,stderr=subprocess.STDOUT)
            try: returncode=process.wait()
            except KeyboardInterrupt:
                result['interrupted']=True
                try: returncode=process.wait(timeout=20.)
                except subprocess.TimeoutExpired:
                    process.terminate();cleanup.append(dict(pid=process.pid,signal='SIGTERM'))
                    try: returncode=process.wait(timeout=5.)
                    except subprocess.TimeoutExpired:
                        process.kill();cleanup.append(dict(pid=process.pid,signal='SIGKILL'));returncode=process.wait()
        xml=output/'pytest.xml'; provenance_path=output/'module_provenance.json'
        summary,cases,_=inventory(xml)
        provenance=read(provenance_path); invalid={}; oracle_rows={}
        supplemental,loaded_supplemental=supplemental_load_evidence(context,shard,provenance)
        for label,row in provenance.items():
            p=Path(row['path'])
            if serial.inside(p,W/'src'):
                relative=str(p.relative_to(W)); valid=relative in context['sources'] and row['sha256']==context['sources'][relative]
            elif serial.inside(p,PROJECT/'devel'):
                valid=context['generated_imports'].get(str(p))==row['sha256'] and sha(p)==row['sha256']
            elif label in supplemental:
                valid=supplemental[label]==row and sha(p)==row['sha256']
            else:
                # This is exactly the serial full-run oracle exception, bound
                # to its original test in the UNION scope, path and bytes. A
                # helper import in the other process gains no wider whitelist.
                valid=(label==serial.ORACLE_LABEL and p==serial.ORACLE and row['sha256']==serial.ORACLE_SHA
                    and serial.ORACLE_TEST in plan['test_scope'] and context['sources'][serial.ORACLE_TEST]==serial.ORACLE_TEST_SHA
                    and sha(serial.ORACLE)==serial.ORACLE_SHA)
                if valid: oracle_rows[label]=row
            if not valid: invalid[label]=row
        loaded_changed={path:any(r['path']==str(W/path) and r['sha256']==context['sources'][path]
            for r in provenance.values()) for path in context['required_changed_sources']}
        source_loaded=any(loaded_changed.values())
        attempts_path=output/'boundary_attempts.jsonl'
        attempts=attempts_path.read_text().splitlines() if attempts_path.exists() else []
        expected=sorted(map(tuple,shard['case_identities']))
        actual=sorted(map(tuple,cases))
        counts=dict(Counter(c[0] for c in cases))
        expected_counts={} if plan['mode']=='collect' else {p:plan['expected_counts_by_file'][p] for p in shard['scope']}
        unchanged=(stable(serial,plan) and sha(plan_path)==plan_sha
            and all(sha(output/p)==h for p,h in generated.items()))
        complete=(actual==expected and counts==expected_counts and summary['tests']==shard['expected_count'])
        collected=[];collection_sha=None;collection_nodes=[]
        if plan['mode']=='collect':
            collected_rows=read(output/'collected_items.json');collection_sha=sha(output/'collected_items.json')
            collected=[r['case_identity'] for r in collected_rows];collection_nodes=[r['nodeid'] for r in collected_rows]
            assert len(collected)==len(set(map(tuple,collected)))==len(set(collection_nodes))
            complete=complete and set(c[0] for c in collected)==set(shard['scope']) and bool(collected)
        result.update(summary=summary,case_identities=cases,test_case_counts_by_file=counts,
            scope_complete=complete,source_unchanged=unchanged,changed_source_loaded=source_loaded,
            loaded_changed_sources=loaded_changed,
            required_supplemental_imports=supplemental,loaded_supplemental_imports=loaded_supplemental,
            foreign_or_inconsistent_imports=invalid,accepted_exact_historical_oracle=oracle_rows,
            external_boundary_attempts=attempts,generated_bootstrap_sha256=generated,
            module_provenance_sha256=sha(provenance_path),xml_sha256=sha(xml),log_sha256=sha(output/'pytest.log'))
        result.update(mode=plan['mode'],test_bodies_executed=(plan['mode']!='collect'),
            collected_case_identities=collected,collected_nodeids=collection_nodes,collection_sha256=collection_sha)
        result['passed']=(returncode==0 and not result['interrupted'] and complete and unchanged
            and all(loaded_changed[p] for p in shard['required_changed_sources']) and not invalid and not attempts
            and all(loaded_supplemental.values())
            and all(summary[k]==0 for k in ('failures','errors','skipped')))
    except BaseException as exc:
        if isinstance(exc,KeyboardInterrupt): result['interrupted']=True
        result.update(error=repr(exc),traceback=traceback.format_exc())
    finally:
        # Every exceptional path reaps the direct pytest child. Its descendants
        # inherit this worker's PGID; the coordinator always checks that entire
        # owned group, including when this worker exits before its descendants.
        if process is not None:
            if process.poll() is None:
                result['passed']=False
                process.terminate();cleanup.append(dict(pid=process.pid,signal='SIGTERM'))
                try: process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    process.kill();cleanup.append(dict(pid=process.pid,signal='SIGKILL'));process.wait()
            returncode=process.wait()
        cpu_after=resource.getrusage(resource.RUSAGE_CHILDREN)
        result.update(returncode=returncode,elapsed_sec=time.monotonic()-started,
            worker_pgid=os.getpgrp(),direct_child_cleanup_signals=cleanup,
            direct_child_reaped=(process is not None and process.returncode is not None),
            pytest_cpu_user_sec=cpu_after.ru_utime-cpu_before.ru_utime,
            pytest_cpu_system_sec=cpu_after.ru_stime-cpu_before.ru_stime,
            reaped_child_max_rss_kib=cpu_after.ru_maxrss,
            reaped_child_max_rss_scope='RUSAGE_CHILDREN high-water maximum in this fresh worker; not simultaneous RSS sum',
            live_ROS_contacted=False,hardware_contacted=False,motion_commands_sent=False,temperature_queried=False)
        serial.storage(output);save(output/'result.json',result)
    return 0 if result['passed'] else 1


def execute(serial,output,plan):
    assert stable(serial,plan)
    if plan['mode'] == 'full':
        assert os.environ.get('GEN57_DURABLE_UNIT') and os.environ.get('INVOCATION_ID'), 'Full regression requires its recorded durable user unit.'
    coordinator = process_stat(os.getpid())
    save(output/'coordinator_identity.json',dict(pid=os.getpid(),identity=coordinator,
        durable_unit=os.environ.get('GEN57_DURABLE_UNIT'),systemd_invocation_id=os.environ.get('INVOCATION_ID'),
        plan_sha256=sha(output/'plan.json'),runner_sha256=sha(__file__),at_ns=time.time_ns()))
    assert plan['resource_budget']['execution_resources_sufficient'], 'Selected workers exceed the recorded CPU/memory budget; no test started.'
    budget=plan['resource_budget']
    launch_budget=resource_budget(argparse.Namespace(workers=str(plan['worker_count']),worker_cap=budget['worker_cap'],
        reserve_cpus=budget['reserved_cpu_count'],memory_per_worker_mib=budget['memory_per_worker_mib'],
        reserve_memory_mib=budget['reserved_memory_mib']))
    save(output/'launch_resource_budget.json',launch_budget)
    assert launch_budget['execution_resources_sufficient'], 'Resources changed before launch; no test started.'
    plan_path=output/'plan.json'; plan_sha=sha(plan_path)
    processes=[]; logs=[]; interrupted=False; signals=[]; started=time.monotonic()
    closed_groups=set();residual_groups=[];cleanup_required=False;launcher_error=None
    monitor=WorkloadMonitor(output);workload=None
    def close_owned_group(process):
        nonlocal cleanup_required
        if process.pid in closed_groups:return
        # Check our exact group even when poll() already reaped its leader.
        # Closing promptly reduces the lifetime of a completed group identity.
        if group_exists(process.pid):cleanup_required=True
        if cleanup_group(process.pid,signals,process):residual_groups.append(process.pid)
        closed_groups.add(process.pid)
    try:
        for index in range(plan['worker_count']):
            log=(output/('worker_%d_launcher.log'%index)).open('x'); logs.append(log)
            command=[sys.executable,'-B',str(Path(__file__).resolve()),'--worker-plan',str(plan_path),'--worker-index',str(index)]
            processes.append(subprocess.Popen(command,cwd=W,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        save(output/'owned_workers.json',dict(plan_sha256=plan_sha,coordinator_pid=os.getpid(),
            workers=[dict(index=i,pid=p.pid,pgid=p.pid,identity=process_stat(p.pid)) for i,p in enumerate(processes)]))
        try:
            while True:
                monitor.sample(processes)
                running=False
                for process in processes:
                    if process.poll() is None:running=True
                    else:close_owned_group(process)
                if not running:break
                time.sleep(.2)
        except KeyboardInterrupt:
            interrupted=True
            for p in processes:
                signal_group(p.pid,signal.SIGINT,signals)
            deadline=time.monotonic()+30.
            while any(p.poll() is None for p in processes) and time.monotonic()<deadline:time.sleep(.1)
    except BaseException as exc:
        interrupted=interrupted or isinstance(exc,KeyboardInterrupt)
        launcher_error=dict(error=repr(exc),traceback=traceback.format_exc())
    finally:
        # Only these explicit offline children are eligible for cleanup. No
        # runtime ROS process group, user service or hardware PID is touched.
        for p in processes:
            close_owned_group(p)
            if p.poll() is None:
                try:p.wait(timeout=2.)
                except subprocess.TimeoutExpired:residual_groups.append(p.pid)
        for log in logs:log.close()
        workload=monitor.close()
    results=[]; missing=[]; all_cases=[]; all_collected=[]; all_nodes=[]; summary=dict.fromkeys(('tests','failures','errors','skipped'),0)
    combined=ET.Element('testsuites'); bindings={}
    for index in range(plan['worker_count']):
        folder=output/('worker_%d'%index);path=folder/'result.json'
        if not path.exists():missing.append(index);continue
        row=read(path);results.append(row);bindings[str(path)]=sha(path)
        assert row['plan_sha256']==plan_sha
        assert row['worker_index']==index and row['test_scope']==plan['shards'][index]['scope']
        if row.get('passed'):
            # Independently bind the child's evidence after BOTH children exit;
            # no summary silently trusts a green child JSON with changed XML,
            # provenance, bootstrap or appended boundary evidence.
            assert sha(folder/'module_provenance.json')==row['module_provenance_sha256']
            assert sha(folder/'pytest.log')==row['log_sha256']
            assert sha(folder/'pytest.xml')==row['xml_sha256']
            assert all(sha(folder/p)==h for p,h in row['generated_bootstrap_sha256'].items())
            boundary=folder/'boundary_attempts.jsonl'
            assert not boundary.exists() or not boundary.read_text().splitlines()
            assert row['source_unchanged'] and row['scope_complete']
            assert row['required_changed_sources']==plan['shards'][index]['required_changed_sources']
            assert all(row['loaded_changed_sources'][p] for p in row['required_changed_sources'])
            supplemental,loaded_supplemental=supplemental_load_evidence(
                plan['context'],plan['shards'][index],read(folder/'module_provenance.json'))
            assert row['required_supplemental_imports']==supplemental
            assert row['loaded_supplemental_imports']==loaded_supplemental and all(loaded_supplemental.values())
            assert row['direct_child_reaped'] and not row['direct_child_cleanup_signals']
            assert row['scope_kind']==plan['scope_kind'] and row['full_regression_scope']==plan['full_regression_scope']
            assert not row['foreign_or_inconsistent_imports'] and not row['external_boundary_attempts']
            child_summary,child_cases,_=inventory(folder/'pytest.xml')
            assert child_summary==row['summary'] and child_cases==row['case_identities']
            if plan['mode']=='collect':
                assert sha(folder/'collected_items.json')==row['collection_sha256']
                collected_rows=read(folder/'collected_items.json')
                assert [r['case_identity'] for r in collected_rows]==row['collected_case_identities']
                assert [r['nodeid'] for r in collected_rows]==row['collected_nodeids']
        if 'summary' in row:
            for key in summary:summary[key]+=row['summary'][key]
        all_cases+=row.get('case_identities',[])
        all_collected+=row.get('collected_case_identities',[]);all_nodes+=row.get('collected_nodeids',[])
        xml=folder/'pytest.xml'
        if row.get('xml_sha256') and sha(xml)==row['xml_sha256']:
            parsed=ET.parse(xml).getroot()
            for suite in parsed.iter('testsuite'):combined.append(deepcopy(suite))
    duplicates=[list(k) for k,n in Counter(map(tuple,all_cases)).items() if n>1]
    expected=set(map(tuple,plan['expected_case_identities']));actual=set(map(tuple,all_cases))
    counts=dict(Counter(c[0] for c in all_cases))
    complete=(not missing and not duplicates and expected==actual and counts==plan['expected_counts_by_file']
        and len(all_cases)==summary['tests']==plan['expected_case_count'])
    if plan['mode']=='collect':
        complete=(complete and bool(all_collected) and len(all_collected)==len(set(map(tuple,all_collected)))==len(set(all_nodes))
            and set(c[0] for c in all_collected)==set(plan['test_scope']))
    unchanged=stable(serial,plan) and sha(plan_path)==plan_sha
    globally_loaded={path:any(r.get('loaded_changed_sources',{}).get(path,False) for r in results)
        for path in plan['required_changed_sources']}
    passed=(not interrupted and launcher_error is None and not cleanup_required and not residual_groups
        and len(processes)==len(results)==plan['worker_count'] and all(p.returncode==0 for p in processes)
        and all(r['passed'] for r in results) and complete and unchanged
        and all(globally_loaded.values())
        and all(summary[k]==0 for k in ('failures','errors','skipped')))
    ET.ElementTree(combined).write(output/'pytest.xml',encoding='utf-8',xml_declaration=True)
    combined_summary,combined_cases,_=inventory(output/'pytest.xml')
    combined_matches=(combined_summary==summary and sorted(map(tuple,combined_cases))==sorted(map(tuple,all_cases)))
    passed=passed and combined_matches
    result=dict(passed=passed,summary=summary,scope_complete=complete,source_unchanged=unchanged,
        kind='GEN57_parallel_'+plan['scope_kind']+'_offline_result',scope_kind=plan['scope_kind'],
        mode=plan['mode'],candidate_manifest_path=str(MANIFEST_PATH),candidate_manifest_sha256=MANIFEST_SHA,
        full_regression_scope=plan['full_regression_scope'],
        full_regression_passed=(passed and plan['full_regression_scope'] and plan['context']['compatibility']['variant']=='candidate'),
        test_bodies_executed=(plan['mode']!='collect'),
        collected_case_identities=all_collected,collected_nodeids=all_nodes,collected_case_count=len(all_collected),
        external_boundary_attempts=[x for r in results for x in r.get('external_boundary_attempts',[])],
        foreign_or_inconsistent_imports={str(i):r['foreign_or_inconsistent_imports'] for i,r in enumerate(results) if r.get('foreign_or_inconsistent_imports')},
        elapsed_sec=time.monotonic()-started,worker_count=plan['worker_count'],resource_budget=plan['resource_budget'],
        measured_workload=workload,
        pytest_cpu_user_sec=sum(r.get('pytest_cpu_user_sec',0.) for r in results),
        pytest_cpu_system_sec=sum(r.get('pytest_cpu_system_sec',0.) for r in results),
        pytest_cpu_scope='Sum of RUSAGE_CHILDREN deltas from disjoint fresh workers; observed pytest process trees only.',
        launch_resource_budget=launch_budget,launch_resource_budget_sha256=sha(output/'launch_resource_budget.json'),
        loaded_changed_sources_global=globally_loaded,test_scope_files=plan['expected_file_count'],source_count=len(plan['context']['sources']),
        test_case_counts_by_file=counts,duplicate_case_identities=duplicates,
        missing_case_identities=[list(c) for c in sorted(expected-actual)],
        unexpected_case_identities=[list(c) for c in sorted(actual-expected)],missing_worker_results=missing,
        worker_result_sha256=bindings,worker_returncodes=[p.returncode for p in processes],
        interrupted=interrupted,launcher_error=launcher_error,
        offline_child_cleanup_required=cleanup_required,offline_child_cleanup_signals=signals,
        residual_offline_process_groups=sorted(set(residual_groups)),plan_sha256=plan_sha,
        combined_xml_matches_worker_results=combined_matches,
        xml_sha256=sha(output/'pytest.xml'),runner_sha256=sha(__file__),
        source_sha256=plan['context']['sources'],source_compatibility=plan['context']['compatibility'],
        accepted_for_current_release=False,live_ROS_contacted=False,hardware_contacted=False,
        motion_commands_sent=False,temperature_queried=False,deployed=False,physical_grasp_verified=False)
    save(output/'result.json',result)
    print(json.dumps(dict(passed=passed,summary=summary,output=str(output))),flush=True)
    return 0 if passed else 1


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path)
    p.add_argument('--manifest',type=Path)
    p.add_argument('--manifest-sha256')
    p.add_argument('--mode',choices=('focus','full','collect'),default='focus')
    p.add_argument('--workers',default='auto',help='auto or an explicit positive count; plans record whether the resource budget permits execution.')
    p.add_argument('--worker-cap',type=int,default=8,help='Safety cap, at most8 for this helper.')
    p.add_argument('--reserve-cpus',type=int,default=4)
    p.add_argument('--memory-per-worker-mib',type=int,default=768)
    p.add_argument('--reserve-memory-mib',type=int,default=2048)
    p.add_argument('--execute',action='store_true',help='Explicitly execute the budgeted complete-file shards; default is plan only.')
    p.add_argument('--execute-plan',type=Path,help='Execute a previously written exact plan from its persistent coordinator.')
    p.add_argument('--plan-sha256')
    p.add_argument('--worker-plan',type=Path,help=argparse.SUPPRESS)
    p.add_argument('--worker-index',type=int,help=argparse.SUPPRESS)
    a=p.parse_args()
    def terminate_as_interrupt(signum, frame):
        raise KeyboardInterrupt('offline coordinator received '+signal.Signals(signum).name)
    signal.signal(signal.SIGTERM,terminate_as_interrupt)
    if a.execute_plan:
        assert a.output is None and a.manifest is None and not a.execute and a.worker_plan is None
        plan_path=a.execute_plan.resolve(strict=True)
        assert a.plan_sha256 and sha(plan_path)==a.plan_sha256
        plan=read(plan_path);bind_manifest(plan['candidate_manifest_path'],plan['candidate_manifest_sha256'])
        serial=load_serial();output=plan_path.parent;serial.storage(output)
        assert plan_path.name=='plan.json' and Path(plan['output']).resolve()==output
        assert not (output/'result.json').exists() and not (output/'coordinator_identity.json').exists()
        assert not list(output.glob('worker_*')) and stable(serial,plan)
        return execute(serial,output,plan)
    assert a.plan_sha256 is None
    if a.worker_plan:
        assert a.output is None and not a.execute and a.worker_index is not None
        plan=read(a.worker_plan)
        bind_manifest(plan['candidate_manifest_path'],plan['candidate_manifest_sha256'])
        return worker(a.worker_plan.resolve(strict=True),a.worker_index)
    assert a.worker_index is None and a.output is not None
    assert a.manifest is not None and a.manifest_sha256 is not None
    bind_manifest(a.manifest,a.manifest_sha256)
    serial=load_serial();output=a.output.resolve();serial.storage(output);assert not output.exists()
    assert a.workers=='auto' or (a.workers.isdecimal() and int(a.workers)>0)
    assert sha(PRIOR_TWO_WORKER)==PRIOR_TWO_WORKER_SHA
    plan=make_plan(serial,output,a.mode,resource_budget(a));output.mkdir();save(output/'plan.json',plan)
    if not a.execute:
        print(json.dumps(dict(plan_only=True,tests_executed=False,plan=str(output/'plan.json'),
            worker_count=plan['worker_count'],execution_resources_sufficient=plan['resource_budget']['execution_resources_sufficient'],
            plan_sha256=sha(output/'plan.json'),shards=[dict(index=s['index'],files=s['test_files'],cases=s['expected_count'],historical_case_time_sec=s['historical_case_time_sec']) for s in plan['shards']])),flush=True)
        return 0
    return execute(serial,output,plan)


if __name__=='__main__':
    raise SystemExit(main())
