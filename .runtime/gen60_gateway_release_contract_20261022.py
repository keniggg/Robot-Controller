"""Exact GEN60 r2 full-regression/install closure; no ROS or execution on import."""
import ast
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET
W=Path(__file__).resolve().parents[1]
M=Path('/media/zhuyupei/SanStand')
E=M/'实验记录/Robot-Controller'
N=E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
O=N/'gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1'
BASE=N/'verification/gen59_release_manifest.json'
BASE_SHA='5fed4cbed7ea1e824eda3bb750657970dcc1ec2d711cd041eff1a3d482ef2c6c'
PREVIOUS=O.parent/'runtime_reload_r1/acceptance/result.json'
PREVIOUS_SHA='b5b53dcf13f51d507187a4c71e7ac20977e3bbc1ed6b51025f30ed5aabfb1082'
C=W/'.runtime/gen60_held_lattice_candidate_r2'
RUNNER=W/'.runtime/verify_gen60_held_lattice_r2_20261022.py'
SELECTOR='src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/held_pregrasp_selection.py'
SELECTOR_SHA='9930078f45949c6c48458b44155598bdb54d411e33c99822b7982f818ef1e4e1'
CONTRACT='src/alicia_flexible_grasp_supervisor/tests/test_gen55_held_pregrasp_reuse.py'
CONTRACT_FUNCTION='test_unsuccessful_solver_retains_exact_failure_and_evidence'
PRODUCTION={SELECTOR}
CONTRACTS={CONTRACT}
ADDED={'src/alicia_flexible_grasp_supervisor/tests/test_gen60_held_pregrasp_lattice.py',
 'src/alicia_flexible_grasp_supervisor/tests/test_gen60_held_lattice_lifecycle.py',
 'src/alicia_flexible_grasp_supervisor/tests/fixtures/gen59_held_lattice_inputs.json'}
CHANGED=PRODUCTION|CONTRACTS|ADDED


def require(value, message):
    if not value:
        raise RuntimeError(message)

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def read(path):
    return json.loads(Path(path).read_bytes())

def storage(path):
    path = Path(path).resolve()
    require(M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY, 'writable SanStand required')
    require(N in path.parents, 'output outside experiment storage')
    parent = path.parent
    while not parent.exists():
        parent = parent.parent
    require(parent.stat().st_dev == M.stat().st_dev, 'output must remain on SanStand')
    config = read(E/'grasp_records/archive_config.json')
    require(config['upload_enabled'] is False and config['delete_enabled'] is False
            and read(E/'maintenance/storage_control.json')['archive_policy'] == 'local_only', 'storage policy differs')
    stat = os.statvfs(E)
    require(stat.f_bavail*stat.f_frsize >= config['min_free_bytes'], 'insufficient configured free space')

def save(path, data):
    storage(path)
    raw = (json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode()
    with Path(path).open('xb') as stream:
        stream.write(raw); stream.flush(); os.fsync(stream.fileno())
    require(Path(path).read_bytes() == raw, 'receipt read-back differs')

def inventory(path):
    root = ET.parse(path).getroot()
    summary = {k: sum(int(s.get(k, 0)) for s in root.iter('testsuite'))
               for k in ('tests', 'failures', 'errors', 'skipped')}
    cases = [(c.get('file', ''), c.get('classname', ''), c.get('name', '')) for c in root.iter('testcase')]
    require(summary['tests'] == len(cases) == len(set(cases)), 'XML cases missing or repeated')
    for key, tag in (('failures', 'failure'), ('errors', 'error'), ('skipped', 'skipped')):
        require(summary[key] == sum(len(c.findall(tag)) for c in root.iter('testcase')), 'XML result counts differ')
    return summary, cases

def verify_contract_body(base_manifest, candidate):
    baseline=W/'.runtime/gen59_observation_pose_candidate_r3'/CONTRACT
    require(sha(baseline)==read(base_manifest)['source_sha256'][CONTRACT], 'frozen old test differs')
    trees=[ast.parse(p.read_text()) for p in (baseline,candidate)]
    for tree in trees:
        hits=[n for n in ast.walk(tree) if isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef)) and n.name==CONTRACT_FUNCTION]
        require(len(hits)==1,'old test contract function missing/ambiguous')
        hits[0].body=[]
    require(ast.dump(trees[0],include_attributes=False)==ast.dump(trees[1],include_attributes=False), 'test change extends beyond reviewed function body')

def validate(manifest_path, full_path, review_path, *, installed=False):
    refs = {}
    def bind(path, expected=None):
        path = Path(path).resolve(strict=True); digest = sha(path)
        require(expected is None or digest == expected, 'dependency changed: '+str(path))
        require(str(path) not in refs or refs[str(path)] == digest, 'conflicting dependency identity')
        refs[str(path)] = digest
        return read(path) if path.suffix == '.json' else path
    base = bind(BASE, BASE_SHA); bind(PREVIOUS, PREVIOUS_SHA)
    manifest = bind(manifest_path); full = bind(full_path); review = bind(review_path)
    require(manifest['kind'] == 'GEN60_held_lattice_frozen_offline_candidate'
            and manifest['full_scope_ready'] and Path(manifest['workspace_root']) == C, 'wrong candidate manifest')
    sources = manifest['source_sha256']; old = base['source_sha256']; context = manifest['context']
    require(len(old) == 1094 and len(sources) == 1097 and set(sources)-set(old) == ADDED, 'source scope changed')
    require({p for p in old if old[p] != sources[p]} == PRODUCTION | CONTRACTS, 'unreviewed source delta')
    require(all(not Path(p).is_absolute() and '..' not in Path(p).parts for p in sources), 'unsafe source path')
    require(all(sha(W/p) == h for p,h in (sources if installed else old).items()) and (installed or all(not (W/p).exists() for p in ADDED)), 'active source state changed')
    require(manifest['revision'] == 2 and sources[SELECTOR] == SELECTOR_SHA, 'exact reviewed production/revision required')
    require(set(manifest['explicit_contract_diffs']) == CONTRACTS, 'explicit test contract delta differs')
    require(len(context['compatibility']['explicit_old_test_contract_changes']) == 1 and context['compatibility']['explicit_old_test_contract_changes'][0]['path'] == CONTRACT, 'old test contract provenance missing')
    verify_contract_body(BASE, C/CONTRACT)
    require(sources == context['sources'] == full['source_sha256'] == review['source_sha256'], 'source bindings differ')
    require(all(sha(C/p) == h for p,h in manifest['copied_input_sha256'].items()), 'candidate input changed')
    bind(RUNNER, manifest['runner_sha256'])
    for p,h in context['compatibility_dependencies'].items():
        bind(p,h)
    for p,h in context['generated_imports'].items():
        bind(p,h)
    require(review['passed'] is True and review['full_checks_retained'] is True
            and review['physical_grasp_verified'] is False, 'patch review incomplete')
    require(set(review['reviewed_changed_paths']) == PRODUCTION | CONTRACTS | ADDED, 'patch review scope differs')
    for p,h in review['dependency_sha256'].items():
        bind(p,h)
    require(full['mode'] == 'full' and full['candidate_manifest_path'] == str(manifest_path)
            and full['candidate_manifest_sha256'] == sha(manifest_path), 'full run manifest binding differs')
    flags = ('passed','full_regression_passed','full_regression_scope','scope_complete','source_unchanged',
             'test_bodies_executed','combined_xml_matches_worker_results')
    require(all(full[k] is True for k in flags), 'full regression incomplete')
    empty = ('external_boundary_attempts','foreign_or_inconsistent_imports','duplicate_case_identities',
             'missing_case_identities','unexpected_case_identities','missing_worker_results','residual_offline_process_groups')
    require(all(not full[k] for k in empty) and not full['interrupted'] and full['launcher_error'] is None
            and not full['offline_child_cleanup_required'], 'full offline audit failed')
    require(full['worker_count'] <= 6 and full['worker_returncodes'] == [0]*full['worker_count'], 'workers failed')
    require(full['runner_sha256'] == sha(RUNNER) and all(full['loaded_changed_sources_global'][p] for p in PRODUCTION), 'changed source was not tested')
    plan = bind(full_path.parent/'plan.json', full['plan_sha256'])
    require(plan['context'] == context, 'full plan source/case contract differs')
    summary, cases = inventory(bind(full_path.parent/'pytest.xml', full['xml_sha256']))
    expected = list(map(tuple, context['case_identities']))
    historical = list(map(tuple, context['old_case_identities']))
    require(len(historical) == len(set(historical)) == 9993 and set(historical) <= set(expected), 'historical cases lost')
    require(sorted(cases) == sorted(expected) and summary == full['summary'] and summary['tests'] == 10044
            and all(summary[k] == 0 for k in ('failures','errors','skipped')), 'full case union/results differ')
    require(full['test_scope_files'] == 352 and full['source_count'] == 1097
            and dict(Counter(c[0] for c in cases)) == full['test_case_counts_by_file'], 'full scope counts differ')
    union = []
    worker_indices = set()
    for name,digest in full['worker_result_sha256'].items():
        worker = bind(name,digest); folder = Path(name).parent
        require(worker['passed'] and worker['source_unchanged'] and worker['scope_complete']
                and worker['test_bodies_executed'] and worker['returncode'] == 0
                and worker['plan_sha256'] == full['plan_sha256'], 'worker audit failed')
        require(worker['worker_index'] not in worker_indices, 'duplicate worker index')
        worker_indices.add(worker['worker_index'])
        worker_summary, worker_cases = inventory(bind(folder/'pytest.xml',worker['xml_sha256']))
        require(worker_summary == worker['summary'] and sorted(worker_cases) == sorted(map(tuple,worker['case_identities'])), 'worker summary differs')
        union.extend(worker_cases)
        bind(folder/'pytest.log',worker['log_sha256'])
        for name,digest in worker['generated_bootstrap_sha256'].items():
            bind(folder/name,digest)
        provenance = bind(folder/'module_provenance.json',worker['module_provenance_sha256'])
        for item in provenance.values():
            bind(item['path'],item['sha256'])
    require(worker_indices == set(range(full['worker_count'])) and sorted(union) == sorted(expected)
            and len(union) == len(set(union)), 'worker partition differs')
    for path,digest in plan['focus_evidence_sha256'].items():
        bind(path,digest)
    require(all(sha(p) == h for p,h in refs.items()), 'evidence changed during validation')
    return dict(source_sha256=sources, baseline_source_sha256=old, evidence_sha256=refs,
        test_summary=summary, source_count=1097, test_scope_files=352,
        new_case_count=len(cases)-9993, full_elapsed_sec=full['elapsed_sec'], worker_count=full['worker_count'])
