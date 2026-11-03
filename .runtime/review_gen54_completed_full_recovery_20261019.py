"""Independent file/proc-only verification of one completed GEN54 test recovery."""
from pathlib import Path
from collections import Counter
import hashlib
import json
import math
import time
import xml.etree.ElementTree as ET

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
V = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
O = V / 'gen54_full_parallel_r1'
OUT = V / 'gen54_full_recovery_independent_review_r1'
REC = V / 'gen54_full_recovery_r1/recovery_receipt.json'
HELPER = W / '.runtime/recover_gen54_completed_full_20261019.py'
PLAN_SHA = '478deefff62d468da9136918ee709c6198491c83bab51909dc1060327b9f20e2'
FULL_SHA = '6963ef9c30e17a4b94c0fc0aee62476bce29af4641c3966ad561c0ec05728adb'
REC_SHA = 'b650aeb6eb2345f47c31eb3e93bfb03b68b24ce26590f543c50d0b734dba72f3'
HELPER_SHA = '159a232e039f26c93211b59ab41d6fcbb38e47aaf78dc9f2fcae0ed6c0b20521'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes())


def inventory(path):
    xt = ET.parse(path).getroot()
    cs = list(xt.iter('testcase'))
    ids = [[c.get(k, '') for k in ('file', 'classname', 'name')] for c in cs]
    summary = {k: sum(int(su.get(k, 0)) for su in xt.iter('testsuite'))
               for k in ('tests', 'failures', 'errors', 'skipped')}
    assert len(cs) == summary['tests'] == len(set(map(tuple, ids)))
    for key, tag in [('failures', 'failure'), ('errors', 'error'), ('skipped', 'skipped')]:
        assert summary[key] == sum(len(c.findall(tag)) for c in cs) == 0
    assert all(math.isfinite(float(c.get('time', '0'))) and float(c.get('time', '0')) >= 0 for c in cs)
    return summary, ids


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    assert not OUT.exists()
    assert sha(O/'plan.json') == PLAN_SHA
    assert sha(O/'result.json') == FULL_SHA and sha(REC) == REC_SHA and sha(HELPER) == HELPER_SHA
    plan = read(O/'plan.json'); ctx = plan['context']
    full = read(O/'result.json'); receipt = read(REC)
    assert plan['runner_sha256'] == sha(W/'.runtime/verify_gen54_parallel_offline_20261018.py') == '2b9023c285ce3aadaaad37c0f734b96a567963dc384838ed3319c3858d24b5f5'
    manifest = read(plan['candidate_manifest_path'])
    assert sha(plan['candidate_manifest_path']) == plan['candidate_manifest_sha256'] == 'aebaa01c523bfb2f8d8351f05b4e060ed6091424f4847cfaf0c76023729a6cfe'
    assert manifest['workspace_root'] == str(W)
    assert manifest['source_sha256'] == ctx['sources'] == full['source_sha256'] == receipt['source_sha256']
    assert len(ctx['sources']) == 1075
    for p, h in ctx['sources'].items():
        assert sha(W/p) == h, p
    for p, h in ctx['compatibility_dependencies'].items():
        assert sha(p) == h, p
    for p, h in ctx['generated_imports'].items():
        assert sha(p) == h, p
    bindings = {str(O/'plan.json'): PLAN_SHA, str(O/'result.json'): FULL_SHA,
                str(REC): REC_SHA, str(HELPER): HELPER_SHA,
                str(Path(plan['candidate_manifest_path'])): plan['candidate_manifest_sha256'],
                str(W/'.runtime/verify_gen54_parallel_offline_20261018.py'): plan['runner_sha256']}
    cases = []; workers = []; pgids = []; loaded_global = set(); boundary_absent = []; worker_hashes = {}
    assert len(plan['shards']) == 7 and plan['mode'] == plan['scope_kind'] == 'full'
    for i, s in enumerate(plan['shards']):
        f = O/f'worker_{i}'; r = read(f/'result.json')
        assert r['worker_index'] == s['index'] == i
        for k in ('passed', 'test_bodies_executed', 'full_regression_scope', 'scope_complete', 'source_unchanged', 'direct_child_reaped'):
            assert r[k] is True, (i, k)
        assert r['returncode'] == 0 and r['interrupted'] is False and not r['direct_child_cleanup_signals']
        assert r['plan_sha256'] == PLAN_SHA and r['test_scope'] == s['scope'] and r['required_changed_sources'] == s['required_changed_sources']
        assert r['mode'] == r['scope_kind'] == 'full'
        assert not r['external_boundary_attempts'] and not r['foreign_or_inconsistent_imports']
        assert r['worker_pgid'] == 32759+i; pgids.append(r['worker_pgid'])
        for name, key in [('pytest.xml', 'xml_sha256'), ('pytest.log', 'log_sha256'), ('module_provenance.json', 'module_provenance_sha256')]:
            assert sha(f/name) == r[key]; bindings[str(f/name)] = r[key]
        assert r['generated_bootstrap_sha256'] == {'bootstrap/sitecustomize.py': ctx['guard_sha256'], 'bootstrap/gen42_import_audit.py': ctx['audit_sha256']}
        for n, h in r['generated_bootstrap_sha256'].items():
            assert sha(f/n) == h; bindings[str(f/n)] = h
        boundary = f/'boundary_attempts.jsonl'
        if boundary.exists():
            assert not boundary.read_text().splitlines(); bindings[str(boundary)] = sha(boundary)
        else:
            boundary_absent.append(str(boundary))
        summary, ids = inventory(f/'pytest.xml')
        assert summary == r['summary'] == dict(tests=s['expected_count'], failures=0, errors=0, skipped=0)
        assert ids == r['case_identities'] and Counter(map(tuple, ids)) == Counter(map(tuple, s['case_identities']))
        counts = dict(Counter(x[0] for x in ids))
        assert counts == r['test_case_counts_by_file'] == {p: plan['expected_counts_by_file'][p] for p in s['scope']}
        prov = read(f/'module_provenance.json'); loaded = {p: False for p in ctx['required_changed_sources']}; oracle = {}
        for label, row in prov.items():
            p = Path(row['path']); h = row['sha256']; assert sha(p) == h
            try:
                rel = str(p.relative_to(W/'src')); relative = 'src/'+rel
                assert ctx['sources'].get(relative) == h
            except ValueError:
                if str(p) in ctx['generated_imports']:
                    assert ctx['generated_imports'][str(p)] == h
                else:
                    assert label == 'alicia_flexible_grasp.vision._gen38_registration_test_baseline'
                    assert p == W/'.runtime/gen38_baseline_source/src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/multiview_surface.py'
                    assert h == '369a2947114c6c3953c439a7937cf26971e337c015e65abf96825720bc5c1ba8'
                    ot = 'src/alicia_flexible_grasp_supervisor/tests/test_gen39_registration_transform.py'
                    assert ot in plan['test_scope'] and ctx['sources'][ot] == 'fcd9fe03951fc4761f1faaf486b1ccf979583cb1cd07be39f9912845a5bd7c71'
                    oracle[label] = row; bindings[str(p)] = h
            for q in loaded:
                if str(W/q) == str(p):
                    assert h == ctx['sources'][q]; loaded[q] = True
        assert loaded == r['loaded_changed_sources'] and oracle == r['accepted_exact_historical_oracle']
        assert all(loaded[p] for p in s['required_changed_sources'])
        loaded_global.update(p for p, b in loaded.items() if b)
        cases.extend(ids); bindings[str(f/'result.json')] = sha(f/'result.json')
        worker_hashes[str(f/'result.json')] = sha(f/'result.json')
        workers.append(dict(index=i, tests=len(ids), provenance_module_count=len(prov),
            result_sha256=sha(f/'result.json'), pytest_returncode=0, pytest_reaped=True,
            wrapper_returncode=None, pgid=r['worker_pgid'], elapsed_sec=r['elapsed_sec'],
            cpu_user_sec=r['pytest_cpu_user_sec'], cpu_system_sec=r['pytest_cpu_system_sec'],
            child_high_water_rss_kib=r['reaped_child_max_rss_kib'], high_water_scope=r['reaped_child_max_rss_scope']))
    assert len(cases) == len(set(map(tuple, cases))) == 9255
    assert Counter(map(tuple, cases)) == Counter(map(tuple, plan['expected_case_identities'])) == Counter(map(tuple, ctx['case_identities']))
    assert dict(Counter(c[0] for c in cases)) == plan['expected_counts_by_file'] == full['test_case_counts_by_file']
    assert len(set(c[0] for c in cases)) == full['test_scope_files'] == 329
    assert loaded_global == set(ctx['required_changed_sources'])
    assert full['loaded_changed_sources_global'] == {p: True for p in loaded_global}
    full_summary, full_cases = inventory(O/'pytest.xml')
    assert full_summary == full['summary'] == receipt['exact_test_summary'] == dict(tests=9255, failures=0, errors=0, skipped=0)
    assert Counter(map(tuple, full_cases)) == Counter(map(tuple, cases))
    assert full['xml_sha256'] == sha(O/'pytest.xml'); bindings[str(O/'pytest.xml')] = full['xml_sha256']
    assert full['worker_result_sha256'] == receipt['worker_result_sha256'] == worker_hashes
    assert full['plan_sha256'] == receipt['original_plan_sha256'] == PLAN_SHA
    assert full['recovery_receipt'] == dict(path=str(REC), sha256=REC_SHA)
    assert full['recovery_helper_sha256'] == receipt['helper_sha256'] == HELPER_SHA
    assert full['summary_recovered_from_completed_workers'] is True
    assert full['passed'] is True and full['full_regression_passed'] is True and full['interrupted'] is True
    assert full['worker_returncodes'] == receipt['worker_wrappers_exit_codes'] == [None]*7
    assert full['pytest_returncodes'] == receipt['pytest_direct_child_exit_codes'] == [0]*7
    assert full['elapsed_sec'] is None and full['longest_worker_elapsed_sec'] == max(r['elapsed_sec'] for r in workers)
    assert full['offline_child_cleanup_required'] is None and full['offline_child_cleanup_signals'] is None
    assert full['launcher_error'] == receipt['original_launcher']
    assert full['launcher_error']['observed_exit_code'] == 143 and full['launcher_error']['cause'] == 'undetermined'
    assert full['accepted_for_current_release'] is False and receipt['requires_independent_release_review'] is True
    assert receipt['tests_reexecuted'] is False and receipt['runtime_reloaded'] is False
    assert not full['missing_worker_results'] and not full['duplicate_case_identities'] and not full['missing_case_identities'] and not full['unexpected_case_identities']
    assert not full['residual_offline_process_groups'] and not full['external_boundary_attempts'] and not full['foreign_or_inconsistent_imports']
    for p, h in receipt['dependency_sha256'].items():
        assert sha(p) == h; bindings[p] = h
    live = []; read_errors = []
    for p in Path('/proc').iterdir():
        if not p.name.isdigit(): continue
        try:
            ss = (p/'stat').read_text().rsplit(')', 1)[1].split()
            if int(ss[2]) in pgids or int(p.name) == 32756:
                live.append(dict(pid=int(p.name), ppid=int(ss[1]), pgid=int(ss[2]), start_ticks=int(ss[19]), state=ss[0]))
        except (FileNotFoundError, ProcessLookupError): pass
        except PermissionError: read_errors.append(str(p))
    assert not live and not read_errors
    assert all(sha(p) == h for p, h in bindings.items())
    result = dict(kind='GEN54_completed_worker_recovery_independent_review', passed=True,
        review_scope='exact_completed_worker_recovery', reviewer='root/gen54_runtime_efficiency',
        recovery_author_and_executor='root', reviewer_executed_recovery=False,
        full_result_sha256=FULL_SHA, recovery_receipt_sha256=REC_SHA, recovery_helper_sha256=HELPER_SHA,
        reviewed_helper_path=str(HELPER), original_plan_sha256=PLAN_SHA, original_output=str(O),
        source_count=1075, source_sha256=ctx['sources'], summary=full_summary, test_file_count=329,
        workers=workers, worker_wrapper_returncodes=[None]*7, pytest_returncodes=[0]*7,
        original_launcher_returncode_known=False, tool_session_exit_code_reported_by_root=143,
        original_launcher_interruption_cause='undetermined', original_parent_cleanup_status='unknown',
        elapsed_sec=None, longest_worker_elapsed_sec=full['longest_worker_elapsed_sec'],
        loaded_changed_sources=sorted(loaded_global), live_original_groups=live, proc_read_errors=read_errors,
        original_parent_absent=True, original_pgids=pgids, boundary_files_absent=boundary_absent,
        recovery_source_review=dict(exact_plan_runner_manifest_pins=True,
            original_runner_current_context_and_stable_reused=True,
            original_worker_evidence_preserved=True, original_parent_or_wrapper_success_not_inferred=True,
            no_test_or_process_control_entrypoint_called=True,
            process_group_check_note='Recovery uses inherited killpg(group, 0) existence probe; no signal is delivered. This independent audit uses only /proc reads.',
            recovery_output_created_only_if_absent=True,
            remaining_release_checks_required='Exact recovery review and all existing sealer source/worker/XML/import/guard checks; no general interrupted-run exception'),
        dependency_sha256=bindings, compatibility_dependencies_verified=len(ctx['compatibility_dependencies']),
        generated_imports_verified=len(ctx['generated_imports']),
        ROS_or_hardware_accessed=False, tests_reexecuted=False, signals_sent=False,
        runtime_reloaded=False, physical_grasp_verified=False, recorded_ns=time.time_ns(),
        review_helper_sha256=sha(__file__))
    OUT.mkdir()
    with (OUT/'result.json').open('x') as f:
        json.dump(result, f, ensure_ascii=False, indent=2, allow_nan=False); f.write('\n')
    print(json.dumps(dict(passed=True, result=str(OUT/'result.json'), sha256=sha(OUT/'result.json'),
        tests=9255, workers=7, sources=1075, dependencies=len(bindings)), ensure_ascii=False))


if __name__ == '__main__':
    main()
