"""Independently preserve one complete failed GEN57 run; never accepts a release.

The reviewed acceptance verifier is unchanged. This companion validates exact
failed-case evidence, imports, isolation, source bytes, and the real unit exit.
"""
from collections import Counter
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import time
import xml.etree.ElementTree as ET

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
O = V/'gen57_full_parallel_r1'
OUT = V/'gen57_regression_independent_full_r1'
M = V/'gen57_candidate_full_ready_r1/candidate_manifest.json'
MSHA = '42929a2e0ba6f1984ba29af284239eefaf179e6c51edcde4c4a9e6cd8a912409'
R = W/'.runtime/verify_gen57_parallel_offline_r2_20261020.py'
RSHA = '0e217bc784b24c603afa1f4bca7c977f35f9036c03b6b263c9801f749d1c2727'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes())


def save(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())


def main():
    assert E.parent.parent.is_mount() and not OUT.exists()
    vp = W/'.runtime/gen57_offline_result_verifier_r2_20261020.py'
    assert sha(vp) == '8fb10512bffef2f34d922fd6baac6747a3df3f6d8c7688d13485697aff2b3ed9'
    spec = importlib.util.spec_from_file_location('gen57_original_result_verifier', vp)
    verifier = importlib.util.module_from_spec(spec); spec.loader.exec_module(verifier)
    manifest, context, runner = verifier.candidate_context(M, MSHA, R, RSHA)
    serial = runner.load_serial(); serial.storage(OUT)
    workspace = Path(manifest['workspace_root'])
    p = O/'result.json'; result = read(p); plan = read(O/'plan.json')
    evidence = {str(p):sha(p), str(M):MSHA, str(R):RSHA, str(vp):sha(vp)}
    assert result['passed'] is False and result['full_regression_passed'] is False
    assert result['mode'] == 'full' and result['full_regression_scope'] is True
    assert result['candidate_manifest_sha256'] == MSHA and result['runner_sha256'] == RSHA
    assert result['candidate_manifest_path'] == str(M)
    assert result['source_sha256'] == plan['context']['sources'] == context['sources']
    assert result['source_count'] == len(context['sources']) == 1088
    assert result['test_scope_files'] == len(context['scope']) == 342
    assert all(result[k] is True for k in ('scope_complete', 'source_unchanged',
        'test_bodies_executed', 'combined_xml_matches_worker_results'))
    assert not any(result[k] for k in ('interrupted', 'launcher_error', 'offline_child_cleanup_required',
        'residual_offline_process_groups', 'external_boundary_attempts', 'foreign_or_inconsistent_imports',
        'duplicate_case_identities', 'missing_case_identities', 'unexpected_case_identities', 'missing_worker_results'))
    assert sha(O/'plan.json') == result['plan_sha256']
    assert plan['expected_case_count'] == 9766 and plan['test_scope'] == context['scope']
    assert plan['mode'] == 'full' and plan['runner_sha256'] == RSHA
    assert plan['candidate_manifest_sha256'] == MSHA
    assert runner.stable(serial, plan)
    assert Counter(map(tuple, plan['expected_case_identities'])) == Counter(map(tuple, context['case_identities']))
    summary, cases, _ = runner.inventory(O/'pytest.xml')
    assert sha(O/'pytest.xml') == result['xml_sha256'] and summary == result['summary']
    assert summary['tests'] == 9766 and summary['failures'] > 0 and summary['errors'] == summary['skipped'] == 0
    assert Counter(map(tuple, cases)) == Counter(map(tuple, context['case_identities']))
    child_union = []; failures = []; worker_details = []; provenance_count = 0
    globally_loaded = {name:False for name in context['required_changed_sources']}
    assert len(result['worker_result_sha256']) == plan['worker_count'] == result['worker_count'] == 7
    seen = set()
    for filename, digest in result['worker_result_sha256'].items():
        path = Path(filename); child = read(path); index = child['worker_index']; folder = path.parent
        assert index not in seen and index in range(7)
        seen.add(index)
        assert path == O/('worker_%d'%index)/'result.json' and sha(path) == digest
        shard = plan['shards'][index]
        assert child['mode'] == 'full' and child['plan_sha256'] == result['plan_sha256']
        assert all(child[k] is True for k in ('scope_complete','source_unchanged','direct_child_reaped','test_bodies_executed'))
        assert not any(child[k] for k in ('interrupted','direct_child_cleanup_signals','external_boundary_attempts','foreign_or_inconsistent_imports'))
        assert child['test_scope'] == shard['scope']
        assert child['returncode'] == result['worker_returncodes'][index]
        for file, key in [('pytest.xml','xml_sha256'),('pytest.log','log_sha256'),('module_provenance.json','module_provenance_sha256')]:
            assert sha(folder/file) == child[key]
            evidence[str(folder/file)] = child[key]
        assert child['generated_bootstrap_sha256'] == {'bootstrap/sitecustomize.py':context['guard_sha256'], 'bootstrap/gen42_import_audit.py':context['audit_sha256']}
        for file, digest in child['generated_bootstrap_sha256'].items():
            assert sha(folder/file) == digest
            evidence[str(folder/file)] = digest
        boundary = folder/'boundary_attempts.jsonl'
        assert not boundary.exists() or not boundary.read_text().splitlines()
        cs, cc, _ = runner.inventory(folder/'pytest.xml')
        assert cs == child['summary'] and cc == child['case_identities']
        assert Counter(map(tuple, cc)) == Counter(map(tuple, shard['case_identities']))
        assert child['passed'] == (child['returncode'] == 0 and cs['failures'] == cs['errors'] == cs['skipped'] == 0)
        provenance = read(folder/'module_provenance.json'); assert provenance
        supplemental, loaded_supplemental = runner.supplemental_load_evidence(context, shard, provenance)
        assert child['required_supplemental_imports'] == supplemental
        assert child['loaded_supplemental_imports'] == loaded_supplemental and all(loaded_supplemental.values())
        for label, imported in provenance.items():
            module_path = Path(imported['path']); digest = imported['sha256']
            if serial.inside(module_path, workspace/'src'):
                valid = context['sources'].get(str(module_path.relative_to(workspace))) == digest
            elif serial.inside(module_path, W/'devel'):
                valid = context['generated_imports'].get(str(module_path)) == digest
            elif label in supplemental:
                valid = supplemental[label] == imported
            else:
                valid = label == serial.ORACLE_LABEL and module_path == serial.ORACLE and digest == serial.ORACLE_SHA and serial.ORACLE_TEST in plan['test_scope'] and context['sources'].get(serial.ORACLE_TEST) == serial.ORACLE_TEST_SHA
            assert valid and sha(module_path) == digest
            provenance_count += 1
        loaded = {name:any(row['path'] == str(workspace/name) and row['sha256'] == context['sources'][name] for row in provenance.values()) for name in context['required_changed_sources']}
        assert child['loaded_changed_sources'] == loaded
        expected_required = sorted({name for test in shard['scope'] for name in context['required_imports_by_test_file'].get(test, [])})
        assert shard['required_changed_sources'] == child['required_changed_sources'] == expected_required
        assert all(loaded[name] for name in expected_required)
        for name in globally_loaded:
            globally_loaded[name] = globally_loaded[name] or loaded[name]
        for case in ET.parse(folder/'pytest.xml').getroot().iter('testcase'):
            for failure in case.findall('failure'):
                failures.append(dict(worker_index=index, case_identity=runner.case_identity(case),
                    message=failure.get('message'), traceback=failure.text, failure_type=failure.get('type'),
                    source_sha256=context['sources'][case.get('file')]))
        child_union.extend(cc); evidence[str(path)] = sha(path)
        worker_details.append(dict(index=index, summary=cs, returncode=child['returncode'],
            elapsed_sec=child['elapsed_sec'], cpu_user_sec=child['pytest_cpu_user_sec'],
            cpu_system_sec=child['pytest_cpu_system_sec'], max_child_rss_kib=child['reaped_child_max_rss_kib'],
            provenance_rows=len(provenance), required_sources_loaded=True,
            passed=child['passed'], result_sha256=sha(path)))
    assert Counter(map(tuple, child_union)) == Counter(map(tuple, cases))
    assert len(failures) == summary['failures'] and all(globally_loaded.values())
    assert globally_loaded == result['loaded_changed_sources_global']
    launch = read(O/'durable_launch_request.json'); submitted = read(O/'durable_launch_submitted.json')
    identity = read(O/'coordinator_identity.json'); unit = launch['unit']
    assert submitted['submitted'] is True and submitted['returncode'] == 0
    assert submitted['request_sha256'] == sha(O/'durable_launch_request.json')
    assert launch['plan_sha256'] == submitted['plan_sha256'] == identity['plan_sha256'] == result['plan_sha256']
    assert unit == submitted['unit'] == identity['durable_unit'] == 'alicia-gen57-offline-full-r1-20261020.service'
    probe = subprocess.run(['systemctl','--user','show',unit,
        '--property=Id,LoadState,ActiveState,SubState,MainPID,ControlGroup,InvocationID,Result,ExecMainCode,ExecMainStatus,ExecMainStartTimestamp,ExecMainExitTimestamp'], text=True,capture_output=True,timeout=10)
    assert probe.returncode == 0
    props = dict(line.split('=',1) for line in probe.stdout.splitlines())
    assert props['Id'] == unit and props['Result'] == 'exit-code'
    assert props['ExecMainCode'] == '1' and props['ExecMainStatus'] == '1'
    assert props['MainPID'] == '0' and props['ActiveState'] == props['SubState'] == 'failed'
    assert props['InvocationID'] == identity['systemd_invocation_id']
    measured = result['measured_workload']; wp = Path(measured['path'])
    assert wp == O/'workload_samples.jsonl' and sha(wp) == measured['sha256']
    samples = [json.loads(line) for line in wp.read_text().splitlines()]
    assert len(samples) == measured['sample_count'] and len(samples) > 1
    live_audit = read(O/'independent_live_environment_audit.json')
    assert live_audit['passed'] is True and len(live_audit['observed_pytest_processes']) == 7
    assert live_audit['plan_sha256'] == result['plan_sha256']
    evidence[str(O/'independent_live_environment_audit.json')] = sha(O/'independent_live_environment_audit.json')
    cpu = result['pytest_cpu_user_sec'] + result['pytest_cpu_system_sec']
    failure_files = dict(Counter(f['case_identity'][0] for f in failures))
    def failure_pattern(failure):
        text = (failure['message'] or '') + '\n' + (failure['traceback'] or '')
        if "'accepted_positions_rad'" in text:
            return 'accepted_positions_rad missing in observation measurement / station owner'
        return re.sub(r'0x[0-9a-fA-F]+', '<address>', (failure['message'] or '')[:500])
    failure_messages = dict(Counter(failure_pattern(f) for f in failures))
    assert runner.current_context(serial) == context
    OUT.mkdir()
    save(OUT/'failures.json', failures)
    save(OUT/'unit_exit.json', dict(properties=props, query_stdout=probe.stdout, at_ns=time.time_ns()))
    report = dict(kind='GEN57_complete_failed_full_independent_review', review_complete=True,
        evidence_consistent=True, full_regression_passed=False, full_execution_complete=True,
        result_path=str(p), result_sha256=sha(p), xml_sha256=result['xml_sha256'],
        summary=summary, passed_test_count=summary['tests']-summary['failures'],
        candidate_manifest_sha256=MSHA, source_count=1088, source_unchanged=True,
        source_mapping_path=str(V/'gen57_regression_independent_preflight_r3/source_mapping_1088.json'),
        source_mapping_sha256=sha(V/'gen57_regression_independent_preflight_r3/source_mapping_1088.json'),
        exact_scope_files=342, exact_cases=9766, baseline_cases=9658, new_cases=108,
        missing_cases=[], duplicate_cases=[], unexpected_cases=[],
        workers=worker_details, worker_returncodes=result['worker_returncodes'],
        provenance_rows_checked=provenance_count, loaded_changed_sources=globally_loaded,
        external_boundary_attempts=[], foreign_or_changed_imports=[],
        guard_sha256=context['guard_sha256'], audit_sha256=context['audit_sha256'],
        failure_files=failure_files, failure_messages=failure_messages,
        failures_path=str(OUT/'failures.json'), failures_sha256=sha(OUT/'failures.json'),
        elapsed_sec=result['elapsed_sec'], pytest_cpu_user_sec=result['pytest_cpu_user_sec'],
        pytest_cpu_system_sec=result['pytest_cpu_system_sec'], mean_cpu_core_equivalents=cpu/result['elapsed_sec'],
        measured_workload=measured, unit_exit_sha256=sha(OUT/'unit_exit.json'), unit_properties=props,
        evidence_sha256=evidence, reviewer_sha256=sha(__file__), at_ns=time.time_ns(),
        existing_acceptance_guard_changed=False, release_accepted=False,
        tests_executed_by_review=False, candidate_modified=False, ROS_or_hardware_used=False,
        physical_grasp_verified=False)
    save(OUT/'result.json', report)
    print(json.dumps({k:report[k] for k in ('review_complete','evidence_consistent','full_regression_passed','summary','failure_files','failure_messages','elapsed_sec','mean_cpu_core_equivalents','result_sha256','xml_sha256')}, ensure_ascii=False, indent=2))
    print('review', OUT/'result.json', sha(OUT/'result.json'))


if __name__ == '__main__':
    main()
