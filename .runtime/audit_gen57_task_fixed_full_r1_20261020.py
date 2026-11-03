"""Independent evidence readback for the reviewed GEN57 task contract fix.

Only the original frozen validator accepts tests. This helper adds measured
process/environment, exact unit exit, and external lossless source evidence.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import time

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
D = W/'.runtime/gen57_candidate_r1'
M = V/'gen57_candidate_full_ready_r2/candidate_manifest.json'
M_SHA = 'b7f8cdbfb21b58447e6ad49f499bdd5215a4bdf5cc3af64c0557b6f12e8571bd'
O = V/'gen57_full_parallel_r2'
OUT = V/'gen57_regression_independent_full_r2'
R = W/'.runtime/verify_gen57_parallel_offline_r2_20261020.py'
R_SHA = '0e217bc784b24c603afa1f4bca7c977f35f9036c03b6b263c9801f749d1c2727'
UNIT = 'alicia-gen57-offline-full-r2-20261020.service'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes())


def save(path, value):
    with path.open('x') as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n'); f.flush(); os.fsync(f.fileno())


def proc_rows(groups):
    result = []
    for path in Path('/proc').iterdir():
        if not path.name.isdecimal():
            continue
        try:
            raw = (path/'stat').read_text(); row = raw[raw.rfind(')')+2:].split()
            if int(row[2]) in groups:
                result.append(dict(pid=int(path.name), ppid=int(row[1]), pgid=int(row[2]),
                    state=row[0], start_ticks=int(row[19])))
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    return result


def unit_properties(unit):
    assert unit in (UNIT, 'alicia-gen57-offline-focus-r2-20261020.service')
    value = subprocess.run(['systemctl','--user','show',unit,
        '--property=Id,LoadState,ActiveState,SubState,MainPID,ControlGroup,InvocationID,Result,ExecMainCode,ExecMainStatus,ExecMainStartTimestamp,ExecMainExitTimestamp'],
        text=True, capture_output=True, timeout=10)
    assert value.returncode == 0
    return dict(line.split('=',1) for line in value.stdout.splitlines())


def live():
    plan = read(O/'plan.json')
    assert plan['candidate_manifest_sha256'] == M_SHA and plan['worker_count'] == 7
    union = [tuple(c) for shard in plan['shards'] for c in shard['case_identities']]
    files = [p for shard in plan['shards'] for p in shard['scope']]
    assert len(union) == len(set(union)) == 9766
    assert len(files) == len(set(files)) == 342
    assert Counter(union) == Counter(map(tuple, plan['expected_case_identities']))
    owned = read(O/'owned_workers.json'); groups = {row['pgid']:row['index'] for row in owned['workers']}
    keys = ['OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS',
        'PYTHONDONTWRITEBYTECODE','PYTHONNOUSERSITE','PYTEST_DISABLE_PLUGIN_AUTOLOAD',
        'ROS_MASTER_URI','ROS_LOG_DIR','ROS_HOME','TMPDIR','TMP','TEMP','XDG_CACHE_HOME',
        'PYTHONPYCACHEPREFIX','OFFLINE_BOUNDARY_LOG','GEN42_MODULE_PROVENANCE','PYTHONPATH','ROS_PACKAGE_PATH']
    result = []
    for row in proc_rows(groups):
        path = Path('/proc')/str(row['pid'])
        try:
            cmd = (path/'cmdline').read_bytes().split(b'\0')
            if cmd[1:3] != [b'-m',b'pytest']:
                continue
            env = dict(v.split(b'=',1) for v in (path/'environ').read_bytes().split(b'\0') if b'=' in v)
            selected = {k:env.get(k.encode(),b'').decode() for k in keys}
            index = groups[row['pgid']]; folder = O/('worker_%d'%index)
            assert row['ppid'] == row['pgid']
            assert all(selected[k] == '1' for k in keys[:7])
            assert selected['ROS_MASTER_URI'] == 'http://127.0.0.1:1'
            for k, sub in [('ROS_LOG_DIR','ros'),('ROS_HOME','ros_home'),('TMPDIR','tmp'),('TMP','tmp'),('TEMP','tmp'),
                    ('XDG_CACHE_HOME','cache'),('PYTHONPYCACHEPREFIX','pycache'),
                    ('OFFLINE_BOUNDARY_LOG','boundary_attempts.jsonl'),('GEN42_MODULE_PROVENANCE','module_provenance.json')]:
                assert selected[k] == str(folder/sub)
            result.append(dict(row, worker_index=index, environment=selected, cwd=str((path/'cwd').resolve())))
        except (FileNotFoundError, ProcessLookupError):
            continue
    assert len(result) == 7
    p = O/'independent_live_environment_audit.json'
    save(p,dict(passed=True, at_ns=time.time_ns(), plan_sha256=sha(O/'plan.json'),
        owned_workers_sha256=sha(O/'owned_workers.json'), observed_pytest_processes=result,
        exact_unique_cases=9766, exact_unique_files=342, reviewer_sha256=sha(__file__), ROS_or_hardware_used=False))
    print(json.dumps(dict(path=str(p),sha256=sha(p),observed_pytest_processes=len(result)),ensure_ascii=False))


def final():
    assert not OUT.exists()
    vp = W/'.runtime/gen57_offline_result_verifier_r2_20261020.py'
    assert sha(vp) == '8fb10512bffef2f34d922fd6baac6747a3df3f6d8c7688d13485697aff2b3ed9'
    spec = importlib.util.spec_from_file_location('gen57_original_acceptance_verifier',vp)
    verifier = importlib.util.module_from_spec(spec); spec.loader.exec_module(verifier)
    manifest, context, result, evidence = verifier.validate_result(O/'result.json',M,M_SHA,R,R_SHA,'full')
    assert result['summary'] == dict(tests=9766,failures=0,errors=0,skipped=0)
    assert result['source_count'] == 1088 and result['test_scope_files'] == 342 and result['worker_count'] == 7
    assert result['resource_budget']['reserved_cpu_count'] == 5
    root_review = V/'gen57_root_task_contract_fix_review_r1/result.json'
    assert sha(root_review) == '33bd977184f688d0c56088392b88f532f49d6d88d0d8a2147640926c1db55d50'
    assert read(root_review)['passed'] is True and read(root_review)['source_sha256'] == context['sources']
    old = read(V/'gen57_before_task_contract_fix_r1/candidate_manifest.json')
    changed = [p for p in old['source_sha256'] if old['source_sha256'][p] != context['sources'][p]]
    station = 'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_task.py'
    assert changed == [station] and context['sources'][station] == '7dd5bb932609cde794ed956be32b8db1bef275d2faf383d738e809bf4fcecd8d'
    unit_proofs = {}
    for folder, unit in [(O,UNIT),(V/'gen57_focus_r2','alicia-gen57-offline-focus-r2-20261020.service')]:
        launch=read(folder/'durable_launch_request.json');submitted=read(folder/'durable_launch_submitted.json')
        identity=read(folder/'coordinator_identity.json'); current_result=read(folder/'result.json')
        assert launch['unit'] == submitted['unit'] == identity['durable_unit'] == unit
        assert submitted['submitted'] is True and submitted['returncode'] == 0
        assert submitted['request_sha256'] == sha(folder/'durable_launch_request.json')
        assert launch['plan_sha256'] == identity['plan_sha256'] == current_result['plan_sha256'] == sha(folder/'plan.json')
        props=unit_properties(unit)
        assert props['Result']=='success' and props['ExecMainCode']=='1' and props['ExecMainStatus']=='0'
        assert props['MainPID']=='0' and props['ActiveState']=='active' and props['SubState']=='exited'
        assert props['InvocationID']==identity['systemd_invocation_id']
        groups={row['pgid'] for row in read(folder/'owned_workers.json')['workers']}
        assert not proc_rows(groups)
        unit_proofs[unit]=dict(properties=props,closed_owned_pgids=sorted(groups),matching_processes=[])
    focus=read(V/'gen57_focus_r2/result.json')
    assert focus['summary']==dict(tests=950,failures=0,errors=0,skipped=0)
    assert focus['source_sha256']==context['sources']
    collect=read(V/'gen57_collect_r3/result.json');prior_collect=read(V/'gen57_collect_r2/result.json')
    assert collect['passed'] and not collect['test_bodies_executed'] and collect['collected_case_count']==108
    assert Counter(map(tuple,collect['collected_case_identities']))==Counter(map(tuple,prior_collect['collected_case_identities']))
    work=result['measured_workload'];wp=Path(work['path'])
    assert wp==O/'workload_samples.jsonl' and sha(wp)==work['sha256']
    samples=[json.loads(line) for line in wp.read_text().splitlines()]
    assert len(samples)==work['sample_count'] and len(samples)>1
    assert max(row['offline_process_groups']['simultaneous_rss_kib'] for row in samples)==work['peak_sampled_group_rss_kib']
    live=read(O/'independent_live_environment_audit.json')
    assert live['passed'] and live['plan_sha256']==result['plan_sha256'] and len(live['observed_pytest_processes'])==7
    OUT.mkdir();mapping=[]
    overlay=set(manifest['allowed_production_paths'])|{r['path'] for r in manifest['new_tests']}
    assert len(overlay)==7
    for path,digest in sorted(context['sources'].items()):
        assert sha(D/path)==digest
        row=dict(relative_path=path,candidate_path=str(D/path),sha256=digest)
        if path in overlay:
            dest=OUT/'frozen_sources'/path;dest.parent.mkdir(parents=True,exist_ok=True)
            with dest.open('xb') as f:
                f.write((D/path).read_bytes());f.flush();os.fsync(f.fileno())
            assert sha(dest)==digest
            row['archived_path']=str(dest)
        else:
            baseline=W/'.runtime/gen57_baseline_source'/path
            assert sha(baseline)==digest
            row['baseline_path']=str(baseline)
        mapping.append(row)
    save(OUT/'source_mapping_1088.json',dict(source_count=1088,overlay_count=7,manifest_sha256=M_SHA,sources=mapping,readback_exact=True))
    save(OUT/'unit_closure.json',dict(at_utc=datetime.now(timezone.utc).isoformat(),units=unit_proofs))
    cpu=result['pytest_cpu_user_sec']+result['pytest_cpu_system_sec']
    report=dict(kind='GEN57_task_contract_fix_complete_independent_evidence_review',passed=True,
        full_regression_passed=True,source_unchanged=True,scope_complete=True,summary=result['summary'],
        source_count=1088,test_scope_files=342,baseline_case_count=9658,new_case_count=108,
        candidate_manifest_path=str(M),candidate_manifest_sha256=M_SHA,result_path=str(O/'result.json'),result_sha256=sha(O/'result.json'),
        xml_sha256=result['xml_sha256'],plan_sha256=result['plan_sha256'],runner_sha256=R_SHA,
        focus_result_sha256=sha(V/'gen57_focus_r2/result.json'),focus_summary=focus['summary'],focus_elapsed_sec=focus['elapsed_sec'],
        collect_result_sha256=sha(V/'gen57_collect_r3/result.json'),collect_bodies_executed=False,
        new_case_identities_exactly_equal_old108=True,independent_patch_review_sha256=sha(root_review),
        source_mapping_sha256=sha(OUT/'source_mapping_1088.json'),unit_closure_sha256=sha(OUT/'unit_closure.json'),
        live_environment_sha256=sha(O/'independent_live_environment_audit.json'),worker_returncodes=result['worker_returncodes'],
        loaded_changed_sources=result['loaded_changed_sources_global'],external_boundary_attempts=[],foreign_or_inconsistent_imports=[],
        duplicate_cases=[],missing_cases=[],unexpected_cases=[],guard_sha256=context['guard_sha256'],audit_sha256=context['audit_sha256'],
        elapsed_sec=result['elapsed_sec'],pytest_cpu_user_sec=result['pytest_cpu_user_sec'],pytest_cpu_system_sec=result['pytest_cpu_system_sec'],
        mean_cpu_core_equivalents=cpu/result['elapsed_sec'],measured_workload=work,
        historical_failed_full=dict(path=str(V/'gen57_full_parallel_r1/result.json'),sha256=sha(V/'gen57_full_parallel_r1/result.json'),
            summary=read(V/'gen57_full_parallel_r1/result.json')['summary'],original_seven_sources_frozen=str(V/'gen57_before_task_contract_fix_r1/frozen_sources')),
        evidence_sha256=evidence,reviewer_sha256=sha(__file__),at_utc=datetime.now(timezone.utc).isoformat(),
        supplemental28_26_12_not_in_formal9766=True,acceptance_guard_modified=False,tests_executed_by_review=False,
        production_W_modified=False,ROS_or_hardware_used=False,deployed=False,physical_grasp_verified=False)
    save(OUT/'result.json',report)
    print(json.dumps({k:report[k] for k in ('passed','full_regression_passed','summary','elapsed_sec','mean_cpu_core_equivalents','result_sha256','xml_sha256','source_mapping_sha256')},ensure_ascii=False,indent=2))
    print('review',OUT/'result.json',sha(OUT/'result.json'))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('phase',choices=('live','final'));args=parser.parse_args()
    assert E.parent.parent.is_mount() and sha(M)==M_SHA and sha(R)==R_SHA
    {'live':live,'final':final}[args.phase]()
