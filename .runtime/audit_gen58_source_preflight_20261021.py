"""File/proc-only source and completed-compute preflight; no runtime imports.

This writes a new immutable SanStand audit. It executes no tests, installer,
ROS API, network request, device access, service mutation or motion operation.
"""
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
import xml.etree.ElementTree as ET

W=Path(__file__).resolve().parents[1]
MOUNT=Path('/media/zhuyupei/SanStand')
E=MOUNT/'实验记录/Robot-Controller'
N=E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
V=N/'verification'
OUTPUT=N/'gen58_resume_r1/verification/source_preflight_r1'
RELEASE=V/'gen57_release_manifest.json'
MANIFEST=V/'missing_depth_prior_freeze_r2/manifest_closed.json'
FULL=V/'missing_depth_prior_full_r2/result.json'
FOCUS=V/'missing_depth_prior_focus_r2/result.json'
INDEPENDENT=FULL.parent/'independent_verification.json'
FINAL_PROCESS=FULL.parent/'final_process_receipt.json'
RUNNER=W/'.runtime/verify_missing_depth_prior_20261020.py'
VERIFIER=W/'.runtime/check_missing_depth_prior_receipts_20261020.py'
PRODUCTION='src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_instance.py'
CONTRACT='src/alicia_grasp_modes/tests/test_observation_arrival_probable_foreground.py'
NEW_TEST='src/alicia_grasp_modes/tests/test_missing_depth_rgb_prior.py'
HELPERS=[W/'.runtime'/name for name in (
    'install_and_seal_gen57_20261020.py','install_and_seal_gen57_cold_20261020.py',
    'gen57_runtime_common_20261020.py','gen57_offline_result_verifier_r2_20261020.py',
    'gen57_cold_install_guard_20261020.py','start_gen57_full_cold_restart_20261020.py',
    'gen57_full_cold_restart_common_20261020.py','verify_gen57_full_cold_restart_20261020.py',
    'finalize_gen57_idle_deployment_20261020.py','verify_active_remote_readiness_20261020.py')]


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_bytes())
def ref(path):return dict(path=str(path),sha256=sha(path))
def save(path,value):
    raw=(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
    with path.open('xb') as stream:stream.write(raw);stream.flush();os.fsync(stream.fileno())
    assert path.read_bytes()==raw

def proc(pid):
    try:
        path=Path('/proc')/str(pid);raw=(path/'stat').read_text();fields=raw[raw.rfind(')')+2:].split()
        return dict(pid=int(pid),state=fields[0],parent_pid=int(fields[1]),pgid=int(fields[2]),
            session=int(fields[3]),start_ticks=int(fields[19]))
    except (FileNotFoundError,ProcessLookupError):return None

def check_sources(root,source):
    matches=Counter();mismatches=[];actual={}
    for path,expected in source.items():
        category='test_or_fixture' if '/tests/' in path else ('production' if path.startswith('src/') else 'other_frozen_input')
        try:observed=sha(root/path);actual[path]=observed
        except OSError as exc:mismatches.append(dict(path=path,category=category,expected_sha256=expected,error=repr(exc)));continue
        if observed==expected:matches[category]+=1
        else:mismatches.append(dict(path=path,category=category,expected_sha256=expected,actual_sha256=observed))
    return dict(root=str(root),expected_source_count=len(source),matched_count=sum(matches.values()),
        matched_counts_by_category=dict(matches),mismatches=mismatches,all_match=not mismatches),actual

def main():
    started=time.time_ns()
    assert MOUNT.is_mount() and not os.statvfs(MOUNT).f_flag & os.ST_RDONLY
    assert E.stat().st_dev==MOUNT.stat().st_dev
    config=read(E/'grasp_records/archive_config.json');policy=read(E/'maintenance/storage_control.json')
    assert config['upload_enabled'] is False and config['delete_enabled'] is False and policy['archive_policy']=='local_only'
    OUTPUT.parent.mkdir(parents=True,exist_ok=True);OUTPUT.mkdir(exist_ok=False)
    save(OUTPUT/'claim.json',dict(created_ns=started,helper=ref(Path(__file__).resolve()),
        scope='File/proc-only audit; no tests or runtime operations executed',pid=os.getpid()))
    manifest,full,focus,release,independent=map(read,(MANIFEST,FULL,FOCUS,RELEASE,INDEPENDENT))
    candidate=Path(manifest['workspace_root']);sources=manifest['source_sha256']
    immutable_inputs=[RELEASE,MANIFEST,FULL,FOCUS,INDEPENDENT,FINAL_PROCESS,RUNNER,VERIFIER,
        FULL.parent/'plan.json',FULL.parent/'pytest.xml',FULL.parent/'owned_workers.json',FULL.parent/'coordinator_identity.json']+HELPERS
    input_hashes={str(p):sha(p) for p in immutable_inputs}
    binding_checks=dict(full_passed=full['passed'] is True and full['full_regression_passed'] is True,
        exact_summary=full['summary']==dict(tests=9893,failures=0,errors=0,skipped=0),
        full_manifest_hash=sha(MANIFEST)==full['candidate_manifest_sha256']==independent['manifest_sha256'],
        full_result_hash=sha(FULL)==independent['full_result_sha256'],
        full_plan_hash=sha(FULL.parent/'plan.json')==full['plan_sha256'],
        full_xml_hash=sha(FULL.parent/'pytest.xml')==full['xml_sha256'],
        runner_hash=sha(RUNNER)==full['runner_sha256']==manifest['runner_sha256'],
        verifier_hash=sha(VERIFIER)==independent['verifier_sha256'],
        focus_same_source=focus['passed'] is True and focus['source_sha256']==sources,
        full_same_source=full['source_sha256']==sources,
        independent_passed=independent['passed'] is True,
        count_partition=independent['historical_case_count']+independent['supplemental_existing_cases']+independent['new_cases']==9893)
    frozen,freeze_actual=check_sources(candidate,sources);live,live_actual=check_sources(W,sources)
    copied_mismatches=[dict(path=p,expected_sha256=h,actual_sha256=sha(candidate/p)) for p,h in manifest['copied_input_sha256'].items() if sha(candidate/p)!=h]
    old=release['source_sha256'];changed=[dict(path=p,old_release_sha256=old[p],tested_candidate_sha256=sources[p],live_sha256=live_actual.get(p)) for p in old if sources.get(p)!=old[p]]
    added=[dict(path=p,tested_candidate_sha256=sources[p],live_sha256=live_actual.get(p)) for p in sources if p not in old]
    xml=ET.parse(FULL.parent/'pytest.xml').getroot();cases=[(c.get('file',''),c.get('classname',''),c.get('name','')) for c in xml.iter('testcase')]
    exact_cases=len(cases)==len(set(cases))==9893 and sorted(cases)==sorted(map(tuple,manifest['context']['case_identities']))
    worker_artifacts=[]
    for filename,digest in full['worker_result_sha256'].items():
        path=Path(filename);row=read(path);folder=path.parent
        checks=dict(result_hash=sha(path)==digest,passed=row['passed'] is True,
            plan_hash=row['plan_sha256']==full['plan_sha256'],
            xml_hash=sha(folder/'pytest.xml')==row['xml_sha256'],log_hash=sha(folder/'pytest.log')==row['log_sha256'],
            provenance_hash=sha(folder/'module_provenance.json')==row['module_provenance_sha256'],
            bootstrap_hashes=all(sha(folder/p)==h for p,h in row['generated_bootstrap_sha256'].items()),
            no_boundary_attempts=not row['external_boundary_attempts'] and not row['foreign_or_inconsistent_imports'])
        boundary=folder/'boundary_attempts.jsonl';checks['no_appended_boundary_attempts']=not boundary.exists() or not boundary.read_text().strip()
        worker_artifacts.append(dict(worker_index=row['worker_index'],checks=checks,passed=all(checks.values())))
    coordinator=read(FULL.parent/'coordinator_identity.json');owners=read(FULL.parent/'owned_workers.json')
    previous=[dict(role='coordinator',pid=coordinator['pid'],identity=coordinator['identity'])]+[
        dict(role='worker_'+str(row['index']),pid=row['pid'],identity=row['identity']) for row in owners['workers']]
    identity_checks=[]
    for row in previous:
        now=proc(row['pid']);identity_checks.append(dict(role=row['role'],recorded_identity=row['identity'],current=now,
            same_process_remains=now is not None and now['start_ticks']==int(row['identity']['start_ticks'])))
    groups={r['pid'] for r in owners['workers']};remaining_groups=[];computations=[];unreadable=[]
    wanted={'verify_missing_depth_prior_20261020.py','verify_gen57_parallel_offline_r2_20261020.py',
        'probe_gen57_native_seed_20261021.py','probe_gen57_unknown_depth_prior_20261020.py',
        'probe_gen57_seed_surface_agent_r1.py','replay_masks_r2.py','extract_rgbd_inputs.py',
        'run_recorded_grasp.py','gen57_cold_attempt_entry_20261021.py','gen57_attempt_qualification_replay.py'}
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():continue
        try:
            value=proc(path.name)
            if value is not None and value['pgid'] in groups:remaining_groups.append(value)
            argv=[x.decode(errors='replace') for x in (path/'cmdline').read_bytes().split(b'\0') if x]
            if not argv:continue
            scripts=[Path(x).name for x in argv[1:] if x.endswith('.py')]
            if Path(argv[0]).name.startswith('python') and (set(scripts)&wanted or ('-m' in argv and 'pytest' in argv)):
                computations.append(dict(identity=value,scripts=scripts,python_module_pytest='-m' in argv and 'pytest' in argv))
        except (FileNotFoundError,ProcessLookupError):pass
        except PermissionError:unreadable.append(int(path.name))
    unit='alicia-missing-depth-offline-full-r2.service'
    command=['systemctl','--user','show',unit,'--property=Id,LoadState,ActiveState,SubState,MainPID,Result,ExecMainStatus,InvocationID,ControlGroup']
    done=subprocess.run(command,capture_output=True,text=True,timeout=10)
    unit_status=dict(line.split('=',1) for line in done.stdout.splitlines() if '=' in line)
    unit_exit_ok=done.returncode==0 and unit_status.get('MainPID')=='0' and unit_status.get('ExecMainStatus')=='0' and unit_status.get('Result')=='success'
    mem={k:int(v.split()[0]) for k,v in (line.split(':',1) for line in Path('/proc/meminfo').read_text().splitlines())}
    affinity=sorted(os.sched_getaffinity(0));quota={};effective=float(len(affinity))
    cgroup=Path('/sys/fs/cgroup/cpu.max')
    if cgroup.exists():
        maximum,period=cgroup.read_text().split();quota=dict(path=str(cgroup),maximum=maximum,period_us=int(period))
        if maximum!='max':effective=min(effective,int(maximum)/int(period))
    disk=shutil.disk_usage(E)
    resources=dict(sample_kind='read_only_preflight_snapshot_not_task_workload',affinity_cpus=affinity,affinity_count=len(affinity),
        effective_cpu_capacity=effective,os_cpu_count=os.cpu_count(),cgroup_cpu_quota=quota,load_average=list(os.getloadavg()),
        memory_total_mib=mem['MemTotal']/1024.,memory_available_mib=mem['MemAvailable']/1024.,
        sanstand_total_gib=disk.total/1024.**3,sanstand_free_gib=disk.free/1024.**3,
        established_offline_budget=dict(max_workers=6,reserved_cpu_slots=5,reserved_memory_mib=2048,numerical_library_threads=1),
        elapsed_task_time_or_actual_acceleration_claimed=False)
    source_after={p:sha(W/p) for p in sources};input_stable=all(sha(p)==h for p,h in input_hashes.items())
    process_clean=not any(r['same_process_remains'] for r in identity_checks) and not remaining_groups and not computations
    report=dict(schema=1,kind='GEN58_resume_missing_depth_source_preflight_readonly',recorded_ns=time.time_ns(),
        passed=all(binding_checks.values()) and frozen['all_match'] and live['all_match'] and not copied_mismatches
            and exact_cases and all(r['passed'] for r in worker_artifacts) and input_stable and source_after==live_actual and process_clean and unit_exit_ok,
        output=str(OUTPUT),helper=ref(Path(__file__).resolve()),input_sha256=input_hashes,binding_checks=binding_checks,
        historical_release=dict(reference=ref(RELEASE),generation=release['generation'],source_count=release['source_count'],
            test_scope_files=release['test_scope_files'],test_summary=release['test_summary'],
            matches_current_tested_sources=False,not_relabelled_or_rewritten=True),
        tested_candidate=dict(manifest=ref(MANIFEST),full_result=ref(FULL),focus_result=ref(FOCUS),independent_verification=ref(INDEPENDENT),
            source_count=len(sources),test_scope_files=full['test_scope_files'],test_summary=full['summary'],
            historical_cases_current_contract=9766,supplemental_existing_cases=106,new_cases=21,
            explicit_old_test_label_expectations_changed=2,explicit_old_test_parameter_names_mapped=10,
            immutable_manifest_full_regression_passed_field=manifest.get('full_regression_passed'),
            note='The candidate manifest was frozen before execution; its historical full_regression_passed=false is not rewritten. Passing full-result and independent receipts are separately hash-bound.'),
        frozen_sources=frozen,live_sources=live,copied_input_mismatches=copied_mismatches,
        release_to_candidate_changed_existing=changed,release_to_candidate_added=added,
        exact_case_inventory_verified=exact_cases,worker_artifact_checks=worker_artifacts,
        input_receipts_unchanged_during_audit=input_stable,live_sources_unchanged_during_audit=source_after==live_actual,
        process_audit=dict(clean=process_clean,recorded_identity_checks=identity_checks,residual_owned_process_groups=remaining_groups,
            current_matching_compute_processes=computations,unreadable_process_cmdlines=unreadable,
            scan_scope='Exact retained runner/replay/attempt basenames and Python -m pytest; unrelated processes are not claimed absent.',
            durable_unit=dict(unit=unit,query_returncode=done.returncode,status=unit_status,successful_completed_exit=unit_exit_ok)),
        resources=resources,tests_rerun=False,release_created=False,installed_or_reloaded=False,
        runtime_or_remote_readiness_verified=False,physical_grasp_verified=False,
        ROS_API_used=False,network_used=False,hardware_accessed=False,temperature_queried=False,
        motion_commands_sent=False,service_mutations=False,GitHub_used=False,
        storage_policy=dict(upload_enabled=config['upload_enabled'],delete_enabled=config['delete_enabled'],archive_policy=policy['archive_policy']))
    save(OUTPUT/'result.json',report)
    print(json.dumps(dict(passed=report['passed'],output=str(OUTPUT),live_all_match=live['all_match'],live_mismatches=live['mismatches'],
        frozen_all_match=frozen['all_match'],changed_existing=changed,added=added,process_clean=process_clean,
        matching_compute_processes=computations,resources=resources),ensure_ascii=False,indent=2))

if __name__=='__main__':main()
