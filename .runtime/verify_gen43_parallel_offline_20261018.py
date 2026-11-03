"""DRAFT: configurable independent offline pytest processes; plan-only default.

This helper is outside the current release. It does not install/deploy anything
or run on import. The default scope is a fixed twenty-file pilot, never a full
regression. --scope-kind full selects all files; --execute is always explicit.
Every child has separate temporary, ROS-log, Python guard, import-provenance and
XML files. The coordinator requires exact testcase identities, not just totals.
It reuses the frozen serial runner's source/compatibility/guard contracts.
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

W = Path(__file__).resolve().parents[1]
SERIAL = W/'.runtime/verify_gen43_offline_tests_20261018_r2.py'
SERIAL_SHA = 'de1b3727a78644ade82690dbfd748745c4e03969eda18556bd1192b54c55698f'
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
HISTORY = V/'gen43_combined_r2/result.json'
HISTORY_SHA = 'ac2a76cc400ffc644c02ad6175557fa905da94f54d72abbe413e838f5a50e299'
TOTAL_FILES, TOTAL_CASES, SOURCE_COUNT = 288, 8258, 1036
PILOT_FILES = (
    'src/alicia_flexible_grasp_supervisor/tests/test_gen43_top_mask_contract.py',
    'src/alicia_grasp_modes/tests/test_gen43_top_mask_handoff.py',
    'src/alicia_grasp_modes/tests/test_gen38_cuboid_chain.py',
    'src/alicia_grasp_modes/tests/test_gen35_minimal_chain_replay.py',
    'src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py',
    'src/alicia_flexible_grasp_supervisor/tests/test_gen39_registration_snapshot_handoff.py',
    'src/alicia_grasp_modes/tests/test_gen37_measured_aperture_chain.py',
    'src/alicia_flexible_grasp_supervisor/tests/test_gen26_registered_box_support_independent.py',
    'src/alicia_grasp_modes/tests/test_gen22_qualified_rgbd_remote.py',
    'src/alicia_grasp_modes/tests/test_gen11_contact_compute_admission_independent.py',
    'src/alicia_grasp_modes/tests/test_locked_rgb_hold.py',
    'src/alicia_grasp_modes/tests/test_gen36_task_object_order.py',
    'src/alicia_grasp_modes/tests/test_frozen_station_retirement_history.py',
    'src/alicia_grasp_modes/tests/test_gen41_station_private_pending.py',
    'src/alicia_grasp_modes/tests/test_observation_measurement_remote.py',
    'src/alicia_grasp_modes/tests/test_contact_station_retirement.py',
    'src/alicia_grasp_modes/tests/test_measured_face_independent.py',
    'src/alicia_grasp_modes/tests/test_gen31_observation_search_progress.py',
    'src/alicia_grasp_modes/tests/test_gen22_commit_boundary_independent.py',
    'src/alicia_grasp_modes/tests/test_station_fragment_typed_propagation.py',
)
CALIBRATION = V/'gen43_two_process_pilot_monitor_r1/verification.json'
CALIBRATION_SHA = 'e8361d8a905351de2be64cd259008c3bc97193f4a0ec1cb16bf26306ceb3066b'
PRIOR_TWO_WORKER = W/'.runtime/verify_gen43_two_process_draft_20261018.py'
PRIOR_TWO_WORKER_SHA = '82c6efbf6947577c561ea7427d4dd8fd346b16f1a7ee0f5302d384751564be3b'


def resource_budget(args):
    assert 1<=args.worker_cap<=8 and args.reserve_cpus>=0
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


def current_context(serial):
    assert sha(serial.BASE) == serial.BASE_SHA
    baseline = read(serial.BASE)
    original, counts = baseline['source_sha256'], baseline['test_case_counts_by_file']
    assert baseline['passed'] and baseline['reviewed'] and baseline['installed']
    assert len(original) == 1034 and len(counts) == 286 and sum(counts.values()) == 8216
    assert set(counts) == set(baseline['test_scope'])
    assert read(serial.BACKUP/'manifest.json')['source_sha256'] == original
    assert all(sha(serial.BACKUP/p) == h for p,h in original.items())
    compatibility, dependencies = serial.compatibility_inputs(original)
    new = sorted(str(p.relative_to(W)) for root in serial.ROOTS for p in (W/root).glob('test_gen43*.py'))
    assert set(new) == set(serial.NEW)
    scope = baseline['test_scope'] + list(serial.NEW)
    assert len(scope) == len(set(scope)) == TOTAL_FILES
    sources = {p:sha(W/p) for p in list(original) + new}
    assert len(sources) == SOURCE_COUNT
    assert {p for p in original if sources[p] != original[p]} == {serial.CHANGE,serial.COMPAT}
    assert all(sources[p] == original[p] for p in counts if p != serial.COMPAT)
    assert all(sources[p] == h for p,h in serial.EXPECTED_SOURCE.items())
    expected_counts = dict(counts, **serial.EXPECTED_NEW_COUNTS)
    assert len(expected_counts) == TOTAL_FILES and sum(expected_counts.values()) == TOTAL_CASES
    guard, audit = serial.bootstrap_sources()
    return dict(scope=scope, sources=sources, expected_counts=expected_counts,
        compatibility=compatibility, compatibility_dependencies=dependencies,
        original=original, baseline_sha256=serial.BASE_SHA,
        guard_sha256=hashlib.sha256(guard.encode()).hexdigest(),
        audit_sha256=hashlib.sha256(audit.encode()).hexdigest())


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


def make_plan(serial, output, scope_kind, resources):
    context = current_context(serial)
    assert sha(HISTORY) == HISTORY_SHA
    history = read(HISTORY); xml = HISTORY.parent/'pytest.xml'
    assert history['passed'] and history['scope_complete'] and history['source_unchanged']
    assert history['summary'] == dict(tests=TOTAL_CASES,failures=0,errors=0,skipped=0)
    assert history['source_sha256'] == context['sources'] and sha(xml) == history['xml_sha256']
    summary,cases,durations = inventory(xml)
    counts = Counter(row[0] for row in cases)
    assert dict(counts) == context['expected_counts'] and summary == history['summary']
    assert scope_kind in ('pilot','full')
    assert set(PILOT_FILES) <= set(context['scope']) and set(serial.NEW) <= set(PILOT_FILES)
    scope = [p for p in context['scope'] if scope_kind=='full' or p in PILOT_FILES]
    expected_counts = {p:context['expected_counts'][p] for p in scope}
    scope_cases = [c for c in cases if c[0] in expected_counts]
    assert len(scope_cases) == sum(expected_counts.values())
    worker_count=resources['selected_workers'];assert worker_count<=len(scope)
    # Every whole file is assigned once by historical LPT. Only shards owning a
    # changed-module contract test require that module to load. Every import in
    # every shard still has strict provenance, and the aggregate must load it.
    assigned = [[] for _ in range(worker_count)]
    loads = [0. for _ in range(worker_count)]
    rest = set(scope)
    for name in sorted(rest,key=lambda n:(-durations.get(n,0.),n)):
        index = min(range(worker_count),key=lambda i:(loads[i],i))
        assigned[index].append(name); loads[index] += durations.get(name,0.)
    shards = []
    for index, names in enumerate(assigned):
        chosen = set(names)
        shard_scope = [n for n in scope if n in chosen]
        shards.append(dict(index=index,scope=shard_scope,test_files=len(shard_scope),
            requires_changed_source=bool(set(shard_scope)&set(serial.NEW)),
            expected_count=sum(expected_counts[p] for p in shard_scope),
            historical_case_time_sec=loads[index],
            case_identities=[c for c in scope_cases if c[0] in chosen]))
    all_scope=[p for s in shards for p in s['scope']]
    assert len(all_scope)==len(set(all_scope)) and set(all_scope)==set(scope)
    assert sum(s['expected_count'] for s in shards) == len(scope_cases)
    return dict(schema=3,kind='draft_configurable_parallel_'+scope_kind+'_complete_file_offline_plan',
        worker_count=worker_count,resource_budget=resources,
        prior_verified_two_worker_runner=str(PRIOR_TWO_WORKER),prior_verified_two_worker_runner_sha256=PRIOR_TWO_WORKER_SHA,
        scope_kind=scope_kind,test_scope=scope,expected_file_count=len(scope),
        expected_case_count=len(scope_cases),expected_counts_by_file=expected_counts,
        full_regression_scope=(scope_kind=='full'),
        runner_path=str(Path(__file__).resolve()),runner_sha256=sha(__file__),
        frozen_serial_contract=str(SERIAL),frozen_serial_contract_sha256=SERIAL_SHA,
        historical_result=str(HISTORY),historical_result_sha256=HISTORY_SHA,
        historical_xml=str(xml),historical_xml_sha256=sha(xml),context=context,
        expected_case_identities=scope_cases,shards=shards,output=str(output),
        maximum_workers=resources['worker_cap'],historical_serial_elapsed_sec=history['elapsed_sec'],
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


def stable(serial, plan):
    return (sha(__file__) == plan['runner_sha256'] and sha(SERIAL) == SERIAL_SHA
        and sha(PRIOR_TWO_WORKER)==PRIOR_TWO_WORKER_SHA and sha(CALIBRATION)==CALIBRATION_SHA
        and sha(HISTORY) == HISTORY_SHA and sha(plan['historical_xml']) == plan['historical_xml_sha256']
        and current_context(serial) == plan['context'])


def worker(plan_path,index):
    serial = load_serial(); plan = read(plan_path); plan_sha = sha(plan_path)
    assert index in range(plan['worker_count']) and stable(serial,plan)
    assert os.getsid(0)==os.getpgrp()==os.getpid(), 'worker must own its coordinator-created session'
    assert Path(plan['output']).resolve() == plan_path.parent.resolve()
    shard = plan['shards'][index]; context = plan['context']
    output = plan_path.parent/('worker_%d'%index)
    serial.storage(output); assert not output.exists(); output.mkdir()
    for name in ('tmp','ros','ros_home','bootstrap'): (output/name).mkdir()
    guard,audit = serial.bootstrap_sources()
    (output/'bootstrap/sitecustomize.py').write_text(guard)
    (output/'bootstrap/gen42_import_audit.py').write_text(audit)
    generated = {str(p.relative_to(output)):sha(p) for p in (output/'bootstrap').iterdir()}
    env = dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1',
        PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',
        ROS_MASTER_URI='http://127.0.0.1:1',ROS_LOG_DIR=str(output/'ros'),
        ROS_HOME=str(output/'ros_home'),TMPDIR=str(output/'tmp'),
        OFFLINE_BOUNDARY_LOG=str(output/'boundary_attempts.jsonl'),
        GEN42_MODULE_PROVENANCE=str(output/'module_provenance.json'))
    for key in ('PYTEST_ADDOPTS','PYTEST_PLUGINS'): env.pop(key,None)
    paths = [output/'bootstrap']+[W/p for p in (*serial.ROOTS,
        'src/alicia_grasp_modes/src','src/alicia_flexible_grasp_supervisor/src')]
    paths += [W/'devel/lib/python3/dist-packages',Path('/opt/ros/noetic/lib/python3/dist-packages')]
    env['PYTHONPATH'] = ':'.join(map(str,paths))
    env['ROS_PACKAGE_PATH'] = str(W/'src')+':/opt/ros/noetic/share'
    command = [sys.executable,'-m','pytest','-q','--rootdir='+str(W),*shard['scope'],
        '-p','no:cacheprovider','-p','gen42_import_audit','--tb=short',
        '--basetemp='+str(output/'pytest_tmp'),'--junitxml='+str(output/'pytest.xml'),
        '-o','junit_family=legacy']
    result = dict(passed=False,worker_index=index,plan_sha256=plan_sha,command=command,
        scope_kind=plan['scope_kind'],full_regression_scope=plan['full_regression_scope'],
        requires_changed_source=shard['requires_changed_source'],
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
        for label,row in provenance.items():
            p=Path(row['path'])
            if serial.inside(p,W/'src'):
                relative=str(p.relative_to(W)); valid=relative in context['sources'] and row['sha256']==context['sources'][relative]
            elif serial.inside(p,W/'devel'): valid=sha(p)==row['sha256']
            else:
                # This is exactly the serial full-run oracle exception, bound
                # to its original test in the UNION scope, path and bytes. A
                # helper import in the other process gains no wider whitelist.
                valid=(label==serial.ORACLE_LABEL and p==serial.ORACLE and row['sha256']==serial.ORACLE_SHA
                    and serial.ORACLE_TEST in plan['test_scope'] and context['sources'][serial.ORACLE_TEST]==serial.ORACLE_TEST_SHA
                    and sha(serial.ORACLE)==serial.ORACLE_SHA)
                if valid: oracle_rows[label]=row
            if not valid: invalid[label]=row
        source_loaded=any(r['path']==str(W/serial.CHANGE) and r['sha256']==context['sources'][serial.CHANGE] for r in provenance.values())
        attempts_path=output/'boundary_attempts.jsonl'
        attempts=attempts_path.read_text().splitlines() if attempts_path.exists() else []
        expected=sorted(map(tuple,shard['case_identities']))
        actual=sorted(map(tuple,cases))
        counts=dict(Counter(c[0] for c in cases))
        expected_counts={p:plan['expected_counts_by_file'][p] for p in shard['scope']}
        unchanged=(stable(serial,plan) and sha(plan_path)==plan_sha
            and all(sha(output/p)==h for p,h in generated.items()))
        complete=(actual==expected and counts==expected_counts and summary['tests']==shard['expected_count'])
        result.update(summary=summary,case_identities=cases,test_case_counts_by_file=counts,
            scope_complete=complete,source_unchanged=unchanged,changed_source_loaded=source_loaded,
            foreign_or_inconsistent_imports=invalid,accepted_exact_historical_oracle=oracle_rows,
            external_boundary_attempts=attempts,generated_bootstrap_sha256=generated,
            module_provenance_sha256=sha(provenance_path),xml_sha256=sha(xml),log_sha256=sha(output/'pytest.log'))
        result['passed']=(returncode==0 and not result['interrupted'] and complete and unchanged
            and (source_loaded or not shard['requires_changed_source']) and not invalid and not attempts
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
            live_ROS_contacted=False,hardware_contacted=False,motion_commands_sent=False,temperature_queried=False)
        serial.storage(output);save(output/'result.json',result)
    return 0 if result['passed'] else 1


def execute(serial,output,plan):
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
        try:
            while True:
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
    results=[]; missing=[]; all_cases=[]; summary=dict.fromkeys(('tests','failures','errors','skipped'),0)
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
            assert row['requires_changed_source']==plan['shards'][index]['requires_changed_source']
            assert row['changed_source_loaded'] or not row['requires_changed_source']
            assert row['direct_child_reaped'] and not row['direct_child_cleanup_signals']
            assert row['scope_kind']==plan['scope_kind'] and row['full_regression_scope']==plan['full_regression_scope']
            assert not row['foreign_or_inconsistent_imports'] and not row['external_boundary_attempts']
            child_summary,child_cases,_=inventory(folder/'pytest.xml')
            assert child_summary==row['summary'] and child_cases==row['case_identities']
        if 'summary' in row:
            for key in summary:summary[key]+=row['summary'][key]
        all_cases+=row.get('case_identities',[])
        xml=folder/'pytest.xml'
        if row.get('xml_sha256') and sha(xml)==row['xml_sha256']:
            parsed=ET.parse(xml).getroot()
            for suite in parsed.iter('testsuite'):combined.append(deepcopy(suite))
    duplicates=[list(k) for k,n in Counter(map(tuple,all_cases)).items() if n>1]
    expected=set(map(tuple,plan['expected_case_identities']));actual=set(map(tuple,all_cases))
    counts=dict(Counter(c[0] for c in all_cases))
    complete=(not missing and not duplicates and expected==actual and counts==plan['expected_counts_by_file']
        and len(all_cases)==summary['tests']==plan['expected_case_count'])
    unchanged=stable(serial,plan) and sha(plan_path)==plan_sha
    globally_loaded=any(r.get('changed_source_loaded') for r in results)
    passed=(not interrupted and launcher_error is None and not cleanup_required and not residual_groups
        and len(processes)==len(results)==plan['worker_count'] and all(p.returncode==0 for p in processes)
        and all(r['passed'] for r in results) and complete and unchanged
        and globally_loaded
        and all(summary[k]==0 for k in ('failures','errors','skipped')))
    ET.ElementTree(combined).write(output/'pytest.xml',encoding='utf-8',xml_declaration=True)
    combined_summary,combined_cases,_=inventory(output/'pytest.xml')
    combined_matches=(combined_summary==summary and sorted(map(tuple,combined_cases))==sorted(map(tuple,all_cases)))
    passed=passed and combined_matches
    result=dict(passed=passed,summary=summary,scope_complete=complete,source_unchanged=unchanged,
        kind='draft_configurable_parallel_'+plan['scope_kind']+'_offline_result',scope_kind=plan['scope_kind'],
        full_regression_scope=plan['full_regression_scope'],
        full_regression_passed=(passed and plan['full_regression_scope']),
        elapsed_sec=time.monotonic()-started,worker_count=plan['worker_count'],resource_budget=plan['resource_budget'],
        launch_resource_budget=launch_budget,launch_resource_budget_sha256=sha(output/'launch_resource_budget.json'),
        changed_source_loaded_any_worker=globally_loaded,test_scope_files=plan['expected_file_count'],source_count=SOURCE_COUNT,
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
    p.add_argument('--scope-kind',choices=('pilot','full'),default='pilot',
        help='Fixed twenty complete files/1414 cases; full explicitly selects 288 files/8258 cases.')
    p.add_argument('--workers',default='auto',help='auto or an explicit positive count; plans record whether the resource budget permits execution.')
    p.add_argument('--worker-cap',type=int,default=8,help='Safety cap, at most8 for this helper.')
    p.add_argument('--reserve-cpus',type=int,default=4)
    p.add_argument('--memory-per-worker-mib',type=int,default=768)
    p.add_argument('--reserve-memory-mib',type=int,default=2048)
    p.add_argument('--execute',action='store_true',help='Explicitly execute the budgeted complete-file shards; default is plan only.')
    p.add_argument('--worker-plan',type=Path,help=argparse.SUPPRESS)
    p.add_argument('--worker-index',type=int,help=argparse.SUPPRESS)
    a=p.parse_args()
    if a.worker_plan:
        assert a.output is None and not a.execute and a.worker_index is not None
        return worker(a.worker_plan.resolve(strict=True),a.worker_index)
    assert a.worker_index is None and a.output is not None
    serial=load_serial();output=a.output.resolve();serial.storage(output);assert not output.exists()
    assert a.workers=='auto' or (a.workers.isdecimal() and int(a.workers)>0)
    assert sha(PRIOR_TWO_WORKER)==PRIOR_TWO_WORKER_SHA
    plan=make_plan(serial,output,a.scope_kind,resource_budget(a));output.mkdir();save(output/'plan.json',plan)
    if not a.execute:
        print(json.dumps(dict(plan_only=True,tests_executed=False,plan=str(output/'plan.json'),
            worker_count=plan['worker_count'],execution_resources_sufficient=plan['resource_budget']['execution_resources_sufficient'],
            plan_sha256=sha(output/'plan.json'),shards=[dict(index=s['index'],files=s['test_files'],cases=s['expected_count'],historical_case_time_sec=s['historical_case_time_sec']) for s in plan['shards']])),flush=True)
        return 0
    return execute(serial,output,plan)


if __name__=='__main__':
    raise SystemExit(main())
