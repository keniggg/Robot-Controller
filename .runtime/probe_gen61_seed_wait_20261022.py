"""Finite development pilot, reusing exact pinned GEN57 guard and worker.
Separate closed snapshot; only two new test files; never a full-regression claim.
"""
import argparse
from collections import Counter
from copy import deepcopy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace
W=Path(__file__).resolve().parents[1]
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z')
E=N/'gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1'
C=W/'.runtime/gen61_seed_wait_candidate_r1'
TRACE_PLUGIN=W/'.runtime/gen61_seed_wait_diagnostic_trace_20261022.py'
PARENT_MANIFEST=E/'pilot_r2/manifest.json'
PARENT_MANIFEST_SHA='f17efd455b547c8da60a5708573c5c8726bfed3cd86e54d93c675d9ad52a63ce'
CASE='test_second_failed_frame_and_ready_cannot_erase_first_unjoined_false_input'
OLD=W/'.runtime/verify_gen57_parallel_offline_r2_20261020.py'
OLD_SHA='0e217bc784b24c603afa1f4bca7c977f35f9036c03b6b263c9801f749d1c2727'
RELEASE=N/'verification/gen60_release_manifest.json'
RELEASE_SHA='bad5d6c82cdb8ba956f84e725e7b584219a37a8f67e6da3148dfdd5b24538217'
BM=E.parent/'verification/freeze_r2/manifest_closed.json'
BM_SHA='af790d4793275bb771f450f6a28332349cb6bf01acbb0c3d25ac7d43062e1312'
BP=E.parent/'verification/full_r2/plan.json'
BP_SHA='6318e911fe734281f024eb85a59d569f0a17d126801f0153f132bd9634b911b5'
P='src/alicia_grasp_modes/'
PROD=[P+'src/alicia_grasp_modes/'+v+'.py' for v in ('measured_mask_seeds','source_connected_owner','station_compute_reacquisition','station_compute_wait_producer','station_compute_wait_remote','station_compute_wait_task','arrival_fragment_group')]+[P+'scripts/unknown_tabletop_perception.py']
TESTS=[P+'tests/test_'+v+'.py' for v in ('gen61_seed_insufficiency','gen61_station_seed_wait')]
NEW=TESTS+[P+'tests/fixtures/gen60_raw_only_seed_insufficient.'+s for s in ('npz','json')]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_bytes())
def save(p,v):
    with Path(p).open('x') as f:
        json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())
assert sha(OLD)==OLD_SHA
spec=importlib.util.spec_from_file_location('gen61_pilot_immutable_worker',OLD)
O=importlib.util.module_from_spec(spec);spec.loader.exec_module(O)
original_load=O.load_serial

def bind(manifest_path):
    m=read(manifest_path);root=Path(m['snapshot']);out=Path(m['output'])
    assert m['runner_sha256']==sha(__file__) and m['infrastructure_sha256']==OLD_SHA
    assert root.parent==W/'.runtime' and root.name.startswith('gen61_seed_wait_pilot_')
    assert out.parent==E and out.name.startswith('pilot_')
    def serial_load():
        serial=original_load();serial.V=out;return serial
    def stable(serial,plan):
        return (sha(__file__)==plan['runner_sha256']==m['runner_sha256']
            and sha(OLD)==OLD_SHA and sha(manifest_path)==plan['candidate_manifest_sha256']
            and all(sha(p)==h for p,h in m['dependencies'].items())
            and all(sha(root/p)==h for p,h in m['copied_input_sha256'].items())
            and all(sha(W/p)==h for p,h in m['live_source_sha256'].items())
            and all(sha(p)==h for p,h in m['context']['generated_imports'].items())
            and plan['context']==m['context'])
    O.W=root;O.PROJECT=W;O.load_serial=serial_load;O.stable=stable
    return m

def make_snapshot(revision):
    root=W/('.runtime/gen61_seed_wait_pilot_'+revision);out=E/('pilot_'+revision)
    assert not root.exists() and not out.exists()
    assert sha(RELEASE)==RELEASE_SHA and sha(BM)==BM_SHA and sha(BP)==BP_SHA
    release=read(RELEASE);old=read(BM)['copied_input_sha256'];original=release['source_sha256']
    assert len(original)==1097 and len(old)==1109 and release['passed'] and release['installed']
    assert all(sha(W/p)==h for p,h in original.items())
    actual={str(p.relative_to(C)):sha(p) for p in C.rglob('*') if p.is_file()}
    assert sha(PARENT_MANIFEST)==PARENT_MANIFEST_SHA
    assert set(actual)==set(old)|set(NEW)
    assert sorted(p for p in old if actual[p]!=old[p])==sorted(PROD)
    context=deepcopy(read(BP)['context']);sources={p:actual[p] for p in list(original)+NEW}
    context.update(sources=sources,required_changed_sources=PROD,scope=TESTS)
    assert len(sources)==1101 and len(actual)==1113
    budget=O.resource_budget(SimpleNamespace(worker_cap=1,reserve_cpus=5,memory_per_worker_mib=768,reserve_memory_mib=2048,workers='1'))
    assert budget['execution_resources_sufficient']
    serial=original_load();serial.V=E;serial.storage(out)
    out.mkdir();root.mkdir()
    for p,h in sorted(actual.items()):
        dst=root/p;dst.parent.mkdir(parents=True,exist_ok=True)
        data=(C/p).read_bytes();assert hashlib.sha256(data).hexdigest()==h
        with dst.open('xb') as f:f.write(data)
    assert actual=={str(p.relative_to(C)):sha(p) for p in C.rglob('*') if p.is_file()}
    assert all(sha(root/p)==h for p,h in actual.items())
    m=dict(kind='GEN61_development_pilot_snapshot',snapshot=str(root),output=str(out),candidate=str(C),
        runner_sha256=sha(__file__),infrastructure_path=str(OLD),infrastructure_sha256=OLD_SHA,
        dependencies={str(RELEASE):RELEASE_SHA,str(BM):BM_SHA,str(BP):BP_SHA,str(PARENT_MANIFEST):PARENT_MANIFEST_SHA,str(TRACE_PLUGIN):sha(TRACE_PLUGIN)},copied_input_sha256=actual,
        live_source_sha256=original,context=context,resource_budget=budget,candidate_stable_while_copied=True,
        old_tests_unchanged=True,full_regression_passed=False,active_source_modified=False,production_deployed=False,created_ns=time.time_ns())
    save(out/'manifest.json',m);return out/'manifest.json'

def phase(manifest_path,mode,cases):
    m=bind(manifest_path);output=Path(m['output'])/mode;output.mkdir()
    scope=[TESTS[1]]
    plan=dict(output=str(output),mode=mode,worker_count=1,context=m['context'],runner_sha256=sha(__file__),
        candidate_manifest_sha256=sha(manifest_path),candidate_manifest_path=str(manifest_path),test_scope=scope,
        scope_kind='development_pilot',full_regression_scope=False,expected_counts_by_file=dict(Counter(c[0] for c in cases)),
        shards=[dict(scope=scope,case_identities=cases,expected_count=len(cases),required_changed_sources=PROD)])
    plan_path=output/'plan.json';save(plan_path,plan)
    monitor=O.WorkloadMonitor(output);started=time.monotonic();owned=[]
    with (output/'coordinator_child.log').open('x') as log:
        proc=subprocess.Popen([sys.executable,'-B',str(Path(__file__).resolve()),'--worker-plan',str(plan_path)],
            cwd=W,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1'),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        identity=O.process_stat(proc.pid)
        try:
            while proc.poll() is None:
                monitor.sample([proc]);time.sleep(.2)
            returncode=proc.wait()
        except BaseException:
            if proc.poll() is None:
                proc.terminate()
                try:proc.wait(timeout=5)
                except subprocess.TimeoutExpired:proc.kill();proc.wait()
            raise
        finally:
            monitor.sample([proc],force=True);monitoring=monitor.close()
    for entry in Path('/proc').iterdir():
        if entry.name.isdecimal():
            row=O.process_stat(entry.name)
            if row and row['pgid']==proc.pid:owned.append(row)
    rp=output/'worker_0/result.json';result=read(rp)
    receipt=dict(returncode=returncode,worker_identity=identity,worker_result_path=str(rp),worker_result_sha256=sha(rp),
        plan_sha256=sha(plan_path),monitoring=monitoring,elapsed_sec=time.monotonic()-started,residual_owned_processes=owned,
        passed=returncode==0 and result['passed'] and not owned,full_regression_passed=False,live_ROS_contacted=False,motion_commands_sent=False)
    save(output/'result.json',receipt)
    print(json.dumps(dict(phase=mode,passed=receipt['passed'],summary=result.get('summary'),output=str(output))),flush=True)
    return receipt,result

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--revision',choices=['r3_single']);ap.add_argument('--worker-plan',type=Path);args=ap.parse_args()
    if args.worker_plan:
        plan=read(args.worker_plan);m=bind(Path(plan['candidate_manifest_path']))
        original_popen=subprocess.Popen
        def diagnostic_popen(command, *a, **kw):
            assert command[1:3]==['-m','pytest']
            bootstrap=Path(plan['output'])/'worker_0/bootstrap'
            plugin=bootstrap/'gen61_diagnostic_trace.py'
            data=TRACE_PLUGIN.read_bytes()
            assert hashlib.sha256(data).hexdigest()==m['dependencies'][str(TRACE_PLUGIN)]
            with plugin.open('xb') as f:f.write(data)
            command.extend(['-k',CASE,'-p','gen61_diagnostic_trace'])
            kw['env']['GEN61_DIAGNOSTIC_TRACE']=str(Path(plan['output'])/'worker_0/diagnostic_trace.json')
            return original_popen(command,*a,**kw)
        subprocess.Popen=diagnostic_popen
        try:return O.worker(args.worker_plan,0)
        finally:subprocess.Popen=original_popen
    assert args.revision
    manifest=make_snapshot(args.revision);start=time.monotonic()
    rows=read(E/'pilot_r2/collect/worker_0/collected_items.json')
    cases=[row['case_identity'] for row in rows if row['nodeid']==TESTS[1]+'::'+CASE]
    assert len(cases)==1
    run,result=phase(manifest,'diagnostic',cases)
    trace_path=manifest.parent/'diagnostic/worker_0/diagnostic_trace.json'
    trace=read(trace_path)
    reasons=[row for row in trace if row['function']=='_station_compute_retire' and row['event']=='call']
    final=dict(passed=run['passed'],mode='development_pilot_single_case',full_regression_passed=False,manifest_path=str(manifest),
        manifest_sha256=sha(manifest),runner_sha256=sha(__file__),run=run,summary=result.get('summary'),
        elapsed_sec=time.monotonic()-start,source_unchanged=result.get('source_unchanged'),scope_complete=result.get('scope_complete'),
        diagnostic_trace_path=str(trace_path),diagnostic_trace_sha256=sha(trace_path),retire_reasons=reasons,
        diagnostic_plugin_sha256=sha(TRACE_PLUGIN),unchanged_test_assertions=True)
    save(manifest.parent/'result.json',final)
    print(json.dumps(dict(summary=final['summary'],reasons=[r['reason'] for r in reasons],trace_path=str(trace_path))))
    return 0 if result.get('scope_complete') and result.get('source_unchanged') and result.get('summary')==dict(tests=1,failures=0,errors=0,skipped=0) and not reasons and run['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
