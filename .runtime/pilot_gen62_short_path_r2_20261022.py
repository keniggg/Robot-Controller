"""Finite GEN62 pilot using the unchanged pinned GEN57 guard and worker.

Copies a closed, independently hashed snapshot before collection or tests.
Never executes tests from the author's partial candidate directory.
"""
import argparse
from collections import Counter
from copy import deepcopy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time
from types import SimpleNamespace

W = Path(__file__).resolve().parents[1]
N = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z')
E = N/'gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1'
V = E/'v62'
C = W/'.runtime/gen62_contact_feedback_candidate_r1'
B = W/'.runtime/gen61_seed_wait_candidate_r1'
INVENTORY = E/'contact_feedback_verification_r1/baseline_inventory.json'
INVENTORY_SHA = 'dcd88a7a0fb3b2a52c710d37c9e16e547a5020b232423242696ef31997281182'
MANIFEST = E/'verification/freeze_r1/manifest_closed.json'
MANIFEST_SHA = 'df01964c58063e6ee52b3d12c986fde49e57b4a7f0cf7c593596a2d453a68b02'
OLD = W/'.runtime/verify_gen57_parallel_offline_r2_20261020.py'
OLD_SHA = '0e217bc784b24c603afa1f4bca7c977f35f9036c03b6b263c9801f749d1c2727'
P = 'src/alicia_grasp_modes/'
PROD = [P+'src/alicia_grasp_modes/contact_station_task.py']
TESTS = ['src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py']
NEW = [P+'tests/test_gen62_contact_feedback.py', P+'tests/fixtures/gen62_contact_feedback_recorded.json']
CASE = 'test_strict_mujoco_gate_complete_pass_reaches_first_physical_action_once'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes())


def save(path, value):
    with Path(path).open('x') as target:
        json.dump(value, target, ensure_ascii=False, indent=2, allow_nan=False)
        target.write('\n')
        target.flush()
        os.fsync(target.fileno())


assert sha(OLD) == OLD_SHA
spec = importlib.util.spec_from_file_location('gen62_pilot_immutable_worker', OLD)
O = importlib.util.module_from_spec(spec)
spec.loader.exec_module(O)
original_load = O.load_serial


def bind(path):
    manifest = read(path)
    root, output = Path(manifest['snapshot']), Path(manifest['output'])
    assert manifest['runner_sha256'] == sha(__file__)
    assert manifest['infrastructure_sha256'] == OLD_SHA
    assert root.parent == W/'.runtime' and root.name.startswith('gen62_contact_feedback_pilot_')
    assert output.parent == V and output.name.startswith('pilot_')
    def serial_load():
        serial = original_load()
        serial.V = output
        return serial
    def stable(serial, plan):
        return (sha(__file__) == plan['runner_sha256'] == manifest['runner_sha256']
            and sha(OLD) == OLD_SHA and sha(path) == plan['candidate_manifest_sha256']
            and all(sha(p) == h for p, h in manifest['dependencies'].items())
            and all(sha(root/p) == h for p, h in manifest['copied_input_sha256'].items())
            and all(sha(W/p) == h for p, h in manifest['live_source_sha256'].items())
            and all(sha(p) == h for p, h in manifest['context']['generated_imports'].items())
            and plan['context'] == manifest['context'])
    O.W, O.PROJECT, O.load_serial, O.stable = root, W, serial_load, stable
    return manifest


def snapshot(revision, baseline):
    assert re.fullmatch(r'r[1-9][0-9]*_[a-z0-9_]+', revision)
    root = W/('.runtime/gen62_contact_feedback_pilot_'+revision)
    output = V/('pilot_'+revision)
    assert not root.exists() and not output.exists()
    assert sha(INVENTORY) == INVENTORY_SHA and sha(MANIFEST) == MANIFEST_SHA
    inventory = read(INVENTORY)
    old = inventory['copied_input_sha256']
    live = inventory['baseline_source_sha256']
    assert len(old) == 1113 and len(live) == 1101
    assert inventory['baseline_case_count'] == 10149
    assert all(sha(B/p) == h for p, h in old.items())
    assert all(sha(W/p) == h for p, h in live.items())
    partial = {str(p.relative_to(C)): sha(p) for p in C.rglob('*') if p.is_file()}
    assert set(partial) == set(old)|set(NEW), sorted(set(partial)^set(old)^set(NEW))
    overlay = {p: h for p, h in partial.items() if p in NEW or not baseline}
    actual = dict(old, **overlay)
    changed = sorted(p for p in old if old[p] != actual[p])
    assert changed == ([] if baseline else sorted(PROD)), changed
    context = deepcopy(read(MANIFEST)['context'])
    sources = {p: actual[p] for p in list(live)+NEW}
    context.update(sources=sources, required_changed_sources=PROD, scope=TESTS)
    budget = O.resource_budget(SimpleNamespace(worker_cap=1, reserve_cpus=5,
        memory_per_worker_mib=768, reserve_memory_mib=2048, workers='1'))
    assert budget['execution_resources_sufficient']
    serial = original_load()
    serial.V = V
    serial.storage(output)
    output.mkdir()
    root.mkdir()
    for name, digest in sorted(actual.items()):
        source = (C if name in overlay else B)/name
        raw = source.read_bytes()
        assert hashlib.sha256(raw).hexdigest() == digest
        target = root/name
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open('xb') as stream:
            stream.write(raw)
    assert partial == {str(p.relative_to(C)): sha(p) for p in C.rglob('*') if p.is_file()}
    assert all(sha(root/p) == h for p, h in actual.items())
    dependencies = dict(inventory['pins'])
    dependencies.update({str(INVENTORY): INVENTORY_SHA, str(MANIFEST): MANIFEST_SHA})
    manifest = dict(kind='GEN62_development_pilot_frozen_snapshot', snapshot=str(root),
        output=str(output), candidate=str(C), baseline_production=baseline,
        runner_sha256=sha(__file__), infrastructure_path=str(OLD), infrastructure_sha256=OLD_SHA,
        dependencies=dependencies, candidate_sha256=partial, copied_input_sha256=actual,
        live_source_sha256=live, context=context, resource_budget=budget,
        baseline_case_count=10149, historical_tests_and_fixtures_unchanged=True,
        candidate_stable_while_copied=True, full_regression_passed=False,
        active_source_modified=False, production_deployed=False, created_ns=time.time_ns())
    save(output/'manifest.json', manifest)
    return output/'manifest.json'


def phase(manifest_path, mode, cases):
    manifest = bind(manifest_path)
    output = Path(manifest['output'])/mode
    output.mkdir()
    plan = dict(output=str(output), mode=mode, worker_count=1, context=manifest['context'],
        runner_sha256=sha(__file__), candidate_manifest_sha256=sha(manifest_path),
        candidate_manifest_path=str(manifest_path), test_scope=TESTS,
        scope_kind='development_pilot', full_regression_scope=False,
        expected_counts_by_file=dict(Counter(case[0] for case in cases)),
        shards=[dict(scope=TESTS, case_identities=cases, expected_count=len(cases),
                     required_changed_sources=[])])
    plan_path = output/'plan.json'
    save(plan_path, plan)
    monitor = O.WorkloadMonitor(output)
    started = time.monotonic()
    with (output/'coordinator_child.log').open('x') as log:
        proc = subprocess.Popen([sys.executable, '-B', str(Path(__file__).resolve()),
            '--worker-plan', str(plan_path)], cwd=W,
            env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1'),
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        identity = O.process_stat(proc.pid)
        try:
            while proc.poll() is None:
                monitor.sample([proc])
                time.sleep(.2)
            returncode = proc.wait()
        except BaseException:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
            raise
        finally:
            monitor.sample([proc], force=True)
            monitoring = monitor.close()
    residual = []
    for entry in Path('/proc').iterdir():
        if entry.name.isdecimal():
            row = O.process_stat(entry.name)
            if row and row['pgid'] == proc.pid:
                residual.append(row)
    result_path = output/'worker_0/result.json'
    result = read(result_path)
    receipt = dict(returncode=returncode, worker_identity=identity,
        worker_result_path=str(result_path), worker_result_sha256=sha(result_path),
        plan_sha256=sha(plan_path), monitoring=monitoring, elapsed_sec=time.monotonic()-started,
        residual_owned_processes=residual,
        passed=returncode == 0 and result['passed'] and not residual,
        full_regression_passed=False, live_ROS_contacted=False, motion_commands_sent=False)
    save(output/'result.json', receipt)
    print(json.dumps(dict(phase=mode, passed=receipt['passed'],
                          summary=result.get('summary'), output=str(output))), flush=True)
    return receipt, result


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--revision')
    parser.add_argument('--worker-plan',type=Path)
    args=parser.parse_args()
    if args.worker_plan:
        plan=read(args.worker_plan);bind(Path(plan['candidate_manifest_path']))
        original_popen=subprocess.Popen
        def selected(command,*a,**kw):
            assert command[1:3]==['-m','pytest']
            command.extend(['-k',CASE])
            return original_popen(command,*a,**kw)
        subprocess.Popen=selected
        try:return O.worker(args.worker_plan,0)
        finally:subprocess.Popen=original_popen
    assert args.revision
    V.mkdir(exist_ok=True)
    manifest=snapshot(args.revision,False)
    cases=[c for c in read(INVENTORY)['case_identities'] if c[0]==TESTS[0] and c[-1]==CASE]
    assert len(cases)==1
    run,result=phase(manifest,'case',cases)
    final=dict(passed=run['passed'],mode='historical_exact_case_path_probe',full_regression_passed=False,
        manifest_path=str(manifest),manifest_sha256=sha(manifest),runner_sha256=sha(__file__),
        summary=result.get('summary'),source_unchanged=result.get('source_unchanged'),
        scope_complete=result.get('scope_complete'),case_identities=cases,selected_case_count=1,
        new_cases_run=0,run=run,original_test_source_unchanged=True)
    save(manifest.parent/'result.json',final)
    return 0 if final['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
