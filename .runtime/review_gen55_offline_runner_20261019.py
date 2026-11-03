"""Bounded offline-only runner review: blocked boundaries and owned-group cleanup."""
import argparse
import ast
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

W=Path(__file__).resolve().parents[1]
RUNNER=W/'.runtime/verify_gen55_parallel_offline_20261019.py'
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path,value):
    with Path(path).open('x') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write('\n');stream.flush();os.fsync(stream.fileno())


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    spec=importlib.util.spec_from_file_location('gen55_runner_review',RUNNER)
    runner=importlib.util.module_from_spec(spec);spec.loader.exec_module(runner)
    output=args.output.resolve();serial=runner.load_serial();serial.storage(output)
    assert not output.exists();output.mkdir()
    sources={str(W/'.runtime'/name):sha(W/'.runtime'/name) for name in
        ('verify_gen55_parallel_offline_20261019.py','start_gen55_offline_unit_20261019.py',
         'prepare_gen55_offline_manifest_20261019.py','review_gen55_offline_runner_20261019.py')}
    for path in sources:ast.parse(Path(path).read_text())
    guard,audit=runner.bootstrap_sources(serial)
    assert hashlib.sha256(guard.encode()).hexdigest()=='5d00e04e91cc09ed375c6dcf7ce728eaee4b0c3cfa9190f4a0ab7c17f7f9e21e'
    assert hashlib.sha256(audit.encode()).hexdigest()=='87d31529b92cf02da30b2c9b19460a3d79917ae9d2d2208bbe5a6eee0f023ac7'
    for name in ('bootstrap','tmp','pycache','cache'): (output/name).mkdir()
    (output/'bootstrap/sitecustomize.py').write_text(guard)
    # Assert the guard is already active before two deliberately blocked
    # probes. The device name is fictitious, and no actual socket is opened.
    probe="""import builtins, json, os, socket, sys
from pathlib import Path
guard=sys.modules['sitecustomize']
assert socket.socket.connect is guard.forbidden
outcomes=[]
for label, operation in [('socket_connect',lambda:socket.socket.connect(None,('127.0.0.1',1))),
                         ('fictional_device_open',lambda:builtins.open('/dev/gen55_offline_nonexistent_probe'))]:
    try:operation()
    except guard.OfflineExternalBoundaryViolation:outcomes.append(label)
    else:raise AssertionError('boundary unexpectedly permitted')
Path(os.environ['GEN55_GUARD_PROBE_RESULT']).write_text(json.dumps(outcomes))
"""
    (output/'guard_probe.py').write_text(probe)
    env=dict(os.environ,PYTHONPATH=str(output/'bootstrap'),PYTHONDONTWRITEBYTECODE='1',
        PYTHONPYCACHEPREFIX=str(output/'pycache'),PYTHONNOUSERSITE='1',TMPDIR=str(output/'tmp'),
        XDG_CACHE_HOME=str(output/'cache'),OFFLINE_BOUNDARY_LOG=str(output/'expected_blocked_probes.jsonl'),
        GEN55_GUARD_PROBE_RESULT=str(output/'guard_probe_result.json'))
    with (output/'guard_probe.log').open('x') as log:
        probe_run=subprocess.run([sys.executable,'-B',str(output/'guard_probe.py')],cwd=output,env=env,
            stdout=log,stderr=subprocess.STDOUT,timeout=10)
    assert probe_run.returncode==0
    outcomes=json.loads((output/'guard_probe_result.json').read_bytes())
    assert outcomes==['socket_connect','fictional_device_open']
    assert len((output/'expected_blocked_probes.jsonl').read_text().splitlines())==2
    monitor=runner.WorkloadMonitor(output);signals=[]
    with (output/'owned_cleanup_probe.log').open('x') as log:
        process=subprocess.Popen([sys.executable,'-B','-c','import time;time.sleep(30)'],cwd=output,env=env,
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            identity=runner.process_stat(process.pid)
            assert identity['pid']==identity['pgid']==identity['session']==process.pid
            monitor.sample([process],force=True)
            residual=runner.cleanup_group(process.pid,signals,process)
            assert not residual and process.wait(timeout=2)<0
            assert signals and all(row['pgid']==process.pid for row in signals)
            assert not runner.group_exists(process.pid)
        finally:
            if process.poll() is None:runner.cleanup_group(process.pid,signals,process)
    metrics=monitor.close()
    fk_result=V/'gen55_held_pregrasp_reuse_r1/result.json'
    assert sha(fk_result)=='72be42134825860b4748057ddfcf095534284d3b4c862b6a36b25b068a73145f'
    fk=json.loads(fk_result.read_bytes())
    assert fk['passed'] and all(sha(path)==digest for path,digest in fk['dependencies'].items())
    assert all(sha(path)==digest for path,digest in sources.items())
    files={str(path.relative_to(output)):sha(path) for path in output.rglob('*') if path.is_file()}
    save(output/'result.json',dict(passed=True,scope='runner_isolation_and_exact_offline_child_cleanup',
        source_sha256=sources,files=files,expected_blocked_boundary_probes=outcomes,
        probe_returncode=probe_run.returncode,owned_cleanup_probe_identity=identity,
        cleanup_signals=signals,residual_offline_process_groups=[],sampled_workload=metrics,
        original_guard_and_import_audit_bytes_preserved=True,
        fk_seal_sha256=sha(fk_result),fk_dependencies_verified=len(fk['dependencies']),
        full_regression=False,candidate_test_bodies_executed=False,production_modified=False,
        actual_network_calls=0,actual_physical_device_opens=0,ROS_initialized=False,temperature_queried=False))
    print(json.dumps(dict(passed=True,result=str(output/'result.json'),result_sha256=sha(output/'result.json'))))


if __name__=='__main__':main()
