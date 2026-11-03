"""Run the reviewed receipt writer and dispatch one unchanged guarded entry."""
import argparse
from pathlib import Path
import hashlib
import json
import os
import re
import subprocess
import sys
import time

W = Path(__file__).resolve().parents[1]
N = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z')
S = N / 'gen58_resume_r1/full_restart_r1'
V = N / 'verification'
ENTRY = W / '.runtime/gen58_cold_attempt_entry_20261021.py'
WRITER = W / '.runtime/gen58_cold_attempt_receipt_20261021.py'
GATE = W / '.runtime/gen58_cold_attempt_gate_20261021.py'
COMMON = W / '.runtime/gen58_cold_attempt_common_20261021.py'
COLLECTOR = W / '.runtime/gen58_cold_attempt_capture_20261021.py'
ENTRY_SHA = '277358ee8ae129ecf6830d22b509f0efe54f30eb74ff65d225455b4e177e6c4e'
WRITER_SHA = '55ff111ef2b9cf1c77f42bd188e3dce41208297c29770d2c7e501310e8e7878f'
GATE_SHA = 'c94d1cfd6ac41bf2b914c80c478d51726c370620e00fe1f215af23930a2b7c78'

def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def save(p, value):
    with p.open('x') as f:
        json.dump(value, f, ensure_ascii=False, indent=2)
        f.write('\n'); f.flush(); os.fsync(f.fileno())

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('suffix')
    parser.add_argument('--independent-review',type=Path,required=True)
    parser.add_argument('--independent-review-sha256',required=True)
    args=parser.parse_args()
    assert re.fullmatch(r'r[0-9]+', args.suffix)
    suffix = args.suffix
    scene = S / ('scene_start_' + suffix)
    ready = S / ('readiness_start_' + suffix) / 'result.json'
    review = scene / 'current_image_review.json'
    assert sha(ENTRY) == ENTRY_SHA and sha(WRITER) == WRITER_SHA and sha(GATE) == GATE_SHA
    independent = args.independent_review.resolve(strict=True)
    assert N in independent.parents and sha(independent) == args.independent_review_sha256
    approved=json.loads(independent.read_text())
    assert approved.get('passed') is True and approved.get('cold_attempt_entry_reviewed') is True
    dependencies=approved['dependency_sha256']
    required=[ENTRY,WRITER,GATE,COMMON,COLLECTOR,Path(__file__).resolve(),
        W/'.runtime/run_reboot_realigned_grasp_20261010.py',
        W/'.runtime/continuous_grasp_authorization_20261012.py',
        W/'.runtime/prewarm_mode_grasp_runner_20261012.py',
        W/'tools/run_recorded_grasp.py',W/'tools/record_grasp_keyframes_readonly.py',W/'tools/grasp_archive.py',
        W/'.runtime/verify_active_remote_readiness_20261020.py']
    assert all(dependencies.get(str(path)) == sha(path) for path in required)
    assert all(sha(Path(path)) == digest for path,digest in dependencies.items())
    base = json.loads((S / ('scene_dispatch_' + suffix) / 'argv.json').read_text())
    assert base[:3] == [sys.executable,'-B',str(COLLECTOR)]
    i = base.index('--output'); del base[i:i + 2]
    writer = list(base); writer[2] = str(WRITER)
    writer[writer.index('--helper-sha256') + 1] = WRITER_SHA
    for k, value in [('readiness', str(ready)), ('readiness-sha256', sha(ready)),
                     ('collector-sha256', sha(COLLECTOR)),
                     ('review', str(review)), ('review-sha256', sha(review))]:
        writer.extend(['--' + k, value])
    d = S / ('attempt_dispatch_' + suffix); d.mkdir()
    save(d / 'receipt_argv.json', writer)
    with (d / 'receipt_console.log').open('x') as log:
        receipt = subprocess.run(writer, cwd=W, stdout=log, stderr=subprocess.STDOUT, timeout=30)
    save(d / 'receipt_dispatch.json', dict(returncode=receipt.returncode, recorded_ns=time.time_ns()))
    assert receipt.returncode == 0, (d / 'receipt_console.log').read_text()
    argv = list(base); argv[2] = str(ENTRY)
    argv[argv.index('--helper-sha256') + 1] = ENTRY_SHA
    for k, value in [('phase', 'launch'), ('observation-sha256', sha(scene / 'operator_observation.json')),
                     ('receipt-sha256', sha(scene / 'receipt_preparation.json')), ('gate-sha256', GATE_SHA)]:
        argv.extend(['--' + k, value])
    keys = ('PATH', 'PYTHONPATH', 'LD_LIBRARY_PATH', 'CMAKE_PREFIX_PATH', 'ROS_PACKAGE_PATH',
            'ROS_ETC_DIR', 'ROS_ROOT', 'ROS_DISTRO', 'ROS_VERSION', 'ROS_PYTHON_VERSION',
            'PKG_CONFIG_PATH', 'ROSLISP_PACKAGE_DIRECTORIES')
    env = {k: os.environ[k] for k in keys if k in os.environ}
    env.update(ROS_MASTER_URI='http://localhost:11311', PYTHONDONTWRITEBYTECODE='1',
               PYTHONNOUSERSITE='1', PYTHONUNBUFFERED='1', OPENBLAS_NUM_THREADS='1',
               OMP_NUM_THREADS='1', ROS_LOG_DIR=str(scene / 'recorded_attempt_ros'))
    unit = 'alicia-gen58-recorded-cold-' + suffix + '-20261021'
    command = ['systemd-run', '--user', '--unit=' + unit, '--property=Type=exec', '--property=Restart=no',
               '--property=WorkingDirectory=' + str(W), '--property=StandardOutput=append:' + str(d / 'console.log'),
               '--property=StandardError=append:' + str(d / 'console.log')]
    command += ['--setenv=' + k + '=' + value for k, value in env.items()] + argv
    save(d / 'dispatch_plan.json', dict(argv=argv, command=command, unit=unit, max_invocations=1,
          automatic_retry=False, dispatcher_sha256=sha(Path(__file__)), independent_review_sha256=sha(independent)))
    assert sha(independent) == args.independent_review_sha256
    assert all(sha(Path(path)) == digest for path,digest in dependencies.items())
    assert time.time_ns() - json.loads((scene / 'preflight.json').read_text())['receipt_ns'] < 90_000_000_000
    result = subprocess.run(command, cwd=W, capture_output=True, text=True, timeout=30)
    payload = dict(returncode=result.returncode, stdout=result.stdout, stderr=result.stderr,
                   recorded_ns=time.time_ns(), unit=unit)
    save(d / 'dispatch_result.json', payload)
    print(json.dumps(payload, ensure_ascii=False), flush=True)
    return result.returncode

if __name__ == '__main__':
    raise SystemExit(main())
