"""Run the reviewed receipt writer and dispatch one unchanged guarded entry."""
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
S = N / 'gen57_resume_r3/full_restart_r1'
V = N / 'verification'
ENTRY = W / '.runtime/gen57_cold_attempt_entry_20261021.py'
WRITER = W / '.runtime/gen57_cold_attempt_receipt_20261021.py'
GATE = W / '.runtime/gen57_cold_attempt_gate_20261021.py'
ENTRY_SHA = 'b8c7d071c8a2a5a0b9868f920c59f9a340bf7c2104e32f3abac8d02b0441f91e'
WRITER_SHA = '1f073856820b5df2b224bc73e8fa43a121d7bd7143eb6ee4960ef8a1338febb3'
GATE_SHA = '46f3b87b1265f196ff62749a1f9ded6474e8bec7c1e75bb832192889fe22f001'

def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def save(p, value):
    with p.open('x') as f:
        json.dump(value, f, ensure_ascii=False, indent=2)
        f.write('\n'); f.flush(); os.fsync(f.fileno())

def main():
    assert len(sys.argv) == 2 and re.fullmatch(r'r[0-9]+', sys.argv[1])
    suffix = sys.argv[1]
    scene = S / ('scene_start_' + suffix)
    ready = S / ('readiness_start_' + suffix) / 'result.json'
    review = scene / 'current_image_review.json'
    assert sha(ENTRY) == ENTRY_SHA and sha(WRITER) == WRITER_SHA and sha(GATE) == GATE_SHA
    independent = V / 'gen57_cold_attempt_entry_independent_r1/result.json'
    assert sha(independent) == '861486598ce6df3b5fd86c6735e3de5c6a1deabfbb0779f836056173553077a3'
    base = json.loads((S / ('scene_dispatch_' + suffix) / 'argv.json').read_text())
    i = base.index('--output'); del base[i:i + 2]
    writer = list(base); writer[2] = str(WRITER)
    writer[writer.index('--helper-sha256') + 1] = WRITER_SHA
    for k, value in [('readiness', str(ready)), ('readiness-sha256', sha(ready)),
                     ('collector-sha256', 'ea3260069e9d356a80139f27d7feb4177824c5509a2ef0cb22537c8cf56b6e85'),
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
    unit = 'alicia-gen57-recorded-cold-' + suffix + '-20261021'
    command = ['systemd-run', '--user', '--unit=' + unit, '--property=Type=exec', '--property=Restart=no',
               '--property=WorkingDirectory=' + str(W), '--property=StandardOutput=append:' + str(d / 'console.log'),
               '--property=StandardError=append:' + str(d / 'console.log')]
    command += ['--setenv=' + k + '=' + value for k, value in env.items()] + argv
    save(d / 'dispatch_plan.json', dict(argv=argv, command=command, unit=unit, max_invocations=1,
          automatic_retry=False, dispatcher_sha256=sha(Path(__file__)), independent_review_sha256=sha(independent)))
    assert time.time_ns() - json.loads((scene / 'preflight.json').read_text())['receipt_ns'] < 90_000_000_000
    result = subprocess.run(command, cwd=W, capture_output=True, text=True, timeout=30)
    payload = dict(returncode=result.returncode, stdout=result.stdout, stderr=result.stderr,
                   recorded_ns=time.time_ns(), unit=unit)
    save(d / 'dispatch_result.json', payload)
    print(json.dumps(payload, ensure_ascii=False), flush=True)
    return result.returncode

if __name__ == '__main__':
    raise SystemExit(main())
