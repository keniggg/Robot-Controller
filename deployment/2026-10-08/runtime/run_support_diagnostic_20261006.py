"""Coordinate a single perception-only selection with exact-source capture."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
assert Path('/media/zhuyupei/SanStand').is_mount()
OUT = E/'ros_log/gen17_support_measurement_20261006'
OUT.mkdir(exist_ok=True)
CAPTURE = OUT/'raw_support_v2'
assert not CAPTURE.exists()
report = dict(scope='One diagnostic perception selection and exact RGB-D/status/TF capture; '
    'no inference request, candidate load, grasp runner, motion or torque command',
    started_wall_ns=time.time_ns(), motion_commands_sent=False, selection_started=False,
    source_sha256={str(p.relative_to(W)):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (Path(__file__).resolve(), W/'.runtime/record_support_gap_15s.py',
                  W/'.runtime/select_support_diagnostic_20261006.py')})
env = dict(os.environ, ROS_LOG_DIR=str(OUT/'ros'))
log = (OUT/'capture_v2.log').open('w')
process = subprocess.Popen([sys.executable, str(W/'.runtime/record_support_gap_15s.py'),
    '--output', str(CAPTURE), '--seconds', '15'], cwd=W, env=env,
    stdout=log, stderr=subprocess.STDOUT)
try:
    deadline = time.monotonic()+7
    while not (CAPTURE/'ready.json').is_file():
        if process.poll() is not None:
            raise RuntimeError('diagnostic recorder exited before readiness')
        if time.monotonic() >= deadline:
            raise RuntimeError('diagnostic recorder readiness timeout')
        time.sleep(.02)
    ready = json.loads((CAPTURE/'ready.json').read_text())
    assert ready['initial_generation'] == 16
    assert time.monotonic()+3 < ready['deadline_monotonic']
    report['capture_ready'] = ready
    report['selection_started'] = True
    report['selection_started_wall_ns'] = time.time_ns()
    with (OUT/'selection_v2.log').open('w') as selection_log:
        result = subprocess.run([sys.executable, str(W/'.runtime/select_support_diagnostic_20261006.py')],
            cwd=W, env=env, stdout=selection_log, stderr=subprocess.STDOUT, timeout=10)
    report['selection_finished_wall_ns'] = time.time_ns()
    report['selection_returncode'] = result.returncode
    assert result.returncode == 0, 'perception selection refused; see selection.log'
    process.wait(timeout=22)
    report['capture_returncode'] = process.returncode
    if (CAPTURE/'summary.json').is_file():
        report['capture_summary'] = json.loads((CAPTURE/'summary.json').read_text())
    assert process.returncode == 0, 'diagnostic capture failed; see capture.log'
except Exception as exc:
    report['error'] = repr(exc)
    raise
finally:
    if process.poll() is None:
        # Only this invocation's diagnostic child, never a robot node.
        process.terminate()
        report['diagnostic_child_terminated'] = True
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            report['diagnostic_child_exit_unconfirmed'] = True
    log.close()
    report['finished_wall_ns'] = time.time_ns()
    report['source_unchanged'] = all(hashlib.sha256((W/p).read_bytes()).hexdigest()==h
                                   for p,h in report['source_sha256'].items())
    (OUT/'run_v2.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('capture_summary','capture_ready')},
                     ensure_ascii=False), flush=True)
