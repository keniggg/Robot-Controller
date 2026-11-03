"""Offline integrated regression, pinned to the installed GEN7 release inputs."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

w = Path(__file__).resolve().parents[1]
s = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_frozen_no_skin_20261012T021050Z')
assert Path('/media/zhuyupei/SanStand').is_mount()
out = s / 'verification/gen7_yield_integrated'
out.mkdir(exist_ok=False)
installed = json.loads((s / 'verification/gen7_yield_install/result.json').read_text())
expected = installed['source_sha256']
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
assert all(sha(w / p) == h for p, h in expected.items())
mode_names = (
    'contact_search_yield', 'contact_search_yield_remote', 'contact_search_yield_accept',
    'contact_search_yield_receipts', 'contact_search_yield_reached_gain',
    'gen16_recorded_contact_search_yield', 'gen17_wire_selection_yield',
    'gen7_contact_yield_completed_budget', 'observe_return_remote',
    'observe_return_supplement_remote', 'observe_return_supplement_task',
    'supplement_measurement_admission', 'supplement_measurement_preparation',
    'supplement_measurement_remote_admission', 'observe_return_measurement',
    'observe_return_transaction', 'observation_measurement_task_boundaries',
    'station_compute_wait_task', 'observation_measurement_remote',
    'observe_return_session_handoff_concurrency', 'station_planning_handoff',
    'observation_measurement_unknown', 'station_compute_wait_producer',
    'observation_arrival_station', 'launcher_selection_commit', 'runner_observe_return',
)
supervisor_names = (
    'contact_sequence_precheck', 'contact_height_window_proposals',
    'contact_height_retry_diagnostic', 'contact_height_margin', 'contact_height_audit_binding',
    'contact_pregrasp_compute_guard', 'contact_pregrasp_gateway',
    'observe_return_view_admission', 'observe_return_gateway', 'observe_return_handoff',
)
scope = ['src/alicia_grasp_modes/tests/test_' + n + '.py' for n in mode_names]
scope += ['src/alicia_flexible_grasp_supervisor/tests/test_' + n + '.py' for n in supervisor_names]
assert all((w / p).is_file() for p in scope)
env = dict(os.environ, PYTHONNOUSERSITE='1', PYTHONDONTWRITEBYTECODE='1',
           OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
           ROS_MASTER_URI='http://127.0.0.1:1', TMPDIR='/tmp',
           GEN10_TEST_LAUNCHER=str(w / '.runtime/run_reboot_realigned_grasp_20261010.py'))
env['PYTHONPATH'] = ':'.join(str(w / p) for p in (
    'src/alicia_flexible_grasp_supervisor/src', 'src/alicia_grasp_modes/src',
    'src/alicia_flexible_grasp_supervisor/tests', 'src/alicia_grasp_modes/tests')) + ':' + env.get('PYTHONPATH', '')
env['ROS_PACKAGE_PATH'] = str(w / 'src') + ':' + env.get('ROS_PACKAGE_PATH', '')
cmd = ['/usr/bin/python3', '-m', 'pytest', '-q', *scope, '-p', 'no:cacheprovider',
       '--junitxml=' + str(out / 'pytest.xml')]
start = time.monotonic()
with (out / 'pytest.log').open('x') as log:
    p = subprocess.run(cmd, cwd=w, env=env, stdout=log, stderr=subprocess.STDOUT)
after = {p: sha(w / p) for p in expected}
summary = {}
for suite in ET.parse(out / 'pytest.xml').getroot().iter('testsuite'):
    for key in ('tests', 'failures', 'errors', 'skipped'):
        summary[key] = summary.get(key, 0) + int(suite.attrib.get(key, 0))
passed = p.returncode == 0 and after == expected and summary['tests'] > 0 and not any(
    summary[k] for k in ('failures', 'errors', 'skipped'))
result = dict(passed=passed, returncode=p.returncode, source_sha256=expected,
              source_after_sha256=after, source_unchanged=after == expected,
              test_scope=scope, command=cmd, summary=summary,
              elapsed_sec=time.monotonic() - start, production_worktree_tested=True,
              motion_commands_sent=False, temperature_queried=False)
(out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps({k: result[k] for k in ('passed', 'summary', 'elapsed_sec')}), flush=True)
print('\n'.join((out / 'pytest.log').read_text().splitlines()[-16:]), flush=True)
raise SystemExit(0 if passed else 1)
