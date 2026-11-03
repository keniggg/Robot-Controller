"""Independent GEN29 callbacks, with optional exact saved GEN28 module overlay."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

work = Path(__file__).resolve().parents[1]
mount = Path('/media/zhuyupei/SanStand')
root = mount / '实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z/verification'
assert len(sys.argv) in (2, 3)
mode = sys.argv[2] if len(sys.argv) == 3 else '--all'
assert mode in ('--all', '--baseline', '--remote', '--counterfactual')
baseline = mode == '--baseline'
counterfactual = mode == '--counterfactual'
out = root / sys.argv[1]
assert out.parent == root and not out.exists()
assert mount.is_mount() and not os.statvfs(mount).f_flag & os.ST_RDONLY
task_test = 'src/alicia_grasp_modes/tests/test_gen29_supplemental_retention_independent.py'
remote_test = 'src/alicia_grasp_modes/tests/test_gen29_supplemental_receipt_clock_independent.py'
tests = [task_test] if baseline else [remote_test] if mode in ('--remote', '--counterfactual') else [task_test, remote_test]
paths = [task_test, remote_test, '.runtime/run_gen29_supplemental_independent.py',
    'src/alicia_grasp_modes/tests/test_station_compute_wait_task.py',
    'src/alicia_grasp_modes/tests/test_task_modes.py',
    'src/alicia_grasp_modes/tests/test_observe_return_task.py',
    'src/alicia_grasp_modes/scripts/mode_aware_grasp_task.py',
    'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py']
paths += ['src/alicia_grasp_modes/tests/' + name + '.py' for name in (
    'test_observation_measurement_remote', 'test_station_compute_wait_remote',
    'test_observation_station_snapshot_scope', 'test_station_compute_work_handoff',
    'test_gen11_contact_remote_receipt', 'test_remote_modes', 'test_unknown_pending_identity')]
production = ['observe_return_task', 'station_compute_wait_task',
              'station_compute_wait_remote', 'station_supplemental_admission']
paths += ['src/alicia_grasp_modes/src/alicia_grasp_modes/' + name + '.py' for name in production]
manifest_path = work / '.runtime/gen29_baseline_source/manifest.json'
manifest = json.loads(manifest_path.read_text())
if baseline:
    paths += ['.runtime/gen29_baseline_source/manifest.json']
    for path, expected in manifest.items():
        saved = '.runtime/gen29_baseline_source/' + path
        assert hashlib.sha256((work / saved).read_bytes()).hexdigest() == expected
        paths.append(saved)
def hashes():
    return {path: hashlib.sha256((work / path).read_bytes()).hexdigest() for path in sorted(paths)}
before = hashes()
out.mkdir()
for name in ('tmp', 'ros'): (out / name).mkdir()
env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1',
    PYTEST_DISABLE_PLUGIN_AUTOLOAD='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
    ROS_MASTER_URI='http://127.0.0.1:1', TMPDIR=str(out / 'tmp'), ROS_LOG_DIR=str(out / 'ros'))
env['PYTHONPATH'] = ':'.join(str(work / path) for path in (
    'src/alicia_flexible_grasp_supervisor/tests', 'src/alicia_grasp_modes/tests',
    'src/alicia_grasp_modes/src', 'src/alicia_flexible_grasp_supervisor/src',
    'devel/lib/python3/dist-packages')) + ':' + env.get('PYTHONPATH', '')
env['ROS_PACKAGE_PATH'] = str(work / 'src') + ':' + env.get('ROS_PACKAGE_PATH', '')
arguments = ['-q', *tests, '-p', 'no:cacheprovider', '--tb=short',
             '--basetemp=' + str(out / 'pytest_tmp'), '--junitxml=' + str(out / 'pytest.xml')]
bootstrap = '''import importlib.util, sys
from pathlib import Path
import alicia_grasp_modes as package
work = Path.cwd()
for short in ('observe_return_task', 'station_compute_wait_task'):
    name = 'alicia_grasp_modes.' + short
    path = work / '.runtime/gen29_baseline_source/src/alicia_grasp_modes/src/alicia_grasp_modes' / (short + '.py')
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    setattr(package, short, module)
import pytest
raise SystemExit(pytest.main(sys.argv[1:]))
'''
counterfactual_bootstrap = '''import sys
from alicia_grasp_modes import station_compute_wait_remote as mod
def straightline(self, plan, ticket):
    with self._stream_condition:
        proof = self._station_compute_work[int(ticket.request_id)]
        reference = self._station_compute_reference
        self._station_compute_context(reference['contract'])
        return mod.build_supplement_receipt(plan,
            ticket=dict(request_id=int(ticket.request_id), generation=int(ticket.generation),
                        target_epoch=int(ticket.target_epoch)),
            source_witnesses=proof['source_witnesses'],
            current_witness=mod.source_witness(reference['contract'], reference['object'], reference['mask']),
            issued_ns=int(self._ros_source_clock_ns()), snapshot_policy=proof['snapshot_policy'])
mod.StationComputeWaitRemoteMixin._station_supplement_computation_receipt = straightline
import pytest
raise SystemExit(pytest.main(sys.argv[1:]))
'''
cmd = [sys.executable, '-B'] + (['-c', bootstrap] if baseline else
      ['-c', counterfactual_bootstrap] if counterfactual else ['-m', 'pytest']) + arguments
started = time.monotonic()
with (out / 'pytest.log').open('x') as stream:
    run = subprocess.run(cmd, cwd=work, env=env, stdout=stream, stderr=subprocess.STDOUT)
summary = {key: sum(int(row.get(key, 0)) for row in ET.parse(out / 'pytest.xml').getroot().iter('testsuite'))
           for key in ('tests', 'failures', 'errors', 'skipped')} if (out / 'pytest.xml').exists() else None
receipt = dict(returncode=run.returncode, summary=summary, source_sha256=before,
    source_unchanged=before == hashes(), elapsed_sec=time.monotonic() - started,
    baseline_overlay=baseline, baseline_manifest=manifest if baseline else None,
    counterfactual_straightline_receipt=counterfactual,
    counterfactual_limit=('Minimal in-memory reversion of context-then-build receipt semantics; '
        'not a complete historical production snapshot and not counted as a GEN28 regression.'
        if counterfactual else None),
    command=cmd, live_ros_contacted=False, motion_commands_sent=False, temperature_queried=False)
(out / 'receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
print(json.dumps(receipt))
print((out / 'pytest.log').read_text())
sys.exit(run.returncode)
