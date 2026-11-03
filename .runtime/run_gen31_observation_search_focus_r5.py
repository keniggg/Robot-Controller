"""Offline final planner/consumer handoff regression; reports only to SanStand."""
from pathlib import Path
import hashlib
import json
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

W = Path(__file__).resolve().parents[1]
C = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen29_restart_20261016T000728Z')
assert Path('/media/zhuyupei/SanStand').is_mount()
prior = C / 'verification/gen31_observation_search_focus_r4/result.json'
record = json.loads(prior.read_text())
out = C / 'verification/gen31_observation_search_focus_r5'
out.mkdir()
tests = record['command'][8:] + [
    'src/alicia_grasp_modes/tests/test_gen31_observation_search_task.py',
    'src/alicia_grasp_modes/tests/test_gen31_budget_handoff_independent.py',
]
assert all((W / p).is_file() and p.endswith('.py') for p in tests)
sources = sorted(set(record['sources_before']) | set(tests) | {
    'src/alicia_grasp_modes/scripts/mode_aware_grasp_task.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_task.py',
    'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_remote.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_visual.py',
})


def hashes():
    return {p: hashlib.sha256((W / p).read_bytes()).hexdigest() for p in sources}


before = hashes()
command = [sys.executable, '-B', '-m', 'pytest', '-q', '-p', 'no:cacheprovider',
           '--junitxml=' + str(out / 'pytest.xml')] + tests
env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1',
           OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', ROS_MASTER_URI='http://127.0.0.1:1')
started = time.monotonic()
with (out / 'pytest.log').open('wb') as log:
    result = subprocess.run(command, cwd=W, env=env, stdout=log, stderr=subprocess.STDOUT)
after = hashes()
suites = []
if (out / 'pytest.xml').is_file():
    suites = [v.attrib for v in ET.parse(out / 'pytest.xml').getroot().iter('testsuite')]
report = dict(command=command, returncode=result.returncode,
    elapsed_sec=time.monotonic()-started, testsuites=suites,
    sources_before=before, sources_after=after, source_unchanged=before == after,
    prior_focus=str(prior), no_ros_or_hardware=True, no_deploy_or_grasp=True,
    delta='A successfully published, exactly station-bound budget failure retains only the paused identity wait through the next station tick. Original current owner, clock, feedback and real target-loss guards still retire the station; no new READY, snapshot policy or work certificate is granted. Actual producer/post-publication poll/task callback integration is included.',
    owned_production=record['owned_production'], new_owned_tests=record['new_owned_tests'])
path = out / 'result.json'
path.write_text(json.dumps(report, indent=2, sort_keys=True, ensure_ascii=False)+'\n')
print(json.dumps(dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
    returncode=result.returncode, source_unchanged=before == after, testsuites=suites)))
sys.exit(result.returncode or (0 if before == after else 2))
