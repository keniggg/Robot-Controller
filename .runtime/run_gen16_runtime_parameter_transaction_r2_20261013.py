#!/usr/bin/env python3
"""Offline paired race reproduction and focused regression, externally recorded."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

W = Path(__file__).resolve().parents[1]
N = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
PRIOR = N / 'verification/gen16_runtime_parameter_transaction_r1'
OUT = N / 'verification/gen16_runtime_parameter_transaction_r2'
C = W / '.runtime/gen16_runtime_parameter_transaction_20261013'
B = W / '.runtime/gen16_runtime_parameter_transaction_baseline_20261013'
NEW = [
    'src/alicia_flexible_grasp_supervisor/tests/test_runtime_refresh_callback_transaction.py',
    'src/alicia_grasp_modes/tests/test_runtime_refresh_session_handoff.py',
]
FOCUSED = NEW + [
    'src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py',
    'src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py',
    'src/alicia_grasp_modes/tests/test_observe_return_session_handoff_concurrency.py',
]

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def sources(root):
    return {str(p.relative_to(root)): sha(p) for p in root.rglob('*')
            if p.is_file() and p.suffix in ('.py', '.xml', '.txt', '.json')
            and '__pycache__' not in p.parts and '.pytest_cache' not in p.parts}

def run(label, root, scope):
    output = OUT / label
    output.mkdir()
    environment = dict(os.environ)
    environment['ROS_MASTER_URI'] = 'http://127.0.0.1:1'
    environment['ROS_PACKAGE_PATH'] = str(root/'src')+':'+environment.get('ROS_PACKAGE_PATH', '')
    environment['PYTHONPATH'] = ':'.join(str(root/'src'/package/'src') for package in
        ('alicia_grasp_modes', 'alicia_flexible_grasp_supervisor'))+':'+environment.get('PYTHONPATH', '')
    command = ['python3', '-m', 'pytest', '-q', *[str(root/rel) for rel in scope],
        '--disable-warnings', '--tb=short', '--junitxml='+str(output/'pytest.xml')]
    started = time.time_ns()
    with (output/'pytest.log').open('w') as log:
        process = subprocess.run(command, cwd=W, env=environment, stdout=log, stderr=subprocess.STDOUT)
    xml = ET.parse(output/'pytest.xml').getroot()
    suites = list(xml.iter('testsuite'))
    counts = {key:sum(int(s.attrib.get(key, 0)) for s in suites)
              for key in ('tests', 'failures', 'errors', 'skipped')}
    result = dict(root=str(root), scope=scope, command=command, started_ns=started,
        finished_ns=time.time_ns(), returncode=process.returncode, summary=counts,
        failing_cases=[c.attrib.get('name') for c in xml.iter('testcase') if c.find('failure') is not None],
        log_sha256=sha(output/'pytest.log'), xml_sha256=sha(output/'pytest.xml'))
    (output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(label, counts, flush=True)
    return result

def main():
    if not Path('/media/zhuyupei/SanStand').is_mount():
        raise RuntimeError('SanStand is not mounted')
    OUT.mkdir()
    release = N/'verification/gen15_phase_timestamp_release_manifest.json'
    sealed = json.loads(release.read_text())['source_sha256']
    assert all(sha(W/rel) == value for rel, value in sealed.items())
    before = {'baseline': sources(B), 'candidate': sources(C)}
    prior = json.loads((PRIOR/'result.json').read_text())
    assert before['baseline'] == prior['source_sha256']['baseline']
    baseline = prior['baseline']
    candidate = run('candidate', C, FOCUSED)
    after = {'baseline': sources(B), 'candidate': sources(C)}
    same_actual = all(sha(W/rel) == value for rel, value in sealed.items())
    expected_baseline = baseline['summary'] == dict(tests=28, failures=25, errors=0, skipped=0)
    case_names = baseline['failing_cases']
    expected_baseline &= sum(name.startswith('test_full_valid_stage_preserves_callback_changes')
                            for name in case_names) == 24
    expected_baseline &= 'test_parameter_refresh_cannot_undo_real_session_pause' in case_names
    log = (PRIOR/'baseline/pytest.log').read_text()
    expected_baseline &= 'obsolete worker entered next planning stage' in log and 'AttributeError' not in log
    result = dict(kind='GEN16_RUNTIME_PARAMETER_TRANSACTION_AUTHOR_PROOF',
        passed=bool(expected_baseline and candidate['returncode'] == 0 and before == after and same_actual),
        baseline_expected_failure_reproduced=expected_baseline, baseline=baseline,
        baseline_reused=str(PRIOR/'result.json'), baseline_reused_sha256=sha(PRIOR/'result.json'),
        r1_nonproduction_failures='14 original tests lacked immutable fixture/config; added exact baseline copies only',
        candidate=candidate, source_unchanged=before == after, actual_unchanged=same_actual,
        installed_source_count=len(sealed), source_sha256=before,
        release_path=str(release), release_sha256=sha(release), runner_sha256=sha(__file__),
        live_ros_contacted=False, motion_commands_sent=False, temperature_queried=False,
        physical_grasp_verified=False, recorded_ns=time.time_ns())
    (OUT/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('passed','baseline_expected_failure_reproduced',
        'source_unchanged','actual_unchanged','installed_source_count')},sort_keys=True),flush=True)
    print(str(OUT/'result.json'), sha(OUT/'result.json'),flush=True)
    assert result['passed']

if __name__ == '__main__':
    main()
