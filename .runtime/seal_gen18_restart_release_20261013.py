"""Seal completed source/test evidence on SanStand; never contact ROS."""
import hashlib
import json
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M / '实验记录/Robot-Controller'
N = E / 'ros_log/full_latest_gen18_restart_20261014T012216Z'
V = N / 'verification'

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def read(path):
    return json.loads(Path(path).read_text())

def write_new(path, value):
    assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
    assert path.parent.stat().st_dev == M.stat().st_dev
    with path.open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())

assert M.is_mount() and N.is_dir()
assert read(E / 'maintenance/storage_control.json')['archive_policy'] == 'local_only'
config = read(E / 'grasp_records/archive_config.json')
assert config['upload_enabled'] is False and config['delete_enabled'] is False
evidence = {}
def bind(path):
    path = Path(path).resolve(strict=True)
    evidence[str(path)] = sha(path)
    return path

phase_results = {}
for phase, expected_tests in [('focused', 590), ('combined', 5815)]:
    folder = V / ('gen18_' + phase)
    receipt = read(bind(folder / 'result.json'))
    assert receipt['passed'] and receipt['source_unchanged'] and receipt['returncode'] == 0
    assert receipt['production_worktree_tested'] and not receipt['live_ros_contacted']
    assert receipt['summary'] == dict(tests=expected_tests, failures=0, errors=0, skipped=0)
    for file, key in [('pytest.log', 'log_sha256'), ('pytest.xml', 'xml_sha256')]:
        assert sha(bind(folder / file)) == receipt[key]
    suites = ET.parse(folder / 'pytest.xml').getroot()
    totals = {key: sum(int(s.get(key, '0')) for s in suites.iter('testsuite'))
              for key in ('tests', 'failures', 'errors', 'skipped')}
    assert totals == receipt['summary']
    for name, digest in receipt['source_sha256'].items():
        assert not Path(name).is_absolute() and '..' not in Path(name).parts
        assert sha(W / name) == digest, name
    for name in ('baseline', 'patch'):
        assert sha(bind(receipt[name])) == receipt[name + '_sha256']
    phase_results[phase] = receipt

combined = phase_results['combined']
assert combined['source_sha256'] == phase_results['focused']['source_sha256']
assert len(combined['source_sha256']) == 924 and len(combined['test_scope']) == 184
review_path = bind(W / '.runtime/gen18_restart_release_review_20261013.json')
assert sha(review_path) == '124a89cd1ed210f29868cbf4d909a26e62cbe2d0e034ac114aa6fa9f59a1b3f9'
review = read(review_path)
assert review['passed'] and review['reviewed'] and not review['blocking_findings']
assert review['source_manifest_sha256'] == combined['patch_sha256']
assert review['focused_receipt_sha256'] == sha(V / 'gen18_focused/result.json')
for name, digest in review['reviewed_source_sha256'].items():
    assert sha(W / name) == digest, name
external_review = V / 'gen18_independent_source_review.json'
write_new(external_review, review)
bind(external_review)

for name in ('.runtime/start_gen18_full_restart_20261013.py',
             '.runtime/verify_gen18_restart_candidate_20261013.py',
             '.runtime/verify_gen18_idle_gui_camera_20261013.py',
             '.runtime/verify_full_contact_restart_20261012.py',
             '.runtime/seal_gen18_restart_release_20261013.py',
             'docs/operations/2026-10-13-r6-recorded-recheck-evidence.md'):
    bind(W / name)
assert sha(W / '.runtime/verify_gen18_restart_candidate_20261013.py') == combined['runner_sha256']
bind(E / 'ros_log/full_latest_contact_admission_20261013T011103Z/gen17_deployment.json')
for name in ('2026-10-13-r6-recorded-recheck-evidence.md',
             '2026-10-13-offline-stable-recheck-recovery.md'):
    source = W / 'docs/operations' / name
    target = E / '文档' / name
    if target.exists():
        assert sha(target) == sha(source)
    else:
        with target.open('xb') as stream:
            stream.write(source.read_bytes())
            stream.flush()
            os.fsync(stream.fileno())
    bind(target)

control_path = E / 'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/control.json'
control = read(control_path)
assert control['state'] == 'paused', 'automatic grasp must remain paused'
instruction_path = N / 'operator_restart_only_instruction.json'
write_new(instruction_path, dict(recorded_ns=time.time_ns(),
    user_text='先恢复节点与上位机，暂不开始抓取',
    reply_reference='call_c09441fef4754b438699de661f40578e:0',
    node_and_gui_restart_authorized=True, automatic_grasp_authorized_now=False,
    automatic_grasp_started=False, continuous_control_path=str(control_path),
    continuous_control_state=control['state'], continuous_control_revision=control['revision']))
bind(instruction_path)
release = dict(schema_version=1, kind='gen18_complete_stable_recheck_classification',
    sealed_ns=time.time_ns(), passed=True, reviewed=True, installed=True,
    installed_semantics='Source is installed and tested in the operational worktree; runtime loading is pending cold-start acceptance.',
    source_sha256=combined['source_sha256'], source_count=924,
    test_scope=combined['test_scope'], test_scope_files=184, test_summary=combined['summary'],
    actual_regression=str(V / 'gen18_combined/result.json'),
    focused_regression=str(V / 'gen18_focused/result.json'),
    candidate_delivery=combined['patch'], combined_review=str(external_review),
    evidence_sha256=evidence, runtime_deployment_pending=True,
    physical_grasp_verified=False, motion_commands_sent=False, temperature_queried=False,
    current_user_scope='Restore nodes and GUI only; keep automatic grasp paused.')
path = V / 'gen18_release_manifest.json'
write_new(path, release)
print(json.dumps(dict(release=str(path), sha256=sha(path), summary=release['test_summary'],
                     source_count=924, evidence_files=len(evidence)), ensure_ascii=False))
