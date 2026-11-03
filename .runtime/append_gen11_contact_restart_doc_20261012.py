"""Append sealed repair/restart and failed attempt evidence to operations docs."""
from pathlib import Path
import hashlib
import json
import os
import time

W = Path(__file__).resolve().parents[1]
EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N = EXT / 'ros_log/full_latest_contact_admission_20261013T011103Z'
R = EXT / 'grasp_records/20261013T012841.933029Z_8537e8805ca34fa9be2d153cdbc37b25'
OUT = N / 'verification/gen11_contact_restart_document_update'
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
read = lambda p: json.loads(Path(p).read_text())
assert Path('/media/zhuyupei/SanStand').is_mount()
release_path = N / 'verification/gen11_contact_release_manifest.json'
release = read(release_path)
assert sha(release_path) == '5d7363f1b46764a9d209bc09d0b175e179ff37a21683ebaf5894ddefba965137'
assert release['test_summary'] == dict(tests=3215, failures=0, errors=0, skipped=0)
assert len(release['source_sha256']) == 882
assert all(sha(W / p) == h for p, h in release['source_sha256'].items())
relative = 'docs/operations/2026-10-11-frozen-grasp-acceptance.md'
assert relative not in release['source_sha256']
paths = [W / relative, EXT / '文档' / Path(relative).name, N / Path(relative).name]
before = paths[0].read_bytes()
assert sha(paths[0]) == '6da6a8757bfda535bde3e6cf7f5b22ed9915b1c8bdc3f60856f86cf55d765bd0'
assert paths[1].read_bytes() == before and not paths[2].exists()
fragment = W / '.runtime/gen11_contact_restart_doc_fragment.txt'
assert '本草稿' not in fragment.read_text() and '【主代理正式追加' not in fragment.read_text()
assert fragment.read_text().splitlines()[0] not in before.decode()
assert read(N / 'full_restart_acceptance/result.json')['deployment_passed'] is True
assert read(N / 'attempt_1_readiness/result.json')['maximum_joint_delta_rad'] == 0.0
record = read(R / 'manifest.json')
assert record['state'] == 'complete' and record['result'] == 'failure' and record['recorder_closed'] is True
assert sha(R / 'manifest.json') == 'c8d5461f752b6d9ea70e2e89bb83b896318ba2b6901ee7fd8aa169066e02f3ce'
outcome = read(R / 'recorder_outcome.json')
assert outcome['bag_closed'] is True
OUT.mkdir(exist_ok=False)
(OUT / 'before.md').write_bytes(before)
after = before + b'\n\n' + fragment.read_bytes()
for destination in paths:
    temporary = destination.with_name(destination.name + '.gen11_restart_update_tmp')
    with temporary.open('xb') as stream:
        stream.write(after)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, destination)
assert all(p.read_bytes() == after for p in paths)
assert all(sha(W / p) == h for p, h in release['source_sha256'].items())
evidence = [release_path, N / 'verification/gen11_contact_actual_integrated/result.json',
    N / 'verification/gen11_contact_actual_integrated_r2/result.json',
    N / 'verification/gen11_contact_fixture_single_file/result.json',
    N / 'verification/gen11_contact_fixture_independent_review.json',
    N / 'verification/gen11_contact_fixture_integrated_install/result.json',
    N / 'startup.json', N / 'node_registration.json', N / 'full_restart_acceptance/result.json',
    N / 'attempt_1_scene/operator_observation.json', N / 'attempt_1_readiness/result.json',
    R / 'manifest.json', R / 'recorder_outcome.json']
receipt = dict(passed=True, recorded_ns=time.time_ns(),
    document_sha256=sha(paths[0]), document_paths=list(map(str, paths)),
    before_sha256=hashlib.sha256(before).hexdigest(), before_copy=str(OUT / 'before.md'),
    fragment_path=str(fragment), fragment_sha256=sha(fragment),
    evidence_sha256={str(p):sha(p) for p in evidence},
    source_count=882, production_sources_unchanged=True,
    history_preserved=True, result_classification='failure', physical_grasp_verified=False,
    motion_commands_sent=False, temperature_queried=False, services_changed=False,
    script_sha256=sha(__file__))
(OUT / 'receipt.json').write_text(json.dumps(receipt, ensure_ascii=False, indent=2) + '\n')
print(json.dumps({k:receipt[k] for k in ('passed', 'document_sha256', 'document_paths')}, ensure_ascii=False))
