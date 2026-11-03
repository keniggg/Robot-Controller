"""Preserve the closed generation-4 failure and its recovered sidecar receipt."""
from pathlib import Path
import hashlib
import json
import os
import time

W = Path(__file__).resolve().parents[1]
EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N = EXT / 'ros_log/full_latest_contact_admission_20261013T011103Z'
R = EXT / 'grasp_records/20261013T083836.150489Z_f4adfdd7ede44edeb725b4bfd7ac4f32'
OUT = N / 'attempt_3_seal'
read = lambda p: json.loads(Path(p).read_text())


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(8 * 2**20), b''):
            digest.update(chunk)
    return digest.hexdigest()


def write(path, value):
    data = (json.dumps(value, ensure_ascii=False, indent=2) + '\n').encode()
    temporary = path.with_name(path.name + '.attempt3_tmp')
    with temporary.open('xb') as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


assert Path('/media/zhuyupei/SanStand').is_mount() and not OUT.exists()
manifest, outcome = read(R / 'manifest.json'), read(R / 'recorder_outcome.json')
reason = 'QUALIFIED_STATION_PHASE_OWNER_CHANGED_OR_EXPIRED'
assert manifest['state'] == 'complete' and manifest['result'] == 'failure'
assert manifest['failure_reason'] == reason and manifest['recorder_closed'] and outcome['bag_closed']
assert manifest['result_evidence']['runner_returncode'] == 6
assert len(manifest['result_evidence']['observations']) >= 2
assert all(x['result'] == 'failure' for x in manifest['result_evidence']['observations'])
assert 'GRASP_RESULT success=False message=' + reason in (R / 'runner.log').read_text()
rows = []
for item in manifest['files']:
    path = R / item['path']
    assert R in path.resolve().parents and path.is_file() and not path.is_symlink()
    assert path.stat().st_size == item['size_bytes'] and sha(path) == item['sha256'], item['path']
    rows.append(dict(item, verified=True))
side_path = N / 'attempt_3_full_frames/close_verification_after_wrapper_assertion.json'
side = read(side_path)
assert side['passed'] and side['recording_closed'] and side['no_open_writer']
bag = Path(side['path'])
assert bag == N / 'attempt_3_full_frames/full_frames.bag'
assert bag.stat().st_size == side['size_bytes'] and sha(bag) == side['sha256']
assert not list(R.glob('*.bag.active')) and not list(bag.parent.glob('*.bag.active'))
manual_path = N / 'attempt_3_after_manual/result.json'
manual = read(manual_path)
assert manual['manual_restored'] and manual['gui_own_publisher_acknowledged'] and manual['verified_inactive']
assert manual['generation'] == 4
selection = read(R / 'authorization.json')['selection']
assert selection['generation'] == 4 and selection['stamp_ns'] == 1791880718524907350
deployment_path = N / 'gen14_deployment.json'
deployment = read(deployment_path)
assert deployment['deployment_passed'] and deployment['whole_patch_deployed']
sources = deployment['source_sha256']
assert len(sources) == 904 and all(sha(W / path) == digest for path, digest in sources.items())
protected_path = EXT / 'maintenance/failed_record_space_guard_protected.json'
protected_before = protected_path.read_bytes()
protected = json.loads(protected_before)
entries = protected['protected_paths']
assert len(entries) == 107
additions = [str(R.relative_to(EXT)), str(bag.parent.relative_to(EXT))]
assert not any(row['path'] in additions for row in entries)
metadata = [N / 'current_progress.json', W / '.runtime/latest_full_ros_start.json']
before = {str(path): path.read_bytes() for path in metadata}
OUT.mkdir()
(OUT / 'protected_before.json').write_bytes(protected_before)
for index, path in enumerate(metadata):
    (OUT / ('progress_before_%d.json' % index)).write_bytes(before[str(path)])
now = time.time_ns()
verification = dict(passed=True, recorded_ns=now, record=str(R), result='failure',
    failure_reason=reason, manifest_sha256=sha(R / 'manifest.json'), files=rows,
    recorder_closed=True, sidecar=dict(receipt=str(side_path), receipt_sha256=sha(side_path),
        bag_path=str(bag), size_bytes=side['size_bytes'], sha256=side['sha256'],
        original_wrapper_assertion_preserved=True, independently_verified_closed=True),
    selection=selection, manual=dict(path=str(manual_path), sha256=sha(manual_path)),
    source_count=904, production_sources_unchanged=True, physical_grasp_verified=False,
    motion_commands_sent=False, temperature_queried=False)
write(OUT / 'manifest_verification.json', verification)
assert protected_path.read_bytes() == protected_before
protected['protected_paths'] = entries + [dict(path=path, reason=
    'Current GEN15 qualified-station phase ownership investigation: confirmed closed attempt 3 failure; preserve all original recordings and metadata.')
    for path in additions]
write(protected_path, protected)
for path in metadata:
    assert path.read_bytes() == before[str(path)]
    current = json.loads(before[str(path)])
    current.update(updated_ns=now, active_work='gen15_qualified_station_phase_owner_diagnosis',
        automatic_grasp_running=False, active_record=None, last_record=str(R), current_record=str(R),
        last_result='failure', last_failure_reason=reason, last_record_sealed=True,
        current_generation=4, expected_generation=4, next_generation=5, selection=selection,
        manual_mode=True, verified_manual_state_evidence=str(manual_path), manual_evidence=str(manual_path),
        manual_state_is_live_snapshot=False, physical_grasp_verified=False,
        closed_record_verification=str(OUT / 'manifest_verification.json'),
        full_frame_sidecar=str(side_path), full_frame_sidecar_bag=str(bag),
        full_frame_sidecar_sha256=side['sha256'], root_cause_confirmed=False)
    write(path, current)
assert read(protected_path)['protected_paths'][:len(entries)] == entries
assert all(sha(W / path) == digest for path, digest in sources.items())
write(OUT / 'result.json', dict(passed=True, recorded_ns=now, record=str(R),
    manifest_verification_sha256=sha(OUT / 'manifest_verification.json'),
    protected_before_count=len(entries), protected_after_count=len(entries) + 2,
    protected_added=additions, protected_before_sha256=hashlib.sha256(protected_before).hexdigest(),
    protected_after_sha256=sha(protected_path), source_count=904,
    metadata_after_sha256={str(path): sha(path) for path in metadata},
    physical_grasp_verified=False, no_recording_deleted=True, motion_commands_sent=False,
    temperature_queried=False, script_sha256=sha(__file__)))
print(json.dumps(dict(passed=True, record=str(R), manifest_files=len(rows), protected_count=109)))
