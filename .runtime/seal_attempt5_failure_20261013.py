"""Seal R5's actual failure without rehashing an already verified large bag."""
from pathlib import Path
import hashlib
import json
import os
import time

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N = E / 'ros_log/full_latest_contact_admission_20261013T011103Z'
R = E / 'grasp_records/20261013T112006.120233Z_d2b04edd648643b4b57d8e9febc4d88c'
O = N / 'attempt_5_seal'
read = lambda p: json.loads(Path(p).read_text())
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()


def save(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write('\n')


assert Path('/media/zhuyupei/SanStand').is_mount() and not O.exists()
manifest = read(R / 'manifest.json')
assert manifest['state'] == 'complete' and manifest['result'] == 'failure'
assert manifest['failure_reason'] == 'STATION_CONTACT_PROVENANCE_INVALID: station_compute_wait_registration_too_late'
assert manifest['recorder_closed'] and read(R / 'recorder_outcome.json')['bag_closed']
extract_path = N / 'verification/attempt5_registration_timeline_r1/result.json'
extract = read(extract_path)
assert extract['extraction_complete'] and extract['source_bag_unchanged']
assert extract['original_manifest_sha256'] == sha(R / 'manifest.json')
files = []
for row in manifest['files']:
    path = R / row['path']
    assert R in path.resolve().parents and not path.is_symlink()
    assert path.stat().st_size == row['size_bytes']
    if row['path'] == 'keyframes.bag':
        assert extract['source_bag'] == str(path) and extract['source_bag_sha256'] == row['sha256']
        provenance = dict(hash_provenance=str(extract_path), hash_provenance_sha256=sha(extract_path))
    else:
        assert sha(path) == row['sha256']
        provenance = dict(hash_computed_now=True)
    files.append(dict(row, verified=True, **provenance))
topics = extract['all_bag_topics']
assert topics['/alicia_controller/follow_joint_trajectory/goal']['count'] == 2
assert topics['/alicia_controller/follow_joint_trajectory/result']['count'] == 2
assert not any('hand_controller' in t and t.endswith('/goal') for t in topics)
last = [row for row in read(N / 'verification/attempt5_registration_timeline_r1/wire.json')
        if row['topic'] == '/grasp/state'][-1]
assert last['message']['state'] == 'FAILED' and last['message']['active'] is False
assert sha(last['file']) == last['sha256']
side_path = N / 'attempt_5_full_frames/result.json'
side = read(side_path)
assert side['recording_closed'] and side['returncode'] == 0
bag = Path(side['path'])
assert bag == N / 'attempt_5_full_frames/full_frames.bag'
assert bag.stat().st_size == side['size_bytes'] and bag.stat().st_mtime_ns <= side['closed_ns']
assert not list(R.glob('*.bag.active')) and not list(bag.parent.glob('*.bag.active'))
parent = Path('/proc') / str(side['pid']) / 'stat'
if parent.exists():
    state = parent.read_text().rsplit(')', 1)[1].split()
    assert state[0] == 'Z' or state[19] != side['start_ticks']
writers, inaccessible = [], 0
for process in Path('/proc').iterdir():
    if not process.name.isdigit():
        continue
    try:
        descriptors = list((process / 'fd').iterdir())
    except (PermissionError, FileNotFoundError):
        inaccessible += 1
        continue
    for fd in descriptors:
        try:
            if os.readlink(fd) not in (str(bag), str(R / 'keyframes.bag')):
                continue
            lines = (process / 'fdinfo' / fd.name).read_text().splitlines()
            flags = int(next(line.split(':', 1)[1].strip() for line in lines if line.startswith('flags:')), 8)
            if flags & 3:
                writers.append(dict(pid=process.name, fd=fd.name))
        except (PermissionError, FileNotFoundError, ProcessLookupError):
            continue
assert not writers
deployment_path = N / 'gen16_deployment.json'
deployment = read(deployment_path)
assert deployment['deployment_passed'] and deployment['whole_patch_deployed']
assert all(sha(W / name) == digest for name, digest in deployment['source_sha256'].items())
manual_path = N / 'attempt_5_after_manual/result.json'
manual = read(manual_path)
assert manual['manual_restored'] and manual['gui_own_publisher_acknowledged'] and manual['verified_inactive']
assert manual['generation'] == read(R / 'authorization.json')['selection']['generation'] == 6
protected = read(E / 'maintenance/failed_record_space_guard_protected.json')['protected_paths']
assert all(any(row['path'] == str(path.relative_to(E)) for row in protected) for path in (R, bag.parent))
O.mkdir()
verification = dict(passed=True, record=str(R), result='failure',
    failure_reason=manifest['failure_reason'], manifest_sha256=sha(R / 'manifest.json'), files=files,
    main_recorder_closed=True, sidecar=dict(receipt=str(side_path), receipt_sha256=sha(side_path),
        path=str(bag), size_bytes=side['size_bytes'], sha256=side['sha256'], recording_closed=True,
        hash_provenance='Owned reviewed recorder hashes every byte after finalization before marking closed'),
    no_accessible_open_writer=True, inaccessible_proc_fd_directories=inaccessible,
    action_goal_counts=dict(arm=2, hand=0), last_task_state=last,
    deployment=str(deployment_path), deployment_sha256=sha(deployment_path),
    source_count=len(deployment['source_sha256']), source_unchanged=True,
    manual_evidence=str(manual_path), manual_evidence_sha256=sha(manual_path),
    manual_restored=True, manual_state_is_live_snapshot=False,
    physical_grasp_verified=False, no_recording_deleted=True, recorded_ns=time.time_ns())
save(O / 'manifest_verification.json', verification)
for index, path in enumerate((N / 'current_progress.json', W / '.runtime/latest_full_ros_start.json')):
    before = path.read_bytes()
    (O / ('progress_before_%d.json' % index)).write_bytes(before)
    value = json.loads(before)
    assert value['last_record'] == str(R) and value['automatic_grasp_running'] is False
    value.update(last_record_sealed=True, main_recorder_closed=True, sidecar_seal_pending=False,
        closed_record_verification=str(O / 'manifest_verification.json'),
        full_frame_sidecar=str(side_path), full_frame_sidecar_bag=str(bag), full_frame_sidecar_sha256=side['sha256'],
        updated_ns=time.time_ns())
    assert path.read_bytes() == before
    temporary = path.with_name(path.name + '.seal5.tmp')
    save(temporary, value)
    temporary.replace(path)
save(O / 'result.json', dict(passed=True, record=str(R), manifest_files=len(files),
    protected_count=len(protected), source_count=len(deployment['source_sha256']),
    verification_sha256=sha(O / 'manifest_verification.json'),
    physical_grasp_verified=False, no_recording_deleted=True, script_sha256=sha(__file__)))
print(json.dumps(dict(passed=True, record=str(R), manifest_files=len(files), physical_grasp_verified=False)))
