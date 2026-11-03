"""Durably record an explicit GEN26 resume and update paused control once; no ROS calls."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import time
import uuid

from continuous_grasp_authorization_20261012 import atomic_json

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M / '实验记录/Robot-Controller'
N = E / 'ros_log/full_latest_gen22_restart_20261015T003700Z'
A = E / 'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/authorization.json'
CHECKPOINT = N / 'gen26_user_pause_checkpoint/checkpoint.json'
AUTH_SHA = '132e83f926a249f8d1294ddeb74e8b38224744ea369a7acf8a6c29802f0c9172'


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_path(name):
    path = Path(name)
    assert not path.is_absolute() and '..' not in path.parts
    resolved = (W / path).resolve(strict=True)
    assert W in resolved.parents
    return resolved


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--user-request', required=True)
    parser.add_argument('--expected-revision', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--review', type=Path,
                        help='Independent review covering any changed checkpoint sources at their current hashes.')
    parser.add_argument('--review-sha256')
    args = parser.parse_args()
    assert args.user_request.strip()
    assert bool(args.review) == bool(args.review_sha256)
    assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
    out = args.output.resolve()
    result_path = out.with_name(out.stem + '_result.json')
    assert N in out.parents and out.parent.is_dir()
    assert out.parent.stat().st_dev == M.stat().st_dev
    assert sha(A) == AUTH_SHA
    auth, checkpoint = read(A), read(CHECKPOINT)
    checkpoint_sha = sha(CHECKPOINT)
    assert checkpoint['paused'] is True and checkpoint['pending_repair'] == 'GEN26'
    assert checkpoint['control']['state'] == 'paused'
    assert checkpoint['control']['revision'] == args.expected_revision
    assert checkpoint['control']['authorization_id'] == auth['authorization_id']
    pinned = {A: AUTH_SHA, CHECKPOINT: checkpoint_sha}
    for name, digest in checkpoint['completed_evidence'].items():
        path = Path(name).resolve(strict=True)
        assert E in path.parents and sha(path) == digest
        pinned[path] = digest
    reviewed = {}
    if args.review:
        review_path = args.review.resolve(strict=True)
        assert E in review_path.parents and sha(review_path) == args.review_sha256
        review = read(review_path)
        assert review['reviewed'] is True and not review['blocking_findings']
        reviewed = review['reviewed_source_sha256']
        assert isinstance(reviewed, dict)
        pinned[review_path] = args.review_sha256
        for name, digest in reviewed.items():
            assert sha(source_path(name)) == digest, name
        for name, digest in review.get('evidence_sha256', {}).items():
            path = Path(name).resolve(strict=True)
            assert E in path.parents or W in path.parents
            assert sha(path) == digest, name
            pinned[path] = digest
    current, changed = {}, {}
    for name, previous in checkpoint['source_sha256'].items():
        actual = sha(source_path(name))
        current[name] = actual
        if actual != previous:
            assert reviewed.get(name) == actual, 'Unreviewed checkpoint source change: ' + name
            changed[name] = dict(checkpoint_sha256=previous, current_sha256=actual,
                                 covered_by_independent_review=True)
    for name, digest in reviewed.items():
        current[name] = digest

    with (A.parent / 'control.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        control_path = A.parent / 'control.json'
        previous = read(control_path)
        # Match the actual paused record, including its evidence, not only its number.
        assert previous == checkpoint['control']
        assert previous['state'] == 'paused' and previous['revision'] == args.expected_revision
        assert previous['authorization_id'] == auth['authorization_id']
        assert not out.exists() and not result_path.exists()
        assert all(sha(path) == digest for path, digest in pinned.items())
        assert all(sha(source_path(name)) == digest for name, digest in current.items())
        now = time.time_ns()
        request = dict(version=1, source='actual_user_message', user_text=args.user_request,
            platform_message_id=None, local_record_id=uuid.uuid4().hex,
            recorded_ns=now, recorded_time_kind='locally_recorded_receipt_time',
            continuous_authorization=dict(path=str(A), sha256=AUTH_SHA),
            checkpoint=dict(path=str(CHECKPOINT), sha256=checkpoint_sha),
            control_before=previous, requested_control_revision=args.expected_revision + 1,
            source_sha256=current, source_changes_since_pause=changed,
            review=(dict(path=str(args.review.resolve()), sha256=args.review_sha256)
                    if args.review else None),
            source_freeze_claimed=False, regression_pass_claimed=False,
            scope='Resume unfinished GEN26 repair, verification and deployment; recorded grasp requires accepted release and fresh scene/readiness.',
            motion_commands_sent=False, temperature_queried=False,
            script_sha256=sha(__file__))
        # Persist the user's request before activating; a crash here leaves paused control.
        atomic_json(out, request)
        resumed = dict(previous, state='active', revision=args.expected_revision + 1,
            updated_ns=now, resumed_ns=now, resume_user_message=args.user_request,
            resume_session=str(N), resume_evidence=str(out),
            resume_scope=request['scope'])
        atomic_json(control_path, resumed)
        assert read(control_path) == resumed
        atomic_json(result_path, dict(resumed=True, control=resumed,
            request=dict(path=str(out), sha256=sha(out)),
            control_sha256=sha(control_path), motion_commands_sent=False,
            temperature_queried=False, recorded_ns=time.time_ns()))
    print(json.dumps(dict(resumed=True, state='active', revision=resumed['revision'],
                          request=str(out), result=str(result_path)), ensure_ascii=False))


if __name__ == '__main__':
    main()
