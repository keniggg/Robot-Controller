"""Record a fresh operator-reviewed GEN26 scene under active authorization; no ROS calls."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import time
import uuid

from continuous_grasp_authorization_20261012 import atomic_json

M = Path('/media/zhuyupei/SanStand')
E = M / '实验记录/Robot-Controller'
N = E / 'ros_log/full_latest_gen22_restart_20261015T003700Z'
A = E / 'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/authorization.json'
AUTH_SHA = '132e83f926a249f8d1294ddeb74e8b38224744ea369a7acf8a6c29802f0c9172'


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def ref(path):
    return dict(path=str(path), sha256=sha(path))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--readiness', type=Path, required=True)
    parser.add_argument('--visual-assessment', required=True,
                        help='Actual operator assessment of this exact image: same target visible and no conflicting hand.')
    parser.add_argument('--control-revision', type=int, required=True)
    parser.add_argument('--user-request', required=True)
    parser.add_argument('--clearance-reply-id', required=True)
    parser.add_argument('--clearance-text', required=True)
    args = parser.parse_args()
    scene, ready = args.scene.resolve(strict=True), args.readiness.resolve(strict=True)
    assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
    assert N in scene.parents and N in ready.parents
    assert scene.stat().st_dev == ready.stat().st_dev == M.stat().st_dev
    assert all(value.strip() for value in (args.visual_assessment, args.user_request,
                                          args.clearance_reply_id, args.clearance_text))
    assert sha(A) == AUTH_SHA
    auth = read(A)
    hands_path = Path(auth['original_hands_clear_receipt']['path']).resolve(strict=True)
    assert E in hands_path.parents
    assert sha(hands_path) == auth['original_hands_clear_receipt']['sha256']
    hands = read(hands_path)
    inputs = {path: sha(path) for path in (A, hands_path, scene / 'preflight.json',
                                         scene / 'current_scene.png', ready)}
    pre, readiness = read(scene / 'preflight.json'), read(ready)
    assert readiness['passed'] is True and readiness['maximum_joint_delta_rad'] <= .004
    assert readiness['selection'] == pre['selection']
    assert readiness['epoch_ns'] == pre['epoch_ns']
    assert readiness['sample_count'] >= 20
    assert readiness['deployment_processes_unchanged'] is True
    assert readiness['source_matches_operational_release'] is True
    assert readiness['motion_commands_sent'] is False and readiness['temperature_queried'] is False
    assert pre['manual_mode'] is True and readiness['manual_mode'] is True
    assert pre['task_active'] is False and readiness['task_active'] is False
    assert 0 <= pre['receipt_ns'] - pre['image_stamp_ns'] <= 800_000_000
    joint = pre['topics']['/alicia_d/accepted_joint_states']
    assert joint['frame_id'] == 'sdk_measured'
    assert 0 <= pre['receipt_ns'] - joint['stamp_ns'] <= 200_000_000

    # Serialize publication with pause/resume so no observation is committed
    # after a control change. These writes do not change authorization state.
    with (A.parent / 'control.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        control = read(A.parent / 'control.json')
        assert control['state'] == 'active' and control['revision'] == args.control_revision
        assert control['authorization_id'] == auth['authorization_id']
        now = time.time_ns()
        assert auth['issued_ns'] <= pre['receipt_ns'] <= readiness['recorded_ns'] <= now
        assert now - pre['receipt_ns'] < 90_000_000_000
        assert all(sha(path) == digest for path, digest in inputs.items())
        for name in ('scene_clear_receipt.json', 'resume_request.json', 'operator_observation.json'):
            assert not (scene / name).exists(), name
        clear = dict(source='actual_user_question_reply', message_id=args.clearance_reply_id,
                     user_message=args.clearance_text, recorded_ns=now,
                     recorded_time_kind='locally_recorded_receipt_time',
                     new_grasp_authorization_claimed=False)
        atomic_json(scene / 'scene_clear_receipt.json', clear)
        resume = dict(source='actual_user_message', user_text=args.user_request,
                      platform_message_id=None, local_record_id=uuid.uuid4().hex,
                      recorded_ns=now, recorded_time_kind='locally_recorded_receipt_time',
                      continuous_authorization=ref(A),
                      scope='Continue resumed GEN26 work and recorded grasp under existing authorization; fresh scene clearance separately confirmed.')
        atomic_json(scene / 'resume_request.json', resume)
        observation = dict(version=1, observation_id=uuid.uuid4().hex,
            new_user_confirmation_claimed=False, same_target_observed=True, target_visible=True,
            hands_clear_basis=dict(prior_explicit_user_reply_id=hands['message_id'],
                current_image_review_no_conflicting_hand_in_motion_area=True,
                current_scene_clear_reply=ref(scene / 'scene_clear_receipt.json')),
            image=dict(ref(scene / 'current_scene.png'), source_stamp_ns=pre['image_stamp_ns']),
            preflight=ref(scene / 'preflight.json'), readiness=ref(ready),
            captured_ns=pre['receipt_ns'], assessed_ns=now, valid_until_ns=now + 120_000_000_000,
            visual_assessment=args.visual_assessment,
            current_resume_request=ref(scene / 'resume_request.json'),
            preparation_script_sha256=sha(__file__), current_user_request=resume,
            current_site_confirmation=clear)
        assert all(sha(path) == digest for path, digest in inputs.items())
        atomic_json(scene / 'operator_observation.json', observation)
    print(json.dumps(dict(observation=str(scene / 'operator_observation.json'),
                          control_state='active', revision=args.control_revision,
                          valid_until_ns=observation['valid_until_ns']), ensure_ascii=False))


if __name__ == '__main__':
    main()
