"""Write an honest fresh-image observation under active67; no ROS or new authority."""
import fcntl
import json
from pathlib import Path
import sys
import time
import traceback
import uuid

from gen61_compute_attempt_common_20261022 import (
    A, AUTH_SHA, CONTROL, CONTROL_SHA, E, REQUEST, REQUEST_SHA, S, W,
    CONTINUE_REQUEST, CONTINUE_REQUEST_SHA,
    load_context, new_json, parser, read, recheck, ref, require, same_selection, sha, storage)

COLLECTOR = W / '.runtime/gen61_compute_attempt_capture_20261022.py'


def main():
    p = parser(__doc__)
    p.add_argument('--scene', type=Path, required=True)
    p.add_argument('--readiness', type=Path, required=True)
    p.add_argument('--readiness-sha256', required=True)
    p.add_argument('--collector-sha256', required=True)
    p.add_argument('--review', type=Path, required=True)
    p.add_argument('--review-sha256', required=True)
    a = p.parse_args()
    scene = a.scene.resolve(strict=True)
    output = storage(scene / 'receipt_preparation.json')
    require(scene.parent == S and scene.name.startswith('scene_start_') and not output.exists(),
            'fresh runtime scene receipt required')
    result = dict(kind='GEN61_honest_current_scene_receipt_preparation', passed=False,
                  started_ns=time.time_ns(), motion_commands_sent=False, ROS_accessed=False,
                  temperature_queried=False, new_user_confirmation_claimed=False,
                  helper_path=str(Path(__file__).resolve()), helper_sha256=sha(__file__))
    try:
        context = load_context(a, __file__)
        readiness_path, review_path = a.readiness.resolve(strict=True), a.review.resolve(strict=True)
        require(readiness_path.parent.parent == S and readiness_path.parent.name.startswith('readiness_start_') and
                readiness_path.name == 'result.json' and review_path.parent == scene,
                'review/readiness outside current scene session')
        require(sha(readiness_path) == a.readiness_sha256 and sha(COLLECTOR) == a.collector_sha256 and
                sha(review_path) == a.review_sha256, 'reviewed collector/readiness/image review digest differs')
        ready, review, pre = read(readiness_path), read(review_path), read(scene / 'preflight.json')
        require(ready['passed'] is True and ready['helper_path'] == str(COLLECTOR) and
                ready['helper_sha256'] == a.collector_sha256 and ready['scene'] == str(scene),
                'strict current collector acceptance missing')
        require(ready['source_sha256'] == context['release']['source_sha256'] and
                ready['release'] == ref(context['release_path']) and
                ready['deployment'] == ref(context['deployment_path']) and
                ready['node_identities_before'] == ready['node_identities_after'] == context['nodes'],
                'readiness is not the exact current runtime release/17 instances')
        for path, digest in context['pins'].items():
            if path not in (Path(__file__).resolve(),):
                require(ready['input_sha256'].get(str(path)) == digest,
                        'collector input binding differs: ' + str(path))
        require(ready['scene_preflight_sha256'] == sha(scene / 'preflight.json') and
                ready['image_sha256'] == sha(scene / 'current_scene.png'), 'captured scene changed')
        require(ready['deployment_processes_unchanged'] is True and
                ready['source_matches_operational_release'] is True and ready['motion_commands_sent'] is False and
                ready['temperature_queried'] is False and ready['sample_count'] >= 20 and
                ready['unique_sample_count'] >= 20 and ready['maximum_joint_delta_rad'] <= .004,
                'current SDK/readiness contract incomplete')
        require(ready['manual_mode'] is True and ready['gui_owning_manual_ack'] is True and
                ready['task_active'] is False and ready['actions_active'] is False and
                ready['feedback_ready'] is True and ready['motion_enabled'] is True and
                ready['require_actuation_confirmation'] is True and ready['actuation_status'].startswith('CONFIRMED:'),
                'current manual/idle/confirmed actuation not accepted')
        require(ready['epoch_ns'] == pre['epoch_ns'] == context['deployment']['epoch_ns'] and
                same_selection(ready['selection'], pre['selection'], context['deployment']['selection']),
                'scene/readiness runtime epoch or selection differs')
        require(pre['manual_mode'] is True and pre['task_active'] is False,
                'captured scene is not manual/idle')
        require(review['source'] == 'actual_current_image_review' and
                review['new_user_confirmation_claimed'] is False and
                review['same_target_observed'] is True and review['target_visible'] is True and
                review['current_image_review_no_conflicting_hand_in_motion_area'] is True and
                isinstance(review['reviewer'], str) and review['reviewer'].strip() and
                isinstance(review['visual_assessment'], str) and review['visual_assessment'].strip(),
                'actual current image/hands assessment required; no substitute user reply')
        require(review['image'] == ref(scene / 'current_scene.png') and
                review['preflight'] == ref(scene / 'preflight.json') and
                review['readiness'] == ref(readiness_path), 'review is not of this exact scene/readiness')
        auth = read(A)
        hands_path = Path(auth['original_hands_clear_receipt']['path']).resolve(strict=True)
        require(E in hands_path.parents and sha(hands_path) == auth['original_hands_clear_receipt']['sha256'],
                'historical hands-clear evidence changed')
        hands = read(hands_path)
        require(hands.get('message_id'), 'original explicit hands-clear provenance missing')
        evidence = {p: sha(p) for p in (scene / 'preflight.json', scene / 'current_scene.png',
                                       readiness_path, review_path, A, hands_path, REQUEST, CONTINUE_REQUEST)}
        now = time.time_ns()
        require(type(review['assessed_ns']) is int and
                auth['issued_ns'] <= pre['receipt_ns'] <= ready['recorded_ns'] <= review['assessed_ns'] <= now and
                now - pre['receipt_ns'] < 90_000_000_000 and now < review['assessed_ns'] + 120_000_000_000,
                'capture/readiness/actual review order or validity expired')
        with (A.parent / 'control.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            recheck(context)
            control = read(CONTROL)
            require(sha(CONTROL) == CONTROL_SHA and control['state'] == 'active' and control['revision'] == 67 and
                    control['authorization_id'] == auth['authorization_id'], 'active67 changed before observation commit')
            require(all(sha(path) == digest for path, digest in evidence.items()), 'observation evidence changed')
            now = time.time_ns()
            require(now - pre['receipt_ns'] < 90_000_000_000 and now < review['assessed_ns'] + 120_000_000_000,
                    'observation expired before commit')
            observation = dict(version=1, observation_id=uuid.uuid4().hex,
                new_user_confirmation_claimed=False, same_target_observed=True, target_visible=True,
                hands_clear_basis=dict(prior_explicit_user_reply_id=hands['message_id'],
                    current_image_review_no_conflicting_hand_in_motion_area=True,
                    current_image_review=ref(review_path), historical_receipt=ref(hands_path),
                    no_new_user_scene_clear_reply_claimed=True),
                image=dict(ref(scene / 'current_scene.png'), source_stamp_ns=pre['image_stamp_ns']),
                preflight=ref(scene / 'preflight.json'), readiness=ref(readiness_path),
                captured_ns=pre['receipt_ns'], assessed_ns=review['assessed_ns'],
                valid_until_ns=review['assessed_ns'] + 120_000_000_000,
                visual_assessment=review['visual_assessment'], reviewer=review['reviewer'],
                current_resume_request=dict(path=str(REQUEST), sha256=REQUEST_SHA,
                    source='actual_user_message', no_platform_message_id_invented=True),
                latest_continue_request=dict(path=str(CONTINUE_REQUEST),sha256=CONTINUE_REQUEST_SHA,
                    source='actual_user_message',new_scene_confirmation_claimed=False),
                continuing_authorization=dict(path=str(A), sha256=AUTH_SHA),
                runtime_deployment=ref(context['deployment_path']), release=ref(context['release_path']),
                control_revision=67, control_sha256=CONTROL_SHA,
                preparation_script=ref(Path(__file__)), prepared_ns=now,
                source='current_image_review_under_existing_continuing_authorization')
            new_json(scene / 'operator_observation.json', observation)
            result.update(passed=True, observation=ref(scene / 'operator_observation.json'),
                          readiness=ref(readiness_path), review=ref(review_path), recorded_ns=now,
                          valid_until_ns=observation['valid_until_ns'], control_revision=67)
    except BaseException as exc:
        result.update(error=repr(exc), traceback=traceback.format_exc())
    finally:
        new_json(output, result)
    print(json.dumps(dict(passed=result['passed'], result=str(output), error=result.get('error')), ensure_ascii=False))
    return 0 if result['passed'] else 2


if __name__ == '__main__':
    sys.exit(main())
