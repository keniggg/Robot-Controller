"""Capture a fresh scene and strictly read current gateway-reloaded-stack readiness; no motion."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import traceback

from recorded_attempt_common import (
    ACTION_OWNER, CAPTURE, CAPTURE_SHA, DRIVER, GUI, W, input_receipts, live_identity,
    load_context, new_json, parser, recheck, ref, require, same_selection, selection_identity, sha, storage)


def main():
    p = parser(__doc__)
    p.add_argument('--scene', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    context = load_context(a, __file__)
    S = context['session_root']
    output, scene = storage(a.output), storage(a.scene)
    require(output.parent == scene.parent == S and scene.name.startswith('scene_start_') and
            output.name.startswith('readiness_start_'), 'fresh scene/readiness roots required')
    require(not output.exists() and not scene.exists(), 'no implicit scene or readiness retry')
    output.mkdir()
    os.environ['ROS_LOG_DIR'] = str(output / 'observer_ros')
    result = dict(kind='current_bound_current_runtime17_scene_and_actuation_readiness', passed=False,
                  started_ns=time.time_ns(), helper_path=str(Path(__file__).resolve()),
                  helper_sha256=sha(__file__), scene=str(scene),
                  motion_commands_sent=False, temperature_queried=False,
                  torque_enable_commands_sent=False, joint_disable_commands_sent=False,
                  new_grasp_authorized=False, scene_authorized=False,
                  state_receipt_time_does_not_prove_new_latched_message_generation=True)
    subscriptions = []
    try:
        result.update(input_sha256=input_receipts(context),
                      source_sha256=context['release']['source_sha256'])
        import rosgraph
        import rospy
        from actionlib_msgs.msg import GoalStatusArray
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Header, String
        from alicia_flexible_grasp_supervisor.msg import GraspState
        rospy.init_node('current_bound_attempt_readonly', anonymous=True, disable_rosout=True)
        master = rosgraph.Master(rospy.get_name())
        before = live_identity(context, rospy, master)
        require(rospy.get_param('/gui/joint_direct_mode') is True, 'not manual before scene capture')
        require(sha(CAPTURE) == CAPTURE_SHA, 'original capture changed')
        with (output / 'scene_capture.log').open('x') as log:
            completed = subprocess.run([sys.executable, '-B', str(CAPTURE), '--output', str(scene)],
                                       cwd=W, stdout=log, stderr=subprocess.STDOUT, timeout=20.,
                                       env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1'))
        result['scene_capture_returncode'] = completed.returncode
        require(completed.returncode == 0, 'original fresh scene capture failed')
        pre = json.loads((scene / 'preflight.json').read_text())
        result['scene_preflight'] = ref(scene / 'preflight.json')
        result['image'] = ref(scene / 'current_scene.png')
        expected_epoch, expected_selection = context['deployment']['epoch_ns'], context['deployment']['selection']
        require(pre['epoch_ns'] == expected_epoch and same_selection(pre['selection'], expected_selection) and
                pre['manual_mode'] is True and pre['task_active'] is False,
                'scene differs from accepted runtime epoch/selection; a changed manual reference requires independent fresh binding')
        lock, latest, history = threading.Lock(), {}, []
        topics = [
            ('joint', '/alicia_d/accepted_joint_states', JointState, DRIVER),
            ('epoch', '/alicia_d/control_reference_epoch', Header, DRIVER),
            ('task', '/grasp/state', GraspState, '/grasp_task_node'),
            ('mode', '/grasp_mode/status', String, '/grasp_mode_router'),
            ('manual', '/gui/joint_direct_mode', Bool, GUI),
            ('arm', '/alicia_controller/follow_joint_trajectory/status', GoalStatusArray, ACTION_OWNER),
            ('hand', '/hand_controller/follow_joint_trajectory/status', GoalStatusArray, ACTION_OWNER),
            ('feedback_ready', '/alicia_d/feedback_ready', Bool, DRIVER),
            ('actuation', '/alicia_d/actuation_status', String, DRIVER),
            ('motion_enabled', '/alicia_d/motion_enabled', Bool, DRIVER),
        ]

        def receive(key, message):
            now = rospy.Time.now().to_nsec()
            with lock:
                latest[key] = (message, now)
                if key == 'joint':
                    history.append(dict(stamp_ns=message.header.stamp.to_nsec(), received_ns=now,
                                        frame_id=message.header.frame_id, names=list(message.name),
                                        positions=list(message.position),
                                        callerid=message._connection_header.get('callerid')))

        subscriptions.extend(rospy.Subscriber(topic, kind, lambda m, k=key: receive(k, m), queue_size=40)
                             for key, topic, kind, _ in topics)
        deadline = time.monotonic() + 8.
        while time.monotonic() < deadline:
            with lock:
                if len(latest) == len(topics) and len(history) >= 20:
                    break
            time.sleep(.025)
        after = live_identity(context, rospy, master)
        require(before == after, 'runtime17/GUI/publisher identities changed during scene capture')
        space = recheck(context)
        # These are new receipts of current latches, not claimed new generations.
        for key, topic, kind, _ in topics:
            if key not in ('joint', 'arm', 'hand'):
                receive(key, rospy.wait_for_message(topic, kind, timeout=3.))
        with lock:
            state, samples = dict(latest), list(history[-20:])
        now = rospy.Time.now().to_nsec()
        result.update(recorded_ns=now, node_identities_before=before['nodes'],
                      node_identities_after=after['nodes'], identity_evidence_before=before,
                      identity_evidence_after=after, samples=samples, sample_count=len(samples),
                      state_received_ns={k: t for k, (_, t) in state.items()}, **space)
        require(len(state) == len(topics) and len(samples) >= 20, 'incomplete current observations')
        # Preserve factual execution state before rejecting UNCONFIRMED or false.
        result.update(actuation_status=state['actuation'][0].data,
                      motion_enabled=state['motion_enabled'][0].data,
                      feedback_ready=state['feedback_ready'][0].data,
                      manual_mode=state['manual'][0].data)
        for key, _, _, owner in topics:
            if owner is not None:
                require(state[key][0]._connection_header.get('callerid') == owner,
                        'callback publisher differs: ' + key)
        require(all(0 <= now - t < 1_000_000_000 for k, (_, t) in state.items() if k != 'joint'),
                'current state receipt is stale')
        require(rospy.get_param('/gui/joint_direct_mode') is True and state['manual'][0].data is True,
                'current GUI own manual acknowledgement missing')
        mode, task = json.loads(state['mode'][0].data), state['task'][0]
        require(not task.active and not any(mode.get(k) for k in ('active', 'pending', 'start_reserved')),
                'task/router busy')
        for key in ('arm', 'hand'):
            message = state[key][0]
            require(0 <= now - message.header.stamp.to_nsec() <= 2_000_000_000 and
                    not any(g.status in (0, 1, 6, 7) for g in message.status_list), 'action active or heartbeat stale')
        require(state['epoch'][0].stamp.to_nsec() == expected_epoch and
                same_selection(rospy.get_param('/grasp_mode/selection'), expected_selection), 'epoch/selection changed')
        require(rospy.get_param('/grasp/require_actuation_confirmation') is True,
                'actuation confirmation policy is not enabled')
        require(state['feedback_ready'][0].data is True, 'FEEDBACK_NOT_READY')
        require(state['motion_enabled'][0].data is True, 'MOTION_NOT_ENABLED')
        require(state['actuation'][0].data.startswith('CONFIRMED:'),
                'ACTUATION_UNCONFIRMED: ' + str(state['actuation'][0].data))
        names = samples[-1]['names']
        require(len(names) == len(set(names)) and {'Joint%d' % i for i in range(1, 7)} <= set(names),
                'SDK joint layout invalid')
        stamps = [s['stamp_ns'] for s in samples]
        require(len(set(stamps)) >= 20 and all(b > a for a, b in zip(stamps, stamps[1:])),
                '20 unique increasing SDK samples required')
        require(all(s['frame_id'] == 'sdk_measured' and s['callerid'] == DRIVER and s['names'] == names and
                    len(s['positions']) == len(names) and all(math.isfinite(q) for q in s['positions']) and
                    expected_epoch <= s['stamp_ns'] <= s['received_ns'] and
                    s['received_ns'] - s['stamp_ns'] < 200_000_000 for s in samples), 'SDK source/layout/age invalid')
        require(0 <= now - stamps[-1] < 200_000_000, 'latest SDK sample stale')
        original = pre['topics']['/alicia_d/accepted_joint_states']
        require(original['frame_id'] == 'sdk_measured' and original['names'] == names and
                len(original['positions']) == len(names) and all(math.isfinite(q) for q in original['positions']) and
                0 <= pre['receipt_ns'] - original['stamp_ns'] <= 200_000_000 and
                0 <= pre['receipt_ns'] - pre['image_stamp_ns'] <= 800_000_000, 'captured SDK/image invalid')
        spans = {n: max(s['positions'][i] for s in samples) - min(s['positions'][i] for s in samples)
                 for i, n in enumerate(names)}
        maximum = max(abs(s['positions'][i] - original['positions'][i]) for s in samples for i in range(len(names)))
        require(max(spans.values()) <= .004 and maximum <= .004, 'SDK not stationary at captured pose')
        require(rospy.get_param('/alicia_d_driver_node/software_temperature_monitoring_enabled') is False and
                rospy.get_param('/alicia_d_driver_node/temperature_poll_rate_hz') == 0.0 and
                rospy.get_param('/grasp/unknown_lift_after_close_enabled') is False and
                rospy.get_param('/grasp/unknown_observe_return_max_observations') == 1,
                'temperature/lift/observation policy changed')
        result.update(passed=True, source_count=len(context['release']['source_sha256']),
                      deployment_processes_unchanged=True, source_matches_operational_release=True,
                      epoch_ns=expected_epoch, selection=pre['selection'],
                      selection_identity=selection_identity(pre['selection']), names=names,
                      positions=samples[-1]['positions'], unique_sample_count=len(set(stamps)),
                      maximum_joint_delta_rad=maximum, sample_spans=spans, task_active=False,
                      task_state=task.state, router_active=False, actions_active=False,
                      gui_owning_manual_ack=True, require_actuation_confirmation=True,
                      scene_preflight_sha256=sha(scene / 'preflight.json'),
                      image_sha256=sha(scene / 'current_scene.png'), release=ref(context['release_path']),
                      deployment=ref(context['deployment_path']))
    except BaseException as exc:
        result.update(error=repr(exc), traceback=traceback.format_exc())
    finally:
        for subscription in subscriptions:
            subscription.unregister()
        new_json(output / 'result.json', result)
    print(json.dumps(dict(passed=result['passed'], output=str(output),
                         actuation_status=result.get('actuation_status'), error=result.get('error')), ensure_ascii=False))
    return 0 if result['passed'] else 2


if __name__ == '__main__':
    sys.exit(main())
