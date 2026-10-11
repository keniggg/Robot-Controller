"""Select once and use the existing audited runner for an explicit grasp request.

This records the actual request without claiming the user confirmed alignment.
All experiment evidence belongs to the already running SanStand recorder.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def wait_for_selection_commit(rospy, prior, timeout_sec=8.0):
    """Wait for the router's completed transaction, not its early parameter.

    The router publishes the latched selection only after both reset ACKs.
    Its matching status supplies either commit completion or the reset error.
    This waits once within the existing selection deadline; it never retries
    selection, computation, or motion and does not require target detection.
    """
    import threading
    from std_msgs.msg import String
    from alicia_grasp_modes.selection import parse_selection

    lock = threading.Lock()
    received = {}
    subscriptions = []
    deadline = time.monotonic() + timeout_sec

    def receive(name, message):
        try:
            payload = json.loads(message.data)
            selection = parse_selection(payload)
        except (TypeError, ValueError):
            return
        with lock:
            received[name] = (selection, payload)

    selected = None
    try:
        for name in ('selection', 'status'):
            subscriptions.append(rospy.Subscriber(
                '/grasp_mode/' + name, String,
                lambda message, topic=name: receive(topic, message), queue_size=2))
        while time.monotonic() < deadline and not rospy.is_shutdown():
            candidate = parse_selection(rospy.get_param('/grasp_mode/selection'))
            if candidate != prior:
                assert candidate['generation'] == prior['generation'] + 1
                assert candidate['stamp_ns'] > prior['stamp_ns']
                assert candidate['mode'] == 'unknown' and candidate['strategy'] == 'observe_return'
                assert selected is None or candidate == selected, 'Selection changed during acknowledgement'
                selected = candidate
            elif selected is not None:
                raise RuntimeError('Selection changed during acknowledgement')
            with lock:
                committed = received.get('selection')
                status = received.get('status')
            if selected is not None and status is not None and status[0] == selected:
                if status[1].get('state') == 'error':
                    raise RuntimeError('Selection reset failed: ' + str(status[1].get('detail', '')))
                if (committed is not None and committed[0] == selected
                        and status[1].get('state') in ('waiting_for_target', 'ready')):
                    # Recheck the exact parameter before releasing the runner;
                    # its existing expected-selection guard also checks on entry.
                    assert parse_selection(rospy.get_param('/grasp_mode/selection')) == selected
                    if time.monotonic() >= deadline or rospy.is_shutdown():
                        break
                    return selected, dict(status[1])
            time.sleep(.05)
        raise RuntimeError('Selection commit acknowledgement missing; no automatic retry')
    finally:
        for subscription in subscriptions:
            subscription.unregister()


def main():
    import rospy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from alicia_flexible_grasp_supervisor.msg import GraspState
    from alicia_grasp_modes.selection import parse_selection
    from alicia_grasp_modes.srv import SetGraspMode

    parser = argparse.ArgumentParser()
    parser.add_argument('--session-dir', type=Path, required=True)
    parser.add_argument('--user-message', required=True)
    args = parser.parse_args()
    mount = Path('/media/zhuyupei/SanStand')
    record = Path(os.environ['GRASP_RECORD_DIR']).resolve()
    session = args.session_dir.resolve()
    assert mount.is_mount() and mount in record.parents and mount in session.parents
    assert args.user_message.strip()
    assert (record / 'recorder_ready.json').is_file()
    rospy.init_node('user_requested_grasp_selection', anonymous=True, disable_signals=True)
    receipt = json.loads((session/'alignment_receipt.json').read_text())
    assert receipt['user_message'] in ('已对准', '已对准，可以继续自动抓取', '已经对准了目标物，你可以执行了', '已经对准了目标物', '已放近并对准，可以继续抓取', '盒子已在画面中央，可以继续', '已对准，手已移开，可以继续自动抓取', '已重启并对准，可以继续自动抓取', '已经重新调整完毕并且对准了')
    assert args.user_message == receipt['user_message']
    from std_msgs.msg import Bool
    control = rospy.Publisher('/gui/joint_direct_mode', Bool, queue_size=1, latch=True)
    deadline = time.monotonic()+5.
    while control.get_num_connections() == 0 and time.monotonic() < deadline: time.sleep(.05)
    assert control.get_num_connections() > 0
    rospy.set_param('/gui/joint_direct_mode', False)
    control.publish(Bool(data=False))
    time.sleep(.4)
    (record/'alignment_receipt.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n')
    prior = parse_selection(rospy.get_param('/grasp_mode/selection'))
    assert prior['mode'] == 'unknown' and prior['strategy'] == 'observe_return'
    assert not rospy.wait_for_message('/grasp/state', GraspState, timeout=5).active
    mode = json.loads(rospy.wait_for_message('/grasp_mode/status', String, timeout=5).data)
    assert not mode.get('active') and not mode.get('start_reserved') and not mode.get('pending')
    assert rospy.get_param('/gui/joint_direct_mode') is False
    assert not rospy.get_param('/rgb_acquisition/ledger', {})
    assert rospy.get_param('/grasp/unknown_lift_after_close_enabled') is False
    # Read policy switches only; never query or subscribe to temperature values.
    assert rospy.get_param('/alicia_d_driver_node/software_temperature_monitoring_enabled') is False
    assert rospy.get_param('/alicia_d_driver_node/temperature_poll_rate_hz') == 0.0
    measured = rospy.wait_for_message('/alicia_d/accepted_joint_states', JointState, timeout=5)
    assert measured.header.frame_id == 'sdk_measured'
    assert 0 <= (rospy.Time.now() - measured.header.stamp).to_sec() <= .2
    preflight = json.loads((session / 'preflight.json').read_text())
    before = preflight['topics']['/alicia_d/accepted_joint_states']
    current = dict(zip(measured.name, measured.position))
    assert all(abs(current[name] - value) <= .02
               for name, value in zip(before['names'], before['positions']))
    authorization = dict(
        confirmation_source=dict(kind='user_message', text=args.user_message,
                                 message_id=receipt['message_id'], user_alignment_claimed=True),
        operator_authorized_autonomous_grasp=True,
        recording_scope='local_only', session_dir=str(session),
        previous_selection=prior, selection=None,
        target_basis='Current user-confirmed aligned image; new measured unknown target selection',
        physical_lift_after_close=False, physical_success='unverified',
        torque_disable_authorized=False, old_plan_reused=False,
        temperature_queried=False,
    )
    def save():
        (record / 'authorization.json').write_text(
            json.dumps(authorization, ensure_ascii=False, indent=2) + '\n')
    save()
    parameter_start_ns = rospy.Time.now().to_nsec()
    # Preserve the actual policy used by this attempt for later offline replay.
    # These are computation settings only; no device telemetry is queried.
    parameters = {name: rospy.get_param(name) for name in (
        '/unknown_perception', '/grasp', '/grasp_6d/remote/multiview')}
    (record / 'perception_execution_configuration.json').write_text(json.dumps(dict(
        source='ROS parameter server before new selection',
        read_start_ns=parameter_start_ns, read_end_ns=rospy.Time.now().to_nsec(),
        parameters=parameters, image_synchronized=False,
        motion_authority=False), ensure_ascii=False, indent=2) + '\n')
    (session / 'active_record.json').write_text(json.dumps(dict(
        record_dir=str(record), launcher_pid=os.getppid(), selection_pid=os.getpid()), indent=2) + '\n')
    rospy.wait_for_service('/grasp_mode/select', timeout=5)
    assert parse_selection(rospy.get_param('/grasp_mode/selection')) == prior
    assert not rospy.wait_for_message('/grasp/state', GraspState, timeout=5).active
    response = rospy.ServiceProxy('/grasp_mode/select', SetGraspMode)('unknown', 'observe_return')
    authorization['selection_response'] = dict(success=response.success, message=response.message)
    save()
    assert response.success, response.message
    selected, commit_status = wait_for_selection_commit(rospy, prior)
    authorization['selection'] = selected
    authorization['selection_commit'] = dict(
        latched_selection_matched=True, router_status=commit_status)
    save()
    print('USER_REQUESTED_GRASP ' + json.dumps(authorization, ensure_ascii=False), flush=True)
    command = ['rosrun', 'alicia_grasp_modes', 'mode_aware_grasp_runner.py',
               '--mode', 'unknown', '--strategy', 'observe_return',
               '--expected-selection-json', json.dumps(selected, sort_keys=True)]
    result = subprocess.call(command)
    (record / 'user_request_runner_exit.json').write_text(json.dumps(dict(
        runner_returncode=result, selection=selected, physical_success='unverified'), indent=2) + '\n')
    return result


if __name__ == '__main__':
    sys.exit(main())
