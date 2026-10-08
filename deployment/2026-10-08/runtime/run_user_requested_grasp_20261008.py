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
                                 message_id=None, user_alignment_claimed=False),
        operator_authorized_autonomous_grasp=True,
        recording_scope='local_only', session_dir=str(session),
        previous_selection=prior, selection=None,
        target_basis='Current camera image: single small box near image center; existing unknown mode',
        physical_lift_after_close=False, physical_success='unverified',
        torque_disable_authorized=False, old_plan_reused=False,
        temperature_queried=False,
    )
    def save():
        (record / 'authorization.json').write_text(
            json.dumps(authorization, ensure_ascii=False, indent=2) + '\n')
    save()
    (session / 'active_record.json').write_text(json.dumps(dict(
        record_dir=str(record), launcher_pid=os.getppid(), selection_pid=os.getpid()), indent=2) + '\n')
    rospy.wait_for_service('/grasp_mode/select', timeout=5)
    assert parse_selection(rospy.get_param('/grasp_mode/selection')) == prior
    assert not rospy.wait_for_message('/grasp/state', GraspState, timeout=5).active
    response = rospy.ServiceProxy('/grasp_mode/select', SetGraspMode)('unknown', 'observe_return')
    authorization['selection_response'] = dict(success=response.success, message=response.message)
    save()
    assert response.success, response.message
    selected = None
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
        candidate = parse_selection(rospy.get_param('/grasp_mode/selection'))
        if candidate != prior:
            assert candidate['generation'] == prior['generation'] + 1
            assert candidate['stamp_ns'] > prior['stamp_ns']
            assert candidate['mode'] == 'unknown' and candidate['strategy'] == 'observe_return'
            selected = candidate
            break
        time.sleep(.05)
    assert selected is not None, 'Selection acknowledgement missing; no automatic retry'
    authorization['selection'] = selected
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
