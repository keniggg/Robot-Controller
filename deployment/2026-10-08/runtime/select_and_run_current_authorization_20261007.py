"""Record this user's explicit alignment and select one new target generation."""
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import time
import argparse
import subprocess

WORK = Path(__file__).resolve().parents[1]
MOUNT = Path('/media/zhuyupei/SanStand')


def main():
    if not MOUNT.is_mount():
        raise RuntimeError('SanStand unavailable; no fallback')
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--user-message', required=True,
                        help='Exact current user alignment instruction; never reuse an old receipt')
    parser.add_argument('--question-item-id', default=None,
                        help='Actual reply ID when supplied by chat; omit for a direct user message')
    args = parser.parse_args()
    if not args.user_message.strip():
        parser.error('--user-message must contain the current user instruction')
    out = args.output.resolve()
    record = Path(os.environ['GRASP_RECORD_DIR']).resolve()
    assert MOUNT in out.parents and MOUNT in record.parents
    assert (record / 'recorder_ready.json').is_file(), 'Full recording must be ready before selection'
    out.mkdir(exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(out / 'ros')
    import rospy
    from sensor_msgs.msg import JointState, Image
    from std_msgs.msg import Header
    from alicia_flexible_grasp_supervisor.msg import GraspState
    from alicia_grasp_modes.srv import SetGraspMode
    spec = importlib.util.spec_from_file_location('alignment_context', WORK / 'tools/diagnostics/prepare_rgb_acquisition_retry.py')
    guard = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(guard)
    rospy.init_node('current_user_alignment_selection', anonymous=True, disable_signals=True)
    captured_ns = rospy.Time.now().to_nsec()
    origin = dict(kind='user_message', text=args.user_message,
                  question_item_id=args.question_item_id,
                  message_id=None,
                  provenance='Current user alignment instruction supplied explicitly to this invocation; no chat timestamp or ID inferred')
    sample = rospy.wait_for_message('/alicia_d/accepted_joint_states', JointState, timeout=4)
    values = dict(zip(sample.name, sample.position))
    feedback = dict(stamp_ns=str(sample.header.stamp.to_nsec()), frame_id=sample.header.frame_id,
                    positions=[values[name] for name in guard.JOINT_NAMES])
    guard._sample(feedback)
    assert sample.header.stamp.to_nsec() >= captured_ns
    assert 0 <= rospy.Time.now().to_nsec() - sample.header.stamp.to_nsec() <= 200_000_000
    epoch = rospy.wait_for_message('/alicia_d/control_reference_epoch', Header, timeout=4)
    assert epoch.frame_id == 'sdk_reference_epoch'
    context = guard.retry_source_context(rospy.get_param('/grasp_mode/selection'), epoch.stamp.to_nsec(),
        rospy.get_param('/camera'), rospy.get_param('/handeye'), rospy.get_param('/robot_description'))
    event = dict(confirmation_source=origin, confirmed_ns=str(captured_ns), target_aligned=True,
                 confirmation_time_semantics='Local capture time, not claimed chat timestamp',
                 source_context=context, bound_feedback=feedback,
                 robot_commands_sent=False, torque_commands_sent=False)
    event_path = out / 'alignment_event.json'
    event_path.write_text(json.dumps(event, ensure_ascii=False, indent=2) + '\n')
    receipt = dict(version=1, kind='explicit_user_target_alignment', target_aligned=True,
                   confirmation_id='local_receipt:' + hashlib.sha256(
                       (str(out) + ':' + str(captured_ns)).encode('utf8')).hexdigest(),
                   confirmation_id_semantics='Local evidence ID, not chat transport ID',
                   confirmation_source=origin, confirmed_ns=str(captured_ns),
                   selection=context['selection'], epoch_ns=str(context['epoch_ns']),
                   bound_feedback=feedback, source_context_sha256=guard.digest(context),
                   event_file=str(event_path), event_file_sha256=hashlib.sha256(event_path.read_bytes()).hexdigest(),
                   recorded_at_utc=datetime.now(timezone.utc).isoformat())
    receipt_path = out / 'alignment_receipt.json'
    receipt_path.write_text(json.dumps(receipt, ensure_ascii=False, indent=2) + '\n')
    assert not rospy.wait_for_message('/grasp/state', GraspState, timeout=4).active
    assert not rospy.get_param('/rgb_acquisition/ledger', {})
    assert rospy.get_param('/grasp/unknown_lift_after_close_enabled') is False
    direct_mode = rospy.get_param('/gui/joint_direct_mode')
    assert type(direct_mode) is bool
    raw_path = out / 'raw_support'
    raw_log = (out / 'raw_support.log').open('w')
    raw = subprocess.Popen(['/usr/bin/python3', str(WORK / '.runtime/record_support_gap_15s.py'),
        '--output', str(raw_path), '--seconds', '15'], stdout=raw_log, stderr=subprocess.STDOUT)
    ready_deadline = time.monotonic() + 6
    while not (raw_path / 'ready.json').is_file():
        assert raw.poll() is None and time.monotonic() < ready_deadline, 'Raw recorder not ready'
        time.sleep(.05)
    ready = json.loads((raw_path / 'ready.json').read_text())
    assert ready['initial_generation'] == context['selection']['generation']
    assert ready['initial_selection_stamp_ns'] == context['selection']['stamp_ns']
    rospy.wait_for_service('/grasp_mode/select', timeout=4)
    current = rospy.wait_for_message('/alicia_d/accepted_joint_states', JointState, timeout=4)
    current_values = dict(zip(current.name, current.position))
    guard._sample(dict(stamp_ns=str(current.header.stamp.to_nsec()), frame_id=current.header.frame_id,
        positions=[current_values[name] for name in guard.JOINT_NAMES]))
    assert current.header.frame_id == 'sdk_measured'
    assert 0 <= rospy.Time.now().to_nsec() - current.header.stamp.to_nsec() <= 200_000_000
    assert rospy.get_param('/gui/joint_direct_mode') is direct_mode
    assert not rospy.wait_for_message('/grasp/state', GraspState, timeout=4).active
    assert max(abs(current_values[name] - q) for name, q in zip(guard.JOINT_NAMES, feedback['positions'])) <= guard.POSE_TOLERANCE_RAD
    latest_epoch = rospy.wait_for_message('/alicia_d/control_reference_epoch', Header, timeout=4)
    current_context = guard.retry_source_context(rospy.get_param('/grasp_mode/selection'), latest_epoch.stamp.to_nsec(),
        rospy.get_param('/camera'), rospy.get_param('/handeye'), rospy.get_param('/robot_description'))
    assert guard.digest(current_context) == receipt['source_context_sha256']
    transition = dict(path=str(out), preview_only=False, alignment_receipt=receipt,
        alignment_receipt_file=str(receipt_path), alignment_receipt_sha256=hashlib.sha256(receipt_path.read_bytes()).hexdigest(),
        previous_selection=context['selection'], previous_ledger=None, new_selection=None,
        reason='New explicit user authorization after prior target expired; fresh generation only',
        record_dir=str(record), raw_recorder_pid=raw.pid,
        selection_kind='explicit_current_alignment_close_hold', motion_commands_sent=False, torque_commands_sent=False)
    try:
        assert not rospy.get_param('/rgb_acquisition/ledger', {})
        assert rospy.get_param('/grasp/unknown_lift_after_close_enabled') is False
        assert raw.poll() is None and time.monotonic() < ready['deadline_monotonic'] - 8
        response = rospy.ServiceProxy('/grasp_mode/select', SetGraspMode)('unknown', 'observe_return')
        transition['response'] = dict(success=response.success, message=response.message)
        assert response.success, response.message
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            new = guard.parse_selection(rospy.get_param('/grasp_mode/selection'))
            if new['generation'] == context['selection']['generation'] + 1 and new['stamp_ns'] > context['selection']['stamp_ns']:
                assert new['mode'] == 'unknown' and new['strategy'] == 'observe_return'
                transition['new_selection'] = new
                break
            time.sleep(.05)
        assert transition['new_selection'] is not None, 'Selection acknowledgement missing; no retry'
    except Exception as exc:
        transition['failure'] = repr(exc)
        raise
    finally:
        path = out / 'new_target_selection.json'
        path.write_text(json.dumps(transition, ensure_ascii=False, indent=2) + '\n')
        (WORK / '.runtime/current_aligned_grasp_20261007.json').write_text(json.dumps(
            dict(directory=str(out), selection_result=str(path), selection=transition['new_selection']), indent=2) + '\n')
        print(json.dumps(dict(path=str(path), selection=transition['new_selection'], failure=transition.get('failure')), ensure_ascii=False), flush=True)
    # Immediate handoff of the new selection; the old target never authorizes motion.
    result = subprocess.call(['/usr/bin/python3', '-u', str(WORK / '.runtime/run_confirmed_unknown_grasp_20261004.py'),
        '--selection-result', str(path)], cwd=str(WORK))
    raw.wait()
    raw_log.close()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
