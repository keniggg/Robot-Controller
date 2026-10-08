"""Record this user's explicit alignment and select one new target generation."""
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import time

WORK = Path(__file__).resolve().parents[1]
MOUNT = Path('/media/zhuyupei/SanStand')


def main():
    if not MOUNT.is_mount():
        raise RuntimeError('SanStand unavailable; no fallback')
    out = MOUNT / '实验记录/Robot-Controller/ros_log' / (
        'aligned_grasp_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'))
    out.mkdir()
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
    origin = dict(kind='user_message', text='已经对准了目标物',
                  message_id=None,
                  provenance='Current chat user message; transport message ID not provided')
    sample = rospy.wait_for_message('/alicia_d/accepted_joint_states', JointState, timeout=4)
    values = dict(zip(sample.name, sample.position))
    feedback = dict(stamp_ns=str(sample.header.stamp.to_nsec()), frame_id=sample.header.frame_id,
                    positions=[values[name] for name in guard.JOINT_NAMES])
    guard._sample(feedback)
    assert sample.header.stamp.to_nsec() >= captured_ns
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
                   confirmation_id='local_receipt:' + out.name,
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
    assert rospy.get_param('/gui/joint_direct_mode') is True
    current = rospy.wait_for_message('/alicia_d/accepted_joint_states', JointState, timeout=4)
    current_values = dict(zip(current.name, current.position))
    assert current.header.frame_id == 'sdk_measured'
    assert max(abs(current_values[name] - q) for name, q in zip(guard.JOINT_NAMES, feedback['positions'])) <= guard.POSE_TOLERANCE_RAD
    latest_epoch = rospy.wait_for_message('/alicia_d/control_reference_epoch', Header, timeout=4)
    current_context = guard.retry_source_context(rospy.get_param('/grasp_mode/selection'), latest_epoch.stamp.to_nsec(),
        rospy.get_param('/camera'), rospy.get_param('/handeye'), rospy.get_param('/robot_description'))
    assert guard.digest(current_context) == receipt['source_context_sha256']
    transition = dict(path=str(out), preview_only=False, alignment_receipt=receipt,
        alignment_receipt_file=str(receipt_path), alignment_receipt_sha256=hashlib.sha256(receipt_path.read_bytes()).hexdigest(),
        previous_selection=context['selection'], previous_ledger=None, new_selection=None,
        reason='Current explicit user alignment after manual slider repair and new control epoch',
        selection_kind='explicit_current_alignment_close_hold', motion_commands_sent=False, torque_commands_sent=False)
    try:
        rospy.wait_for_service('/grasp_mode/select', timeout=4)
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
    # Raw display evidence only. This snapshot is not a reusable plan/source.
    from cv_bridge import CvBridge
    import cv2
    message = rospy.wait_for_message('/supervisor/camera/color/image_raw', Image, timeout=4)
    cv2.imwrite(str(out / 'current_rgb.png'), CvBridge().imgmsg_to_cv2(message, 'bgr8'))
    (out / 'current_rgb.json').write_text(json.dumps(dict(stamp_ns=message.header.stamp.to_nsec(), frame_id=message.header.frame_id)) + '\n')


if __name__ == '__main__':
    main()
