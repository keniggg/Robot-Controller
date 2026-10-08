"""Archive the actual quiet-scene reply for camera-only diagnosis, never alignment."""
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import time

W = Path(__file__).resolve().parents[1]
MOUNT = Path('/media/zhuyupei/SanStand')
EXT = MOUNT/'实验记录/Robot-Controller'


def main():
    assert MOUNT.is_mount(), 'SanStand unavailable; no fallback'
    out = EXT/'ros_log'/('stationary_reply_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    out.mkdir()
    os.environ['ROS_LOG_DIR'] = str(out/'ros')
    import rospy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header
    spec = importlib.util.spec_from_file_location('static_receipt_sample', W/'tools/diagnostics/prepare_rgb_acquisition_retry.py')
    guard = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(guard)
    rospy.init_node('record_actual_stationary_capture_reply', anonymous=True, disable_signals=True)
    recorded = rospy.Time.now().to_nsec()
    deadline = time.monotonic()+5
    while True:
        msg = rospy.wait_for_message('/alicia_d/accepted_joint_states', JointState, timeout=3)
        if msg.header.frame_id == 'sdk_measured' and msg.header.stamp.to_nsec() >= recorded:
            values = dict(zip(msg.name, msg.position))
            feedback = dict(stamp_ns=str(msg.header.stamp.to_nsec()), frame_id='sdk_measured',
                            positions=[values[n] for n in guard.JOINT_NAMES])
            guard._sample(feedback)
            guard.require_fresh_source(msg.header.stamp.to_nsec(), rospy.Time.now().to_nsec())
            break
        assert time.monotonic() < deadline, 'no fresh measured joints'
    epoch = rospy.wait_for_message('/alicia_d/control_reference_epoch', Header, timeout=3)
    assert epoch.frame_id == 'sdk_reference_epoch'
    assert not rospy.get_param('/gui/joint_direct_mode', True), 'manual control active'
    common = dict(kind='explicit_user_stationary_capture', scene_stationary=True,
        camera_diagnostic_only=True, motion_authorized=False,
        confirmation_id='call_dae6a28c0c4a44b3a064c5b6c4cc5fb0:0',
        confirmation_source=dict(kind='user_message', message_id=json.dumps(
            ['request_user_input_async', 'call_dae6a28c0c4a44b3a064c5b6c4cc5fb0', 0]),
            text='可以，现场已保持静止'),
        confirmed_ns=str(recorded), bound_feedback=feedback, epoch_ns=str(epoch.stamp.to_nsec()),
        confirmation_time_semantics='host receipt-recording time after the actual user reply; not the chat transport timestamp',
        scope='Stationary RGB-D diagnosis only; no target alignment, identity, geometry or motion authority',
        robot_commands_sent=False, torque_commands_sent=False)
    assert 0 < int(common['epoch_ns']) <= recorded <= int(feedback['stamp_ns'])
    event = dict(common, context_diagnostic_only=dict(selection=rospy.get_param('/grasp_mode/selection')))
    event_path = out/'stationary_event.json'
    event_path.write_text(json.dumps(event, ensure_ascii=False, indent=2)+'\n')
    receipt = dict(common, event_file=str(event_path),
                   event_file_sha256=hashlib.sha256(event_path.read_bytes()).hexdigest())
    path = out/'stationary_receipt.json'
    path.write_text(json.dumps(receipt, ensure_ascii=False, indent=2)+'\n')
    index = dict(receipt=str(path), confirmation_id=common['confirmation_id'])
    (W/'.runtime/current_stationary_capture_receipt_20261007.json').write_text(json.dumps(index, indent=2)+'\n')
    print(json.dumps(index, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
