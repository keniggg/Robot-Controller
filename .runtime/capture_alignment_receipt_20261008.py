"""Record an actual user alignment reply and fresh input; no robot commands."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import threading
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--message-id', required=True)
    parser.add_argument('--user-message', default='已对准，可以继续自动抓取')
    parser.add_argument('--prior-record', required=True)
    parser.add_argument('--session-root', type=Path, default=Path(
        '/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z'))
    args = parser.parse_args()
    mount = Path('/media/zhuyupei/SanStand')
    root = args.session_root.resolve()
    assert mount.is_mount() and args.message_id.strip()
    assert (mount / '实验记录/Robot-Controller/ros_log').resolve() in root.parents
    assert (root / 'current_progress.json').is_file()
    out = root / ('realigned_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    out.mkdir(exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(out / 'capture_ros')
    import cv2
    import numpy as np
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image, JointState
    from std_msgs.msg import Header, String
    rospy.init_node('capture_actual_alignment_reply', anonymous=True, disable_rosout=True)
    start = rospy.Time.now().to_nsec()
    lock = threading.Lock()
    pairs, latest = {}, {}

    def image(kind, msg):
        with lock:
            ns = msg.header.stamp.to_nsec()
            if ns < start:
                return
            pairs.setdefault(ns, {})[kind] = msg
            if len(pairs[ns]) == 2:
                latest.update(pair=pairs[ns], stamp=ns)
            for old in sorted(pairs)[:-6]:
                pairs.pop(old, None)

    subscribers = [rospy.Subscriber(topic, Image, lambda msg, k=kind: image(k, msg),
        queue_size=2, buff_size=2**24) for kind, topic in (
            ('color', '/supervisor/camera/color/image_raw'),
            ('depth', '/supervisor/camera/depth/image_raw'))]
    try:
        deadline = time.monotonic() + 6.
        while not latest and time.monotonic() < deadline:
            time.sleep(.05)
        assert latest, 'No new exact RGB-D pair'
        q = rospy.wait_for_message('/alicia_d/accepted_joint_states', JointState, timeout=3)
        epoch = rospy.wait_for_message('/alicia_d/control_reference_epoch', Header, timeout=3)
        status = rospy.wait_for_message('/alicia_d/actuation_status', String, timeout=3).data
        with lock:
            pair = dict(latest)
        assert q.header.frame_id == 'sdk_measured'
        assert 0 <= (rospy.Time.now() - q.header.stamp).to_sec() < .3
        assert 0 <= (rospy.Time.now().to_nsec() - pair['stamp']) / 1e9 < .8
        receipt = dict(user_message=args.user_message, message_id=args.message_id,
            receipt_ns=rospy.Time.now().to_nsec(), image_stamp_ns=pair['stamp'],
            actuation_status=status, epoch_ns=epoch.stamp.to_nsec(),
            selection=rospy.get_param('/grasp_mode/selection'),
            topics={'/alicia_d/accepted_joint_states': dict(
                stamp_ns=q.header.stamp.to_nsec(), frame_id=q.header.frame_id,
                names=list(q.name), positions=list(q.position))},
            previous_alignment_reused=False, prior_automatic_record=args.prior_record)
        for name in ('alignment_receipt.json', 'preflight.json'):
            (out / name).write_text(json.dumps(receipt, ensure_ascii=False, indent=2) + '\n')
        bridge = CvBridge()
        assert cv2.imwrite(str(out / 'aligned_scene.png'),
            bridge.imgmsg_to_cv2(pair['pair']['color'], 'bgr8'))
        np.savez_compressed(out / 'aligned_depth.npz',
            depth=bridge.imgmsg_to_cv2(pair['pair']['depth'], 'passthrough'), stamp_ns=pair['stamp'])
        (out / 'camera_configuration.json').write_text(json.dumps(
            rospy.get_param('/camera'), indent=2) + '\n')
        work = Path(__file__).resolve().parents[1]
        index = dict(session_dir=str(out), alignment_receipt=str(out / 'alignment_receipt.json'),
                     previous_alignment_reused=False)
        (work / '.runtime/current_realigned_20261008.json').write_text(
            json.dumps(index, ensure_ascii=False, indent=2) + '\n')
        for path in (root / 'current_progress.json', work / '.runtime/latest_full_ros_start.json'):
            data = json.loads(path.read_text())
            data.update(alignment_receipt=index['alignment_receipt'],
                        awaiting_alignment_confirmation=False, awaiting_user_alignment=False,
                        manual_reposition_requested=False, manual_adjustment_requested=False,
                        new_alignment_confirmation_received=True,
                        current_alignment_pose_is_current=True,
                        alignment_message_id=receipt['message_id'],
                        current_alignment_received_ns=receipt['receipt_ns'],
                        current_alignment_positions_rad=list(q.position),
                        alignment_consumed_by_record=None,
                        alignment_control_state='user_alignment_received',
                        alignment_positions_are_historical_start_measurement=False,
                        automatic_grasp_running=False, automatic_grasp_started=False,
                        updated_ns=receipt['receipt_ns'])
            path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps(index, ensure_ascii=False), flush=True)
        print(json.dumps(receipt, ensure_ascii=False), flush=True)
    finally:
        for subscriber in subscribers:
            subscriber.unregister()


if __name__ == '__main__':
    main()
