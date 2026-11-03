"""Capture current RGB-D and measured pose; never claim a new user reply."""
import argparse
import json
import os
from pathlib import Path
import threading
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    storage = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
    out = args.output.resolve()
    assert Path('/media/zhuyupei/SanStand').is_mount()
    assert (storage / 'ros_log') in out.parents
    out.mkdir(exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(out / 'capture_ros')
    import cv2
    import numpy as np
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image, JointState
    from std_msgs.msg import Header
    from alicia_flexible_grasp_supervisor.msg import GraspState

    rospy.init_node('capture_continuing_grasp_scene', anonymous=True, disable_rosout=True)
    started_ns = rospy.Time.now().to_nsec()
    lock = threading.Lock()
    pairs, latest = {}, {}

    def receive(kind, message):
        with lock:
            if kind not in ('color', 'depth'):
                latest[kind] = message
                return
            stamp = message.header.stamp.to_nsec()
            if stamp < started_ns:
                return
            pairs.setdefault(stamp, {})[kind] = message
            if len(pairs[stamp]) == 2:
                latest['pair'] = dict(pairs[stamp])
                latest['image_stamp_ns'] = stamp
            for old in sorted(pairs)[:-6]:
                del pairs[old]

    topics = (
        ('color', '/supervisor/camera/color/image_raw', Image),
        ('depth', '/supervisor/camera/depth/image_raw', Image),
        ('q', '/alicia_d/accepted_joint_states', JointState),
        ('epoch', '/alicia_d/control_reference_epoch', Header),
        ('task', '/grasp/state', GraspState),
    )
    subscriptions = [rospy.Subscriber(topic, kind,
        lambda message, key=key: receive(key, message), queue_size=4,
        buff_size=2**24) for key, topic, kind in topics]
    try:
        deadline = time.monotonic() + 8.
        captured = None
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with lock:
                if all(key in latest for key in ('pair', 'q', 'epoch', 'task')):
                    now = rospy.Time.now().to_nsec()
                    q = latest['q']
                    if (0 <= now - q.header.stamp.to_nsec() <= 200_000_000
                            and 0 <= now - latest['image_stamp_ns'] <= 800_000_000):
                        captured = dict(latest)
                        captured['receipt_ns'] = now
                        break
            time.sleep(.02)
        assert captured is not None, 'No fresh exact RGB-D and SDK feedback'
        assert not captured['task'].active, 'Task still active'
        q = captured['q']
        assert q.header.frame_id == 'sdk_measured'
        result = dict(receipt_ns=captured['receipt_ns'],
            image_stamp_ns=captured['image_stamp_ns'],
            epoch_ns=captured['epoch'].stamp.to_nsec(),
            selection=rospy.get_param('/grasp_mode/selection'),
            manual_mode=rospy.get_param('/gui/joint_direct_mode'),
            task_active=False, task_state=str(captured['task'].state),
            topics={'/alicia_d/accepted_joint_states': dict(
                stamp_ns=q.header.stamp.to_nsec(), frame_id=q.header.frame_id,
                names=list(q.name), positions=list(q.position))},
            new_user_confirmation_claimed=False, motion_commands_sent=False,
            temperature_queried=False, operator_image_review_pending=True)
        bridge = CvBridge()
        assert cv2.imwrite(str(out / 'current_scene.png'),
            bridge.imgmsg_to_cv2(captured['pair']['color'], 'bgr8'))
        np.savez_compressed(out / 'current_depth.npz',
            depth=bridge.imgmsg_to_cv2(captured['pair']['depth'], 'passthrough'),
            stamp_ns=captured['image_stamp_ns'])
        (out / 'preflight.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        (out / 'camera_configuration.json').write_text(json.dumps(
            rospy.get_param('/camera'), indent=2) + '\n')
        print(json.dumps(dict(output=str(out), **result), ensure_ascii=False), flush=True)
    finally:
        for subscription in subscriptions:
            subscription.unregister()


if __name__ == '__main__':
    main()
