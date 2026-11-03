#!/usr/bin/env python3
"""Twelve-second passive capture: RGB-D, accepted positions, TF, camera config.

No diagnostic, device-info, rosout, temperature or command subscriptions. No
publishers, services or parameter writes. Joint effort is deliberately omitted.
"""
import argparse
from collections import OrderedDict
import hashlib
import json
import os
from pathlib import Path
import threading
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    assert Path('/media/zhuyupei/SanStand').is_mount()
    assert Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller') in args.output.resolve().parents
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(args.output/'ros')
    import rospy
    import rosbag
    import cv2
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image, JointState
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    rospy.init_node('arrival_rgbd_readonly', anonymous=True, disable_rosout=True)
    bag_path = args.output/'arrival.bag'
    bag = rosbag.Bag(str(bag_path), 'w', compression=rosbag.Compression.LZ4)
    lock = threading.Lock()
    pending = OrderedDict()
    state = dict(closed=False, count=0, latest=None, last_saved_ns=0)
    image_topics = ('/supervisor/camera/color/image_raw', '/supervisor/camera/depth/image_raw')
    topics = image_topics + ('/alicia_d/accepted_joint_states', '/tf', '/tf_static')

    def image(topic, message):
        ns = message.header.stamp.to_nsec()
        with lock:
            if state['closed'] or ns <= state['last_saved_ns']:
                return
            pending.setdefault(ns, {})[topic] = message
            pair = pending[ns]
            if len(pair) == 2 and ns-state['last_saved_ns'] >= 300000000:
                now = rospy.Time.now()
                for key in image_topics:
                    bag.write(key,pair[key],now)
                state.update(count=state['count']+1,latest=pair[image_topics[0]],last_saved_ns=ns)
                print('PAIR %d %d' % (state['count'],ns),flush=True)
            while len(pending) > 12:
                pending.popitem(last=False)

    def metadata(topic, message):
        if topic == '/alicia_d/accepted_joint_states':
            # Only known arm/finger kinematics are copied; never effort or diagnostics.
            if message.header.frame_id != 'sdk_measured':
                return
            safe = JointState();safe.header = message.header
            indices = [i for i,n in enumerate(message.name)
                       if n in ['Joint%d'%j for j in range(1,7)]+['right_finger']]
            safe.name = [message.name[i] for i in indices]
            safe.position = [message.position[i] for i in indices]
            if len(message.velocity) == len(message.name):
                safe.velocity = [message.velocity[i] for i in indices]
            message = safe
        with lock:
            if not state['closed']:
                bag.write(topic,message,rospy.Time.now())

    subscribers = [rospy.Subscriber(topic,Image,lambda m,t=topic:image(t,m),queue_size=3,buff_size=2**24)
                   for topic in image_topics]
    subscribers += [rospy.Subscriber(topic,kind,lambda m,t=topic:metadata(t,m),queue_size=60)
                    for topic,kind in (('/alicia_d/accepted_joint_states',JointState),('/tf',TFMessage),('/tf_static',TFMessage))]
    started = time.monotonic()
    start_ns = rospy.Time.now().to_nsec()
    configurations = []
    print('READONLY_CAPTURE_STARTED output='+str(args.output),flush=True)
    try:
        while time.monotonic()-started < 12. and not rospy.is_shutdown():
            if not configurations or time.monotonic()-configurations[-1]['monotonic'] >= 2.:
                row = dict(stamp_ns=rospy.Time.now().to_nsec(),monotonic=time.monotonic(),
                           parameters=rospy.get_param('/camera'))
                configurations.append(row)
                metadata('/arrival_diagnostic/camera_parameters',String(data=json.dumps(row)))
            time.sleep(.05)
    finally:
        for subscriber in subscribers:
            subscriber.unregister()
        with lock:
            state['closed']=True
            bag.close()
        png = args.output/'current_rgb.png'
        if state['latest'] is not None:
            assert cv2.imwrite(str(png),CvBridge().imgmsg_to_cv2(state['latest'],'bgr8'))
        report = dict(scope='stationary arrival diagnostic; not target identity or execution authority',
            started_ns=start_ns,finished_ns=rospy.Time.now().to_nsec(),duration_sec=time.monotonic()-started,
            subscriptions=list(topics),camera_parameter_reads=len(configurations),pairs=state['count'],
            last_rgb_stamp_ns=state['last_saved_ns'],bag_path=str(bag_path),png_path=str(png),
            physical_motion_requested=False,parameters_written=False,diagnostic_topics_subscribed=False,
            joint_effort_recorded=False,execution_authority=False,
            script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        (args.output/'capture.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(report,ensure_ascii=False),flush=True)


if __name__ == '__main__':
    main()
