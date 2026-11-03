"""Bounded passive observation of explicit operator movement; no publishers."""
import argparse
import json
import os
from pathlib import Path
import threading
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration-sec', type=float, default=300.)
    args = parser.parse_args()
    mount = Path('/media/zhuyupei/SanStand')
    root = mount/'实验记录/Robot-Controller/ros_log'
    out = args.output.resolve()
    if not mount.is_mount() or root not in out.parents or not 1 <= args.duration_sec <= 300:
        raise RuntimeError('Mounted external output and finite observation required')
    out.mkdir(exist_ok=False)
    if out.stat().st_dev != mount.stat().st_dev:
        raise RuntimeError('No output fallback')
    os.environ['ROS_LOG_DIR'] = str(out/'observer_ros')
    import rospy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Header, String
    rospy.init_node('gen62_manual_reconfirmation_readonly', anonymous=True, disable_rosout=True)
    lock = threading.Lock()
    count, latest = {}, {}
    started = time.monotonic()
    with (out/'events.jsonl').open('x') as events:
        def receive(key, message):
            row = dict(key=key, received_ns=rospy.Time.now().to_nsec(),
                       callerid=message._connection_header.get('callerid'))
            if isinstance(message, JointState):
                row.update(stamp_ns=message.header.stamp.to_nsec(), frame_id=message.header.frame_id,
                           names=list(message.name), positions=list(message.position))
            elif isinstance(message, Header):
                row['stamp_ns'] = message.stamp.to_nsec()
            else:
                row['value'] = message.data
            with lock:
                count[key] = count.get(key, 0) + 1
                latest[key] = row
                events.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')

        topics = [('joint', '/alicia_d/accepted_joint_states', JointState),
                  ('command', '/joint_commands', JointState),
                  ('positive_enable_wire', '/demonstration', Bool),
                  ('epoch', '/alicia_d/control_reference_epoch', Header),
                  ('actuation', '/alicia_d/actuation_status', String),
                  ('motion_enabled', '/alicia_d/motion_enabled', Bool),
                  ('feedback_ready', '/alicia_d/feedback_ready', Bool),
                  ('manual', '/gui/joint_direct_mode', Bool)]
        subs = [rospy.Subscriber(topic, cls, lambda msg, key=key: receive(key, msg), queue_size=100)
                for key, topic, cls in topics]
        try:
            while time.monotonic()-started < args.duration_sec and not rospy.is_shutdown():
                with lock:
                    value = dict(recorded_ns=time.time_ns(), counts=dict(count), latest=dict(latest),
                                 motion_commands_sent=False, temperature_queried=False,
                                 observation_is_not_execution_acceptance=True)
                    events.flush()
                raw = (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode()
                temp = out/'latest.tmp'
                with temp.open('wb') as stream:
                    stream.write(raw); stream.flush()
                os.replace(temp, out/'latest.json')
                if (out/'STOP').exists():
                    break
                time.sleep(.5)
        finally:
            for sub in subs:
                sub.unregister()
            with lock:
                events.flush(); os.fsync(events.fileno())
    result = dict(recorded_ns=time.time_ns(), elapsed_sec=time.monotonic()-started,
                  counts=count, latest=latest, motion_commands_sent=False,
                  temperature_queried=False, observer_closed=True)
    with (out/'result.json').open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())


if __name__ == '__main__':
    main()
