"""Bounded, passive recorder. No publishers, services, or arm commands."""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import queue
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=600.)
    args = parser.parse_args()
    mount = Path('/media/zhuyupei/SanStand')
    if not mount.is_mount() or mount not in args.output.resolve().parents:
        raise RuntimeError('SanStand output required; no fallback')
    if not 0 < args.seconds <= 900:
        raise ValueError('Bounded duration required')
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_LOG_DIR'] = str(args.output / 'ros')
    import rospy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Header, String
    rospy.init_node('slider_recovery_readonly', anonymous=True, disable_signals=True)
    events = queue.Queue(maxsize=5000)
    drops = Counter()
    counts = Counter()
    latest = {}

    def callback(topic):
        def receive(msg):
            row = dict(topic=topic, received_ns=time.time_ns(),
                       caller=msg._connection_header.get('callerid', ''))
            if isinstance(msg, JointState):
                row.update(stamp_ns=msg.header.stamp.to_nsec(), frame_id=msg.header.frame_id,
                           names=list(msg.name), positions=list(msg.position),
                           velocities=list(msg.velocity), effort=list(msg.effort))
            elif isinstance(msg, Header):
                row.update(stamp_ns=msg.stamp.to_nsec(), frame_id=msg.frame_id)
            else:
                row['data'] = msg.data
            try:
                events.put_nowait(row)
            except queue.Full:
                drops[topic] += 1
        return receive

    topics = {'/joint_commands': JointState, '/alicia_d/sdk_command': JointState,
              '/alicia_d/accepted_joint_states': JointState,
              '/alicia_d/control_reference_epoch': Header,
              '/alicia_d/actuation_status': String, '/alicia_d/motion_enabled': Bool,
              '/alicia_d/feedback_ready': Bool, '/gui/joint_direct_mode': Bool}
    subscribers = [rospy.Subscriber(topic, kind, callback(topic), queue_size=200)
                   for topic, kind in topics.items()]
    start = time.monotonic()
    last_flush = start
    with (args.output / 'events.jsonl').open('x') as stream:
        while not rospy.is_shutdown() and time.monotonic() - start < args.seconds:
            try:
                row = events.get(timeout=.2)
            except queue.Empty:
                continue
            stream.write(json.dumps(row, separators=(',', ':')) + '\n')
            topic = row['topic']
            if topic == '/joint_commands' or (
                    topic == '/alicia_d/actuation_status' and
                    row.get('data') != latest.get(topic, {}).get('data')):
                print(json.dumps(row), flush=True)
            latest[topic] = row
            counts[topic] += 1
            if time.monotonic() - last_flush >= 1.:
                stream.flush()
                last_flush = time.monotonic()
        stream.flush()
    for sub in subscribers:
        sub.unregister()
    result = dict(duration_sec=time.monotonic() - start, counts=dict(counts),
                  dropped=dict(drops), latest=latest, remaining_queue=events.qsize(),
                  motion_commands_sent=False)
    (args.output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')


if __name__ == '__main__':
    main()
