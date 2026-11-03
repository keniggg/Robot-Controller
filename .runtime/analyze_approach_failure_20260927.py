#!/usr/bin/env python3
"""Offline extraction only; does not initialize ROS or publish any commands."""
import collections
import json
import math
import pathlib
import rosbag
import sys

ROOT = pathlib.Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RECORD = ROOT / 'grasp_records/20260927T063422.563534Z_094d509c116648378bf54b272bcc3489'
OUT = ROOT / 'ros_log/unknown_grasp_20260923/approach_failure_20260927'
T0, T1 = 1790491032., 1790491098.

def plain(v):
    if isinstance(v, (str, int, float, bool)) or v is None:
        return v
    if isinstance(v, (list, tuple)):
        return [plain(x) for x in v]
    if hasattr(v, 'to_nsec'):
        return {'sec': v.to_sec(), 'ns': v.to_nsec()}
    if hasattr(v, '__slots__'):
        return {k: plain(getattr(v, k)) for k in v.__slots__}
    return str(v)

def positions(m):
    return list(m.position)

def main():
    if not pathlib.Path('/media/zhuyupei/SanStand').is_mount():
        raise SystemExit('SanStand not mounted; refusing fallback')
    OUT.mkdir(parents=True, exist_ok=True)
    events, series = collections.defaultdict(list), collections.defaultdict(list)
    with rosbag.Bag(str(RECORD / 'keyframes.bag')) as bag:
        topics = bag.get_type_and_topic_info().topics
        relevant = [k for k, v in topics.items() if v.msg_type != 'sensor_msgs/Image'
                    and not k.startswith('/tf') and not k.startswith('/perception/')
                    and not k.endswith('/status') and not k.endswith('/object_geometry')]
        meta = {k: {'type': v.msg_type, 'count': v.message_count} for k,v in topics.items()}
        for topic, msg, stamp in bag.read_messages(topics=relevant):
            t = stamp.to_sec()
            if topics[topic].msg_type == 'sensor_msgs/JointState':
                if T0 <= t <= T1:
                    series[topic].append({'t': t, 'stamp': msg.header.stamp.to_sec(),
                                          'names': list(msg.name), 'position': positions(msg),
                                          'velocity': list(msg.velocity), 'effort': list(msg.effort)})
            elif topic == '/alicia_controller/state':
                if T0 <= t <= T1:
                    series[topic].append({'t': t, 'msg': plain(msg)})
            elif topic == '/rosout_agg':
                if T0 <= t <= T1 and ('gateway' in msg.name or 'controller' in msg.name
                        or 'grasp_task' in msg.name or 'move_group' in msg.name):
                    events[topic].append({'t': t, 'msg': plain(msg)})
            elif topic in ['/alicia_d/sdk_diagnostics', '/alicia_d/actuation_status']:
                if T0 <= t <= T1:
                    events[topic].append({'t': t, 'msg': plain(msg)})
            else:
                events[topic].append({'t': t, 'msg': plain(msg)})
    result = {'record': str(RECORD), 'window_unix': [T0,T1], 'topics': meta,
              'events': events, 'series': series}
    (OUT / 'bag_forensics.json').write_text(json.dumps(result, indent=2))
    summaries = []
    for e in events['/alicia_controller/follow_joint_trajectory/goal']:
        trajectory=e['msg']['goal']['trajectory']; ps=trajectory['points']
        summaries.append({'time':e['t'], 'id':e['msg']['goal_id']['id'],
           'point_count':len(ps), 'duration':ps[-1]['time_from_start']['sec'],
           'first':ps[0]['positions'],'last':ps[-1]['positions'],
           'max_abs_velocity':[max(abs(p['velocities'][j]) for p in ps) if all(len(p['velocities'])>j for p in ps) else None for j in range(6)],
           'max_abs_acceleration':[max(abs(p['accelerations'][j]) for p in ps) if all(len(p['accelerations'])>j for p in ps) else None for j in range(6)]})
    small = {'goals':summaries,
       'results':events['/alicia_controller/follow_joint_trajectory/result'],
       'epochs':events['/alicia_d/control_reference_epoch'],
       'temperatures':events['/alicia_d/temperatures_c'],
       'protection_values':sorted(set(e['msg']['data'] for e in events['/alicia_d/protection_latched'])),
       'motion_enabled_values':sorted(set(e['msg']['data'] for e in events['/alicia_d/motion_enabled'])),
       'run_status_values':dict(collections.Counter(e['msg']['data'] for e in events['/alicia_d/run_status'])),
       'diagnostic_example':events['/alicia_d/sdk_diagnostics'][-1:]}
    for name in ['/joint_states','/alicia_d/accepted_joint_states','/alicia_d/sdk_command','/alicia_d/control_reference','/joint_commands']:
        rows=[r for r in series[name] if 1790491081.835 <= r['t'] <= 1790491095.824]
        small[name]={'count':len(rows), 'start':rows[0] if rows else None, 'end':rows[-1] if rows else None,
                     'min_j2':min(r['position'][1] for r in rows) if rows else None,
                     'max_j2':max(r['position'][1] for r in rows) if rows else None}
    (OUT/'bag_summary.json').write_text(json.dumps(small,indent=2))
    print(json.dumps({k:v for k,v in small.items() if k not in ('temperatures','diagnostic_example')},indent=2))
    print('Written:', OUT)

if __name__ == '__main__':
    main()
