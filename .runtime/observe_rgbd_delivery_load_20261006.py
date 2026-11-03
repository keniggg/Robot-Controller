"""Sixty-second passive RGB-D delivery/connection observation; no control calls."""
import argparse
import collections
import hashlib
import json
import os
from pathlib import Path
import threading
import time
import xmlrpc.client

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from observe_gen15_rgbd_transport import proc, sockets

EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
TOPICS = ('/supervisor/camera/color/image_raw', '/supervisor/camera/depth/image_raw')
NODES = ('/supervisor_camera', '/unknown_tabletop_perception')

class BoundedTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = .3
        return connection

def rpc(uri, method, caller, *args):
    with xmlrpc.client.ServerProxy(uri, transport=BoundedTransport()) as proxy:
        reply = getattr(proxy, method)(caller, *args)
    if reply[0] != 1:
        raise RuntimeError('%s: %s' % (method, reply[1]))
    return reply[2]

def bus_snapshot(owners, caller, unavailable_stats):
    rows = []
    for name, owner in owners.items():
        row = dict(node=name, requested_wall_ns=time.time_ns(), expected_pid=owner['pid'])
        try:
            current = rpc(owner['uri'], 'getPid', caller)
            row['observed_pid'] = current
            if current != owner['pid']:
                raise RuntimeError('node owner changed; counters not joined to original PID')
            info = rpc(owner['uri'], 'getBusInfo', caller)
            row['connections'] = [v for v in info if len(v) >= 5 and v[4] in TOPICS]
            if name in unavailable_stats:
                row['statistics_unavailable'] = unavailable_stats[name]
            else:
                try:
                    stats = rpc(owner['uri'], 'getBusStats', caller)
                    row['statistics'] = {'publish': [v for v in stats[0] if v[0] in TOPICS],
                                         'subscribe': [v for v in stats[1] if v[0] in TOPICS]}
                except xmlrpc.client.Fault as exc:
                    if 'int exceeds XML-RPC limits' not in str(exc):
                        raise
                    # Long-running rospy byte counters exceed XML-RPC's int32.
                    # Keep the successful topic/socket mapping and use ss native
                    # counters; repeated failed statistics RPCs add no evidence.
                    unavailable_stats[name] = str(exc)
                    row['statistics_unavailable'] = str(exc)
        except Exception as exc:
            row['error'] = type(exc).__name__ + ': ' + str(exc)
        row['completed_wall_ns'] = time.time_ns()
        rows.append(row)
    return rows

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--seconds', default=60., type=float)
    args = parser.parse_args()
    if not 0 < args.seconds <= 60:
        parser.error('duration must be positive and at most 60 seconds')
    out = args.out.resolve()
    if not Path('/media/zhuyupei/SanStand').is_mount() or EXT not in out.parents:
        parser.error('SanStand output required; no system-disk fallback')
    out.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(out/'ros')
    rospy.init_node('rgbd_delivery_load_observer', anonymous=True, disable_signals=True)
    caller = rospy.get_name()
    master_uri = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
    owners = {}
    for name in NODES:
        uri = rpc(master_uri, 'lookupNode', caller, name)
        owners[name] = dict(uri=uri, pid=rpc(uri, 'getPid', caller))
    owners[caller] = dict(uri=rospy.get_node_uri(), pid=os.getpid())
    pids = [row['pid'] for row in owners.values()]
    events, passive = [], []
    lock = threading.Lock()
    state = dict(open=True, streams={}, ready=False)
    def append(row):
        with lock:
            if state['open']:
                events.append(row)
    def image(stream, msg):
        wall, mono = time.time_ns(), time.monotonic()
        source = msg.header.stamp.to_nsec()
        row = dict(kind='image_header', stream=stream, source_stamp_ns=source,
                   seq=msg.header.seq, receive_wall_ns=wall, receive_monotonic=mono,
                   source_age_sec=(wall-source)*1e-9, data_bytes=len(msg.data))
        with lock:
            if state['open']:
                events.append(row)
                state['streams'][stream] = row
    def status(msg):
        wall, mono = time.time_ns(), time.monotonic()
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        append(dict(kind='unknown_status', receive_wall_ns=wall, receive_monotonic=mono,
                    **{key: data[key] for key in ('generation', 'source_stamp_ns', 'detail', 'state',
                       'target_lost', 'frame_timing', 'rgbd_ingress_timing') if key in data}))
    subscriptions = [rospy.Subscriber(topic, Image, lambda msg, stream=stream: image(stream, msg),
                         queue_size=2, buff_size=2**24)
                     for topic, stream in zip(TOPICS, ('color', 'depth'))]
    subscriptions.append(rospy.Subscriber('/perception/unknown/detector_status', String, status, queue_size=4))
    start, startwall = time.monotonic(), time.time_ns()
    tick_index = 0
    unavailable_stats = {}
    try:
        while time.monotonic()-start < args.seconds and not rospy.is_shutdown():
            tick = time.monotonic()
            sample = dict(wall_ns=time.time_ns(), monotonic=tick,
                processes=[proc(pid) for pid in pids],
                psi={name: Path('/proc/pressure', name).read_text() for name in ('cpu', 'memory', 'io')},
                loadavg=Path('/proc/loadavg').read_text().strip())
            if tick_index % 2 == 0:
                sample['sockets'] = sockets(pids)
            if tick_index % 5 == 0:
                sample['ros_connections'] = bus_snapshot(owners, caller, unavailable_stats)
            passive.append(sample)
            with lock:
                ready = not state['ready'] and set(state['streams']) == {'color', 'depth'}
                if ready:
                    state['ready'] = True
                    ready_data = dict(observer_pid=os.getpid(), started_wall_ns=startwall,
                        observed_wall_ns=time.time_ns(), owners=owners,
                        first_observed_sources={k: v['source_stamp_ns'] for k, v in state['streams'].items()},
                        scope='observer receiving both streams; not grasp readiness or source qualification')
            if ready:
                (out/'ready.json').write_text(json.dumps(ready_data, indent=2)+'\n')
                print('OBSERVER_READY '+str(out/'ready.json'), flush=True)
            tick_index += 1
            remaining = min(start+args.seconds, tick+1.)-time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
    finally:
        with lock:
            state['open'] = False
        for subscription in subscriptions:
            subscription.unregister()
        with (out/'events.jsonl').open('w') as handle:
            for row in events:
                handle.write(json.dumps(row, sort_keys=True, allow_nan=False)+'\n')
        with (out/'passive.jsonl').open('w') as handle:
            for row in passive:
                handle.write(json.dumps(row, sort_keys=True, allow_nan=False)+'\n')
        source = Path(__file__)
        report = dict(schema=2, started_wall_ns=startwall, elapsed_sec=time.monotonic()-start,
            owners=owners, observer_pid=os.getpid(), ready=state['ready'],
            source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
            helper_sha256=hashlib.sha256(source.with_name('observe_gen15_rgbd_transport.py').read_bytes()).hexdigest(),
            counts=dict(collections.Counter(row['kind'] for row in events)),
            scope='Two extra full-image subscriptions queue2/buff16MiB; headers only retained. Adds transport load. getBusInfo/getBusStats/getPid/lookupNode are read-only, bounded .3s each. No control, parameter writes or temperature queries. A delayed RPC sample is not an image-synchronous measurement.',
            limitations='No production enqueue/deserialization/GIL probes. Healthy interval cannot distinguish historical fault source; thread wait and socket queue snapshots alone are not proof of causal blocking.')
        (out/'manifest.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(dict(output=str(out), **report)), flush=True)

if __name__ == '__main__':
    main()
