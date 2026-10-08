"""Bounded passive camera/resource observation; no publishers or robot commands."""
import json, os, time, threading, argparse, collections
from pathlib import Path
from datetime import datetime, timezone


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=300.)
    args = parser.parse_args()
    if not 1 <= args.duration <= 600: raise ValueError('duration outside bounded observation')
    mount = Path('/media/zhuyupei/SanStand')
    if not mount.is_mount(): raise RuntimeError('SanStand is not mounted')
    out = mount/'实验记录/Robot-Controller/ros_log'/('camera_transport_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    out.mkdir(); (out/'ros').mkdir(); os.environ['ROS_LOG_DIR'] = str(out/'ros')
    import rospy
    from sensor_msgs.msg import Image, JointState
    from rosgraph_msgs.msg import Log
    from alicia_flexible_grasp_supervisor.msg import GraspState
    rospy.init_node('passive_camera_transport', anonymous=True, disable_signals=True)
    events = collections.deque(maxlen=40000)
    resources = []
    lock = threading.Lock()
    start = time.time_ns()
    start_mono = time.monotonic_ns()
    def event(kind, **values):
        with lock: events.append(dict(kind=kind, wall_ns=time.time_ns(), monotonic_ns=time.monotonic_ns(), **values))
    def frame(msg, kind): event(kind, source_ns=msg.header.stamp.to_nsec(), seq=msg.header.seq)
    def state(msg): event('task', active=msg.active, state=msg.state, message=msg.message)
    def log(msg):
        if (msg.name == '/supervisor_camera' and ('slow' in msg.msg.lower() or 'diagnostic' in msg.msg.lower())) or ('TARGET_LOST' in msg.msg or 'source_frame_stale' in msg.msg):
            event('log', source_ns=msg.header.stamp.to_nsec(), node=msg.name, message=msg.msg)
    subscribers = [
        rospy.Subscriber('/supervisor/camera/color/image_raw', Image, lambda m: frame(m, 'color'), queue_size=1, buff_size=2**24),
        rospy.Subscriber('/supervisor/camera/depth/image_raw', Image, lambda m: frame(m, 'depth'), queue_size=1, buff_size=2**24),
        rospy.Subscriber('/alicia_d/accepted_joint_states', JointState, lambda m: frame(m, 'joint'), queue_size=20),
        rospy.Subscriber('/rosout_agg', Log, log, queue_size=100),
        rospy.Subscriber('/grasp/state', GraspState, state, queue_size=10)]
    paths = ['/proc/pressure/'+s for s in ('cpu','io','memory')]+['/proc/stat','/proc/diskstats','/proc/loadavg', '/sys/bus/usb/devices/4-1/power/runtime_status']
    def read(path):
        try: return Path(path).read_text().strip()
        except OSError as exc: return str(exc)
    record = dict(directory=str(out), started_ns=start, duration_sec=args.duration, hardware_commands_sent=0, images_written=False)
    Path('.runtime/latest_camera_transport.json').write_text(json.dumps(record, ensure_ascii=False, indent=2)+'\n')
    (out/'ready.json').write_text(json.dumps(record, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(record, ensure_ascii=False), flush=True)
    try:
        while not rospy.is_shutdown() and (time.monotonic_ns()-start_mono)*1e-9 < args.duration:
            wall = time.time_ns(); mono = time.monotonic_ns()
            values = {p:read(p) for p in paths}
            values['/proc/stat'] = values['/proc/stat'].splitlines()[0]
            resources.append(dict(wall_ns=wall, monotonic_ns=mono, values=values))
            time.sleep(.5)
    finally:
        for sub in subscribers: sub.unregister()
        with lock: rows = list(events)
        record.update(ended_ns=time.time_ns(), events=rows, resources=resources,
            limitation='Passive guest observation cannot identify host USB routing or prove root cause; callback receipt gaps may include scheduling delays.')
        if not mount.is_mount(): raise RuntimeError('SanStand unavailable: capture retained in process memory only')
        (out/'raw.json').write_text(json.dumps(record, ensure_ascii=False, indent=2)+'\n')
        result = {}
        for kind in ('color','depth','joint'):
            selected=[r for r in rows if r['kind']==kind]
            gaps=[(b['source_ns']-a['source_ns'])*1e-9 for a,b in zip(selected,selected[1:])]
            result[kind]=dict(samples=len(selected), max_source_gap_sec=max(gaps,default=0.), gaps_over_800ms=sum(x>.8 for x in gaps), gaps_over_200ms=sum(x>.2 for x in gaps), max_receipt_age_sec=max(((r['wall_ns']-r['source_ns'])*1e-9 for r in selected),default=0.))
        result['resource_sample_max_gap_sec']=max(((b['monotonic_ns']-a['monotonic_ns'])*1e-9 for a,b in zip(resources,resources[1:])),default=0.)
        result['hardware_commands_sent']=0
        (out/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result), flush=True)

if __name__ == '__main__': main()
