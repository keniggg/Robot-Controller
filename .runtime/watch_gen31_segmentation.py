#!/usr/bin/env python3
"""Read-only bounded numeric detector timing subscriber, generation 31 only."""
import json
import time
from pathlib import Path
import rospy
from std_msgs.msg import String

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z/gen31_handoff_analysis')
assert Path('/media/zhuyupei/SanStand').is_mount()
ROOT.mkdir(exist_ok=True)
output = ROOT / ('live_segmentation_' + str(time.time_ns()) + '.jsonl')
stream = output.open('x')
seen = set()
counts = dict(messages=0, timed=0, alerts=0)
started = time.monotonic()
last_print = [started]

def callback(message):
    data = json.loads(message.data)
    if data.get('generation') != 31:
        return
    timing = data.get('frame_timing') or {}
    key = (data.get('source_stamp_ns'), data.get('state'), data.get('detail'), timing.get('status_build_ros_ns'))
    if key in seen:
        return
    seen.add(key)
    counts['messages'] += 1
    counts['timed'] += bool(timing)
    row = {k:data.get(k) for k in ('generation','selection_stamp_ns','source_stamp_ns','state','detail','last_good_source_stamp_ns')}
    row.update(received_ros_ns=rospy.Time.now().to_nsec(), received_monotonic=time.monotonic(), frame_timing=timing)
    alerts = []
    age = timing.get('failure_source_age_sec', timing.get('status_source_age_sec', 0.))
    if isinstance(age, (int,float)) and age > .6:
        alerts.append('current_source_age_gt_0.6')
    components = timing.get('segmentation_components') or {}
    for name, entries in components.items():
        limit = .05 if name != 'segment_with_support_fallback' else .25
        if any(e.get('wall_sec',0.) > limit for e in entries):
            alerts.append(name+'_wall_spike')
    if timing.get('stage_wall_sec',{}).get('segmentation',0.) > .3:
        alerts.append('aggregate_segmentation_gt_0.3')
    row['alerts'] = alerts
    stream.write(json.dumps(row,allow_nan=False)+'\n');stream.flush()
    if alerts:
        counts['alerts'] += 1
        print(json.dumps(row,allow_nan=False),flush=True)
    elif time.monotonic()-last_print[0] > 15.:
        last_print[0] = time.monotonic()
        print(json.dumps(dict(progress=counts,source=row['source_stamp_ns'],state=row['state'],detail=row['detail'],age=age,
                              segmentation_components=components)),flush=True)

rospy.init_node('readonly_gen31_segmentation_watch', anonymous=True, disable_rosout=True)
sub = rospy.Subscriber('/perception/unknown/detector_status', String, callback, queue_size=10)
print('OUTPUT '+str(output),flush=True)
try:
    while not rospy.is_shutdown() and time.monotonic()-started < 300.:
        time.sleep(.2)
finally:
    sub.unregister();stream.close()
    print(json.dumps(dict(done=counts,output=str(output))),flush=True)
