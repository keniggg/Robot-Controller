"""Finite read-only validation of the loaded perception status channel."""
from collections import Counter
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import threading
import time

MOUNT = Path('/media/zhuyupei/SanStand')
if not MOUNT.is_mount():
    raise RuntimeError('SanStand unavailable')
OUT = MOUNT/'实验记录/Robot-Controller/ros_log'/('rgb_interest_live_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
OUT.mkdir(); (OUT/'ros').mkdir()
os.environ['ROS_LOG_DIR'] = str(OUT/'ros')

import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState

rospy.init_node('watch_rgb_interest_readonly', anonymous=True, disable_signals=True)
lock = threading.Lock(); rows = []; telemetry = {}


def status(message):
    value = json.loads(message.data)
    with lock:
        if len(rows) < 128:
            rows.append(dict(received_ros_ns=rospy.Time.now().to_nsec(), status=value))


def state(message, name):
    with lock:
        if name == 'accepted':
            telemetry[name] = dict(stamp_ns=message.header.stamp.to_nsec(), frame_id=message.header.frame_id,
                                   name=list(message.name), positions=list(message.position))
        else:
            telemetry[name] = message.data


subscribers = [rospy.Subscriber('/perception/unknown/detector_status', String, status, queue_size=10)]
for name,topic in [('enabled','/alicia_d/motion_enabled'), ('ready','/alicia_d/feedback_ready'),
                   ('detected','/perception/unknown/object_detected')]:
    subscribers.append(rospy.Subscriber(topic, Bool, lambda m,n=name:state(m,n), queue_size=2))
subscribers.append(rospy.Subscriber('/alicia_d/accepted_joint_states', JointState,
    lambda m:state(m,'accepted'), queue_size=10))
started = time.monotonic()
while time.monotonic()-started < 18. and not rospy.is_shutdown():
    time.sleep(.1)
for sub in subscribers:
    sub.unregister()
with lock:
    rows = list(rows); telemetry = dict(telemetry)
interests = [row['status'].get('observation_interest',{}) for row in rows]
accepted = [row for row in interests if row.get('observation_interest_ready') is True]
summary = dict(status_count=len(rows), observation_ready_count=len(accepted),
    reasons=dict(Counter(row.get('reason','missing') for row in interests)),
    original_detector_details=dict(Counter(row['status'].get('detail') for row in rows)),
    max_interest_processing_sec=max([r.get('processing_seconds',0.) for r in interests] or [0.]),
    max_ready_source_age_sec=max([r.get('source_age_at_status_sec',0.) for r in accepted] or [0.]),
    authority_false_for_all=all(r.get('motion_authorized') is False
        and r.get('contact_geometry_authorized') is False for r in interests),
    telemetry=telemetry, robot_commands_sent=False)
if not MOUNT.is_mount():
    raise RuntimeError('SanStand disappeared')
(OUT/'statuses.json').write_text(json.dumps(dict(rows=rows,summary=summary),indent=2,ensure_ascii=False)+'\n')
print(json.dumps(dict(directory=str(OUT),**summary),indent=2,ensure_ascii=False))
