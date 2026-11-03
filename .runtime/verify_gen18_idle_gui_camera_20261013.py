"""Read fresh RGB-D and GUI-owned manual state without sending control."""
import argparse
import json
import os
from pathlib import Path
import threading
import time

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
out = args.output.resolve()
mount = Path('/media/zhuyupei/SanStand')
assert mount.is_mount() and mount / '实验记录/Robot-Controller/ros_log' in out.parents
assert out.parent.stat().st_dev == mount.stat().st_dev
out.mkdir(exist_ok=False)
os.environ['ROS_LOG_DIR'] = str(out / 'ros')
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from actionlib_msgs.msg import GoalStatusArray
from alicia_flexible_grasp_supervisor.msg import GraspState

rospy.init_node('gen18_idle_gui_camera_readonly', anonymous=True, disable_rosout=True)
lock = threading.Lock()
values, stamps, gui = {}, {'color': set(), 'depth': set()}, []


def receive(key, message):
    with lock:
        values[key] = message
        if key in stamps:
            stamps[key].add(message.header.stamp.to_nsec())
        if key == 'gui':
            gui.append(dict(value=message.data, caller=message._connection_header.get('callerid'),
                            latched=message._connection_header.get('latching')))


topics = [('color', '/supervisor/camera/color/image_raw', Image),
          ('depth', '/supervisor/camera/depth/image_raw', Image),
          ('gui', '/gui/joint_direct_mode', Bool),
          ('task', '/grasp/state', GraspState), ('mode', '/grasp_mode/status', String),
          ('arm', '/alicia_controller/follow_joint_trajectory/status', GoalStatusArray),
          ('hand', '/hand_controller/follow_joint_trajectory/status', GoalStatusArray)]
subs = [rospy.Subscriber(topic, kind, lambda msg, key=key: receive(key, msg),
                        queue_size=5, buff_size=2**24) for key, topic, kind in topics]
result = dict(passed=False, motion_commands_sent=False, temperature_queried=False,
              automatic_grasp_started=False, operator_clearance_claimed=False)
try:
    end = time.monotonic() + 12.
    while time.monotonic() < end:
        with lock:
            ready = (len(values) == len(topics) and all(len(s) >= 5 for s in stamps.values())
                     and any(r['caller'] == '/alicia_supervisor_gui' for r in gui))
        if ready:
            break
        time.sleep(.05)
    with lock:
        state, gui_rows = dict(values), list(gui)
        distinct = {k: len(s) for k, s in stamps.items()}
    assert ready, ('incomplete telemetry', sorted(state), distinct, gui_rows)
    assert rospy.get_param('/gui/joint_direct_mode') is True
    own = [r for r in gui_rows if r['caller'] == '/alicia_supervisor_gui']
    assert own[-1]['value'] is True and own[-1]['latched'] == '1'
    assert not any(r['value'] is False for r in gui_rows), gui_rows
    assert not state['task'].active
    mode = json.loads(state['mode'].data)
    assert not any(mode.get(k) for k in ('active', 'start_reserved', 'pending'))
    assert all(not any(g.status in (0, 1, 6, 7) for g in state[k].status_list) for k in ('arm', 'hand'))
    now = rospy.Time.now().to_nsec()
    images = {}
    for key in ('color', 'depth'):
        message = state[key]
        age = (now - message.header.stamp.to_nsec()) / 1e9
        assert 0 <= age < .8 and message.width > 0 and message.height > 0
        images[key] = dict(stamp_ns=message.header.stamp.to_nsec(), age_sec=age,
                           width=message.width, height=message.height, encoding=message.encoding,
                           distinct_frames=distinct[key])
    bridge = CvBridge()
    color = bridge.imgmsg_to_cv2(state['color'], 'bgr8')
    depth = bridge.imgmsg_to_cv2(state['depth'], 'passthrough')
    valid = np.isfinite(depth) & (depth > 0)
    assert np.count_nonzero(valid) > 0, 'camera depth has no finite positive samples'
    assert cv2.imwrite(str(out / 'current_scene.png'), color)
    result.update(passed=True, recorded_ns=now, manual_mode=True,
                  gui_own_latched_manual_ack=own[-1], task_state=state['task'].state,
                  task_active=False, mode=mode, images=images,
                  positive_depth_fraction=float(np.mean(valid)), selection=rospy.get_param('/grasp_mode/selection'))
except Exception as exc:
    result['error'] = repr(exc)
    raise
finally:
    for sub in subs:
        sub.unregister()
    assert mount.is_mount() and out.stat().st_dev == mount.stat().st_dev
    (out / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(result, ensure_ascii=False), flush=True)
