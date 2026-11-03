"""Read-only idle check after the requested offline GEN31 regression pause."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import threading
import time

ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument('--output', required=True, type=Path)
args = ap.parse_args()
mount = Path('/media/zhuyupei/SanStand')
assert mount.is_mount() and not os.statvfs(mount).f_flag & os.ST_RDONLY
output = args.output.resolve()
output.relative_to(mount/'实验记录/Robot-Controller/ros_log')
assert not output.exists()
output.mkdir()
os.environ['ROS_LOG_DIR'] = str(output/'ros')

import rospy
from std_msgs.msg import Bool, String
from actionlib_msgs.msg import GoalStatusArray
from alicia_flexible_grasp_supervisor.msg import GraspState

rospy.init_node('gen31_completed_pause_readonly', anonymous=True, disable_rosout=True)
lock, gui = threading.Lock(), []


def on_gui(msg):
    with lock:
        gui.append(dict(value=msg.data, caller=msg._connection_header.get('callerid'),
            latched=msg._connection_header.get('latching'), received_ns=rospy.Time.now().to_nsec()))


subscription = rospy.Subscriber('/gui/joint_direct_mode', Bool, on_gui, queue_size=20)
selection = rospy.get_param('/grasp_mode/selection')
task = rospy.wait_for_message('/grasp/state', GraspState, timeout=5)
router = json.loads(rospy.wait_for_message('/grasp_mode/status', String, timeout=5).data)
actions = {}
for topic in ('/alicia_controller/follow_joint_trajectory/status',
              '/hand_controller/follow_joint_trajectory/status'):
    msg = rospy.wait_for_message(topic, GoalStatusArray, timeout=5)
    actions[topic] = [dict(id=g.goal_id.id, status=int(g.status)) for g in msg.status_list]
end = time.monotonic()+2.
while time.monotonic() < end:
    time.sleep(.05)
manual = rospy.get_param('/gui/joint_direct_mode')
with lock:
    gui_rows = list(gui)
own = [r for r in gui_rows if r['caller'] == '/alicia_supervisor_gui']
active = any(g['status'] in (0, 1, 6, 7) for rows in actions.values() for g in rows)
passed = (rospy.get_param('/grasp_mode/selection') == selection and manual is True
    and bool(own) and own[-1]['value'] is True and own[-1]['latched'] == '1'
    and not task.active and not any(router.get(k) for k in ('active', 'start_reserved', 'pending'))
    and not active)
result = dict(passed=bool(passed), recorded_ns=rospy.Time.now().to_nsec(),
    selection=selection, manual_mode=manual, task_active=bool(task.active),
    router_status=router, actions=actions, actions_active=active, gui_deliveries=gui_rows,
    gui_own_manual_ack=bool(own) and own[-1]['value'] is True,
    motion_commands_sent=False, temperature_queried=False, parameters_changed=False,
    source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
with (output/'result.json').open('x') as stream:
    json.dump(result, stream, ensure_ascii=False, indent=2)
    stream.write('\n')
    stream.flush()
    os.fsync(stream.fileno())
subscription.unregister()
print(json.dumps({k: result[k] for k in ('passed', 'manual_mode', 'task_active',
    'actions_active', 'gui_own_manual_ack')}), flush=True)
raise SystemExit(0 if passed else 1)
