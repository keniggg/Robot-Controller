"""Read-only idle confirmation after the user's explicit GEN30 pause."""
import hashlib
import json
import os
from pathlib import Path
import threading
import time

M = Path('/media/zhuyupei/SanStand')
N = M / '实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z'
O = N / 'gen30_user_paused_20261015/idle_check'
assert M.is_mount() and not O.exists()
O.mkdir()
os.environ['ROS_LOG_DIR'] = str(O / 'ros')
import rospy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState
from actionlib_msgs.msg import GoalStatusArray
from alicia_flexible_grasp_supervisor.msg import GraspState

rospy.init_node('gen30_pause_idle_readonly', anonymous=True, disable_rosout=True)
lock, gui, joints = threading.Lock(), [], []


def on_gui(msg):
    with lock:
        gui.append(dict(value=msg.data, caller=msg._connection_header.get('callerid'),
                        latched=msg._connection_header.get('latching'), received_ns=rospy.Time.now().to_nsec()))


def on_joint(msg):
    if msg.header.frame_id != 'sdk_measured':
        return
    named = dict(zip(msg.name, msg.position))
    if not all('Joint%d' % i in named for i in range(1, 7)):
        return
    with lock:
        joints.append(dict(stamp_ns=msg.header.stamp.to_nsec(),
            positions=[float(named['Joint%d' % i]) for i in range(1, 7)]))


subscriptions = [rospy.Subscriber('/gui/joint_direct_mode', Bool, on_gui, queue_size=20),
    rospy.Subscriber('/alicia_d/accepted_joint_states', JointState, on_joint, queue_size=100)]
selection = rospy.get_param('/grasp_mode/selection')
task = rospy.wait_for_message('/grasp/state', GraspState, timeout=5)
mode = json.loads(rospy.wait_for_message('/grasp_mode/status', String, timeout=5).data)
actions = {}
for topic in ('/alicia_controller/follow_joint_trajectory/status',
              '/hand_controller/follow_joint_trajectory/status'):
    msg = rospy.wait_for_message(topic, GoalStatusArray, timeout=5)
    actions[topic] = [dict(id=g.goal_id.id, status=int(g.status)) for g in msg.status_list]
end = time.monotonic() + 3.
while time.monotonic() < end:
    time.sleep(.05)
manual = rospy.get_param('/gui/joint_direct_mode')
with lock:
    gui_rows, joint_rows = list(gui), list(joints)
own = [r for r in gui_rows if r['caller'] == '/alicia_supervisor_gui']
span = ([max(r['positions'][i] for r in joint_rows) - min(r['positions'][i] for r in joint_rows)
         for i in range(6)] if joint_rows else [])
active = any(g['status'] in (0, 1, 6, 7) for values in actions.values() for g in values)
now = rospy.Time.now().to_nsec()
passed = (selection['generation'] == 11 and rospy.get_param('/grasp_mode/selection') == selection
    and manual is True and bool(own) and own[-1]['value'] is True and own[-1]['latched'] == '1'
    and not task.active and not any(mode.get(k) for k in ('active', 'start_reserved', 'pending'))
    and not active and len(joint_rows) >= 20 and 0 <= now - joint_rows[-1]['stamp_ns'] < 200000000
    and max(span) <= .004)
result = dict(passed=bool(passed), recorded_ns=now, selection=selection,
    manual_mode=manual, task_active=bool(task.active), router_status=mode, actions=actions,
    actions_active=active, gui_deliveries=gui_rows, accepted_joint_count=len(joint_rows),
    accepted_joint_span_rad=span, last_accepted_joint=joint_rows[-1] if joint_rows else None,
    gui_own_manual_ack=bool(own) and own[-1]['value'] is True,
    motion_commands_sent=False, temperature_queried=False, parameters_changed=False,
    script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
(O / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
for subscription in subscriptions:
    subscription.unregister()
print(json.dumps({k: result[k] for k in ('passed', 'manual_mode', 'task_active', 'actions_active',
    'accepted_joint_count', 'accepted_joint_span_rad', 'gui_own_manual_ack')}), flush=True)
raise SystemExit(0 if passed else 1)
