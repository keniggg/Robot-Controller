"""Read current terminal/manual SDK state; never sends a control command."""
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import time
import traceback

S = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/full_restart_r1')
assert Path('/media/zhuyupei/SanStand').is_mount()
out = S / 'post_attempt_idle_r1'
out.mkdir(exist_ok=False)
os.environ['ROS_LOG_DIR'] = str(out / 'ros')
result = dict(passed=False, motion_commands_sent=False, parameters_changed=False,
              temperature_queried=False, source=str(Path(__file__).resolve()),
              source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
subscriptions = []
try:
    import rospy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Header, String
    from actionlib_msgs.msg import GoalStatusArray
    from alicia_flexible_grasp_supervisor.msg import GraspState
    rospy.init_node('gen58_post_attempt_readonly', anonymous=True, disable_rosout=True)
    deployment = json.loads((S / 'acceptance/result.json').read_text())
    def identities():
        for row in deployment['after'].values():
            fields = (Path('/proc') / str(row['pid']) / 'stat').read_text().rsplit(')', 1)[1].split()
            assert fields[0] != 'Z' and int(fields[19]) == int(row['start_ticks'])
        return True
    identities()
    lock, samples = threading.Lock(), []
    def receive(msg):
        with lock:
            samples.append(dict(stamp_ns=msg.header.stamp.to_nsec(), received_ns=rospy.Time.now().to_nsec(),
                caller=msg._connection_header.get('callerid'), frame=msg.header.frame_id,
                names=list(msg.name), positions=list(msg.position)))
    subscriptions.append(rospy.Subscriber('/alicia_d/accepted_joint_states', JointState, receive, queue_size=40))
    def one(topic, kind, owner=None):
        msg = rospy.wait_for_message(topic, kind, timeout=5.)
        if owner:
            assert msg._connection_header.get('callerid') == owner
        return msg
    driver = '/alicia_d_driver_node'
    manual = one('/gui/joint_direct_mode', Bool, '/alicia_supervisor_gui')
    task = one('/grasp/state', GraspState, '/grasp_task_node')
    mode = json.loads(one('/grasp_mode/status', String, '/grasp_mode_router').data)
    actions = [one(t, GoalStatusArray) for t in (
        '/alicia_controller/follow_joint_trajectory/status', '/hand_controller/follow_joint_trajectory/status')]
    epoch = one('/alicia_d/control_reference_epoch', Header, driver).stamp.to_nsec()
    actuation = one('/alicia_d/actuation_status', String, driver).data
    enabled = one('/alicia_d/motion_enabled', Bool, driver).data
    ready = one('/alicia_d/feedback_ready', Bool, driver).data
    deadline = time.monotonic() + 5.
    while len(samples) < 20 and time.monotonic() < deadline:
        time.sleep(.025)
    with lock:
        samples = samples[-20:]
    now = rospy.Time.now().to_nsec()
    assert len(samples) == 20 and len({s['stamp_ns'] for s in samples}) == 20
    assert all(b['stamp_ns'] > a['stamp_ns'] for a, b in zip(samples, samples[1:]))
    names = samples[-1]['names']
    assert {'Joint%d' % i for i in range(1, 7)} <= set(names) and len(names) == len(set(names))
    assert all(s['caller'] == driver and s['frame'] == 'sdk_measured' and s['names'] == names
        and len(s['positions']) == len(names) and all(math.isfinite(q) for q in s['positions'])
        and epoch <= s['stamp_ns'] <= s['received_ns'] < s['stamp_ns'] + 200000000 for s in samples)
    spans = {n: max(s['positions'][i] for s in samples) - min(s['positions'][i] for s in samples)
             for i, n in enumerate(names)}
    assert max(spans.values()) <= .004 and 0 <= now - samples[-1]['stamp_ns'] < 200000000
    assert manual.data is True and manual._connection_header.get('latching') == '1'
    assert rospy.get_param('/gui/joint_direct_mode') is True
    assert not task.active and not any(mode.get(k) for k in ('active', 'pending', 'start_reserved'))
    assert all(not any(g.status in (0, 1, 6, 7) for g in a.status_list) for a in actions)
    assert enabled and ready and actuation.startswith('CONFIRMED:')
    selection = rospy.get_param('/grasp_mode/selection')
    assert selection['generation'] == 2
    identities()
    result.update(passed=True, recorded_ns=now, epoch_ns=epoch, selection=selection,
        gui_own_manual_ack=True, task_active=task.active, task_state=task.state,
        task_message=task.message, router=mode, actions_active=False, samples=samples,
        sample_spans_rad=spans, actuation_status=actuation, motion_enabled=enabled, feedback_ready=ready,
        accepted_processes_unchanged=True, automatic_attempt_authorized_by_this_receipt=False)
except BaseException as exc:
    result.update(error=repr(exc), traceback=traceback.format_exc())
finally:
    for sub in subscriptions:
        sub.unregister()
    (out / 'result.json').write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
print(json.dumps({k: v for k, v in result.items() if k not in ('samples', 'router')}, ensure_ascii=False))
raise SystemExit(0 if result['passed'] else 2)
