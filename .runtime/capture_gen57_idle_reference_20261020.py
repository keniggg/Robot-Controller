"""Read-only acceptance of a resumed SDK reference, without motion authority."""
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import time
import xmlrpc.client

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N = E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
OUTPUT = N/'gen57_preparation_r1/reference_r1'
RELEASE = N/'verification/gen56_release_manifest.json'
DEPLOYMENT = N/'gen56_runtime/gen56_runtime_deployment.json'
CONTROL = E/'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/control.json'
PINS = {
    RELEASE: '784af71745650269b1d114a5f9cbba32eaa792f8cf2029825eb5a56d52a8f255',
    DEPLOYMENT: 'd02a3dde8f8b7d52b93ed57ce958a64526a68d11a912dd518435ffcd74995f85',
    CONTROL: '5d61243e74d4e88908a6893e579f10836f2a33829e8395fa4e86c85ad113f5fd',
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def require(value, reason):
    if not value:
        raise RuntimeError(reason)


def read(path):
    return json.loads(Path(path).read_bytes())


def save(path, data):
    with path.open('x') as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())


class Transport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 3.
        return connection


def main():
    require(E.parents[1].is_mount(), 'SanStand not mounted')
    require(not OUTPUT.exists(), 'new output required')
    require(all(sha(p) == h for p, h in PINS.items()), 'input changed')
    release, deployment, control = map(read, (RELEASE, DEPLOYMENT, CONTROL))
    require(control['state'] == 'active' and control['revision'] == 62, 'not resumed')
    source = release['source_sha256']
    require(len(source) == 1085 and source == deployment['source_sha256'], 'baseline differs')
    require(all(sha(W/p) == h for p, h in source.items()), 'live source differs')
    expected = dict(deployment['after'], **deployment['preserved_after'])
    require(len(expected) == 11, 'node inventory differs')
    OUTPUT.mkdir()
    os.environ['ROS_LOG_DIR'] = str(OUTPUT/'ros')
    import rosgraph
    import rospy
    from std_msgs.msg import Header, String, Bool
    from sensor_msgs.msg import JointState
    from actionlib_msgs.msg import GoalStatusArray
    from alicia_flexible_grasp_supervisor.msg import GraspState
    rospy.init_node('gen57_idle_reference_readonly', anonymous=True, disable_rosout=True)
    master = rosgraph.Master(rospy.get_name())

    def identities():
        actual = {}
        for name, row in expected.items():
            endpoint = xmlrpc.client.ServerProxy(master.lookupNode(name), transport=Transport())
            code, _, pid = endpoint.getPid(rospy.get_name())
            require(code == 1 and pid == row['pid'], 'node ownership differs: '+name)
            fields = (Path('/proc')/str(pid)/'stat').read_text().rsplit(')', 1)[1].split()
            actual[name] = dict(pid=pid, parent_pid=int(fields[1]), start_ticks=fields[19])
            require(fields[0] != 'Z' and actual[name] == row, 'node instance differs: '+name)
        return actual

    before_nodes = identities()
    selection_before = rospy.get_param('/grasp_mode/selection')
    epoch_before = rospy.wait_for_message('/alicia_d/control_reference_epoch', Header, timeout=5.).stamp.to_nsec()
    require(epoch_before > 0, 'invalid epoch')
    lock, values, samples = threading.Lock(), {}, []

    def receive(key, message):
        with lock:
            values[key] = (message, time.time_ns())
            if key == 'joint':
                samples.append(dict(stamp_ns=message.header.stamp.to_nsec(),
                    received_ns=time.time_ns(), frame_id=message.header.frame_id,
                    names=list(message.name), positions=list(message.position)))

    topics = (
        ('joint', '/alicia_d/accepted_joint_states', JointState),
        ('epoch', '/alicia_d/control_reference_epoch', Header),
        ('task', '/grasp/state', GraspState),
        ('mode', '/grasp_mode/status', String),
        ('arm', '/alicia_controller/follow_joint_trajectory/status', GoalStatusArray),
        ('hand', '/hand_controller/follow_joint_trajectory/status', GoalStatusArray),
        ('feedback_ready', '/alicia_d/feedback_ready', Bool),
        ('actuation_status', '/alicia_d/actuation_status', String),
        ('motion_enabled', '/alicia_d/motion_enabled', Bool),
    )
    subscriptions = [rospy.Subscriber(topic, kind, lambda message, k=key: receive(k, message),
        queue_size=40) for key, topic, kind in topics]
    result = dict(kind='GEN57_idle_reference_readonly_acceptance', passed=False,
        started_ns=time.time_ns(), helper_path=str(Path(__file__).resolve()), helper_sha256=sha(__file__),
        input_sha256={str(p): h for p, h in PINS.items()}, source_sha256=source,
        release_path=str(RELEASE), deployment_path=str(DEPLOYMENT), control_path=str(CONTROL),
        control_snapshot=control, historical_epoch_ns=deployment['epoch_ns'],
        scene_authorized=False, motion_authorized=False, motion_commands_sent=False,
        temperature_queried=False, joint_disable_commands_sent=False)
    try:
        end = time.monotonic()+8.
        while time.monotonic() < end:
            with lock:
                ready = len(values) == len(topics) and len(samples) >= 20
            if ready:
                break
            time.sleep(.025)
        after_nodes = identities()
        selection_after = rospy.get_param('/grasp_mode/selection')
        epoch_after = rospy.wait_for_message('/alicia_d/control_reference_epoch', Header, timeout=5.).stamp.to_nsec()
        manual = rospy.get_param('/gui/joint_direct_mode')
        with lock:
            observed, history = dict(values), list(samples)
        now = time.time_ns()
        result.update(recorded_ns=now, node_identities_before=before_nodes, node_identities_after=after_nodes,
            selection_before=selection_before, selection_after=selection_after,
            epoch_before_ns=epoch_before, epoch_after_ns=epoch_after, samples=history,
            sample_count=len(history), missing_topics=[k for k, _, _ in topics if k not in observed])
        require(not result['missing_topics'] and len(history) >= 20, 'incomplete observations')
        require(selection_before == selection_after, 'selection changed')
        require(epoch_before == epoch_after == observed['epoch'][0].stamp.to_nsec(), 'epoch changed')
        require(manual is True, 'not manual')
        task = observed['task'][0]
        mode = json.loads(observed['mode'][0].data)
        actions = {k: [g.status for g in observed[k][0].status_list] for k in ('arm', 'hand')}
        require(not task.active and not any(mode.get(k) for k in ('active', 'pending', 'start_reserved')), 'task/router busy')
        require(not any(s in (0, 1, 6, 7) for statuses in actions.values() for s in statuses), 'active action')
        require(all(0 <= now-observed[k][1] < 1_000_000_000 for k in ('arm', 'hand', 'mode')), 'state callbacks stale')
        require(observed['feedback_ready'][0].data is True, 'feedback not ready')
        names = history[-1]['names']
        require(names and len(set(names)) == len(names) and set('Joint%d' % i for i in range(1, 7)) <= set(names), 'joint layout invalid')
        require(len({s['stamp_ns'] for s in history}) >= 20, 'not 20 unique SDK samples')
        require(all(s['frame_id'] == 'sdk_measured' and s['names'] == names and
            len(s['positions']) == len(names) and all(math.isfinite(q) for q in s['positions']) and
            epoch_after <= s['stamp_ns'] <= s['received_ns'] and
            s['received_ns']-s['stamp_ns'] < 200_000_000 for s in history), 'invalid or stale SDK sample')
        require(0 <= now-history[-1]['stamp_ns'] < 200_000_000, 'latest feedback stale')
        spans = {name: max(s['positions'][i] for s in history)-min(s['positions'][i] for s in history)
            for i, name in enumerate(names)}
        require(all(v <= .004 for v in spans.values()), 'joint positions not stable')
        require(all(sha(p) == h for p, h in PINS.items()), 'input changed during observation')
        require(all(sha(W/p) == h for p, h in source.items()), 'source changed during observation')
        result.update(passed=True, source_count=len(source), node_identities=after_nodes,
            manual_mode=True, task_active=False, task_state=task.state, router_active=False,
            actions_active=False, action_statuses=actions, feedback_ready=True,
            actuation_status=observed['actuation_status'][0].data,
            motion_enabled=bool(observed['motion_enabled'][0].data),
            epoch_ns=epoch_after, selection=selection_after, sample_spans=spans,
            unique_sample_count=len({s['stamp_ns'] for s in history}),
            latest_feedback_age_ns=now-history[-1]['stamp_ns'])
    except BaseException as exc:
        result['error'] = repr(exc)
        raise
    finally:
        save(OUTPUT/'result.json', result)
        for sub in subscriptions:
            sub.unregister()
        rospy.signal_shutdown('read-only reference capture finished')
    print(json.dumps({k: result[k] for k in ('passed', 'epoch_ns', 'sample_count',
        'unique_sample_count', 'manual_mode', 'actuation_status', 'motion_enabled')}, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
