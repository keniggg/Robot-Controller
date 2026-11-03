"""Read-only current idle baseline for the reviewed GEN40 compute reload."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import threading
import time
import xmlrpc.client

from verify_gen40_offline_tests_20261017 import W, E, M, BASE_SHA, sha, guard, write


class Transport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 3.
        return connection


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    for name in ('output', 'release', 'previous', 'control'):
        ap.add_argument('--'+name, required=True, type=Path)
    a = ap.parse_args()
    guard(a.output)
    assert not a.output.exists()
    a.output.mkdir()
    pinned = [(p, p.read_bytes()) for p in (a.release, a.previous, a.control)]
    input_sha = {str(p.resolve()): hashlib.sha256(raw).hexdigest() for p, raw in pinned}
    release, previous, control = [json.loads(raw) for _, raw in pinned]
    assert release['passed'] and release['reviewed'] and release['installed']
    assert release['baseline_release_sha256'] == BASE_SHA
    assert previous['whole_patch_deployed'] and previous['deployment_passed']
    assert previous['source_manifest_sha256'] == BASE_SHA
    assert control['state'] in ('active', 'paused') and type(control['revision']) is int
    expected = dict(previous['after'], **previous['preserved_after'])
    assert len(expected) == 11

    def sources():
        assert all(sha(W/n) == d for n, d in release['source_sha256'].items())
        assert all(sha(p) == d for p, d in release['evidence_sha256'].items())

    sources()
    os.environ['ROS_LOG_DIR'] = str(a.output/'ros')
    import rosgraph
    import rospy
    from actionlib_msgs.msg import GoalStatusArray
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header, String
    from alicia_flexible_grasp_supervisor.msg import GraspState
    rospy.init_node('gen40_readonly_reload_preflight', anonymous=True, disable_rosout=True)
    master = rosgraph.Master(rospy.get_name())

    def identities():
        actual = {}
        for name in expected:
            code, _, pid = xmlrpc.client.ServerProxy(
                master.lookupNode(name), transport=Transport()).getPid(rospy.get_name())
            assert code == 1
            fields = (Path('/proc')/str(pid)/'stat').read_text().rsplit(')', 1)[1].split()
            assert fields[0] != 'Z'
            actual[name] = dict(pid=pid, start_ticks=fields[19], parent_pid=int(fields[1]))
        assert actual == expected
        return actual

    identities()
    selection_before = rospy.get_param('/grasp_mode/selection')
    epoch_before = rospy.wait_for_message('/alicia_d/control_reference_epoch',
                                         Header, timeout=5.).stamp.to_nsec()
    lock = threading.Lock()
    values, samples = {}, []

    def receive(key, msg):
        with lock:
            values[key] = msg
            if key == 'joint':
                samples.append(msg)

    topics = [('joint', '/alicia_d/accepted_joint_states', JointState),
        ('epoch', '/alicia_d/control_reference_epoch', Header),
        ('task', '/grasp/state', GraspState), ('mode', '/grasp_mode/status', String),
        ('arm', '/alicia_controller/follow_joint_trajectory/status', GoalStatusArray),
        ('hand', '/hand_controller/follow_joint_trajectory/status', GoalStatusArray)]
    subscriptions = [rospy.Subscriber(topic, kind,
        lambda msg, key=key: receive(key, msg), queue_size=30) for key, topic, kind in topics]
    try:
        end = time.monotonic()+8.
        while time.monotonic() < end:
            with lock:
                if len(values) == len(topics) and len(samples) >= 20:
                    break
            time.sleep(.05)
        with lock:
            state, history = dict(values), list(samples)
        assert len(state) == len(topics) and len(history) >= 20
        q = state['joint']
        assert q.header.frame_id == 'sdk_measured'
        assert 0 <= (rospy.Time.now()-q.header.stamp).to_sec() < .2
        assert not state['task'].active
        mode = json.loads(state['mode'].data)
        assert not any(mode.get(k) for k in ('active', 'start_reserved', 'pending'))
        assert all(not any(g.status in (0, 1, 6, 7) for g in state[k].status_list)
                   for k in ('arm', 'hand'))
        assert rospy.get_param('/gui/joint_direct_mode') is True
        assert rospy.get_param('/grasp/unknown_observe_return_max_observations') == 1
        assert rospy.get_param('/grasp/unknown_lift_after_close_enabled') is False
        assert rospy.get_param('/alicia_d_driver_node/software_temperature_monitoring_enabled') is False
        assert rospy.get_param('/alicia_d_driver_node/temperature_poll_rate_hz') == 0.
        sources()
        node_identities = identities()
        assert json.loads(a.control.read_text()) == control
        assert all(sha(path) == digest for path, digest in input_sha.items())
        assert rospy.get_param('/grasp_mode/selection') == selection_before
        assert state['epoch'].stamp.to_nsec() == epoch_before
        assert rospy.wait_for_message('/alicia_d/control_reference_epoch',
                                     Header, timeout=5.).stamp.to_nsec() == epoch_before
        result = dict(passed=True, recorded_ns=rospy.Time.now().to_nsec(),
            source_manifest=str(a.release.resolve()), source_manifest_sha256=input_sha[str(a.release.resolve())],
            previous_deployment=str(a.previous.resolve()), previous_deployment_sha256=input_sha[str(a.previous.resolve())],
            input_sha256=input_sha,
            node_identities=node_identities, selection=selection_before,
            manual_mode=True, epoch_ns=state['epoch'].stamp.to_nsec(),
            names=list(q.name), positions=list(q.position), sample_count=len(history),
            task_active=False, actions_active=False, control_path=str(a.control.resolve()),
            control_snapshot=control, motion_commands_sent=False, temperature_queried=False,
            control_sha256=input_sha[str(a.control.resolve())],
            scene_permission=False)
        write(a.output/'result.json', result)
        print(json.dumps(dict(passed=True, output=str(a.output)), ensure_ascii=False))
    finally:
        for sub in subscriptions:
            sub.unregister()


if __name__ == '__main__':
    main()
