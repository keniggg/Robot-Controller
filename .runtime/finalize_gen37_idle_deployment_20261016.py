"""Verify idle compute/gateway reload receipts and publish GEN37 runtime pointers.

No motion, enable, controller, temperature or lifecycle interfaces are called.
The grasp authorization remains paused until a separate fresh scene is admitted.
"""
import argparse
import json
import os
from pathlib import Path
import time
import xmlrpc.client

from continuous_grasp_authorization_20261012 import atomic_json
from verify_gen37_offline_tests_20261016 import W, E, M, BASE_SHA, sha


def read(path):
    return json.loads(Path(path).read_text())


class Transport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 3.
        return connection


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('release', 'compute-deployment', 'gateway-deployment', 'previous', 'session'):
        parser.add_argument('--'+name, required=True, type=Path)
    args = parser.parse_args()
    N = args.session.resolve(strict=True)
    assert M.is_mount() and N.stat().st_dev == M.stat().st_dev
    N.relative_to(E/'ros_log')
    assert not (N/'gen37_runtime_deployment.json').exists()
    release, compute, gateway, previous = map(read, (
        args.release, args.compute_deployment, args.gateway_deployment, args.previous))
    assert release['passed'] and release['reviewed'] and release['installed']
    assert release['baseline_release_sha256'] == BASE_SHA
    assert previous['source_manifest_sha256'] == BASE_SHA
    assert previous['deployment_passed'] and previous['whole_patch_deployed']
    assert compute['deployment_passed'] and gateway['deployment_passed']
    source = release['source_sha256']
    assert compute['source_sha256'] == gateway['source_sha256'] == source

    def sources():
        assert all(sha(W/name) == value for name, value in source.items())
        assert all(sha(path) == value for path, value in release['evidence_sha256'].items())

    sources()
    baseline = dict(previous['after'], **previous['preserved_after'])
    assert dict(compute['before'], **compute['preserved_before']) == baseline
    assert compute['preserved_before'] == compute['preserved_after']
    intermediate = dict(compute['after'], **compute['preserved_after'])
    assert gateway['before'] == intermediate['/motion_gateway']
    assert gateway['preserved_before'] == gateway['preserved_after']
    assert gateway['preserved_before'] == {
        key:value for key, value in intermediate.items() if key != '/motion_gateway'}
    after = dict(compute['after'], **{'/motion_gateway':gateway['after']})
    preserve = {key:value for key, value in baseline.items() if key not in after}
    assert len(after) == 4 and len(preserve) == 7
    assert all(after[key] != baseline[key] for key in after)
    expected = dict(after, **preserve)
    assert len(expected) == 11
    control_path = E/'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/control.json'
    control = read(control_path)
    assert control['state'] == 'paused' and control['revision'] == 41
    os.environ['ROS_LOG_DIR'] = str(N/'deployment_observer_ros')
    import rosgraph
    import rospy
    from actionlib_msgs.msg import GoalStatusArray
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header, String
    from alicia_flexible_grasp_supervisor.msg import GraspState
    rospy.init_node('gen37_idle_deployment_readonly', anonymous=True, disable_rosout=True)
    master = rosgraph.Master(rospy.get_name())

    def identities():
        for name, row in expected.items():
            code, _, pid = xmlrpc.client.ServerProxy(
                master.lookupNode(name), transport=Transport()).getPid(rospy.get_name())
            assert code == 1 and pid == row['pid'], name
            fields = (Path('/proc')/str(pid)/'stat').read_text().rsplit(')', 1)[1].split()
            assert fields[0] != 'Z' and fields[19] == row['start_ticks'], name
            assert int(fields[1]) == row['parent_pid'], name

    def idle():
        assert rospy.get_param('/gui/joint_direct_mode') is True
        assert not rospy.wait_for_message('/grasp/state', GraspState, timeout=5.).active
        mode = json.loads(rospy.wait_for_message('/grasp_mode/status', String, timeout=5.).data)
        assert not any(mode.get(key) for key in ('active', 'start_reserved', 'pending'))
        for topic in ('/alicia_controller/follow_joint_trajectory/status',
                      '/hand_controller/follow_joint_trajectory/status'):
            message = rospy.wait_for_message(topic, GoalStatusArray, timeout=5.)
            assert not any(goal.status in (0, 1, 6, 7) for goal in message.status_list)

    identities()
    idle()
    epoch = rospy.wait_for_message('/alicia_d/control_reference_epoch', Header, timeout=5.).stamp.to_nsec()
    assert epoch == previous['feedback']['epoch_ns']
    assert epoch == gateway['feedback_before']['epoch_ns'] == gateway['feedback_after']['epoch_ns']
    assert rospy.get_param('/grasp_mode/selection') == gateway['selection'] == previous['selection']
    policy = {
        '/grasp/unknown_observe_return_max_observations':1,
        '/grasp/unknown_lift_after_close_enabled':False,
        '/alicia_d_driver_node/software_temperature_monitoring_enabled':False,
        '/alicia_d_driver_node/temperature_poll_rate_hz':0.0}
    assert all(rospy.get_param(key) == value for key, value in policy.items())
    q = rospy.wait_for_message('/alicia_d/accepted_joint_states', JointState, timeout=5.)
    assert q.header.frame_id == 'sdk_measured'
    assert 0 <= (rospy.Time.now()-q.header.stamp).to_sec() < .2
    positions = dict(zip(q.name, q.position))
    old = gateway['feedback_after']
    assert all(abs(positions[name]-value) <= .004 for name, value in zip(old['names'], old['positions']))
    sources()
    identities()
    idle()
    assert read(control_path) == control
    deployment = dict(deployment_passed=True, whole_patch_deployed=True,
        deployment_kind='idle_three_compute_and_gateway_reload_hardware_preserved',
        source_manifest=str(args.release.resolve()), source_manifest_sha256=sha(args.release),
        source_sha256=source, test_summary=release['test_summary'],
        before={key:baseline[key] for key in after}, after=after,
        preserved_before=preserve, preserved_after=preserve,
        unit=compute['unit'], previous_deployment=str(args.previous.resolve()),
        evidence_sha256={str(path.resolve()):sha(path) for path in (
            args.release, args.compute_deployment, args.gateway_deployment, args.previous)},
        manual_mode=True, task_active=False, actions_active=False,
        selection=gateway['selection'], epoch_ns=epoch, recorded_ns=time.time_ns(),
        motion_commands_sent=False, temperature_queried=False,
        torque_disable_commands_sent=False, physical_grasp_verified=False)
    deployment_path = N/'gen37_runtime_deployment.json'
    atomic_json(deployment_path, deployment)
    progress = dict(runtime_session=str(N), directory=str(N),
        status='gen37_deployed_manual_idle_pending_fresh_grasp_scene',
        deployment_passed=True, whole_patch_deployed=True, gen37_deployed=True,
        source_manifest=str(args.release.resolve()), source_manifest_sha256=sha(args.release),
        latest_deployment=str(deployment_path), current_test_result=release['actual_regression'],
        source_count=release['source_count'], test_scope_files=release['test_scope_files'],
        test_summary=release['test_summary'], deployed_node_identities=expected,
        control_state='paused', control_revision=41, manual_mode=True,
        task_active=False, actions_active=False, automatic_grasp_started=False,
        user_request='已经对准了目标物，可以开始抓取任务',
        pending_work='Fresh scene/readiness, explicit authorization resume, then one recorded observe_return grasp.',
        physical_grasp_verified=False, updated_ns=time.time_ns())
    atomic_json(N/'current_progress.json', progress)
    pointer = W/'.runtime/current_resumed_grasp_session.json'
    atomic_json(N/'previous_current_resumed_grasp_session.json', read(pointer))
    atomic_json(pointer, dict(directory=str(N), runtime_session=str(N),
        control_state='paused', control_revision=41, updated_ns=time.time_ns()))
    atomic_json(N/'deployment_finalization.json', dict(passed=True, recorded_ns=time.time_ns(),
        evidence_sha256={str(path):sha(path) for path in (
            deployment_path, N/'current_progress.json', Path(__file__))},
        automatic_grasp_started=False, physical_grasp_verified=False))
    print(json.dumps(dict(passed=True, deployment=str(deployment_path)), ensure_ascii=False))


if __name__ == '__main__':
    main()
