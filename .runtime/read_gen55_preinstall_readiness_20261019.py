"""Read GEN54 idle/source/feedback readiness before GEN55 installation.

Only root runs this observer. It grants no new grasp, changes no control and
does not query temperatures. Imports and subscriptions occur only in main().
"""
import argparse
import hashlib
import json
import math
import os
import shutil
import threading
import time
from pathlib import Path

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M/'实验记录/Robot-Controller'
N = E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
RELEASE = N/'verification/gen54_release_manifest.json'
RELEASE_SHA = 'ec0d1c1d2ebdb88fc9c77174790190dfff71024e58fea6958b356cb9ebf5d3f8'
DEPLOYMENT = E/'ros_log/full_validated_gen54_restart_20261020T012700Z/gen54_current_runtime_identity.json'
DEPLOYMENT_SHA = '7e256ee6f5cb64756d40e4c2b07468a791d10704c81d4154144b141a11f23bd3'
CONTROL = E/'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/control.json'
CONTROL_EXPECTED_SHA = 'c9d29dac7b6caadcef967e497fe0bf628a2b6fcec642d48ca13713722020a0f0'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes())


def process_matches(row):
    fields = (Path('/proc')/str(row['pid'])/'stat').read_text().rsplit(')', 1)[1].split()
    return (fields[0] != 'Z' and fields[19] == row['start_ticks']
            and int(fields[1]) == row['parent_pid'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--expected-control-revision', type=int, required=True)
    args = parser.parse_args()
    assert M.is_mount() and args.expected_control_revision == 58
    output = args.output.resolve()
    assert N in output.parents and not output.exists()
    assert sha(RELEASE) == RELEASE_SHA and sha(DEPLOYMENT) == DEPLOYMENT_SHA
    release, deployment, control = [read(path) for path in (RELEASE, DEPLOYMENT, CONTROL)]
    assert release['passed'] is True and release['installed'] is True and release['generation'] == 'GEN54'
    assert deployment['whole_patch_deployed'] is True and deployment['deployment_passed'] is True
    assert Path(deployment['source_manifest']).resolve(strict=True) == RELEASE.resolve(strict=True)
    assert deployment['source_manifest_sha256'] == RELEASE_SHA
    source = release['source_sha256']
    assert len(source) == release['source_count'] == 1075
    assert source == deployment['source_sha256']
    assert all(sha(W/path) == digest for path, digest in source.items())
    assert control['state'] == 'active' and type(control['revision']) is int
    assert control['revision'] == args.expected_control_revision == 58
    assert sha(CONTROL) == CONTROL_EXPECTED_SHA
    control_sha = sha(CONTROL)
    assert len(deployment['after']) == 4 and len(deployment['preserved_after']) == 7
    processes = dict(deployment['after'], **deployment['preserved_after'])
    assert len(processes) == 11 and all(process_matches(row) for row in processes.values())
    config, storage_control = [read(path) for path in
        (E/'grasp_records/archive_config.json', E/'maintenance/storage_control.json')]
    assert config['upload_enabled'] is False and config['delete_enabled'] is False
    assert storage_control['archive_policy'] == 'local_only'
    free = shutil.disk_usage(E).free
    assert free >= config['min_free_bytes']
    output.mkdir()
    os.environ['ROS_LOG_DIR'] = str(output/'ros')
    import rospy
    from actionlib_msgs.msg import GoalStatusArray
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header, String
    from alicia_flexible_grasp_supervisor.msg import GraspState

    rospy.init_node('gen55_preinstall_readonly', anonymous=True, disable_rosout=True)
    lock, values, samples = threading.Lock(), {}, []

    def receive(key, message):
        with lock:
            values[key] = message
            if key == 'joint':
                samples.append(message)

    topics = [('joint', '/alicia_d/accepted_joint_states', JointState),
        ('epoch', '/alicia_d/control_reference_epoch', Header),
        ('task', '/grasp/state', GraspState), ('mode', '/grasp_mode/status', String),
        ('arm_action', '/alicia_controller/follow_joint_trajectory/status', GoalStatusArray),
        ('hand_action', '/hand_controller/follow_joint_trajectory/status', GoalStatusArray)]
    subscriptions = [rospy.Subscriber(topic, kind, lambda message, k=key: receive(k, message), queue_size=20)
        for key, topic, kind in topics]
    try:
        end = time.monotonic()+8.
        while time.monotonic() < end:
            with lock:
                ready = len(values) == len(topics) and len(samples) >= 20
            if ready:
                break
            time.sleep(.05)
        with lock:
            state, history = dict(values), list(samples)
        assert len(state) == len(topics) and len(history) >= 20
        joint = state['joint']
        now = rospy.Time.now().to_nsec()
        assert all(message.header.frame_id == 'sdk_measured' for message in history)
        assert 0 <= now-joint.header.stamp.to_nsec() < 200_000_000
        assert len(joint.name) == len(joint.position) > 0 and len(set(joint.name)) == len(joint.name)
        assert all(message.name == joint.name and len(message.position) == len(joint.position)
            and all(math.isfinite(value) for value in message.position) for message in history)
        assert state['epoch'].stamp.to_nsec() == deployment['epoch_ns']
        assert len({message.header.stamp.to_nsec() for message in history}) >= 20
        assert all(message.header.stamp.to_nsec() >= deployment['epoch_ns'] for message in history)
        sample_spans = {name:max(message.position[i] for message in history)-
            min(message.position[i] for message in history) for i,name in enumerate(joint.name)}
        assert all(span <= .004 for span in sample_spans.values())
        assert not state['task'].active
        mode = json.loads(state['mode'].data)
        assert not any(mode.get(key) for key in ('active', 'start_reserved', 'pending'))
        assert all(not any(goal.status in (0, 1, 6, 7) for goal in state[key].status_list)
            for key in ('arm_action', 'hand_action'))
        assert rospy.get_param('/gui/joint_direct_mode') is True
        selection = rospy.get_param('/grasp_mode/selection')
        assert type(selection['generation']) is int and selection == deployment['selection']
        assert selection['mode'] == 'unknown' and selection['strategy'] == 'observe_return'
        assert rospy.get_param('/grasp/unknown_lift_after_close_enabled') is False
        assert rospy.get_param('/grasp/unknown_observe_return_max_observations') == 1
        assert rospy.get_param('/alicia_d_driver_node/software_temperature_monitoring_enabled') is False
        assert rospy.get_param('/alicia_d_driver_node/temperature_poll_rate_hz') == 0.0
        assert sha(RELEASE) == RELEASE_SHA and sha(DEPLOYMENT) == DEPLOYMENT_SHA
        assert all(sha(W/path) == digest for path, digest in source.items())
        assert all(process_matches(row) for row in processes.values())
        assert read(CONTROL) == control and sha(CONTROL) == control_sha
        assert rospy.get_param('/grasp_mode/selection') == selection
        assert rospy.get_param('/gui/joint_direct_mode') is True
        result = dict(passed=True, recorded_ns=now, names=list(joint.name),
            positions=list(joint.position), sample_count=len(history), epoch_ns=state['epoch'].stamp.to_nsec(),
            sample_position_spans=sample_spans, sample_stable_limit=.004,
            cold_start_position_used_as_current_reference=False,
            task_state=state['task'].state, task_active=False, actions_active=False, mode=mode, selection=selection,
            manual_mode=True, source_count=len(source), deployment_processes_unchanged=True,
            node_identities=processes, source_matches_operational_release=True, operational_script_override={},
            operational_validation=release['test_summary'], free_bytes=free, min_free_bytes=config['min_free_bytes'],
            release=str(RELEASE), release_sha256=RELEASE_SHA, deployment=str(DEPLOYMENT),
            deployment_sha256=DEPLOYMENT_SHA, control_path=str(CONTROL), control_snapshot=control,
            control_sha256=control_sha, helper_sha256=sha(__file__), motion_commands_sent=False,
            temperature_queried=False, new_grasp_authorized=False)
        with (output/'result.json').open('x') as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        print(json.dumps(dict(passed=True, result=str(output/'result.json'), sha256=sha(output/'result.json'))))
    finally:
        for subscription in subscriptions:
            subscription.unregister()


if __name__ == '__main__':
    main()
