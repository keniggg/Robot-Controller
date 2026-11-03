#!/usr/bin/env python3
"""Read GEN59 terminal/manual SDK state. No control command or new attempt."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import time
import traceback

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
N = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z')
O = N / 'gen58_resume_r1/observation_pose_fix'
RELEASE = N / 'verification/gen59_release_manifest.json'
ACCEPTED = O / 'runtime_reload_r1/acceptance/result.json'
PINS = {
    RELEASE: '5fed4cbed7ea1e824eda3bb750657970dcc1ec2d711cd041eff1a3d482ef2c6c',
    ACCEPTED: 'b5b53dcf13f51d507187a4c71e7ac20977e3bbc1ed6b51025f30ed5aabfb1082',
    W / '.runtime/read_gen58_post_attempt_20261021.py': '5c3b5bc908838506637684771c051383b0e06405921b9725ff8beea4751ced8b',
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify_sources(source):
    for path, expected in PINS.items():
        assert sha(path) == expected, str(path)
    for path, expected in source.items():
        assert sha(W / path) == expected, path


def identities(nodes):
    observed = {}
    for name, expected in nodes.items():
        fields = (Path('/proc') / str(expected['pid']) / 'stat').read_text().rsplit(')', 1)[1].split()
        assert fields[0] != 'Z', name + ': zombie'
        row = dict(pid=expected['pid'], start_ticks=fields[19], parent_pid=int(fields[1]))
        assert row == expected, name + ': process identity changed'
        observed[name] = row
    return observed


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--expected-generation', type=int, required=True)
    args = ap.parse_args()
    assert args.expected_generation > 0
    assert Path('/media/zhuyupei/SanStand').is_mount()
    out = args.output.resolve()
    assert (O / 'actual_attempt_r1').resolve() in out.parents
    out.mkdir(exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(out / 'ros')
    result = dict(passed=False, motion_commands_sent=False, parameters_changed=False,
                  temperature_queried=False, torque_disable_commands_sent=False,
                  automatic_attempt_authorized_by_this_receipt=False,
                  source=str(Path(__file__).resolve()), source_sha256=sha(__file__),
                  expected_generation=args.expected_generation,
                  input_sha256={str(p): h for p, h in PINS.items()})
    subscriptions = []
    try:
        for path, expected in PINS.items():
            assert sha(path) == expected, str(path)
        release = json.loads(RELEASE.read_text())
        deployment = json.loads(ACCEPTED.read_text())
        assert release['generation'] == deployment['generation'] == 'GEN59'
        assert release['passed'] and release['installed'] and deployment['deployment_passed'] and deployment['whole_patch_deployed']
        assert deployment['source_manifest'] == str(RELEASE) and deployment['source_manifest_sha256'] == PINS[RELEASE]
        source = release['source_sha256']
        assert source == deployment['source_sha256'] and len(source) == 1094
        verify_sources(source)
        nodes = deployment['full_runtime_nodes']
        assert len(nodes) == 17 and nodes == deployment['final_all_registered_nodes']
        assert nodes['/remote_grasp6d_node'] == deployment['new_remote_identity']
        before = identities(nodes)
        import rospy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Header, String
        from actionlib_msgs.msg import GoalStatusArray
        from alicia_flexible_grasp_supervisor.msg import GraspState
        from alicia_grasp_modes.selection import parse_selection
        rospy.init_node('gen59_post_attempt_readonly', anonymous=True, disable_rosout=True)
        selection_before = parse_selection(rospy.get_param('/grasp_mode/selection'))
        assert selection_before['generation'] == args.expected_generation
        assert selection_before['mode'] == 'unknown' and selection_before['strategy'] == 'observe_return'
        driver = '/alicia_d_driver_node'
        topics = [
            ('joint', '/alicia_d/accepted_joint_states', JointState, driver),
            ('epoch', '/alicia_d/control_reference_epoch', Header, driver),
            ('gui', '/gui/joint_direct_mode', Bool, '/alicia_supervisor_gui'),
            ('task', '/grasp/state', GraspState, '/grasp_task_node'),
            ('mode', '/grasp_mode/status', String, '/grasp_mode_router'),
            ('actuation', '/alicia_d/actuation_status', String, driver),
            ('enabled', '/alicia_d/motion_enabled', Bool, driver),
            ('ready', '/alicia_d/feedback_ready', Bool, driver),
            ('arm', '/alicia_controller/follow_joint_trajectory/status', GoalStatusArray, None),
            ('hand', '/hand_controller/follow_joint_trajectory/status', GoalStatusArray, None),
        ]
        lock, samples, values = threading.Lock(), [], {}

        def receive(key, msg):
            received_ns = rospy.Time.now().to_nsec()
            with lock:
                values[key] = (msg, received_ns)
                if key == 'joint':
                    samples.append(dict(stamp_ns=msg.header.stamp.to_nsec(), received_ns=received_ns,
                                        caller=msg._connection_header.get('callerid'), frame=msg.header.frame_id,
                                        names=list(msg.name), positions=list(msg.position)))
                    samples[:] = samples[-80:]

        for key, topic, kind, _ in topics:
            subscriptions.append(rospy.Subscriber(topic, kind, lambda m, k=key: receive(k, m), queue_size=40))
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            with lock:
                complete = len(samples) >= 20 and len(values) == len(topics)
            if complete:
                break
            time.sleep(.025)
        # Refresh latched state deliveries; continuous subscriptions keep SDK and
        # both action heartbeat timestamps current during these reads.
        for key, topic, kind, _ in topics:
            if key not in ('joint', 'arm', 'hand'):
                receive(key, rospy.wait_for_message(topic, kind, timeout=3.0))
        selection = parse_selection(rospy.get_param('/grasp_mode/selection'))
        assert selection == selection_before
        assert rospy.get_param('/gui/joint_direct_mode') is True
        with lock:
            observed_samples = list(samples[-20:])
            states = {k: v[0] for k, v in values.items()}
            received_times = {k: v[1] for k, v in values.items()}
        now = rospy.Time.now().to_nsec()
        assert not rospy.is_shutdown() and len(states) == len(topics)
        assert all(0 <= now - t < 1_000_000_000 for t in received_times.values())
        for key, _, _, owner in topics:
            if owner is not None:
                assert states[key]._connection_header.get('callerid') == owner, key
        epoch = states['epoch'].stamp.to_nsec()
        assert epoch == deployment['epoch_ns'] and epoch > 0
        assert len(observed_samples) == 20 and len({s['stamp_ns'] for s in observed_samples}) == 20
        assert all(b['stamp_ns'] > a['stamp_ns'] for a, b in zip(observed_samples, observed_samples[1:]))
        names = observed_samples[-1]['names']
        assert {'Joint%d' % i for i in range(1, 7)} <= set(names) and len(names) == len(set(names))
        assert all(s['caller'] == driver and s['frame'] == 'sdk_measured' and s['names'] == names
                   and len(s['positions']) == len(names) and all(math.isfinite(q) for q in s['positions'])
                   and epoch <= s['stamp_ns'] <= s['received_ns'] < s['stamp_ns'] + 200_000_000
                   for s in observed_samples)
        spans = {n: max(s['positions'][i] for s in observed_samples) - min(s['positions'][i] for s in observed_samples)
                 for i, n in enumerate(names)}
        assert max(spans.values()) <= .004 and 0 <= now - observed_samples[-1]['stamp_ns'] < 200_000_000
        manual, task = states['gui'], states['task']
        mode = json.loads(states['mode'].data)
        assert manual.data is True and manual._connection_header.get('latching') == '1'
        assert not task.active and not any(mode.get(k) for k in ('active', 'pending', 'start_reserved'))
        action_evidence = {}
        for key in ('arm', 'hand'):
            msg = states[key]
            stamp = msg.header.stamp.to_nsec()
            assert 0 <= now - stamp <= 2_000_000_000
            assert not any(g.status in (0, 1, 6, 7) for g in msg.status_list)
            action_evidence[key] = dict(stamp_ns=stamp, received_ns=received_times[key],
                                        caller=msg._connection_header.get('callerid'),
                                        statuses=[g.status for g in msg.status_list])
        actuation = states['actuation'].data
        assert states['enabled'].data and states['ready'].data and actuation.startswith('CONFIRMED:')
        after = identities(nodes)
        verify_sources(source)
        result.update(passed=True, recorded_ns=now, epoch_ns=epoch, selection=selection,
                      gui_own_manual_ack=True, task_active=task.active, task_state=task.state,
                      task_message=task.message, router=mode, actions_active=False, actions=action_evidence,
                      samples=observed_samples, sample_count=20, sample_spans_rad=spans,
                      state_received_ns=received_times, latch_receipt_time_is_not_source_generation_time=True,
                      actuation_status=actuation, motion_enabled=True, feedback_ready=True,
                      full_runtime_nodes_before=before, full_runtime_nodes_after=after,
                      accepted_processes_unchanged=True, source_count=1094,
                      freshness_scope='Final sampled snapshot; no new attempt or motion authorization.')
    except BaseException as exc:
        result.update(error=repr(exc), traceback=traceback.format_exc())
    finally:
        for sub in subscriptions:
            sub.unregister()
        with (out / 'result.json').open('x') as f:
            json.dump(result, f, indent=2, ensure_ascii=False); f.write('\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ('samples', 'router')}, ensure_ascii=False))
    return 0 if result['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
