#!/usr/bin/env python3
"""Read GEN60 mixed-runtime terminal/manual SDK state. No control command or new attempt."""
import argparse
import hashlib
import importlib.util
import sys
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
E60 = O / 'held_lattice_fix_r1'
RELEASE = N / 'verification/gen60_release_manifest.json'
ACCEPTED = E60 / 'runtime_reload_r1/acceptance/result.json'
RUNTIME = W / '.runtime/gen60_gateway_reload_common_20261022.py'
PINS = {W / '.runtime/read_gen59_post_attempt_20261022.py':
    '51a588cfc1ea13f5c9fba5e33ed3373503f1dd3b26011cebc5780abcfeee0a69'}
sys.dont_write_bytecode = True


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify_sources(source):
    for path, expected in PINS.items():
        assert sha(path) == expected, str(path)
    for path, expected in source.items():
        assert sha(W / path) == expected, path



def require(condition, reason):
    if not condition:
        raise RuntimeError(reason)


def read(path):
    return json.loads(Path(path).read_bytes())


def selection_identity(value):
    from alicia_grasp_modes.selection import parse_selection
    require(type(value) is dict and set(value)=={'mode','strategy','generation','stamp_ns'},
            'complete exact selection required')
    return parse_selection(value)


def returned_selection(entry, runner_exit, authorization, deployment, expected_generation):
    require(type(expected_generation) is int and expected_generation==4,
            'this one GEN60 attempt expects generation4')
    require(entry['kind']=='GEN60_remote_one_recorded_attempt_recorded'
        and entry['original_runner_returned'] is True and entry['gate_returned_to_original_mode_line'] is True
        and entry['automatic_retry'] is False and entry['new_motion_authority_created'] is False,
        'current original runner did not return after its guarded boundary')
    require(type(entry['runner_returncode']) is int and type(runner_exit['runner_returncode']) is int
        and entry['runner_returncode']==runner_exit['runner_returncode']
        and runner_exit['physical_success']=='unverified','runner return evidence differs')
    selected=selection_identity(runner_exit['selection'])
    prior=selection_identity(deployment['selection'])
    require(selected==selection_identity(authorization['selection'])
        and prior==selection_identity(authorization['previous_selection'])
        and selected['generation']==expected_generation==prior['generation']+1
        and selected['mode']==prior['mode']=='unknown'
        and selected['strategy']==prior['strategy']=='observe_return'
        and selected['stamp_ns']>prior['stamp_ns'], 'exact returned selection/predecessor differs')
    require(authorization['selection_response']['success'] is True
        and authorization['selection_commit']['latched_selection_matched'] is True,
        'selection commit was not acknowledged')
    return selected


def load_bindings(args):
    require(args.release.resolve(strict=True)==RELEASE and args.deployment.resolve(strict=True)==ACCEPTED,
            'unexpected GEN60 release/acceptance')
    supplied={RELEASE:args.release_sha256,ACCEPTED:args.deployment_sha256,RUNTIME:args.runtime_common_sha256,
              Path(__file__).resolve():args.helper_sha256,args.attempt_result.resolve(strict=True):args.attempt_result_sha256,
              args.runner_exit.resolve(strict=True):args.runner_exit_sha256}
    PINS.update(supplied)
    require(all(sha(p)==h for p,h in PINS.items()),'post-idle evidence/helper changed')
    deployment=read(ACCEPTED);release=read(RELEASE)
    require(deployment['source_manifest']==str(RELEASE) and deployment['source_manifest_sha256']==args.release_sha256
        and deployment['input_sha256'][str(RUNTIME)]==args.runtime_common_sha256,'accepted runtime/source differs')
    spec=importlib.util.spec_from_file_location('gen60_post_idle_runtime',RUNTIME)
    runtime=importlib.util.module_from_spec(spec);spec.loader.exec_module(runtime)
    ctx=runtime.accepted_context(ACCEPTED,args.deployment_sha256)
    require(ctx['acceptance']==deployment and ctx['release']==release,'validated GEN60 acceptance differs')
    for p,h in ctx['pins'].items():PINS[Path(p)]=h
    source=release['source_sha256'];require(len(source)==1097,'source closure differs')
    entry_path=args.attempt_result.resolve();exit_path=args.runner_exit.resolve()
    entry=read(entry_path);runner_exit=read(exit_path)
    require(entry_path.parent.parent==E60/'actual_attempt_r1' and entry_path.parent.name.startswith('scene_start_')
        and entry_path.name=='recorded_entry_result.json','unexpected current attempt entry path')
    record=Path(entry['record_dir']).resolve(strict=True)
    require(record.parent==Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records')
        and exit_path==record/'user_request_runner_exit.json','runner exit not bound to current record')
    require(entry['input_sha256'][str(RELEASE)]==args.release_sha256
        and entry['input_sha256'][str(ACCEPTED)]==args.deployment_sha256,'returned attempt was another deployment')
    auth_path=record/'authorization.json';authorization=read(auth_path);PINS[auth_path]=sha(auth_path)
    require(Path(authorization['session_dir']).resolve()==entry_path.parent,'record authorization scene differs')
    selected=returned_selection(entry,runner_exit,authorization,deployment,args.expected_generation)
    verify_sources(source)
    return runtime,ctx,release,deployment,selected,record


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
    for key in ('release','deployment','attempt-result','runner-exit'):
        ap.add_argument('--'+key,type=Path,required=True)
        ap.add_argument('--'+key+'-sha256',required=True)
    ap.add_argument('--runtime-common-sha256',required=True)
    ap.add_argument('--helper-sha256',required=True)
    args = ap.parse_args()
    assert args.expected_generation == 4
    assert Path('/media/zhuyupei/SanStand').is_mount()
    out = args.output.resolve()
    assert (E60 / 'actual_attempt_r1').resolve() in out.parents
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
        runtime,ctx,release,deployment,selected,record=load_bindings(args)
        source=release['source_sha256'];nodes=deployment['full_runtime_nodes']
        require(len(nodes)==17 and nodes==deployment['final_all_registered_nodes'], 'complete GEN60 inventory differs')
        before=identities(nodes)
        result.update(input_sha256={str(p):h for p,h in PINS.items()},record_dir=str(record),
            source_manifest=str(RELEASE),source_manifest_sha256=args.release_sha256,
            deployment=str(ACCEPTED),deployment_sha256=args.deployment_sha256,
            actual_returned_selection=selected,composite_runtime=deployment['composite_runtime'])
        import rospy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Header, String
        from actionlib_msgs.msg import GoalStatusArray
        from alicia_flexible_grasp_supervisor.msg import GraspState
        from alicia_grasp_modes.selection import parse_selection
        rospy.init_node('gen60_post_attempt_readonly', anonymous=True, disable_rosout=True)
        import rosgraph
        master=rosgraph.Master(rospy.get_name())
        graph_before=runtime.graph(ctx,rospy,master,gateway=nodes[runtime.GATEWAY])
        selection_before = selection_identity(rospy.get_param('/grasp_mode/selection'))
        require(selection_before==selected,'live selection is not exact returned attempt selection')
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
            ('arm', '/alicia_controller/follow_joint_trajectory/status', GoalStatusArray, '/bessica_d_hw_interface'),
            ('hand', '/hand_controller/follow_joint_trajectory/status', GoalStatusArray, '/bessica_d_hw_interface'),
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
        selection = selection_identity(rospy.get_param('/grasp_mode/selection'))
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
        graph_after=runtime.graph(ctx,rospy,master,gateway=nodes[runtime.GATEWAY],auxiliaries=graph_before['auxiliary_nodes'])
        require(graph_before==graph_after,'instance graph or publishers changed during post-idle sampling')
        for topic in ('/alicia_controller/follow_joint_trajectory/status','/hand_controller/follow_joint_trajectory/status'):
            require(graph_after['publishers'].get(topic)==['/bessica_d_hw_interface'],'action status owner differs')
        verify_sources(source)
        runtime.recheck(ctx,source)
        result.update(passed=True, recorded_ns=now, epoch_ns=epoch, selection=selection,
                      gui_own_manual_ack=True, task_active=task.active, task_state=task.state,
                      task_message=task.message, router=mode, actions_active=False, actions=action_evidence,
                      samples=observed_samples, sample_count=20, sample_spans_rad=spans,
                      state_received_ns=received_times, latch_receipt_time_is_not_source_generation_time=True,
                      actuation_status=actuation, motion_enabled=True, feedback_ready=True,
                      full_runtime_nodes_before=before, full_runtime_nodes_after=after,
                      accepted_processes_unchanged=True, source_count=1097,
                      ownership_before=graph_before,ownership_after=graph_after,
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
