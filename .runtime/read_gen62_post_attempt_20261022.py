#!/usr/bin/env python3
"""Read accepted GEN62 post-attempt idle state. No control command or new attempt."""
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
E61 = E60 / 'seed_wait_fix_r1'
F = E61 / 'contact_feedback_fix_r1'
ATTEMPTS = F / 'actual_attempt_r1'
RELEASE = N / 'verification/gen62_release_manifest.json'
ACCEPTED = F / 'runtime_reload_r2/acceptance/result.json'
RUNTIME = W / '.runtime/gen62_compute_runtime_r2_20261022.py'
RUNTIME_SHA = '610ad034d0f3846e37e76e93feaba7c107b4918b82d3df897a0600b44a08dd6e'
COMMON = W / '.runtime/gen62_compute_attempt_common_20261022.py'
COMMON_SHA = 'fe9d300548d62defd8123d24b21d096012186817a2208cf0f673b33b6c41e33f'
ENTRY = W / '.runtime/gen62_compute_attempt_entry_20261022.py'
ENTRY_SHA = 'bf12e9a66d663618c3a98b54c63a642549d090a05156499107b64615b795130c'
ORIGINAL = W / '.runtime/run_reboot_realigned_grasp_20261010.py'
ORIGINAL_SHA = 'cb7996419e32954b683fc4b148f54c9e467f2a8b6f259664fa2a5bb1d0d5ed11'
PINS = {RUNTIME: RUNTIME_SHA,
        COMMON: COMMON_SHA, ENTRY: ENTRY_SHA, ORIGINAL: ORIGINAL_SHA,
        W / '.runtime/read_gen61_post_attempt_20261022.py':
        '29435535f9b5dffe4b7930556b503462cb67b2be814881d92abae69787a7d9a1'}
sys.dont_write_bytecode = True


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify_sources(source):
    for path, expected in PINS.items():
        require(sha(path) == expected, 'pinned post-idle input changed: ' + str(path))
    for path, expected in source.items():
        relative = Path(path)
        require(not relative.is_absolute() and '..' not in relative.parts
                and sha(W / relative) == expected, 'release source changed: ' + path)


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
    """Only the original runner's exact acknowledged successor is permitted.

    This checks historical return/selection evidence. It does not renew the
    scene, identity, geometry or authorization deadlines for another action.
    """
    require(type(expected_generation) is int and expected_generation == 6,
            'this one GEN62 attempt expects generation6')
    require(entry['kind'] == 'GEN62_remote_one_recorded_attempt_recorded'
        and entry['original_runner_returned'] is True and entry['gate_returned_to_original_mode_line'] is True
        and entry['automatic_retry'] is False and entry['new_motion_authority_created'] is False
        and entry['original_guards_replaced'] is False and entry['temperature_queried'] is False
        and entry['physical_success'] == 'unverified' and entry['original_mode_line'] == 121
        and entry['helper'] == dict(path=str(ENTRY), sha256=ENTRY_SHA),
        'current original runner did not return after its guarded boundary')
    require(type(entry['runner_returncode']) is int and type(runner_exit['runner_returncode']) is int
        and entry['runner_returncode'] == runner_exit['runner_returncode']
        and runner_exit['physical_success'] == 'unverified', 'runner return evidence differs')
    selected = selection_identity(runner_exit['selection'])
    prior = selection_identity(deployment['selection'])
    require(selected == selection_identity(authorization['selection'])
        and prior == selection_identity(authorization['previous_selection'])
        and selected['generation'] == expected_generation == prior['generation'] + 1
        and selected['mode'] == prior['mode'] == 'unknown'
        and selected['strategy'] == prior['strategy'] == 'observe_return'
        and selected['stamp_ns'] > prior['stamp_ns'], 'exact returned selection/predecessor differs')
    require(authorization['selection_response']['success'] is True
        and authorization['selection_commit']['latched_selection_matched'] is True,
        'selection commit was not acknowledged')
    committed = authorization['selection_commit']['router_status']
    require(selection_identity({key: committed[key] for key in selected}) == selected,
            'router commit was not the exact returned selection')
    release_commit = authorization['continuation_release_commit']
    require(selection_identity(release_commit['selection']) == selected
        and release_commit['control_revision'] == 69
        and release_commit['policy'] == 'final_checked_authorization_commit_before_pipe_release'
        and authorization['operator_authorized_autonomous_grasp'] is True
        and authorization['recording_scope'] == 'local_only'
        and authorization['old_plan_reused'] is False
        and authorization['physical_lift_after_close'] is False
        and authorization['physical_success'] == 'unverified'
        and authorization['temperature_queried'] is False
        and authorization['torque_disable_authorized'] is False,
        'original selection release authorization differs')
    return selected


def scene_predecessor(authorization, preflight, deployment, scene, entry_inputs, common):
    """Bind the original capture's epoch/prior selection without reusing its age."""
    observation = authorization['operator_observation']
    preflight_path = scene / 'preflight.json'
    require(Path(authorization['session_dir']).resolve() == scene
        and observation['runtime_deployment'] == dict(path=str(ACCEPTED), sha256=PINS[ACCEPTED])
        and observation['release'] == dict(path=str(RELEASE), sha256=PINS[RELEASE])
        and observation['preflight'] == dict(path=str(preflight_path), sha256=entry_inputs[str(preflight_path)])
        and preflight['epoch_ns'] == deployment['epoch_ns']
        and selection_identity(preflight['selection']) == selection_identity(deployment['selection']),
        'captured predecessor/epoch or current scene authorization differs')
    continuing = authorization['continuing_authorization']
    require(continuing['path'] == str(common.A) and continuing['sha256'] == common.AUTH_SHA
        and observation['control_revision'] == 69 and observation['control_sha256'] == common.CONTROL_SHA,
        'continuing authorization/control binding differs')


def add_pin(path, digest):
    """An explicit caller pin may never replace an existing, different pin."""
    path = Path(path)
    require(type(digest) is str and len(digest) == 64
        and all(c in '0123456789abcdef' for c in digest), 'explicit SHA256 required')
    require(path not in PINS or PINS[path] == digest, 'conflicting post-idle input pin')
    PINS[path] = digest


def closed_record(args, entry, runner_exit):
    """Bind a sealed record, without claiming its result was a physical success."""
    record = args.record.resolve(strict=True)
    require(record.is_dir()
        and record.parent == Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records')
        and Path(entry['record_dir']).resolve(strict=True) == record
        and args.runner_exit.resolve(strict=True) == record / 'user_request_runner_exit.json'
        and args.authorization.resolve(strict=True) == record / 'authorization.json'
        and args.record_manifest.resolve(strict=True) == record / 'manifest.json',
        'entry/exit/authorization/manifest not bound to explicit record')
    for path, digest in ((args.attempt_result, args.attempt_result_sha256),
            (args.runner_exit, args.runner_exit_sha256),
            (args.authorization, args.authorization_sha256),
            (args.record_manifest, args.record_manifest_sha256)):
        require(sha(path) == digest, 'sealed record evidence changed: ' + str(path))
    manifest = read(args.record_manifest)
    require(manifest['schema'] == 1 and manifest['record_id'] == record.name
        and manifest['state'] == 'complete' and manifest['recorder_closed'] is True
        and type(manifest['completed_at']) is str and bool(manifest['completed_at'])
        and type(manifest['result_evidence']['runner_returncode']) is int,
        'record wrapper has not completed and sealed')
    # The recorder wrapper's code is distinct from the original runner's code.
    # Only entry versus runner_exit compare the latter; never conflate them.
    metadata = manifest['metadata']
    command = metadata['command']
    require(metadata['launcher'] == 'run_recorded_grasp.py'
        and type(command) is list and all(type(item) is str for item in command)
        and command.count(str(ENTRY)) == 1, 'record wrapper entry differs')
    flags = {'--release': str(RELEASE), '--release-sha256': args.release_sha256,
        '--deployment': str(ACCEPTED), '--deployment-sha256': args.deployment_sha256,
        '--common-sha256': COMMON_SHA, '--helper-sha256': ENTRY_SHA,
        '--phase': 'recorded', '--scene': str(args.attempt_result.resolve().parent)}
    require(all(command.count(key) == 1 and command.index(key)+1 < len(command)
        and command[command.index(key)+1] == value for key,value in flags.items()),
        'record wrapper command not bound to this scene/deployment')
    auth_rows = [row for row in manifest['files'] if row['path'] == 'authorization.json']
    require(len(auth_rows) == 1 and auth_rows[0]['sha256'] == args.authorization_sha256
        and auth_rows[0]['size_bytes'] == args.authorization.stat().st_size,
        'sealed authorization entry differs')
    return record


def load_bindings(args):
    require(args.release.resolve(strict=True) == RELEASE and args.deployment.resolve(strict=True) == ACCEPTED
        and args.runtime_common_sha256 == RUNTIME_SHA and args.common_sha256 == COMMON_SHA,
        'unexpected accepted GEN62 release/runtime')
    supplied = {Path(__file__).resolve(): args.helper_sha256,
        RELEASE: args.release_sha256, ACCEPTED: args.deployment_sha256,
        args.attempt_result.resolve(strict=True): args.attempt_result_sha256,
        args.runner_exit.resolve(strict=True): args.runner_exit_sha256,
        args.authorization.resolve(strict=True): args.authorization_sha256,
        args.record_manifest.resolve(strict=True): args.record_manifest_sha256}
    for path, digest in supplied.items():
        add_pin(path, digest)
    require(all(sha(p) == h for p, h in PINS.items()), 'post-idle evidence/helper changed')
    spec = importlib.util.spec_from_file_location('gen62_post_idle_common', COMMON)
    common = importlib.util.module_from_spec(spec); spec.loader.exec_module(common)
    # The pinned common validates the actual release, accepted three units,
    # preparation bundle, transition and retained fourteen process instances.
    # Its graph checks do not assert the pre-attempt selection generation.
    ctx = common.load_context(args, Path(__file__).resolve())
    release, deployment = ctx['release'], ctx['deployment']
    require(release['source_count'] == len(release['source_sha256']) == 1103,
            'accepted source closure differs')
    for path, digest in ctx['pins'].items():
        add_pin(Path(path), digest)
    entry_path, exit_path = args.attempt_result.resolve(), args.runner_exit.resolve()
    require(entry_path.parent.parent == ATTEMPTS and entry_path.parent.name.startswith('scene_start_')
        and entry_path.name == 'recorded_entry_result.json', 'unexpected current attempt entry path')
    entry, runner_exit = read(entry_path), read(exit_path)
    record = closed_record(args, entry, runner_exit)
    entry_inputs = entry['input_sha256']
    require(all(entry_inputs.get(str(path)) == digest for path, digest in ctx['pins'].items()
                if Path(path) != Path(__file__).resolve())
        and entry_inputs.get(str(ORIGINAL)) == ORIGINAL_SHA,
        'returned attempt was not this exact accepted deployment/original runner')
    auth_path = record / 'authorization.json'
    require(sha(auth_path) == args.authorization_sha256, 'record authorization changed')
    add_pin(auth_path, args.authorization_sha256)
    authorization = read(auth_path)
    preflight_path = entry_path.parent / 'preflight.json'
    require(sha(preflight_path) == entry_inputs[str(preflight_path)], 'original preflight changed')
    add_pin(preflight_path, entry_inputs[str(preflight_path)])
    scene_predecessor(authorization, read(preflight_path), deployment, entry_path.parent, entry_inputs, common)
    selected = returned_selection(entry, runner_exit, authorization, deployment, args.expected_generation)
    verify_sources(release['source_sha256'])
    return common, ctx, release, deployment, selected, record


def accepted_graph(common, ctx, rospy, master):
    deployment = ctx['deployment']
    graph = ctx['runtime'].graph(ctx['runtime_base'], ctx['runtime_context'], rospy, master,
        deployment['targets'], ctx['transition']['units'], deployment['auxiliary_nodes'])
    require(graph['node_identities'] == ctx['nodes'] and graph['services'] == deployment['after']['services'],
            'GEN62 three-unit or retained instance ownership changed')
    common.require_action_status_publishers(graph['publishers'], graph['node_identities'])
    return graph


def readonly_policy(rospy):
    # Configuration switches only. Never read a temperature value or subscribe
    # to telemetry for temperature. No parameter or service write is made.
    values = {name: rospy.get_param(name) for name in (
        '/alicia_d_driver_node/software_temperature_monitoring_enabled',
        '/alicia_d_driver_node/temperature_poll_rate_hz',
        '/grasp/require_actuation_confirmation',
        '/grasp/unknown_lift_after_close_enabled',
        '/grasp/unknown_observe_return_max_observations')}
    require(values['/alicia_d_driver_node/software_temperature_monitoring_enabled'] is False
        and type(values['/alicia_d_driver_node/temperature_poll_rate_hz']) in (int, float)
        and values['/alicia_d_driver_node/temperature_poll_rate_hz'] == 0.
        and values['/grasp/require_actuation_confirmation'] is True
        and values['/grasp/unknown_lift_after_close_enabled'] is False
        and type(values['/grasp/unknown_observe_return_max_observations']) is int
        and values['/grasp/unknown_observe_return_max_observations'] == 1,
        'temperature/confirmation/lift/observation policy changed')
    return values


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
    ap.add_argument('--record', type=Path, required=True)
    for key in ('release','deployment','attempt-result','runner-exit','authorization','record-manifest'):
        ap.add_argument('--'+key,type=Path,required=True)
        ap.add_argument('--'+key+'-sha256',required=True)
    ap.add_argument('--runtime-common-sha256',required=True)
    ap.add_argument('--helper-sha256',required=True)
    ap.add_argument('--common-sha256',required=True)
    args = ap.parse_args()
    require(not sys.flags.optimize, 'original assertion gates require unoptimized Python')
    require(args.expected_generation == 6, 'one exact generation5-to6 successor required')
    mount = Path('/media/zhuyupei/SanStand')
    require(mount.is_mount() and not os.statvfs(mount).f_flag & os.ST_RDONLY, 'writable SanStand required')
    out = args.output.resolve()
    require(ATTEMPTS.resolve() in out.parents and out.parent.exists()
            and out.parent.stat().st_dev == mount.stat().st_dev, 'output outside current external attempt')
    out.mkdir(exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(out / 'ros')
    result = dict(kind='GEN62_post_attempt_readonly', passed=False, motion_commands_sent=False, parameters_changed=False,
                  temperature_queried=False, torque_disable_commands_sent=False,
                  automatic_attempt_authorized_by_this_receipt=False,
                  source=str(Path(__file__).resolve()), source_sha256=sha(__file__),
                  expected_generation=args.expected_generation,
                  input_sha256={str(p): h for p, h in PINS.items()})
    subscriptions = []
    try:
        common,ctx,release,deployment,selected,record=load_bindings(args)
        source=release['source_sha256'];nodes=deployment['full_runtime_nodes']
        require(len(nodes)==17 and nodes==deployment['before']['node_identities']==deployment['after']['node_identities'],
                'complete GEN62 accepted inventory differs')
        before=identities(nodes)
        result.update(input_sha256={str(p):h for p,h in PINS.items()},record_dir=str(record),
            source_manifest=str(RELEASE),source_manifest_sha256=args.release_sha256,
            deployment=str(ACCEPTED),deployment_sha256=args.deployment_sha256,
            actual_returned_selection=selected, prior_accepted_selection=selection_identity(deployment['selection']),
            replaced_count=3, preserved_count=14, accepted_compute_units=ctx['transition']['units'],
            original_runner_exit=dict(path=str(args.runner_exit.resolve()), sha256=args.runner_exit_sha256),
            selection_authorization=dict(path=str(record/'authorization.json'), sha256=args.authorization_sha256),
            closed_record_manifest=dict(path=str(args.record_manifest.resolve()), sha256=args.record_manifest_sha256),
            deadline_refresh_performed=False)
        import rospy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Header, String
        from actionlib_msgs.msg import GoalStatusArray
        from alicia_flexible_grasp_supervisor.msg import GraspState
        rospy.init_node('gen62_post_attempt_readonly', anonymous=True, disable_rosout=True)
        import rosgraph
        master=rosgraph.Master(rospy.get_name())
        graph_before=accepted_graph(common,ctx,rospy,master)
        policy_before=readonly_policy(rospy)
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
        graph_after=accepted_graph(common,ctx,rospy,master)
        policy_after=readonly_policy(rospy)
        require(policy_before==policy_after, 'read-only policy changed during sampling')
        require(graph_before==graph_after,'instance graph or publishers changed during post-idle sampling')
        for topic in ('/alicia_controller/follow_joint_trajectory/status','/hand_controller/follow_joint_trajectory/status'):
            require(graph_after['publishers'].get(topic)==['/bessica_d_hw_interface'],'action status owner differs')
        verify_sources(source)
        common.recheck(ctx)
        result.update(passed=True, recorded_ns=now, epoch_ns=epoch, selection=selection,
                      gui_own_manual_ack=True, task_active=task.active, task_state=task.state,
                      task_message=task.message, router=mode, actions_active=False, actions=action_evidence,
                      samples=observed_samples, sample_count=20, sample_spans_rad=spans,
                      state_received_ns=received_times, latch_receipt_time_is_not_source_generation_time=True,
                      actuation_status=actuation, motion_enabled=True, feedback_ready=True,
                      full_runtime_nodes_before=before, full_runtime_nodes_after=after,
                      accepted_processes_unchanged=True, source_count=release['source_count'],
                      readonly_policy=policy_after,
                      ownership_before=graph_before,ownership_after=graph_after,
                      freshness_scope='Final sampled snapshot; no new attempt or motion authorization.')
    except BaseException as exc:
        result.update(error=repr(exc), traceback=traceback.format_exc())
    finally:
        for sub in subscriptions:
            sub.unregister()
        result['input_sha256']={str(p):h for p,h in PINS.items()}
        raw=(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False)+'\n').encode()
        with (out / 'result.json').open('xb') as f:
            f.write(raw); f.flush(); os.fsync(f.fileno())
        require((out/'result.json').read_bytes()==raw, 'post-idle receipt readback differs')
    print(json.dumps({k: v for k, v in result.items() if k not in ('samples', 'router')}, ensure_ascii=False))
    return 0 if result['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
