"""Select one repositioned perception target while manual and idle.

The sole writable ROS service is /grasp_mode/select. This script never starts
inference or a task, publishes a robot command, or changes driver ownership.
"""
import argparse
import ast
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import time
import xmlrpc.client

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M / '实验记录/Robot-Controller'
COMMIT_HELPER = '.runtime/run_reboot_realigned_grasp_20261010.py'
CONFIRMATION = 'call_8691c0cf7b8b4a1c96950f77777aef9b:0'
BUSY = (0, 1, 6, 7)
NAMES = ['Joint%d' % i for i in range(1, 7)]


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=False, allow_nan=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--expected-generation', type=int, required=True)
    parser.add_argument('--expected-epoch', type=int, required=True)
    for name in ('release', 'deployment'):
        parser.add_argument('--' + name, type=Path, required=True)
        parser.add_argument('--' + name + '-sha256', required=True)
    args = parser.parse_args()
    assert args.expected_generation > 0 and args.expected_epoch > 0
    assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
    out = args.output.resolve()
    assert E / 'ros_log' in out.parents and not out.exists()
    for path in (args.release, args.deployment):
        assert E in path.resolve(strict=True).parents
    assert sha(args.release) == args.release_sha256
    assert sha(args.deployment) == args.deployment_sha256
    release = json.loads(args.release.read_text())
    deployment = json.loads(args.deployment.read_text())
    assert release['passed'] is True and release['reviewed'] is True
    assert release['kind'] == 'gen28_retained_original_frame_box_qualification'
    assert release['source_count'] == len(release['source_sha256']) == 965
    assert release['test_summary'] == dict(tests=7067, failures=0, errors=0, skipped=0)
    assert deployment['deployment_passed'] is True and deployment['whole_patch_deployed'] is True
    assert Path(deployment['source_manifest']).resolve() == args.release.resolve()
    assert deployment['source_manifest_sha256'] == args.release_sha256
    assert deployment['source_sha256'] == release['source_sha256']
    expected_processes = dict(deployment['preserved_after'], **deployment['after'])
    assert len(expected_processes) == 11
    source_hashes = dict(release['source_sha256'])
    source_hashes[str(Path(__file__).resolve().relative_to(W))] = sha(__file__)

    def source_check():
        assert sha(args.release) == args.release_sha256
        assert sha(args.deployment) == args.deployment_sha256
        for name, expected in source_hashes.items():
            assert not Path(name).is_absolute() and '..' not in Path(name).parts
            assert sha(W / name) == expected, 'Source changed: ' + name

    source_check()
    out.mkdir()
    (out / 'ros').mkdir()
    (out / 'tmp').mkdir()
    os.environ.update(ROS_LOG_DIR=str(out / 'ros'), TMPDIR=str(out / 'tmp'), PYTHONDONTWRITEBYTECODE='1')
    import rospy
    from actionlib_msgs.msg import GoalStatusArray
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Header, String
    from alicia_flexible_grasp_supervisor.msg import GraspState
    from alicia_grasp_modes.srv import SetGraspMode
    from alicia_grasp_modes.selection import parse_selection, read_authoritative_selection
    from alicia_flexible_grasp.robot.observation_view_gateway import _camera_configuration_contract

    # Reuse the exact reviewed function without importing the grasp-launching main.
    helper_tree = ast.parse((W / COMMIT_HELPER).read_text())
    functions = [node for node in helper_tree.body
        if isinstance(node, ast.FunctionDef) and node.name == 'wait_for_selection_commit']
    assert len(functions) == 1
    namespace = dict(json=json, time=time)
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(W / COMMIT_HELPER), 'exec'), namespace)
    wait_for_selection_commit = namespace['wait_for_selection_commit']
    rospy.init_node('gen28_repositioned_target_selection_only', anonymous=True, disable_rosout=True)
    lock = threading.RLock()
    latest, received, events, samples, sdk_rows, faults = {}, {}, [], [], [], []
    request_started = False
    sdk_reference = None
    selection = None
    subscriptions = []
    result = dict(passed=False, selection_service_calls=0, selection_response_unknown=False,
        purpose='Perception reset for explicitly repositioned target; offline input capture follows separately.',
        user_clearance_confirmation_reference=CONFIRMATION,
        user_report='Hands and forearms removed; target adjustment stopped.',
        expected_generation=args.expected_generation, expected_epoch_ns=args.expected_epoch,
        release=str(args.release.resolve()), release_sha256=args.release_sha256,
        deployment=str(args.deployment.resolve()), deployment_sha256=args.deployment_sha256,
        source_sha256=source_hashes, commit_helper_source_sha256=source_hashes[COMMIT_HELPER],
        started_ns=time.time_ns(), publishers_created=False, motion_commands_sent=False,
        inference_start_requested=False, task_start_requested=False, enable_requested=False,
        manual_mode_changed=False, temperature_queried=False, execution_authority=False)

    def save(name, value):
        target = out / name
        temporary = target.with_name(target.name + '.pending')
        encoded = json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
        with temporary.open('w') as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target)

    def persist():
        with lock:
            result.update(events=deepcopy(events), joint_samples=deepcopy(samples),
                sdk_commands=deepcopy(sdk_rows), faults=deepcopy(faults))
            save('result.json', result)

    def fault(reason):
        faults.append(dict(reason=reason, received_ns=rospy.Time.now().to_nsec()))

    def receive(key, message):
        nonlocal sdk_reference
        wall = time.monotonic()
        now = rospy.Time.now().to_nsec()
        caller = (getattr(message, '_connection_header', {}) or {}).get('callerid', '')
        with lock:
            try:
                if key in ('q', 'sdk'):
                    names, positions = list(message.name), list(message.position)
                    assert len(names) == len(positions) == len(set(names)) and set(NAMES) <= set(names)
                    assert all(math.isfinite(value) for value in positions)
                    row = dict(stamp_ns=message.header.stamp.to_nsec(), frame_id=message.header.frame_id,
                        names=names, positions=positions, received_ns=now, caller_id=caller)
                    if key == 'q':
                        assert row['frame_id'] == 'sdk_measured' and caller == '/alicia_d_driver_node'
                        assert 0 <= now - row['stamp_ns'] <= 200_000_000
                        assert not samples or row['stamp_ns'] > samples[-1]['stamp_ns']
                        row['arm_positions_rad'] = [dict(zip(names, positions))[n] for n in NAMES]
                        for i, value in enumerate(row['arm_positions_rad']):
                            previous = [r['arm_positions_rad'][i] for r in samples]
                            assert not previous or max(previous+[value])-min(previous+[value]) <= .0025
                        samples.append(row)
                    else:
                        assert row['frame_id'] == 'sdk_transmitted' and caller == '/alicia_d_driver_node'
                        if sdk_reference is None:
                            sdk_reference = (names, positions)
                        assert names == sdk_reference[0]
                        assert max(abs(x-y) for x,y in zip(positions,sdk_reference[1])) <= 1e-12
                        sdk_rows.append(row)
                elif key == 'manual':
                    row = dict(value=bool(message.data), received_ns=now, caller_id=caller)
                    assert row['value'] is True and caller == '/alicia_supervisor_gui'
                elif key == 'epoch':
                    row = dict(epoch_ns=message.stamp.to_nsec(), received_ns=now, caller_id=caller)
                    assert row['epoch_ns'] == args.expected_epoch and caller == '/alicia_d_driver_node'
                elif key == 'task':
                    row = dict(active=bool(message.active), state=message.state, stage=message.stage,
                        message=message.message, stamp_ns=message.header.stamp.to_nsec(),
                        received_ns=now, caller_id=caller)
                    assert not row['active'] and caller == '/grasp_task_node'
                elif key == 'mode':
                    row = json.loads(message.data)
                    assert caller == '/grasp_mode_router'
                    assert not any(row.get(k) for k in ('active', 'pending', 'start_reserved'))
                    assert row.get('state') not in ('error', 'switch_queued', 'executing', 'execution_response_unknown')
                    selected = parse_selection(row)
                    assert selected['mode'] == 'unknown' and selected['strategy'] == 'observe_return'
                    assert selected['generation'] in ((args.expected_generation, args.expected_generation+1)
                        if request_started else (args.expected_generation,))
                    assert request_started or row.get('state') != 'switching'
                    row = dict(row, received_ns=now, caller_id=caller)
                else:
                    row = dict(received_ns=now, caller_id=caller,
                        statuses=[dict(status=int(s.status), goal_id=s.goal_id.id,
                            goal_stamp_ns=s.goal_id.stamp.to_nsec()) for s in message.status_list])
                    assert not any(s['status'] in BUSY for s in row['statuses'])
                latest[key], received[key] = row, wall
                if key not in ('q', 'sdk'):
                    events.append(dict(kind=key, **row))
            except Exception as exc:
                fault(key + ': ' + repr(exc))

    topics = [('q','/alicia_d/accepted_joint_states',JointState),
        ('sdk','/alicia_d/sdk_command',JointState),
        ('epoch','/alicia_d/control_reference_epoch',Header),
        ('manual','/gui/joint_direct_mode',Bool),('task','/grasp/state',GraspState),
        ('mode','/grasp_mode/status',String),
        ('arm','/alicia_controller/follow_joint_trajectory/status',GoalStatusArray),
        ('hand','/hand_controller/follow_joint_trajectory/status',GoalStatusArray)]
    required = {key for key,_,_ in topics} - {'sdk'}

    def check():
        with lock:
            assert not faults, 'Observed invalid or non-idle event: ' + repr(faults[:2])
            assert required <= set(latest), 'State topics incomplete'
            assert all(sub.get_num_connections() > 0 for sub in subscriptions)
            assert not rospy.is_shutdown()
            assert time.monotonic() - received['q'] < .2
            assert all(time.monotonic() - received[k] < 2.5 for k in ('mode','arm','hand'))
            assert 0 <= rospy.Time.now().to_nsec() - latest['q']['stamp_ns'] <= 200_000_000
            assert latest['manual']['value'] and not latest['task']['active']
            assert rospy.get_param('/gui/joint_direct_mode') is True
            if selection is not None:
                assert parse_selection(latest['mode']) == selection
            if samples:
                spans = [max(row['arm_positions_rad'][i] for row in samples) -
                         min(row['arm_positions_rad'][i] for row in samples) for i in range(6)]
                assert max(spans) <= .0025, 'Accepted joints moved during selection window'
            return deepcopy(latest)

    def wait_window(after_ns=0, timeout=6.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with lock:
                assert not faults, repr(faults[:2])
                rows = [row for row in samples if row['stamp_ns'] > after_ns]
                ready = required <= set(latest) and len(rows) >= 20
                if ready and rows[-1]['stamp_ns'] - rows[-20]['stamp_ns'] >= 1_500_000_000:
                    state = check()
                    return dict(state=state, first=rows[-20], last=rows[-1], count=20)
            time.sleep(.02)
        raise RuntimeError('Twenty fresh stable accepted samples unavailable')

    class TimeoutTransport(xmlrpc.client.Transport):
        def make_connection(self, host):
            connection = super().make_connection(host)
            connection.timeout = 2.
            return connection

    def process_check():
        observed = {}
        for name, expected in expected_processes.items():
            code, _message, uri = rospy.get_master().lookupNode(name)
            assert code == 1
            proxy = xmlrpc.client.ServerProxy(uri, transport=TimeoutTransport())
            code, _message, pid = proxy.getPid(rospy.get_name())
            assert code == 1 and pid == expected['pid'], name
            fields = Path('/proc/%d/stat' % pid).read_text().rsplit(')',1)[1].split()
            actual = dict(pid=pid, start_ticks=fields[19], parent_pid=int(fields[1]))
            assert actual == expected, name
            observed[name] = actual
        return observed

    def parameters():
        names = ('/grasp_mode/selection','/gui/joint_direct_mode','/grasp','/grasp_6d/remote',
                 '/unknown_perception','/gripper','/handeye','/camera')
        before = rospy.Time.now().to_nsec()
        values = {name:rospy.get_param(name) for name in names}
        stable = deepcopy(values)
        stable.pop('/grasp_mode/selection')
        stable['/camera'] = _camera_configuration_contract(stable['/camera'])
        return dict(read_start_ns=before, read_end_ns=rospy.Time.now().to_nsec(), values=values,
            stable_configuration_sha256=hashlib.sha256(canonical(stable).encode()).hexdigest())

    class CommitRosFacade:
        Subscriber = staticmethod(rospy.Subscriber)
        get_param = staticmethod(rospy.get_param)
        @staticmethod
        def is_shutdown():
            with lock:
                return bool(faults) or rospy.is_shutdown()

    try:
        subscriptions = [rospy.Subscriber(topic, kind, lambda msg,k=key:receive(k,msg), queue_size=100)
            for key,topic,kind in topics]
        result['processes_before'] = process_check()
        prior = read_authoritative_selection(rospy.get_master())
        assert prior['generation'] == args.expected_generation
        assert prior['mode'] == 'unknown' and prior['strategy'] == 'observe_return'
        result['prior_selection'] = prior
        params_before = parameters()
        save('parameters_before.json', params_before)
        result['before_window'] = wait_window()
        rospy.wait_for_service('/grasp_mode/select', timeout=3.)
        # GraspState is latched event state, not a heartbeat. Refresh it just
        # before the sole write while the persistent subscriber latches busy.
        immediate_task = rospy.wait_for_message('/grasp/state', GraspState, timeout=3.)
        receive('task', immediate_task)
        assert read_authoritative_selection(rospy.get_master()) == prior
        with lock:
            assert parse_selection(latest['mode']) == prior
        check()
        request_started = True
        result['selection_request_ns'] = rospy.Time.now().to_nsec()
        result['selection_service_calls'] = 1
        result['selection_response_unknown'] = True
        persist()
        response = rospy.ServiceProxy('/grasp_mode/select', SetGraspMode)('unknown','observe_return')
        result['selection_response_unknown'] = False
        result['selection_response'] = dict(success=response.success, message=response.message)
        persist()
        assert response.success is True and response.message == 'SWITCHING', 'Select was not an immediate switch'
        selection, status = wait_for_selection_commit(CommitRosFacade, prior)
        result['selection'], result['selection_commit_status'] = selection, status
        assert selection['generation'] == args.expected_generation+1
        assert read_authoritative_selection(rospy.get_master()) == selection
        check()
        commit_ns = rospy.Time.now().to_nsec()
        result['selection_commit_ns'] = commit_ns
        result['after_window'] = wait_window(after_ns=commit_ns)
        params_after = parameters()
        save('parameters_after.json', params_after)
        assert params_before['stable_configuration_sha256'] == params_after['stable_configuration_sha256']
        assert parse_selection(params_after['values']['/grasp_mode/selection']) == selection
        result['processes_after'] = process_check()
        source_check()
        check()
        assert read_authoritative_selection(rospy.get_master()) == selection
        result['passed'] = True
    except Exception as exc:
        result.update(passed=False, error=repr(exc))
    finally:
        for sub in subscriptions:
            sub.unregister()
        with lock:
            if faults:
                result.update(passed=False, error='Latched non-idle/source violation: ' + repr(faults[:2]))
            result.update(finished_ns=time.time_ns(), automatic_retry_performed=False,
                only_requested_write_service='/grasp_mode/select',
                perceptual_target_ready_verified=False, new_input_capture_required=True)
            persist()
        print(json.dumps({key:result.get(key) for key in ('passed','error','selection','selection_service_calls',
            'selection_response_unknown','motion_commands_sent','execution_authority')},ensure_ascii=False),flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
