"""Read current GEN57 ownership/idle state; --stop stops only its two bound units.

No motion, mode change, temperature read/query, or disable operation. A stop
requires the sealed GEN58 source set and a fresh current-instance remote check.
"""
import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import xmlrpc.client

sys.dont_write_bytecode = True
import gen57_cold_attempt_common_20261021 as old
import gen57_full_cold_restart_common_20261020 as cold
import verify_gen57_full_cold_restart_20261020 as v

W, E, N = old.W, old.E, old.N
S = N / 'gen58_resume_r1'
CONTINUE = S / 'user_continue_request.json'
CONTINUE_SHA = '55dd4a17729108de8964bb72692d7ee0a326c14715cf5b29e354cc659d533281'
FULL = N / 'verification/missing_depth_prior_full_r2/result.json'
FULL_SHA = '8a332657019ad0790a53e9a71bd5459cafd51d956b759dcfe00a89fdd7a477a9'
INDEPENDENT = FULL.parent / 'independent_verification.json'
INDEPENDENT_SHA = '1800bacf740ba45208966296de430ba663764e8ccf439c5bd11b04b8c996451e'
AUXILIARY = '/move_group_commander_wrappers_1792548324398432690'
PRIOR_PINS = {
    'startup.json': '19861cb6dbb9f1bf82869c517fcc266adf3575664353651a454a45356a265e2b',
    'startup_configuration.json': 'da307074579d7fb8af1f45c6e12d3957f95329e57206e75cc4cdd616d210b176',
    'acceptance/result.json': 'cb5a5fca99da4556dda93cfe037d83e9197e58c549cc7b9841351f23ba48dc25',
}


def require(ok, why):
    if not ok:
        raise RuntimeError(why)


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def read(p):
    return json.loads(Path(p).read_bytes())


def save(path, data):
    require(old.M.is_mount() and S in path.parents and path.parent.stat().st_dev == old.M.stat().st_dev,
            'Mounted external output required')
    raw = (json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n').encode()
    with path.open('xb') as f:
        f.write(raw)
        f.flush()
        os.fsync(f.fileno())
    require(path.read_bytes() == raw, 'Evidence read-back failed')


def source_context():
    cold.control_and_request()
    require(sha(CONTINUE) == CONTINUE_SHA and read(CONTINUE)['text'] == '继续处理', 'Current continuation changed')
    require(sha(FULL) == FULL_SHA and sha(INDEPENDENT) == INDEPENDENT_SHA, 'Regression evidence changed')
    full, independent = read(FULL), read(INDEPENDENT)
    require(full['passed'] and full['scope_complete'] and full['full_regression_passed'] and
            independent['passed'] and independent['total_cases'] == 9893 and independent['exact_case_union'], 'Regression incomplete')
    sources = full['source_sha256']
    require(len(sources) == 1089 and full['test_scope_files'] == 348, 'Candidate scope differs')
    v.source_check(sources)
    previous_release = read(N / 'verification/gen57_release_manifest.json')
    changed = sorted(k for k in sources if k in previous_release['source_sha256'] and sources[k] != previous_release['source_sha256'][k])
    added = sorted(set(sources) - set(previous_release['source_sha256']))
    require(changed == ['src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_instance.py',
                        'src/alicia_grasp_modes/tests/test_observation_arrival_probable_foreground.py'] and
            added == ['src/alicia_grasp_modes/tests/test_missing_depth_rgb_prior.py'] and
            not set(previous_release['source_sha256']) - set(sources), 'Unexpected release delta')
    cfg, policy = read(E / 'grasp_records/archive_config.json'), read(E / 'maintenance/storage_control.json')
    require(cfg['upload_enabled'] is False and cfg['delete_enabled'] is False and policy['archive_policy'] == 'local_only', 'Storage policy changed')
    usage = os.statvfs(old.M)
    require(not usage.f_flag & os.ST_RDONLY and usage.f_bavail * usage.f_frsize >= cfg['min_free_bytes'], 'External storage unavailable/full')
    return sources


def current_identity(context, rospy, master):
    """Bind the historical gateway's observed lazy roscpp alias, no wildcard."""
    dep, nodes = context['deployment'], context['nodes']
    core = v.properties(dep['unit'])
    require(core == dep['core_service'] and v.process(master.getPid()) == dep['master_process'], 'Core/master instance changed')
    deadline = time.monotonic() + 3.
    while True:
        graph = master.getSystemState()
        names = {name for group in graph for _, owners in group for name in owners} - {rospy.get_name()}
        if names == set(nodes) | {AUXILIARY}:
            break
        require(time.monotonic() < deadline, 'Runtime graph differs from 17 configured owners and exact bound gateway alias')
        time.sleep(.05)
    actual = {}
    for name in sorted(names):
        response = xmlrpc.client.ServerProxy(master.lookupNode(name), transport=old.Transport()).getPid(rospy.get_name())
        require(response[0] == 1, 'Missing node PID: ' + name)
        row = v.process(response[2])
        expected = nodes['/motion_gateway'] if name == AUXILIARY else nodes[name]
        require(row == expected, 'Node process ownership changed: ' + name)
        actual[name] = row
    configured = v.configured_nodes({n: actual[n] for n in nodes}, context['config'], core)
    publishers = dict(graph[0]); ownership = {}
    for topic, owner in old.PUBLISHERS.items():
        names = publishers.get(topic, [])
        require((owner in names if topic == '/gui/joint_direct_mode' else names == [owner]), 'Publisher owner changed: ' + topic)
        ownership[topic] = list(names)
    return dict(nodes={n: actual[n] for n in nodes}, auxiliary_nodes={AUXILIARY: actual[AUXILIARY]},
                configured_nodes=configured, publishers=ownership, core_service=core,
                master_process=dep['master_process'], gui_windows=v.gui_windows(nodes['/alicia_supervisor_gui']['pid']))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--stop', action='store_true')
    p.add_argument('--release', type=Path)
    p.add_argument('--release-sha256')
    args = p.parse_args()
    out = args.output.resolve()
    require(old.M.is_mount() and S in out.parents and not out.exists(), 'New mounted output required')
    out.mkdir(parents=True)
    os.environ['ROS_LOG_DIR'] = str(out / 'observer_ros')
    result = dict(passed=False, stop_requested=args.stop, stop_performed=False,
                  motion_commands_sent=False, mode_changes_sent=False, temperature_queried=False,
                  torque_disable_commands_sent=False, helper_sha256=sha(__file__))
    subscriptions = []
    try:
        sources = source_context()
        require(all(sha(old.S / p) == h for p, h in PRIOR_PINS.items()), 'Historical startup/deployment binding changed')
        dep_path, startup_path = old.S / 'acceptance/result.json', old.S / 'startup.json'
        dep, startup, config = read(dep_path), read(startup_path), read(old.S / 'startup_configuration.json')
        pins = {str(p.resolve()): sha(p) for p in (CONTINUE, FULL, INDEPENDENT, dep_path, startup_path, old.S / 'startup_configuration.json', Path(__file__))}
        require(dep['deployment_passed'] and dep['all_processes_replaced'] and dep['source_manifest_sha256'] == sha(N / 'verification/gen57_release_manifest.json'), 'Prior runtime evidence differs')
        require(all(sha(p) == h for p, h in startup['operational_source_sha256'].items()), 'Prior runtime helper changed')
        context = dict(nodes=dep['all_registered_nodes'], deployment=dep, verifier=v, config=config)
        if args.stop:
            require(args.release is not None and args.release_sha256 and sha(args.release) == args.release_sha256, 'Sealed release required before stop')
            release = read(args.release)
            require(release['generation'] == 'GEN58' and release['source_sha256'] == sources and
                    all(release[k] is True for k in ('passed', 'reviewed', 'installed')), 'New release invalid')
            require(all(sha(k) == h for k, h in release['evidence_sha256'].items()), 'New release evidence changed')
            pins[str(args.release.resolve())] = args.release_sha256
            result['release'] = dict(path=str(args.release.resolve()), sha256=args.release_sha256)
        import rosgraph
        import rospy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Header, String, Bool
        from actionlib_msgs.msg import GoalStatusArray
        from alicia_flexible_grasp_supervisor.msg import GraspState
        rospy.init_node('gen58_transition_readonly', anonymous=True, disable_rosout=True)
        master = rosgraph.Master(rospy.get_name())
        ownership_before = current_identity(context, rospy, master)
        units = {k: dict(unit=startup[k]['unit'], properties=v.properties(startup[k]['unit'])) for k in ('core', 'simulation')}
        require(units['core']['properties'] == dep['core_service'], 'Core unit differs')
        sim = units['simulation']['properties']; saved_sim = dep['simulation_service']
        require(units['simulation']['unit'] == dep['simulation_unit'] and
                int(sim['MainPID']) == saved_sim['pid'] and sim['InvocationID'] == saved_sim['invocation_id'] and
                sim['ControlGroup'] == saved_sim['control_group'], 'Simulation unit instance differs from accepted deployment')
        binding = cold.checker_module().load_binding(Path(dep['remote_dependency']['startup_binding']), dep['remote_dependency']['startup_binding_sha256'])
        health_dir = out / 'remote_readiness'; health_dir.mkdir()
        health = cold.checker_module().evaluate(binding, health_dir)
        save(health_dir / 'result.json', health)
        require(health['passed'] and health['dependency_ready'], 'Remote dependency not ready')
        lock, values, history = threading.Lock(), {}, []

        def receive(key, msg):
            with lock:
                if key == 'gui' and msg._connection_header.get('callerid') != '/alicia_supervisor_gui':
                    return
                values[key] = msg
                if key == 'joint':
                    history.append(msg)

        topics = [('joint', '/alicia_d/accepted_joint_states', JointState),
                  ('epoch', '/alicia_d/control_reference_epoch', Header),
                  ('task', '/grasp/state', GraspState), ('mode', '/grasp_mode/status', String),
                  ('gui', '/gui/joint_direct_mode', Bool), ('actuation', '/alicia_d/actuation_status', String),
                  ('motion', '/alicia_d/motion_enabled', Bool), ('feedback', '/alicia_d/feedback_ready', Bool),
                  ('arm', '/alicia_controller/follow_joint_trajectory/status', GoalStatusArray),
                  ('hand', '/hand_controller/follow_joint_trajectory/status', GoalStatusArray)]
        subscriptions = [rospy.Subscriber(topic, kind, lambda m, k=key: receive(k, m), queue_size=40) for key, topic, kind in topics]
        end = time.monotonic() + 8.
        while time.monotonic() < end:
            with lock:
                ready = len(values) == len(topics) and len({m.header.stamp.to_nsec() for m in history}) >= 20
            if ready:
                break
            time.sleep(.025)
        ownership_after = current_identity(context, rospy, master)
        with lock:
            states, samples = dict(values), list(history)
        require(len(states) == len(topics) and len({m.header.stamp.to_nsec() for m in samples}) >= 20, 'Missing state or 20 unique SDK samples')
        now = rospy.Time.now().to_nsec(); q = states['joint']; epoch = states['epoch'].stamp.to_nsec()
        require(len(q.name) >= 6 and len(set(q.name)) == len(q.name) and len(q.position) == len(q.name), 'Invalid joint layout')
        require(0 <= now - q.header.stamp.to_nsec() < 200_000_000 and epoch > 0, 'Stale feedback/invalid epoch')
        require(all(m.header.frame_id == 'sdk_measured' and m.name == q.name and len(m.position) == len(q.name) and
                    m.header.stamp.to_nsec() >= epoch and all(math.isfinite(x) for x in m.position) for m in samples), 'Invalid measured SDK feedback')
        spans = {n: max(m.position[i] for m in samples) - min(m.position[i] for m in samples) for i, n in enumerate(q.name)}
        require(all(x <= .004 for x in spans.values()), 'Joint movement during idle check')
        require(not states['task'].active and states['gui'].data is True and rospy.get_param('/gui/joint_direct_mode') is True, 'Task active or GUI not manual')
        router = json.loads(states['mode'].data)
        require(not any(router.get(k) for k in ('active', 'start_reserved', 'pending')), 'Router active')
        require(all(not any(g.status in (0, 1, 6, 7) for g in states[k].status_list) for k in ('arm', 'hand')), 'Active trajectory')
        require(rospy.get_param('/alicia_d_driver_node/software_temperature_monitoring_enabled') is False and
                rospy.get_param('/alicia_d_driver_node/temperature_poll_rate_hz') == 0., 'Temperature policy changed')
        require(ownership_before == ownership_after and all(v.properties(row['unit']) == row['properties'] for row in units.values()), 'Instance changed')
        require(source_context() == sources and all(sha(p) == h for p, h in pins.items()), 'Pinned input changed')
        result['before'] = dict(recorded_ns=now, epoch_ns=epoch, source_sha256=sources, node_identities=context['nodes'],
            ownership=ownership_after, units=units, selection=rospy.get_param('/grasp_mode/selection'), sample_count=len(samples),
            names=list(q.name), positions=list(q.position), sample_position_spans=spans, sample_stable_limit=.004,
            manual_mode=True, gui_owning_manual_ack=True, task_active=False, task_state=states['task'].state,
            router=router, actions_active=False, motion_enabled=states['motion'].data, feedback_ready=states['feedback'].data,
            actuation_status=states['actuation'].data, control_snapshot=read(old.CONTROL), control_sha256=sha(old.CONTROL))
        result['input_sha256'] = pins
        for s in subscriptions:
            s.unregister()
        subscriptions = []
        if args.stop:
            # The observed current epoch is allowed to differ from historical startup.
            # Only the two exact owning units are stopped; no actuator command is sent.
            save(out / 'stop_plan.json', result)
            result['stop_commands'] = []
            for key in ('core', 'simulation'):
                row = units[key]
                require(v.properties(row['unit']) == row['properties'], 'Unit changed immediately before stop')
                cold.control_and_request()
                command = ['systemctl', '--user', 'stop', row['unit']]
                done = subprocess.run(command, capture_output=True, text=True, timeout=45.)
                result['stop_commands'].append(dict(command=command, returncode=done.returncode, stdout=done.stdout, stderr=done.stderr))
                require(done.returncode == 0, 'Owned unit stop failed; no retries')
            import gen57_cold_install_guard_20261020 as guard
            result['cold_after'] = guard.cold_pair()
            result['stop_performed'] = True
        result['passed'] = True
    except BaseException as exc:
        result['error'] = repr(exc)
        raise
    finally:
        for s in subscriptions:
            s.unregister()
        save(out / 'result.json', result)
        print(json.dumps({k: result.get(k) for k in ('passed', 'stop_requested', 'stop_performed', 'error')}, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
