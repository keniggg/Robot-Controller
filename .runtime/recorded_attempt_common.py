"""Immutable, parameterized bindings for one recorded attempt; no ROS work on import.

Historical validation and current instance/source/configuration evidence are
separate roles. Current feedback is always collected again by capture/final gate.
"""
import argparse
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import xmlrpc.client

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M / '实验记录/Robot-Controller'
DRIVER = '/alicia_d_driver_node'
GUI = '/alicia_supervisor_gui'
ACTION_OWNER = '/bessica_d_hw_interface'
CAPTURE = W / '.runtime/capture_continuation_scene_20261012.py'
CAPTURE_SHA = '4ed0d920eb6283c0fd973e18cd35469577ff9fa2ef65b4c14211ea26fda37149'
CHECKER = W / '.runtime/verify_active_remote_readiness_20261020.py'
CHECKER_SHA = 'eaa51eaf5b636b74d9957798bbf3b0407a0d25d5c6b11097c02b22a5e97948e5'
PUBLISHERS = {
    '/supervisor/camera/color/image_raw': '/supervisor_camera',
    '/supervisor/camera/depth/image_raw': '/supervisor_camera',
    '/alicia_d/accepted_joint_states': DRIVER,
    '/alicia_d/control_reference_epoch': DRIVER,
    '/alicia_d/feedback_ready': DRIVER,
    '/alicia_d/motion_enabled': DRIVER,
    '/alicia_d/actuation_status': DRIVER,
    '/gui/joint_direct_mode': GUI,
    '/grasp/state': '/grasp_task_node',
    '/grasp_mode/status': '/grasp_mode_router',
    '/alicia_controller/follow_joint_trajectory/status': ACTION_OWNER,
    '/hand_controller/follow_joint_trajectory/status': ACTION_OWNER,
}


def require(value, reason):
    if not value:
        raise RuntimeError(reason)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes())


def ref(path):
    return dict(path=str(Path(path).resolve(strict=True)), sha256=sha(path))


def selection_identity(value):
    from alicia_grasp_modes.selection import parse_selection
    require(type(value) is dict and set(value) == {'mode', 'strategy', 'generation', 'stamp_ns'},
            'complete exact selection required')
    return parse_selection(value)


def same_selection(*values):
    require(len(values) >= 2, 'two selections required')
    identities = [selection_identity(v) for v in values]
    return all(v == identities[0] for v in identities[1:])


def storage(path):
    path = Path(path).resolve()
    require(M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY,
            'writable SanStand required; no system-disk fallback')
    require(E / 'ros_log' in path.parents and path.parent.exists()
            and path.parent.stat().st_dev == M.stat().st_dev,
            'output outside existing SanStand experiment session')
    return path


def new_json(path, value):
    path = storage(path)
    raw = (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n').encode()
    with path.open('xb') as stream:
        stream.write(raw); stream.flush(); os.fsync(stream.fileno())
    require(path.read_bytes() == raw, 'evidence readback differs')


def parser(description):
    require(not sys.flags.optimize, 'optimized Python would remove original assertion gates')
    result = argparse.ArgumentParser(description=description)
    result.add_argument('--contract', type=Path, required=True)
    result.add_argument('--contract-sha256', required=True)
    result.add_argument('--helper-sha256', required=True)
    result.add_argument('--common-sha256', required=True)
    return result


def checked_module(path, expected, name):
    require(sha(path) == expected, 'helper changed: ' + str(path))
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def file_identity(path):
    s = Path(path).stat()
    return dict(device=s.st_dev, inode=s.st_ino, size=s.st_size,
                mtime_ns=s.st_mtime_ns, ctime_ns=s.st_ctime_ns)


def process(pid):
    fields = (Path('/proc') / str(pid) / 'stat').read_text().rsplit(')', 1)[1].split()
    require(fields[0] != 'Z', 'process is a zombie')
    return dict(pid=int(pid), parent_pid=int(fields[1]), start_ticks=fields[19])


class Verifier:
    process = staticmethod(process)


def validate_values(contract, artifacts):
    """Pure cross-artifact validation; historical snapshots grant no freshness."""
    require(contract.get('schema') == 1 and contract.get('kind') == 'current_single_recorded_attempt',
            'single-attempt contract required')
    release, baseline, targeted = [artifacts[k] for k in ('release', 'historical_full', 'targeted')]
    require(all(release[k] is True for k in ('passed', 'reviewed', 'installed')),
            'reviewed installed release required')
    require(release['full_regression_passed'] == contract['validation_scope']['current_full_regression_passed']
            and release['baseline_full_regression'] == contract['refs']['historical_full']
            and release['actual_targeted_regression'] == contract['refs']['targeted'],
            'historical and current validation roles differ')
    require(targeted['passed'] is True and targeted['total_distinct_case_count'] == release['targeted_distinct_cases']
            and targeted['full_regression_passed'] == release['full_regression_passed'], 'targeted validation differs')
    require(baseline.get('passed') is True and baseline.get('full_regression_passed') is True
            and baseline.get('scope_complete') is True, 'historical complete baseline failed')
    accepted, registered, camera = [artifacts[k] for k in ('runtime_acceptance', 'camera_registered', 'camera_deployment')]
    require(accepted['passed'] is True and accepted['full_restart'] is True
            and accepted['source_manifest'] == contract['refs']['release'], 'full startup baseline differs')
    require(registered['passed'] is True and camera['passed'] is True
            and camera['non_camera_instances_preserved'] == 16
            and camera['control_reference_preserved'] is True
            and camera['original_geometric_and_execution_gates_preserved'] is True,
            'camera-only revision not accepted')
    require(camera['plan'] == contract['refs']['camera_plan']
            and camera['persistent_profile']['sha256'] == contract['current_file_sha256'].get(camera['persistent_profile']['path']),
            'camera revision/current profile binding differs')
    camera_source = artifacts['camera_source_start']
    require(camera_source['pid'] == registered['new_camera']['pid']
            and camera_source['plan_sha256'] == contract['refs']['camera_plan']['sha256']
            and camera_source['memory_attestation'] is False, 'camera source/start revision differs')
    snapshot = artifacts['runtime_snapshot']
    require(snapshot['all_expected_node_identities_match'] is True
            and snapshot['nodes_before'] == snapshot['nodes_after'], 'snapshot instances changed')
    rows = snapshot['nodes_after']
    require(len(rows) == len(accepted['nodes']) == 17 and set(rows) == set(accepted['nodes']),
            'complete runtime inventory differs')
    for name, row in rows.items():
        expected = registered['new_camera'] if name == '/supervisor_camera' else {
            k: accepted['nodes'][name][k] for k in ('pid', 'parent_pid', 'start_ticks')}
        require(row['identity'] == expected, 'accepted process changed: ' + name)
        if name == '/supervisor_camera':
            require(row['uri'] == registered['node_uri'], 'current camera URI differs')
        else:
            require(row['uri'] == accepted['nodes'][name]['uri']
                    and row['cmdline_sha256'] == accepted['nodes'][name]['cmdline_sha256'],
                    'accepted command/URI changed: ' + name)
    reference = contract['reference']
    require(reference['epoch_ns'] == accepted['state']['epoch_ns'] == registered['state']['epoch_ns']
            and same_selection(reference['selection'], snapshot['selection_before'], accepted['selection']),
            'accepted reference epoch/selection differs')
    remote = artifacts['remote_binding']
    require(remote['node_identity'] == dict(rows['/remote_grasp6d_node']['identity'],
            cmdline_sha256=rows['/remote_grasp6d_node']['cmdline_sha256']), 'remote instance binding differs')
    require(remote['parameters'] == artifacts['startup_configuration']['remote_parameters'],
            'remote construction configuration differs')
    twin = artifacts['local_twin']
    require(twin['passed'] is True and twin['service_properties_before'] == twin['service_properties_after']
            and twin['source_refs']['accepted_startup'] == contract['refs']['runtime_acceptance']
            and all(twin['consumers'][n] == rows[n] for n in twin['consumers']),
            'local twin construction/current consumer binding differs')
    require(artifacts['control']['state'] == 'active'
            and artifacts['control']['authorization_id'] == artifacts['authorization']['authorization_id'],
            'continuing authorization inactive or mismatched')
    require(artifacts['user_request']['source'] == 'actual_user_message'
            and bool(artifacts['user_request']['text'].strip()), 'actual current user request required')
    inventory = artifacts['source_inventory']
    require(inventory['release'] == contract['refs']['release'] and inventory['all_sha256_verified'] is True
            and set(inventory['files']) == set(release['source_sha256'])
            and all(v['sha256'] == release['source_sha256'][n] for n, v in inventory['files'].items()),
            'source inventory is not the complete release')
    require(inventory['camera_source_start'] == contract['refs']['camera_source_start']
            and {n:v['sha256'] for n,v in inventory['camera_sources'].items()} == camera_source['sources'],
            'camera source/library identity inventory differs')
    return rows


def load_context(args, helper):
    require(not sys.flags.optimize, 'optimized Python forbidden')
    require(sha(helper) == args.helper_sha256 and sha(__file__) == args.common_sha256,
            'attempt helper/common changed')
    contract_path = args.contract.resolve(strict=True)
    require(sha(contract_path) == args.contract_sha256, 'attempt contract changed')
    c = read(contract_path)
    require(c['workspace_root'] == str(W) and Path(c['output_root']).resolve().is_dir(), 'workspace/session differs')
    session = Path(c['output_root']).resolve(); storage(session / '_path_check')
    pins = {contract_path: args.contract_sha256, Path(helper).resolve(): args.helper_sha256,
            Path(__file__).resolve(): args.common_sha256, CAPTURE: CAPTURE_SHA, CHECKER: CHECKER_SHA}
    artifacts = {}
    for name, item in c['refs'].items():
        path = Path(item['path']).resolve(strict=True)
        require(sha(path) == item['sha256'], 'contract reference changed: ' + name)
        pins[path] = item['sha256']; artifacts[name] = read(path)
    for path, digest in c['helper_sha256'].items(): pins[Path(path)] = digest
    for path, digest in c['current_file_sha256'].items(): pins[Path(path)] = digest
    for reference in artifacts['local_twin']['source_refs'].values():
        pins[Path(reference['path'])] = reference['sha256']
    require(c['helper_sha256'].get(str(Path(helper).resolve())) == args.helper_sha256
            and c['helper_sha256'].get(str(Path(__file__).resolve())) == args.common_sha256,
            'entry/common not reviewed in contract')
    rows = validate_values(c, artifacts)
    observers = []
    for item in c.get('observers', []):
        path = Path(item['path']).resolve(strict=True)
        require(sha(path) == item['sha256'], 'observer declaration changed')
        pins[path] = item['sha256']; observer = read(path)
        require(observer['publishers'] == [] and observer.get('reviewed_for_current_automatic_attempt') is True
                and observer.get('motion_authority_granted') is False, 'readonly observer not reviewed or publishes')
        require(set(observer['services']) <= {observer['node'] + '/get_loggers', observer['node'] + '/set_logger_level',
                                             observer['node'] + '/tf2_frames'},
                'observer has nonstandard services')
        observers.append(observer)
    nodes = {n: row['identity'] for n, row in rows.items()}
    dep = dict(epoch_ns=c['reference']['epoch_ns'], selection=c['reference']['selection'],
               auxiliary_nodes=artifacts['runtime_acceptance']['auxiliary_nodes'])
    context = dict(contract=c, contract_path=contract_path, artifacts=artifacts, session_root=session,
        release=artifacts['release'], release_path=Path(c['refs']['release']['path']),
        deployment=dep, deployment_path=contract_path, nodes=nodes, runtime_rows=rows,
        pins=pins, observers=observers, verifier=Verifier(),
        binding_path=Path(c['refs']['remote_binding']['path']),
        startup=dict(boot_id=c['boot_id']), source_inventory=artifacts['source_inventory'],
        authorization_path=Path(c['refs']['authorization']['path']), control_path=Path(c['refs']['control']['path']),
        user_request_path=Path(c['refs']['user_request']['path']))
    require(c['boot_id'] == Path('/proc/sys/kernel/random/boot_id').read_text().strip(), 'boot changed')
    recheck(context)
    return context


def recheck(context):
    storage(context['session_root'] / '_path_check')
    require(all(sha(p) == h for p, h in context['pins'].items()), 'pinned attempt input changed')
    for name, entry in context['source_inventory']['files'].items():
        relative = Path(name)
        require(not relative.is_absolute() and '..' not in relative.parts
                and file_identity(W / relative) == entry['identity'], 'verified source identity changed: ' + name)
    for name, entry in context['source_inventory']['camera_sources'].items():
        require(file_identity(name) == entry['identity'], 'verified camera source/library identity changed: ' + name)
    config, policy = read(E / 'grasp_records/archive_config.json'), read(E / 'maintenance/storage_control.json')
    require(config['upload_enabled'] is False and config['delete_enabled'] is False
            and policy['archive_policy'] == 'local_only', 'storage policy changed')
    free = shutil.disk_usage(E).free
    require(free >= config['min_free_bytes'], 'recording space insufficient')
    return dict(free_bytes=free, min_free_bytes=config['min_free_bytes'])


class Transport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host); connection.timeout = 3.; return connection


def _node_row(master, name, caller):
    uri = master.lookupNode(name)
    response = xmlrpc.client.ServerProxy(uri, transport=Transport()).getPid(caller)
    require(response[0] == 1, 'node PID unavailable: ' + name)
    row = process(response[2])
    return dict(identity=row, uri=uri,
        cmdline_sha256=sha(Path('/proc') / str(row['pid']) / 'cmdline'))


def validate_observer_graph(observer, system, row):
    require(row == {k: observer[k] for k in ('identity', 'uri', 'cmdline_sha256')}, 'observer instance changed')
    for index, key in enumerate(('publishers', 'subscribers', 'services')):
        actual = sorted(topic for topic, owners in system[index] if observer['node'] in owners)
        require(actual == sorted(observer[key]), 'observer graph changed: ' + key)


def check_parameters(rospy, expected):
    """Read each namespace once; compare all declared leaves, permit runtime additions."""
    roots = {'/' + key.strip('/').split('/')[0] for key in expected}
    current = {root: rospy.get_param(root) for root in sorted(roots)}
    for key, value in expected.items():
        parts = key.strip('/').split('/'); observed = current['/' + parts[0]]
        for part in parts[1:]:
            require(type(observed) is dict and part in observed, 'configured parameter missing: ' + key)
            observed = observed[part]
        # ROS dynamic_reconfigure may expose an unchanged YAML integer as the
        # declared floating parameter type (observed max_safe_path_cost: 1 -> 1.0).
        numeric_equivalent = type(observed) in (int, float) and type(value) in (int, float)
        require(observed == value and (type(observed) is type(value) or numeric_equivalent),
                'current configuration differs: ' + key)


def check_local_twin(context):
    twin = context['artifacts']['local_twin']
    expected = twin['process_identity']; observed = process(expected['pid'])
    observed['cmdline_sha256'] = sha(Path('/proc') / str(expected['pid']) / 'cmdline')
    require(observed == expected, 'local twin process changed')
    output = subprocess.check_output(['systemctl', '--user', 'show', twin['unit'],
        '--property=ActiveState', '--property=SubState', '--property=MainPID', '--property=InvocationID',
        '--property=ControlGroup'], text=True, timeout=2.)
    properties = dict(line.split('=', 1) for line in output.splitlines() if '=' in line)
    require(properties == twin['service_properties_after'], 'local twin service invocation changed')


def camera_configuration_identity(value, stationary_policy=None):
    """Exclude only camera_node's changing per-frame diagnostic status.

    camera_node._publish_temporal_status updates this exact subtree with frame
    counters/times and regional qualification. Configuration enable switches,
    calibration, scale, filters, preset and all other metadata remain exact.
    This comparison grants no stationarity or perception qualification.
    """
    result = copy.deepcopy(value)
    profile = result.get('runtime_profile')
    if type(profile) is dict:
        status = profile.get('stationary_temporal')
        if status is not None:
            require(type(status) is dict and type(stationary_policy) is dict,
                    'present temporal status requires reviewed stable policy')
            required = {'alpha', 'delta_depth_units', 'persistence', 'stationary_window_sec',
                        'evidence_max_age_sec', 'visual_stationarity_required', 'visual_window_sec',
                        'visual_max_motion_full_resolution_px', 'owner_schema'}
            require(set(stationary_policy) == required, 'stable temporal policy fields differ')
            for key, expected in stationary_policy.items():
                actual = status.get(key)
                require(actual == expected and (type(expected) is not bool or type(actual) is bool),
                        'stable temporal policy differs: ' + key)
            configured = value['mode_depth_preset']
            require(status.get('enabled') is configured['stationary_temporal_enabled']
                    and status.get('regional_enabled') is configured['regional_stationary_enabled'],
                    'temporal enable/region status differs from configuration')
        profile.pop('stationary_temporal', None)
    return result


def live_identity(context, rospy, master, graph_wait=3.):
    system = master.getSystemState(); caller = rospy.get_name()
    names = {n for group in system for _, owners in group for n in owners} - {caller}
    observers_present = []
    for observer in context['observers']:
        if observer['node'] in names:
            validate_observer_graph(observer, system, _node_row(master, observer['node'], caller))
            require(all(sha(reference['path']) == reference['sha256']
                        for reference in [observer['source']] + observer.get('source_dependencies', [])),
                    'present observer source changed')
            names.remove(observer['node']); observers_present.append(observer['node'])
    auxiliary = context['deployment']['auxiliary_nodes']
    require(names == set(context['nodes']) | set(auxiliary), 'runtime graph differs: ' + repr(sorted(names ^ (set(context['nodes']) | set(auxiliary)))))
    rows = {n: _node_row(master, n, caller) for n in sorted(context['nodes'])}
    require(rows == context['runtime_rows'], 'current accepted node/URI/command instance changed')
    require(process(master.getPid()) == context['contract']['master_process'], 'ROS master instance changed')
    for name, expected in auxiliary.items():
        row = _node_row(master, name, caller)
        require(all(row['identity'][k] == expected[k] for k in ('pid', 'parent_pid', 'start_ticks'))
                and row['uri'] == expected['uri'], 'gateway auxiliary instance changed')
    publishers = {t: [n for n in owners if n != caller and n not in observers_present]
                  for t, owners in system[0]}
    require(all(publishers.get(t) == [n] for t, n in PUBLISHERS.items()), 'current publisher ownership differs')
    actual_services = {t: sorted(n for n in owners if n in context['nodes'] or n in auxiliary)
                       for t, owners in system[2] if any(n in context['nodes'] or n in auxiliary for n in owners)}
    require(actual_services == context['contract']['expected_services'], 'current service ownership differs')
    policy = context['contract']['camera_stationary_policy']
    require(camera_configuration_identity(rospy.get_param('/camera'), policy) ==
            camera_configuration_identity(context['artifacts']['camera_registered']['camera_configuration'], policy),
            'current camera configuration differs')
    check_parameters(rospy, context['contract']['policy_parameters'])
    check_local_twin(context)
    require(same_selection(rospy.get_param('/grasp_mode/selection'), context['deployment']['selection']),
            'current selection changed before attempt')
    return dict(nodes={n:r['identity'] for n,r in rows.items()}, current_rows=rows,
                services=actual_services, publishers={t:publishers[t] for t in PUBLISHERS},
                auxiliary_nodes=auxiliary,
                declared_readonly_observers=[o['node'] for o in context['observers']])


def input_receipts(context):
    return {str(path): digest for path, digest in context['pins'].items()}
