#!/usr/bin/env python3
"""Prepare, start once, or inspect the installed full stack in manual mode.

Use a new mounted-SanStand session. `plan` freezes the current release, user
instruction and actual roslaunch expansion; `start` dispatches the existing
simulation and full-stack entries; `accept` only observes their new instances.
No generation/PID/selection from a previous session is treated as current.
No operation starts a grasp, switches mode, queries temperature or disables a
joint. The existing reviewed driver retains its positive startup enable.
"""
import argparse
import base64
from collections import deque
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import sys
import threading
import time
import xmlrpc.client

sys.dont_write_bytecode = True
W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M/'实验记录/Robot-Controller'
ENTRY = W/'.runtime/start_full_latest_gen36_20261016.sh'
LAUNCH = W/'.runtime/full_latest_gen36_single_observation_20261016.launch'
REMOTE = W/'.runtime/verify_active_remote_readiness_20261020.py'
SIM = W/'.runtime/full_startup_candidate/candidate_simulation_startup.py'
DESKTOP = W/'.runtime/gen58_cold_install_guard_20261021.py'
BACKENDS = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/gen56_runtime/remote_startup_binding_r1/startup_binding.json'
CONTROL = E/'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/control.json'
DRIVER_BUILD = E/'ros_log/pending_actuation_drift_20261007T104410.504268Z/driver_build.json'
CLIENT = W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/remote_grasp6d_client.py'
HAND_EYE = W/'.runtime/candidate_models/expanded_metric_20260929/handeye_candidate_config.yaml'


def require(ok, why):
    if not ok:
        raise RuntimeError(why)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def ref(path):
    path = Path(path).resolve(strict=True)
    return dict(path=str(path), sha256=sha(path))


def read_ref(value):
    raw = Path(value['path']).read_bytes()
    require(hashlib.sha256(raw).hexdigest() == value['sha256'], 'reference changed: '+value['path'])
    return json.loads(raw)


def load(path):
    # Execute verified helper source; do not let an old pyc replace the helper.
    path = Path(path)
    module = type(sys)('_cold_'+path.stem)
    module.__file__ = str(path)
    module.__name__ = '_cold_'+path.stem
    exec(compile(path.read_bytes(), str(path), 'exec'), module.__dict__)
    return module


def storage(path):
    path = Path(path).resolve()
    require(M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY, 'writable SanStand required; no fallback')
    require(E/'ros_log' in path.parents, 'new session must be under SanStand ros_log')
    ancestor = path
    while not ancestor.exists():
        ancestor = ancestor.parent
    require(ancestor.stat().st_dev == M.stat().st_dev, 'output is not on mounted SanStand')
    archive = json.loads((E/'grasp_records/archive_config.json').read_bytes())
    policy = json.loads((E/'maintenance/storage_control.json').read_bytes())
    require(archive['upload_enabled'] is False and archive['delete_enabled'] is False
            and policy['archive_policy'] == 'local_only', 'storage policy changed')
    free = shutil.disk_usage(M).free
    require(free >= archive['min_free_bytes'], 'SanStand recording minimum not available')
    return dict(free_bytes=free, min_free_bytes=archive['min_free_bytes'], archive_policy='local_only')


def write(path, value):
    path = Path(path); storage(path)
    raw = (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode()
    with path.open('xb') as stream:
        stream.write(raw); stream.flush(); os.fsync(stream.fileno())
    require(path.read_bytes() == raw, 'evidence readback differs: '+str(path))
    return ref(path)


def release_evidence(release):
    """Preserve full-baseline versus current targeted evidence as distinct roles."""
    require(all(release.get(k) is True for k in ('passed', 'reviewed', 'installed')), 'release is not installed/reviewed')
    require(release.get('source_count') == len(release['source_sha256']), 'release source count differs')
    require(release.get('kind') == 'targeted_compute_release'
            and release.get('full_regression_passed') is False, 'this entry expects a truthful targeted release')
    full = read_ref(release['baseline_full_regression'])
    focus = read_ref(release['actual_targeted_regression'])
    require(full.get('passed') is True and full.get('full_regression_passed') is True
            and full.get('scope_complete') is True and full.get('test_bodies_executed') is True,
            'historical full regression incomplete')
    summary = full['summary']
    require(summary['tests'] > 0 and all(summary[k] == 0 for k in ('failures', 'errors', 'skipped')),
            'historical full regression failed')
    cases = focus['case_results']
    identities = [tuple(row['identity']) for row in cases]
    require(focus.get('passed') is True and focus.get('full_regression_passed') is False
            and focus.get('exact_case_inventory') is True and focus.get('duplicate_cases') == 0
            and focus.get('omitted_cases') == 0 and all(row['passed'] is True for row in cases)
            and len(set(identities)) == len(cases) == focus['total_distinct_case_count'] == release['targeted_distinct_cases'],
            'targeted case evidence differs/incomplete')
    changed = dict(focus['production_sha256'], **focus['current_test_sha256'])
    require(set(focus['production_sha256']) == set(release['changed_production_sources'])
            and set(focus['current_test_sha256']) == set(release['changed_test_sources']), 'targeted source scope differs')
    expected = dict(full['source_sha256'], **changed)
    require(expected == release['source_sha256'], 'release is not exact historical baseline plus tested delta')
    return dict(baseline_full=release['baseline_full_regression'], baseline_summary=summary,
                baseline_scope=release['baseline_full_scope'], targeted=release['actual_targeted_regression'],
                targeted_distinct_cases=len(cases), current_full_regression_passed=False)


def source_check(mapping):
    for name, digest in mapping.items():
        path = Path(name)
        require(not path.is_absolute() and '..' not in path.parts, 'invalid relative source')
        require(sha(W/path) == digest, 'installed source changed: '+name)


def configuration():
    import roslaunch
    cfg = roslaunch.config.load_config_default([str(LAUNCH)], 11311, verbose=False, assign_machines=False)
    nodes = {}
    for n in cfg.nodes+cfg.nodes_core:
        name = n.namespace.rstrip('/')+'/'+n.name
        require(name not in nodes and n.machine_name is None, 'unexpected node construction: '+name)
        nodes[name] = dict(package=n.package, executable=n.type, args=n.args,
                           launch_prefix=n.launch_prefix, respawn=n.respawn, required=n.required)
    params = {k:v.value for k,v in cfg.params.items()}
    policy = {'/gui/joint_direct_mode': True,
              '/alicia_d_driver_node/software_temperature_monitoring_enabled': False,
              '/alicia_d_driver_node/temperature_poll_rate_hz': 0.0,
              '/alicia_d_driver_node/self_check_poll_rate_hz': 0.0,
              '/alicia_d_driver_node/auto_torque_on_startup': True,
              '/camera/simulate': False, '/camera/fallback_to_simulation': False}
    require(all(params.get(k) == v and (type(v) is not bool or type(params[k]) is bool)
                for k,v in policy.items()), 'manual/temperature/real-camera startup policy differs')
    required = {'/supervisor_camera', '/alicia_supervisor_gui', '/alicia_d_driver_node',
                '/grasp_task_node', '/remote_grasp6d_node', '/unknown_tabletop_perception', '/motion_gateway'}
    require(required <= set(nodes) and not any('tactile' in n for n in nodes), 'full launch scope differs')
    loader = nodes['/controller_loader_once']
    require(loader['executable'] == 'controller_loader_once.py' and loader['args'].split() ==
            ['alicia_controller', 'hand_controller'], 'unexpected controller loader')
    check = load(REMOTE)
    remote = {k:params[k] for k in check.PARAMETERS}; check.validate_parameters(remote)
    require(re.findall(r'^export GRASP6D_URL=(\S+)$', ENTRY.read_text(), re.M) == [remote[check.URL]],
            'entry URL differs from expanded remote configuration')
    return dict(nodes=nodes, persistent_nodes=sorted(set(nodes)-{'/controller_loader_once'}),
                parameters=params, remote_parameters=remote,
                launch_source_sha256={str(Path(p).resolve()):sha(p) for p in cfg.roslaunch_files},
                launch=str(LAUNCH), entry=str(ENTRY), parameters_uploaded=False)


def process(pid):
    fields = (Path('/proc')/str(pid)/'stat').read_text().rsplit(')', 1)[1].split()
    require(fields[0] != 'Z', 'zombie process')
    return dict(pid=int(pid), start_ticks=fields[19], parent_pid=int(fields[1]))


def in_group(pid, group):
    return any(line.rsplit(':', 1)[-1] == group or line.rsplit(':', 1)[-1].startswith(group+'/')
               for line in (Path('/proc')/str(pid)/'cgroup').read_text().splitlines())


def properties(unit):
    raw = subprocess.check_output(['systemctl', '--user', 'show', unit,
        '--property=ActiveState,SubState,MainPID,InvocationID,ControlGroup'], text=True, timeout=4.)
    result = dict(line.split('=', 1) for line in raw.splitlines() if '=' in line)
    require(result['ActiveState'] == 'active' and result['SubState'] == 'running'
            and int(result['MainPID']) > 1 and len(result['InvocationID']) == 32, 'service not running: '+unit)
    return result


def device_inventory():
    alias = Path('/dev/alicia_arm'); device = alias.resolve(strict=True)
    require(alias.is_symlink() and stat.S_ISCHR(device.stat().st_mode), 'arm device alias invalid')
    raw = subprocess.check_output(['udevadm', 'info', '--query=property', '--name='+str(device)], text=True, timeout=4.)
    props = dict(line.split('=', 1) for line in raw.splitlines() if '=' in line)
    require(props.get('ID_SERIAL_SHORT') == '5B3D052693', 'reviewed arm USB identity differs')
    devices = {str(device):device.stat().st_rdev}
    for path in Path('/dev').glob('video*'):
        if stat.S_ISCHR(path.stat().st_mode):
            devices[str(path)] = path.stat().st_rdev
    usb = []
    for path in Path('/sys/bus/usb/devices').iterdir():
        try:
            product = (path/'product').read_text().strip()
            if 'RealSense' not in product:
                continue
            serial = (path/'serial').read_text().strip()
            bus, number = int((path/'busnum').read_text()), int((path/'devnum').read_text())
            node = Path('/dev/bus/usb')/('%03d'%bus)/('%03d'%number)
            devices[str(node)] = node.stat().st_rdev
            usb.append(dict(product=product, serial=serial, device=str(node),
                            vendor=(path/'idVendor').read_text().strip(),
                            product_id=(path/'idProduct').read_text().strip()))
        except (FileNotFoundError, NotADirectoryError):
            continue
    require(len(usb) == 1 and (usb[0]['vendor'], usb[0]['product_id']) == ('8086', '0b5b'),
            'one real D405 USB device required')
    return dict(serial_alias=str(alias), serial_device=str(device), serial_id=props['ID_SERIAL_SHORT'],
                camera_usb=usb, devices=devices)


def cold_scan(config, allow_simulation=None):
    """Inspect present owners; never equate a changed parent PID with exit."""
    devices = device_inventory(); rdevs = set(devices['devices'].values())
    desktop = load(DESKTOP)
    names = {'__name:='+prefix+n.lstrip('/') for n in config['nodes'] for prefix in ('', '/')}
    entries = {r['executable'] for r in config['nodes'].values()} | {
        'roslaunch', 'roscore', 'rosmaster', 'candidate_simulation_startup.py', 'run_recorded_grasp.py'}
    conflicts, unreadable = [], []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            uid = proc.stat().st_uid
            argv = (proc/'cmdline').read_bytes().rstrip(b'\0').decode(errors='replace').split('\0')
            if not argv or not argv[0]:
                continue
            permitted = allow_simulation and in_group(int(proc.name), allow_simulation['ControlGroup'])
            if not permitted and (names.intersection(argv) or any(Path(a).name in entries for a in argv)):
                conflicts.append(dict(pid=int(proc.name), reason='runtime process'))
            try:
                maps = (proc/'maps').read_text()
                if 'pyrealsense2' in maps or 'librealsense' in maps:
                    conflicts.append(dict(pid=int(proc.name), reason='RealSense SDK still mapped'))
                fds = list((proc/'fd').iterdir())
            except PermissionError:
                helper = desktop.protected_system_session_helper(proc, argv) if uid == os.getuid() else False
                require(uid != os.getuid() or helper, 'unreadable same-user device owner: '+proc.name)
                unreadable.append(dict(pid=int(proc.name), uid=uid, known_desktop_helper=bool(helper)))
                continue
            for fd in fds:
                try:
                    actual = fd.stat()
                    if stat.S_ISCHR(actual.st_mode) and actual.st_rdev in rdevs:
                        conflicts.append(dict(pid=int(proc.name), fd=fd.name, reason='device held'))
                except (FileNotFoundError, ProcessLookupError):
                    pass
                except PermissionError:
                    require(uid != os.getuid(), 'unreadable same-user descriptor: '+str(fd))
        except (FileNotFoundError, ProcessLookupError):
            continue
    listeners = []
    for net in ('tcp', 'tcp6'):
        for line in (Path('/proc/net')/net).read_text().splitlines()[1:]:
            row = line.split(); port = int(row[1].rsplit(':', 1)[1], 16)
            if row[3] == '0A' and port in (11311, 8002) and not (port == 8002 and allow_simulation):
                listeners.append(port)
    query = subprocess.run(['fuser', *devices['devices']], capture_output=True, text=True, timeout=5.)
    require(query.returncode == 1 and not query.stdout.strip(), 'serial/camera owner exists or fuser failed')
    require(not conflicts and not listeners, 'stack/devices not cold: '+repr(dict(conflicts=conflicts, listeners=listeners)))
    return dict(recorded_ns=time.time_ns(), boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                devices=devices, conflicts=conflicts, listeners=listeners,
                uninspectable_system_processes=unreadable,
                owner_scope='all visible device FDs and SDK maps; identified protected desktop/other-user tables remain unreadable')


def prestart_health(config, contract, output):
    output.mkdir()
    check = load(REMOTE)
    binding = dict(parameters=config['remote_parameters'], allowed_backends=contract['allowed_backends'],
                   _client_contract=check.client_contract(contract['client']))
    url = check.valid_url(config['remote_parameters'][check.URL])+'/health'
    probe = check.fetch(url, {k:os.environ[k] for k in check.PROXY_KEYS if k in os.environ})
    raw = base64.b64decode(probe.pop('body_base64'), validate=True) if 'body_base64' in probe else None
    result = dict(passed=False, probe=probe, current_node_dependency_ready=False, prediction_requests=0)
    if raw is not None:
        path = output/'health_response.body'; path.write_bytes(raw)
        require(path.read_bytes() == raw, 'health response readback differs')
        probe.update(body_path=str(path), body_sha256=sha(path))
    try:
        require('error_type' not in probe and probe.get('http_status') == 200 and probe.get('response_url') == url
                and probe.get('body_complete') is True and raw is not None, 'REMOTE_DEPENDENCY_NOT_READY')
        result['capabilities'] = check.validate_health(check.strict_json(raw), binding)
        result['passed'] = True
    except Exception as exc:
        result['error'] = repr(exc)
    write(output/'result.json', result)
    require(result['passed'], 'REMOTE_DEPENDENCY_NOT_READY: '+str(result.get('error')))
    return ref(output/'result.json')


def validate_contract(contract):
    require(contract['schema'] == 1 and contract['kind'] == 'manual_full_stack_cold_start'
            and contract['workspace_root'] == str(W), 'wrong startup contract')
    out = Path(contract['output_root']); storage(out)
    for path, digest in contract['current_sha256'].items():
        require(sha(path) == digest, 'current startup input changed: '+path)
    instruction = read_ref(contract['instruction']); control = read_ref(contract['control'])
    require(instruction.get('source') == 'actual_user_message' and instruction.get('full_restart_authorized') is True
            and instruction.get('new_scene_confirmation_claimed') is False and bool(instruction.get('text')),
            'actual restart instruction required')
    require(control['state'] == 'active' and control == contract['control_snapshot'], 'current control changed')
    release = read_ref(contract['release']); source_check(release['source_sha256'])
    require(release_evidence(release) == contract['verification'], 'release evidence roles changed')
    config = read_ref(contract['configuration'])
    require(configuration() == config, 'expanded launch/source configuration changed')
    require(not Path(contract['pycache_prefix']).exists(), 'fresh unused Python cache prefix was populated')
    require(sha(W/'devel/lib/alicia_d_driver/alicia_d_driver_node') == contract['driver_binary_sha256'], 'driver binary changed')
    return out, config, release


def plan(args):
    out = args.session.resolve(); storage(out); out.mkdir(exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(out/'configuration_ros')
    release_ref = dict(path=str(args.release.resolve()), sha256=args.release_sha256)
    instruction_ref = dict(path=str(args.request.resolve()), sha256=args.request_sha256)
    release = read_ref(release_ref); verification = release_evidence(release)
    source_check(release['source_sha256'])
    config = configuration(); config_ref = write(out/'startup_configuration.json', config)
    driver = json.loads(DRIVER_BUILD.read_bytes())
    require(driver['returncode'] == 0, 'reviewed driver build failed')
    source_check(driver['source_sha256'])
    backend = json.loads(BACKENDS.read_bytes())
    paths = [Path(__file__).resolve(), ENTRY, LAUNCH, REMOTE, SIM, DESKTOP, DRIVER_BUILD, BACKENDS, CLIENT,
             HAND_EYE, CONTROL, E/'grasp_records/archive_config.json', E/'maintenance/storage_control.json']
    # Relay files, the camera's local SDK wrapper, and launch dependencies are current construction inputs.
    for row in config['nodes'].values():
        entry = W/'devel/lib'/row['package']/row['executable']
        if entry.is_file():
            paths.append(entry)
    paths.append(W/'tools/with_local_realsense_sdk.sh')
    paths.extend(Path(p) for p in config['launch_source_sha256'])
    contract = dict(schema=1, kind='manual_full_stack_cold_start', workspace_root=str(W), output_root=str(out),
        release=release_ref, instruction=instruction_ref, control=ref(CONTROL), control_snapshot=read_ref(ref(CONTROL)),
        configuration=config_ref, verification=verification, client=ref(CLIENT), allowed_backends=backend['allowed_backends'],
        driver_build=ref(DRIVER_BUILD), driver_binary_sha256=driver['binary_sha256'],
        current_sha256={str(p.resolve()):sha(p) for p in sorted(set(paths))},
        pycache_prefix=str(out/'unused_python_cache'), manual_only=True, automatic_grasp_authorized=False,
        physical_grasp_verified=False, temperature_queried=False, torque_disable_commands_sent=False)
    validate_contract(contract)
    before = cold_scan(config); time.sleep(.25); after = cold_scan(config)
    require(before['boot_id'] == after['boot_id'] and before['devices'] == after['devices'], 'cold device/boot identity changed')
    contract['cold_boot_id'] = after['boot_id']
    contract['cold_devices'] = after['devices']
    health = prestart_health(config, contract, out/'prestart_remote_health')
    contract_ref = write(out/'contract.json', contract)
    write(out/'plan.json', dict(contract=contract_ref, cold_before=before, cold_after=after,
         prestart_remote_health=health, prepared_ns=time.time_ns(), starts_services=False))
    print(json.dumps(dict(prepared=True, contract=contract_ref, verification=verification), ensure_ascii=False))


def start(contract, contract_ref):
    out, config, release = validate_contract(contract)
    require(Path('/proc/sys/kernel/random/boot_id').read_text().strip() == contract['cold_boot_id'], 'host boot changed')
    cold = cold_scan(config)
    require(cold['devices'] == contract['cold_devices'], 'device identity changed since plan')
    health = prestart_health(config, contract, out/'dispatch_remote_health')
    validate_contract(contract)
    write(out/'start_claim.json', dict(pid=os.getpid(), claimed_ns=time.time_ns(), contract=contract_ref))
    suffix = hashlib.sha256(str(out).encode()).hexdigest()[:12]
    result = dict(contract=contract_ref, started_ns=time.time_ns(), release_label=release['generation'],
                  full_restart=True, manual_only=True, automatic_grasp_started=False, components={},
                  dispatch_remote_health=health, cold=cold, startup_positive_enable_configured=True,
                  temperature_queried=False, torque_disable_commands_sent=False)
    try:
        for component in ('simulation', 'core'):
            if component == 'core':
                sim = load(SIM); sim.UNIT = result['components']['simulation']['unit']
                sim.wait_ready(timeout=40.)
                result['simulation_ready'] = sim.service_snapshot()
                write(out/'before_core_owner_check.json', cold_scan(config, properties(sim.UNIT)))
                validate_contract(contract)
            unit = 'alicia-manual-full-'+component+'-'+suffix+'.service'
            entry = ['/usr/bin/python3', '-B', str(SIM), '--serve'] if component == 'simulation' else ['/bin/bash', str(ENTRY)]
            command = ['systemd-run', '--user', '--collect', '--unit='+unit, '--property=Type=exec',
                '--property=Restart=no', '--property=WorkingDirectory='+str(W),
                '--property=StandardOutput=append:'+str(out/(component+'.log')),
                '--property=StandardError=append:'+str(out/(component+'.log')),
                '--setenv=ROS_LOG_DIR='+str(out/(component+'_ros')),
                '--setenv=PYTHONDONTWRITEBYTECODE=1', '--setenv=PYTHONNOUSERSITE=1',
                '--setenv=PYTHONPYCACHEPREFIX='+contract['pycache_prefix'], *entry]
            write(out/(component+'_launch_intent.json'), dict(unit=unit, command=command, contract=contract_ref))
            run = subprocess.run(command, capture_output=True, text=True, timeout=30.)
            row = dict(unit=unit, command=command, returncode=run.returncode, stdout=run.stdout, stderr=run.stderr)
            result['components'][component] = row; write(out/(component+'_dispatch.json'), row)
            require(run.returncode == 0, 'service dispatch failed: '+component)
        result['dispatched'] = True
    except BaseException as exc:
        result.update(dispatched=False, error=repr(exc), no_automatic_retry=True)
        raise
    finally:
        result['finished_ns'] = time.time_ns(); write(out/'startup.json', result)
    print(json.dumps(dict(dispatched=True, startup=str(out/'startup.json'), components=result['components']), ensure_ascii=False))


class Monitor:
    def __init__(self, config):
        import rospy
        from actionlib_msgs.msg import GoalStatusArray
        from sensor_msgs.msg import JointState, Image
        from std_msgs.msg import Bool, Header, String
        from alicia_flexible_grasp_supervisor.msg import GraspState
        self.rospy = rospy; self.lock = threading.Lock(); self.values = {}
        self.joints = deque(maxlen=80); self.images = {'color':deque(maxlen=30), 'depth':deque(maxlen=30)}
        rows = [('joint', '/alicia_d/accepted_joint_states', JointState), ('epoch', '/alicia_d/control_reference_epoch', Header),
                ('task', '/grasp/state', GraspState), ('mode', '/grasp_mode/status', String),
                ('manual', '/gui/joint_direct_mode', Bool), ('arm', '/alicia_controller/follow_joint_trajectory/status', GoalStatusArray),
                ('hand', '/hand_controller/follow_joint_trajectory/status', GoalStatusArray),
                ('motion_enabled', '/alicia_d/motion_enabled', Bool), ('actuation', '/alicia_d/actuation_status', String),
                ('color', config['parameters']['/camera/color_topic'], Image), ('depth', config['parameters']['/camera/depth_topic'], Image)]
        self.subscriptions = [rospy.Subscriber(topic, kind, lambda msg,k=key:self.receive(k,msg), queue_size=40) for key,topic,kind in rows]

    def receive(self, key, msg):
        with self.lock:
            self.values[key] = (time.monotonic(), msg)
            if key == 'joint': self.joints.append(msg)
            if key in self.images: self.images[key].append(msg)

    def sample(self, started_ns):
        with self.lock:
            data = dict(self.values); joints = list(self.joints)[-20:]
            images = {k:list(v) for k,v in self.images.items()}
        require(len(data) == 11 and len(joints) == 20, 'startup telemetry pending')
        values = {k:v[1] for k,v in data.items()}; now = self.rospy.Time.now().to_nsec()
        owners = dict(joint='/alicia_d_driver_node', epoch='/alicia_d_driver_node', manual='/alicia_supervisor_gui',
            task='/grasp_task_node', mode='/grasp_mode_router', arm='/bessica_d_hw_interface', hand='/bessica_d_hw_interface',
            motion_enabled='/alicia_d_driver_node', actuation='/alicia_d_driver_node', color='/supervisor_camera', depth='/supervisor_camera')
        require(all(m._connection_header.get('callerid') == owners[k] for k,m in values.items()), 'telemetry owner differs')
        epoch = values['epoch'].stamp.to_nsec(); names = list(joints[-1].name)
        stamps = [m.header.stamp.to_nsec() for m in joints]
        require(epoch >= started_ns and epoch <= now and {'Joint%d'%i for i in range(1,7)} <= set(names)
                and len(names) == len(set(names)), 'new driver epoch/joint layout invalid')
        require(all(m.header.frame_id == 'sdk_measured' and list(m.name) == names and len(m.position) == len(names)
                    and all(math.isfinite(v) for v in m.position) and epoch <= m.header.stamp.to_nsec() <= now for m in joints)
                and all(b > a for a,b in zip(stamps, stamps[1:])) and 0 <= now-stamps[-1] < 200_000_000,
                'real SDK window stale/unordered/invalid')
        spans = [max(m.position[i] for m in joints)-min(m.position[i] for m in joints) for i in range(len(names))]
        require(all(v <= .004 for v in spans), 'SDK window not stationary')
        require(values['manual'].data is True and not values['task'].active and time.monotonic()-data['task'][0] < 2., 'manual idle required')
        mode = json.loads(values['mode'].data)
        require(not any(mode.get(k) for k in ('active','start_reserved','pending')), 'router busy')
        for k in ('arm','hand'):
            msg = values[k]
            require(0 <= now-msg.header.stamp.to_nsec() < 2_000_000_000 and
                    not any(g.status in (0,1,6,7) for g in msg.status_list), 'trajectory active/stale')
        paired = {}
        for kind,messages in images.items():
            for m in messages:
                stamp = m.header.stamp.to_nsec()
                if 0 <= now-stamp < 500_000_000 and m._connection_header.get('callerid') == '/supervisor_camera':
                    require(m.width > 0 and m.height > 0 and m.step > 0 and len(m.data) == m.step*m.height
                            and m.encoding == ('bgr8' if kind == 'color' else '16UC1'), 'RGB-D layout invalid')
                    paired.setdefault(stamp, {})[kind] = dict(stamp_ns=stamp, width=m.width, height=m.height, encoding=m.encoding)
        complete = [v for stamp,v in sorted(paired.items()) if set(v) == {'color','depth'}]
        require(complete, 'fresh synchronous RGB-D pending')
        require(type(values['motion_enabled'].data) is bool and bool(values['actuation'].data), 'execution status missing')
        return dict(epoch_ns=epoch, feedback_stamps_ns=stamps, names=names, positions=list(joints[-1].position), spans=spans,
            manual_mode=True, gui_owning_manual_ack=True, task_active=False, actions_active=False, camera=complete[-1],
            motion_enabled=values['motion_enabled'].data, actuation_status=values['actuation'].data,
            actuation_confirmation_not_required_for_manual_startup=True, scene_clearance_granted=False)

    def close(self):
        for sub in self.subscriptions: sub.unregister()


def node_snapshot(master, core, config, caller):
    system = master.getSystemState()
    observed = set()
    for group in system:
        for _,names in group: observed.update(names)
    observed.discard(caller)
    expected = set(config['persistent_nodes'])
    extra = observed-expected
    require(expected <= observed and len(extra) <= 1
            and all(re.fullmatch('/move_group_commander_wrappers_[0-9]+', name) for name in extra),
            'configured graph differs: '+repr(dict(missing=sorted(expected-observed), extra=sorted(extra))))
    result = {}
    transport = load(REMOTE).RpcTransport
    for name in sorted(expected):
        uri = master.lookupNode(name)
        reply = xmlrpc.client.ServerProxy(uri, transport=transport()).getPid(caller)
        require(reply[0] == 1, 'node PID unavailable: '+name)
        row = process(reply[2]); proc = Path('/proc')/str(row['pid'])
        require(row['parent_pid'] == int(core['MainPID']) and in_group(row['pid'], core['ControlGroup']), 'node outside new core: '+name)
        command = (proc/'cmdline').read_bytes(); argv = command.rstrip(b'\0').decode().split('\0')
        require(any(Path(a).name == config['nodes'][name]['executable'] for a in argv), 'node executable differs: '+name)
        result[name] = dict(row, cmdline_sha256=hashlib.sha256(command).hexdigest(), uri=uri)
    require(len({v['pid'] for v in result.values()}) == len(result), 'duplicate node PID')
    aliases = {}
    for name in sorted(extra):
        uri = master.lookupNode(name)
        reply = xmlrpc.client.ServerProxy(uri, transport=transport()).getPid(caller)
        require(reply[0] == 1, 'gateway alias PID unavailable')
        row = process(reply[2])
        gateway = result['/motion_gateway']
        require(all(row[k] == gateway[k] for k in ('pid','start_ticks','parent_pid')),
                'helper name is not the actual motion_gateway process')
        aliases[name] = dict(row, uri=uri)
    publishers = dict(system[0])
    owners = {'/alicia_d/accepted_joint_states':'/alicia_d_driver_node',
              '/alicia_d/control_reference_epoch':'/alicia_d_driver_node',
              '/alicia_d/motion_enabled':'/alicia_d_driver_node',
              '/alicia_d/actuation_status':'/alicia_d_driver_node',
              config['parameters']['/camera/color_topic']:'/supervisor_camera',
              config['parameters']['/camera/depth_topic']:'/supervisor_camera'}
    require(all(publishers.get(topic) == [owner] for topic,owner in owners.items()), 'feedback/camera publisher not unique')
    return result, aliases


def gui_windows(pid):
    env = dict(os.environ, DISPLAY=':0', XAUTHORITY='/run/user/1000/gdm/Xauthority')
    raw = subprocess.check_output(['xprop', '-root', '_NET_CLIENT_LIST'], env=env, text=True, timeout=3.)
    found = []
    for window in re.findall(r'0x[0-9a-fA-F]+', raw):
        details = subprocess.check_output(['xprop','-id',window,'_NET_WM_PID'], env=env, text=True, timeout=3.)
        match = re.search(r'=\s*(\d+)\s*$', details)
        if match and int(match.group(1)) == pid: found.append(window)
    require(found, 'GUI has no mapped X11 window')
    return found


def accept(contract, contract_ref):
    out, config, release = validate_contract(contract)
    destination = out/'acceptance'; destination.mkdir(exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(destination/'observer_ros')
    startup_ref = ref(out/'startup.json'); startup = read_ref(startup_ref)
    require(startup['contract'] == contract_ref and startup['dispatched'] is True, 'startup incomplete/wrong contract')
    result = dict(passed=False, full_restart=True, verification=contract['verification'], manual_only=True,
        automatic_grasp_started=False, motion_commands_sent=False, temperature_queried=False, torque_disable_commands_sent=False,
        physical_grasp_verified=False, actuation_confirmation_not_implied_by_restart=True, contract=contract_ref,
        started_ns=time.time_ns(), source_manifest=contract['release'])
    monitor = None
    try:
        import rosgraph
        import rospy
        import yaml
        from controller_manager_msgs.srv import ListControllers
        rospy.init_node('manual_full_start_acceptance', anonymous=True, disable_rosout=True)
        master = rosgraph.Master(rospy.get_name())
        core = properties(startup['components']['core']['unit'])
        core_proc = Path('/proc')/core['MainPID']
        argv = (core_proc/'cmdline').read_bytes().rstrip(b'\0').decode().split('\0')
        require(argv[-2:] == ['--skip-log-check', str(LAUNCH.relative_to(W))]
                and core_proc.joinpath('cwd').resolve() == W, 'core entry/cwd differs')
        master_pid = master.getPid()
        require(process(master_pid)['parent_pid'] == int(core['MainPID']) and in_group(master_pid,core['ControlGroup']), 'master outside new core')
        monitor = Monitor(config)
        deadline = time.monotonic()+35.; last = None
        while time.monotonic() < deadline:
            try:
                nodes, aliases = node_snapshot(master,core,config,rospy.get_name())
                measured = monitor.sample(startup['started_ns']); break
            except Exception as exc:
                last = repr(exc); time.sleep(.1)
        else: raise RuntimeError('full startup pending: '+str(last))
        result.update(nodes=nodes, auxiliary_nodes=aliases, core_service=core, state=measured,
                      gui_windows=gui_windows(nodes['/alicia_supervisor_gui']['pid']))
        # Existing launch parameters were frozen before dispatch, including remote URL.
        for key in ('/gui/joint_direct_mode','/alicia_d_driver_node/software_temperature_monitoring_enabled',
                    '/alicia_d_driver_node/temperature_poll_rate_hz','/alicia_d_driver_node/self_check_poll_rate_hz',
                    '/alicia_d_driver_node/auto_torque_on_startup','/camera/simulate','/camera/fallback_to_simulation',
                    '/grasp/unknown_observe_return_max_observations','/grasp/unknown_lift_after_close_enabled'):
            require(rospy.get_param(key) == config['parameters'][key], 'live startup policy differs: '+key)
        for name,row in nodes.items():
            proc = Path('/proc')/str(row['pid'])
            env = dict(item.decode().split('=',1) for item in (proc/'environ').read_bytes().split(b'\0') if b'=' in item)
            require(env.get('PYTHONPYCACHEPREFIX') == contract['pycache_prefix']
                    and env.get('PYTHONDONTWRITEBYTECODE') == '1' and env.get('PYTHONNOUSERSITE') == '1'
                    and env.get('ROS_LOG_DIR') == str(out/'core_ros'), 'source/cache/external-log environment differs: '+name)
        require(sha(Path('/proc')/str(nodes['/alicia_d_driver_node']['pid'])/'exe') == contract['driver_binary_sha256'], 'running driver binary differs')
        controllers = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)().controller
        result['controllers'] = {c.name:c.state for c in controllers}
        require(all(result['controllers'].get(n) == 'running' for n in ('alicia_controller','hand_controller')), 'controller not running')
        log = (out/'core.log').read_bytes()[:2*1024*1024].decode(errors='replace')
        begun = re.findall(r'process\[controller_loader_once-([0-9]+)\]: started with pid \[([0-9]+)\]',log)
        ended = re.findall(r'\[controller_loader_once-([0-9]+)\] process has finished cleanly',log)
        require(len(begun) == 1 and ended == [begun[0][0]], 'one-shot controller loader not sealed')
        result['controller_loader_once'] = dict(pid=int(begun[0][1]), finished_cleanly=True)
        sim = load(SIM); sim.UNIT = startup['components']['simulation']['unit']; sim.wait_ready(timeout=40.)
        result['simulation'] = sim.service_snapshot()
        require(hashlib.sha256(rospy.get_param('/robot_description').encode()).hexdigest() == sim.IDENTITY['candidate_model_sha256'], 'paired URDF differs')
        require(rospy.get_param('/handeye') == yaml.safe_load(HAND_EYE.read_text())['handeye'], 'paired hand-eye differs')
        selection = rospy.get_param('/grasp_mode/selection')
        require(selection['mode'] == 'unknown' and selection['strategy'] == 'observe_return', 'startup selection differs')
        remote = nodes['/remote_grasp6d_node']; identity = {k:remote[k] for k in ('pid','parent_pid','start_ticks','cmdline_sha256')}
        binding = dict(schema=1, node_name='/remote_grasp6d_node', master_uri='http://localhost:11311',
            parameters=config['remote_parameters'], allowed_backends=contract['allowed_backends'],
            client_contract_source=contract['client'], node_identity=identity,
            evidence=[contract_ref, startup_ref, contract['configuration'], contract['release'], contract['instruction']]+
                [dict(path=p,sha256=h) for p,h in contract['current_sha256'].items()],
            construction_evidence_review_required=True, construction_provenance='actual frozen full-stack launch and new process')
        binding_ref = write(destination/'remote_startup_binding.json',binding)
        check = load(REMOTE); validated = check.load_binding(Path(binding_ref['path']),binding_ref['sha256'])
        remote_result = check.evaluate(validated,destination)
        result['remote_dependency'] = write(destination/'remote_readiness.json',remote_result)
        require(remote_result['passed'] is True and remote_result['dependency_ready'] is True, 'REMOTE_DEPENDENCY_NOT_READY: '+str(remote_result.get('error')))
        validate_contract(contract)
        require(node_snapshot(master,core,config,rospy.get_name()) == (nodes, aliases) and properties(startup['components']['core']['unit']) == core,
                'runtime instance changed during acceptance')
        require(rospy.get_param('/grasp_mode/selection') == selection and master.getPid() == master_pid, 'reference/master changed')
        final = monitor.sample(startup['started_ns'])
        require(final['epoch_ns'] == measured['epoch_ns'], 'driver reference changed')
        require(time.time_ns()-remote_result['finished_ns'] < 30_000_000_000, 'remote health expired during acceptance')
        result.update(passed=True,state=final,selection=selection,all_configured_nodes_started=True,
                      source_cache_process_binding=True,memory_attestation=False)
    except BaseException as exc:
        result['error'] = repr(exc); raise
    finally:
        if monitor: monitor.close()
        result['finished_ns'] = time.time_ns(); write(destination/'result.json',result)
    print(json.dumps(dict(passed=True,result=ref(destination/'result.json'),state=result['state']),ensure_ascii=False))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='operation', required=True)
    p = sub.add_parser('plan')
    for name in ('session','release','request'): p.add_argument('--'+name,type=Path,required=True)
    for name in ('release-sha256','request-sha256'): p.add_argument('--'+name,required=True)
    for name in ('start','accept'):
        p = sub.add_parser(name); p.add_argument('--contract',type=Path,required=True); p.add_argument('--sha256',required=True)
    args = parser.parse_args()
    if args.operation == 'plan':
        try:
            return plan(args)
        except BaseException as exc:
            out = args.session.resolve()
            if out.is_dir() and not (out/'contract.json').exists() and not (out/'failure.json').exists():
                write(out/'failure.json', dict(passed=False,stage='plan',error=repr(exc),recorded_ns=time.time_ns(),
                     services_started=False,motion_commands_sent=False,temperature_queried=False))
            raise
    reference = dict(path=str(args.contract.resolve(strict=True)),sha256=args.sha256)
    contract = read_ref(reference)
    return start(contract,reference) if args.operation == 'start' else accept(contract,reference)


if __name__ == '__main__':
    main()
