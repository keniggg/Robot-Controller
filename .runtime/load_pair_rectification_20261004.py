#!/usr/bin/env python3
"""Explicit, visual-only deployment. Importing this module performs no actions.

The operator must supply --execute after reviewing this script and its manifest.
No robot, torque, temperature, task or selection command is issued. Only camera
parameters and the four named visual processes may change. The SDK stream stays
640x480; published canvas geometry is verified before consumers are restarted.
"""
import argparse
import ast
from collections import OrderedDict
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import xmlrpc.client


WORK = Path(__file__).resolve().parents[1]
MOUNT = Path('/media/zhuyupei/SanStand')
EXT = MOUNT / '实验记录/Robot-Controller'
MANIFEST = WORK / '.runtime/rgbd_rectification_validation_20261004.json'
HELPER = '.runtime/gen4_root_cause/reload_camera_visual_gate.py'
SELF = '.runtime/load_pair_rectification_20261004.py'
TEST = '.runtime/test_load_pair_rectification_20261004.py'
STAGE_WRAPPER = WORK / '.runtime/camera_stage_observer.py'
MAIN = {
    'camera': 'src/alicia_flexible_grasp_supervisor/scripts/camera_node.py',
    'unknown': 'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py',
    'remote': 'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py',
    'gateway': 'src/alicia_flexible_grasp_supervisor/scripts/motion_gateway_node.py',
}
RELAY = {role: 'devel/lib/' + ('alicia_grasp_modes/' if role in ('unknown', 'remote')
         else 'alicia_flexible_grasp_supervisor/') + Path(path).name for role, path in MAIN.items()}
NODES = {'camera': '/supervisor_camera', 'unknown': '/unknown_tabletop_perception',
         'remote': '/remote_grasp6d_node', 'gateway': '/motion_gateway'}
EXPECTED_PIDS = {'camera': 183122, 'unknown': 168274, 'remote': 12755, 'gateway': 168334}
PRESERVE = {'/alicia_d_driver_node': 133511, '/bessica_d_hw_interface': 8957,
            '/grasp_task_node': 9003}
LAUNCH_PID = 8763
SERVICE_PARENT_PID = 1703
EXTRA_ENV = ('GRASP6D_URL', 'ALICIA_UNKNOWN_LOG_DIR', 'ALICIA_EXPERIMENT_STORAGE_BASE',
             'ALICIA_SESSION_LOG_DIR', 'GRASP_RECORD_ROOT', 'LANG', 'LC_ALL', 'LC_CTYPE')
REQUIRED_SOURCES = set(MAIN.values()) | set(RELAY.values()) | {SELF, HELPER,
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/rgbd_rectification.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/realsense_manager.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_observation_source.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/tabletop.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_noncontact_scene.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_observation_handoff.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_observation_set_handoff.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_instance.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_observation_region.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/silhouette_snapshot_diagnostic.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/rgb_acquisition_gateway.py'}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verified_path(name, root=WORK):
    path = Path(name)
    if path.is_absolute() or '..' in path.parts or str(path) != name:
        raise RuntimeError('noncanonical manifest path: ' + str(name))
    resolved = (root / path).resolve()
    if root.resolve() not in resolved.parents:
        raise RuntimeError('manifest path escaped workspace: ' + name)
    return resolved


def verify_manifest(manifest, root=WORK, required=None):
    required = REQUIRED_SOURCES if required is None else set(required)
    if (manifest.get('test_exit_code') != 0 or not manifest.get('test_result')
            or manifest.get('source_unchanged_during_test') is not True
            or manifest.get('changed_during_test') != []):
        raise RuntimeError('regression did not pass with unchanged sources')
    sources, tests = manifest.get('source_sha256', {}), manifest.get('test_sha256', {})
    if not isinstance(sources, dict) or not isinstance(tests, dict):
        raise RuntimeError('missing source/test hash maps')
    if not required <= set(sources) or TEST not in tests:
        raise RuntimeError('manifest scope incomplete: ' + repr(sorted(required - set(sources))) +
                           '; loader test must be in test_sha256')
    if set(sources) & set(tests):
        raise RuntimeError('overlapping manifest source/test scopes')
    runs = [manifest] + manifest.get('additional_test_runs', [])
    executed = set()
    for run in runs:
        if run.get('test_exit_code') != 0 or not run.get('test_result'):
            raise RuntimeError('additional regression failed or incomplete')
        command = run.get('command', [])
        if not isinstance(command, list) or 'pytest' not in command:
            raise RuntimeError('unrecognized regression command')
        executed.update(arg for arg in command if isinstance(arg, str) and
                        arg.endswith('.py') and 'test' in Path(arg).name)
    if set(tests) != executed:
        raise RuntimeError('test hashes differ from executed test files')
    for name, expected in dict(sources, **tests).items():
        if (not isinstance(expected, str) or len(expected) != 64
                or any(c not in '0123456789abcdef' for c in expected)
                or sha(verified_path(name, root)) != expected):
            raise RuntimeError('tested file changed: ' + name)
    return dict(sources, **tests)


def direct_argv(role, old_argv, root=WORK):
    """Only known catkin/source entries and ROS remaps; peel stage flags once."""
    if (len(old_argv) < 3 or any(type(a) is not str or not a for a in old_argv)
            or old_argv[0] != '/usr/bin/python3'):
        raise RuntimeError('unexpected Python entry for ' + role)
    entry, tail = old_argv[1], list(old_argv[2:])
    if role == 'camera' and Path(entry).resolve() == (root / '.runtime/camera_stage_observer.py').resolve():
        if tail.count('--') != 1:
            raise RuntimeError('stage wrapper delimiter invalid')
        split = tail.index('--')
        options, tail = tail[:split], tail[split + 1:]
        required = {'--camera-script', '--output', '--seconds', '--samples', '--interval', '--warmup'}
        parsed = {}
        if len(options) != 2 * len(required):
            raise RuntimeError('stage wrapper option count invalid')
        for key, value in zip(options[::2], options[1::2]):
            if key not in required or key in parsed or not value or value.startswith('--'):
                raise RuntimeError('stage wrapper options invalid')
            parsed[key] = value
        if set(parsed) != required or not Path(parsed['--output']).is_absolute():
            raise RuntimeError('stage wrapper options incomplete')
        try:
            seconds, interval, warmup = [float(parsed[k]) for k in ('--seconds', '--interval', '--warmup')]
            samples = int(parsed['--samples'])
        except ValueError:
            raise RuntimeError('stage wrapper numeric arguments invalid')
        if (not all(math.isfinite(v) for v in (seconds, interval, warmup))
                or not 0 < seconds <= 30 or not .5 <= interval <= 5
                or not 0 <= warmup < seconds or not 1 <= samples <= 12):
            raise RuntimeError('stage wrapper limits invalid')
        entry = parsed['--camera-script']
    accepted = {(root / MAIN[role]).resolve(), (root / RELAY[role]).resolve()}
    if not Path(entry).is_absolute() or Path(entry).resolve() not in accepted:
        raise RuntimeError('unknown or nested entry for ' + role)
    seen = set()
    for arg in tail:
        if arg.startswith('-') or ':=' not in arg:
            raise RuntimeError('non-ROS argument for ' + role)
        key, value = arg.split(':=', 1)
        if not key or not value or key in seen:
            raise RuntimeError('invalid or repeated ROS remap for ' + role)
        seen.add(key)
    if '__name:=' + NODES[role][1:] not in tail:
        raise RuntimeError('node name mismatch for ' + role)
    return [old_argv[0], entry] + tail


def validate_relay(role, root=WORK):
    """Hashed catkin relays must point to this exact tested main source."""
    tree = ast.parse((root / RELAY[role]).read_text())
    values = [n.value.s for n in ast.walk(tree) if isinstance(n, ast.Assign)
              and any(isinstance(t, ast.Name) and t.id == 'python_script' for t in n.targets)
              and isinstance(n.value, ast.Str)]
    if values != [str(root / MAIN[role])]:
        raise RuntimeError('catkin relay source mismatch: ' + role)


def read_environment(pid, allowed):
    rows = (Path('/proc') / str(pid) / 'environ').read_bytes().split(b'\0')
    values = dict(row.decode().split('=', 1) for row in rows if b'=' in row)
    return {key: values[key] for key in allowed if key in values}


def service_argv(role, old, environment, out, unit, allowed):
    if role not in ('camera', 'unknown', 'remote'):
        raise RuntimeError('gateway must only respawn under its original roslaunch')
    argv = [a for a in direct_argv(role, old['argv']) if not a.startswith('__log:=')]
    argv.append('__log:=' + str(out / (role + '.ros.log')))
    command = ['systemd-run', '--user', '--collect', '--unit=' + unit,
        '--property=Type=exec', '--property=Restart=no', '--property=WorkingDirectory=' + old['cwd'],
        '--property=StandardOutput=append:' + str(out / (role + '.stdout.log')),
        '--property=StandardError=append:' + str(out / (role + '.stderr.log'))]
    command += ['--setenv=' + key + '=' + environment[key] for key in allowed if key in environment]
    return command + ['--setenv=ROS_LOG_DIR=' + str(out / 'ros'),
                      '--setenv=PYTHONUNBUFFERED=1'] + argv


def enabled_configuration(original):
    result = deepcopy(original)
    profile = original.get('runtime_profile', {})
    native = profile.get('color_intrinsics', {})
    if (profile.get('source') != 'realsense_active_profile'
            or (native.get('width'), native.get('height')) != (640, 480)
            or original.get('align_depth_to_color', True) is not True
            or original.get('simulate', False)
            or original.get('color_projection_correction', {}).get('enabled', False)
            or profile.get('rectification') is not None):
        raise RuntimeError('unexpected original camera geometry or incompatible correction')
    result['stream_width'], result['stream_height'] = 640, 480
    result['pair_rectification'] = dict(result.get('pair_rectification', {}), enabled=True)
    return result


def unchanged_camera_settings(original, current):
    outputs = {'runtime_profile', 'width', 'height', 'fx', 'fy', 'cx', 'cy',
               'model', 'coeffs', 'distortion_model', 'distortion_coefficients',
               'intrinsics_source', 'stream_width', 'stream_height', 'pair_rectification'}
    if ({k: v for k, v in original.items() if k not in outputs} !=
            {k: v for k, v in current.items() if k not in outputs}):
        raise RuntimeError('unrelated camera configuration changed')


def rollback_configuration(original, activation_ns):
    result = deepcopy(original)
    # Preserve the old configuration, but the new disabled camera owner must
    # publish a new boundary too: no queued rectified frame may use the raw K.
    result['runtime_profile']['geometry_activation_stamp_ns'] = str(activation_ns)
    return result


def terminal_ledger(ledger, now_ns):
    if not isinstance(ledger, dict) or not ledger:
        raise RuntimeError('expected existing terminal gen13 ledger')
    if ledger.get('phase') in ('revoked', 'expired', 'exhausted', 'handed_off'):
        return 'explicit_terminal_phase'
    # Expired nonexecuting persisted status cannot be reused. Do not rewrite it
    # or infer completion from the robot pose or elapsed loading time.
    if ledger.get('phase') not in ('awaiting_measured_target', 'planning_rejected', 'preflight_rejected'):
        raise RuntimeError('RGB observation is not terminal')
    deadline = ledger.get('deadline_ns')
    if isinstance(deadline, str) and deadline.isascii() and deadline.isdecimal():
        deadline = int(deadline)
    if type(deadline) is not int or not 0 < deadline < now_ns:
        raise RuntimeError('RGB observation deadline has not expired')
    return 'original_deadline_expired_nonexecuting'


def perform_reload(io):
    """Testable ordering, with no alternative hardware action on failure."""
    mutated = False
    try:
        io.verify()
        # Both consumers must be dead before parameters or camera owner change.
        for role in ('unknown', 'remote'):
            mutated = True
            io.stop(role)
        io.stop('camera')
        io.configure(True)
        io.start('camera')
        io.validate_camera(True)
        for role in ('unknown', 'remote'):
            io.start(role)
        io.restart_gateway()
        io.final_validate()
    except BaseException:
        if mutated:
            io.rollback()
        raise


class Runtime:
    def __init__(self, manifest, out):
        # Only reached after --execute + complete manifest validation.
        import rospy
        from sensor_msgs.msg import Image, JointState
        from std_msgs.msg import Header
        from alicia_flexible_grasp_supervisor.msg import GraspState
        self.ros = rospy
        spec = importlib.util.spec_from_file_location('rectification_loader_helper', WORK / HELPER)
        self.helper = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.helper)
        self.manifest, self.out = manifest, out
        self.allowed_env = self.helper.ENV_KEYS + EXTRA_ENV
        self.report = dict(loaded=False, started_at_utc=datetime.now(timezone.utc).isoformat(),
            motion_commands_sent=False, selection_commands_sent=False, torque_commands_sent=False,
            temperature_queries_sent=False, manifest_sha256=sha(MANIFEST), events=[])
        self.before, self.current, self.units, self.environments = {}, {}, {}, {}
        self.camera_started_ns = None
        self.verified_camera = None
        self.lock = threading.RLock()
        self.faults, self.latest, self.frozen, self.pairs = [], {}, {}, OrderedDict()
        rospy.init_node('load_pair_rectification', anonymous=True, disable_signals=True)
        self.subscribers = [
            rospy.Subscriber('/alicia_d/sdk_command', JointState, self.sdk_cb, queue_size=100),
            rospy.Subscriber('/alicia_d/control_reference_epoch', Header, self.epoch_cb, queue_size=100),
            rospy.Subscriber('/grasp/state', GraspState, self.task_cb, queue_size=100)]
        self.helper.wait(lambda: all(k in self.latest for k in ('sdk', 'epoch', 'task')), 5., 'passive baseline topics')
        with self.lock:
            if self.latest['task'] is not False:
                raise RuntimeError('active grasp task')
            self.frozen = deepcopy(self.latest)
        for name, expected_pid in PRESERVE.items():
            if self.helper.node_pid(rospy, name) != expected_pid:
                raise RuntimeError('preserved PID differs from reviewed deployment: ' + name)
            self.before[name] = self.helper.identity(expected_pid)
        self.launch = self.helper.identity(LAUNCH_PID)
        if '/opt/ros/noetic/bin/roslaunch' not in self.launch['argv']:
            raise RuntimeError('gateway owner is not original roslaunch')
        for role, name in NODES.items():
            pid = self.helper.node_pid(rospy, name)
            if pid != EXPECTED_PIDS[role]:
                raise RuntimeError('visual PID differs from reviewed deployment: ' + role)
            old = self.helper.identity(pid)
            direct_argv(role, old['argv'])
            validate_relay(role)
            if old['ppid'] != (LAUNCH_PID if role == 'gateway' else SERVICE_PARENT_PID):
                raise RuntimeError('unexpected process owner: ' + role)
            self.before[role] = self.current[role] = old
            self.environments[role] = read_environment(pid, self.allowed_env)
        gateway_log = [a.split(':=', 1)[1] for a in self.before['gateway']['argv'] if a.startswith('__log:=')]
        if len(gateway_log) != 1 or EXT not in Path(gateway_log[0]).resolve().parents:
            raise RuntimeError('original respawning gateway log is not on external storage')
        self.configuration = rospy.get_param('/camera')
        self.new_configuration = enabled_configuration(self.configuration)
        self.selection, self.handeye = rospy.get_param('/grasp_mode/selection'), rospy.get_param('/handeye')
        self.ledger = rospy.get_param('/rgb_acquisition/ledger')
        if self.ledger.get('selection') != self.selection:
            raise RuntimeError('terminal ledger is not the unchanged current selection')
        terminal = terminal_ledger(self.ledger, rospy.Time.now().to_nsec())
        self.report.update(before=deepcopy(self.before), launch=self.launch, camera_before=self.configuration,
            selection=self.selection, handeye=self.handeye, ledger=self.ledger, terminal_evidence=terminal,
            sdk_hold=self.frozen['sdk'], epoch_ns=self.frozen['epoch'], environments=self.environments)
        for role in ('unknown', 'remote'):
            for key in ('ALICIA_UNKNOWN_LOG_DIR', 'ALICIA_EXPERIMENT_STORAGE_BASE',
                        'ALICIA_SESSION_LOG_DIR', 'GRASP_RECORD_ROOT'):
                value = self.environments[role].get(key)
                if value is None or not (Path(value).resolve() == EXT or EXT in Path(value).resolve().parents):
                    raise RuntimeError('missing/external storage environment mismatch: ' + role + '/' + key)
        if not self.environments['remote'].get('GRASP6D_URL'):
            raise RuntimeError('remote endpoint environment missing')
        for stream, key in (('rgb', 'color_topic'), ('depth', 'depth_topic')):
            default = '/supervisor/camera/' + ('color' if stream == 'rgb' else 'depth') + '/image_raw'
            self.subscribers.append(rospy.Subscriber(self.configuration.get(key, default), Image,
                lambda msg, kind=stream: self.image_cb(msg, kind), queue_size=3, buff_size=2**24))
        self.record('preflight_snapshot')

    def latch(self, key, value):
        with self.lock:
            if key in self.frozen and value != self.frozen[key]:
                self.faults.append(key + '_changed')
            self.latest[key] = value
            if key == 'sdk':
                self.latest['sdk_receipt'] = time.monotonic()

    def sdk_cb(self, msg):
        try:
            if (len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name)
                    or not all('Joint' + str(i) in msg.name for i in range(1, 7))
                    or not all(math.isfinite(q) for q in msg.position)):
                raise ValueError('invalid SDK hold')
            self.latch('sdk', dict(zip(msg.name, msg.position)))
        except Exception as exc:
            with self.lock: self.faults.append(str(exc))

    def epoch_cb(self, msg):
        value = msg.stamp.to_nsec()
        if type(value) is not int or value <= 0:
            with self.lock: self.faults.append('invalid_epoch')
        else:
            self.latch('epoch', value)

    def task_cb(self, msg):
        self.latch('task', bool(msg.active))

    def image_cb(self, msg, kind):
        with self.lock:
            stamp = msg.header.stamp.to_nsec()
            self.pairs.setdefault(stamp, {})[kind] = dict(width=msg.width, height=msg.height,
                encoding=msg.encoding, step=msg.step, bytes=len(msg.data),
                frame_id=msg.header.frame_id,
                caller=(getattr(msg, '_connection_header', {}) or {}).get('callerid'))
            while len(self.pairs) > 12:
                self.pairs.popitem(last=False)

    def guard(self):
        if not MOUNT.is_mount() or self.ros.is_shutdown():
            raise RuntimeError('storage/ROS unavailable')
        with self.lock:
            if self.faults:
                raise RuntimeError('latched preservation failure: ' + repr(self.faults))
            if time.monotonic() - self.latest.get('sdk_receipt', 0.) > 1.:
                raise RuntimeError('SDK hold telemetry unavailable')
        for name in PRESERVE:
            before = self.before[name]
            if not self.helper.alive(before) or self.helper.node_pid(self.ros, name) != before['pid']:
                raise RuntimeError('preserved process changed: ' + name)
        if not self.helper.alive(self.launch):
            raise RuntimeError('original roslaunch changed')
        for key, expected in (('/grasp_mode/selection', self.selection), ('/handeye', self.handeye),
                              ('/rgb_acquisition/ledger', self.ledger)):
            if self.ros.get_param(key) != expected:
                raise RuntimeError('preserved parameter changed: ' + key)
        if self.verified_camera is not None:
            from alicia_grasp_modes.rgb_observation_source import camera_geometry_snapshot
            camera = self.ros.get_param('/camera')
            unchanged_camera_settings(self.configuration, camera)
            settings = {key: camera.get(key) for key in ('pair_rectification', 'stream_width', 'stream_height')}
            if (camera_geometry_snapshot(camera) != self.verified_camera
                    or settings != self.verified_camera_settings):
                raise RuntimeError('verified camera geometry changed during consumer restart')

    def verify(self):
        verify_manifest(self.manifest)
        if sha(MANIFEST) != self.report['manifest_sha256']:
            raise RuntimeError('manifest changed during reload')
        self.guard()

    def record(self, event, **fields):
        self.report['events'].append(dict(event=event, monotonic=time.monotonic(), **fields))
        (self.out / 'result.json').write_text(json.dumps(self.report, indent=2, allow_nan=False) + '\n')

    def registered(self, role):
        try:
            return self.helper.node_pid(self.ros, NODES[role])
        except self.helper.CameraRegistrationPending:
            return None

    def stop(self, role):
        self.verify()
        old = self.current.get(role)
        if old is None or not self.helper.alive(old):
            if self.registered(role) is not None:
                raise RuntimeError('unowned replacement process appeared: ' + role)
            return
        if self.registered(role) != old['pid']:
            raise RuntimeError('registered process changed before stop: ' + role)
        self.record('stop_requested', role=role, pid=old['pid'])
        self.verify()
        if self.registered(role) != old['pid'] or not self.helper.alive(old):
            raise RuntimeError('visual owner changed immediately before signal: ' + role)
        os.kill(old['pid'], signal.SIGINT)
        self.helper.wait(lambda: not self.helper.alive(old), 15., role + ' exit', self.guard)
        self.current[role] = None

    def configure(self, enable):
        self.verify()
        if any(self.current.get(role) is not None for role in ('camera', 'unknown', 'remote')):
            raise RuntimeError('visual consumers/camera must stop before geometry changes')
        configuration = (self.new_configuration if enable else
                         rollback_configuration(self.configuration, self.ros.Time.now().to_nsec()))
        self.verified_camera = None
        self.ros.set_param('/camera', configuration)
        self.record('camera_configuration', rectification_enabled=enable)

    def start(self, role):
        self.verify()
        if self.current.get(role) is not None or self.registered(role) is not None:
            raise RuntimeError('duplicate visual process would be created: ' + role)
        unit = 'alicia-pair-' + role + '-' + datetime.now(timezone.utc).strftime('%Y%m%dt%H%M%S%f')
        command = service_argv(role, self.before[role], self.environments[role], self.out, unit, self.allowed_env)
        self.record('start_requested', role=role, command=command)
        self.verify()
        # Track the unit before launch, including ambiguous systemd-run replies.
        self.units[role] = unit
        if role == 'camera':
            self.camera_started_ns = self.ros.Time.now().to_nsec()
        subprocess.run(command, check=True, timeout=12)
        def ready():
            result = subprocess.run(['systemctl', '--user', 'show', unit, '--property=MainPID', '--value'],
                                    check=True, stdout=subprocess.PIPE, text=True, timeout=3)
            pid = int(result.stdout.strip() or 0)
            if pid <= 0:
                return False
            current = self.helper.identity(pid)
            self.current[role] = current
            if (current['exe_sha256'] != self.before[role]['exe_sha256']
                    or current['cwd'] != self.before[role]['cwd']
                    or current['argv'] != command[command.index('/usr/bin/python3'):]):
                raise RuntimeError('replacement process differs from reviewed launch: ' + role)
            return current if self.registered(role) == pid else False
        self.helper.wait(ready, 25., role + ' registration', self.guard)

    def validate_camera(self, enabled):
        from alicia_grasp_modes.rgb_observation_source import camera_geometry_snapshot
        from alicia_flexible_grasp.vision.rgbd_rectification import (
            rectification_contract, require_rectified_source_after_activation)
        def ready():
            camera = self.ros.get_param('/camera')
            profile = camera.get('runtime_profile', {})
            activation = profile.get('geometry_activation_stamp_ns')
            if (not isinstance(activation, str) or not activation.isascii() or not activation.isdecimal()
                    or int(activation) <= self.camera_started_ns):
                return False
            contract = rectification_contract(camera)
            if enabled and contract is None:
                return False
            if not enabled and contract is not None:
                raise RuntimeError('rollback camera still rectified')
            expected_setting = (self.new_configuration if enabled else self.configuration).get('pair_rectification')
            if camera.get('pair_rectification') != expected_setting:
                raise RuntimeError('rectification setting changed')
            expected_native = self.configuration['runtime_profile']['color_intrinsics']
            unchanged_camera_settings(self.configuration, camera)
            if profile.get('color_intrinsics') != expected_native:
                raise RuntimeError('native SDK camera profile changed')
            for key in ('device', 'native_depth_intrinsics', 'alignment', 'depth_to_color_extrinsics'):
                if profile.get(key) != self.configuration['runtime_profile'].get(key):
                    raise RuntimeError('physical camera profile changed: ' + key)
            if (camera.get('stream_width', camera.get('width')) != 640
                    or camera.get('stream_height', camera.get('height')) != 480
                    or camera.get('depth_scale') != self.configuration.get('depth_scale')):
                raise RuntimeError('stream dimensions/depth units changed')
            intrinsic = profile.get('geometry_intrinsics', {})
            for key in ('width', 'height', 'fx', 'fy', 'cx', 'cy'):
                if camera.get(key) != intrinsic.get(key):
                    raise RuntimeError('published geometry disagrees with SDK contract')
            if enabled and (intrinsic.get('model') != 'none' or intrinsic.get('coeffs') != [0.] * 5):
                raise RuntimeError('published output is not ideal pinhole')
            with self.lock:
                pairs = deepcopy(self.pairs)
            valid = []
            now = self.ros.Time.now().to_nsec()
            for stamp, pair in pairs.items():
                if set(pair) != {'rgb', 'depth'} or not 0 <= now - stamp <= 800000000:
                    continue
                try:
                    require_rectified_source_after_activation(camera, stamp)
                except ValueError:
                    continue
                for kind, pixel_bytes, encodings in (('rgb', 3, ('rgb8', 'bgr8')), ('depth', 2, ('16UC1',))):
                    image = pair[kind]
                    if (image['caller'] != NODES['camera'] or image['encoding'] not in encodings
                            or (image['width'], image['height']) != (camera['width'], camera['height'])
                            or image['step'] < image['width'] * pixel_bytes
                            or image['bytes'] != image['step'] * image['height']):
                        raise RuntimeError('new camera pair contract mismatch')
                if any(image['frame_id'] != self.configuration.get('frame_id', 'camera_link')
                       for image in pair.values()):
                    raise RuntimeError('new camera pair frame mismatch')
                valid.append(stamp)
            if len(valid) < 3:
                return False
            self.report['camera_after' if enabled else 'camera_rollback'] = camera
            self.report['verified_pair_stamps_ns'] = valid
            self.verified_camera = camera_geometry_snapshot(camera)
            self.verified_camera_settings = {key: deepcopy(camera.get(key)) for key in
                                            ('pair_rectification', 'stream_width', 'stream_height')}
            return True
        self.helper.wait(ready, 25., 'new paired geometry and three fresh native publications', self.guard)
        self.record('camera_validated', rectification_enabled=enabled)

    def restart_gateway(self):
        self.verify()
        old = self.current['gateway']
        if not self.helper.alive(old) or self.registered('gateway') != old['pid']:
            raise RuntimeError('gateway changed before shutdown')
        self.record('gateway_shutdown_requested', pid=old['pid'], original_launch_pid=LAUNCH_PID)
        self.verify()
        code, detail, uri = self.ros.get_master().lookupNode(NODES['gateway'])
        if code != 1:
            raise RuntimeError(detail)
        gateway = xmlrpc.client.ServerProxy(uri, transport=self.helper.Transport())
        code, detail, pid = gateway.getPid(self.ros.get_name())
        if code != 1 or pid != old['pid'] or not self.helper.alive(old):
            raise RuntimeError('gateway owner changed immediately before shutdown')
        try:
            gateway.shutdown(
                self.ros.get_name(), 'load tested paired RGB-D source consumers; task inactive')
        except OSError as exc:
            self.report['gateway_shutdown_reply_error'] = str(exc)
        self.helper.wait(lambda: not self.helper.alive(old), 15., 'old gateway exit', self.guard)
        def replaced():
            pid = self.registered('gateway')
            if pid is None or pid == old['pid']:
                return False
            current = self.helper.identity(pid)
            if (current['ppid'] != LAUNCH_PID or current['argv'] != old['argv']
                    or current['cwd'] != old['cwd'] or current['exe_sha256'] != old['exe_sha256']):
                raise RuntimeError('gateway did not respawn under original owner')
            self.current['gateway'] = current
            return True
        self.helper.wait(replaced, 35., 'original roslaunch gateway respawn', self.guard)
        self.ros.wait_for_service('/supervisor/plan_and_execute_rgb_acquisition_pose_strict', timeout=10.)

    def final_validate(self):
        self.verify()
        for role, current in self.current.items():
            if (current is None or not self.helper.alive(current)
                    or self.registered(role) != current['pid'] or current['pid'] == self.before[role]['pid']):
                raise RuntimeError('visual reload incomplete: ' + role)
        self.report.update(after=deepcopy(self.current), loaded=True,
                           preserved_nodes_unchanged=True, finished_at_utc=datetime.now(timezone.utc).isoformat())
        self.record('load_validated')
        self.verify()

    def rollback(self):
        self.report['loaded'] = False
        self.report['rollback'] = dict(attempted=True, restored=False)
        try:
            # Never continue after a safety/source/hash fault; restoring visual
            # availability is subordinate to preserving the original owner.
            self.verify()
            for role in ('unknown', 'remote', 'camera'):
                if role in self.units and self.current.get(role) is None:
                    unit = self.units[role]
                    subprocess.run(['systemctl', '--user', 'stop', unit], check=True, timeout=15)
                self.stop(role)
            self.configure(False)
            self.start('camera')
            self.validate_camera(False)
            self.start('unknown')
            self.start('remote')
            self.report['rollback'].update(restored=True, after=deepcopy(self.current),
                note='Current tested compatible code, original visual configuration; driver untouched.')
        except BaseException as exc:
            self.report['rollback']['error'] = repr(exc)
        self.record('rollback_finished')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='perform the reviewed visual-only reload')
    args = parser.parse_args(argv)
    if not args.execute:
        parser.error('--execute is required; importing/preparing the loader has no runtime effect')
    manifest = json.loads(MANIFEST.read_text())
    verify_manifest(manifest)
    if not MOUNT.is_mount():
        raise RuntimeError('SanStand unavailable; no fallback')
    out = EXT / 'ros_log' / ('pair_rectification_load_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'))
    out.mkdir(parents=False, exist_ok=False)
    (out / 'ros').mkdir()
    os.environ['ROS_LOG_DIR'] = str(out / 'ros')
    (out / 'validation_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    runtime = None
    try:
        runtime = Runtime(manifest, out)
        perform_reload(runtime)
    except BaseException as exc:
        report = runtime.report if runtime is not None else dict(loaded=False)
        report.update(loaded=False, error=repr(exc), finished_at_utc=datetime.now(timezone.utc).isoformat())
        (out / 'result.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        print(json.dumps(dict(loaded=False, report=str(out / 'result.json'), error=repr(exc))), flush=True)
        raise
    print(json.dumps(dict(loaded=True, report=str(out / 'result.json'))), flush=True)


if __name__ == '__main__':
    main()
