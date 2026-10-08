#!/usr/bin/env python3
"""Explicit, recorded driver-only reload and one bounded GUI recovery.

This module does nothing on import. Only --execute may stop the specified
old driver and start its replacement; all other ROS nodes remain running.
There is no torque-off publication, process-group signal or retry operation.
The resulting systemd user service survives this script and the tool session.
"""
import argparse
import datetime
import hashlib
import http.client
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xmlrpc.client

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / '.runtime'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
NODE = '/alicia_d_driver_node'
NAMES = ['Joint%d' % i for i in range(1, 7)] + ['right_finger']
QUANTUM = 2 * math.pi / 4096
PROBE_RAD = .025
BINARY = ROOT / 'devel/lib/alicia_d_driver/alicia_d_driver_node'
EXPECTED_SHA256 = '5af5e79be1108dfb8535fb7c35680ff5e31c4550834316e81bc278a114a9aac1'
MOUNT = Path('/media/zhuyupei/SanStand')
OUTROOT = MOUNT / '实验记录/Robot-Controller/ros_log/full_ros_latest_start_20261002/gen4_root_cause'


def finite7(values):
    q = np.asarray(values, dtype=float)
    if q.shape != (7,) or not np.isfinite(q).all():
        raise RuntimeError('require seven finite joint/gripper values')
    if np.any(abs(q[:6]) > math.pi):
        raise RuntimeError('arm position outside SDK range')
    return q


def words7(values, stroke):
    q = finite7(values)
    if not math.isfinite(stroke) or stroke <= 0 or not 0 <= q[6] <= stroke:
        raise RuntimeError('invalid gripper stroke/reference')
    # Match positive SDK lround, including exact half-counts.
    arm = np.minimum(4095, np.floor((q[:6] + math.pi) / QUANTUM + .5)).astype(int)
    return arm.tolist() + [int(math.floor(q[6] / stroke * 1000 + .5))]


def validate_sync(reference, actual, stroke, tolerance=.05):
    reference, actual = finite7(reference), finite7(actual)
    words7(reference, stroke)
    if np.max(abs(reference[:6] - actual[:6])) > tolerance:
        raise RuntimeError('old SDK hold fails existing driver reconnect sync tolerance')
    if abs(reference[6] - actual[6]) > .0001:
        raise RuntimeError('gripper moved across driver reload')


class OneEpochTransition:
    """One reload-origin change, then immutable; never a timed blanket waiver."""
    def __init__(self, old_ns):
        self.old_ns = int(old_ns)
        if self.old_ns <= 0:
            raise RuntimeError('missing original command epoch')
        self.barrier_ns = None
        self.new_ns = None

    def allow_start(self, barrier_ns):
        if self.barrier_ns is not None or barrier_ns <= self.old_ns:
            raise RuntimeError('invalid/repeated driver start epoch barrier')
        self.barrier_ns = int(barrier_ns)

    def observe(self, ns, caller):
        ns = int(ns)
        if caller != NODE or ns <= 0:
            raise RuntimeError('unknown command epoch publisher/value')
        if self.new_ns is not None:
            if ns != self.new_ns:
                raise RuntimeError('second command epoch change after driver reload')
        elif ns != self.old_ns:
            if self.barrier_ns is None or ns < self.barrier_ns or ns <= self.old_ns:
                raise RuntimeError('command epoch changed outside designated driver startup')
            self.new_ns = ns
        return self.new_ns


class WireFence:
    def __init__(self, reference_words):
        if len(reference_words) != 7:
            raise RuntimeError('missing original SDK words')
        self.original = list(reference_words)
        self.minimum_j5 = self.original[4]
        self.stage = 'old_driver_hold'

    def awaiting_sync(self):
        self.stage = 'awaiting_explicit_sync'

    def allow_sync(self):
        if self.stage != 'awaiting_explicit_sync':
            raise RuntimeError('sync is only allowed once after driver replacement')
        self.stage = 'original_sdk_sync'

    def allow_probe(self, probe_words):
        for i in range(7):
            if i != 4 and probe_words[i] != self.original[i]:
                raise RuntimeError('bootstrap changes an unedited SDK word')
        # Deliberately narrow to this reviewed deployment. Twenty-four words
        # are NOT extra motion permission: the actual probe still requires
        # <= .025 rad and the unchanged source-bound J5 geometry guard. A
        # larger servo residual fails closed rather than enlarging this bound.
        if not self.original[4] - 24 <= probe_words[4] < self.original[4]:
            raise RuntimeError('bootstrap SDK displacement exceeds fixed 24-count envelope')
        self.minimum_j5 = probe_words[4]
        self.stage = 'bootstrap_and_return'

    def restore_original_hold(self):
        self.minimum_j5 = self.original[4]
        self.stage = 'restored_original_hold'

    def check(self, words):
        if self.stage == 'awaiting_explicit_sync':
            raise RuntimeError('new driver transmitted before explicit frozen SDK sync')
        if len(words) != 7 or any(words[i] != self.original[i] for i in range(7) if i != 4):
            raise RuntimeError('unedited SDK/gripper word changed during reload/bootstrap')
        if not self.minimum_j5 <= words[4] <= self.original[4]:
            raise RuntimeError('J5 SDK word outside fixed original/probe corridor')


def process_identity(pid):
    path = Path('/proc') / str(int(pid))
    stat = (path / 'stat').read_text()
    fields = stat[stat.rfind(')') + 2:].split()
    return {'pid': int(pid), 'ppid': int(fields[1]), 'start_ticks': int(fields[19]),
            'exe': os.readlink(str(path / 'exe')),
            'exe_sha256': hashlib.sha256((path / 'exe').read_bytes()).hexdigest(),
            'argv': (path / 'cmdline').read_bytes().decode().strip('\0').split('\0')}


def same_process(identity):
    try:
        current = process_identity(identity['pid'])
    except FileNotFoundError:
        return False
    return current['start_ticks'] == identity['start_ticks']


def systemd_command(unit, out, env):
    command = ['systemd-run', '--user', '--collect', '--unit=' + unit,
               '--property=Type=exec', '--property=Restart=no',
               '--property=WorkingDirectory=' + str(ROOT),
               '--property=StandardOutput=append:' + str(out / 'driver.stdout.log'),
               '--property=StandardError=append:' + str(out / 'driver.stderr.log')]
    allowed = ('ROS_MASTER_URI', 'ROS_IP', 'ROS_HOSTNAME', 'ROS_DISTRO',
               'ROS_VERSION', 'ROS_PYTHON_VERSION', 'ROS_PACKAGE_PATH',
               'CMAKE_PREFIX_PATH', 'LD_LIBRARY_PATH', 'PYTHONPATH', 'PATH')
    for key in allowed:
        if env.get(key):
            command.append('--setenv=' + key + '=' + env[key])
    command += ['--setenv=ROS_LOG_DIR=' + str(out / 'ros'), str(BINARY),
                '__name:=alicia_d_driver_node', '__ns:=/',
                '__log:=' + str(out / 'driver.ros.log')]
    return command


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        return http.client.HTTPConnection(host, timeout=2.)


class DriverRegistrationPending(RuntimeError):
    """Only the ROS master's exact transient unknown-node registration reply."""


def master_driver_pid(rospy):
    code, detail, uri = rospy.get_master().lookupNode(NODE)
    if code != 1:
        if detail == 'unknown node [' + NODE + ']':
            raise DriverRegistrationPending('driver master registration unavailable: ' + detail)
        raise RuntimeError('driver master registration unavailable: ' + detail)
    api = xmlrpc.client.ServerProxy(uri, transport=TimeoutTransport())
    code, detail, pid = api.getPid(rospy.get_name())
    if code != 1:
        raise RuntimeError('driver PID unavailable: ' + detail)
    return int(pid)


def replacement_process_ready(rospy, unit, old_pid, expected_sha):
    try:
        pid = int(subprocess.check_output(['systemctl','--user','show',unit,'--property=MainPID','--value'],text=True,timeout=2).strip())
        if pid <= 0 or pid == old_pid or master_driver_pid(rospy) != pid:
            return False
        identity = process_identity(pid)
        if identity['exe_sha256'] != expected_sha:
            raise RuntimeError('running replacement binary hash differs')
        return identity
    except (DriverRegistrationPending, subprocess.SubprocessError, OSError, xmlrpc.client.Error):
        return False


def resume_startup_log_proof(directory):
    """Corroborates the bound live startup epoch; silence alone is not proof."""
    directory = Path(directory)
    logs = []
    hashes = {}
    for name in ('driver.stdout.log','driver.stderr.log'):
        path = directory / name
        raw = path.read_bytes()
        hashes[name] = hashlib.sha256(raw).hexdigest()
        logs.append(raw.decode(errors='replace'))
    text = '\n'.join(logs)
    if text.count('Requesting positive SDK torque_on') != 1 or text.count('Requesting positive SDK torque_on from startup;') != 1:
        raise RuntimeError('resume requires exactly one original startup positive enable')
    forbidden = ('Received /joint_commands', 'Streaming SDK command', 'GUI_DIRECT_PROFILE_ACCEPTED',
                 'Seeded SDK command interpolation', 'COMMAND_SYNCHRONIZED')
    if any(token in text for token in forbidden):
        raise RuntimeError('startup logs contain an admitted/streamed SDK command; cannot resume unsynchronized bootstrap')
    return {'log_sha256':hashes,'startup_positive_enable_count':1,
            'no_admitted_or_streamed_command_log':True,
            'proof_requires_same_live_PID_epoch_and_PENDING_POSITIVE_ENABLE_REQUESTED':True}


def wait_until(check, timeout, label, guard=lambda: None):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        guard()
        result = check()
        if result:
            return result
        time.sleep(.04)
    raise RuntimeError('timeout: ' + label)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--old-pid', type=int)
    parser.add_argument('--old-parent-pid', type=int)
    parser.add_argument('--resume-session', type=Path)
    parser.add_argument('--resume-pid', type=int)
    parser.add_argument('--resume-epoch-ns', type=int)
    parser.add_argument('--expected-new-sha256', default=EXPECTED_SHA256)
    args = parser.parse_args(argv)
    if not args.execute:
        parser.error('review-only module: --execute is required for any runtime action')
    previous_session = args.resume_session.resolve() if args.resume_session else None
    previous_result = None
    if previous_session is not None:
        if OUTROOT.resolve() not in previous_session.parents or not args.resume_pid or not args.resume_epoch_ns:
            parser.error('resume requires an existing SanStand session and explicit live PID/epoch')
        previous_result = json.loads((previous_session/'result.json').read_text())
        expected_error = 'driver master registration unavailable: unknown node [' + NODE + ']'
        if previous_result.get('complete') or not previous_result.get('old_stopped') or previous_result.get('error') != expected_error or previous_result.get('new_pid') is not None:
            raise RuntimeError('resume is restricted to the recorded pre-registration/no-sync interruption')
    elif args.old_pid is None or args.old_parent_pid is None:
        parser.error('new reload requires explicit old PID and parent PID')
    if not MOUNT.is_mount():
        raise RuntimeError('SanStand is not mounted; no fallback log directory')
    if hashlib.sha256(BINARY.read_bytes()).hexdigest() != args.expected_new_sha256:
        raise RuntimeError('new driver binary differs from reviewed SHA256')
    # Prove the user manager exists BEFORE retiring the live driver. Do not
    # print its environment (it may contain unrelated account information).
    subprocess.run(['systemctl', '--user', 'show-environment'], check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, timeout=5)
    suffix = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    out = previous_session / ('resume_' + suffix) if previous_session else OUTROOT / ('gui_profile_reload_' + suffix)
    out.mkdir(); (out / 'ros').mkdir()
    os.environ['ROS_LOG_DIR'] = str(out / 'ros')
    import rospy
    from std_msgs.msg import Bool
    from sensor_msgs.msg import JointState
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
    from tf.transformations import quaternion_matrix
    import gen4_single_axis_plan as base
    from gen4_j5_same_wire_plan import J5SameWireAudit
    from collect_single_axis import Recorder, AutoMotion

    rospy.init_node('recorded_gui_profile_driver_reload', anonymous=True, disable_signals=True)
    recorder = None
    unit = previous_result['new_unit'] if previous_result else 'alicia-gui-profile-driver-' + out.name.rsplit('_', 1)[-1].lower()
    state = {'old_stopped': bool(previous_session), 'new_unit': unit, 'new_pid': None, 'complete': False,
             'resume_of':str(previous_session) if previous_session else None,
             'runtime_restart_requested':False,'positive_enable_requested_by_script':False}

    class ReloadRecorder(Recorder):
        def __init__(self, directory):
            self.transition = None; self.fence = None; self.messages = {}; self.wire_pairs = {}
            self.latest_full_wire = None; self.last_full_wire_ns = 0
            self.wire_message_stamps = []; self.external_command_seen = False
            self.actual_origin = None; self.actual_bounds = None; self.stroke = None
            super().__init__(directory)
            self.subs.append(rospy.Subscriber('/alicia_d/protection_latched', Bool,
                                             self.callback('/alicia_d/protection_latched'), queue_size=1))

        def receive(self, topic, msg):
            if self.active:
                self.messages[topic] = msg
                if topic == '/joint_commands' and getattr(msg,'_connection_header',{}).get('callerid','') != rospy.get_name():
                    self.external_command_seen = True
                if topic in ('/alicia_d/sdk_command','/alicia_d/control_reference'):
                    self.wire_message_stamps.append(msg.header.stamp.to_nsec())
                    self.wire_message_stamps = self.wire_message_stamps[-200:]
                if topic == '/alicia_d/control_reference_epoch' and self.transition is not None:
                    try:
                        self.transition.observe(msg.stamp.to_nsec(), getattr(msg, '_connection_header', {}).get('callerid', ''))
                    except Exception as exc:
                        self.fatal = str(exc)
                    # Superclass's broad time waiver is replaced by the exact
                    # one-transition policy above, never enabled here.
                    self.epoch_ns = msg.stamp.to_nsec()
                if topic in ('/alicia_d/sdk_command', '/alicia_d/control_reference') and self.fence is not None:
                    ns = msg.header.stamp.to_nsec()
                    new_epoch = self.transition.new_ns if self.transition else None
                    # Delayed callbacks from the retired process retain old
                    # header stamps. Record them without interpreting them as
                    # a replacement-driver transmission before its sync.
                    barrier = self.transition.barrier_ns if self.transition else None
                    if ns > 0 and (barrier is None or ns >= barrier) and (new_epoch is None or ns >= new_epoch):
                        if self.fence.stage == 'awaiting_explicit_sync':
                            self.fatal = 'new driver SDK/reference message before explicit original-word sync'
                        pair = self.wire_pairs.setdefault(ns, {})
                        pair[topic] = msg
                        if len(pair) == 2:
                            sdk = pair['/alicia_d/sdk_command']; ref = pair['/alicia_d/control_reference']
                            arm = dict(zip(sdk.name, sdk.position)); vals = dict(zip(ref.name, ref.position))
                            if sdk.header.frame_id != 'sdk_transmitted' or ref.header.frame_id != 'sdk_manual' or len(arm) != 6 or len(vals) != 7 or set(vals) != set(NAMES) or set(arm) != set(NAMES[:6]):
                                self.fatal = 'invalid SDK/control-reference pair'
                            else:
                                words = words7([vals[n] for n in NAMES], self.stroke)
                                if words[:6] != words7([arm[n] for n in NAMES[:6]] + [vals[NAMES[6]]], self.stroke)[:6]:
                                    self.fatal = 'paired SDK and control-reference words differ'
                                try: self.fence.check(words)
                                except Exception as exc: self.fatal = str(exc)
                                self.latest_full_wire = words; self.last_full_wire_ns = ns
                        for old in sorted(self.wire_pairs)[:-30]: del self.wire_pairs[old]
            super().receive(topic, msg)
            if topic == '/alicia_d/accepted_joint_states' and self.states and self.actual_origin is not None:
                ns, q = self.states[-1]
                q = np.asarray(q)
                if self.actual_bounds is None:
                    if np.max(abs(q[:6] - self.actual_origin[:6])) > math.radians(.2):
                        self.fatal = 'actual posture changed across zero-new-target driver reload'
                else:
                    other = [0, 1, 2, 3, 5]
                    if np.max(abs(q[other] - self.actual_origin[other])) > math.radians(.5):
                        self.fatal = 'unedited actual axis drift during bootstrap'
                    if not self.actual_bounds[0] <= q[4] <= self.actual_bounds[1]:
                        self.fatal = 'J5 actual escaped fixed bootstrap response envelope'
                if abs(q[6] - self.actual_origin[6]) > .0001:
                    self.fatal = 'actual gripper changed during reload/bootstrap'

        def fresh_q(self):
            self.guard()
            with self.lock:
                if not self.states: raise RuntimeError('no accepted SDK feedback')
                ns, q = self.states[-1]
                if not 0 <= (rospy.Time.now().to_nsec() - ns) * 1e-9 <= .3:
                    raise RuntimeError('stale/future accepted SDK feedback')
                return np.asarray(q), ns

    def interrupted(signum, _frame):
        raise RuntimeError('reload/bootstrap interrupted by signal %d' % signum)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGALRM): signal.signal(sig, interrupted)
    signal.alarm(180)
    try:
        recorder = ReloadRecorder.__new__(ReloadRecorder); ReloadRecorder.__init__(recorder, out)
        rospy.wait_for_service('/check_state_validity', timeout=3)
        proxy = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
        def valid(q, gripper):
            req = GetStateValidityRequest(group_name='alicia')
            req.robot_state = RobotState()
            req.robot_state.joint_state = JointState(name=NAMES, position=list(q) + [gripper])
            return proxy(req).valid
        if previous_session is not None:
            old = json.loads((previous_session/'old_driver_identity.json').read_text())
            params = json.loads((previous_session/'old_driver_parameters.json').read_text())
            snapshot = json.loads((previous_session/'before_reload/snapshot.json').read_text())
            xml = (previous_session/'before_reload/robot_description.urdf').read_text()
            events = [json.loads(row) for row in (previous_session/'events.jsonl').read_text().splitlines()]
            starts = [row for row in events if row.get('event') == 'start_independent_driver']
            frozen = [row for row in events if row.get('event') == 'reload_frozen']
            if len(starts) != 1 or len(frozen) != 1 or frozen[0]['new_binary_sha256'] != args.expected_new_sha256:
                raise RuntimeError('resume provenance does not identify one reviewed replacement startup')
            if same_process(old): raise RuntimeError('retired driver still running; resume cannot stop it')
            startup_log_proof = resume_startup_log_proof(previous_session)
            reference = finite7(snapshot['sdk_q_rad']).copy(); origin = finite7(snapshot['q_rad']).copy()
            stroke = float(rospy.get_param(NODE+'/gripper_stroke_m'))
            frozen_words = words7(reference,stroke)
            if frozen_words != frozen[0]['sdk_words'] or snapshot['source_stamp_ns'] != frozen[0]['source_stamp_ns']:
                raise RuntimeError('frozen reference/source differs from original reload event')
            old_audit = J5SameWireAudit(snapshot,xml,valid)
            # Do not start, stop, change parameters, or request enable here.
            wait_until(lambda: recorder.epoch_ns == args.resume_epoch_ns, 5, 'explicit current resume epoch', recorder.guard)
            with recorder.lock:
                if any(ns >= args.resume_epoch_ns for ns in recorder.wire_message_stamps) or recorder.external_command_seen:
                    raise RuntimeError('SDK/reference or external joint command observed before unsynchronized resume')
                recorder.stroke = stroke
                recorder.transition = OneEpochTransition(snapshot['epoch_ns'])
                recorder.transition.allow_start(starts[0]['time_ns'])
                recorder.transition.observe(args.resume_epoch_ns,NODE)
                recorder.fence = WireFence(frozen_words); recorder.fence.awaiting_sync()
                recorder.actual_origin = origin.copy(); recorder.armed = True
            recorder.mark('resume_bound_original_reference',previous_session=str(previous_session),expected_new_pid=args.resume_pid,
                          expected_new_epoch_ns=args.resume_epoch_ns,reference=reference.tolist(),sdk_words=frozen_words,
                          source_stamp_ns=snapshot['source_stamp_ns'],startup_log_proof=startup_log_proof,
                          no_restart_no_parameter_mutation_no_enable=True)
        else:
            old = process_identity(args.old_pid)
            if old['ppid'] != args.old_parent_pid or Path(old['exe'][:-10] if old['exe'].endswith(' (deleted)') else old['exe']).name != BINARY.name or master_driver_pid(rospy) != args.old_pid:
                raise RuntimeError('old driver identity/parent/master PID does not match explicit arguments')
            params = rospy.get_param(NODE)
            if not params.get('auto_torque_on_startup') or params.get('use_trajectory_smoothing') or rospy.get_param('/gui/joint_direct_mode') is not True:
                raise RuntimeError('require existing positive startup, legacy smoothing off, manual ownership')
            (out / 'old_driver_identity.json').write_text(json.dumps(old, indent=2) + '\n')
            (out / 'old_driver_parameters.json').write_text(json.dumps(params, indent=2) + '\n')
            before = out / 'before_reload'; before.mkdir()
            snapshot, xml = base.capture(before)
            if snapshot['task_active'] or snapshot['protection_latched'] or not snapshot['feedback_ready'] or not snapshot['gui_direct_mode'] or not snapshot['actuation_status'].startswith('CONFIRMED:'):
                raise RuntimeError('old frozen reference is not an eligible confirmed manual hold')
            reference = finite7(snapshot['sdk_q_rad']).copy(); origin = finite7(snapshot['q_rad']).copy()
            stroke = float(rospy.get_param('/alicia_d_driver_node/gripper_stroke_m'))
            frozen_words = words7(reference, stroke)
            validate_sync(reference, origin, stroke, float(params.get('reconnect_sync_tolerance_rad', .05)))
            if np.max(abs(reference[:6]-origin[:6])) >= float(params.get('actuation_command_probe_min_delta_rad', .02)):
                raise RuntimeError('unchanged SDK sync would itself open a response probe; this minimal reload does not support that case')
            old_audit = J5SameWireAudit(snapshot, xml, valid)
            old_audit.guard_segment(origin, reference)
            with recorder.lock:
                if recorder.epoch_ns != snapshot['epoch_ns']: raise RuntimeError('epoch changed while freezing old source')
                recorder.stroke = stroke; recorder.transition = OneEpochTransition(snapshot['epoch_ns'])
                recorder.fence = WireFence(frozen_words); recorder.actual_origin = origin.copy(); recorder.armed = True
            wait_until(lambda: recorder.latest_full_wire == frozen_words, 2, 'original complete SDK words', recorder.guard)
            recorder.mark('reload_frozen',old=old,reference=reference.tolist(),sdk_words=frozen_words,source_stamp_ns=snapshot['source_stamp_ns'],new_binary_sha256=args.expected_new_sha256)
            # No process-group signal and no parent roslaunch or controller restart.
            recorder.guard()
            if not same_process(old) or master_driver_pid(rospy) != old['pid']:
                raise RuntimeError('old driver identity changed before SIGINT')
            recorder.mark('old_driver_sigint_requested',pid=old['pid'],start_ticks=old['start_ticks'])
            state['runtime_restart_requested'] = True
            os.kill(old['pid'], signal.SIGINT)
            state['old_stop_signal_sent'] = True
            wait_until(lambda: not same_process(old), 12, 'specified old driver exit', recorder.guard)
            state['old_stopped'] = True
            if not Path('/proc/%d' % args.old_parent_pid).exists(): raise RuntimeError('parent roslaunch unexpectedly exited')
            rospy.set_param(NODE + '/gui_direct_profile_enabled', True)
            with recorder.lock:
                recorder.transition.allow_start(rospy.Time.now().to_nsec()); recorder.fence.awaiting_sync()
            command = systemd_command(unit, out, os.environ)
            recorder.mark('start_independent_driver',systemd_argv=command,params={'gui_direct_profile_enabled':True,'auto_torque_on_startup':True,'use_trajectory_smoothing':False})
            result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=10)
            (out / 'systemd_start.txt').write_text(result.stdout)
            if result.returncode: raise RuntimeError('systemd driver start failed: ' + result.stdout)
        new = wait_until(lambda: replacement_process_ready(rospy,unit,old['pid'],args.expected_new_sha256),
                         12,'new independent driver PID/master binding',recorder.guard)
        if previous_session is not None and new['pid'] != args.resume_pid:
            raise RuntimeError('resume PID no longer matches the explicitly observed existing driver')
        state['new_pid'] = new['pid']; recorder.mark('new_driver_identity',identity=new)
        post_params = rospy.get_param(NODE)
        if post_params.get('gui_direct_profile_enabled') is not True or post_params.get('use_trajectory_smoothing') is not False or post_params.get('auto_torque_on_startup') is not True:
            raise RuntimeError('replacement driver runtime parameters differ from reviewed policy')
        (out / 'post_reload_runtime_configuration.json').write_text(json.dumps({
            'driver_identity':new,'driver_parameters':post_params,'systemd_user_unit':unit,
            'expected_binary_sha256':args.expected_new_sha256,
            'note':'Actual /proc/PID/exe SHA and ROS master PID are bound; internal profile activation is additionally logged by this binary at startup.'},indent=2)+'\n')
        def new_feedback_ready():
            with recorder.lock:
                ns = recorder.transition.new_ns
                required = ['/alicia_d/feedback_ready','/alicia_d/protection_latched','/alicia_d/actuation_status']
                if ns is None or not recorder.states or any(k not in recorder.messages for k in required): return False
                stamp,q = recorder.states[-1]
                return stamp >= ns and 0 <= (rospy.Time.now().to_nsec()-stamp)*1e-9 <= .3 and recorder.messages[required[0]].data and not recorder.messages[required[1]].data and recorder.messages[required[2]].data == 'PENDING:POSITIVE_ENABLE_REQUESTED'
        wait_until(new_feedback_ready, 10, 'new-generation pending ready feedback', recorder.guard)
        if previous_session is not None:
            def untouched_pending_window():
                if not new_feedback_ready(): return False
                with recorder.lock:
                    rows = [(ns,q) for ns,q in recorder.states if ns >= args.resume_epoch_ns]
                    if len(rows) < 7 or rows[-1][0]-rows[0][0] < 700000000: return False
                    if recorder.wire_message_stamps or recorder.external_command_seen:
                        raise RuntimeError('wire/external command appeared while proving unsynchronized resume')
                    q = np.asarray([row[1] for row in rows])
                    if np.max(abs(q[:,:6]-origin[:6])) > math.radians(.2):
                        raise RuntimeError('resume actual state exceeds original frozen pose by .2 degree')
                    return True
            wait_until(untouched_pending_window,5,'same-epoch pending/no-SDK stationary resume window',recorder.guard)
            proof = resume_startup_log_proof(previous_session)
            proof.update(new_pid=new['pid'],new_epoch_ns=args.resume_epoch_ns,
                         status='PENDING:POSITIVE_ENABLE_REQUESTED',observed_sdk_messages=0,
                         maximum_actual_post_reload_delta_deg=float(np.max(abs(np.degrees(recorder.fresh_q()[0][:6]-origin[:6])))))
            recorder.mark('unsynchronized_resume_proven',proof=proof)

        motion = AutoMotion(out)
        original_guard, original_apply = motion.guard, motion.apply
        def guard():
            recorder.guard(); original_guard()
            if rospy.get_param('/gui/joint_direct_mode') is not True: raise RuntimeError('manual ownership parameter changed')
            if recorder.transition.new_ns is None: raise RuntimeError('new driver epoch not bound')
            if not motion.status.startswith(('PENDING:', 'CONFIRMED:')):
                raise RuntimeError('bootstrap actuation lost; no retry permitted: ' + motion.status)
        motion.guard = guard
        current, _ = recorder.fresh_q()
        validate_sync(reference, current, stroke, float(params.get('reconnect_sync_tolerance_rad', .05)))
        if np.max(abs(reference[:6]-current[:6])) >= float(params.get('actuation_command_probe_min_delta_rad', .02)):
            raise RuntimeError('new accepted state makes unchanged SDK sync a nontrivial actuation probe')
        old_sync_proof = old_audit.guard_segment(current, reference)
        recorder.mark('original_sdk_sync_geometry',audit=old_sync_proof)
        # The sole pre-capture sync reasserts exactly the device's old words.
        # AutoMotion.apply's ordinary recovery target is NOT used here.
        guard()
        with recorder.lock: recorder.fence.allow_sync(); sync_after = rospy.Time.now().to_nsec()
        original_apply(('sync', reference.tolist(), 'restore exact previous successful SDK hold'))
        wait_until(lambda: recorder.latest_full_wire == frozen_words and recorder.last_full_wire_ns >= sync_after, 3, 'exact first new SDK seven-word hold', guard)
        wait_until(lambda: motion.status == 'PENDING:COMMAND_SYNCHRONIZED', 3, 'unchanged SDK sync acknowledged', guard)
        fresh = out / 'fresh_bootstrap'; fresh.mkdir()
        new_snapshot, new_xml = base.capture(fresh)
        if new_snapshot['epoch_ns'] != recorder.transition.new_ns or words7(new_snapshot['sdk_q_rad'],stroke) != frozen_words or new_snapshot['task_active'] or new_snapshot['protection_latched'] or not new_snapshot['feedback_ready'] or not new_snapshot['gui_direct_mode'] or new_snapshot['actuation_status'] != 'PENDING:COMMAND_SYNCHRONIZED':
            raise RuntimeError('new fresh bootstrap source lost exact synced epoch/ownership/hold')
        plan = J5SameWireAudit(new_snapshot, new_xml, valid)
        probe = reference.copy(); probe[4] = float(new_snapshot['q_rad'][4]) - PROBE_RAD
        probe_words = words7(probe,stroke)
        fresh_actual = finite7(new_snapshot['q_rad'])
        # Also bound the physical request relative to fresh actual; SDK residual
        # is preserved independently and is never relabelled physical motion.
        if abs(probe[4]-fresh_actual[4]) > PROBE_RAD + 1e-12: raise RuntimeError('probe exceeds .025 rad actual request')
        proof = plan.guard_segment(fresh_actual,probe)
        with recorder.lock:
            recorder.fence.allow_probe(probe_words)
            recorder.actual_origin = fresh_actual.copy()
            recorder.actual_bounds = (min(fresh_actual[4],probe[4],reference[4])-math.radians(.8),max(fresh_actual[4],probe[4],reference[4])+math.radians(.8))
        recorder.mark('bootstrap_admitted',probe=probe.tolist(),probe_sdk_words=probe_words,actual_bounds=list(recorder.actual_bounds),audit=proof)
        def apply(decision):
            action,target,detail=decision
            guard()
            if action == 'enable': raise RuntimeError('startup positive enable already used; no retry enable permitted')
            if action in ('sync','probe','publish'):
                if action == 'sync': raise RuntimeError('unexpected second sync after original SDK restoration')
                if motion.recovery.last_action_edited_index != 4: raise RuntimeError('bootstrap may edit only J5')
                wire = reference.copy(); wire[4] = float(target[4])
                recorder.fence.check(words7(wire,stroke))
                if action == 'probe' and abs(wire[4]-motion.q[4]) > PROBE_RAD + 1e-9:
                    raise RuntimeError('recovery probe exceeds .025 rad from current accepted state')
                t = recorder.buffer.lookup_transform('tool0','camera_link',rospy.Time(0),rospy.Duration(.2)).transform
                X = quaternion_matrix([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]); X[:3,3]=[t.translation.x,t.translation.y,t.translation.z]
                if not np.allclose(X,plan.snapshot['T_tool_camera'],rtol=0,atol=1e-9): raise RuntimeError('live handeye TF changed')
                audit = plan.guard_segment(np.asarray(motion.q),wire)
                recorder.mark('actual_submission_geometry',kind=action,target=wire.tolist(),audit=audit)
                guard(); return original_apply((action,wire.tolist(),detail))
            return original_apply(decision)
        motion.apply = apply
        delta = math.degrees(probe[4]-motion.q[4])
        if not .01 <= abs(delta) <= math.degrees(PROBE_RAD)+.15: raise RuntimeError('actual posture changed before bounded probe')
        motion.move(4,delta)
        if not motion.status.startswith('CONFIRMED:'): raise RuntimeError('probe did not establish real confirmation')
        recorder.mark('probe_confirmed',q=motion.q,status=motion.status)
        time.sleep(1.)
        guard()
        delta = math.degrees(reference[4]-motion.q[4])
        if not .01 <= delta <= 4.: raise RuntimeError('exact old-SDK return is not a bounded positive J5 move')
        motion.move(4,delta)
        wait_until(lambda: recorder.latest_full_wire == frozen_words, 3, 'exact original SDK return', guard)
        with recorder.lock: recorder.fence.restore_original_hold()
        if not motion.status.startswith('CONFIRMED:'): raise RuntimeError('return lacks confirmed actuation')
        settled_ns = rospy.Time.now().to_nsec(); time.sleep(2.)
        recorder.snapshot('bootstrap_return_original_sdk', settled_ns)
        current,_ = recorder.fresh_q()
        recorder.mark('reload_bootstrap_complete',new_unit=unit,new_pid=new['pid'],old_sdk_words=frozen_words,new_epoch_ns=recorder.transition.new_ns,final_q=current.tolist(),actual_return_error_deg=np.degrees(current[:6]-origin[:6]).tolist(),return_is_exact_sdk_not_assumed_exact_measured_pose=True)
        state['complete'] = True
    except BaseException as exc:
        state['error'] = str(exc)
        if recorder is not None:
            try: recorder.mark('reload_bootstrap_failed',reason=str(exc),state=state)
            except Exception: pass
        raise
    finally:
        signal.alarm(0)
        (out / 'result.json').write_text(json.dumps(state,indent=2)+'\n')
        if recorder is not None: recorder.close()
        print(json.dumps({'out':str(out),**state},ensure_ascii=False),flush=True)


if __name__ == '__main__':
    main()
