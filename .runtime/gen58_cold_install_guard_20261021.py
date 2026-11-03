"""File-only GEN57 installation guard for the observed stopped ROS stack.

No ROS import, hardware command, temperature access or service mutation. A cold
scan proves absence of current software writers; it is not feedback acceptance.
"""
import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
import subprocess
import time

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M/'实验记录/Robot-Controller'
N = E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
SESSION = N/'gen57_resume_r3'
CONTROL = E/'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/control.json'
CONTROL_SHA = '8beaf2eeabac1d86f217959524083c09b927969f0be4f22646d6c306e6865754'
CONTROL_SNAPSHOT = SESSION/'control.active67.json'
REQUEST = SESSION/'user_resume_request.json'
REQUEST_SHA = '89128addf34ef53109378f3d08f50d927131c206f7e43a62ad341d6aa6396023'
BASE_RELEASE = N/'verification/gen56_release_manifest.json'
BASE_SHA = '784af71745650269b1d114a5f9cbba32eaa792f8cf2029825eb5a56d52a8f255'
PREVIOUS = N/'gen56_runtime/gen56_runtime_deployment.json'
PREVIOUS_SHA = 'd02a3dde8f8b7d52b93ed57ce958a64526a68d11a912dd518435ffcd74995f85'
NODES = frozenset(('grasp_task_node', 'remote_grasp6d_node', 'unknown_tabletop_perception',
    'alicia_d_driver_node', 'bessica_d_hw_interface', 'move_group', 'motion_gateway',
    'supervisor_camera', 'compliant_gripper_controller', 'grasp_mode_router',
    'alicia_supervisor_gui'))
ENTRIES = frozenset(('roscore', 'rosmaster', 'roslaunch', 'alicia_d_driver_node',
    'alicia_d_hw_interface_node', 'move_group', 'mode_aware_grasp_task.py',
    'mode_aware_remote_grasp6d.py', 'unknown_tabletop_perception.py',
    'motion_gateway_node.py', 'camera_node.py', 'compliant_gripper_controller.py',
    'mode_router.py', 'mode_aware_gui.py', 'candidate_simulation_startup.py',
    'run_recorded_grasp.py'))


def require(value, reason):
    if not value:
        raise RuntimeError(reason)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def storage(path):
    path = Path(path).resolve()
    require(M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY,
            'writable SanStand required')
    require(N in path.parents and path != N, 'new output must remain in external session')
    existing = path
    while not existing.exists():
        existing = existing.parent
    require(existing.stat().st_dev == M.stat().st_dev, 'output is not on SanStand')
    config = read(E/'grasp_records/archive_config.json')
    policy = read(E/'maintenance/storage_control.json')
    require(config['upload_enabled'] is False and config['delete_enabled'] is False
            and policy['archive_policy'] == 'local_only', 'storage policy differs')
    require(shutil.disk_usage(E).free >= config['min_free_bytes'], 'external space below minimum')


def write(path, value):
    path = Path(path)
    storage(path)
    require(not path.exists(), 'immutable result already exists: '+str(path))
    with path.open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())


def control_check():
    require(sha(CONTROL) == sha(CONTROL_SNAPSHOT) == CONTROL_SHA
            and sha(REQUEST) == REQUEST_SHA, 'current resume/control binding changed')
    control, request = read(CONTROL), read(REQUEST)
    require(control['state'] == 'active' and control['revision'] == 67
            and request['source'] == 'actual_user_message'
            and request['text'] == '继续处理，并且要重启启动各节点和上位机'
            and control['resume_instruction'] == dict(path=str(REQUEST), sha256=REQUEST_SHA),
            'actual resume/full restart instruction missing')
    return control


def dbus_secret_service_identity(proc, argv):
    """Recognize only the current installed DBus desktop secret-service owner.

    Its non-dumpable descriptor table and executable symlink remain unreadable.
    This is system-service attribution, never a claim those FDs were inspected.
    No secret-service API is called: only the bus daemon's owner/PID metadata.
    """
    expected = ['/usr/bin/gnome-keyring-daemon', '--start', '--foreground', '--components=secrets']
    if argv != expected:
        return False
    service = Path('/usr/share/dbus-1/services/org.freedesktop.secrets.service')
    binary = Path(expected[0])
    for path in (service, binary):
        info = path.lstat()
        if not stat.S_ISREG(info.st_mode) or info.st_uid != 0 or info.st_mode & 0o022:
            return False
    service_raw = service.read_bytes()
    declarations = dict(line.split('=', 1) for line in service_raw.decode().splitlines() if '=' in line)
    if declarations.get('Name') != 'org.freedesktop.secrets' or declarations.get('Exec') != ' '.join(expected):
        return False
    status = dict(line.split(':', 1) for line in (proc/'status').read_text().splitlines() if ':' in line)
    if status['Name'].strip() != 'gnome-keyring-d' or [int(x) for x in status['Uid'].split()] != [os.getuid()]*4:
        return False
    fields = (proc/'stat').read_text().rsplit(')', 1)[1].split()
    parent = Path('/proc')/status['PPid'].strip()
    parent_fields = (parent/'stat').read_text().rsplit(')', 1)[1].split()
    if parent.stat().st_uid != os.getuid() or (parent/'exe').resolve(strict=True) != Path('/usr/lib/systemd/systemd'):
        return False
    parent_args = (parent/'cmdline').read_bytes().rstrip(b'\0').decode().split('\0')
    if parent_args not in (['/lib/systemd/systemd', '--user'], ['/usr/lib/systemd/systemd', '--user']):
        return False
    group = '/user.slice/user-%d.slice/user@%d.service/dbus.service' % (os.getuid(), os.getuid())
    groups = [line.rsplit(':', 1)[-1] for line in (proc/'cgroup').read_text().splitlines()]
    if group not in groups:
        return False

    def bus_identity():
        prefix = ['busctl', '--user', 'call', 'org.freedesktop.DBus', '/org/freedesktop/DBus', 'org.freedesktop.DBus']
        owner = subprocess.check_output(prefix + ['GetNameOwner', 's', 'org.freedesktop.secrets'], text=True, timeout=3.).strip()
        pid = subprocess.check_output(prefix + ['GetConnectionUnixProcessID', 's', 'org.freedesktop.secrets'], text=True, timeout=3.).strip()
        return dict(owner=owner, pid=pid)

    before = bus_identity()
    if before['pid'] != 'u '+proc.name or not before['owner'].startswith('s ":'):
        return False
    props = subprocess.check_output(['systemctl', '--user', 'show', 'dbus.service', '--property=MainPID',
        '--property=ControlGroup', '--property=ActiveState', '--property=SubState'], text=True, timeout=3.)
    properties = dict(line.split('=', 1) for line in props.splitlines() if '=' in line)
    if properties.get('ControlGroup') != group or properties.get('ActiveState') != 'active' or properties.get('SubState') != 'running':
        return False
    dbus = Path('/proc')/properties['MainPID']
    if dbus.stat().st_uid != os.getuid() or (dbus/'exe').resolve(strict=True) != Path('/usr/bin/dbus-daemon'):
        return False
    if before != bus_identity() or (proc/'stat').read_text().rsplit(')', 1)[1].split()[19] != fields[19] or (parent/'stat').read_text().rsplit(')', 1)[1].split()[19] != parent_fields[19] or service.read_bytes() != service_raw:
        return False
    return dict(kind='excluded_system_service_unreadable_fd', bus_service='org.freedesktop.secrets',
        bus_identity=before, pid=int(proc.name), start_ticks=fields[19], parent_pid=int(parent.name),
        parent_start_ticks=parent_fields[19], parent_exe=str((parent/'exe').resolve()),
        dbus_service_properties=properties, installed_service=str(service), installed_service_sha256=sha(service),
        installed_binary=str(binary), installed_binary_sha256=sha(binary), command_line=argv,
        process_executable_symlink_verified=False, descriptor_table_verified=False,
        scope='DBus owner/PID, installed service command, parent and cgroup attribution only; non-dumpable executable symlink and descriptors remain unverified')


def protected_system_session_helper(proc, argv):
    """Identify the non-dumpable desktop daemons; their FDs stay unverified."""
    secret_service = dbus_secret_service_identity(proc, argv)
    if secret_service:
        return secret_service
    status = dict(line.split(':', 1) for line in (proc/'status').read_text().splitlines()
                  if ':' in line)
    parent = Path('/proc')/status['PPid'].strip()
    groups = [line.rsplit(':', 1)[-1] for line in (proc/'cgroup').read_text().splitlines()]
    user_root = '/user.slice/user-%d.slice/' % os.getuid()
    if argv == ['(sd-pam)']:
        parent_args = (parent/'cmdline').read_bytes().rstrip(b'\0').split(b'\0')
        return (status['Name'].strip() == '(sd-pam)' and parent.stat().st_uid == os.getuid()
            and parent_args in ([b'/lib/systemd/systemd', b'--user'],
                                [b'/usr/lib/systemd/systemd', b'--user'])
            and (parent/'exe').resolve(strict=True) == Path('/usr/lib/systemd/systemd')
            and user_root+'user@%d.service/init.scope' % os.getuid() in groups)
    desktop = (
        ['/usr/bin/gnome-keyring-daemon', '--daemonize', '--login'],
        ['/usr/bin/ssh-agent', '/usr/bin/im-launch', 'env', 'GNOME_SHELL_SESSION_MODE=ubuntu',
         '/usr/bin/gnome-session', '--systemd', '--session=ubuntu'],
        ['/usr/bin/vmtoolsd', '-n', 'vmusr', '--blockFd', '3'])
    if argv not in desktop or not any(g.startswith(user_root) and
        (g.endswith('.scope') or g.endswith('gnome-session-manager@ubuntu.service')) for g in groups):
        return False
    executable = Path(argv[0])
    info = executable.stat()
    if not (status['Name'].strip() == executable.name[:15]
            and info.st_uid == 0 and not info.st_mode & 0o022):
        return False
    if executable.name == 'gnome-keyring-daemon':
        return parent.name == '1' and (parent/'comm').read_text().strip() == 'systemd'
    parent_exe = (parent/'exe').resolve(strict=True)
    parent_args = (parent/'cmdline').read_bytes().rstrip(b'\0').decode().split('\0')
    if executable.name == 'ssh-agent':
        return parent_exe == Path('/usr/libexec/gnome-session-binary') and parent_args == [
            '/usr/libexec/gnome-session-binary', '--systemd', '--systemd', '--session=ubuntu']
    return parent_exe == Path('/usr/lib/systemd/systemd') and parent_args in (
        ['/lib/systemd/systemd', '--user'], ['/usr/lib/systemd/systemd', '--user'])


def cold_scan():
    """Refuse any ROS/attempt process, reserved listener or serial-device owner."""
    storage(SESSION)
    control = control_check()
    require(sha(BASE_RELEASE) == BASE_SHA and sha(PREVIOUS) == PREVIOUS_SHA,
            'historical baseline changed')
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    alias = Path('/dev/alicia_arm')
    require(alias.is_symlink(), 'arm device alias missing')
    device = alias.resolve(strict=True)
    info = device.stat()
    require(device.parent == Path('/dev') and device.name.startswith('ttyACM')
            and stat.S_ISCHR(info.st_mode), 'unexpected arm device')
    names = {'__name:='+prefix+name for name in NODES for prefix in ('', '/')}
    processes, owners, uninspectable, scan_count = [], [], [], 0
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            uid = proc.stat().st_uid
            argv = (proc/'cmdline').read_bytes().rstrip(b'\0').decode(errors='replace').split('\0')
            if not argv or not argv[0]:
                continue
            scan_count += 1
            if names.intersection(argv) or any(Path(arg).name in ENTRIES for arg in argv):
                processes.append(dict(pid=int(proc.name), argv=argv))
            # All inspectable FDs are checked. Same-user visibility is mandatory.
            try:
                fds = list((proc/'fd').iterdir())
            except PermissionError:
                session_helper = protected_system_session_helper(proc, argv)
                require(uid != os.getuid() or session_helper,
                        'unreadable same-user descriptor table: '+proc.name)
                fields = (proc/'stat').read_text().rsplit(')', 1)[1].split()
                uninspectable.append(dict(pid=int(proc.name), uid=uid, parent_pid=int(fields[1]),
                    start_ticks=fields[19], argv=argv if session_helper else None,
                    reason='excluded_system_service_unreadable_fd' if isinstance(session_helper, dict) else
                        ('identified_non_dumpable_desktop_helper' if session_helper else 'other_uid'),
                    system_service_identity=session_helper if isinstance(session_helper, dict) else None))
                continue
            for fd in fds:
                try:
                    actual = fd.stat()
                    if stat.S_ISCHR(actual.st_mode) and actual.st_rdev == info.st_rdev:
                        owners.append(dict(pid=int(proc.name), fd=fd.name))
                except (FileNotFoundError, ProcessLookupError):
                    pass
                except PermissionError:
                    require(uid != os.getuid(), 'unreadable same-user descriptor: '+str(fd))
        except (FileNotFoundError, ProcessLookupError):
            continue
        except PermissionError:
            require(proc.stat().st_uid != os.getuid(), 'unreadable same-user process: '+proc.name)
    listeners = []
    for network in ('tcp', 'tcp6'):
        for line in (Path('/proc/net')/network).read_text().splitlines()[1:]:
            fields = line.split()
            port = int(fields[1].rsplit(':', 1)[1], 16)
            if fields[3] == '0A' and port in (11311, 8002):
                listeners.append(dict(network=network, port=port, endpoint=fields[1]))
    query = subprocess.run(['fuser', str(device)], capture_output=True, text=True, timeout=3.)
    require(query.returncode == 1 and not query.stdout.strip(), 'serial device open or owner query failed')
    require(not processes and not owners and not listeners,
            'runtime is not cold: '+json.dumps(dict(processes=processes, owners=owners, listeners=listeners)))
    require(Path('/proc/sys/kernel/random/boot_id').read_text().strip() == boot
            and control_check() == control, 'boot/control changed during cold scan')
    return dict(recorded_ns=time.time_ns(), boot_id=boot, scanned_processes=scan_count,
        runtime_processes=processes, serial_owners=owners, protected_port_listeners=listeners,
        uninspectable_system_descriptor_tables=uninspectable,
        descriptor_scope='all inspectable FDs; unreadable same-user programs rejected except explicitly identified non-dumpable desktop daemons; their FDs remain unverified',
        serial_alias=str(alias), serial_device=str(device), serial_rdev=info.st_rdev,
        serial_owner_query_returncode=query.returncode, control_sha256=CONTROL_SHA,
        observation_kind='absence_of_known_runtime_processes_and_visible_serial_writers',
        manual_mode_verified=False, live_feedback_verified=False,
        motion_commands_sent=False, temperature_queried=False)


def cold_pair():
    before = cold_scan()
    time.sleep(.25)
    after = cold_scan()
    require(before['boot_id'] == after['boot_id']
            and before['serial_device'] == after['serial_device']
            and before['serial_rdev'] == after['serial_rdev'], 'cold identity changed')
    return dict(before=before, after=after, passed=True)
