#!/usr/bin/env python3
"""One cold start of a reviewed release; logs stay on mounted SanStand.

This starts simulation and the existing manual-mode full stack. The reviewed
driver requests positive enable on connection. It does not start a grasp, query
temperature, stop a service, send disable, or retry a partly attempted start.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import socket
import stat
import subprocess
import time
import xml.etree.ElementTree as ET

W = Path(__file__).resolve().parents[1]
MOUNT = Path('/media/zhuyupei/SanStand')
EXT = MOUNT / '实验记录/Robot-Controller'
LOG_ROOT = EXT / 'ros_log'
PREVIOUS = LOG_ROOT / 'full_latest_contact_admission_20261013T011103Z'
DRIVER_BUILD = LOG_ROOT / 'pending_actuation_drift_20261007T104410.504268Z/driver_build.json'
SERIAL = '5B3D052693'
NODES = ('grasp_task_node', 'remote_grasp6d_node', 'unknown_tabletop_perception',
         'alicia_d_driver_node', 'bessica_d_hw_interface', 'move_group',
         'motion_gateway', 'supervisor_camera', 'compliant_gripper_controller',
         'grasp_mode_router', 'alicia_supervisor_gui')
DEPENDENCIES = {
    '.runtime/start_full_latest_frozen_two_observations_20261011.sh': '7335f1a80225840b6b6d5ef787bfcee32281ea8b54699bac82e95b724448c7fa',
    '.runtime/full_latest_frozen_two_observations_20261011.launch': '150c0130619c22cf1638be26113005f2b6af83248c7d2fa02525a37ccd68b1ab',
    '.runtime/full_latest_no_skin_20261010.launch': '28ec4d2ba5138b48fb4f44b9824c586448e718d31a567701c395b1c0ae3645ed',
    '.runtime/full_startup_candidate/candidate_pair_and_simulation.launch': '04e4d86d82afbd4272f0e1a21fcf3e03208d0f5e8a3b9186d826ff494989e6e0',
    '.runtime/candidate_models/expanded_metric_20260929/candidate_parameters.launch': '4fe1fdf65ff3ea5b1f2b66f0a019feaf8cfba15769844a180ee2d96f74ca38cf',
    '.runtime/full_startup_candidate/candidate_simulation_startup.py': 'dab922722b43e85ab7723ba4c44b88f53d31498e4ef206d6f8f7c43a07542dfd',
    '.runtime/verify_full_contact_restart_20261012.py': '297cefe0b3c5e57cc0acdb6d0b86be1b345a68bacf73ba7d88f3589d8a680e2e',
    'src/real-arm/alicia_d_driver/launch/alicia_d_bringup.launch': 'c00e3d155b253c127e5db19dbef12191766c1b9e1e84b43226c975bbd2f72e3b',
    'src/alicia_grasp_modes/launch/full_system_modes.launch': '4bb21cf982369edb58366676f1c129647b26028f3931834961e00f6c20795d56',
    'tools/with_local_realsense_sdk.sh': '47e352b6e81a9bfd313f92b1595a26c779324e65fa65172d0b3da4c77d0aa096',
}


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def storage_check(session):
    require(MOUNT.is_mount(), 'SanStand is not mounted; no system-disk fallback')
    require(not os.statvfs(MOUNT).f_flag & os.ST_RDONLY, 'SanStand is read-only')
    require(LOG_ROOT.resolve() in session.parents and session != PREVIOUS.resolve(),
            'a new SanStand ros_log session is required')
    require(session.is_dir() and session.stat().st_dev == MOUNT.stat().st_dev,
            'session must already exist on SanStand')
    config = read(EXT / 'grasp_records/archive_config.json')
    control = read(EXT / 'maintenance/storage_control.json')
    require(config.get('upload_enabled') is False and config.get('delete_enabled') is False,
            'archive upload/delete policy changed')
    require(control.get('archive_policy') == 'local_only', 'archive is not local_only')
    minimum = config.get('min_free_bytes')
    require(type(minimum) is int and minimum > 0, 'invalid recording minimum free bytes')
    free = shutil.disk_usage(EXT).free
    require(free >= minimum, 'recording free space below configured minimum')
    return {'free_bytes': free, 'min_free_bytes': minimum, 'local_only': True}


def source_check(mapping):
    require(isinstance(mapping, dict) and bool(mapping), 'empty source map')
    for relative, expected in mapping.items():
        path = Path(relative)
        require(not path.is_absolute() and '..' not in path.parts,
                'invalid source path: ' + relative)
        require(sha(W / path) == expected, 'source hash changed: ' + relative)


def verify_release(path):
    release = read(path)
    require(all(release.get(key) is True for key in ('passed', 'reviewed', 'installed')),
            'release must be sealed, reviewed and installed')
    summary = release['test_summary']
    require(type(summary.get('tests')) is int and summary['tests'] > 0,
            'release has no completed tests')
    require(all(type(summary.get(k)) is int and summary[k] == 0
                for k in ('failures', 'errors', 'skipped')), 'release regression incomplete')
    source_check(release['source_sha256'])
    evidence = release['evidence_sha256']
    require(isinstance(evidence, dict) and bool(evidence), 'release evidence empty')
    for name, expected in evidence.items():
        require(Path(name).is_absolute() and sha(name) == expected,
                'release evidence hash changed: ' + name)
    source_check(DEPENDENCIES)
    build = read(DRIVER_BUILD)
    require(build.get('returncode') == 0, 'recorded driver build failed')
    source_check(build['source_sha256'])
    require(sha(W / 'devel/lib/alicia_d_driver/alicia_d_driver_node') == build['binary_sha256'],
            'driver binary differs from reviewed build')
    return release


def serial_check():
    alias = Path('/dev/alicia_arm')
    require(alias.is_symlink(), 'expected udev /dev/alicia_arm alias is missing')
    device = alias.resolve(strict=True)
    require(device.parent == Path('/dev') and device.name.startswith('ttyACM') and
            stat.S_ISCHR(device.stat().st_mode), 'unexpected arm device alias target')
    raw = subprocess.check_output(['udevadm', 'info', '--query=property', '--name=' + str(device)],
                                  text=True, timeout=5.)
    props = dict(line.split('=', 1) for line in raw.splitlines() if '=' in line)
    require(props.get('ID_SERIAL_SHORT') == SERIAL, 'arm USB serial identity differs')
    return {'alias': str(alias), 'device': str(device), 'usb_serial': SERIAL}


def idle_process_check():
    existing = []
    names = {('__name:=' + prefix + name).encode()
             for name in NODES for prefix in ('', '/')}
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            argv = (proc / 'cmdline').read_bytes().split(b'\0')
        except (FileNotFoundError, ProcessLookupError):
            continue
        except PermissionError:
            # An unreadable same-user process cannot be silently certified idle.
            require(proc.stat().st_uid != os.getuid(), 'unreadable same-user process: ' + proc.name)
            continue
        if names.intersection(argv):
            existing.append(int(proc.name))
    require(not existing, 'existing ROS node processes: ' + repr(existing))
    for port in (11311, 8002):
        with socket.socket() as client:
            client.settimeout(.3)
            require(client.connect_ex(('127.0.0.1', port)) != 0,
                    'existing loopback listener: ' + str(port))


def parameter_source_check():
    root = ET.parse(W / '.runtime/full_latest_no_skin_20261010.launch').getroot()
    parameters = {p.attrib['name']: p.attrib.get('value') for p in root.findall('param')}
    require(parameters.get('/gui/joint_direct_mode') == 'true', 'startup is not manual mode')
    require(parameters.get('/alicia_d_driver_node/software_temperature_monitoring_enabled') == 'false'
            and parameters.get('/alicia_d_driver_node/temperature_poll_rate_hz') == '0.0',
            'temperature polling source policy changed')
    args = {a.attrib['name']: a.attrib.get('value') for a in root.find('include').findall('arg')}
    require(args.get('driver_port') == '/dev/alicia_arm' and args.get('driver_baudrate') == '1000000'
            and args.get('auto_torque_on_startup') == 'true' and args.get('self_check_poll_rate_hz') == '0.0'
            and args.get('start_tactile') == 'false' and args.get('start_gui') == 'true',
            'reviewed driver/GUI startup arguments changed')
    require((W / '.runtime/realsense-2.56.5/pyrealsense2.cpython-38-x86_64-linux-gnu.so').is_file(),
            'isolated RealSense SDK missing')
    require(Path('/run/user/1000/gdm/Xauthority').is_file(), 'reviewed desktop Xauthority missing')


def commands(session):
    old_path = PREVIOUS / 'startup.json'
    old = read(old_path)
    stamp = re.sub('[^a-z0-9-]', '-', session.name.lower())[:110]
    suffix = hashlib.sha256(str(session).encode()).hexdigest()[:10]
    result = {}
    for component in ('simulation', 'core'):
        prior = old[component]
        unit = 'alicia-gen18-' + component + '-' + stamp + '-' + suffix
        entry = (['/usr/bin/python3', str(W / '.runtime/full_startup_candidate/candidate_simulation_startup.py'), '--serve']
                 if component == 'simulation' else
                 ['/bin/bash', str(W / '.runtime/start_full_latest_frozen_two_observations_20261011.sh')])
        expected = ['systemd-run', '--user', '--collect', '--unit=' + prior['unit'],
                    '--property=Type=exec', '--property=Restart=no', '--property=WorkingDirectory=' + str(W),
                    '--property=StandardOutput=append:' + str(PREVIOUS / (component + '.log')),
                    '--property=StandardError=append:' + str(PREVIOUS / (component + '.log')),
                    '--setenv=ROS_LOG_DIR=' + str(PREVIOUS / (component + '_ros')), *entry]
        require(prior['command'] == expected, 'previous command template differs: ' + component)
        command = [part.replace(str(PREVIOUS), str(session)).replace(prior['unit'], unit)
                   for part in expected]
        result[component] = {'unit': unit, 'command': command}
    return result, {'path': str(old_path), 'sha256': sha(old_path)}


def save(session, value):
    storage_check(session)
    path = session / 'startup.json'
    temporary = session / ('startup.json.' + str(os.getpid()) + '.tmp')
    with temporary.open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', required=True, type=Path)
    parser.add_argument('--release', required=True, type=Path)
    args = parser.parse_args()
    session, release_path = args.session.resolve(), args.release.resolve(strict=True)
    storage = storage_check(session)
    current = read(session / 'startup.json') if (session / 'startup.json').exists() else {}
    require(not any(current.get(k) for k in ('core', 'simulation', 'launch_attempted')),
            'this session already attempted startup; no implicit retry')
    release = verify_release(release_path)
    parameter_source_check()
    serial = serial_check()
    idle_process_check()
    planned, template = commands(session)
    # Atomic ownership also prevents two concurrent invocations passing preflight.
    storage_check(session)
    with (session / 'cold_start_claim.json').open('x') as stream:
        json.dump({'pid': os.getpid(), 'claimed_ns': time.time_ns(), 'script_sha256': sha(__file__)}, stream)
        stream.write('\n')
    current.update(directory=str(session), source_manifest=str(release_path),
                   source_manifest_sha256=sha(release_path), test_summary=release['test_summary'],
                   startup_dependencies_sha256=DEPENDENCIES, startup_script_sha256=sha(__file__),
                   command_template=template, storage=storage, serial=serial, full_restart=True,
                   pre_start_existing_nodes=[], pre_start_listener_ports=[], pre_start_checked_ns=time.time_ns(),
                   automatic_grasp_running=False, automatic_grasp_started=False, start_tactile=False,
                   software_temperature_monitoring_enabled=False, temperature_poll_rate_hz=0.0,
                   temperature_queries_sent=False, torque_disable_commands_sent=False,
                   startup_positive_enable_configured=True, launch_attempted=True,
                   active_work='starting_reviewed_gen18_full_stack', planned_components=planned)
    save(session, current)
    for component in ('simulation', 'core'):
        row = dict(planned[component], requested_ns=time.time_ns(), dispatch_attempted=True)
        current[component] = row
        save(session, current)
        try:
            completed = subprocess.run(row['command'], capture_output=True, text=True, timeout=30.)
            row.update(returncode=completed.returncode, stdout=completed.stdout, stderr=completed.stderr)
        except Exception as exc:
            row.update(error=repr(exc), launch_state_uncertain=True)
            current['active_work'] = 'startup_dispatch_failed_no_automatic_retry'
            save(session, current)
            raise
        if completed.returncode != 0:
            current['active_work'] = 'startup_dispatch_failed_no_automatic_retry'
        save(session, current)
        print(json.dumps({'component': component, **row}, ensure_ascii=False), flush=True)
        require(completed.returncode == 0, 'component startup failed: ' + component)
    current['active_work'] = 'waiting_for_new_stack_acceptance'
    save(session, current)


if __name__ == '__main__':
    main()
