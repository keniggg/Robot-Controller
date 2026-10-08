#!/usr/bin/env python3
"""Staged full-start dependency; no ROS imports, node control, or robot commands.

Default is review-only. --serve starts the already frozen simulation wrapper;
--wait-ready only reads its loopback /health and existing model bindings.
"""
import argparse
import hashlib
import json
import os
import subprocess
from pathlib import Path
import tempfile
import time
import urllib.error
import urllib.request

PROJECT = Path(__file__).resolve().parents[2]
MOUNT = Path('/media/zhuyupei/SanStand')
STORAGE_ROOT = MOUNT / '实验记录/Robot-Controller/ros_log'
START = PROJECT / '.runtime/candidate_simulation/start_candidate.sh'
START_SHA = 'f781bab7329eca0defcf3242df7f0bab38da1b08eb717e9b9bac5e9baf4e23c3'
MANIFEST = PROJECT / '.runtime/candidate_simulation/simulation_manifest.json'
MANIFEST_SHA = 'b9a642e51e83f7da591a2188020255e590ce05f1d1d05f4c66ef0b820a891d73'
URL = 'http://127.0.0.1:8002'
UNIT = 'alicia-candidate-simulation.service'
IDENTITY = {
    'candidate_model_sha256': '7812d6b69e854ed9f37b4e3cbd813ecd53dab3504065b2d04d287abc6fd852e0',
    'candidate_handeye_sha256': '637701accca5eb5042075c213c5e00e9b189d3cd7788d916c8c06bf89f7b1a0c',
    'candidate_pair_sha256': '841ca1491fb2e05bced1a1f962749a496a9c4b9944d2833eefa2c3f1b02c3789',
    'model_xml_sha256': 'dc9e8bbc1a6764485cfd9412d68f5470713ef53a8ee0f8304cdb233ac96457cc',
}


def frozen_manifest():
    if hashlib.sha256(START.read_bytes()).hexdigest() != START_SHA:
        raise ValueError('frozen candidate start script changed')
    manifest_bytes = MANIFEST.read_bytes()
    if hashlib.sha256(manifest_bytes).hexdigest() != MANIFEST_SHA:
        raise ValueError('frozen simulation manifest changed')
    manifest = json.loads(manifest_bytes)
    if any(manifest.get(k) != v for k, v in IDENTITY.items()):
        raise ValueError('candidate manifest identity changed')
    if manifest.get('server_url') != URL:
        raise ValueError('candidate simulation endpoint changed')
    bindings = manifest.get('health_bindings', {})
    for key, value in IDENTITY.items():
        if bindings.get('digital_twin.' + key) != value:
            raise ValueError('candidate health identity binding changed')
    return manifest


def require_storage():
    if not os.path.ismount(str(MOUNT)):
        raise RuntimeError('EXPERIMENT_STORAGE_UNAVAILABLE: SanStand is not mounted')
    root = STORAGE_ROOT.resolve()
    if MOUNT.resolve() not in root.parents:
        raise RuntimeError('experiment path escapes SanStand')
    return root


def reserve_output(manifest):
    root = require_storage()
    root.mkdir(parents=True, exist_ok=True)
    require_storage()
    prefix = time.strftime('candidate_simulation_full_start_%Y%m%dT%H%M%SZ_', time.gmtime())
    invocation = Path(tempfile.mkdtemp(prefix=prefix, dir=str(root)))
    # The frozen wrapper requires its own output directory NOT to exist.
    output = invocation / 'service'
    metadata = dict(schema=1, scope='local_candidate_simulation_only',
                    allocated_at_ns=time.time_ns(), pid=os.getpid(),
                    invocation_id=os.environ.get('INVOCATION_ID', ''),
                    output=str(output), endpoint=URL, start_script=str(START),
                    health_bindings=manifest['health_bindings'],
                    start_script_sha256=START_SHA, **IDENTITY)
    with (invocation / 'startup_request.json').open('x') as stream:
        json.dump(metadata, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    return invocation, output


def run_server():
    manifest = frozen_manifest()
    invocation, output = reserve_output(manifest)
    with (invocation / 'bootstrap.log').open('xb', buffering=0) as log:
        require_storage()
        os.dup2(log.fileno(), 1)
        os.dup2(log.fileno(), 2)
        os.execv('/bin/bash', ['/bin/bash', str(START), '--serve', '--output', str(output)])


def validate_health(health, manifest):
    # This dedicated backend deliberately has top-level inference ok=false.
    if health.get('digital_twin', {}).get('ok') is not True:
        raise ValueError('candidate digital twin is not ready')
    for dotted, expected in manifest['health_bindings'].items():
        actual = health
        for part in dotted.split('.'):
            if not isinstance(actual, dict) or part not in actual:
                raise ValueError('missing candidate health binding: ' + dotted)
            actual = actual[part]
        if actual != expected or (isinstance(expected, bool) and type(actual) is not bool):
            raise ValueError('candidate health binding mismatch: ' + dotted)



def validate_service_properties(properties):
    """An old healthy server cannot substitute for the requested unit."""
    if properties.get('ActiveState') != 'active' or properties.get('SubState') != 'running':
        raise RuntimeError('requested candidate simulation unit is not running')
    try:
        pid = int(properties['MainPID'])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError('candidate service MainPID is missing') from exc
    invocation = str(properties.get('InvocationID', ''))
    group = str(properties.get('ControlGroup', ''))
    if pid <= 1 or len(invocation) != 32 or any(c not in '0123456789abcdef' for c in invocation):
        raise ValueError('candidate service invocation identity is invalid')
    if not group.startswith('/') or group.rsplit('/', 1)[-1] != UNIT:
        raise ValueError('candidate service control group is invalid')
    return dict(pid=pid, invocation_id=invocation, control_group=group)


def service_snapshot():
    completed = subprocess.run(
        ['systemctl', '--user', 'show', UNIT, '--property=ActiveState',
         '--property=SubState', '--property=MainPID', '--property=InvocationID',
         '--property=ControlGroup'], check=True, text=True, capture_output=True, timeout=2.)
    return validate_service_properties(dict(line.split('=', 1)
        for line in completed.stdout.splitlines() if '=' in line))


def process_in_group(pid, group):
    try:
        lines = (Path('/proc') / str(pid) / 'cgroup').read_text().splitlines()
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return False
    return any(line.rsplit(':', 1)[-1] == group
               or line.rsplit(':', 1)[-1].startswith(group + '/') for line in lines)


def require_listener_owned_by(identity):
    # The frozen server binds IPv4 loopback: its listening socket must belong
    # to this service invocation, not the old timestamped service on port 8002.
    inodes = {row[9] for line in Path('/proc/net/tcp').read_text().splitlines()[1:]
              if len(row := line.split()) > 9 and row[1] == '0100007F:1F42' and row[3] == '0A'}
    if not inodes:
        raise OSError('candidate loopback listener not available yet')
    if not process_in_group(identity['pid'], identity['control_group']):
        raise RuntimeError('candidate MainPID no longer belongs to its control group')
    owned = set()
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or not process_in_group(int(entry.name), identity['control_group']):
            continue
        try:
            for fd in (entry / 'fd').iterdir():
                try:
                    target = os.readlink(str(fd))
                except (FileNotFoundError, PermissionError):
                    continue
                if target.startswith('socket:[') and target.endswith(']'):
                    owned.add(target[8:-1])
        except (FileNotFoundError, PermissionError):
            continue
    if not inodes.issubset(owned):
        raise RuntimeError('port 8002 listener belongs to another service; refusing old health')


def wait_ready(timeout=40.0):
    manifest = frozen_manifest()
    require_storage()
    expected = service_snapshot()
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError('candidate simulation did not become ready within 40 seconds')
        if service_snapshot() != expected:
            raise RuntimeError('candidate service invocation changed during readiness wait')
        try:
            require_listener_owned_by(expected)
            with urllib.request.urlopen(URL + '/health', timeout=min(1.0, remaining)) as response:
                health = json.loads(response.read())
        except (OSError, urllib.error.URLError):
            time.sleep(min(.2, max(0., deadline - time.monotonic())))
            continue
        validate_health(health, manifest)
        if service_snapshot() != expected:
            raise RuntimeError('candidate service invocation changed during health read')
        require_listener_owned_by(expected)
        print(json.dumps(dict(status='candidate_simulation_ready', endpoint=URL,
                              service_identity=expected, **IDENTITY)))
        return


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--serve', action='store_true')
    modes.add_argument('--wait-ready', action='store_true')
    args = parser.parse_args(argv)
    if args.serve:
        run_server()
    elif args.wait_ready:
        wait_ready()
    else:
        frozen_manifest()
        print(json.dumps(dict(status='staged_review_only', endpoint=URL,
                              creates_output=False, opens_network=False,
                              starts_service=False, **IDENTITY), indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
