#!/usr/bin/env python3
"""Run the existing WSL GraspNet service and the verified MuJoCo deployment.

Activate grasp6d118 before running. This launcher never sends robot commands.
Ctrl-C stops only processes started by this invocation; reused services stay up.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import urllib.request

DEPLOYMENT_ID = '6047f9a740465aaf01759f261b8aff436bbdbf47adea951624b7d245f6e3aec5'
SOURCE_SHA256 = '919ddaf6c9499c86eb913ad5d44bd40dbb6aabde4e5268c0267c9fa7601b7dd6'
SIM_CONTRACT = {
    'ok': True, 'backend': 'mujoco', 'mujoco': '3.2.3',
    'deployment_id': DEPLOYMENT_ID, 'source_sha256': SOURCE_SHA256,
    'max_snapshot_age_sec': 120.0, 'min_lift_success_m': 0.015,
    'initial_unknown_ik_position_tolerance_m': 0.0002,
    'initial_unknown_ik_orientation_tolerance_rad': 0.005,
    'strict_dynamic_object_lift_required': True,
    'request_bound_gripper_close_contract': True,
    'opposed_contact_gate_version': 1, 'gripper_joint_effort_limit_n': 5.0,
    'max_lift_joint_speed_rad_s': 0.08,
}


def read_health(port):
    # Local service health must not be routed through a shell HTTP proxy.
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open('http://127.0.0.1:%d/health' % port, timeout=3) as response:
        value = json.load(response)
    if not isinstance(value, dict):
        raise RuntimeError('Health response is not a JSON object on port %d' % port)
    return value


def port_open(port):
    try:
        with socket.create_connection(('127.0.0.1', port), timeout=1):
            return True
    except OSError:
        return False


def require_fields(actual, expected, name):
    differences = {key: {'expected': value, 'actual': actual.get(key)}
                   for key, value in expected.items() if actual.get(key) != value}
    if differences:
        raise RuntimeError('%s health mismatch: %s' % (name, json.dumps(differences)))


def validate_simulation(health):
    # A dedicated simulation service deliberately has top-level ok=false:
    # its embedded GraspNet backend is not configured.
    require_fields(health, {'protocol_version': 3}, 'MuJoCo protocol')
    require_fields(health.get('digital_twin', {}), SIM_CONTRACT, 'MuJoCo')


def validate_graspnet(health, baseline_root, checkpoint, device):
    require_fields(health, {'protocol_version': 3}, 'GraspNet protocol')
    expected = {'ok': True, 'backend': 'graspnet_baseline', 'loaded': True,
                'baseline_root': str(baseline_root), 'checkpoint': str(checkpoint)}
    if device.startswith('cuda'):
        expected['torch_cuda_available'] = True
    require_fields(health.get('grasp_backend', health), expected, 'GraspNet')


def verify_bundle(bundle):
    manifest = json.loads((bundle / 'manifest.json').read_text())
    if manifest.get('deployment_id') != DEPLOYMENT_ID:
        raise RuntimeError('Unexpected MuJoCo deployment: %s' % bundle)
    hashes = manifest['sha256']
    if hashes.get('tools/mujoco_digital_twin_server.py') != SOURCE_SHA256:
        raise RuntimeError('Unexpected MuJoCo source fingerprint')
    for relative, expected in hashes.items():
        path = (bundle / relative).resolve()
        if bundle.resolve() not in path.parents:
            raise RuntimeError('Invalid deployment manifest path: ' + relative)
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise RuntimeError('Deployment file changed: ' + relative)


class ServiceStack:
    def __init__(self, startup_timeout=180):
        self.startup_timeout = startup_timeout
        self.owned = []

    def ensure(self, name, port, validator, command, cwd, env):
        if port_open(port):
            validator(read_health(port))
            print('REUSE %s port=%d (existing process retained)' % (name, port), flush=True)
            return
        print('START %s port=%d' % (name, port), flush=True)
        process = subprocess.Popen(command, cwd=str(cwd), env=env, start_new_session=True)
        self.owned.append((name, process))
        deadline = time.monotonic() + self.startup_timeout
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError('%s exited during startup: %d' % (name, process.returncode))
            if port_open(port):
                validator(read_health(port))
                print('READY %s port=%d' % (name, port), flush=True)
                return
            time.sleep(0.5)
        raise RuntimeError('%s startup timed out after %ss' % (name, self.startup_timeout))

    def supervise(self, checks):
        failures = {name: 0 for name, _, _ in checks}
        while True:
            time.sleep(5)
            for name, process in self.owned:
                if process.poll() is not None:
                    raise RuntimeError('%s exited: %d' % (name, process.returncode))
            for name, port, validator in checks:
                try:
                    validator(read_health(port))
                    failures[name] = 0
                except Exception as exc:
                    failures[name] += 1
                    print('HEALTH_WARNING %s %s' % (name, exc), flush=True)
                    if failures[name] >= 3:
                        raise RuntimeError('%s failed three consecutive health checks' % name)

    def close(self):
        # Each child has its own process group, including shell descendants.
        # Never signal a process discovered by probing a pre-existing port.
        for name, process in reversed(self.owned):
            print('STOP_OWNED %s pid=%d' % (name, process.pid), flush=True)
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + 5
        for _, process in reversed(self.owned):
            try:
                process.wait(timeout=max(0.01, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()
        self.owned.clear()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo-root', type=Path, default=Path(os.environ.get(
        'ALICIA_WSL_REPO', str(Path.home() / 'grasp6d_ws/Robot-Controller-v3'))))
    parser.add_argument('--bundle-root', type=Path, default=Path(os.environ.get(
        'ALICIA_MUJOCO_DEPLOYMENT', str(Path(os.environ.get('XDG_DATA_HOME',
        str(Path.home() / '.local/share'))) / ('alicia-mujoco-20260923-' + DEPLOYMENT_ID[:12])))))
    parser.add_argument('--check-only', action='store_true', help='Check both services without starting any process')
    args = parser.parse_args(argv)
    repo, bundle = args.repo_root.expanduser().resolve(), args.bundle_root.expanduser().resolve()
    baseline = Path(os.environ.get('GRASPNET_BASELINE_ROOT', str(Path.home() / 'grasp6d_ws/graspnet-baseline'))).expanduser().resolve()
    checkpoint = Path(os.environ.get('GRASPNET_CHECKPOINT', str(Path.home() / 'grasp6d_ws/checkpoints/checkpoint-rs.tar'))).expanduser().resolve()
    device = os.environ.get('GRASPNET_DEVICE', 'cuda:0')
    model = Path(os.environ.get('MUJOCO_ALICIA_MODEL_XML', str(repo / 'src/arm-mujoco/synriard/mjcf/Alicia_D_v5_6/Alicia_D_v5_6_gripper_50mm.xml'))).expanduser().resolve()
    verify_bundle(bundle)
    grasp_validator = lambda health: validate_graspnet(health, baseline, checkpoint, device)
    checks = [('GraspNet', 8000, grasp_validator), ('MuJoCo', 8001, validate_simulation)]
    if args.check_only:
        for name, port, validator in checks:
            validator(read_health(port))
            print('VERIFIED %s port=%d' % (name, port), flush=True)
        return 0
    for required in (repo / 'tools/start_mujoco_digital_twin_wsl.sh', baseline, checkpoint, model):
        if not required.exists():
            raise RuntimeError('Required path missing: %s' % required)
    env = dict(os.environ, GRASPNET_BASELINE_ROOT=str(baseline),
               GRASPNET_CHECKPOINT=str(checkpoint), GRASPNET_DEVICE=device,
               MUJOCO_ALICIA_MODEL_XML=str(model), MUJOCO_TWIN_HOST='0.0.0.0',
               MUJOCO_TWIN_PORT='8000', PYTHONUNBUFFERED='1', PYTHONDONTWRITEBYTECODE='1')
    stack = ServiceStack()
    try:
        stack.ensure('MuJoCo', 8001, validate_simulation,
                     ['bash', str(bundle / 'start_wsl.sh')], bundle, env)
        stack.ensure('GraspNet', 8000, grasp_validator,
                     ['bash', str(repo / 'tools/start_mujoco_digital_twin_wsl.sh'),
                      '--pass-score', '80', '--min-lift-success-m', '0.015',
                      '--max-snapshot-age-sec', '120.0', '--warmup'], repo, env)
        print('READY_WSL_GRASP_STACK GraspNet=:8000 MuJoCo=:8001 deployment=' + DEPLOYMENT_ID, flush=True)
        print('Keep this terminal open. Ctrl-C stops only services started here.', flush=True)
        stack.supervise(checks)
    finally:
        stack.close()
    return 0


if __name__ == '__main__':
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
    except Exception as exc:
        print('WSL_STACK_ERROR: %s' % exc, file=sys.stderr, flush=True)
        sys.exit(1)
