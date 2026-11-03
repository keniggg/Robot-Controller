#!/usr/bin/env python3
"""Check the configured remote dependency before the authorized grasp launcher.

This adds no motion authority. The original launcher still owns recording,
current-scene authorization, selection, feedback and execution guards.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time


W = Path(__file__).resolve().parents[1]
STORE = Path('/media/zhuyupei/SanStand')
SESSION = STORE / '实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen56_runtime'
CHECKER = W / '.runtime/verify_active_remote_readiness_20261020.py'
LAUNCHER = W / '.runtime/start_gen56_recorded_attempt_20261020.sh'
LAUNCHER_SHA = 'e6e64ee60c740845c55334fc505b1849dd9cf724fbdb030cc5d642d84db42a76'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--startup-binding', type=Path, required=True)
    parser.add_argument('--startup-binding-sha256', required=True)
    parser.add_argument('--checker-sha256', required=True)
    args = parser.parse_args()
    scene = args.scene.resolve(strict=True)
    if (not STORE.is_mount() or scene.parent != SESSION or not scene.name.startswith('scene_start_')
            or sha(CHECKER) != args.checker_sha256 or sha(LAUNCHER) != LAUNCHER_SHA):
        raise RuntimeError('Reviewed launcher, dependency checker or scene location differs')
    preflight = json.loads((scene / 'preflight.json').read_text())
    if not (scene / 'operator_observation.json').is_file():
        raise RuntimeError('Current image review and continuing authorization receipt required')
    output = scene / 'remote_dependency_readiness'
    completed = subprocess.run([
        sys.executable, '-B', str(CHECKER), '--output', str(output),
        '--startup-binding', str(args.startup_binding.resolve(strict=True)),
        '--startup-binding-sha256', args.startup_binding_sha256,
    ], check=False)
    if completed.returncode != 0:
        print('REMOTE_DEPENDENCY_NOT_READY: recorded grasp launcher was not started', flush=True)
        return 2
    result_path = output / 'result.json'
    result = json.loads(result_path.read_text())
    if (result.get('passed') is not True or result.get('dependency_ready') is not True
            or result.get('helper_sha256') != args.checker_sha256
            or result.get('startup_binding_sha256') != args.startup_binding_sha256
            or result['after']['selection'] != preflight['selection']
            or not 0 <= time.time_ns() - result['finished_ns'] <= 5_000_000_000
            or sha(CHECKER) != args.checker_sha256 or sha(LAUNCHER) != LAUNCHER_SHA):
        raise RuntimeError('Current dependency acceptance differs, changed or expired')
    receipt = dict(checked_ns=time.time_ns(), checker_sha256=args.checker_sha256,
                   readiness=str(result_path), readiness_sha256=sha(result_path),
                   original_launcher=str(LAUNCHER), original_launcher_sha256=LAUNCHER_SHA,
                   next_action='exec original authorized recorded launcher',
                   new_motion_authority_created=False)
    with (scene / 'remote_dependency_launch_receipt.json').open('x') as stream:
        json.dump(receipt, stream, indent=2)
        stream.write('\n')
    os.execv('/bin/bash', ['/bin/bash', str(LAUNCHER), str(scene)])


if __name__ == '__main__':
    sys.exit(main())
