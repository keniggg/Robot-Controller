#!/usr/bin/env python3
"""Preserve prior regression scope and bind new RGB-D integration coverage."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess

WORK = Path(__file__).resolve().parents[1]
MOUNT = Path('/media/zhuyupei/SanStand')
OUT = MOUNT / '实验记录/Robot-Controller/ros_log/full_ros_resume_20261004T002930Z'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    if not MOUNT.is_mount():
        raise RuntimeError('SanStand unavailable; no fallback')
    previous = json.loads((WORK / '.runtime/rgb_acquisition_validation_20261004.json').read_text())
    command = previous['command'][:]
    patterns = {
        'src/alicia_flexible_grasp_supervisor/tests': (
            'test_rgbd_rectification*.py', 'test_pair_rectification_integration.py', 'test_color_projection.py', 'test_camera*.py',
            'test_realsense*.py', 'test_latest_rgbd_publication.py'),
        'src/alicia_grasp_modes/tests': ('test_silhouette*.py', 'test_remote_modes.py'),
        'tools/tests': ('test_camera_runtime_metadata.py',),
    }
    for directory, globs in patterns.items():
        for glob in globs:
            for path in sorted((WORK / directory).glob(glob)):
                rel = str(path.relative_to(WORK))
                if rel not in command:
                    command.append(rel)
    tests = [arg for arg in command[command.index('-ra')+1:] if arg.endswith('.py')]
    sources = list(previous['source_sha256'])
    additions = [
        'src/alicia_flexible_grasp_supervisor/scripts/camera_node.py',
        'src/alicia_flexible_grasp_supervisor/config/grasp_supervisor.yaml',
        'src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_instance.py',
        'src/alicia_grasp_modes/src/alicia_grasp_modes/silhouette_snapshot_diagnostic.py',
        'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py',
        '.runtime/validate_rectification_samples_20261004.py',
        '.runtime/run_rectification_regression_20261004.py',
    ]
    additions += [str(p.relative_to(WORK)) for p in sorted(
        (WORK / 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision').glob('*.py'))]
    for rel in additions:
        if (WORK / rel).is_file() and rel not in sources:
            sources.append(rel)
    before = {rel:sha(WORK / rel) for rel in sources + tests}
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    log = OUT / ('rgbd_rectification_regression_' + stamp + '.log')
    with log.open('w') as stream:
        run = subprocess.run(command, cwd=str(WORK), stdout=stream, stderr=subprocess.STDOUT)
    after = {rel:sha(WORK / rel) for rel in before}
    unchanged = before == after
    report = dict(recorded_at_utc=datetime.now(timezone.utc).isoformat(), command=command,
        test_exit_code=run.returncode, test_result=log.read_text().splitlines()[-1], test_log=str(log),
        source_unchanged_during_test=unchanged,
        changed_during_test=[rel for rel in before if before[rel] != after[rel]],
        source_sha256={rel:before[rel] for rel in sources}, test_sha256={rel:before[rel] for rel in tests},
        previous_validation_log=previous['test_log'])
    path = OUT / ('rgbd_rectification_validation_' + stamp + '.json')
    path.write_text(json.dumps(report, indent=2) + '\n')
    if run.returncode == 0 and unchanged:
        (WORK / '.runtime/rgbd_rectification_validation_20261004.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(dict(report=str(path), test_result=report['test_result'],
                         test_exit_code=run.returncode, source_unchanged=unchanged)), flush=True)
    raise SystemExit(run.returncode or (0 if unchanged else 2))


if __name__ == '__main__':
    main()
