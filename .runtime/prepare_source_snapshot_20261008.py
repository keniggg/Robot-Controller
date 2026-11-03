#!/usr/bin/env python3
"""Copy source-only restore dependencies; never start ROS or alter live config."""
import hashlib
import json
import os
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROJECT = Path('/home/zhuyupei/alicia_wa_full')
DEST = ROOT / 'deployment/2026-10-08'
EXTENSIONS = {'.py', '.sh', '.launch', '.yaml', '.yml', '.urdf', '.xml',
              '.cpp', '.hpp', '.h', '.bash', '.service', '.ini', '.js', '.md'}
SKIP = {'__pycache__', '.pytest_cache', 'build', 'devel', 'CMakeFiles',
        'realsense-2.56.5', 'mujoco-3.2.3'}
JSON_CONFIGS = {
    'd405_carton_depth_baseline.json', 'experiment_storage.json',
    'full_startup_candidate/legacy_owner_commands.json',
    'full_startup_candidate/staging_manifest.json',
    'candidate_models/expanded_metric_20260929/staging_manifest.json',
    'candidate_simulation/pins.json', 'candidate_simulation/simulation_manifest.json',
}


def main():
    pairs = []
    for base, directories, files in os.walk(str(ROOT / '.runtime')):
        directories[:] = [d for d in directories if d not in SKIP]
        for name in files:
            source = Path(base) / name
            rel = source.relative_to(ROOT / '.runtime')
            if source.is_symlink():
                continue
            if source.suffix in EXTENSIONS or str(rel) in JSON_CONFIGS:
                pairs.append((source, DEST / 'runtime' / rel, 'runtime/' + str(rel)))
    for rel in (
        '.ros_log/ros_live_20260922_session/start_ros.sh',
        '.ros_log/ros_live_20260922_session/start_camera_sdk.sh',
        '.ros_log/ros_live_20260922_session/resume_modes.launch',
        '.ros_log/unknown_grasp_20260923/sanstand_storage.sh',
    ):
        pairs.append((PROJECT / rel, DEST / 'root_workspace' / rel,
                      'root_workspace/' + rel))
    sdk_manifest = ROOT / '.runtime/realsense-2.56.5/install_manifest.json'
    pairs.append((sdk_manifest, DEST / 'runtime/realsense-2.56.5/install_manifest.json',
                  'runtime/realsense-2.56.5/install_manifest.json'))
    extras = {
        '/home/zhuyupei/.ros/easy_handeye/d405_cc200_auto_20260927_validated_eye_on_hand.yaml':
            'external_config/d405_cc200_auto_20260927_validated_eye_on_hand.yaml',
        '/home/zhuyupei/.config/systemd/user/alicia-candidate-simulation.service':
            'external_config/alicia-candidate-simulation.service',
        '/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/grasp_repair_resume_20260928T120606Z/expanded_metric_mjcf_review_20260929/server_source_contract.json':
            'external_config/server_source_contract.json',
        '/home/zhuyupei/alicia_wa_full/AGENTS.md': 'AGENTS.md',
    }
    for source, rel in extras.items():
        pairs.append((Path(source), DEST / rel, rel))
    entries = []
    for source, target, rel in sorted(pairs, key=lambda p: p[2]):
        data = source.read_bytes()
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(str(source), str(target))
        assert target.read_bytes() == data
        entries.append(dict(source=str(source), snapshot=rel, bytes=len(data),
                            sha256=hashlib.sha256(data).hexdigest(),
                            mode=oct(source.stat().st_mode & 0o777)))
    manifest = dict(schema=1, label='10.8日，一次性抓取通过的完整代码',
                    scope='current source snapshot; historical success is not a physical retest',
                    source_worktree=str(ROOT), copied_files=entries,
                    excluded=['experiment recordings and runtime results',
                              'build/devel and caches', 'credentials',
                              'third-party native SDK binaries; restore pinned versions separately'])
    (DEST / 'source_dependencies.json').write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(copied_files=len(entries), bytes=sum(x['bytes'] for x in entries),
                          destination=str(DEST)), ensure_ascii=False))


if __name__ == '__main__':
    main()
