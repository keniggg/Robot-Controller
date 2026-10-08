#!/usr/bin/env python3
"""Archive offline summaries only; intentionally imports no ROS interface."""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MOUNT = Path('/media/zhuyupei/SanStand')
BASE = MOUNT / '实验记录/Robot-Controller'
OLD = BASE / 'ros_log/grasp_repair_resume_20260928T104059Z'
DOC = ROOT / 'docs/verification/2026-09-28-new-fixed-board-small-angle-diagnosis.md'
SESSIONS = [
    (5, 'bounded_joint_response_20260928T105805Z_23293', 'j5_multiframe_105805'),
    (2, 'bounded_joint_response_20260928T105915Z_24761', 'j2_multiframe_105915'),
    (3, 'bounded_joint3_response_20260928T110035Z_26293', 'j3_multiframe_110035'),
]


def read(path):
    return json.loads(path.read_text())


def source(path):
    return dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def main():
    if not os.path.ismount(str(MOUNT)):
        raise RuntimeError('SanStand is not mounted; no fallback')
    output = Path(read(ROOT / '.runtime/current_grasp_repair_resume.json')['directory'])
    if MOUNT not in output.resolve().parents or not output.is_dir():
        raise RuntimeError('Current resume directory must already exist on SanStand')
    sessions = []
    for joint, name, multi in SESSIONS:
        path = BASE / 'ros_log' / name
        report_path = OLD / multi / 'report.json'
        report = read(report_path)
        events = [json.loads(line) for line in (path / 'events.jsonl').read_text().splitlines() if line]
        depth = read(path / 'depth_vs_pnp_diagnostic.json')
        by_geometry = {row['geometry']: row for row in depth['samples']}
        key_depth = {label: by_geometry[file] for label, file in [
            ('baseline', 'baseline_geometry.json'),
            ('outbound_stationary_before_return', 'return_original_sdk_geometry.json'),
            ('return_stationary', 'return_post_settle_geometry.json'),
        ] if file in by_geometry}
        row = dict(joint=joint, directory=str(path), epoch_ns=read(path / 'plan.json')['frozen_epoch_ns'],
                   terminal=read(path / 'terminal.json'), command_events=[e for e in events if e['event'] in ('command', 'explicit_initial_sync_once')],
                   multiframe_report=report, representative_source_paired_depth=key_depth,
                   raw_sdk_accepted_post_settle=read(path / 'motion_analysis.json')['post_settle_windows'],
                   sources=[source(report_path)] + [source(path / f) for f in (
                       'plan.json', 'terminal.json', 'events.jsonl', 'depth_vs_pnp_diagnostic.json', 'motion_analysis.json', 'robot_description.urdf')])
        for filename in ('same_sdk_return_comparison.json', 'return_path_decomposition.json'):
            if (path / filename).exists():
                row[filename[:-5]] = read(path / filename)
                row['sources'].append(source(path / filename))
        sessions.append(row)
    result = dict(schema=1, generated_utc=datetime.now(timezone.utc).isoformat(),
                  scope='Offline historical evidence only. No ROS calls, commands, parameter changes, hand-eye refit or new motion authorization.',
                  archive_policy='local_only', upload_enabled=False, delete_enabled=False,
                  sessions=sessions, report_source=source(DOC),
                  limitations=[
                      'All three old experiments belong to epoch 1790591799745386018; never replay their plans after restart.',
                      'No comparison of absolute board poses across separately positioned boards.',
                      'Multiframe quality/TF rows and exact archived RGB rows are separately counted; return raw-RGB subsets are empty.',
                      'Pair combinations are correlated descriptive statistics, not independent confidence intervals.',
                      'Depth and RGB-PnP share camera geometry assumptions and unmeasured printed board scale; neither is external truth.',
                      'J3 return was not commanded, so no return measurement exists.',
                      'Local observations do not resolve or compensate the prior 10–11 mm failed-grasp height error.',
                  ])
    (output / 'new_fixed_board_diagnosis.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    (output / 'new_fixed_board_diagnosis.md').write_bytes(DOC.read_bytes())
    (output / 'compile_new_fixed_board_diagnosis.py').write_bytes(Path(__file__).read_bytes())
    (BASE / '文档' / DOC.name).write_bytes(DOC.read_bytes())
    print(json.dumps(dict(output=str(output), sessions=len(sessions), source_report_sha256=source(DOC)['sha256']), ensure_ascii=False))


if __name__ == '__main__':
    main()
