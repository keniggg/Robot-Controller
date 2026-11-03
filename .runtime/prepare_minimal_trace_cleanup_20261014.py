#!/usr/bin/env python3
"""Create a finite cleanup list under the user's minimal-history authorization."""
import hashlib
import json
import os
from pathlib import Path
import stat
import struct
import sys
import time

WORK = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
sys.path.insert(0, str(WORK / 'tools'))
from minimal_experiment_trace import TraceStore, atomic, identity, sha_file

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RUN = ROOT / 'maintenance/minimal_trace_cleanup_1791956657642651189'
TEXT = {'.json', '.jsonl', '.md', '.log', '.txt', '.yaml', '.yml', '.xml', '.csv', '.tsv', '.rosmsg', '.roswire'}
CURRENT = [
    'maintenance', '文档', 'archive_inventory', 'storage_validation', 'ros_default_log',
    'ros_log/full_latest_gen18_restart_20261014T012216Z',
    'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012',
    'ros_log/full_latest_contact_admission_20261013T011103Z/attempt_6_full_frames',
    'grasp_records/20261014T015629.706423Z_f0e3415e597e418fa8d9a43fce55ab9f',
    'grasp_records/20261014T025930.273999Z_a85100322c834ad3834e547a9abad6c8',
    'grasp_records/20261013T123054.826269Z_1578aafa633140ab9ac8b88d9513de5e',
    'grasp_records/20261012T140905.069509Z_022d4230fcb4456d8808a52b6c1db9ca',
]
CAL_WORDS = ('handeye', 'calibration', 'charuco', 'fixed_board', 'orthogonal_probe', 'board_hold')


def below(relative, prefixes):
    return any(relative == p or relative.startswith(p + '/') for p in prefixes)


def indexed_bag(path):
    with path.open('rb') as f:
        if f.read(13) != b'#ROSBAG V2.0\n':
            return False
        length = struct.unpack('<I', f.read(4))[0]
        if length > 1024 * 1024:
            return False
        header = f.read(length)
    fields, offset = {}, 0
    while offset < len(header):
        length = struct.unpack('<I', header[offset:offset + 4])[0]
        offset += 4
        k, v = header[offset:offset + length].split(b'=', 1)
        fields[k] = v
        offset += length
    return fields.get(b'op') == b'\x03' and struct.unpack('<Q', fields[b'index_pos'])[0] > 0


def physical_success(directory):
    for name in ('physical_result_user_confirmed.json', 'result_summary.json'):
        path = directory / name
        if path.exists():
            v = json.loads(path.read_text())
            if v.get('physical_grip_confirmed_by_user') is True or v.get('result') == 'success':
                return True
    return False


def main():
    assert ROOT.parents[1].is_mount()
    cutoff = time.time_ns() - 3600 * 1000000000
    files, all_dirs = [], []
    for base, dirs, names in os.walk(str(ROOT), followlinks=False):
        directory = Path(base)
        if directory.is_symlink():
            dirs[:] = []
            continue
        all_dirs.append(directory)
        for name in names:
            path = directory / name
            info = path.lstat()
            if stat.S_ISREG(info.st_mode) and info.st_nlink == 1:
                files.append((path, info))
    excluded = set(CURRENT)
    exclusion_reasons = {p: 'Current unresolved representative data, live controls, calibration or existing maintenance/migration material' for p in CURRENT}
    migrated = set()
    for directory in all_dirs:
        relative = str(directory.relative_to(ROOT))
        if directory == ROOT:
            continue
        if (directory / 'github_cleanup_index.json').exists() or (directory / 'archive_manifest.json').exists() or (directory / 'recovery_index.json').exists():
            migrated.add(relative)
            excluded.add(relative)
            exclusion_reasons[relative] = 'Previously migrated/archive-indexed material: leave current state unchanged'
        if any(word in relative.lower() for word in CAL_WORDS):
            excluded.add(relative)
            exclusion_reasons[relative] = 'Calibration/reprojection/board reference data retained'
        if physical_success(directory):
            excluded.add(relative)
            exclusion_reasons[relative] = 'Keep representative physical-success reference'
    mirror = ROOT / 'ros_log/.active_ros_log_mirror.json'
    if mirror.exists():
        v = json.loads(mirror.read_text())
        for field in ('files', 'aliases'):
            for name in v.get(field, {}):
                excluded.add('ros_log/' + name)
                exclusion_reasons['ros_log/' + name] = 'Live log mirror target'
    excluded.update(('ros_log/.active_ros_log_mirror.json', 'ros_log/.active_ros_log_mirror.lock'))
    # Collapse nested exclusions, while keeping their reasons in the audit.
    prefixes = []
    for name in sorted(excluded, key=lambda n: (n.count('/'), n)):
        if not below(name, prefixes):
            prefixes.append(name)
    text, raw, skipped = [], [], []
    for path, info in files:
        relative = str(path.relative_to(ROOT))
        if below(relative, prefixes) or info.st_mtime_ns > cutoff:
            continue
        if relative.split('/')[0] not in ('ros_log', 'grasp_records'):
            continue
        suffix = path.suffix.lower()
        rotated_log = '.log.' in path.name
        if suffix in TEXT or rotated_log:
            remove = relative.startswith('ros_log/') and not path.name.endswith('.lock') and path.name not in (
                'current_progress.json', 'archive_config.json', 'startup.json', 'control.json', 'owner_registry.json')
            text.append(dict(path=relative, identity=identity(info), allocated_bytes=info.st_blocks * 512,
                             kind='exact_trace_text', remove_original=remove))
        elif suffix == '.bag' and info.st_size >= 64 * 1024 * 1024:
            try:
                if not indexed_bag(path):
                    skipped.append(dict(path=relative, reason='Not a finalized indexed rosbag'))
                    continue
                evidence = [str(p.relative_to(ROOT)) for p in path.parent.iterdir()
                            if p.is_file() and p.suffix in ('.json', '.md', '.txt', '.log')
                            and p.stat().st_mtime_ns <= cutoff and not below(str(p.relative_to(ROOT)), prefixes)]
                if len(evidence) < 2:
                    skipped.append(dict(path=relative, reason='Insufficient problem/result context'))
                    continue
                outcome = 'diagnostic_result_not_classified'
                expected = None
                manifest = path.parent / 'manifest.json'
                if relative.startswith('grasp_records/'):
                    v = json.loads(manifest.read_text())
                    o = json.loads((path.parent / 'recorder_outcome.json').read_text())
                    if v.get('state') != 'complete' or v.get('recorder_closed') is not True or o.get('bag_closed') is not True:
                        skipped.append(dict(path=relative, reason='Canonical recording not independently closed'))
                        continue
                    if v.get('result_evidence', {}).get('observations'):
                        skipped.append(dict(path=relative, reason='Operator observation retained as physical result reference'))
                        continue
                    outcome = v.get('result', 'unknown')
                    expected = next((item.get('sha256') for item in v.get('files', []) if item['path'] == path.name), None)
                raw.append(dict(path=relative, identity=identity(info), allocated_bytes=info.st_blocks * 512,
                                kind='raw_bag', indexed_closed=True, evidence_paths=evidence,
                                original_result=outcome, expected_sha256=expected))
            except (ValueError, OSError, struct.error, KeyError) as error:
                skipped.append(dict(path=relative, reason=repr(error)))
    names = {row['path'] for row in text}
    raw = [row for row in raw if all(name in names for name in row['evidence_paths'])]
    raw.sort(key=lambda row: (row['identity'][3], row['path']))
    authorization = json.loads((RUN / 'authorization.json').read_text())
    plan = dict(schema=1, retention_profile='minimal_trace_local', root=str(ROOT),
        authorization_text=authorization['text'], upload_enabled=False, target_free_bytes=80 * (1 << 30),
        excluded_prefixes=prefixes, text_files=sorted(text, key=lambda row: row['path']), raw_files=raw,
        excluded_reasons=exclusion_reasons, skipped=skipped, source_tool_sha256=sha_file(WORK / 'tools/minimal_experiment_trace.py'),
        generated_ns=time.time_ns(), closure_basis='Finalized ROS index plus unchanged file identity, no open descriptions via kernel write lease; canonical records additionally require both closure receipts.',
        unknown_result_policy='Historical unknown stays unknown; not relabeled failure. All non-visual ROS messages and first/last visual samples retained before raw removal.')
    atomic(RUN / 'plan.json', plan)
    summary = dict(raw_candidates=len(raw), raw_candidate_bytes=sum(r['identity'][2] for r in raw),
                   raw_candidate_allocated_bytes=sum(r['allocated_bytes'] for r in raw),
                   text_candidates=len(text), text_candidate_bytes=sum(r['identity'][2] for r in text),
                   removable_text_allocated_bytes=sum(r['allocated_bytes'] for r in text if r['remove_original']),
                   migrated_scopes_untouched=len(migrated), excluded_prefixes=len(prefixes),
                   plan=str(RUN / 'plan.json'), plan_sha256=sha_file(RUN / 'plan.json'))
    atomic(RUN / 'plan_summary.json', summary)
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == '__main__':
    main()
