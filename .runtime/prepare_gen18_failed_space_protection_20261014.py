#!/usr/bin/env python3
"""Print a protection-only repair plan; --apply saves it with an exact backup.

Never deletes records, edits eligibility, invokes a cleanup worker, or controls
ROS. The sole canonicalization keeps the exact same protected directory.
"""
import argparse
import copy
import datetime as dt
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess

MOUNT = Path('/media/zhuyupei/SanStand')
ROOT = MOUNT / '实验记录/Robot-Controller'
MAINT = ROOT / 'maintenance'
PROTECTED = MAINT / 'failed_record_space_guard_protected.json'
RELATIVE = 'grasp_records/20261012T112501.434384Z_ec184365e63145858bb80bea45a03b94'
CURRENT = {
    'ros_log/full_latest_gen18_restart_20261014T012216Z':
        'Current GEN18 restart, active recording, validation and failure investigation; preserve all raw evidence.',
    'grasp_records/20261014T015629.706423Z_f0e3415e597e418fa8d9a43fce55ab9f':
        'Current GEN18 recorded grasp; outcome not classified for cleanup. Preserve all raw evidence during recording and investigation.',
}
R6 = (
    'grasp_records/20261013T123054.826269Z_1578aafa633140ab9ac8b88d9513de5e',
    'ros_log/full_latest_contact_admission_20261013T011103Z/attempt_6_full_frames',
)


def encoded(value):
    return (json.dumps(value, ensure_ascii=False, indent=2) + '\n').encode()


def digest(data):
    return hashlib.sha256(data).hexdigest()


def plan(before):
    original = json.loads(before)
    assert original['schema'] == 1
    proposed = copy.deepcopy(original)
    changes, additions = [], []
    for entry in proposed['protected_paths']:
        if entry['path'] == str(ROOT / RELATIVE):
            changes.append({'before': entry['path'], 'after': RELATIVE})
            entry['path'] = RELATIVE
        path = Path(entry['path'])
        assert not path.is_absolute() and '..' not in path.parts, 'unexpected noncanonical protection'
    known = {entry['path'] for entry in proposed['protected_paths']}
    assert all(path in known for path in R6), 'existing R6 protection missing'
    for path, reason in CURRENT.items():
        assert (ROOT / path).is_dir(), 'current protected path missing'
        if path not in known:
            proposed['protected_paths'].append({'path': path, 'reason': reason})
            additions.append(path)
    # Every original entry, including its reason and any metadata, survives.
    assert len(proposed['protected_paths']) == len(original['protected_paths']) + len(additions)
    return proposed, {'schema': 1, 'scope': 'protection_only_no_cleanup',
        'before_sha256': digest(before), 'after_sha256': digest(encoded(proposed)),
        'before_count': len(original['protected_paths']),
        'after_count': len(proposed['protected_paths']), 'canonicalizations': changes,
        'added_protections': additions, 'existing_r6_protection_preserved': True,
        'records_deleted': False, 'eligibility_changed': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    before = PROTECTED.read_bytes()
    proposed, report = plan(before)
    if not args.apply:
        print(json.dumps(dict(report, proposed_protection=proposed), ensure_ascii=False, indent=2))
        return
    assert MOUNT.is_mount() and MAINT.stat().st_dev == MOUNT.stat().st_dev
    fields = subprocess.check_output(['findmnt', '-n', '-o', 'UUID,FSTYPE,OPTIONS',
                                     '-T', str(MOUNT)], text=True).split()
    assert len(fields) == 3 and fields[:2] == ['4899-D74E', 'exfat'] and 'rw' in fields[2].split(',')
    with (MAINT / 'failed_record_space_guard_lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        assert PROTECTED.read_bytes() == before, 'protection changed during review'
        stamp = dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        audit = MAINT / ('failed_record_space_guard_protection_repair_' + stamp + '_' + str(os.getpid()))
        audit.mkdir()
        (audit / 'before.json').write_bytes(before)
        (audit / 'plan.json').write_bytes(encoded(report))
        after = encoded(proposed)
        (audit / 'after.json').write_bytes(after)
        temporary = MAINT / ('failed_record_space_guard_protected.' + str(os.getpid()) + '.tmp')
        with temporary.open('xb') as stream:
            stream.write(after)
            stream.flush()
            os.fsync(stream.fileno())
        assert PROTECTED.read_bytes() == before, 'protection changed before replacement'
        temporary.replace(PROTECTED)
        assert PROTECTED.read_bytes() == after
        report.update(applied=True, audit_directory=str(audit))
        (audit / 'result.json').write_bytes(encoded(report))
        print(json.dumps(report, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
