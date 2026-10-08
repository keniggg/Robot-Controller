#!/usr/bin/env python3
"""Remove only byte-verified system-disk duplicates after external migration.

Dry run is the default. Never copies/deletes external data, removes directories,
or contacts network/ROS. Operational scripts/configurations remain in place.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import stat

from grasp_archive import ArchiveError, atomic_json, locked, utc_now
from archive_legacy_grasp_records import identity

PROJECT = Path('/home/zhuyupei/alicia_wa_full')
MOUNT = Path('/media/zhuyupei/SanStand')
BASE = MOUNT / '实验记录/Robot-Controller'
SOURCE_MAP = ((PROJECT / '.ros_log', 'ros_log'),
              (PROJECT / '.worktrees/protocol-v3-upgrade/.grasp_records', 'grasp_records'))
KEEP_RUNTIME_STATUS_PATHS = frozenset(
    SOURCE_MAP[1][0] / name for name in ('disk_status.json', 'worker_status.json'))
BATCH_SIZE = 32
BATCH_BYTES = 128 << 20
KEEP_SUFFIXES = {'.py', '.pyc', '.sh', '.bash', '.launch', '.yaml', '.yml', '.rviz', '.cfg', '.ini'}


def mountpoints():
    def unescape(value):
        return re.sub(r'\\([0-7]{3})', lambda m: chr(int(m.group(1), 8)), value)
    return {Path(unescape(line.split()[4])) for line in Path('/proc/self/mountinfo').read_text().splitlines()}


def require_mount(base, mount=MOUNT):
    base, mount = Path(base), Path(mount)
    if not base.is_absolute() or not mount.is_absolute() or '..' in base.parts or '..' in mount.parts:
        raise ArchiveError('migration paths must be absolute without parent traversal')
    if mount not in mountpoints() or (base != mount and mount not in base.parents):
        raise ArchiveError('required external disk is not mounted at its configured path')
    if not base.is_dir() or any(p.is_symlink() for p in (base, *base.parents)):
        raise ArchiveError('external destination is missing or contains a symlink')


def descriptor_snapshot(paths=(), roots=()):
    """Scan /proc once for both read-open sources and write-open destinations."""
    exact = {str(p) for p in paths}
    prefixes = tuple(str(p).rstrip('/') + '/' for p in roots)
    opened, writers = set(), set()
    for process in Path('/proc').glob('[0-9]*'):
        try:
            if process.stat().st_uid != os.getuid():
                continue
            fds = list((process / 'fd').iterdir())
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        for fd in fds:
            try:
                target = os.readlink(str(fd))
                if target not in exact and not (prefixes and target.startswith(prefixes)):
                    continue
                opened.add(target)
                try:
                    text = (process / 'fdinfo' / fd.name).read_text()
                    flags = int(next(x.split()[1] for x in text.splitlines() if x.startswith('flags:')), 8)
                    if flags & os.O_ACCMODE:
                        writers.add(target)
                except (OSError, ValueError, StopIteration):
                    # If access mode cannot be read, conservatively protect it.
                    writers.add(target)
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                continue
    return {'open': opened, 'writers': writers}


def has_open_descriptor(path, writers_only=False, snapshot=None):
    snapshot = descriptor_snapshot(paths=(path,)) if snapshot is None else snapshot
    return str(path) in snapshot['writers' if writers_only else 'open']


def exclusion(path):
    if path.is_symlink() or any(p.is_symlink() for p in path.parents):
        return 'symlink'
    if path in KEEP_RUNTIME_STATUS_PATHS:
        return 'project_control_source'
    if path.suffix.lower() in KEEP_SUFFIXES or re.fullmatch(r'archive_config.*\.json', path.name):
        return 'project_control_source'
    if path.name.endswith(('.partial', '.active', '.lock')):
        return 'partial_active_or_lock'
    return None


def hash_stable(path):
    before = path.stat()
    if not stat.S_ISREG(before.st_mode) or before.st_nlink != 1 or path.is_symlink():
        raise ArchiveError('not a private regular file')
    digest = hashlib.sha256()
    fd = os.open(str(path), os.O_RDONLY | os.O_NOFOLLOW)
    try:
        if identity(os.fstat(fd)) != identity(before):
            raise ArchiveError('file replaced while opening')
        while True:
            block = os.read(fd, 4 << 20)
            if not block:
                break
            digest.update(block)
        if identity(os.fstat(fd)) != identity(before) or identity(path.stat()) != identity(before):
            raise ArchiveError('file changed while hashing')
    finally:
        os.close(fd)
    return before, digest.hexdigest()


def verify_pair(source, destination, snapshot=None, refresh_open=True):
    reason = exclusion(source)
    if reason:
        return {'state': 'retained', 'reason': reason}
    snapshot = descriptor_snapshot(paths=(source, destination)) if snapshot is None else snapshot
    if has_open_descriptor(source, snapshot=snapshot):
        return {'state': 'retained', 'reason': 'source_open_descriptor'}
    if any(p.is_symlink() for p in (destination, *destination.parents)):
        return {'state': 'mismatch', 'reason': 'destination_symlink'}
    if not destination.is_file():
        return {'state': 'retained', 'reason': 'external_copy_missing'}
    if has_open_descriptor(destination, writers_only=True, snapshot=snapshot):
        return {'state': 'retained', 'reason': 'destination_open_writer'}
    if source.stat().st_size != destination.stat().st_size:
        return {'state': 'mismatch', 'reason': 'size_mismatch'}
    src_stat, src_hash = hash_stable(source)
    dst_stat, dst_hash = hash_stable(destination)
    if src_hash != dst_hash:
        return {'state': 'mismatch', 'reason': 'sha256_mismatch'}
    latest = descriptor_snapshot(paths=(source, destination)) if refresh_open else snapshot
    if identity(source.stat()) != identity(src_stat) or has_open_descriptor(source, snapshot=latest):
        return {'state': 'retained', 'reason': 'source_changed_or_opened'}
    if identity(destination.stat()) != identity(dst_stat) or has_open_descriptor(destination, writers_only=True, snapshot=latest):
        return {'state': 'retained', 'reason': 'destination_changed_or_opened'}
    return {'state': 'verified', 'sha256': src_hash, 'size_bytes': src_stat.st_size,
            'source_identity': identity(src_stat), 'destination_identity': identity(dst_stat)}


def walk_sources(root):
    """Walk only original roots; never follow or remove directories."""
    root = Path(root)
    if root.is_symlink():
        yield root
        return
    if not root.exists():
        return
    if any(p.is_symlink() for p in root.parents):
        yield root
        return
    for directory, dirs, files in os.walk(str(root), followlinks=False):
        for name in list(dirs):
            path = Path(directory) / name
            if path.is_symlink():
                dirs.remove(name)
                yield path
        for name in files:
            yield Path(directory) / name


def process(base=BASE, mappings=SOURCE_MAP, apply=False, mount=MOUNT, batch_size=BATCH_SIZE):
    base = Path(base)
    require_mount(base, mount)
    if type(batch_size) is not int or not 1 <= batch_size <= 1024:
        raise ArchiveError('batch size must be an integer from 1 to 1024')
    roots = []
    for raw_root, destination_name in mappings:
        root = Path(raw_root)
        if not root.is_absolute() or '..' in root.parts or root == Path(mount) or Path(mount) in root.parents:
            raise ArchiveError('migration source must be an absolute system-disk path outside the external mount')
        if Path(destination_name).name != destination_name or destination_name in ('', '.', '..'):
            raise ArchiveError('invalid migration destination mapping')
        roots.append((root, destination_name))
    index_path = base / 'system_disk_migration_cleanup.json'
    previous = json.loads(index_path.read_text()) if index_path.exists() else {'schema': 1, 'files': {}}
    if previous.get('schema') != 1:
        raise ArchiveError('unsupported migration manifest')
    report = {'apply': apply, 'verified': 0, 'removed': 0, 'retained': 0, 'mismatch': 0, 'failed': 0}
    index = previous
    index.update(external_base=str(base), updated_at=utc_now())
    pending, processed, byte_count = [], 0, 0
    snapshot = None

    def save():
        require_mount(base, mount)
        index['updated_at'] = utc_now()
        atomic_json(index_path, index)

    def flush_batch():
        # Same recoverable JSON format, one durable intent for a bounded batch.
        # Every pending file has already passed complete source+destination SHA.
        for source, destination, entry in pending:
            entry['state'] = 'removal_pending'
        if apply:
            save()
        for source, destination, entry in pending:
            try:
                require_mount(base, mount)
                # The batch screening snapshot is never deletion authority.
                # Refresh both descriptor sets in one scan AFTER durable intent.
                current = descriptor_snapshot(paths=(source, destination))
                if (exclusion(source) or any(p.is_symlink() for p in (destination, *destination.parents)) or
                        has_open_descriptor(source, snapshot=current) or
                        has_open_descriptor(destination, writers_only=True, snapshot=current) or
                        identity(source.stat()) != entry['source_identity'] or
                        identity(destination.stat()) != entry['destination_identity']):
                    raise ArchiveError('verified files changed or opened after removal journal')
                source.unlink()
                fd = os.open(str(source.parent), os.O_RDONLY | os.O_DIRECTORY)
                try:
                    os.fsync(fd)
                finally:
                    os.close(fd)
                entry.update(state='removed', removed_at=utc_now())
                report['removed'] += 1
            except (OSError, ArchiveError, ValueError) as exc:
                # If unlink succeeded but the directory fsync failed, retain
                # the pending recovery intent instead of losing its proof.
                entry.update(state='failed' if source.exists() else 'removal_pending', reason=str(exc))
                report['failed'] += 1
        if apply and pending:
            save()
        pending.clear()

    for root, destination_name in roots:
        for source in walk_sources(root):
            destination = base / destination_name / source.relative_to(root)
            entry = {'source_path': str(source), 'destination_path': str(destination), 'checked_at': utc_now()}
            try:
                if snapshot is None:
                    snapshot = descriptor_snapshot(roots=[p for pair in roots for p in (pair[0], base / pair[1])])
                entry.update(verify_pair(source, destination, snapshot=snapshot, refresh_open=not apply))
                if entry['state'] == 'verified' and apply:
                    pending.append((source, destination, entry))
                    byte_count += entry['size_bytes']
                else:
                    report[entry['state']] += 1
            except (OSError, ArchiveError, ValueError) as exc:
                entry.update(state='failed', reason=str(exc))
                report['failed'] += 1
            index['files'][str(source)] = entry
            processed += 1
            if processed >= batch_size or byte_count >= BATCH_BYTES:
                flush_batch()
                snapshot, processed, byte_count = None, 0, 0
    if processed:
        flush_batch()
    # If power was lost just after unlink, confirm the external copy before closing the journal.
    recovered = False
    for source, entry in index['files'].items():
        if entry.get('state') == 'removal_pending' and not Path(source).exists():
            require_mount(base, mount)
            destination = Path(entry['destination_path'])
            if any(p.is_symlink() for p in (destination, *destination.parents)) or base not in destination.parents:
                raise ArchiveError('invalid recovery destination')
            info, digest = hash_stable(destination)
            if info.st_size != entry['size_bytes'] or digest != entry['sha256']:
                raise ArchiveError('external recovery copy no longer matches')
            current = descriptor_snapshot(paths=(destination,))
            if identity(destination.stat()) != identity(info) or has_open_descriptor(destination, writers_only=True, snapshot=current):
                raise ArchiveError('external recovery copy changed or opened')
            if apply:
                entry.update(state='removed', recovered_at=utc_now())
                report['removed'] += 1
                recovered = True
    if apply and (recovered or not roots):
        save()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--external-base', type=Path, default=BASE)
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--batch-size', type=int, default=BATCH_SIZE,
                        help='maximum files per durable intent batch (default: 32; flush after 128 MiB)')
    args = parser.parse_args()
    require_mount(args.external_base)
    if args.apply:
        with locked(args.external_base / '.system_disk_cleanup.lock', blocking=False):
            result = process(args.external_base, apply=True, batch_size=args.batch_size)
    else:
        result = process(args.external_base, batch_size=args.batch_size)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 1 if result['failed'] or result['mismatch'] else 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ArchiveError, OSError, ValueError) as exc:
        print('Migration cleanup stopped: ' + str(exc))
        raise SystemExit(1)
