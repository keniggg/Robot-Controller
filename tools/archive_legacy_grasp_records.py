#!/usr/bin/env python3
"""Archive an explicit legacy experiment inventory; never operates ROS/hardware.

Dry run is the default. --apply uploads and verifies a whole record before any
local pruning. The central index is retained for resumable upload and restore.
No filesystem walk ever expands the inventory's source file set.
"""
import argparse
import copy
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import sys

from grasp_archive import ArchiveError, atomic_json, locked, utc_now
from grasp_archive_transport import GithubReleaseStore, plan_file

SOURCE_ROOT = Path('/home/zhuyupei/alicia_wa_full/.ros_log')
EXTERNAL_MOUNT = Path('/media/zhuyupei/SanStand')
REPOSITORY = 'keniggg/Robot-Controller'
SECTIONS = ('legacy_grasp_evidence', 'other_bag_evidence')


def inventory_digest(document):
    scope = {key: document.get(key, []) for key in (*SECTIONS, 'formal_records')}
    return hashlib.sha256(json.dumps(scope, sort_keys=True, separators=(',', ':'), ensure_ascii=False).encode()).hexdigest()
SCRIPT_CONFIG_SUFFIXES = {'.py', '.pyc', '.sh', '.bash', '.yaml', '.yml', '.launch', '.rviz', '.cfg', '.ini'}


def identity(st):
    return [st.st_dev, st.st_ino, st.st_size, st.st_mtime_ns, st.st_ctime_ns]


def open_for_write(path):
    """Read accessible same-user /proc FDs, alongside mandatory stable-file checks."""
    expected = str(path)
    for process in Path('/proc').glob('[0-9]*'):
        try:
            if process.stat().st_uid != os.getuid():
                continue
            fds = list((process / 'fd').iterdir())
        except FileNotFoundError:
            continue
        except PermissionError:
            # Linux hides descriptors of unrelated nondumpable desktop daemons.
            continue
        for fd in fds:
            try:
                target = os.readlink(str(fd))
                if target not in (expected, expected + ' (deleted)'):
                    continue
                text = (process / 'fdinfo' / fd.name).read_text()
                flags = int(next(line.split()[1] for line in text.splitlines() if line.startswith('flags:')), 8)
                if flags & os.O_ACCMODE:
                    return True
            except (FileNotFoundError, ProcessLookupError):
                continue
            except PermissionError:
                continue
    return False


def require_external_storage(path):
    path = Path(path)
    if (path == EXTERNAL_MOUNT or EXTERNAL_MOUNT in path.parents) and not EXTERNAL_MOUNT.is_mount():
        raise ArchiveError('EXPERIMENT_STORAGE_UNAVAILABLE: SanStand is not mounted')


def safe_path(raw, root=SOURCE_ROOT, regular=True, allow_missing=False):
    path, root = Path(raw), Path(root)
    require_external_storage(path)
    if not path.is_absolute() or '..' in path.parts or path == root or root not in path.parents:
        raise ArchiveError('source is outside the fixed experiment root')
    if any(p.is_symlink() for p in (path, *path.parents)):
        raise ArchiveError('symlink source or ancestor')
    if regular:
        name = path.name
        if (path.suffix.lower() in SCRIPT_CONFIG_SUFFIXES or
                name.endswith(('.partial', '.active')) or
                name in {'archive_config.json', 'processes.json', '.queue.lock'}):
            raise ArchiveError('script, configuration or incomplete source excluded')
        if not path.exists() and allow_missing:
            return path
        st = path.stat()
        if not stat.S_ISREG(st.st_mode) or st.st_nlink != 1:
            raise ArchiveError('source must be a private regular file')
        if open_for_write(path):
            raise ArchiveError('source is open for writing')
    elif not path.is_dir():
        raise ArchiveError('record directory is missing')
    return path


def mapped_path(raw, logical_root, data_root):
    path, logical_root = Path(raw), Path(logical_root)
    if '..' in path.parts or logical_root not in path.parents:
        raise ArchiveError('logical source is outside the fixed experiment root')
    return Path(data_root) / path.relative_to(logical_root)


def audit_mappings(paths, root=SOURCE_ROOT, data_root=None):
    """Accept only completed lossless audits, never compression proposals."""
    root = Path(root)
    data_root = Path(data_root or root)
    result = {}
    for source in paths:
        document = json.loads(Path(source).read_text())
        for item in document.get('files', []):
            if document.get('lossless') is True and item.get('decompressed_sha256_verified') is True and item.get('original_unlinked') is True:
                original = root.parent / item['original_path']
                compressed = root.parent / item['compressed_path']
                digest, size = item['original_sha256'], item['original_size']
                compressed_size = item['compressed_size']
            elif item.get('status') == 'compressed_verified_original_removed' and item.get('sha256') == item.get('decompressed_sha256'):
                original = root / item['relative_path']
                compressed = root / item['gzip_relative_path']
                digest, size = item['sha256'], item['original_bytes']
                compressed_size = item['gzip_bytes']
                if item.get('decompressed_bytes') != size:
                    raise ArchiveError('compression audit size differs from verified original')
            else:
                continue
            if str(compressed) != str(original) + '.gz' or not re.fullmatch('[0-9a-f]{64}', digest):
                raise ArchiveError('invalid exact compression mapping')
            safe_path(mapped_path(original, root, data_root), data_root, allow_missing=True)
            safe_path(mapped_path(compressed, root, data_root), data_root)
            entry = {'original_path': str(original), 'compressed_path': str(compressed),
                     'original_sha256': digest, 'original_size_bytes': size,
                     'compressed_size_bytes': compressed_size,
                     'audit_path': str(Path(source).absolute()), 'decompressed_sha256_verified': True}
            if str(original) in result and result[str(original)] != entry:
                raise ArchiveError('conflicting compression audits')
            result[str(original)] = entry
    return result


def migration_evidence(index, logical_source, source, inventory_item, before):
    """Bind timestamp precision changes to one previously hash-verified copy."""
    if not index or index.get('schema') != 1:
        raise ArchiveError('inventory source changed; verified migration evidence required')
    evidence = index.get('files', {}).get(str(logical_source), {})
    source_identity = evidence.get('source_identity')
    if (evidence.get('state') not in ('verified', 'removed') or
            evidence.get('source_path') != str(logical_source) or
            evidence.get('destination_path') != str(source) or
            not isinstance(source_identity, list) or len(source_identity) != 5 or
            source_identity[2] != inventory_item['size_bytes'] or
            source_identity[3] != inventory_item['mtime_ns'] or
            evidence.get('destination_identity') != identity(before) or
            evidence.get('size_bytes') != inventory_item['size_bytes'] or
            before.st_size != inventory_item['size_bytes'] or
            not re.fullmatch('[0-9a-f]{64}', evidence.get('sha256', ''))):
        raise ArchiveError('migration proof does not bind this exact inventory source and destination')
    return copy.deepcopy(evidence)


def prepare(inventory, compression_audits=(), root=SOURCE_ROOT, data_root=None, migration_index=None):
    root = Path(root)
    data_root = Path(data_root or root)
    mappings = audit_mappings(compression_audits, root, data_root)
    migration = json.loads(Path(migration_index).read_text()) if migration_index else None
    by_compressed = {v['compressed_path']: v for v in mappings.values()}
    inventory = copy.deepcopy(inventory)
    protected = {str(Path(x['directory'])) for x in inventory.get('formal_records', [])}
    seen, records = set(), []
    for section in SECTIONS:
        for group in inventory.get(section, []):
            logical_directory = Path(group['directory'])
            directory = safe_path(mapped_path(logical_directory, root, data_root), data_root, regular=False)
            if any(str(p) in protected for p in (logical_directory, *logical_directory.parents)):
                raise ArchiveError('formal records cannot enter the legacy archive')
            manifest_path = directory / 'manifest.json'
            if manifest_path.is_file() and json.loads(manifest_path.read_text()).get('schema') == 1:
                raise ArchiveError('formal record manifest cannot enter the legacy archive')
            record_id = hashlib.sha256(str(logical_directory.relative_to(root)).encode()).hexdigest()[:24]
            files = []
            for item in group['local_files']:
                logical_source = Path(item['path'])
                source = mapped_path(logical_source, root, data_root)
                compression = by_compressed.get(str(logical_source))
                if not source.exists() and str(logical_source) in mappings:
                    compression = mappings[str(logical_source)]
                    if item['size_bytes'] != compression['original_size_bytes']:
                        raise ArchiveError('inventory differs from verified compression original')
                    source = mapped_path(compression['compressed_path'], root, data_root)
                source = safe_path(source, data_root)
                if directory not in source.parents or str(source) in seen:
                    raise ArchiveError('duplicate file or source outside its record')
                if any(str(p) in protected for p in logical_source.parents):
                    raise ArchiveError('formal record data is protected')
                before = source.stat()
                copied = None
                if compression and source.name.endswith('.gz'):
                    if before.st_size != compression['compressed_size_bytes']:
                        raise ArchiveError('compressed file differs from lossless audit')
                elif (migration is not None and data_root != root) or before.st_size != item['size_bytes'] or before.st_mtime_ns != item['mtime_ns']:
                    copied = migration_evidence(migration, logical_source, source, item, before)
                if compression:
                    digest = hashlib.sha256()
                    size = 0
                    with gzip.open(source, 'rb') as stream:
                        for block in iter(lambda: stream.read(1 << 20), b''):
                            size += len(block)
                            if size > compression['original_size_bytes']:
                                raise ArchiveError('gzip exceeds verified original size')
                            digest.update(block)
                    if size != compression['original_size_bytes'] or digest.hexdigest() != compression['original_sha256']:
                        raise ArchiveError('gzip does not restore the verified original')
                relative = str(source.relative_to(directory))
                plan = plan_file(record_id, source, relative)
                if copied and plan['sha256'] != copied['sha256']:
                    raise ArchiveError('migration destination bytes differ from verified original')
                if identity(source.stat()) != identity(before) or open_for_write(source):
                    raise ArchiveError('source changed or opened during planning')
                files.append({'source_path': str(source), 'original_inventory_path': item['path'],
                              'relative_path': relative, 'size_bytes': plan['size'], 'sha256': plan['sha256'],
                              'identity': identity(before), 'gzip': source.suffix == '.gz',
                              'compression': compression, 'upload': None, 'local_deleted': False})
                if copied:
                    files[-1]['migration'] = {'index_path': str(Path(migration_index).absolute()), **copied}
                seen.add(str(source))
            if files:
                records.append({'record_id': record_id, 'source_directory': str(logical_directory),
                                'physical_source_directory': str(directory),
                                'category': group.get('category', section),
                                'release_tag': 'legacy-grasp-' + record_id,
                                'files': files, 'state': 'pending', 'remote_manifest': None})
    ids = [r['record_id'] for r in records]
    if len(set(ids)) != len(ids):
        raise ArchiveError('duplicate record directories')
    return {'schema': 1, 'repository': REPOSITORY, 'source_root': str(data_root), 'logical_root': str(root),
            'inventory_sha256': inventory_digest(inventory), 'prepared_at': utc_now(), 'records': records}


def assert_unchanged(item, root=SOURCE_ROOT, missing_ok=False):
    path = safe_path(item['source_path'], root, allow_missing=missing_ok)
    if not path.exists():
        return None
    before = path.stat()
    if identity(before) != item['identity']:
        raise ArchiveError('source stat changed since frozen plan')
    plan = plan_file('verification', path, item['relative_path'])
    if plan['sha256'] != item['sha256'] or plan['size'] != item['size_bytes'] or identity(path.stat()) != item['identity'] or open_for_write(path):
        raise ArchiveError('source changed since frozen plan')
    return path


def entry_matches(entry, item):
    return (entry.get('size') == item['size_bytes'] and entry.get('sha256') == item['sha256'] and
            entry.get('path') == item['relative_path'])


def make_store(record):
    owner, repo = REPOSITORY.split('/')
    return GithubReleaseStore(owner, repo, record['release_tag'])


def archive_record(index, record, state_path, store, root=SOURCE_ROOT):
    """Resume one explicit directory; persist each upload and each unlink."""
    state_path = Path(state_path)
    save = lambda: atomic_json(state_path, index)
    deleting = bool(record.get('deletion_started_at'))
    for item in record['files']:
        assert_unchanged(item, root, missing_ok=deleting)
        entry = item.get('upload')
        if entry and entry.get('verified') is True:
            if not entry_matches(entry, item) or not store.verify_file(entry):
                raise ArchiveError('previous remote file failed fresh verification')
            continue
        if deleting:
            raise ArchiveError('incomplete remote checkpoint during deletion recovery')
        def progress(entry, item=item):
            item['upload'] = copy.deepcopy(entry)
            save()
        entry = store.upload_file(record['record_id'], item['source_path'], item['relative_path'], progress=progress)
        if not entry_matches(entry, item) or not store.verify_file(entry):
            raise ArchiveError('uploaded bytes differ from frozen source')
        item['upload'] = entry
        save()
    # Stable remote manifest omits mutable cleanup/verification state.
    manifest = {'schema': 1, 'record_id': record['record_id'], 'repository': REPOSITORY,
                'source_directory': record['source_directory'], 'category': record['category'],
                'release_tag': record['release_tag'], 'files': [
                    {key: copy.deepcopy(item[key]) for key in ('source_path', 'original_inventory_path',
                     'relative_path', 'size_bytes', 'sha256', 'gzip', 'compression', 'upload')}
                    for item in record['files']]}
    for frozen, original in zip(manifest['files'], record['files']):
        if original.get('migration'):
            frozen['migration'] = copy.deepcopy(original['migration'])
    manifest_file = state_path.parent / (record['record_id'] + '.remote_manifest.json')
    atomic_json(manifest_file, manifest)
    manifest_plan = plan_file(record['record_id'], manifest_file, 'archive_manifest.json')
    old = record.get('remote_manifest')
    if old:
        if (old.get('sha256') != manifest_plan['sha256'] or old.get('size') != manifest_plan['size'] or
                old.get('path') != 'archive_manifest.json' or not store.verify_file(old)):
            raise ArchiveError('remote recovery manifest failed fresh verification')
    else:
        record['remote_manifest'] = store.upload_file(record['record_id'], manifest_file, 'archive_manifest.json')
        if (record['remote_manifest'].get('sha256') != manifest_plan['sha256'] or
                record['remote_manifest'].get('size') != manifest_plan['size'] or
                record['remote_manifest'].get('path') != 'archive_manifest.json'):
            raise ArchiveError('remote manifest differs from local recovery manifest')
        save()
    # All remote sources and the recovery manifest must pass afresh before unlink.
    if not store.verify_file(record['remote_manifest']):
        raise ArchiveError('recovery manifest no longer verified')
    for item in record['files']:
        if not store.verify_file(item['upload']):
            raise ArchiveError('remote data no longer verified')
    for item in record['files']:
        assert_unchanged(item, root, missing_ok=deleting)
    record['deletion_started_at'] = record.get('deletion_started_at') or utc_now()
    record['state'] = 'deleting'
    save()
    for item in record['files']:
        path = assert_unchanged(item, root, missing_ok=True)
        if path is not None:
            # Recheck inode and writer immediately before unlink; no recursive deletion.
            if identity(path.stat()) != item['identity'] or open_for_write(path):
                raise ArchiveError('source changed immediately before unlink')
            path.unlink()
            fd = os.open(str(path.parent), os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
        item['local_deleted'] = True
        save()
    record.update(state='deleted', local_deleted_at=utc_now(), error='')
    save()


def run(index, state_path, store_factory=make_store, record_ids=(), max_records=None, root=SOURCE_ROOT, event_sink=None):
    if index.get('repository') != REPOSITORY or index.get('source_root') != str(root):
        raise ArchiveError('index repository/root mismatch')
    if max_records is not None and max_records < 1:
        raise ArchiveError('max-records must be positive')
    selected = set(record_ids)
    if selected - {r['record_id'] for r in index['records']}:
        raise ArchiveError('unknown record ID')
    report = {'deleted': [], 'failed': []}
    attempted = 0
    for record in index['records']:
        if record['state'] == 'deleted' or selected and record['record_id'] not in selected:
            continue
        if max_records is not None and attempted >= max_records:
            break
        attempted += 1
        if event_sink:
            event_sink({'record_id': record['record_id'], 'state': 'starting', 'files': len(record['files'])})
        try:
            archive_record(index, record, state_path, store_factory(record), root)
            report['deleted'].append(record['record_id'])
            if event_sink:
                event_sink({'record_id': record['record_id'], 'state': 'deleted'})
        except Exception as exc:
            record.update(error=str(exc), last_attempt_at=utc_now())
            atomic_json(state_path, index)
            report['failed'].append({'record_id': record['record_id'], 'error': str(exc)})
            if event_sink:
                event_sink({'record_id': record['record_id'], 'state': 'failed', 'error': str(exc)})
    return report


def restore_legacy_manifest(manifest_path, destination, store_factory=make_store):
    """Restore the exact archived files, including audited lossless .gz files."""
    manifest = json.loads(Path(manifest_path).read_text())
    record_id = manifest.get('record_id', '')
    if (manifest.get('schema') != 1 or manifest.get('repository') != REPOSITORY or
            not re.fullmatch('[0-9a-f]{24}', record_id) or
            manifest.get('release_tag') != (
                ('supplemental-grasp-' if manifest.get('category') == 'historical_supplement'
                 else 'legacy-grasp-') + record_id) or
            not isinstance(manifest.get('files'), list) or not manifest['files']):
        raise ArchiveError('invalid legacy recovery manifest')
    destination = Path(destination).expanduser().absolute()
    require_external_storage(destination)
    if '..' in destination.parts or destination.exists() or any(p.is_symlink() for p in (destination, *destination.parents)):
        raise ArchiveError('restore destination must be new and contain no symlinks')
    names = set()
    for item in manifest['files']:
        relative = Path(item['relative_path'])
        if (relative.is_absolute() or not relative.parts or '..' in relative.parts or
                str(relative) in names or str(relative) == 'restored_manifest.json' or
                not entry_matches(item['upload'], item)):
            raise ArchiveError('invalid legacy restore file entry')
        names.add(str(relative))
    store = store_factory(manifest)
    destination.mkdir(parents=True)
    for item in manifest['files']:
        target = destination / item['relative_path']
        target.parent.mkdir(parents=True, exist_ok=True)
        store.restore_file(item['upload'], target)
        restored = plan_file(record_id, target, item['relative_path'])
        if restored['size'] != item['size_bytes'] or restored['sha256'] != item['sha256']:
            raise ArchiveError('restored legacy file differs from original archive bytes')
    atomic_json(destination / 'restored_manifest.json', manifest)
    return {'restored': len(manifest['files']), 'destination': str(destination), 'verified': True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inventory', type=Path)
    parser.add_argument('--state-dir', type=Path)
    parser.add_argument('--migration-index', type=Path, help='verified system_disk_migration_cleanup.json for exact copied-file proof')
    parser.add_argument('--restore-manifest', type=Path)
    parser.add_argument('--restore-directory', type=Path)
    parser.add_argument('--data-root', type=Path, default=SOURCE_ROOT,
                        help='physical ros_log root; inventory/tag identities retain their original logical paths')
    parser.add_argument('--compression-audit', action='append', default=[], type=Path)
    parser.add_argument('--record-id', action='append', default=[])
    parser.add_argument('--max-records', type=int)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    if args.restore_manifest or args.restore_directory:
        if not args.restore_manifest or not args.restore_directory or args.apply or args.inventory or args.state_dir:
            parser.error('restore requires only --restore-manifest and --restore-directory')
        print(json.dumps(restore_legacy_manifest(args.restore_manifest, args.restore_directory), ensure_ascii=False, indent=2))
        return 0
    if args.inventory is None or args.state_dir is None:
        parser.error('--inventory and --state-dir are required for archive operations')
    # Repository and logical root are fixed; only the physical mount is configurable.
    state_dir = args.state_dir.absolute()
    require_external_storage(state_dir)
    if any(p.is_symlink() for p in (state_dir, *state_dir.parents)):
        raise ArchiveError('symlink state directory')
    document = json.loads(args.inventory.read_text())
    groups = [g for key in SECTIONS for g in document.get(key, [])]
    physical_groups = [mapped_path(g['directory'], SOURCE_ROOT, args.data_root) for g in groups]
    if any(p == state_dir or p in state_dir.parents for p in physical_groups):
        raise ArchiveError('central recovery index must be outside every pruned directory')
    if not args.apply:
        index = prepare(document, args.compression_audit, data_root=args.data_root, migration_index=args.migration_index)
        print(json.dumps({'dry_run': True, 'records': len(index['records']),
                          'files': sum(len(r['files']) for r in index['records']),
                          'bytes': sum(f['size_bytes'] for r in index['records'] for f in r['files']),
                          'record_ids': [r['record_id'] for r in index['records']]}, indent=2))
        return 0
    state_dir.mkdir(parents=True, exist_ok=True)
    state_path = state_dir / 'recovery_index.json'
    with locked(state_dir / '.legacy_archive.lock', blocking=False):
        if state_path.exists():
            index = json.loads(state_path.read_text())
            if index.get('inventory_sha256') != inventory_digest(document):
                raise ArchiveError('resume inventory differs from frozen explicit scope')
        else:
            index = prepare(document, args.compression_audit, data_root=args.data_root, migration_index=args.migration_index)
            atomic_json(state_path, index)
        result = run(index, state_path, record_ids=args.record_id, max_records=args.max_records, root=args.data_root,
                     event_sink=lambda event: print(json.dumps(event, ensure_ascii=False), flush=True))
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 1 if result['failed'] else 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ArchiveError, OSError, ValueError) as exc:
        print('Archive stopped: ' + str(exc), file=sys.stderr)
        sys.exit(1)
