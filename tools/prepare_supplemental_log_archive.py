#!/usr/bin/env python3
"""Prepare one lossless supplemental bundle and a legacy.run-compatible index.

Default/--prepare-only reads experiment data and writes external staging only.
--upload sends the prepared bundle while retaining original experiment files.
--cleanup is a separate explicit phase after complete fresh remote verification.
--restore expands an already downloaded bundle after verifying every member.
The member index binds logical paths, SHA256, source identity and migration proof.
"""
import argparse
import copy
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import sys
import tarfile

from grasp_archive import ArchiveError, atomic_json, utc_now
import archive_legacy_grasp_records as legacy
from grasp_archive_transport import plan_file

SCOPE = 'historical-supplement-20260927'
TAG_PREFIX = 'supplemental-grasp-'
CATEGORY = 'historical_supplement'
BLOCK = 1 << 20
ALLOWED = re.compile(r'.*\.(?:log(?:\.\d+)?|txt|jsonl|json|md|jpg|jpeg|png|npy|npz|pcd|ply|tif|tiff|bmp|webp|mp4|csv)(?:\.gz)?$')
SKIP_NAMES = frozenset({'processes.json', 'archive_config.json', 'manifest.json',
                        'result_summary.json', 'archive_manifest.json'})


def digest_document(document):
    return hashlib.sha256(json.dumps(document, ensure_ascii=False, sort_keys=True,
                                     separators=(',', ':')).encode()).hexdigest()


def compatible_inventory(report, logical_root=legacy.SOURCE_ROOT, scope=SCOPE):
    """Explicit adapter; preserve snapshot metadata, never walk to add files."""
    logical_root = Path(logical_root)
    if not re.fullmatch(r'[a-z][a-z0-9-]{3,63}', scope):
        raise ArchiveError('invalid supplemental scope')
    rows = []; seen = set(); skipped = []
    for item in report['files']:
        relative = Path(item['relative_path'])
        if relative.is_absolute() or '..' in relative.parts or not relative.parts:
            raise ArchiveError('unsafe supplementary relative path')
        name = str(relative)
        if name in seen:
            raise ArchiveError('duplicate supplementary file')
        seen.add(name)
        if relative.name in SKIP_NAMES:
            skipped.append({'relative_path': name, 'reason': 'configuration_or_process_metadata'})
            continue
        if any(p.startswith('.') for p in relative.parts) or not ALLOWED.fullmatch(name):
            raise ArchiveError('unexpected supplementary file type: ' + name)
        if type(item['size_bytes']) is not int or item['size_bytes'] < 0:
            raise ArchiveError('invalid supplementary size')
        if type(item['mtime_ns']) is not int:
            raise ArchiveError('invalid supplementary timestamp')
        rows.append({'path': str(logical_root / relative), 'relative_path': name,
                     'size_bytes': item['size_bytes'], 'mtime_ns': item['mtime_ns'],
                     'locations': copy.deepcopy(item.get('locations', []))})
    rows.sort(key=lambda x: x['relative_path'])
    if not rows:
        raise ArchiveError('supplemental inventory is empty')
    return {'schema': 1, 'archive_scope': scope, 'logical_root': str(logical_root),
            'category': CATEGORY, 'release_tag_prefix': TAG_PREFIX,
            'source_audit_sha256': digest_document(report), 'local_files': rows,
            'excluded_metadata': skipped}


def verified_source(item, logical_root, data_root, migration):
    logical = Path(item['path'])
    source = legacy.safe_path(legacy.mapped_path(logical, logical_root, data_root), data_root)
    before = source.stat(); copied = None
    # Formal experiment directories must still use their canonical manifests.
    for parent in source.parents:
        if parent == Path(data_root):
            break
        manifest = parent / 'manifest.json'
        if manifest.is_file() and json.loads(manifest.read_text()).get('schema') == 1:
            raise ArchiveError('formal record cannot enter supplemental archive')
    if before.st_size != item['size_bytes']:
        raise ArchiveError('supplementary source size changed')
    if Path(data_root) != Path(logical_root):
        proof = (migration or {}).get('files', {}).get(str(logical), {})
        original = proof.get('source_identity')
        if not isinstance(original, list) or len(original) != 5:
            raise ArchiveError('verified migration proof required for external supplementary source')
        # Audit may have observed the pre-copy inode or the copied external inode.
        snapshots = [(item['size_bytes'], item['mtime_ns'])]
        snapshots.extend((entry.get('bytes'), entry.get('mtime_ns')) for entry in item['locations'])
        if ((before.st_size, before.st_mtime_ns) not in snapshots and
                (original[2], original[3]) not in snapshots):
            raise ArchiveError('audit does not bind migration snapshot')
        frozen_item = {'size_bytes': item['size_bytes'], 'mtime_ns': original[3]}
        copied = legacy.migration_evidence(migration, logical, source, frozen_item, before)
    elif before.st_mtime_ns != item['mtime_ns']:
        raise ArchiveError('supplementary source timestamp changed')
    planned = plan_file('supplemental-member', source, item['relative_path'])
    if copied and planned['sha256'] != copied['sha256']:
        raise ArchiveError('supplementary bytes differ from verified migration')
    if legacy.identity(source.stat()) != legacy.identity(before):
        raise ArchiveError('supplementary source changed while hashing')
    legacy.safe_path(source, data_root)
    result = {'original_inventory_path': str(logical), 'source_path': str(source),
              'relative_path': item['relative_path'], 'size_bytes': before.st_size,
              'sha256': planned['sha256'], 'identity': legacy.identity(before),
              'inventory_mtime_ns': copied['source_identity'][3] if copied else item['mtime_ns'],
              'audit_snapshot_mtime_ns': item['mtime_ns'], 'local_deleted': False}
    if copied:
        result['migration'] = copied
    return result


def declared_members(members):
    expected = {}
    for item in members:
        relative = Path(item['relative_path'])
        if (relative.is_absolute() or '..' in relative.parts or not relative.parts or
                str(relative) != item['relative_path'] or any(p.startswith('.') for p in relative.parts) or
                str(relative) in expected or not re.fullmatch('[0-9a-f]{64}', item['sha256']) or
                type(item['size_bytes']) is not int or item['size_bytes'] < 0):
            raise ArchiveError('invalid or duplicate supplemental member declaration')
        expected[str(relative)] = item
    if not expected:
        raise ArchiveError('empty supplemental member declaration')
    return expected


def verify_bundle(bundle, members):
    expected = declared_members(members); seen = set()
    with tarfile.open(bundle, 'r|gz') as archive:
        for member in archive:
            if not member.isfile() or member.name not in expected or member.name in seen:
                raise ArchiveError('unexpected/duplicate/nonregular bundle member')
            item = expected[member.name]
            if member.size != item['size_bytes']:
                raise ArchiveError('bundle member size differs from source')
            digest = hashlib.sha256(); size = 0
            with archive.extractfile(member) as stream:
                for block in iter(lambda: stream.read(BLOCK), b''):
                    digest.update(block); size += len(block)
            if size != item['size_bytes'] or digest.hexdigest() != item['sha256']:
                raise ArchiveError('bundle member SHA256 differs from frozen source')
            seen.add(member.name)
    if seen != set(expected):
        raise ArchiveError('bundle missing declared members')
    return True


def scoped_compression_proofs(paths, selected_paths, logical_root):
    """Keep verified mappings only for declared gzip members; no unrelated walks."""
    result = {}
    logical_root = Path(logical_root)
    for audit_path in paths:
        document = json.loads(Path(audit_path).read_text())
        for item in document.get('files', []):
            if (document.get('lossless') is True and
                    item.get('decompressed_sha256_verified') is True and item.get('original_unlinked') is True):
                original = logical_root.parent / item['original_path']
                compressed = logical_root.parent / item['compressed_path']
                sha, size = item['original_sha256'], item['original_size']
                compressed_size = item['compressed_size']
            elif (item.get('status') == 'compressed_verified_original_removed' and
                  item.get('sha256') == item.get('decompressed_sha256')):
                original = logical_root / item['relative_path']
                compressed = logical_root / item['gzip_relative_path']
                sha, size = item['sha256'], item['original_bytes']
                compressed_size = item['gzip_bytes']
                if item.get('decompressed_bytes') != size:
                    raise ArchiveError('compression audit size differs from verified original')
            else:
                continue
            if str(compressed) not in selected_paths:
                continue
            if (str(compressed) != str(original) + '.gz' or logical_root not in original.parents or
                    '..' in original.parts or not re.fullmatch('[0-9a-f]{64}', sha)):
                raise ArchiveError('invalid supplementary compression mapping')
            proof = {'original_path': str(original), 'compressed_path': str(compressed),
                     'original_sha256': sha, 'original_size_bytes': size,
                     'compressed_size_bytes': compressed_size, 'audit_path': str(Path(audit_path).absolute()),
                     'decompressed_sha256_verified': True}
            if str(compressed) in result and result[str(compressed)] != proof:
                raise ArchiveError('conflicting supplementary compression proofs')
            result[str(compressed)] = proof
    return result


def verify_compressed_member(member, proof):
    if member['size_bytes'] != proof['compressed_size_bytes']:
        raise ArchiveError('compressed member differs from compression proof')
    digest = hashlib.sha256(); size = 0
    with gzip.open(member['source_path'], 'rb') as stream:
        for block in iter(lambda: stream.read(BLOCK), b''):
            size += len(block)
            if size > proof['original_size_bytes']:
                raise ArchiveError('compressed member expands beyond verified original')
            digest.update(block)
    if size != proof['original_size_bytes'] or digest.hexdigest() != proof['original_sha256']:
        raise ArchiveError('compressed member does not restore the audited original')


def prepare(inventory, staging_root, data_root, migration_index=None,
            compression_audits=(), logical_root=legacy.SOURCE_ROOT):
    """Only produce bundle, member recovery index and a compatible upload plan."""
    staging_root, data_root, logical_root = map(Path, (staging_root, data_root, logical_root))
    if inventory.get('logical_root') != str(logical_root) or inventory.get('category') != CATEGORY:
        raise ArchiveError('supplemental inventory root/category mismatch')
    scope = inventory['archive_scope']
    if not re.fullmatch(r'[a-z][a-z0-9-]{3,63}', scope):
        raise ArchiveError('invalid supplemental scope')
    inventory_sha = digest_document(inventory)
    record_id = hashlib.sha256((scope + '\0' + inventory_sha).encode()).hexdigest()[:24]
    release_tag = TAG_PREFIX + record_id
    legacy.require_external_storage(staging_root)
    if not staging_root.is_absolute() or '..' in staging_root.parts or any(p.is_symlink() for p in (staging_root, *staging_root.parents)):
        raise ArchiveError('unsafe staging path')
    if staging_root == data_root or data_root in staging_root.parents:
        raise ArchiveError('staging must be outside source experiment tree')
    work = staging_root / (scope + '-' + record_id)
    if work.exists():
        raise ArchiveError('staging record already exists; use its frozen recovery_index.json')
    migration = json.loads(Path(migration_index).read_text()) if migration_index else None
    by_compressed = scoped_compression_proofs(
        compression_audits, {item['path'] for item in inventory['local_files']}, logical_root)
    members = []
    seen = set()
    for item in inventory['local_files']:
        relative = Path(item['relative_path'])
        if (relative.is_absolute() or '..' in relative.parts or not relative.parts or
                str(logical_root / relative) != item['path'] or
                str(relative) in seen or not ALLOWED.fullmatch(str(relative)) or relative.name in SKIP_NAMES):
            raise ArchiveError('invalid or duplicate declared supplementary member')
        member = verified_source(item, logical_root, data_root, migration)
        if item['path'] in by_compressed:
            member['compression'] = copy.deepcopy(by_compressed[item['path']])
            verify_compressed_member(member, member['compression'])
        members.append(member); seen.add(str(relative))
    if not members:
        raise ArchiveError('supplemental inventory is empty')
    members.sort(key=lambda x: x['relative_path'])
    data = work / 'data'; data.mkdir(parents=True, exist_ok=False)
    bundle = data / 'historical_supplement.tar.gz'
    temporary = data / 'historical_supplement.tar.gz.partial'
    with temporary.open('xb') as raw:
        with gzip.GzipFile(filename='', mode='wb', fileobj=raw, compresslevel=1, mtime=0) as zipped:
            with tarfile.open(fileobj=zipped, mode='w|', format=tarfile.PAX_FORMAT) as tar:
                for item in members:
                    source = legacy.assert_unchanged(item, data_root)
                    header = tarfile.TarInfo(item['relative_path'])
                    header.size = item['size_bytes']; header.mode = 0o600
                    header.mtime = item['inventory_mtime_ns'] // 1_000_000_000
                    with source.open('rb') as stream:
                        tar.addfile(header, stream)
                    legacy.assert_unchanged(item, data_root)
        raw.flush(); os.fsync(raw.fileno())
    verify_bundle(temporary, members)
    os.rename(temporary, bundle)
    member_document = {'schema': 1, 'archive_scope': scope, 'record_id': record_id,
                       'release_tag': release_tag, 'logical_root': str(logical_root),
                       'physical_root': str(data_root), 'inventory_sha256': inventory_sha,
                       'migration_index_path': str(migration_index) if migration_index else None,
                       'bundle_name': bundle.name, 'files': members}
    member_path = data / 'members.json'; atomic_json(member_path, member_document)
    files = []
    for source in (bundle, member_path):
        planned = plan_file(record_id, source, source.name)
        files.append({'source_path': str(source), 'original_inventory_path': str(logical_root / scope / source.name),
                      'relative_path': source.name, 'size_bytes': planned['size'], 'sha256': planned['sha256'],
                      'identity': legacy.identity(source.stat()), 'gzip': source.suffix == '.gz',
                      'compression': None, 'upload': None, 'local_deleted': False})
    record = {'record_id': record_id, 'source_directory': str(logical_root / scope),
              'physical_source_directory': str(data), 'category': CATEGORY,
              'release_tag': release_tag, 'files': files, 'state': 'pending', 'remote_manifest': None}
    index = {'schema': 1, 'repository': legacy.REPOSITORY, 'source_root': str(work),
             'logical_root': str(logical_root), 'archive_scope': scope,
             'inventory_sha256': inventory_sha, 'prepared_at': utc_now(),
             'records': [record], 'supplemental_members': member_document,
             'source_cleanup_state': 'not_started'}
    atomic_json(work / 'explicit_inventory.json', inventory)
    atomic_json(work / 'recovery_index.json', index)
    return index, work / 'recovery_index.json'


def cleanup_sources_verified(index, state_path, store):
    """Explicit second phase; never invoked by preparation or the CLI.

    Reverify the uploaded bundle, exact member index and recovery manifest before
    validating every original. Persist each unlink for crash recovery.
    """
    records = index.get('records', [])
    if (index.get('archive_scope') is None or len(records) != 1 or
            records[0].get('category') != CATEGORY):
        raise ArchiveError('not a supplemental bundle index')
    record = records[0]; members = index['supplemental_members']
    if (record['release_tag'] != TAG_PREFIX + record['record_id'] or
            members.get('record_id') != record['record_id'] or
            members.get('release_tag') != record['release_tag'] or
            record.get('state') != 'deleted'):
        raise ArchiveError('complete verified bundle upload is required before source cleanup')
    by_name = {item['relative_path']: item for item in record['files']}
    if set(by_name) != {'historical_supplement.tar.gz', 'members.json'}:
        raise ArchiveError('unexpected supplemental upload assets')
    raw_members = (json.dumps(members, ensure_ascii=False, indent=2, allow_nan=False) + '\n').encode()
    if (hashlib.sha256(raw_members).hexdigest() != by_name['members.json']['sha256'] or
            len(raw_members) != by_name['members.json']['size_bytes']):
        raise ArchiveError('source cleanup member index differs from uploaded recovery bytes')
    for item in record['files']:
        if not item.get('upload') or not legacy.entry_matches(item['upload'], item) or not store.verify_file(item['upload']):
            raise ArchiveError('supplemental remote data failed fresh verification')
    manifest_path = Path(state_path).parent / (record['record_id'] + '.remote_manifest.json')
    manifest = json.loads(manifest_path.read_text())
    expected_files = [{key: copy.deepcopy(item[key]) for key in (
        'source_path', 'original_inventory_path', 'relative_path', 'size_bytes',
        'sha256', 'gzip', 'compression', 'upload')} for item in record['files']]
    if (manifest.get('files') != expected_files or manifest.get('record_id') != record['record_id'] or
            manifest.get('release_tag') != record['release_tag'] or manifest.get('category') != CATEGORY):
        raise ArchiveError('remote recovery manifest does not bind this bundle')
    planned = plan_file(record['record_id'], manifest_path, 'archive_manifest.json')
    remote = record.get('remote_manifest') or {}
    if (remote.get('sha256') != planned['sha256'] or remote.get('size') != planned['size'] or
            remote.get('path') != 'archive_manifest.json' or not store.verify_file(remote)):
        raise ArchiveError('supplemental remote recovery manifest failed fresh verification')
    data_root = Path(members['physical_root']); logical_root = Path(members['logical_root'])
    resuming = index.get('source_cleanup_state') == 'deleting'
    seen = set()
    for item in members['files']:
        relative = Path(item['relative_path'])
        if (relative.is_absolute() or '..' in relative.parts or str(relative) in seen or
                Path(item['source_path']) != data_root / relative or
                Path(item['original_inventory_path']) != logical_root / relative):
            raise ArchiveError('supplemental cleanup member escaped original paths')
        legacy.assert_unchanged(item, data_root, missing_ok=resuming or index.get('source_cleanup_state') == 'deleted')
        seen.add(str(relative))
    index['source_cleanup_state'] = 'deleting'
    index.setdefault('source_cleanup_deleted', [])
    legacy.atomic_json(state_path, index)
    for item in members['files']:
        source = legacy.assert_unchanged(item, data_root, missing_ok=True)
        if source is not None:
            if legacy.identity(source.stat()) != item['identity'] or legacy.open_for_write(source):
                raise ArchiveError('source changed immediately before supplemental cleanup')
            source.unlink()
            directory_fd = os.open(source.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        if item['relative_path'] not in index['source_cleanup_deleted']:
            index['source_cleanup_deleted'].append(item['relative_path'])
        legacy.atomic_json(state_path, index)
    index['source_cleanup_state'] = 'deleted'; index['source_cleanup_completed_at'] = utc_now()
    legacy.atomic_json(state_path, index)
    return {'deleted': len(index['source_cleanup_deleted']), 'remote_verified': True}


def restore_members(bundle, member_index, destination):
    """Restore verified bundle members into a new directory without extractall."""
    document = json.loads(Path(member_index).read_text())
    members = document['files']; destination = Path(destination).absolute()
    legacy.require_external_storage(destination)
    if destination.exists() or '..' in destination.parts or any(p.is_symlink() for p in (destination, *destination.parents)):
        raise ArchiveError('restore destination must be new without symlinks')
    expected = declared_members(members)
    verify_bundle(bundle, members)
    destination.mkdir(parents=True)
    seen = set()
    with tarfile.open(bundle, 'r|gz') as archive:
        for member in archive:
            if not member.isfile() or member.name not in expected or member.name in seen:
                raise ArchiveError('unexpected/duplicate/nonregular member during restore')
            if member.size != expected[member.name]['size_bytes']:
                raise ArchiveError('member size changed during restore')
            seen.add(member.name)
            target = destination / member.name
            target.parent.mkdir(parents=True, exist_ok=True)
            with archive.extractfile(member) as source, target.open('xb') as output:
                for block in iter(lambda: source.read(BLOCK), b''):
                    output.write(block)
    if seen != set(expected):
        raise ArchiveError('members missing during restore')
    for item in members:
        target = destination / item['relative_path']
        result = plan_file(document['record_id'], target, item['relative_path'])
        if result['size'] != item['size_bytes'] or result['sha256'] != item['sha256']:
            raise ArchiveError('restored supplemental file differs from archived original')
    return {'restored': len(members), 'verified': True, 'destination': str(destination)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--prepare-only', action='store_true', help='default: only freeze sources and create bundle/index')
    mode.add_argument('--upload', action='store_true', help='upload existing prepared bundle; original experiments remain')
    mode.add_argument('--cleanup', action='store_true', help='explicitly remove original members after fresh remote verification')
    mode.add_argument('--restore', action='store_true', help='restore an already downloaded bundle into a new directory')
    parser.add_argument('--bundle', type=Path, help='verified downloaded historical_supplement.tar.gz for --restore')
    parser.add_argument('--members', type=Path, help='downloaded members.json for --restore')
    parser.add_argument('--restore-directory', type=Path, help='new destination for restored original files')
    parser.add_argument('--index', type=Path, help='existing staged recovery_index.json for --upload/--cleanup')
    parser.add_argument('--audit', type=Path)
    parser.add_argument('--inventory-output', type=Path)
    parser.add_argument('--scope')
    parser.add_argument('--staging-root', type=Path)
    parser.add_argument('--data-root', type=Path)
    parser.add_argument('--migration-index', type=Path)
    parser.add_argument('--compression-audit', action='append', default=[], type=Path)
    args = parser.parse_args(argv)
    if args.restore:
        if (not args.bundle or not args.members or not args.restore_directory or args.index or
                args.audit or args.inventory_output or args.scope is not None or args.staging_root or
                args.data_root or args.migration_index or args.compression_audit):
            parser.error('--restore requires only --bundle, --members and --restore-directory')
        result = restore_members(args.bundle, args.members, args.restore_directory)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    if args.bundle or args.members or args.restore_directory:
        parser.error('--bundle, --members and --restore-directory require --restore')
    if args.scope is None:
        args.scope = SCOPE
    if args.upload or args.cleanup:
        if not args.index or args.audit or args.inventory_output or args.staging_root or args.data_root or args.migration_index or args.compression_audit:
            parser.error('--upload/--cleanup require only --index')
        state_path = args.index.absolute()
        legacy.require_external_storage(state_path)
        if legacy.EXTERNAL_MOUNT not in state_path.parents or any(p.is_symlink() for p in (state_path, *state_path.parents)):
            raise ArchiveError('supplemental execution index must be on SanStand without symlinks')
        with legacy.locked(state_path.parent / '.supplemental_archive.lock', blocking=False):
            index = json.loads(state_path.read_text())
            if (index.get('archive_scope') is None or index.get('source_root') != str(state_path.parent) or
                    len(index.get('records', [])) != 1 or index['records'][0].get('category') != CATEGORY or
                    index['records'][0].get('release_tag') != TAG_PREFIX + index['records'][0]['record_id']):
                raise ArchiveError('not a frozen supplemental execution index')
            if args.upload:
                result = legacy.run(index, state_path, root=state_path.parent,
                    event_sink=lambda event: print(json.dumps(event), flush=True))
                result['original_source_cleanup_performed'] = False
            else:
                result = cleanup_sources_verified(index, state_path, legacy.make_store(index['records'][0]))
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return 1 if result.get('failed') else 0
    if not args.audit or not args.inventory_output or args.index:
        parser.error('preparation requires --audit and --inventory-output; no --index')
    if args.staging_root and not args.data_root:
        parser.error('--data-root is required with --staging-root')
    inventory = compatible_inventory(json.loads(args.audit.read_text()), scope=args.scope)
    if args.inventory_output.exists():
        if json.loads(args.inventory_output.read_text()) != inventory:
            raise ArchiveError('existing inventory output differs from explicit scope')
    else:
        atomic_json(args.inventory_output, inventory)
    summary = {'inventory': str(args.inventory_output), 'files': len(inventory['local_files']),
               'excluded_metadata': len(inventory['excluded_metadata']), 'upload_performed': False,
               'source_cleanup_performed': False}
    if args.staging_root:
        if legacy.EXTERNAL_MOUNT not in args.staging_root.absolute().parents:
            raise ArchiveError('staging must be on SanStand')
        legacy.require_external_storage(args.staging_root.absolute())
        index, path = prepare(inventory, args.staging_root.absolute(), args.data_root.absolute(),
                              args.migration_index, args.compression_audit)
        summary.update(index=str(path), release_tag=index['records'][0]['release_tag'],
                       bundle_bytes=index['records'][0]['files'][0]['size_bytes'])
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ArchiveError, OSError, ValueError) as error:
        print('Supplemental preparation stopped: ' + str(error), file=sys.stderr)
        raise SystemExit(1)
