#!/usr/bin/env python3
"""Register explicit supplemental evidence or prepare one raw incomplete archive.

No network/upload/delete operations. Default supplementary registration is read
only; --apply uses the existing queue/record locks and manifest API. Preparing an
incomplete index preserves every raw byte and never asserts recorder closure.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path

from grasp_archive import (ArchiveError, atomic_json, data_path, load_manifest,
                           locked, process_alive, save_manifest, utc_now)
from archive_legacy_grasp_records import (SOURCE_ROOT, REPOSITORY, identity,
                                         inventory_digest, mapped_path, migration_evidence, safe_path)
from grasp_archive_transport import plan_file

SUPPLEMENT_NAMES = frozenset({'operator_outcome_20260926.json',
                             'operator_observation.json', 'continuation.log'})
INCOMPLETE_ID = '20260923T133400.039499Z_55b32bea53ea42a2bb0ce3b0bbd00825'
PERSISTENT = frozenset({'manifest.json', 'result_summary.json', 'archive_manifest.json', '.lock'})


def declared_file(row, item, logical_root, data_root, metadata_snapshot=False, migration=None):
    logical_dir, logical = Path(row['directory']), Path(item['path'])
    relative = Path(item['relative_path'])
    if logical != logical_dir / relative or relative.is_absolute() or '..' in relative.parts:
        raise ArchiveError('inventory file escapes its formal record')
    actual = safe_path(mapped_path(logical, logical_root, data_root), data_root)
    before = actual.stat()
    copied = None
    if not metadata_snapshot and ((migration is not None and Path(data_root) != Path(logical_root)) or
                                  before.st_size != item['size_bytes'] or before.st_mtime_ns != item['mtime_ns']):
        copied = migration_evidence(migration, logical, actual, item, before)
    plan = plan_file(row['record_id'], actual, str(relative))
    if copied:
        if plan['sha256'] != copied['sha256']:
            raise ArchiveError('migration destination bytes differ from verified original')
        plan['migration'] = copied
    if identity(actual.stat()) != identity(before):
        raise ArchiveError('evidence changed during registration hash')
    # Check the live writer list again after potentially long hashing.
    safe_path(actual, data_root)
    return actual, before, plan


def register_supplements(inventory, data_root, apply=False, logical_root=SOURCE_ROOT, migration_index=None):
    migration = json.loads(Path(migration_index).read_text()) if migration_index else None
    report = {'apply': apply, 'records': []}
    for row in inventory.get('formal_records', []):
        selected = [f for f in row['local_files'] if
                    not f.get('in_manifest', False) and f['relative_path'] in SUPPLEMENT_NAMES]
        if not selected:
            continue
        if len({f['relative_path'] for f in selected}) != len(selected):
            raise ArchiveError('duplicate supplementary path in inventory')
        directory = mapped_path(row['directory'], logical_root, data_root)
        def update():
            manifest = load_manifest(directory)
            if manifest['record_id'] != row['record_id']:
                raise ArchiveError('record identity differs from inventory')
            if (manifest['state'] != 'complete' or not manifest.get('recorder_closed') or
                    manifest.get('local_data_deleted') or manifest.get('deletion_started_at') or process_alive(manifest)):
                raise ArchiveError('supplements require a closed, inactive, retained record')
            result = copy.deepcopy(manifest)
            indexed = {f['path']: f for f in result['files']}
            added = []
            for item in selected:
                actual, before, plan = declared_file(row, item, logical_root, data_root, migration=migration)
                relative = item['relative_path']
                if data_path(directory, relative) != actual:
                    raise ArchiveError('registered path differs from fixed evidence path')
                if relative in indexed:
                    existing = indexed[relative]
                    if existing['size_bytes'] != plan['size'] or existing['sha256'] != plan['sha256']:
                        raise ArchiveError('existing supplement entry differs from actual evidence')
                    continue
                result['files'].append({'path': relative, 'size_bytes': plan['size'],
                                        'sha256': plan['sha256'], 'upload': {'status': 'pending'}})
                if plan.get('migration'):
                    result['files'][-1]['migration'] = {'index_path': str(Path(migration_index).absolute()), **plan['migration']}
                added.append(relative)
            if added:
                result['upload'] = {'status': 'pending', 'attempts': 0, 'next_retry_at': 0, 'error': ''}
                result.pop('remote_manifest', None)
                if apply:
                    # Original task result, timestamps and closure fields are unchanged.
                    save_manifest(directory, result)
            report['records'].append({'record_id': manifest['record_id'], 'added': added,
                                      'requires_remote_manifest_refresh': bool(added)})
        if apply:
            with locked(directory.parent / '.queue.lock'):
                with locked(directory / '.lock'):
                    update()
        else:
            update()
    return report


def prepare_incomplete(inventory, data_root, logical_root=SOURCE_ROOT, record_id=INCOMPLETE_ID, migration_index=None):
    """Prepare an archive_legacy run index only for explicitly blocked raw evidence."""
    migration = json.loads(Path(migration_index).read_text()) if migration_index else None
    if record_id != INCOMPLETE_ID:
        raise ArchiveError('only the explicitly identified incomplete experiment is authorized here')
    rows = [row for row in inventory.get('formal_records', []) if row['record_id'] == record_id]
    if len(rows) != 1:
        raise ArchiveError('incomplete experiment is missing or ambiguous in inventory')
    row = rows[0]
    directory = mapped_path(row['directory'], logical_root, data_root)
    manifest = load_manifest(directory)
    if (manifest['record_id'] != record_id or manifest.get('state') != 'incomplete' or
            manifest.get('upload', {}).get('status') != 'blocked_unclosed' or
            manifest.get('recorder_closed') or manifest.get('local_data_deleted') or manifest.get('files') or process_alive(manifest)):
        raise ArchiveError('raw archive requires inactive blocked_unclosed evidence, never a closed record')
    manifest_before = (directory / 'manifest.json').read_bytes()
    files = []
    for item in row['local_files']:
        relative = item['relative_path']
        if relative in {'.lock', 'archive_manifest.json'}:
            continue
        if item.get('in_manifest'):
            raise ArchiveError('independent raw archive must not duplicate registered formal data')
        actual, before, plan = declared_file(row, item, logical_root, data_root,
                                             metadata_snapshot=relative in {'manifest.json', 'result_summary.json'}, migration=migration)
        files.append({'source_path': str(actual), 'original_inventory_path': item['path'],
                      'relative_path': relative, 'size_bytes': plan['size'], 'sha256': plan['sha256'],
                      'identity': identity(before), 'gzip': actual.suffix == '.gz', 'compression': None,
                      'upload': None, 'local_deleted': False})
        if plan.get('migration'):
            files[-1]['migration'] = {'index_path': str(Path(migration_index).absolute()), **plan['migration']}
    if not any(f['relative_path'] == 'manifest.json' for f in files) or not any(f['relative_path'].endswith('.bag') for f in files):
        raise ArchiveError('raw archive needs the original state manifest and captured bag bytes')
    if (directory / 'manifest.json').read_bytes() != manifest_before:
        raise ArchiveError('incomplete state changed during preparation')
    digest = hashlib.sha256(str(Path(row['directory']).relative_to(logical_root)).encode()).hexdigest()[:24]
    record = {'record_id': 'incomplete-' + digest, 'source_directory': row['directory'],
              'physical_source_directory': str(directory),
              'category': 'incomplete_experiment_raw_evidence_unconfirmed_recorder_closure',
              'release_tag': 'incomplete-grasp-' + record_id,
              'files': files, 'state': 'pending', 'remote_manifest': None}
    return {'schema': 1, 'repository': REPOSITORY, 'source_root': str(data_root),
            'logical_root': str(logical_root), 'inventory_sha256': inventory_digest(inventory),
            'prepared_at': utc_now(), 'raw_evidence_only': True,
            'normal_recorder_closure_asserted': False, 'records': [record]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['register-supplements', 'prepare-incomplete'])
    parser.add_argument('--inventory', required=True, type=Path)
    parser.add_argument('--data-root', required=True, type=Path)
    parser.add_argument('--migration-index', type=Path, help='exact SHA-verified migration proof; required when timestamps differ')
    parser.add_argument('--output', type=Path, help='new central index for prepare-incomplete; never inside a record')
    parser.add_argument('--apply', action='store_true', help='apply supplementary registration only; never uploads/deletes')
    args = parser.parse_args()
    inventory = json.loads(args.inventory.read_text())
    if args.action == 'register-supplements':
        result = register_supplements(inventory, args.data_root, apply=args.apply, migration_index=args.migration_index)
    else:
        if args.apply:
            raise ArchiveError('prepare-incomplete never modifies original manifests; use --output for a new index')
        index = prepare_incomplete(inventory, args.data_root, migration_index=args.migration_index)
        if args.output:
            output = args.output.absolute()
            record_dir = Path(index['records'][0]['physical_source_directory'])
            if output.exists() or record_dir in output.parents or any(p.is_symlink() for p in (output, *output.parents)):
                raise ArchiveError('output must be a new central file outside the evidence record')
            output.parent.mkdir(parents=True, exist_ok=True)
            atomic_json(output, index)
        result = {'prepared_raw_evidence': True, 'normal_recorder_closure_asserted': False,
                  'files': len(index['records'][0]['files']), 'release_tag': index['records'][0]['release_tag'],
                  'output': str(args.output) if args.output else None}
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    try:
        main()
    except (ArchiveError, OSError, ValueError) as exc:
        print('Formal evidence preparation stopped: ' + str(exc))
        raise SystemExit(1)
