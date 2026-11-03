"""Offline supplemental bundle proof; all archives and removals use tmp_path."""
import copy
import hashlib
import io
import json
import os
from pathlib import Path
import sys
import tarfile

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import prepare_supplemental_log_archive as extra
import archive_legacy_grasp_records as legacy


class MemoryStore:
    def __init__(self):
        self.remote = {}; self.corrupt = False
    def upload_file(self, record_id, source, relative, progress=None):
        payload = Path(source).read_bytes(); key = record_id + '/' + relative
        self.remote[key] = payload
        result = {'key': key, 'path': relative, 'size': len(payload),
                  'sha256': hashlib.sha256(payload).hexdigest(), 'verified': True}
        if progress: progress(result)
        return result
    def verify_file(self, entry):
        payload = self.remote.get(entry['key'])
        return (not self.corrupt and payload is not None and len(payload) == entry['size']
                and hashlib.sha256(payload).hexdigest() == entry['sha256'])
    def restore_file(self, entry, destination):
        Path(destination).write_bytes(self.remote[entry['key']])


@pytest.fixture
def sample(tmp_path):
    root = tmp_path / 'ros_log'; directory = root / 'session'; directory.mkdir(parents=True)
    files = [directory / 'data.log', directory / 'depth.npy', root / 'loose.json']
    for index, path in enumerate(files): path.write_bytes(bytes([index]) * (512 + index))
    report = {'files': [{'relative_path': str(p.relative_to(root)), 'size_bytes': p.stat().st_size,
                         'mtime_ns': p.stat().st_mtime_ns} for p in files]}
    inventory = extra.compatible_inventory(report, root)
    return root, files, report, inventory, tmp_path / 'staging'


def make_plan(sample):
    root, _, _, inventory, staging = sample
    return extra.prepare(inventory, staging, root, logical_root=root)


def test_prepare_exact_bytes_independent_tag_and_no_source_mutation(sample):
    root, files, _, inventory, _ = sample
    before = {str(p): p.read_bytes() for p in files}
    index, path = make_plan(sample)
    assert path.is_file()
    record = index['records'][0]
    assert record['release_tag'].startswith('supplemental-grasp-')
    ordinary = hashlib.sha256(b'session').hexdigest()[:24]
    assert record['record_id'] != ordinary
    assert len(record['files']) == 2
    assert [f['relative_path'] for f in record['files']] == ['historical_supplement.tar.gz', 'members.json']
    assert extra.verify_bundle(record['files'][0]['source_path'], index['supplemental_members']['files'])
    assert before == {str(p): p.read_bytes() for p in files}
    assert all(f['original_inventory_path'].startswith(str(root)) for f in index['supplemental_members']['files'])


def test_legacy_upload_only_prunes_staging_not_originals_and_can_restore(sample, tmp_path):
    _, files, _, _, _ = sample
    index, path = make_plan(sample); store = MemoryStore()
    assert legacy.run(index, path, lambda _: store, root=Path(index['source_root']))['deleted']
    assert all(p.exists() for p in files)
    manifest_path = path.parent / (index['records'][0]['record_id'] + '.remote_manifest.json')
    restored = tmp_path / 'restored'
    result = legacy.restore_legacy_manifest(manifest_path, restored, lambda _: store)
    assert result['restored'] == 2
    member_document = json.loads((restored / 'members.json').read_text())
    assert extra.verify_bundle(restored / 'historical_supplement.tar.gz', member_document['files'])


@pytest.mark.parametrize('kind', ['traversal', 'duplicate', 'script', 'active', 'changed', 'formal'])
def test_invalid_scope_and_sources_refused(sample, kind):
    root, files, report, inventory, staging = sample
    if kind == 'traversal':
        report['files'][0]['relative_path'] = '../escape.log'
    elif kind == 'duplicate': report['files'].append(copy.deepcopy(report['files'][0]))
    elif kind == 'script': report['files'][0]['relative_path'] = 'source.py'
    elif kind == 'changed': files[0].write_bytes(b'changed')
    elif kind == 'formal': (files[0].parent / 'manifest.json').write_text('{"schema":1}')
    if kind in ('traversal', 'duplicate', 'script'):
        with pytest.raises(legacy.ArchiveError): extra.compatible_inventory(report, root)
    elif kind == 'active':
        with files[0].open('ab'):
            with pytest.raises(legacy.ArchiveError, match='writing'):
                extra.prepare(inventory, staging, root, logical_root=root)
    else:
        with pytest.raises(legacy.ArchiveError):
            extra.prepare(inventory, staging, root, logical_root=root)
    assert all(p.exists() for p in files)


def test_inventory_hash_and_scope_change_tags(sample):
    root, _, _, inventory, staging = sample
    a, _ = extra.prepare(inventory, staging, root, logical_root=root)
    changed = copy.deepcopy(inventory); changed['archive_scope'] = 'historical-supplement-different'
    b, _ = extra.prepare(changed, staging, root, logical_root=root)
    assert a['records'][0]['release_tag'] != b['records'][0]['release_tag']
    with pytest.raises(legacy.ArchiveError, match='already exists'):
        extra.prepare(inventory, staging, root, logical_root=root)


def test_migration_proof_preserved_and_byte_mismatch_rejected(sample, tmp_path):
    root, files, _, inventory, staging = sample
    external = tmp_path / 'external'; proof = {'schema': 1, 'files': {}}
    for original in files:
        dest = external / original.relative_to(root); dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_bytes(original.read_bytes())
        st = original.stat()
        os.utime(dest, ns=(st.st_atime_ns, st.st_mtime_ns // 1_000_000_000 * 1_000_000_000))
        proof['files'][str(original)] = {'state': 'verified', 'source_path': str(original),
             'destination_path': str(dest), 'source_identity': legacy.identity(st),
             'destination_identity': legacy.identity(dest.stat()), 'size_bytes': st.st_size,
             'sha256': hashlib.sha256(original.read_bytes()).hexdigest()}
    index_path = tmp_path / 'migration.json'; index_path.write_text(json.dumps(proof))
    result, _ = extra.prepare(inventory, staging, external, index_path, logical_root=root)
    assert all(f.get('migration') for f in result['supplemental_members']['files'])
    proof['files'][str(files[0])]['sha256'] = '0' * 64
    index_path.write_text(json.dumps(proof))
    with pytest.raises(legacy.ArchiveError, match='verified migration'):
        extra.prepare(inventory, tmp_path / 'different-staging', external, index_path, logical_root=root)


def test_corrupt_tar_member_rejected(sample, tmp_path):
    index, _ = make_plan(sample)
    member = index['supplemental_members']['files'][0]
    target = tmp_path / 'corrupt.tar.gz'
    with tarfile.open(target, 'w:gz') as archive:
        info = tarfile.TarInfo(member['relative_path']); info.size = member['size_bytes']
        archive.addfile(info, io.BytesIO(b'x' * member['size_bytes']))
    with pytest.raises(legacy.ArchiveError, match='SHA256'):
        extra.verify_bundle(target, index['supplemental_members']['files'])


def test_metadata_excluded_without_expanding_audit(sample):
    root, _, report, _, _ = sample
    report['files'].append({'relative_path': 'session/processes.json', 'size_bytes': 59, 'mtime_ns': 0})
    result = extra.compatible_inventory(report, root)
    assert len(result['local_files']) == 3
    assert result['excluded_metadata'][0]['relative_path'] == 'session/processes.json'


def uploaded(sample):
    index, path = make_plan(sample); store = MemoryStore()
    assert legacy.run(index, path, lambda _: store, root=Path(index['source_root']))['deleted']
    return index, path, store


def test_source_cleanup_only_after_remote_bundle_manifest_and_members_verify(sample):
    root, files, _, _, _ = sample
    keep = root / 'unlisted.log'; keep.write_text('keep')
    index, path, store = uploaded(sample)
    assert all(p.exists() for p in files)
    assert extra.cleanup_sources_verified(index, path, store)['deleted'] == len(files)
    assert all(not p.exists() for p in files)
    assert keep.read_text() == 'keep'


@pytest.mark.parametrize('kind', ['remote_corruption', 'changed_source', 'tampered_index', 'incomplete_upload'])
def test_cleanup_refuses_without_complete_unchanged_evidence(sample, kind):
    _, files, _, _, _ = sample
    index, path, store = uploaded(sample)
    if kind == 'remote_corruption': store.corrupt = True
    elif kind == 'changed_source': files[0].write_bytes(b'changed')
    elif kind == 'tampered_index': index['supplemental_members']['files'][0]['sha256'] = '0' * 64
    else: index['records'][0]['state'] = 'pending'
    with pytest.raises(legacy.ArchiveError): extra.cleanup_sources_verified(index, path, store)
    assert all(p.exists() for p in files)


def test_source_cleanup_crash_recovery_reverifies_remote(sample, monkeypatch):
    _, files, _, _, _ = sample
    index, path, store = uploaded(sample)
    actual = legacy.atomic_json
    def crash(target, document):
        if len(document.get('source_cleanup_deleted', [])) == 1:
            raise KeyboardInterrupt('crash after one source unlink')
        actual(target, document)
    monkeypatch.setattr(legacy, 'atomic_json', crash)
    with pytest.raises(KeyboardInterrupt): extra.cleanup_sources_verified(index, path, store)
    monkeypatch.setattr(legacy, 'atomic_json', actual)
    reloaded = json.loads(path.read_text())
    assert extra.cleanup_sources_verified(reloaded, path, store)['deleted'] == len(files)


def test_full_bundle_restore_preserves_every_original_byte(sample, tmp_path):
    _, files, _, _, _ = sample
    expected = {p.name: p.read_bytes() for p in files}
    index, path = make_plan(sample)
    record = index['records'][0]
    bundle, members = [Path(f['source_path']) for f in record['files']]
    target = tmp_path / 'restored-members'
    assert extra.restore_members(bundle, members, target)['restored'] == 3
    for p in target.rglob('*'):
        if p.is_file(): assert p.read_bytes() == expected[p.name]


def test_compression_proof_only_for_explicit_scope_and_preserves_raw_sha(sample, tmp_path):
    import gzip
    root, files, report, _, staging = sample
    original = files[0]; payload = original.read_bytes(); zipped = original.with_name(original.name + '.gz')
    zipped.write_bytes(gzip.compress(payload)); original.unlink()
    report['files'][0].update(relative_path=str(zipped.relative_to(root)), size_bytes=zipped.stat().st_size,
                               mtime_ns=zipped.stat().st_mtime_ns)
    audit = tmp_path / 'compression.json'
    proof = {'status': 'compressed_verified_original_removed', 'relative_path': str(original.relative_to(root)),
         'gzip_relative_path': str(zipped.relative_to(root)), 'sha256': hashlib.sha256(payload).hexdigest(),
         'decompressed_sha256': hashlib.sha256(payload).hexdigest(), 'original_bytes': len(payload),
         'decompressed_bytes': len(payload), 'gzip_bytes': zipped.stat().st_size}
    unrelated = copy.deepcopy(proof); unrelated.update(relative_path='already_deleted/other.log',
                                                     gzip_relative_path='already_deleted/other.log.gz')
    audit.write_text(json.dumps({'files': [proof, unrelated]}))
    index, _ = extra.prepare(extra.compatible_inventory(report, root), staging, root,
                              compression_audits=[audit], logical_root=root)
    member = next(f for f in index['supplemental_members']['files'] if f['relative_path'].endswith('.gz'))
    assert member['compression']['original_sha256'] == hashlib.sha256(payload).hexdigest()


def test_external_only_audit_timestamp_uses_original_proof_mtime(sample, tmp_path):
    root, files, report, _, staging = sample
    external = tmp_path / 'external'; proof = {'schema': 1, 'files': {}}
    source_mtimes = {}
    for row, original in zip(report['files'], files):
        st = original.stat(); source_mtimes[row['relative_path']] = st.st_mtime_ns
        dest = external / original.relative_to(root); dest.parent.mkdir(parents=True, exist_ok=True)
        payload = original.read_bytes(); dest.write_bytes(payload)
        os.utime(dest, ns=(st.st_atime_ns, st.st_mtime_ns // 1_000_000_000 * 1_000_000_000))
        proof['files'][str(original)] = {'state': 'removed', 'source_path': str(original),
             'destination_path': str(dest), 'source_identity': legacy.identity(st),
             'destination_identity': legacy.identity(dest.stat()), 'size_bytes': st.st_size,
             'sha256': hashlib.sha256(payload).hexdigest()}
        row['mtime_ns'] = dest.stat().st_mtime_ns
        row['locations'] = [{'path': str(dest), 'bytes': dest.stat().st_size,
                             'mtime_ns': dest.stat().st_mtime_ns, 'location': 'SanStand'}]
        original.unlink()
    index_path = tmp_path / 'migration.json'; index_path.write_text(json.dumps(proof))
    inventory = extra.compatible_inventory(report, root)
    result, _ = extra.prepare(inventory, staging, external, index_path, logical_root=root)
    for member in result['supplemental_members']['files']:
        assert member['inventory_mtime_ns'] == source_mtimes[member['relative_path']]
        assert member['identity'] == proof['files'][member['original_inventory_path']]['destination_identity']
        assert member['migration']['source_identity'][3] == source_mtimes[member['relative_path']]


@pytest.mark.parametrize('kind', ['symlink', 'hardlink', 'device', 'traversal', 'undeclared', 'duplicate'])
def test_tar_restore_rejects_unsafe_or_unlisted_members_before_writes(sample, tmp_path, kind):
    index, _ = make_plan(sample)
    document = index['supplemental_members']; first = document['files'][0]
    target = tmp_path / (kind + '.tar.gz')
    info = tarfile.TarInfo(first['relative_path']); payload = b''
    if kind == 'symlink': info.type = tarfile.SYMTYPE; info.linkname = '../outside'
    elif kind == 'hardlink': info.type = tarfile.LNKTYPE; info.linkname = '../outside'
    elif kind == 'device': info.type = tarfile.CHRTYPE
    elif kind == 'traversal': info.name = '../escape.log'
    elif kind == 'undeclared': info.name = 'not-declared.log'
    else: payload = Path(first['source_path']).read_bytes(); info.size = len(payload)
    with tarfile.open(target, 'w:gz') as archive:
        archive.addfile(info, io.BytesIO(payload))
        if kind == 'duplicate': archive.addfile(info, io.BytesIO(payload))
    members = tmp_path / (kind + '.json'); members.write_text(json.dumps(document))
    destination = tmp_path / ('restore-' + kind)
    with pytest.raises(legacy.ArchiveError): extra.restore_members(target, members, destination)
    assert not destination.exists()


def test_default_cli_only_emits_inventory(sample, tmp_path, monkeypatch):
    root, files, report, _, _ = sample
    path = tmp_path / 'audit.json'; path.write_text(json.dumps(report))
    output = tmp_path / 'inventory.json'
    monkeypatch.setattr(legacy, 'make_store', lambda _: (_ for _ in ()).throw(AssertionError('network path called')))
    assert extra.main(['--audit', str(path), '--inventory-output', str(output)]) == 0
    assert output.is_file() and all(p.exists() for p in files)



def test_restore_cli_restores_downloaded_assets_offline(sample, tmp_path, monkeypatch, capsys):
    root, files, _, _, _ = sample
    expected = {str(p.relative_to(root)): p.read_bytes() for p in files}
    index, path, store = uploaded(sample)
    manifest = path.parent / (index['records'][0]['record_id'] + '.remote_manifest.json')
    assets = tmp_path / 'downloaded-assets'
    legacy.restore_legacy_manifest(manifest, assets, lambda _: store)
    monkeypatch.setattr(legacy, 'make_store', lambda _: pytest.fail('offline restore used network'))
    destination = tmp_path / 'original-files'
    assert extra.main(['--restore', '--bundle', str(assets / 'historical_supplement.tar.gz'),
                       '--members', str(assets / 'members.json'),
                       '--restore-directory', str(destination)]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result == {'restored': len(expected), 'verified': True, 'destination': str(destination)}
    assert {str(p.relative_to(destination)): p.read_bytes() for p in destination.rglob('*') if p.is_file()} == expected
    assert all(p.read_bytes() == expected[str(p.relative_to(root))] for p in files)


@pytest.mark.parametrize('conflicting', [
    ['--prepare-only'], ['--upload'], ['--cleanup'], ['--index', 'index.json'],
    ['--audit', 'audit.json'], ['--inventory-output', 'inventory.json'],
    ['--scope', extra.SCOPE], ['--scope=' + extra.SCOPE],
    ['--staging-root', 'stage'], ['--data-root', 'data'],
    ['--migration-index', 'migration.json'], ['--compression-audit', 'compression.json'],
])
def test_restore_cli_rejects_mixed_modes_and_parameters(monkeypatch, conflicting):
    monkeypatch.setattr(extra, 'restore_members', lambda *args: pytest.fail('restore started before argument validation'))
    arguments = ['--restore', '--bundle', 'bundle.tar.gz', '--members', 'members.json',
                 '--restore-directory', 'destination'] + conflicting
    with pytest.raises(SystemExit) as stopped:
        extra.main(arguments)
    assert stopped.value.code == 2


@pytest.mark.parametrize('missing', ['--bundle', '--members', '--restore-directory'])
def test_restore_cli_requires_all_restore_paths(monkeypatch, missing):
    monkeypatch.setattr(extra, 'restore_members', lambda *args: pytest.fail('restore started without required paths'))
    arguments = ['--restore']
    for flag in ['--bundle', '--members', '--restore-directory']:
        if flag != missing: arguments.extend([flag, 'value'])
    with pytest.raises(SystemExit) as stopped:
        extra.main(arguments)
    assert stopped.value.code == 2


@pytest.mark.parametrize('option', ['--bundle', '--members', '--restore-directory'])
def test_restore_cli_paths_require_restore_mode(option):
    with pytest.raises(SystemExit) as stopped:
        extra.main([option, 'value', '--audit', 'audit.json', '--inventory-output', 'inventory.json'])
    assert stopped.value.code == 2



def test_restore_cli_refuses_existing_destination(sample, tmp_path):
    index, _ = make_plan(sample)
    bundle, members = [f['source_path'] for f in index['records'][0]['files']]
    destination = tmp_path / 'existing'; destination.mkdir()
    sentinel = destination / 'keep'; sentinel.write_text('untouched')
    with pytest.raises(legacy.ArchiveError, match='destination must be new'):
        extra.main(['--restore', '--bundle', bundle, '--members', members,
                    '--restore-directory', str(destination)])
    assert list(destination.iterdir()) == [sentinel]
    assert sentinel.read_text() == 'untouched'
