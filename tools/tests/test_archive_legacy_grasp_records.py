"""Offline proof of explicit legacy scope, remote integrity and crash recovery."""
import copy
import gzip
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import archive_legacy_grasp_records as archive


class MemoryStore:
    def __init__(self):
        self.remote = {}
        self.uploads = []
        self.fail_name = None
        self.verify_hook = None

    def upload_file(self, record_id, source, relative_path, progress=None):
        if relative_path == self.fail_name:
            raise OSError('simulated upload interruption')
        payload = Path(source).read_bytes()
        key = record_id + '/' + relative_path
        self.remote[key] = payload
        entry = {'key': key, 'path': relative_path, 'size': len(payload),
                 'sha256': hashlib.sha256(payload).hexdigest(), 'verified': True}
        self.uploads.append(key)
        if progress:
            progress(entry)
        return entry

    def verify_file(self, entry):
        if self.verify_hook:
            self.verify_hook(entry)
        payload = self.remote.get(entry['key'])
        return (payload is not None and len(payload) == entry['size'] and
                hashlib.sha256(payload).hexdigest() == entry['sha256'])


@pytest.fixture
def setup(tmp_path):
    root = tmp_path / '.ros_log'
    root.mkdir()
    record = root / 'old_attempt'
    record.mkdir()
    (record / 'capture.bag').write_bytes(b'actual closed bag bytes')
    (record / 'operator_observation.json').write_text('{"physical_result":"failure"}')
    (record / 'keep_unlisted.json').write_text('leave intact')
    inventory = {'formal_records': [], 'legacy_grasp_evidence': [
        {'directory': str(record), 'local_files': [
            {'path': str(p), 'size_bytes': p.stat().st_size, 'mtime_ns': p.stat().st_mtime_ns}
            for p in [record / 'capture.bag', record / 'operator_observation.json']]}], 'other_bag_evidence': []}
    state = tmp_path / 'central' / 'recovery_index.json'
    state.parent.mkdir()
    return root, record, inventory, state


def prepared(setup):
    root, record, inventory, state = setup
    index = archive.prepare(inventory, root=root)
    archive.atomic_json(state, index)
    return index


def test_exact_scope_remote_manifest_and_central_restore_index(setup):
    root, record, inventory, state = setup
    index = prepared(setup)
    store = MemoryStore()
    result = archive.run(index, state, lambda _: store, root=root)
    assert result['deleted'] == [index['records'][0]['record_id']]
    assert not result['failed']
    assert (record / 'keep_unlisted.json').read_text() == 'leave intact'
    assert not (record / 'capture.bag').exists()
    manifest = json.loads(store.remote[index['records'][0]['remote_manifest']['key']])
    assert [f['relative_path'] for f in manifest['files']] == ['capture.bag', 'operator_observation.json']
    assert manifest['files'][0]['source_path'] == str(record / 'capture.bag')
    assert state.is_file()
    assert archive.run(json.loads(state.read_text()), state, lambda _: store, root=root) == {'deleted': [], 'failed': []}


def test_dry_plan_makes_no_changes(setup):
    index = prepared(setup)
    assert len(index['records']) == 1
    assert all(Path(f['source_path']).is_file() for f in index['records'][0]['files'])


def test_partial_upload_resumes_and_never_deletes_before_full_record(setup):
    root, record, _, state = setup
    index = prepared(setup)
    store = MemoryStore()
    store.fail_name = 'operator_observation.json'
    assert archive.run(index, state, lambda _: store, root=root)['failed']
    assert (record / 'capture.bag').is_file()
    store.fail_name = None
    loaded = json.loads(state.read_text())
    assert archive.run(loaded, state, lambda _: store, root=root)['deleted']
    assert sum(x.endswith('/capture.bag') for x in store.uploads) == 1


def test_remote_corruption_prevents_any_unlink(setup):
    root, record, _, state = setup
    index = prepared(setup)
    store = MemoryStore()
    def hook(entry):
        if entry['path'] == 'archive_manifest.json':
            for key in list(store.remote):
                if key.endswith('/capture.bag'):
                    store.remote[key] = b'corrupt'
    store.verify_hook = hook
    assert archive.run(index, state, lambda _: store, root=root)['failed']
    assert (record / 'capture.bag').is_file()
    assert (record / 'operator_observation.json').is_file()


def test_source_change_during_remote_verification_blocks_all_deletion(setup):
    root, record, _, state = setup
    index = prepared(setup)
    store = MemoryStore()
    def hook(entry):
        if entry['path'] == 'archive_manifest.json':
            (record / 'capture.bag').write_bytes(b'changed after upload')
    store.verify_hook = hook
    assert archive.run(index, state, lambda _: store, root=root)['failed']
    assert (record / 'operator_observation.json').is_file()
    assert (record / 'capture.bag').read_bytes() == b'changed after upload'


def test_crash_after_unlink_resumes_with_verified_backup(setup, monkeypatch):
    root, record, _, state = setup
    index = prepared(setup)
    store = MemoryStore()
    original = archive.atomic_json
    def crash_after_unlink(path, data):
        if 'records' in data and data['records'][0].get('deletion_started_at') and not (record / 'capture.bag').exists():
            raise KeyboardInterrupt('power loss')
        original(path, data)
    monkeypatch.setattr(archive, 'atomic_json', crash_after_unlink)
    with pytest.raises(KeyboardInterrupt):
        archive.run(index, state, lambda _: store, root=root)
    monkeypatch.setattr(archive, 'atomic_json', original)
    loaded = json.loads(state.read_text())
    assert not (record / 'capture.bag').exists()
    assert archive.run(loaded, state, lambda _: store, root=root)['deleted']


@pytest.mark.parametrize('kind', ['outside', 'symlink', 'script', 'partial', 'formal', 'duplicate', 'changed'])
def test_invalid_or_changed_input_rejected(setup, kind):
    root, record, inventory, _ = setup
    item = inventory['legacy_grasp_evidence'][0]['local_files'][0]
    if kind == 'outside':
        item['path'] = str(root.parent / 'outside.bag')
    elif kind == 'symlink':
        (record / 'capture.bag').unlink()
        (record / 'capture.bag').symlink_to(record / 'operator_observation.json')
    elif kind == 'script':
        target = record / 'source.py'
        target.write_text('print(1)')
        item.update(path=str(target), size_bytes=target.stat().st_size, mtime_ns=target.stat().st_mtime_ns)
    elif kind == 'partial':
        target = record / 'capture.bag.gz.partial'
        target.write_bytes(b'partial')
        item.update(path=str(target), size_bytes=target.stat().st_size, mtime_ns=target.stat().st_mtime_ns)
    elif kind == 'formal':
        (record / 'manifest.json').write_text('{"schema":1}')
    elif kind == 'duplicate':
        inventory['legacy_grasp_evidence'][0]['local_files'].append(copy.deepcopy(item))
    else:
        (record / 'capture.bag').write_bytes(b'replaced')
    with pytest.raises((archive.ArchiveError, FileNotFoundError)):
        archive.prepare(inventory, root=root)


def test_open_writer_rejected_even_with_unchanged_size(setup):
    root, record, inventory, _ = setup
    with (record / 'capture.bag').open('ab'):
        with pytest.raises(archive.ArchiveError, match='open for writing'):
            archive.prepare(inventory, root=root)


def test_exact_verified_gzip_mapping_preserves_original_hash(setup):
    root, record, inventory, state = setup
    source = record / 'capture.bag'
    raw = source.read_bytes()
    zipped = source.with_suffix('.bag.gz')
    with gzip.open(zipped, 'wb') as stream:
        stream.write(raw)
    source.unlink()
    audit = root.parent / 'audit.json'
    audit.write_text(json.dumps({'lossless': True, 'files': [{
        'original_path': str(source.relative_to(root.parent)),
        'compressed_path': str(zipped.relative_to(root.parent)),
        'original_sha256': hashlib.sha256(raw).hexdigest(), 'original_size': len(raw),
        'compressed_size': zipped.stat().st_size, 'decompressed_sha256_verified': True,
        'original_unlinked': True}]}))
    index = archive.prepare(inventory, [audit], root)
    item = index['records'][0]['files'][0]
    assert item['gzip'] is True
    assert item['compression']['original_sha256'] == hashlib.sha256(raw).hexdigest()
    store = MemoryStore()
    assert archive.run(index, state, lambda _: store, root=root)['deleted']
    manifest = json.loads(store.remote[index['records'][0]['remote_manifest']['key']])
    assert manifest['files'][0]['original_inventory_path'] == str(source)
    assert gzip.decompress(store.remote[item['upload']['key']]) == raw


def test_unverified_gzip_substitution_is_not_allowed(setup):
    root, record, inventory, _ = setup
    source = record / 'capture.bag'
    source.with_suffix('.bag.gz').write_bytes(gzip.compress(source.read_bytes()))
    source.unlink()
    with pytest.raises(FileNotFoundError):
        archive.prepare(inventory, root=root)


def test_wrong_gzip_original_hash_is_rejected(setup):
    root, record, inventory, _ = setup
    source = record / 'capture.bag'
    raw = source.read_bytes()
    zipped = source.with_suffix('.bag.gz')
    zipped.write_bytes(gzip.compress(raw))
    source.unlink()
    audit = root.parent / 'audit.json'
    audit.write_text(json.dumps({'lossless': True, 'files': [{
        'original_path': str(source.relative_to(root.parent)), 'compressed_path': str(zipped.relative_to(root.parent)),
        'original_sha256': '0' * 64, 'original_size': len(raw), 'compressed_size': zipped.stat().st_size,
        'decompressed_sha256_verified': True, 'original_unlinked': True}]}))
    with pytest.raises(archive.ArchiveError, match='verified original'):
        archive.prepare(inventory, [audit], root)


def test_external_root_uses_same_stable_tag_and_exact_logical_mapping(setup, tmp_path):
    import shutil
    root, record, inventory, state = setup
    before = archive.prepare(inventory, root=root)
    external = tmp_path / 'SanStand' / '实验记录' / 'ros_log'
    external.parent.mkdir(parents=True)
    shutil.move(str(root), external)
    after = archive.prepare(inventory, root=root, data_root=external)
    assert after['records'][0]['release_tag'] == before['records'][0]['release_tag']
    assert after['records'][0]['source_directory'] == str(record)
    assert all(str(external) in f['source_path'] for f in after['records'][0]['files'])
    store = MemoryStore()
    assert archive.run(after, state, lambda _: store, root=external)['deleted']
    assert (external / 'old_attempt/keep_unlisted.json').is_file()
    assert not (external / 'old_attempt/capture.bag').exists()


def test_writer_opened_after_upload_prevents_unlink(setup):
    root, record, _, state = setup
    index = prepared(setup)
    store = MemoryStore()
    opened = []
    def hook(entry):
        if entry['path'] == 'archive_manifest.json' and not opened:
            opened.append((record / 'capture.bag').open('ab'))
    store.verify_hook = hook
    try:
        assert archive.run(index, state, lambda _: store, root=root)['failed']
        assert (record / 'capture.bag').is_file()
        assert (record / 'operator_observation.json').is_file()
    finally:
        for stream in opened:
            stream.close()


def test_record_limit_and_stable_ids_are_not_completion_timestamps(setup):
    root, record, inventory, state = setup
    other = root / 'second_attempt'
    other.mkdir()
    path = other / 'note.txt'
    path.write_text('separate record')
    inventory['legacy_grasp_evidence'].append({'directory': str(other), 'local_files': [
        {'path': str(path), 'size_bytes': path.stat().st_size, 'mtime_ns': path.stat().st_mtime_ns}]})
    index = archive.prepare(inventory, root=root)
    store = MemoryStore()
    result = archive.run(index, state, lambda _: store, root=root, max_records=1)
    assert len(result['deleted']) == 1
    assert path.is_file()
    result = archive.run(index, state, lambda _: store, root=root, record_ids=[index['records'][1]['record_id']])
    assert len(result['deleted']) == 1
    assert not path.exists()


def copied_with_migration_proof(setup, tmp_path):
    import shutil
    root, record, inventory, _ = setup
    external = tmp_path / 'external' / 'ros_log'
    shutil.copytree(root, external)
    source = record / 'capture.bag'
    destination = external / source.relative_to(root)
    st = destination.stat()
    os.utime(destination, ns=(st.st_atime_ns, (st.st_mtime_ns // 10000000) * 10000000))
    assert destination.stat().st_mtime_ns != source.stat().st_mtime_ns
    proof = {'schema': 1, 'external_base': str(external.parent), 'files': {str(source): {
        'state': 'removed', 'source_path': str(source), 'destination_path': str(destination),
        'source_identity': archive.identity(source.stat()),
        'destination_identity': archive.identity(destination.stat()),
        'size_bytes': source.stat().st_size,
        'sha256': hashlib.sha256(source.read_bytes()).hexdigest()}}}
    for item in inventory['legacy_grasp_evidence'][0]['local_files'][1:]:
        original = Path(item['path'])
        copied = external / original.relative_to(root)
        proof['files'][str(original)] = {
            'state': 'removed', 'source_path': str(original), 'destination_path': str(copied),
            'source_identity': archive.identity(original.stat()),
            'destination_identity': archive.identity(copied.stat()),
            'size_bytes': original.stat().st_size,
            'sha256': hashlib.sha256(original.read_bytes()).hexdigest()}
    proof_path = tmp_path / 'migration.json'
    proof_path.write_text(json.dumps(proof))
    return external, proof_path, proof, source, destination


def test_exfat_precision_change_requires_exact_migration_identity_and_hash(setup, tmp_path):
    root, record, inventory, state = setup
    external, proof_path, proof, source, destination = copied_with_migration_proof(setup, tmp_path)
    original_scope = archive.inventory_digest(inventory)
    with pytest.raises(archive.ArchiveError, match='verified migration evidence required'):
        archive.prepare(inventory, root=root, data_root=external)
    index = archive.prepare(inventory, root=root, data_root=external, migration_index=proof_path)
    assert index['inventory_sha256'] == original_scope
    assert index['records'][0]['files'][0]['migration']['source_identity'][3] == source.stat().st_mtime_ns
    store = MemoryStore()
    assert archive.run(index, state, lambda _: store, root=external)['deleted']
    remote = json.loads(store.remote[index['records'][0]['remote_manifest']['key']])
    assert remote['files'][0]['migration']['sha256'] == proof['files'][str(source)]['sha256']
    assert source.is_file()
    assert not destination.exists()


@pytest.mark.parametrize('kind', ['unverified', 'pending', 'source_mtime', 'destination_identity', 'destination_path', 'wrong_hash'])
def test_migration_evidence_cannot_relax_arbitrary_changed_files(setup, tmp_path, kind):
    root, record, inventory, _ = setup
    external, proof_path, proof, source, destination = copied_with_migration_proof(setup, tmp_path)
    evidence = proof['files'][str(source)]
    if kind == 'unverified': evidence['state'] = 'retained'
    elif kind == 'pending': evidence['state'] = 'removal_pending'
    elif kind == 'source_mtime': evidence['source_identity'][3] += 1
    elif kind == 'destination_identity': evidence['destination_identity'][4] += 1
    elif kind == 'destination_path': evidence['destination_path'] += '.other'
    else: evidence['sha256'] = '0' * 64
    proof_path.write_text(json.dumps(proof))
    with pytest.raises(archive.ArchiveError, match='migration'):
        archive.prepare(inventory, root=root, data_root=external, migration_index=proof_path)
    assert source.is_file() and destination.is_file()


def test_nested_legacy_groups_with_disjoint_files_are_independent(setup):
    root, record, inventory, _ = setup
    nested = record / 'nested'
    nested.mkdir()
    payload = nested / 'different.log'
    payload.write_text('distinct child evidence')
    inventory['other_bag_evidence'].append({'directory': str(nested), 'local_files': [
        {'path': str(payload), 'size_bytes': payload.stat().st_size, 'mtime_ns': payload.stat().st_mtime_ns}]})
    index = archive.prepare(inventory, root=root)
    assert len(index['records']) == 2
    assert sum(len(r['files']) for r in index['records']) == 3
    inventory['legacy_grasp_evidence'][0]['local_files'].append(
        copy.deepcopy(inventory['other_bag_evidence'][0]['local_files'][0]))
    with pytest.raises(archive.ArchiveError, match='duplicate file'):
        archive.prepare(inventory, root=root)
    assert payload.is_file()


def test_formal_supplementary_file_in_legacy_parent_is_protected(setup):
    root, record, inventory, _ = setup
    formal = record / 'formal'
    formal.mkdir()
    observation = formal / 'operator_observation.json'
    observation.write_text('{"outcome":"failure"}')
    inventory['formal_records'] = [{'directory': str(formal)}]
    inventory['legacy_grasp_evidence'][0]['local_files'].append({
        'path': str(observation), 'size_bytes': observation.stat().st_size,
        'mtime_ns': observation.stat().st_mtime_ns})
    with pytest.raises(archive.ArchiveError, match='formal record data is protected'):
        archive.prepare(inventory, root=root)
    assert observation.is_file()


def test_legacy_manifest_restores_exact_payload_without_local_originals(setup, tmp_path):
    root, record, _, state = setup
    index = prepared(setup)
    store = MemoryStore()
    store.restore_file = lambda entry, target: Path(target).write_bytes(store.remote[entry['key']])
    assert archive.run(index, state, lambda _: store, root=root)['deleted']
    manifest_path = state.parent / (index['records'][0]['record_id'] + '.remote_manifest.json')
    restored = tmp_path / 'restored'
    result = archive.restore_legacy_manifest(manifest_path, restored, lambda _: store)
    assert result['verified'] is True and result['restored'] == 2
    assert (restored / 'capture.bag').read_bytes() == b'actual closed bag bytes'
    assert json.loads((restored / 'operator_observation.json').read_text())['physical_result'] == 'failure'
    assert (restored / 'restored_manifest.json').is_file()


def test_legacy_restore_rejects_escaping_file_before_creating_directory(setup, tmp_path):
    root, record, _, state = setup
    index = prepared(setup)
    store = MemoryStore()
    assert archive.run(index, state, lambda _: store, root=root)['deleted']
    path = state.parent / (index['records'][0]['record_id'] + '.remote_manifest.json')
    manifest = json.loads(path.read_text())
    manifest['files'][0]['relative_path'] = '../outside.bag'
    manifest['files'][0]['upload']['path'] = '../outside.bag'
    path.write_text(json.dumps(manifest))
    restored = tmp_path / 'restored'
    with pytest.raises(archive.ArchiveError, match='invalid legacy restore file entry'):
        archive.restore_legacy_manifest(path, restored, lambda _: store)
    assert not restored.exists()


def test_supplied_migration_proof_binds_bytes_even_when_timestamp_matches(setup, tmp_path):
    root, record, inventory, _ = setup
    external, proof_path, proof, source, destination = copied_with_migration_proof(setup, tmp_path)
    unchanged_timestamp_source = record / 'operator_observation.json'
    proof['files'][str(unchanged_timestamp_source)]['sha256'] = '0' * 64
    proof_path.write_text(json.dumps(proof))
    with pytest.raises(archive.ArchiveError, match='migration destination bytes differ'):
        archive.prepare(inventory, root=root, data_root=external, migration_index=proof_path)
    assert (external / unchanged_timestamp_source.relative_to(root)).is_file()


def test_legacy_restore_to_unmounted_external_path_does_not_create_fallback(setup, tmp_path, monkeypatch):
    root, record, _, state = setup
    index = prepared(setup)
    store = MemoryStore()
    assert archive.run(index, state, lambda _: store, root=root)['deleted']
    manifest = state.parent / (index['records'][0]['record_id'] + '.remote_manifest.json')
    mount = tmp_path / 'unmounted'
    mount.mkdir()
    monkeypatch.setattr(archive, 'EXTERNAL_MOUNT', mount)
    destination = mount / 'restored'
    with pytest.raises(archive.ArchiveError, match='EXPERIMENT_STORAGE_UNAVAILABLE'):
        archive.restore_legacy_manifest(manifest, destination, lambda _: store)
    assert not destination.exists()
