"""Supplement registration and incomplete evidence preservation without IO services."""
import copy
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import prepare_formal_archive_evidence as prep
import grasp_archive as archive


def fixture_record(tmp_path, record_id='record-1', state='complete'):
    logical = tmp_path / 'logical_ros_log'
    physical = tmp_path / 'external/ros_log'
    directory = physical / 'session' / record_id
    directory.mkdir(parents=True)
    manifest = {'schema': 1, 'record_id': record_id, 'state': state,
                'recorder_closed': state == 'complete', 'result': 'interrupted' if state == 'incomplete' else 'unknown',
                'failure_reason': 'unverified physical hold', 'result_evidence': {'original': 'kept'},
                'started_at': '2026-09-26T12:26:29+00:00', 'completed_at': '2026-09-26T12:30:06+00:00',
                'local_data_deleted': False, 'files': [], 'owner': {'boot_id': 'old'},
                'upload': {'status': 'blocked_unclosed' if state == 'incomplete' else 'verified'},
                'remote_manifest': {'verified': True, 'entry': {'old': 'remote manifest'}}}
    archive.save_manifest(directory, manifest)
    (directory / 'keyframes.bag').write_bytes(b'original bag bytes')
    (directory / 'operator_observation.json').write_text('{"physical_result":"failed"}')
    files = [{'path': str(logical / 'session' / record_id / p.name), 'relative_path': p.name,
              'size_bytes': p.stat().st_size, 'mtime_ns': p.stat().st_mtime_ns, 'in_manifest': False}
             for p in [directory / 'manifest.json', directory / 'result_summary.json',
                       directory / 'keyframes.bag', directory / 'operator_observation.json']]
    row = {'directory': str(logical / 'session' / record_id), 'record_id': record_id, 'local_files': files}
    return logical, physical, directory, {'formal_records': [row]}, manifest


def test_dry_run_then_idempotent_registration_preserves_task_facts(tmp_path):
    logical, physical, directory, inventory, original = fixture_record(tmp_path)
    before = (directory / 'manifest.json').read_bytes()
    result = prep.register_supplements(inventory, physical, logical_root=logical)
    assert result['records'][0]['added'] == ['operator_observation.json']
    assert (directory / 'manifest.json').read_bytes() == before
    prep.register_supplements(inventory, physical, True, logical)
    updated = archive.load_manifest(directory)
    for key in ['state', 'result', 'result_evidence', 'failure_reason', 'started_at', 'completed_at', 'recorder_closed']:
        assert updated[key] == original[key]
    assert updated['upload']['status'] == 'pending'
    assert 'remote_manifest' not in updated
    assert updated['files'][0]['path'] == 'operator_observation.json'
    assert not (directory / 'keyframes.bag').name in [f['path'] for f in updated['files']]
    assert prep.register_supplements(inventory, physical, True, logical)['records'][0]['added'] == []


@pytest.mark.parametrize('kind', ['changed', 'writer', 'already_deleting', 'not_closed', 'wrong_identity'])
def test_invalid_supplement_registration_keeps_manifest(tmp_path, kind):
    logical, physical, directory, inventory, original = fixture_record(tmp_path)
    stream = None
    if kind == 'changed':
        (directory / 'operator_observation.json').write_text('changed bytes')
    elif kind == 'writer':
        stream = (directory / 'operator_observation.json').open('ab')
    elif kind == 'already_deleting':
        original['deletion_started_at'] = 'started'
        archive.save_manifest(directory, original)
    elif kind == 'not_closed':
        original['recorder_closed'] = False
        archive.save_manifest(directory, original)
    else:
        inventory['formal_records'][0]['record_id'] = 'other'
    before = (directory / 'manifest.json').read_bytes()
    try:
        with pytest.raises(archive.ArchiveError):
            prep.register_supplements(inventory, physical, True, logical)
        assert (directory / 'manifest.json').read_bytes() == before
    finally:
        if stream:
            stream.close()


def test_incomplete_original_bytes_and_status_are_not_finalized(tmp_path):
    logical, physical, directory, inventory, original = fixture_record(tmp_path, prep.INCOMPLETE_ID, 'incomplete')
    before = {p.name: p.read_bytes() for p in directory.iterdir() if p.is_file()}
    index = prep.prepare_incomplete(inventory, physical, logical)
    assert index['normal_recorder_closure_asserted'] is False
    record = index['records'][0]
    assert record['release_tag'] == 'incomplete-grasp-' + prep.INCOMPLETE_ID
    assert {f['relative_path'] for f in record['files']} == set(before)
    assert {p.name: p.read_bytes() for p in directory.iterdir() if p.is_file()} == before
    assert archive.load_manifest(directory)['upload']['status'] == 'blocked_unclosed'


@pytest.mark.parametrize('kind', ['closed', 'other_record', 'registered_data', 'changed_bag'])
def test_incomplete_archive_cannot_expand_or_duplicate_formal_data(tmp_path, kind):
    logical, physical, directory, inventory, original = fixture_record(tmp_path, prep.INCOMPLETE_ID, 'incomplete')
    if kind == 'closed':
        original['recorder_closed'] = True
        archive.save_manifest(directory, original)
    elif kind == 'other_record':
        inventory['formal_records'][0]['record_id'] = 'different'
    elif kind == 'registered_data':
        inventory['formal_records'][0]['local_files'][2]['in_manifest'] = True
    else:
        (directory / 'keyframes.bag').write_bytes(b'changed')
    with pytest.raises(archive.ArchiveError):
        prep.prepare_incomplete(inventory, physical, logical)


def test_current_registered_files_also_block_independent_incomplete_archive(tmp_path):
    logical, physical, directory, inventory, original = fixture_record(tmp_path, prep.INCOMPLETE_ID, 'incomplete')
    original['files'] = [{'path': 'keyframes.bag', 'upload': {'status': 'verified'}}]
    archive.save_manifest(directory, original)
    with pytest.raises(archive.ArchiveError):
        prep.prepare_incomplete(inventory, physical, logical)


def migration_fixture(tmp_path, incomplete=False):
    import hashlib
    import os
    logical, physical, directory, inventory, original = fixture_record(
        tmp_path, prep.INCOMPLETE_ID if incomplete else 'record-1', 'incomplete' if incomplete else 'complete')
    entries = {}
    for item in inventory['formal_records'][0]['local_files']:
        path = directory / item['relative_path']
        old = prep.identity(path.stat())
        rounded = item['mtime_ns'] // 10000000 * 10000000
        if rounded == item['mtime_ns']:
            rounded -= 10000000
        os.utime(path, ns=(path.stat().st_atime_ns, rounded))
        entries[item['path']] = {'state': 'removed', 'source_path': item['path'], 'destination_path': str(path),
                                'source_identity': old, 'destination_identity': prep.identity(path.stat()),
                                'size_bytes': path.stat().st_size, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
    index = tmp_path / 'migration.json'
    index.write_text(json.dumps({'schema': 1, 'files': entries}))
    return logical, physical, directory, inventory, index


def test_migrated_supplement_requires_exact_proof_and_complete_hash(tmp_path):
    logical, physical, directory, inventory, proof = migration_fixture(tmp_path)
    with pytest.raises(archive.ArchiveError, match='migration evidence required'):
        prep.register_supplements(inventory, physical, logical_root=logical)
    prep.register_supplements(inventory, physical, True, logical, migration_index=proof)
    registered = archive.load_manifest(directory)['files'][0]
    assert registered['migration']['source_path'].endswith('operator_observation.json')
    assert registered['sha256'] == registered['migration']['sha256']


def test_migrated_incomplete_preserves_scope_hash_and_evidence_binding(tmp_path):
    logical, physical, directory, inventory, proof = migration_fixture(tmp_path, incomplete=True)
    index = prep.prepare_incomplete(inventory, physical, logical, migration_index=proof)
    assert index['inventory_sha256'] == prep.inventory_digest(inventory)
    bag = next(f for f in index['records'][0]['files'] if f['relative_path'] == 'keyframes.bag')
    assert bag['migration']['sha256'] == bag['sha256']
    assert not index['normal_recorder_closure_asserted']


@pytest.mark.parametrize('mutation', ['sha256', 'destination_path', 'source_mtime', 'destination_identity'])
def test_false_migration_proof_never_registers_evidence(tmp_path, mutation):
    logical, physical, directory, inventory, proof = migration_fixture(tmp_path)
    document = json.loads(proof.read_text())
    entry = next(v for k, v in document['files'].items() if k.endswith('operator_observation.json'))
    if mutation == 'sha256':
        entry['sha256'] = '0' * 64
    elif mutation == 'destination_path':
        entry['destination_path'] += '.wrong'
    elif mutation == 'source_mtime':
        entry['source_identity'][3] -= 1
    else:
        entry['destination_identity'][4] -= 1
    proof.write_text(json.dumps(document))
    before = (directory / 'manifest.json').read_bytes()
    with pytest.raises(archive.ArchiveError):
        prep.register_supplements(inventory, physical, True, logical, migration_index=proof)
    assert (directory / 'manifest.json').read_bytes() == before
