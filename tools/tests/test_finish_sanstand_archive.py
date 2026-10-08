"""Session orchestration only: temporary files, fake remote services, no hardware."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess

import pytest

SCRIPT = Path('/home/zhuyupei/alicia_wa_full/.ros_log/unknown_grasp_20260923/finish_sanstand_archive_20260927.py')
spec = importlib.util.spec_from_file_location('finish_sanstand_session', SCRIPT)
finish = importlib.util.module_from_spec(spec)
spec.loader.exec_module(finish)


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))


@pytest.fixture
def layout(tmp_path, monkeypatch):
    result = finish.Layout(tmp_path / 'external')
    result.base.mkdir()
    write(result.base / 'maintenance/storage_control.json', {'schema': 1, 'state': 'resumed', 'archive_policy': 'github_verified'})
    monkeypatch.setattr(finish.migration, 'require_mount', lambda *a, **kw: None)
    monkeypatch.setattr(finish, 'finalizer_finished', lambda: None)
    return result


def minimal_scope():
    formal = []
    for i in range(59):
        record_id = finish.evidence.INCOMPLETE_ID if i == 0 else 'formal_%02d' % i
        directory = finish.PROJECT / '.ros_log/unknown_grasp_20260923' / record_id
        names = ['manifest.json', 'result_summary.json', 'keyframes.bag', 'recorder.log',
                 'recorder_outcome.json', 'recorder_ready.json', 'runner.log'] if i == 0 else []
        formal.append({'record_id': record_id, 'directory': str(directory),
                       'local_files': [{'path': str(directory / name), 'relative_path': name,
                                        'size_bytes': 1, 'mtime_ns': 1, 'in_manifest': False} for name in names]})
    groups = []
    for i in range(108):
        directory = finish.legacy.SOURCE_ROOT / ('legacy_%03d' % i)
        groups.append({'directory': str(directory), 'local_files': [
            {'path': str(directory / 'data.log'), 'size_bytes': 1, 'mtime_ns': 1}]})
    return {'formal_records': formal, 'legacy_grasp_evidence': groups, 'other_bag_evidence': [],
            'latest_five_formal_by_actual_completion': formal[-5:]}


@pytest.fixture
def staged(layout, monkeypatch):
    inventory = minimal_scope()
    audit = {'files': [{'relative_path': 'extra/data_%04d.log' % i, 'size_bytes': 1, 'mtime_ns': 1}
                       for i in range(1861)]}
    compatible = finish.extra.compatible_inventory(audit)
    monkeypatch.setattr(finish, 'INVENTORY_SHA', finish.legacy.inventory_digest(inventory))
    monkeypatch.setattr(finish, 'EXTRA_SHA', finish.extra.digest_document(compatible))
    write(layout.ops / 'record_inventory_20260927.json', inventory)
    audit_path = layout.base / 'archive_inventory/extra_archive_inventory_20260927.json'
    write(audit_path, audit)
    for name in ('closed_bag_compression_20260927.json', 'closed_log_compression_20260927.json'):
        write(layout.ops / name, {'files': []})
    events = []
    def prepare_legacy(document, audits, data_root, migration_index):
        events.append('legacy')
        records = []
        for group in document['legacy_grasp_evidence']:
            record_id = hashlib.sha256(str(Path(group['directory']).relative_to(finish.legacy.SOURCE_ROOT)).encode()).hexdigest()[:24]
            records.append({'record_id': record_id, 'release_tag': 'legacy-grasp-' + record_id, 'files': [
                {'original_inventory_path': item['path'], 'source_path': str(data_root / Path(item['path']).relative_to(finish.legacy.SOURCE_ROOT))}
                for item in group['local_files']]})
        return {'schema': 1, 'repository': finish.legacy.REPOSITORY, 'source_root': str(data_root),
                'inventory_sha256': finish.INVENTORY_SHA, 'records': records}
    def prepare_raw(document, data_root, migration_index):
        events.append('raw')
        row = document['formal_records'][0]
        return {'schema': 1, 'repository': finish.legacy.REPOSITORY, 'source_root': str(data_root),
                'inventory_sha256': finish.INVENTORY_SHA, 'raw_evidence_only': True,
                'normal_recorder_closure_asserted': False, 'records': [{
                    'release_tag': 'incomplete-grasp-' + finish.evidence.INCOMPLETE_ID,
                    'files': [{'original_inventory_path': item['path'],
                               'source_path': str(data_root / Path(item['path']).relative_to(finish.legacy.SOURCE_ROOT))}
                              for item in row['local_files']]}]}
    def prepare_extra(document, staging_root, data_root, migration_index, audits):
        events.append('extra')
        record_id = hashlib.sha256((document['archive_scope'] + '\0' + finish.EXTRA_SHA).encode()).hexdigest()[:24]
        work = staging_root / (document['archive_scope'] + '-' + record_id)
        result = {'schema': 1, 'repository': finish.legacy.REPOSITORY, 'source_root': str(work),
                  'inventory_sha256': finish.EXTRA_SHA, 'records': [{}], 'supplemental_members': {
                    'inventory_sha256': finish.EXTRA_SHA, 'files': [
                        {'original_inventory_path': item['path'], 'relative_path': item['relative_path'],
                         'source_path': str(data_root / item['relative_path'])} for item in document['local_files']]}}
        path = work / 'recovery_index.json'; write(path, result)
        return result, path
    monkeypatch.setattr(finish.legacy, 'prepare', prepare_legacy)
    monkeypatch.setattr(finish.evidence, 'prepare_incomplete', prepare_raw)
    monkeypatch.setattr(finish.extra, 'prepare', prepare_extra)
    return layout, inventory, audit_path, events


def test_all_indexes_frozen_before_activation_and_resume_does_not_prepare_deleted_sources(staged, monkeypatch):
    layout, inventory, audit_path, events = staged
    finish.prepare_all(layout, audit_path)
    assert events == ['legacy', 'raw', 'extra']
    assert all(path.is_file() for path in layout.indices.values())
    (layout.ops / 'record_inventory_20260927.json').unlink()
    audit_path.unlink()
    def prohibited(*args, **kwargs):
        pytest.fail('prepared source data must not be read again after deletion')
    for module, name in [(finish.legacy, 'prepare'), (finish.evidence, 'prepare_incomplete'), (finish.extra, 'prepare')]:
        monkeypatch.setattr(module, name, prohibited)
    resumed = finish.Layout(layout.base)
    result, _ = finish.prepare_all(resumed, audit_path)
    assert result == inventory
    assert resumed.indices == layout.indices


def test_missing_migration_evidence_stops_before_activation_or_upload(staged, monkeypatch):
    layout, inventory, audit_path, events = staged
    def missing(*args, **kwargs):
        raise finish.formal.ArchiveError('verified migration proof required')
    monkeypatch.setattr(finish.evidence, 'prepare_incomplete', missing)
    monkeypatch.setattr(finish, 'enable_retention', lambda *a: pytest.fail('must not enable before all indexes exist'))
    monkeypatch.setattr(finish, 'run_archives', lambda *a: pytest.fail('must not upload before all indexes exist'))
    with pytest.raises(finish.formal.ArchiveError, match='migration proof'):
        finish.execute(layout, audit_path, 1)
    assert layout.indices['legacy'].is_file()
    assert not layout.indices['raw'].exists()
    assert finish.read(layout.status)['complete'] is False


def test_changed_index_scope_is_rejected_before_remote_actions(staged):
    layout, inventory, audit_path, events = staged
    finish.prepare_all(layout, audit_path)
    index = finish.read(layout.indices['legacy'])
    index['records'][0]['files'][0]['source_path'] = str(layout.data / 'unlisted.log')
    write(layout.indices['legacy'], index)
    with pytest.raises(finish.formal.ArchiveError, match='physical source changed'):
        finish.prepare_all(layout, audit_path)


def test_changed_compatible_inventory_never_expands_the_frozen_scope(staged):
    layout, inventory, audit_path, events = staged
    audit = finish.read(audit_path)
    audit['files'][0]['relative_path'] = 'unlisted/file.log'
    write(audit_path, audit)
    with pytest.raises(finish.formal.ArchiveError, match='1861-file scope'):
        finish.prepare_all(layout, audit_path)
    assert events == []


def test_finalizer_still_active_stops_before_prepare_or_storage_changes(layout, monkeypatch):
    def active(): raise finish.Pending('waiting for finalizer')
    monkeypatch.setattr(finish, 'finalizer_finished', active)
    monkeypatch.setattr(finish, 'prepare_all', lambda *a: pytest.fail('finalizer still active'))
    with pytest.raises(finish.Pending):
        finish.execute(layout, Path('unused'), 1)
    assert not layout.status.exists()


def test_publishing_pending_preserves_unrelated_migration_facts(layout):
    write(layout.storage_status, {'migration_copy_completed': True, 'source_cleanup_complete': False})
    status = finish.publish(layout, 'upload_legacy', event={'state': 'starting'})
    saved = finish.read(layout.storage_status)
    assert status['complete'] is False and saved['github_upload_complete'] is False
    assert saved['migration_copy_completed'] is True and saved['source_cleanup_complete'] is False


def test_retention_enable_requires_sidecars_and_real_restore(layout, monkeypatch):
    cfg = {'delete_enabled': False, 'repo': finish.legacy.REPOSITORY,
           'deletion_validation': {'external_restore_validation': 'fixture'}}
    inventory = {'latest_five_formal_by_actual_completion': [{'record_id': str(i)} for i in range(5)]}
    layout.work.mkdir(parents=True); layout.ops.mkdir(parents=True)
    events = []
    original_lock = finish.formal.locked
    def record_lock(path, blocking=True):
        if path == layout.ops / '.queue.lock':
            assert blocking is True
            events.append('wait-current-record-boundary')
        return original_lock(path, blocking=blocking)
    monkeypatch.setattr(finish.formal, 'locked', record_lock)
    monkeypatch.setattr(finish, 'validate_retention', lambda *a: copy.deepcopy(cfg))
    monkeypatch.setattr(finish, 'current_retention', lambda *a: ([], {str(i) for i in range(5)}))
    def missing(*a):
        events.append('sidecars')
        raise finish.formal.ArchiveError('missing supplementary evidence')
    monkeypatch.setattr(finish, 'verify_supplements', missing)
    monkeypatch.setattr(finish, 'verify_restore', lambda *a: events.append('restore'))
    with pytest.raises(finish.formal.ArchiveError): finish.enable_retention(layout, inventory)
    assert events == ['sidecars']
    assert not (layout.ops / 'archive_config.json').exists()
    monkeypatch.setattr(finish, 'verify_supplements', lambda *a: events.append('sidecars-ok'))
    finish.enable_retention(layout, inventory)
    assert events[-3:] == ['sidecars-ok', 'restore', 'wait-current-record-boundary']
    assert finish.read(layout.ops / 'archive_config.json')['delete_enabled'] is True
    assert finish.read(layout.work / 'retention_enabled.json')['supplements_verified'] == 4


def test_bounded_legacy_queue_pending_prevents_extra_execution(layout, monkeypatch):
    layout.indices['extra'] = layout.work / 'extra/recovery_index.json'
    for kind, path in layout.indices.items():
        write(path, {'source_root': str(layout.data), 'records': [
            {'state': 'deleted' if kind == 'raw' else 'pending', 'record_id': kind}]})
    calls = []
    def run(index, path, **kwargs):
        calls.append((path, kwargs['max_records']))
        return {'deleted': [], 'failed': []}
    monkeypatch.setattr(finish.legacy, 'run', run)
    monkeypatch.setattr(finish.extra, 'cleanup_sources_verified', lambda *a: pytest.fail('extra must wait'))
    with pytest.raises(finish.Pending, match='legacy archive queue remains'):
        finish.run_archives(layout, 1)
    assert calls == [(layout.indices['legacy'], 1)]


@pytest.fixture
def completed(layout):
    inventory = minimal_scope()
    keep = {row['record_id'] for row in inventory['formal_records'][-5:]}
    for row in inventory['formal_records'][1:]:
        directory = layout.formal_directory(row)
        item = {'path': 'capture.bag', 'size_bytes': 1, 'sha256': hashlib.sha256(b'x').hexdigest(),
                'upload': {'status': 'verified', 'entry': {'key': row['record_id']}}}
        manifest = {'schema': 1, 'record_id': row['record_id'], 'result': 'unknown',
                    'state': 'complete', 'recorder_closed': True, 'completed_at': row['record_id'],
                    'files': [item], 'local_data_deleted': row['record_id'] not in keep,
                    'upload': {'status': 'verified'}, 'remote_manifest': {'verified': True, 'entry': {'key': 'manifest'}}}
        write(directory / 'manifest.json', manifest)
        if row['record_id'] in keep: (directory / 'capture.bag').write_bytes(b'x')
    layout.indices['extra'] = layout.work / 'extra/recovery_index.json'
    for kind, path in layout.indices.items():
        write(path, {'records': [{'state': 'deleted', 'remote_manifest': {'key': 'manifest'},
                                 'files': [{'upload': {'key': 'file'}, 'local_deleted': True}]}],
                     'source_cleanup_state': 'deleted'})
    return layout, inventory


def test_completion_requires_exact_global_five_and_all_formal_remote_status(completed):
    layout, inventory = completed
    assert finish.completion_snapshot(layout, inventory)['complete'] is True
    directory = layout.formal_directory(inventory['formal_records'][1])
    manifest = finish.read(directory / 'manifest.json')
    manifest['upload']['status'] = 'pending'
    write(directory / 'manifest.json', manifest)
    assert finish.completion_snapshot(layout, inventory)['complete'] is False


def test_sixth_formal_payload_blocks_complete_even_when_uploaded(completed):
    layout, inventory = completed
    directory = layout.formal_directory(inventory['formal_records'][1])
    (directory / 'capture.bag').write_bytes(b'x')
    result = finish.completion_snapshot(layout, inventory)
    assert result['complete'] is False and result['local_complete_records'] == 6


def test_extra_original_source_cleanup_must_finish_before_complete(completed):
    layout, inventory = completed
    index = finish.read(layout.indices['extra']); index['source_cleanup_state'] = 'deleting'
    write(layout.indices['extra'], index)
    assert finish.completion_snapshot(layout, inventory)['complete'] is False


def test_remote_verification_failure_never_writes_complete(completed, monkeypatch):
    layout, inventory = completed
    monkeypatch.setattr(finish, 'prepare_all', lambda *a: (inventory, {}))
    monkeypatch.setattr(finish, 'enable_retention', lambda *a: None)
    monkeypatch.setattr(finish, 'run_archives', lambda *a: None)
    def broken(*a): raise finish.Pending('remote checksum mismatch')
    monkeypatch.setattr(finish, 'verify_completed_remote', broken)
    with pytest.raises(finish.Pending, match='checksum'):
        finish.execute(layout, Path('unused'), 1)
    assert finish.read(layout.status)['complete'] is False


def test_complete_is_only_published_after_fresh_remote_verification(completed, monkeypatch):
    layout, inventory = completed
    events = []
    monkeypatch.setattr(finish, 'prepare_all', lambda *a: (inventory, {}))
    monkeypatch.setattr(finish, 'enable_retention', lambda *a: events.append('enable'))
    monkeypatch.setattr(finish, 'run_archives', lambda *a: events.append('queues'))
    monkeypatch.setattr(finish, 'verify_completed_remote', lambda *a: events.append('verify'))
    result = finish.execute(layout, Path('unused'), 1)
    assert result['complete'] is True and events == ['enable', 'queues', 'verify']
    assert finish.read(layout.storage_status)['github_upload_complete'] is True


def add_new_formal(layout, inventory, record_id='formal_60', verified=True):
    original = layout.formal_directory(inventory['formal_records'][-1])
    manifest = copy.deepcopy(finish.read(original / 'manifest.json'))
    manifest.update(record_id=record_id, completed_at=record_id, local_data_deleted=False)
    status = 'verified' if verified else 'pending'
    manifest['upload']['status'] = status
    manifest['remote_manifest']['verified'] = verified
    manifest['files'][0]['upload'] = {'status': status, 'entry': {'key': record_id}}
    directory = layout.canonical / record_id
    write(directory / 'manifest.json', manifest)
    (directory / 'capture.bag').write_bytes(b'x')
    return directory


def emulate_formal_worker_retention_in_tmp(layout):
    current, protected = finish.current_retention(layout)
    for directory, manifest in current:
        if manifest['record_id'] in protected:
            continue
        for item in manifest['files']:
            path = directory / item['path']
            if path.exists(): path.unlink()
        manifest['local_data_deleted'] = True
        write(directory / 'manifest.json', manifest)


def test_new_sixtieth_record_rotates_dynamic_keep_without_changing_frozen_scope(completed):
    layout, inventory = completed
    old_scope = finish.legacy.inventory_digest(inventory)
    historical_keep = {row['record_id'] for row in inventory['latest_five_formal_by_actual_completion']}
    directory = add_new_formal(layout, inventory)
    assert finish.completion_snapshot(layout, inventory)['complete'] is False
    emulate_formal_worker_retention_in_tmp(layout)
    result = finish.completion_snapshot(layout, inventory)
    assert result['complete'] is True and result['formal_records'] == 59
    assert directory.name in result['protected_record_ids']
    assert set(result['protected_record_ids']) != historical_keep
    assert finish.legacy.inventory_digest(inventory) == old_scope
    assert len(inventory['formal_records']) == 59


def test_latest_new_record_must_be_fully_remote_verified(completed):
    layout, inventory = completed
    directory = add_new_formal(layout, inventory, verified=False)
    emulate_formal_worker_retention_in_tmp(layout)
    result = finish.completion_snapshot(layout, inventory)
    assert result['local_complete_records'] == 5 and result['complete'] is False
    assert result['latest_records_pending_upload'] == [directory.name]
    manifest = finish.read(directory / 'manifest.json')
    manifest['upload']['status'] = 'verified'
    manifest['remote_manifest']['verified'] = True
    manifest['files'][0]['upload']['status'] = 'verified'
    write(directory / 'manifest.json', manifest)
    assert finish.completion_snapshot(layout, inventory)['complete'] is True


def test_retention_validation_accepts_current_keep_instead_of_old_snapshot(completed):
    layout, inventory = completed
    add_new_formal(layout, inventory)
    roots = [str(layout.canonical), str(layout.ops)]
    cfg = dict(finish.formal.DEFAULTS, repo=finish.legacy.REPOSITORY, retention_roots=roots,
               deletion_validation={'repo': finish.legacy.REPOSITORY, 'retain_count': 5,
                                    'upload_verified': True, 'restore_verified': True, 'retention_verified': True})
    for root in (layout.canonical, layout.ops): write(root / 'archive_config.json', cfg)
    result = finish.validate_retention(layout, inventory)
    assert result['retention_roots'] == roots


def test_latest_new_record_receives_fresh_remote_verification(completed, monkeypatch):
    layout, inventory = completed
    new = add_new_formal(layout, inventory)
    emulate_formal_worker_retention_in_tmp(layout)
    verified = []
    class Store:
        def verify_file(self, entry): return True
    def store(cfg, manifest):
        verified.append(manifest['record_id'])
        return Store()
    monkeypatch.setattr(finish.formal, 'load_config', lambda *a: {})
    monkeypatch.setattr(finish.formal, 'make_store', store)
    monkeypatch.setattr(finish.legacy, 'make_store', lambda *a: Store())
    finish.verify_completed_remote(layout, inventory)
    assert new.name in verified
    assert len(verified) == 59  # Original 58 closed records plus the new latest record.
    assert finish.evidence.INCOMPLETE_ID not in verified


def test_latest_set_change_during_remote_verification_defers_complete(completed, monkeypatch):
    layout, inventory = completed
    monkeypatch.setattr(finish, 'prepare_all', lambda *a: (inventory, {}))
    monkeypatch.setattr(finish, 'enable_retention', lambda *a: None)
    monkeypatch.setattr(finish, 'run_archives', lambda *a: None)
    def complete_new_record_during_network(*args):
        if not (layout.canonical / 'formal_60').exists():
            add_new_formal(layout, inventory)
            emulate_formal_worker_retention_in_tmp(layout)
    monkeypatch.setattr(finish, 'verify_completed_remote', complete_new_record_during_network)
    first = finish.execute(layout, Path('unused'), 1)
    assert first['complete'] is False and first['retention_changed_during_remote_verification'] is True
    second = finish.execute(layout, Path('unused'), 1)
    assert second['complete'] is True


def test_duplicate_live_record_id_is_not_counted_as_a_five_record_set(completed):
    layout, inventory = completed
    original = layout.formal_directory(inventory['formal_records'][-1])
    write(layout.canonical / original.name / 'manifest.json', finish.read(original / 'manifest.json'))
    with pytest.raises(finish.formal.ArchiveError, match='duplicate record_id'):
        finish.completion_snapshot(layout, inventory)


@pytest.mark.parametrize('operation', ['execute', 'enable_retention', 'run_archives'])
def test_local_only_blocks_upload_and_retention_before_any_action(layout, monkeypatch, operation):
    write(layout.base / 'maintenance/storage_control.json', {'schema': 1, 'state': 'resumed', 'archive_policy': 'local_only'})
    monkeypatch.setattr(finish, 'finalizer_finished', lambda: pytest.fail('policy must reject before service checks'))
    monkeypatch.setattr(finish, 'validate_retention', lambda *a: pytest.fail('must not activate retention'))
    monkeypatch.setattr(finish.legacy, 'run', lambda *a, **kw: pytest.fail('must not upload'))
    with pytest.raises(finish.formal.ArchiveError, match='GitHub archival disabled'):
        if operation == 'execute': finish.execute(layout, Path('unused'), 1)
        elif operation == 'enable_retention': finish.enable_retention(layout, {})
        else: finish.run_archives(layout, 1)
    assert not layout.status.exists()
    assert not (layout.work / 'retention_enabled.json').exists()
