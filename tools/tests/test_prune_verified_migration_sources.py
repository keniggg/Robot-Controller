"""System-disk duplicate removal exercised only in temporary directories."""
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import prune_verified_migration_sources as migration


@pytest.fixture
def setup(tmp_path, monkeypatch):
    source = tmp_path / 'source'
    source.mkdir()
    mount = tmp_path / 'external'
    mount.mkdir()
    base = mount / '实验记录/Robot-Controller'
    target = base / 'ros_log'
    target.mkdir(parents=True)
    monkeypatch.setattr(migration, 'mountpoints', lambda: {mount})
    (source / 'capture.bag').write_bytes(b'closed experimental record')
    (target / 'capture.bag').write_bytes(b'closed experimental record')
    return source, target, base, mount


def run(setup, apply=False):
    source, target, base, mount = setup
    return migration.process(base, [(source, 'ros_log')], apply=apply, mount=mount)


def test_dry_run_is_read_only_then_apply_retains_external_bytes(setup):
    source, target, base, mount = setup
    assert run(setup)['verified'] == 1
    assert (source / 'capture.bag').exists()
    assert not (base / 'system_disk_migration_cleanup.json').exists()
    assert run(setup, True)['removed'] == 1
    assert not (source / 'capture.bag').exists()
    assert source.is_dir()
    assert (target / 'capture.bag').read_bytes() == b'closed experimental record'
    index = json.loads((base / 'system_disk_migration_cleanup.json').read_text())
    assert index['files'][str(source / 'capture.bag')]['state'] == 'removed'
    assert run(setup, True)['removed'] == 0


@pytest.mark.parametrize('name', ['tool.py', 'launch.sh', 'driver.launch', 'params.yaml', 'params.yml',
                                  'archive_config.json', 'archive_config_backup.json', 'capture.bag.partial', 'live.bag.active'])
def test_control_sources_and_incomplete_files_remain(setup, name):
    source, target, base, mount = setup
    (source / name).write_bytes(b'keep')
    (target / name).write_bytes(b'keep')
    assert run(setup, True)['retained'] == 1
    assert (source / name).is_file()


def test_only_explicit_runtime_status_paths_are_retained(setup, monkeypatch):
    source, target, base, _ = setup
    names = ('disk_status.json', 'worker_status.json')
    protected = {source / name for name in names}
    monkeypatch.setattr(migration, 'KEEP_RUNTIME_STATUS_PATHS', protected)
    for root in (source, target):
        (root / 'experiment').mkdir()
        for name in names:
            (root / name).write_text('current runtime status')
            (root / 'experiment' / name).write_text('closed experimental evidence')
    result = run(setup, True)
    assert result['retained'] == 2
    assert result['removed'] == 3
    assert all(path.is_file() for path in protected)
    assert all(not (source / 'experiment' / name).exists() for name in names)
    assert all((target / 'experiment' / name).is_file() for name in names)


def test_same_size_wrong_hash_never_removed(setup):
    source, target, _, _ = setup
    (target / 'capture.bag').write_bytes(b'x' * (source / 'capture.bag').stat().st_size)
    assert run(setup, True)['mismatch'] == 1
    assert (source / 'capture.bag').is_file()


def test_missing_copy_and_symlinks_are_preserved(setup):
    source, target, _, _ = setup
    (target / 'capture.bag').unlink()
    (source / 'alias').symlink_to(source / 'capture.bag')
    (source / 'linked_dir').symlink_to(target, target_is_directory=True)
    assert run(setup, True)['retained'] == 3
    assert (source / 'capture.bag').is_file()
    assert (source / 'alias').is_symlink()


def test_missing_mount_refuses_even_when_mount_directory_exists(setup, monkeypatch):
    source, target, _, _ = setup
    monkeypatch.setattr(migration, 'mountpoints', lambda: set())
    with pytest.raises(migration.ArchiveError, match='not mounted'):
        run(setup, True)
    assert (source / 'capture.bag').is_file()


def test_open_source_or_target_writer_is_not_removed(setup):
    source, target, _, _ = setup
    with (source / 'capture.bag').open('rb'):
        assert run(setup, True)['retained'] == 1
    with (target / 'capture.bag').open('ab'):
        assert run(setup, True)['retained'] == 1
    assert (source / 'capture.bag').is_file()


def test_source_changed_while_destination_hashes_is_preserved(setup, monkeypatch):
    source, target, _, _ = setup
    original = migration.hash_stable
    def hash_then_change(path):
        result = original(path)
        if path == target / 'capture.bag':
            (source / 'capture.bag').write_bytes(b'new live data')
        return result
    monkeypatch.setattr(migration, 'hash_stable', hash_then_change)
    assert run(setup, True)['retained'] == 1
    assert (source / 'capture.bag').read_bytes() == b'new live data'


def test_crash_after_unlink_recovers_pending_manifest(setup, monkeypatch):
    source, target, base, mount = setup
    original = migration.atomic_json
    def crash(path, value):
        if not (source / 'capture.bag').exists():
            raise KeyboardInterrupt('simulated power loss')
        original(path, value)
    monkeypatch.setattr(migration, 'atomic_json', crash)
    with pytest.raises(KeyboardInterrupt):
        run(setup, True)
    monkeypatch.setattr(migration, 'atomic_json', original)
    assert run(setup, True)['removed'] == 1
    assert (target / 'capture.bag').is_file()
    entry = json.loads((base / 'system_disk_migration_cleanup.json').read_text())['files'][str(source / 'capture.bag')]
    assert entry['state'] == 'removed'


@pytest.mark.parametrize('name', ['layout.rviz', 'settings.cfg', 'runtime.ini', 'launch.bash', 'module.pyc'])
def test_additional_operational_configuration_suffixes_remain(setup, name):
    source, target, _, _ = setup
    (source / name).write_text('operational configuration')
    (target / name).write_text('operational configuration')
    assert run(setup, True)['retained'] == 1
    assert (source / name).is_file()


def test_external_directory_cannot_be_used_as_prune_source(setup):
    source, target, base, mount = setup
    with pytest.raises(migration.ArchiveError, match='outside the external mount'):
        migration.process(base, [(target, 'ros_log')], apply=True, mount=mount)
    assert (target / 'capture.bag').is_file()


def test_external_base_parent_traversal_is_rejected(setup):
    source, target, base, mount = setup
    with pytest.raises(migration.ArchiveError, match='without parent traversal'):
        migration.process(base / '..' / base.name, [(source, 'ros_log')], apply=True, mount=mount)
    assert (source / 'capture.bag').is_file()


def add_pairs(setup, count):
    source, target, _, _ = setup
    paths = [source / 'capture.bag']
    for i in range(1, count):
        name = 'extra_%02d.json' % i
        (source / name).write_text('record %d' % i)
        (target / name).write_text('record %d' % i)
        paths.append(source / name)
    return paths


def test_batch_uses_one_screen_scan_plus_one_fresh_scan_per_unlink(setup, monkeypatch):
    paths = add_pairs(setup, 8)
    source, target, base, mount = setup
    scans, saves = [], []
    original_save = migration.atomic_json
    def scan(*, paths=(), roots=()):
        scans.append((tuple(paths), tuple(roots)))
        return {'open': set(), 'writers': set()}
    def save(path, value):
        saves.append({key: row['state'] for key, row in value['files'].items()})
        return original_save(path, value)
    monkeypatch.setattr(migration, 'descriptor_snapshot', scan)
    monkeypatch.setattr(migration, 'atomic_json', save)
    report = migration.process(base, [(source, 'ros_log')], apply=True, mount=mount, batch_size=8)
    assert report['removed'] == 8
    assert len(scans) == 9
    assert len(scans[0][1]) == 2
    assert all(len(exact) == 2 and not roots for exact, roots in scans[1:])
    assert len(saves) == 2
    assert set(saves[0].values()) == {'removal_pending'}
    assert set(saves[1].values()) == {'removed'}
    assert all(not path.exists() for path in paths)


def test_batch_intent_is_durable_for_every_file_before_any_unlink(setup, monkeypatch):
    paths = add_pairs(setup, 3)
    source, target, base, mount = setup
    original_unlink = Path.unlink
    pending_snapshots = []
    def unlink(path, *args, **kwargs):
        if path in paths:
            persisted = json.loads((base / 'system_disk_migration_cleanup.json').read_text())
            pending_snapshots.append(persisted)
            assert all(persisted['files'][str(p)]['state'] == 'removal_pending' for p in paths)
            assert all(persisted['files'][str(p)]['sha256'] for p in paths)
        return original_unlink(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'unlink', unlink)
    assert run(setup, True)['removed'] == 3
    assert len(pending_snapshots) == 3


def test_crash_mid_batch_keeps_all_pending_and_recovers_already_unlinked(setup, monkeypatch):
    paths = add_pairs(setup, 3)
    source, target, base, mount = setup
    original_unlink = Path.unlink
    removed = []
    def crash(path, *args, **kwargs):
        result = original_unlink(path, *args, **kwargs)
        if path in paths:
            removed.append(path)
            raise KeyboardInterrupt('power loss after first unlink')
        return result
    monkeypatch.setattr(Path, 'unlink', crash)
    with pytest.raises(KeyboardInterrupt):
        run(setup, True)
    persisted = json.loads((base / 'system_disk_migration_cleanup.json').read_text())
    assert len(removed) == 1
    assert set(row['state'] for row in persisted['files'].values()) == {'removal_pending'}
    assert sum(path.exists() for path in paths) == 2
    monkeypatch.setattr(Path, 'unlink', original_unlink)
    assert run(setup, True)['removed'] == 3
    assert all(not path.exists() for path in paths)
    assert all((target / path.name).is_file() for path in paths)
    persisted = json.loads((base / 'system_disk_migration_cleanup.json').read_text())
    assert set(row['state'] for row in persisted['files'].values()) == {'removed'}


def test_crash_after_batch_intent_before_first_delete_rehashes_on_restart(setup, monkeypatch):
    paths = add_pairs(setup, 3)
    source, target, base, mount = setup
    original_save = migration.atomic_json
    def crash(path, value):
        original_save(path, value)
        if any(row['state'] == 'removal_pending' for row in value['files'].values()):
            raise KeyboardInterrupt('power loss after durable intent')
    monkeypatch.setattr(migration, 'atomic_json', crash)
    with pytest.raises(KeyboardInterrupt):
        run(setup, True)
    assert all(path.exists() for path in paths)
    (target / paths[1].name).write_text('changed after crash')
    monkeypatch.setattr(migration, 'atomic_json', original_save)
    report = run(setup, True)
    assert report['removed'] == 2 and report['mismatch'] == 1
    assert paths[1].exists()


@pytest.mark.parametrize('which', ['source', 'destination'])
def test_fresh_fd_scan_after_batch_journal_catches_new_open_handle(setup, monkeypatch, which):
    source, target, base, mount = setup
    chosen = source / 'capture.bag' if which == 'source' else target / 'capture.bag'
    original_save = migration.atomic_json
    handles = []
    def open_after_intent(path, value):
        original_save(path, value)
        if not handles and any(row['state'] == 'removal_pending' for row in value['files'].values()):
            handles.append(chosen.open('rb' if which == 'source' else 'ab'))
    monkeypatch.setattr(migration, 'atomic_json', open_after_intent)
    try:
        report = run(setup, True)
        assert report['removed'] == 0 and report['failed'] == 1
        assert (source / 'capture.bag').is_file()
    finally:
        for handle in handles:
            handle.close()


@pytest.mark.parametrize('which', ['source', 'destination'])
def test_fresh_identity_after_batch_journal_catches_same_size_rewrite(setup, monkeypatch, which):
    source, target, base, mount = setup
    chosen = source / 'capture.bag' if which == 'source' else target / 'capture.bag'
    original_save = migration.atomic_json
    mutated = []
    def change_after_intent(path, value):
        original_save(path, value)
        if not mutated and any(row['state'] == 'removal_pending' for row in value['files'].values()):
            mutated.append(True)
            chosen.write_bytes(b'x' * chosen.stat().st_size)
    monkeypatch.setattr(migration, 'atomic_json', change_after_intent)
    report = run(setup, True)
    assert report['removed'] == 0 and report['failed'] == 1
    assert (source / 'capture.bag').is_file()


def test_new_batch_retains_prior_removed_journal_entries(setup):
    source, target, base, mount = setup
    assert run(setup, True)['removed'] == 1
    first = json.loads((base / 'system_disk_migration_cleanup.json').read_text())['files'][str(source / 'capture.bag')]
    (source / 'next.json').write_text('next')
    (target / 'next.json').write_text('next')
    assert run(setup, True)['removed'] == 1
    current = json.loads((base / 'system_disk_migration_cleanup.json').read_text())['files']
    assert current[str(source / 'capture.bag')] == first
    assert current[str(source / 'next.json')]['state'] == 'removed'


def test_batch_flushes_early_at_byte_budget(setup, monkeypatch):
    paths = add_pairs(setup, 3)
    source, target, base, mount = setup
    monkeypatch.setattr(migration, 'BATCH_BYTES', 1)
    original_save = migration.atomic_json
    saved = []
    def save(path, value):
        saved.append(sum(row['state'] == 'removal_pending' for row in value['files'].values()))
        return original_save(path, value)
    monkeypatch.setattr(migration, 'atomic_json', save)
    assert run(setup, True)['removed'] == 3
    assert saved == [1, 0, 1, 0, 1, 0]


@pytest.mark.parametrize('batch_size', [0, -1, 1025, True, 1.2])
def test_invalid_batch_size_has_no_data_effect(setup, batch_size):
    source, target, base, mount = setup
    with pytest.raises(migration.ArchiveError, match='batch size'):
        migration.process(base, [(source, 'ros_log')], apply=True, mount=mount, batch_size=batch_size)
    assert (source / 'capture.bag').is_file()
    assert not (base / 'system_disk_migration_cleanup.json').exists()
