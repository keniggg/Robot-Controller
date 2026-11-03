import hashlib
import errno
import fcntl
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import zlib

import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from minimal_experiment_trace import Cleanup, TraceStore, StorageControlChanged, DeletionIncomplete, atomic, bag_trace_blocks, identity


def database_fd(store):
    expected = store.path.stat()
    found = []
    for path in Path('/proc/self/fd').iterdir():
        try:
            info = path.stat()
        except OSError:
            continue
        if (info.st_dev, info.st_ino) == (expected.st_dev, expected.st_ino):
            found.append(int(path.name))
    assert len(found) == 1
    return found[0]


def test_exact_content_deduplicates_without_losing_original_names(tmp_path):
    data = b'real failure; repaired offline; physical grasp not confirmed\n' * 40000
    store = TraceStore(tmp_path / 'history.sqlite3')
    try:
        for name in ('first.log', 'duplicate.log'):
            store.add_stream(name, [data], {'original_result': 'failure'})
        assert b''.join(store.iter_bytes('first.log')) == data
        assert store.verify() == 2
        assert store.stats()['database_bytes'] < len(data) // 10
        assert store.stats()['distinct_chunks'] == 3
    finally:
        store.close()


def test_corrupt_compressed_object_does_not_emit_recoverable_result(tmp_path):
    store = TraceStore(tmp_path / 'history.sqlite3')
    try:
        store.add_stream('proof.log', [b'original result is failure'], {})
        with store.db:
            store.db.execute("UPDATE blobs SET data=?", (b'corrupt bytes',))
        with pytest.raises(Exception):
            store.verify('proof.log')
    finally:
        store.close()


def sample_bag(path):
    import rosbag
    import rospy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    with rosbag.Bag(str(path), 'w') as bag:
        for i in range(5):
            stamp = rospy.Time(i + 1, 1234)
            bag.write('/grasp/state', String(data='failure' if i == 4 else 'observing'), stamp)
            bag.write('/camera/image', Image(height=1, width=2, encoding='mono8', step=2,
                                           data=bytes([i, i + 1])), stamp)


def test_sparse_ros_trace_keeps_all_state_and_first_last_visual(tmp_path):
    bag = tmp_path / 'source.bag'
    sample_bag(bag)
    counts = {}
    with bag.open('rb') as stream:
        rows = [json.loads(line) for line in b''.join(bag_trace_blocks(stream, counts)).splitlines()]
    assert counts == {'/grasp/state': 5, '/camera/image': 2}
    assert rows[0]['source_is_complete_raw_recording'] is False
    assert rows[-1]['omitted_visual_message_counts']['/camera/image'] == 3
    assert [r['stamp_ns'] for r in rows if r.get('topic') == '/camera/image'] == [1000001234, 5000001234]


@pytest.mark.parametrize('times', [[1, 2, 2], [1, 1, 1], [1]])
def test_visual_endpoints_use_record_identity_when_timestamps_repeat(tmp_path, times):
    import base64
    import rosbag
    import rospy
    from sensor_msgs.msg import Image
    path = tmp_path / 'repeated.bag'
    with rosbag.Bag(str(path), 'w') as bag:
        for i, stamp in enumerate(times):
            bag.write('/image', Image(data=bytes([i])), rospy.Time(stamp))
    with path.open('rb') as stream:
        rows = [json.loads(line) for line in b''.join(bag_trace_blocks(stream, {})).splitlines()]
    values = []
    for row in rows:
        if row.get('kind') == 'message':
            msg = Image().deserialize(base64.b64decode(row['data_base64']))
            values.append(msg.data[0])
    assert values == ([0] if len(times) == 1 else [0, len(times) - 1])


def fixture(tmp_path, raw=False, protected=False):
    root = tmp_path / 'storage'
    run = root / 'maintenance' / 'minimal_trace_cleanup_fixture'
    directory = root / 'ros_log' / 'old'
    run.mkdir(parents=True)
    directory.mkdir(parents=True)
    text = directory / 'result.json'
    text.write_text('{"result":"unknown","failure_fixed":false}\n')
    row = dict(path='ros_log/old/result.json', identity=identity(text.stat()),
               allocated_bytes=text.stat().st_blocks * 512, kind='exact_trace_text', remove_original=True)
    plan = dict(schema=1, root=str(root), retention_profile='minimal_trace_local', authorization_text='retain minimal useful history',
                upload_enabled=False, target_free_bytes=1 << 62, excluded_prefixes=['ros_log/old'] if protected else [],
                text_files=[row], raw_files=[])
    if raw:
        bag = directory / 'source.bag'
        sample_bag(bag)
        rawrow = dict(path='ros_log/old/source.bag', identity=identity(bag.stat()),
                      allocated_bytes=bag.stat().st_blocks * 512, kind='raw_bag', indexed_closed=True,
                      evidence_paths=['ros_log/old/result.json'], original_result='unknown',
                      expected_sha256=hashlib.sha256(bag.read_bytes()).hexdigest())
        plan['raw_files'] = [rawrow]
    atomic(run / 'plan.json', plan)
    return Cleanup(root, run, production=False), text, row


def test_unknown_record_stays_unknown_after_verified_raw_removal(tmp_path):
    cleanup, text, row = fixture(tmp_path, raw=True)
    result = cleanup.run_cleanup()
    assert result['state'] == 'complete' and result['raw_deleted'] == 1
    store = TraceStore(cleanup.run / 'trace.sqlite3', readonly=True)
    try:
        assert json.loads(b''.join(store.iter_bytes(row['path'])))['result'] == 'unknown'
        receipt = store.disposition('ros_log/old/source.bag')['receipt']
        assert receipt['original_result'] == 'unknown' and receipt['full_raw_recoverable'] is False
        assert store.info(receipt['trace_reference'])['metadata']['all_non_visual_messages_retained'] is True
    finally:
        store.close()


def test_source_modified_after_archive_is_retained(tmp_path):
    cleanup, text, row = fixture(tmp_path)
    old_handler = signal.signal(signal.SIGIO, lambda *_: setattr(cleanup, 'broken', True))
    try:
        cleanup.snapshot_text(row)
        text.write_text('{"result":"success","failure_fixed":false}\n')
        with pytest.raises(AssertionError):
            cleanup.delete_text(row)
        assert text.exists() and 'success' in text.read_text()
    finally:
        signal.signal(signal.SIGIO, old_handler)
        cleanup.store.close()


def test_open_file_cannot_be_archived_for_deletion(tmp_path):
    cleanup, text, row = fixture(tmp_path)
    try:
        with text.open('rb'):
            with pytest.raises(BlockingIOError):
                cleanup.snapshot_text(row)
        assert text.exists()
    finally:
        cleanup.store.close()


def test_migrated_or_protected_scope_rejected(tmp_path):
    cleanup, text, row = fixture(tmp_path, protected=True)
    try:
        with pytest.raises(ValueError, match='protected'):
            cleanup.snapshot_text(row)
        assert text.exists()
    finally:
        cleanup.store.close()


def test_corrupt_archive_prevents_original_deletion(tmp_path):
    cleanup, text, row = fixture(tmp_path)
    old_handler = signal.signal(signal.SIGIO, lambda *_: setattr(cleanup, 'broken', True))
    try:
        cleanup.snapshot_text(row)
        with cleanup.store.db:
            cleanup.store.db.execute('UPDATE blobs SET data=?', (b'corrupt archive',))
        with pytest.raises(Exception):
            cleanup.delete_text(row)
        assert text.exists()
        assert cleanup.store.disposition(row['path']) is None
    finally:
        signal.signal(signal.SIGIO, old_handler)
        cleanup.store.close()


def test_target_lease_covers_intent_and_source_unlink(tmp_path, monkeypatch):
    cleanup, text, row = fixture(tmp_path)
    old_handler = signal.signal(signal.SIGIO, lambda *_: setattr(cleanup, 'broken', True))
    observed = []
    try:
        cleanup.snapshot_text(row)
        target_fd = database_fd(cleanup.store)
        journal = cleanup.store.journal
        unlink = Path.unlink
        def checked_journal(path, state, receipt):
            assert fcntl.fcntl(target_fd, fcntl.F_GETLEASE) == fcntl.F_WRLCK
            observed.append(state)
            return journal(path, state, receipt)
        def checked_unlink(path, *args, **kwargs):
            if path == text:
                assert fcntl.fcntl(target_fd, fcntl.F_GETLEASE) == fcntl.F_WRLCK
                observed.append('unlink')
            return unlink(path, *args, **kwargs)
        monkeypatch.setattr(cleanup.store, 'journal', checked_journal)
        monkeypatch.setattr(Path, 'unlink', checked_unlink)
        cleanup.delete_text(row)
        assert observed == ['intent', 'unlink', 'deleted']
        assert fcntl.fcntl(target_fd, fcntl.F_GETLEASE) == fcntl.F_UNLCK
    finally:
        signal.signal(signal.SIGIO, old_handler)
        cleanup.store.close()


def test_same_size_target_rewrite_after_intent_preserves_source(tmp_path, monkeypatch):
    cleanup, text, row = fixture(tmp_path)
    old_handler = signal.signal(signal.SIGIO, lambda *_: setattr(cleanup, 'broken', True))
    attempted, blocked = [], []
    try:
        cleanup.snapshot_text(row)
        compressed = zlib.compress(text.read_bytes(), 9)
        offset = cleanup.store.path.read_bytes().index(compressed) + len(compressed) - 1
        journal = cleanup.store.journal
        def rewrite_after_intent(path, state, receipt):
            journal(path, state, receipt)
            if state == 'intent':
                attempted.append(True)
                try:
                    fd = os.open(cleanup.store.path, os.O_WRONLY | os.O_NONBLOCK | os.O_NOFOLLOW)
                except BlockingIOError:
                    blocked.append(True)
                else:
                    try:
                        os.pwrite(fd, bytes([compressed[-1] ^ 0xff]), offset)
                        os.fsync(fd)
                    finally:
                        os.close(fd)
        monkeypatch.setattr(cleanup.store, 'journal', rewrite_after_intent)
        with pytest.raises((DeletionIncomplete, StorageControlChanged, AssertionError, OSError)):
            cleanup.delete_text(row)
        assert attempted and blocked and text.exists() and cleanup.broken
        assert cleanup.store.disposition(row['path'])['state'] == 'intent'
    finally:
        signal.signal(signal.SIGIO, old_handler)
        cleanup.store.close()


def test_target_disk_corruption_before_lease_is_not_hidden_by_sqlite_cache(tmp_path):
    cleanup, text, row = fixture(tmp_path)
    old_handler = signal.signal(signal.SIGIO, lambda *_: setattr(cleanup, 'broken', True))
    try:
        cleanup.snapshot_text(row)
        cleanup.store.verify(row['path'])
        compressed = zlib.compress(text.read_bytes(), 9)
        before = cleanup.store.path.stat()
        offset = cleanup.store.path.read_bytes().index(compressed) + len(compressed) - 1
        fd = os.open(cleanup.store.path, os.O_WRONLY | os.O_NONBLOCK | os.O_NOFOLLOW)
        try:
            os.pwrite(fd, bytes([compressed[-1] ^ 0xff]), offset)
            os.fsync(fd)
        finally:
            os.close(fd)
        os.utime(cleanup.store.path, ns=(before.st_atime_ns, before.st_mtime_ns))
        assert cleanup.store.path.stat().st_size == before.st_size
        assert cleanup.store.path.read_bytes()[offset] == compressed[-1] ^ 0xff
        with pytest.raises(Exception):
            cleanup.delete_text(row)
        assert text.exists() and cleanup.store.disposition(row['path']) is None
    finally:
        signal.signal(signal.SIGIO, old_handler)
        cleanup.store.close()


def test_uncommitted_target_transaction_cannot_authorize_source_deletion(tmp_path):
    cleanup, text, row = fixture(tmp_path)
    old_handler = signal.signal(signal.SIGIO, lambda *_: setattr(cleanup, 'broken', True))
    try:
        cleanup.snapshot_text(row)
        cleanup.store.db.execute('UPDATE files SET size=size WHERE path=?', (row['path'],))
        assert cleanup.store.db.in_transaction
        with pytest.raises(StorageControlChanged):
            cleanup.delete_text(row)
        assert text.exists() and cleanup.store.disposition(row['path']) is None
    finally:
        cleanup.store.db.rollback()
        signal.signal(signal.SIGIO, old_handler)
        cleanup.store.close()


def test_owned_fd_disk_corruption_after_intent_requires_fresh_readback(tmp_path, monkeypatch):
    cleanup, text, row = fixture(tmp_path)
    old_handler = signal.signal(signal.SIGIO, lambda *_: setattr(cleanup, 'broken', True))
    try:
        cleanup.snapshot_text(row)
        compressed = zlib.compress(text.read_bytes(), 9)
        offset = cleanup.store.path.read_bytes().index(compressed) + len(compressed) - 1
        target_fd = database_fd(cleanup.store)
        journal = cleanup.store.journal
        def corrupt_owned_descriptor(path, state, receipt):
            journal(path, state, receipt)
            if state == 'intent':
                # This owned FD is allowed to write while leased. Injecting a
                # real page change here proves the second read bypasses cache.
                assert fcntl.fcntl(target_fd, fcntl.F_GETLEASE) == fcntl.F_WRLCK
                os.pwrite(target_fd, bytes([compressed[-1] ^ 0xff]), offset)
                os.fsync(target_fd)
        monkeypatch.setattr(cleanup.store, 'journal', corrupt_owned_descriptor)
        with pytest.raises(DeletionIncomplete):
            cleanup.delete_text(row)
        assert text.exists() and not cleanup.broken
        assert cleanup.store.disposition(row['path'])['state'] == 'intent'
    finally:
        signal.signal(signal.SIGIO, old_handler)
        cleanup.store.close()


@pytest.mark.parametrize('raw', [False, True])
@pytest.mark.parametrize('change', ['content_and_its_digest', 'entry_metadata'])
def test_post_intent_target_verify_uses_frozen_proof(tmp_path, monkeypatch, raw, change):
    cleanup, text, row = fixture(tmp_path, raw=raw)
    old_handler = signal.signal(signal.SIGIO, lambda *_: setattr(cleanup, 'broken', True))
    try:
        cleanup.snapshot_text(row)
        source = cleanup.root / cleanup.plan['raw_files'][0]['path'] if raw else text
        journal = cleanup.store.journal
        changed = []
        def rewrite_consistent_entry(path, state, receipt):
            journal(path, state, receipt)
            if state != 'intent':
                return
            reference = receipt['trace_reference']
            info = cleanup.store.info(reference)
            if change == 'content_and_its_digest':
                data = b''.join(cleanup.store.iter_bytes(reference))
                replacement = data[:1] + bytes([data[1] ^ 1]) + data[2:]
                assert len(replacement) == len(data) and replacement != data
                cleanup.store.add_stream(reference, [replacement], info['metadata'])
            else:
                metadata = dict(info['metadata'], kind='rewritten_entry')
                with cleanup.store.db:
                    cleanup.store.db.execute('UPDATE files SET metadata=? WHERE path=?',
                                             (json.dumps(metadata), reference))
            # The modified entry is internally self-consistent; that is not
            # evidence that it still equals the original retained bytes.
            cleanup.store.verify(reference)
            changed.append(True)
        monkeypatch.setattr(cleanup.store, 'journal', rewrite_consistent_entry)
        with pytest.raises((DeletionIncomplete, StorageControlChanged, AssertionError, ValueError)):
            cleanup.delete_raw(cleanup.plan['raw_files'][0]) if raw else cleanup.delete_text(row)
        assert changed and source.exists()
    finally:
        signal.signal(signal.SIGIO, old_handler)
        cleanup.store.close()


@pytest.mark.parametrize('mode', [os.O_RDONLY, os.O_RDWR])
def test_preopened_target_descriptor_prevents_source_deletion(tmp_path, mode):
    cleanup, text, row = fixture(tmp_path)
    old_handler = signal.signal(signal.SIGIO, lambda *_: setattr(cleanup, 'broken', True))
    try:
        cleanup.snapshot_text(row)
        fd = os.open(cleanup.store.path, mode | os.O_NONBLOCK | os.O_NOFOLLOW)
        try:
            with pytest.raises((StorageControlChanged, BlockingIOError, AssertionError)):
                cleanup.delete_text(row)
            assert text.exists() and cleanup.store.disposition(row['path']) is None
        finally:
            os.close(fd)
    finally:
        signal.signal(signal.SIGIO, old_handler)
        cleanup.store.close()


@pytest.mark.parametrize('mode', [os.O_RDONLY, os.O_RDWR])
def test_other_process_preopened_target_is_rejected_by_kernel_lease(tmp_path, mode):
    cleanup, text, row = fixture(tmp_path)
    old_handler = signal.signal(signal.SIGIO, lambda *_: setattr(cleanup, 'broken', True))
    child = None
    try:
        cleanup.snapshot_text(row)
        child = subprocess.Popen([sys.executable, '-B', '-c',
            'import os,sys; fd=os.open(sys.argv[1],int(sys.argv[2])); '
            'print("ready",flush=True); sys.stdin.read(1); os.close(fd)',
            str(cleanup.store.path), str(mode)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        assert child.stdout.readline().strip() == 'ready'
        # The parent still has exactly one target FD. The kernel, rather than
        # a /proc scan of the other process, must reject the lease.
        database_fd(cleanup.store)
        with pytest.raises(StorageControlChanged):
            cleanup.delete_text(row)
        assert text.exists() and cleanup.store.disposition(row['path']) is None
    finally:
        if child is not None:
            child.communicate('x', timeout=10)
        signal.signal(signal.SIGIO, old_handler)
        cleanup.store.close()


def test_source_same_size_rewrite_with_restored_mtime_is_preserved(tmp_path):
    cleanup, text, row = fixture(tmp_path)
    old_handler = signal.signal(signal.SIGIO, lambda *_: setattr(cleanup, 'broken', True))
    try:
        cleanup.snapshot_text(row)
        before = text.stat()
        data = text.read_bytes()
        text.write_bytes(b'X' + data[1:])
        os.utime(text, ns=(before.st_atime_ns, before.st_mtime_ns))
        assert identity(text.stat()) == row['identity']
        with pytest.raises((AssertionError, ValueError, StorageControlChanged)):
            cleanup.delete_text(row)
        assert text.exists()
    finally:
        signal.signal(signal.SIGIO, old_handler)
        cleanup.store.close()


def test_real_source_lease_break_during_target_verification_preserves_source(tmp_path, monkeypatch):
    cleanup, text, row = fixture(tmp_path)
    old_handler = signal.signal(signal.SIGIO, lambda *_: setattr(cleanup, 'broken', True))
    try:
        cleanup.snapshot_text(row)
        verify = cleanup.store.verify
        blocked = []
        def request_source_write(*args, **kwargs):
            result = verify(*args, **kwargs)
            if not blocked:
                with pytest.raises(BlockingIOError):
                    fd = os.open(text, os.O_WRONLY | os.O_NONBLOCK | os.O_NOFOLLOW)
                    os.close(fd)
                blocked.append(True)
            return result
        monkeypatch.setattr(cleanup.store, 'verify', request_source_write)
        with pytest.raises(StorageControlChanged):
            cleanup.delete_text(row)
        assert blocked and cleanup.broken and text.exists()
        assert cleanup.store.disposition(row['path']) is None
    finally:
        signal.signal(signal.SIGIO, old_handler)
        cleanup.store.close()


def test_unsupported_target_lease_preserves_source(tmp_path, monkeypatch):
    cleanup, text, row = fixture(tmp_path)
    old_handler = signal.signal(signal.SIGIO, lambda *_: setattr(cleanup, 'broken', True))
    try:
        cleanup.snapshot_text(row)
        target_fd = database_fd(cleanup.store)
        original = fcntl.fcntl
        def unsupported(fd, command, *args):
            if fd == target_fd and command == fcntl.F_SETLEASE and args == (fcntl.F_WRLCK,):
                raise OSError(errno.EOPNOTSUPP, 'target filesystem does not support leases')
            return original(fd, command, *args)
        monkeypatch.setattr(fcntl, 'fcntl', unsupported)
        with pytest.raises((StorageControlChanged, OSError)):
            cleanup.delete_text(row)
        assert text.exists() and cleanup.store.disposition(row['path']) is None
    finally:
        signal.signal(signal.SIGIO, old_handler)
        cleanup.store.close()


def test_target_path_replacement_after_intent_preserves_source(tmp_path, monkeypatch):
    cleanup, text, row = fixture(tmp_path)
    old_handler = signal.signal(signal.SIGIO, lambda *_: setattr(cleanup, 'broken', True))
    try:
        cleanup.snapshot_text(row)
        replacement = cleanup.run / 'replacement.sqlite3'
        replacement.write_bytes(cleanup.store.path.read_bytes())
        journal = cleanup.store.journal
        def replace_after_intent(path, state, receipt):
            journal(path, state, receipt)
            if state == 'intent':
                replacement.replace(cleanup.store.path)
        monkeypatch.setattr(cleanup.store, 'journal', replace_after_intent)
        with pytest.raises((DeletionIncomplete, StorageControlChanged, AssertionError, OSError)):
            cleanup.delete_text(row)
        assert text.exists()
    finally:
        signal.signal(signal.SIGIO, old_handler)
        cleanup.store.close()


def test_target_path_replacement_during_final_mount_check_preserves_source(tmp_path, monkeypatch):
    cleanup, text, row = fixture(tmp_path)
    old_handler = signal.signal(signal.SIGIO, lambda *_: setattr(cleanup, 'broken', True))
    try:
        cleanup.snapshot_text(row)
        replacement = cleanup.run / 'replacement.sqlite3'
        replacement.write_bytes(cleanup.store.path.read_bytes())
        mounted = cleanup.mounted
        replaced = []
        def replace_during_mount_check():
            mounted()
            disposition = cleanup.store.disposition(row['path'])
            if disposition and disposition['state'] == 'intent' and not replaced:
                replacement.replace(cleanup.store.path)
                replaced.append(True)
        monkeypatch.setattr(cleanup, 'mounted', replace_during_mount_check)
        with pytest.raises((DeletionIncomplete, StorageControlChanged, OSError)):
            cleanup.delete_text(row)
        assert replaced and text.exists()
    finally:
        signal.signal(signal.SIGIO, old_handler)
        cleanup.store.close()


def test_lease_break_is_not_reset_by_opening_the_next_source(tmp_path):
    cleanup, text, row = fixture(tmp_path)
    cleanup.broken = True
    descriptor = None
    try:
        with pytest.raises(StorageControlChanged):
            descriptor = cleanup.open_leased(text, row)
        assert cleanup.broken and text.exists()
    finally:
        if descriptor is not None:
            os.close(descriptor)
        cleanup.store.close()


def test_unresolved_intent_blocks_automatic_continuation(tmp_path):
    cleanup, text, row = fixture(tmp_path)
    try:
        cleanup.store.journal(row['path'], 'intent', {'original_result': 'unknown'})
        with pytest.raises(AssertionError, match='unresolved'):
            cleanup.run_cleanup()
        assert text.exists()
    finally:
        cleanup.store.close()


@pytest.mark.parametrize('failure', ['unlink', 'directory_fsync'])
def test_post_intent_failure_stops_without_complete_receipt(tmp_path, monkeypatch, failure):
    import minimal_experiment_trace as module
    cleanup, text, row = fixture(tmp_path)
    if failure == 'unlink':
        original = Path.unlink
        def unlink(path, *args, **kwargs):
            if path == text:
                raise OSError('injected unlink failure')
            return original(path, *args, **kwargs)
        monkeypatch.setattr(Path, 'unlink', unlink)
    else:
        original = module.fsync_dir
        def fsync_dir(path):
            if path == text.parent:
                raise OSError('injected directory fsync failure')
            return original(path)
        monkeypatch.setattr(module, 'fsync_dir', fsync_dir)
    with pytest.raises(DeletionIncomplete, match='unresolved'):
        cleanup.run_cleanup()
    assert not (cleanup.run / 'receipt.json').exists()
    progress = json.loads((cleanup.run / 'progress.json').read_text())
    assert progress['state'] == 'stopped' and progress['text_deleted'] == 0
    store = TraceStore(cleanup.run / 'trace.sqlite3', readonly=True)
    try:
        assert store.disposition(row['path'])['state'] == 'intent'
        assert json.loads(b''.join(store.iter_bytes(row['path'])))['result'] == 'unknown'
    finally:
        store.close()


def test_resume_receipt_includes_prior_durable_deletions(tmp_path):
    cleanup, text, row = fixture(tmp_path, raw=True)
    cleanup.snapshot_text(row)
    cleanup.delete_text(row)
    cleanup.store.close()
    resumed = Cleanup(cleanup.root, cleanup.run, production=False)
    result = resumed.run_cleanup()
    assert result['text_deleted'] == 1 and result['raw_deleted'] == 1
    assert result['deleted_allocated_bytes'] == sum(r['allocated_bytes'] for r in
                                                  resumed.plan['text_files'] + resumed.plan['raw_files'])


def test_invalid_path_cannot_escape_storage(tmp_path):
    cleanup, text, row = fixture(tmp_path)
    try:
        for name in ('../private', '/etc/passwd', 'ros_log/../../private'):
            with pytest.raises(ValueError):
                cleanup.path(name)
        assert text.exists()
    finally:
        cleanup.store.close()


def production_controls(cleanup, monkeypatch):
    import minimal_experiment_trace as module
    monkeypatch.setattr(module, 'ROOT', cleanup.root)
    monkeypatch.setattr(module, 'MOUNT', cleanup.root.parent)
    monkeypatch.setattr(Path, 'is_mount', lambda p: p == cleanup.root.parent)
    control = cleanup.root / 'maintenance/storage_control.json'
    atomic(control, dict(state='resumed', archive_policy='local_only'))
    config = cleanup.root / 'grasp_records/archive_config.json'
    config.parent.mkdir()
    atomic(config, dict(archive_policy='local_only', upload_enabled=False, delete_enabled=False))
    calls = []
    def mount_info(*args, **kwargs):
        calls.append(args)
        return '4899-D74E exfat rw,noatime\n'
    monkeypatch.setattr(module.subprocess, 'check_output', mount_info)
    cleanup.production = True
    return control, calls


def test_cached_mount_check_keeps_immediate_plan_identity_check(tmp_path, monkeypatch):
    cleanup, text, row = fixture(tmp_path)
    try:
        control, calls = production_controls(cleanup, monkeypatch)
        for _ in range(20):
            cleanup.mounted()
        assert len(calls) == 1
        value = json.loads(cleanup.plan_path.read_text())
        value['authorization_text'] = 'changed authorization'
        atomic(cleanup.plan_path, value)
        with pytest.raises(StorageControlChanged, match='plan identity'):
            cleanup.mounted()
        assert text.exists()
    finally:
        cleanup.store.close()


def test_storage_pause_blocks_mutation_with_cached_mount_info(tmp_path, monkeypatch):
    cleanup, text, row = fixture(tmp_path)
    try:
        control, calls = production_controls(cleanup, monkeypatch)
        cleanup.mounted()
        atomic(control, dict(state='paused', archive_policy='local_only'))
        with pytest.raises(StorageControlChanged, match='authorization changed'):
            cleanup.snapshot_text(row)
        assert len(calls) == 1 and text.exists()
    finally:
        cleanup.store.close()
