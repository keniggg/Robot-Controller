import hashlib
import io
import json
import os
from pathlib import Path
import signal
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from minimal_experiment_trace import Cleanup, TraceStore, StorageControlChanged, DeletionIncomplete, atomic, bag_trace_blocks, identity


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
