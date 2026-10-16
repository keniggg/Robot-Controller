#!/usr/bin/env python3
"""Locally retain searchable history and verified compressed evidence before cleanup."""
import argparse
import base64
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import sqlite3
import stat
import subprocess
import sys
import time
import zlib

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
MOUNT = Path('/media/zhuyupei/SanStand')
CHUNK = 1024 * 1024
HEAVY_TYPES = {'sensor_msgs/Image', 'sensor_msgs/CompressedImage', 'sensor_msgs/PointCloud2'}


class StorageControlChanged(Exception):
    pass


class DeletionIncomplete(Exception):
    """A durable deletion intent must be reconciled before continuing."""


def identity(info):
    return [info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns, info.st_mode, info.st_nlink]


def sha_file(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(8 * CHUNK), b''):
            h.update(block)
    return h.hexdigest()


def atomic(path, value):
    data = (json.dumps(value, ensure_ascii=False, indent=2) + '\n').encode()
    temp = path.with_name('.' + path.name + '.' + str(os.getpid()) + '.tmp')
    with temp.open('xb') as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())
    os.replace(temp, path)
    fsync_dir(path.parent)


def fsync_dir(path):
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


class TraceStore:
    """Content-addressed, compressed chunks with original paths and SHA256."""
    def __init__(self, path, readonly=False):
        self.path = Path(path)
        self.readonly = readonly
        self.db = sqlite3.connect('file:' + str(self.path) + '?mode=ro' if readonly else str(self.path),
                                  uri=readonly, timeout=30)
        if not readonly:
            self.db.execute('PRAGMA journal_mode=DELETE')
            self.db.execute('PRAGMA synchronous=FULL')
            self.db.executescript('''
              CREATE TABLE IF NOT EXISTS blobs(sha TEXT PRIMARY KEY, size INTEGER NOT NULL, data BLOB NOT NULL);
              CREATE TABLE IF NOT EXISTS files(path TEXT PRIMARY KEY, sha TEXT NOT NULL, size INTEGER NOT NULL,
                metadata TEXT NOT NULL);
              CREATE TABLE IF NOT EXISTS members(path TEXT NOT NULL, position INTEGER NOT NULL, sha TEXT NOT NULL,
                PRIMARY KEY(path,position));
              CREATE TABLE IF NOT EXISTS deletions(path TEXT PRIMARY KEY, state TEXT NOT NULL, receipt TEXT NOT NULL);
            ''')
            self.db.commit()

    def close(self):
        self.db.close()

    def add_stream(self, path, blocks, metadata):
        assert not self.readonly
        h = hashlib.sha256()
        total, position = 0, 0
        buffer = bytearray()
        with self.db:
            self.db.execute('DELETE FROM members WHERE path=?', (path,))
            def write(block):
                nonlocal total, position
                digest = hashlib.sha256(block).hexdigest()
                self.db.execute('INSERT OR IGNORE INTO blobs VALUES(?,?,?)',
                                (digest, len(block), zlib.compress(block, 9)))
                self.db.execute('INSERT INTO members VALUES(?,?,?)', (path, position, digest))
                h.update(block)
                total += len(block)
                position += 1
            for block in blocks:
                buffer.extend(block)
                while len(buffer) >= CHUNK:
                    write(bytes(buffer[:CHUNK]))
                    del buffer[:CHUNK]
            if buffer:
                write(bytes(buffer))
            digest = h.hexdigest()
            self.db.execute('INSERT OR REPLACE INTO files VALUES(?,?,?,?)',
                (path, digest, total, json.dumps(metadata, ensure_ascii=False)))
        self.verify(path)
        return dict(path=path, sha256=digest, size_bytes=total)

    def iter_bytes(self, path):
        row = self.db.execute('SELECT sha,size FROM files WHERE path=?', (path,)).fetchone()
        if row is None:
            raise KeyError(path)
        cursor = self.db.execute('SELECT blobs.sha,blobs.size,blobs.data FROM members JOIN blobs '
                                'ON members.sha=blobs.sha WHERE members.path=? ORDER BY members.position', (path,))
        h = hashlib.sha256()
        total = 0
        for digest, size, compressed in cursor:
            data = zlib.decompress(compressed)
            if len(data) != size or hashlib.sha256(data).hexdigest() != digest:
                raise ValueError('corrupt compressed trace chunk: ' + path)
            h.update(data)
            total += len(data)
            yield data
        if h.hexdigest() != row[0] or total != row[1]:
            raise ValueError('corrupt trace file: ' + path)

    def verify(self, path=None):
        paths = [path] if path else [r[0] for r in self.db.execute('SELECT path FROM files')]
        for name in paths:
            for _ in self.iter_bytes(name):
                pass
        return len(paths)

    def info(self, path):
        row = self.db.execute('SELECT sha,size,metadata FROM files WHERE path=?', (path,)).fetchone()
        if row is None:
            raise KeyError(path)
        return dict(path=path, sha256=row[0], size_bytes=row[1], metadata=json.loads(row[2]))

    def disposition(self, path):
        row = self.db.execute('SELECT state,receipt FROM deletions WHERE path=?', (path,)).fetchone()
        return None if row is None else dict(state=row[0], receipt=json.loads(row[1]))

    def journal(self, path, state, receipt):
        with self.db:
            self.db.execute('INSERT OR REPLACE INTO deletions VALUES(?,?,?)',
                            (path, state, json.dumps(receipt, ensure_ascii=False)))

    def deletion_totals(self):
        totals = dict(text_deleted=0, raw_deleted=0, deleted_allocated_bytes=0)
        for (value,) in self.db.execute("SELECT receipt FROM deletions WHERE state='deleted'"):
            receipt = json.loads(value)
            key = 'text_deleted' if receipt['full_raw_recoverable'] else 'raw_deleted'
            totals[key] += 1
            totals['deleted_allocated_bytes'] += receipt['allocated_bytes']
        return totals

    def stats(self):
        return dict(files=self.db.execute('SELECT count(*) FROM files').fetchone()[0],
                    original_bytes=self.db.execute('SELECT coalesce(sum(size),0) FROM files').fetchone()[0],
                    distinct_chunks=self.db.execute('SELECT count(*) FROM blobs').fetchone()[0],
                    database_bytes=self.path.stat().st_size,
                    deleted_files=self.db.execute("SELECT count(*) FROM deletions WHERE state='deleted'").fetchone()[0])


def bag_trace_blocks(stream, counts):
    """Keep all non-image messages, definitions, and first/last visual samples."""
    import rosbag
    with rosbag.Bag(stream, 'r') as bag:
        info = bag.get_type_and_topic_info()
        connections = [dict(topic=c.topic, datatype=c.datatype, md5sum=c.md5sum,
                            definition=c.msg_def) for c in bag._get_connections()]
        totals = {topic: item.message_count for topic, item in info.topics.items()}
        heavy = [topic for topic, item in info.topics.items() if item.msg_type in HEAVY_TYPES]
        normal = [topic for topic in info.topics if topic not in heavy]
        header = dict(kind='trace_header', source_is_complete_raw_recording=False,
                      sparse_visual_samples=True, full_message_counts=totals, connections=connections)
        yield (json.dumps(header, ensure_ascii=False) + '\n').encode()
        def encoded(topic, msg, stamp, sample):
            datatype, data, md5sum = msg[:3]
            counts[topic] = counts.get(topic, 0) + 1
            return (json.dumps(dict(kind='message', topic=topic, datatype=datatype,
                md5sum=md5sum, stamp_ns=stamp.to_nsec(), sparse_visual_sample=sample,
                data_base64=base64.b64encode(data).decode('ascii')), ensure_ascii=False) + '\n').encode()
        if normal:
            for topic, msg, stamp in bag.read_messages(topics=normal, raw=True):
                yield encoded(topic, msg, stamp, False)
        for topic in heavy:
            ids = [c.id for c in bag._get_connections(topics=[topic])]
            indexes = [entry for i in ids for entry in bag._connection_indexes[i]]
            if not indexes or len(indexes) != totals[topic]:
                raise ValueError('incomplete visual index')
            # Equal timestamps still identify different records. Use the physical
            # position as tie-breaker and read that exact indexed record.
            key = lambda entry: (entry.time.to_nsec(), entry.position)
            first, last = min(indexes, key=key), max(indexes, key=key)
            for entry in ([first] if first.position == last.position else [first, last]):
                sample = bag._read_message(entry.position, raw=True)
                if sample.topic != topic:
                    raise ValueError('visual index topic mismatch')
                yield encoded(sample.topic, sample.message, sample.timestamp, True)
        if any(counts.get(topic, 0) != totals[topic] for topic in normal):
            raise ValueError('non-visual message trace incomplete')
        yield (json.dumps(dict(kind='trace_footer', retained_message_counts=counts,
                              omitted_visual_message_counts={t: totals[t] - counts.get(t, 0) for t in heavy}),
                          ensure_ascii=False) + '\n').encode()


class Cleanup:
    def __init__(self, root, run, production=True):
        self.root, self.run = Path(root), Path(run)
        self.production = production
        self.plan_path = self.run / 'plan.json'
        self.plan = json.loads(self.plan_path.read_text())
        self.plan_sha = sha_file(self.plan_path)
        self.plan_identity = identity(self.plan_path.stat())
        self.device = None
        self.next_integrity_check = 0.0
        assert self.plan['schema'] == 1 and self.plan['retention_profile'] == 'minimal_trace_local'
        assert self.plan['authorization_text'].strip() and self.plan['upload_enabled'] is False
        assert self.plan['root'] == str(self.root)
        if production:
            assert sha_file(Path(__file__)) == self.plan['source_tool_sha256'], 'cleanup tool changed after planning'
        self.store = TraceStore(self.run / 'trace.sqlite3')
        self.broken = False
        self.progress = dict(state='packing', text_packed=0, text_deleted=0, raw_deleted=0,
                             deleted_allocated_bytes=0, skipped=[], github_contacted=False,
                             motion_commands_sent=False, temperature_queried=False)
        self.progress.update(self.store.deletion_totals())

    def mounted(self):
        if not self.production:
            return
        if self.root != ROOT or not MOUNT.is_mount():
            raise StorageControlChanged('SanStand unavailable')
        device = self.root.stat().st_dev
        if device != MOUNT.stat().st_dev or (self.device is not None and device != self.device):
            raise StorageControlChanged('SanStand device changed')
        if identity(self.plan_path.stat()) != self.plan_identity:
            raise StorageControlChanged('cleanup plan identity changed')
        # The parsed scope stays fixed. Check its file identity for each mutation;
        # rehash the large plan and resolve the mount once per second, rather
        # than spawning findmnt and hashing the same plan for every small file.
        if time.monotonic() >= self.next_integrity_check:
            fields = subprocess.check_output(['findmnt', '-n', '-o', 'UUID,FSTYPE,OPTIONS', '-T', str(MOUNT)], text=True).split()
            if fields[:2] != ['4899-D74E', 'exfat'] or 'rw' not in fields[2].split(',') or sha_file(self.plan_path) != self.plan_sha:
                raise StorageControlChanged('mount or plan contents changed')
            self.device = device
            self.next_integrity_check = time.monotonic() + 1.0
        control = json.loads((self.root / 'maintenance/storage_control.json').read_text())
        config = json.loads((self.root / 'grasp_records/archive_config.json').read_text())
        if (control['state'] != 'resumed' or control['archive_policy'] != 'local_only'
                or config['archive_policy'] != 'local_only'
                or config['upload_enabled'] is not False or config['delete_enabled'] is not False):
            raise StorageControlChanged('local storage maintenance authorization changed')

    def path(self, relative):
        p = Path(relative)
        if p.is_absolute() or not p.parts or any(x in ('..', '.') for x in p.parts):
            raise ValueError('invalid cleanup path')
        path = self.root / p
        if path.resolve() != path or any(q.is_symlink() for q in (path, *path.parents)):
            raise ValueError('noncanonical cleanup path')
        for name in self.plan['excluded_prefixes']:
            excluded = self.root / name
            if path == excluded or excluded in path.parents:
                raise ValueError('protected cleanup path: ' + relative)
        assert self.root in path.parents and self.run not in path.parents
        return path

    def check_fd(self, descriptor, path, row):
        info = os.fstat(descriptor)
        assert stat.S_ISREG(info.st_mode) and info.st_nlink == 1
        assert identity(info) == row['identity'] == identity(path.lstat())
        assert info.st_dev == self.root.stat().st_dev and not self.broken
        assert fcntl.fcntl(descriptor, fcntl.F_GETLEASE) == fcntl.F_WRLCK

    def open_leased(self, path, row):
        self.broken = False
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW)
        try:
            fcntl.fcntl(fd, fcntl.F_SETOWN, os.getpid())
            fcntl.fcntl(fd, fcntl.F_SETLEASE, fcntl.F_WRLCK)
            self.check_fd(fd, path, row)
            return fd
        except BaseException:
            os.close(fd)
            raise

    def snapshot_text(self, row):
        self.mounted()
        path = self.path(row['path'])
        prior = self.store.disposition(row['path'])
        if not path.exists() and prior and prior['state'] == 'deleted':
            self.store.verify(row['path'])
            return
        fd = self.open_leased(path, row)
        try:
            try:
                previous = self.store.info(row['path'])
            except KeyError:
                previous = None
            if previous and previous['metadata'].get('identity') == row['identity']:
                self.store.verify(row['path'])
            else:
                blocks = iter(lambda: os.read(fd, CHUNK), b'')
                self.store.add_stream(row['path'], blocks, dict(kind='exact_source', identity=row['identity']))
            self.check_fd(fd, path, row)
        finally:
            os.close(fd)
        self.progress['text_packed'] += 1

    def unlink_verified(self, row, fd, reference, original_sha):
        path = self.path(row['path'])
        self.mounted()
        self.check_fd(fd, path, row)
        self.store.verify(reference)
        receipt = dict(path=row['path'], original_sha256=original_sha,
                       original_identity=row['identity'], allocated_bytes=row['allocated_bytes'],
                       retained_in=str(self.store.path), trace_reference=reference,
                       original_result=row.get('original_result', 'not_classified'),
                       full_raw_recoverable=row['kind'] != 'raw_bag', intent_ns=time.time_ns())
        self.store.journal(row['path'], 'intent', receipt)
        try:
            self.check_fd(fd, path, row)
            path.unlink()
            fsync_dir(path.parent)
            receipt['deleted_ns'] = time.time_ns()
            self.store.journal(row['path'], 'deleted', receipt)
        except Exception as error:
            raise DeletionIncomplete('unresolved deletion intent: ' + row['path']) from error
        self.progress['deleted_allocated_bytes'] += row['allocated_bytes']

    def delete_text(self, row):
        self.mounted()
        path = self.path(row['path'])
        if not path.exists() and self.store.disposition(row['path']):
            return
        fd = self.open_leased(path, row)
        try:
            h = hashlib.sha256()
            for block in iter(lambda: os.read(fd, CHUNK), b''):
                if self.broken:
                    raise RuntimeError('source opened by another process')
                h.update(block)
            assert h.hexdigest() == self.store.info(row['path'])['sha256']
            self.unlink_verified(row, fd, row['path'], h.hexdigest())
        finally:
            os.close(fd)
        self.progress['text_deleted'] += 1

    def delete_raw(self, row):
        self.mounted()
        path = self.path(row['path'])
        assert path.suffix == '.bag' and row['indexed_closed'] is True and row['evidence_paths']
        if not path.exists() and self.store.disposition(row['path']):
            return
        # A closure flag is retained exactly; it is never converted to a success.
        for name in row['evidence_paths']:
            if (self.root / name).exists():
                assert sha_file(self.root / name) == self.store.info(name)['sha256']
            self.store.verify(name)
        fd = self.open_leased(path, row)
        try:
            h = hashlib.sha256()
            for block in iter(lambda: os.read(fd, 8 * CHUNK), b''):
                if self.broken:
                    raise RuntimeError('raw recording requested by another process')
                h.update(block)
            source_sha = h.hexdigest()
            if row.get('expected_sha256'):
                assert source_sha == row['expected_sha256']
            self.check_fd(fd, path, row)
            os.lseek(fd, 0, os.SEEK_SET)
            reference = '@ros_trace/' + row['path'] + '.jsonl'
            counts = {}
            stream = os.fdopen(os.dup(fd), 'rb')
            try:
                self.store.add_stream(reference, bag_trace_blocks(stream, counts), dict(
                    kind='sparse_ros_trace', original_path=row['path'], original_sha256=source_sha,
                    original_identity=row['identity'], original_result=row['original_result'],
                    full_raw_recoverable=False, all_non_visual_messages_retained=True))
            finally:
                if not stream.closed:
                    stream.close()
            self.check_fd(fd, path, row)
            self.unlink_verified(row, fd, reference, source_sha)
        finally:
            os.close(fd)
        self.progress['raw_deleted'] += 1
        print(json.dumps(dict(raw_deleted=row['path'], counts=counts,
                              deleted_allocated_bytes=self.progress['deleted_allocated_bytes']), ensure_ascii=False), flush=True)

    def report(self, state=None):
        if state:
            self.progress['state'] = state
        if state in ('complete', 'stopped'):
            self.progress.update(self.store.deletion_totals())
        values = os.statvfs(self.root)
        self.progress.update(free_bytes=values.f_bavail * values.f_frsize,
                             trace=self.store.stats(), updated_ns=time.time_ns())
        atomic(self.run / 'progress.json', self.progress)

    def run_cleanup(self):
        self.mounted()
        assert self.store.db.execute("SELECT count(*) FROM deletions WHERE state='intent'").fetchone()[0] == 0, 'unresolved deletion intent requires review'
        atomic(self.run / 'execution_source.json', dict(tool_sha256=sha_file(Path(__file__)),
               plan_sha256=self.plan_sha, retention_profile='minimal_trace_local'))
        old_handler = signal.signal(signal.SIGIO, lambda *_: setattr(self, 'broken', True))
        try:
            for i, row in enumerate(self.plan['text_files']):
                try:
                    self.snapshot_text(row)
                except (BlockingIOError, AssertionError, OSError, RuntimeError) as error:
                    self.progress['skipped'].append(dict(path=row['path'], phase='pack', reason=repr(error)))
                if i % 100 == 0:
                    self.report()
                    print(json.dumps(dict(phase='packing', complete=i + 1, total=len(self.plan['text_files'])), ensure_ascii=False), flush=True)
            self.store.verify()
            self.report('removing_verified_text_duplicates')
            for i, row in enumerate(self.plan['text_files']):
                if not row['remove_original']:
                    continue
                try:
                    self.store.info(row['path'])
                    self.delete_text(row)
                except (BlockingIOError, AssertionError, OSError, RuntimeError, KeyError) as error:
                    self.progress['skipped'].append(dict(path=row['path'], phase='text_cleanup', reason=repr(error)))
                if i % 100 == 0:
                    self.report()
                    print(json.dumps(dict(phase='text_cleanup', complete=i + 1, total=len(self.plan['text_files'])), ensure_ascii=False), flush=True)
            self.report('retaining_ros_trace_before_raw_cleanup')
            for row in self.plan['raw_files']:
                values = os.statvfs(self.root)
                if values.f_bavail * values.f_frsize >= self.plan['target_free_bytes']:
                    break
                try:
                    self.delete_raw(row)
                except (BlockingIOError, AssertionError, OSError, RuntimeError, KeyError, ValueError) as error:
                    self.progress['skipped'].append(dict(path=row['path'], phase='raw_cleanup', reason=repr(error)))
                self.report()
            self.store.verify()
            assert self.store.db.execute("SELECT count(*) FROM deletions WHERE state='intent'").fetchone()[0] == 0, 'unresolved deletion intent requires review'
            values = os.statvfs(self.root)
            self.progress['stop_reason'] = ('target_reached' if values.f_bavail * values.f_frsize >= self.plan['target_free_bytes']
                                            else 'eligible_exhausted')
            self.report('complete')
            atomic(self.run / 'receipt.json', self.progress)
            return self.progress
        except BaseException as error:
            self.progress['error'] = repr(error)
            self.report('stopped')
            raise
        finally:
            signal.signal(signal.SIGIO, old_handler)
            self.store.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('apply')
    p.add_argument('run', type=Path)
    p = sub.add_parser('verify')
    p.add_argument('database', type=Path)
    p = sub.add_parser('read')
    p.add_argument('database', type=Path)
    p.add_argument('original_path')
    args = parser.parse_args()
    if args.command == 'apply':
        assert args.run.parent == ROOT / 'maintenance' and args.run.name.startswith('minimal_trace_cleanup_')
        print(json.dumps(Cleanup(ROOT, args.run).run_cleanup(), ensure_ascii=False), flush=True)
    else:
        store = TraceStore(args.database, readonly=True)
        try:
            if args.command == 'verify':
                print(json.dumps(dict(verified_files=store.verify(), **store.stats())))
            else:
                # Validate before emitting any recovered bytes.
                store.verify(args.original_path)
                for block in store.iter_bytes(args.original_path):
                    sys.stdout.buffer.write(block)
        finally:
            store.close()


if __name__ == '__main__':
    main()
