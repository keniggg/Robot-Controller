#!/usr/bin/env python3
"""Append-only-source mirror of open ROS text logs; never starts or controls ROS."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import sys
import time
import uuid

SOURCE_ROOT = Path('/home/zhuyupei/alicia_wa_full/.ros_log')
MOUNT_POINT = Path('/media/zhuyupei/SanStand')
DESTINATION_ROOT = MOUNT_POINT / '实验记录/Robot-Controller/ros_log'
LOG_NAME = re.compile(r'.*\.(?:log(?:\.\d+)?(?:\.gz)?|txt|jsonl|out)$')
BLOCK = 1024 * 1024
TAIL = 65536


class MirrorError(RuntimeError):
    pass


def identity(info):
    return [info.st_dev, info.st_ino]


def fingerprint(info):
    return [info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns, info.st_ctime_ns]


def relative_log(path, root):
    try:
        relative = Path(path).relative_to(root)
    except ValueError:
        return None
    if '..' in relative.parts or any(p.startswith('.') for p in relative.parts):
        return None
    if not LOG_NAME.fullmatch(relative.name):
        return None
    return str(relative)


def writable_fds(proc_root=Path('/proc'), uid=None):
    """Return same-user regular files opened with WRONLY/RDWR, including renamed FDs."""
    uid = os.getuid() if uid is None else uid
    result = []
    for process in proc_root.iterdir():
        if not process.name.isdigit():
            continue
        try:
            if process.stat().st_uid != uid:
                continue
            for fd in (process / 'fd').iterdir():
                try:
                    flags = next(line.split()[1] for line in
                                 (process / 'fdinfo' / fd.name).read_text().splitlines()
                                 if line.startswith('flags:'))
                    if (int(flags, 8) & os.O_ACCMODE) not in (os.O_WRONLY, os.O_RDWR):
                        continue
                    info = fd.stat()
                    if stat.S_ISREG(info.st_mode):
                        result.append({'path': os.readlink(fd), 'reader': str(fd),
                                       'identity': identity(info)})
                except (OSError, StopIteration, ValueError):
                    continue
        except OSError:
            continue
    return result


class ActiveLogMirror:
    def __init__(self, source=SOURCE_ROOT, destination=DESTINATION_ROOT,
                 mount=MOUNT_POINT, excludes=(), mount_check=os.path.ismount):
        self.source = Path(source).absolute()
        self.destination = Path(destination).absolute()
        self.mount = Path(mount).absolute()
        self.excludes = tuple(str(Path(p)) for p in excludes)
        if any(Path(p).is_absolute() or '..' in Path(p).parts for p in self.excludes):
            raise MirrorError('--exclude must be a relative file or directory')
        self.mount_check = mount_check
        self.handles = {}
        self.state_path = self.destination / '.active_ros_log_mirror.json'
        self.state = {'version': 1, 'source': str(self.source), 'files': {}}
        self._storage()
        if self.state_path.exists():
            self.state = json.loads(self.state_path.read_text())
            if self.state.get('version') != 1 or self.state.get('source') != str(self.source):
                raise MirrorError('mirror state belongs to another source/version')

    def _storage(self):
        if not self.mount_check(str(self.mount)):
            raise MirrorError('EXPERIMENT_STORAGE_UNAVAILABLE: mount is not mounted: ' + str(self.mount))
        if self.mount not in self.destination.parents:
            raise MirrorError('destination must be beneath the required mount point')
        for path in (self.destination, *self.destination.parents):
            if path.is_symlink():
                raise MirrorError('symlink in destination path: ' + str(path))
            if path == self.mount:
                break
        self.destination.mkdir(parents=True, exist_ok=True)
        if self.destination.stat().st_dev != self.mount.stat().st_dev:
            raise MirrorError('destination is not on the required mounted filesystem')
        if not self.source.is_dir():
            raise MirrorError('source directory is unavailable')

    def _excluded(self, relative):
        return any(relative == p or relative.startswith(p.rstrip('/') + '/')
                   for p in self.excludes)

    def _destination(self, relative):
        if relative_log(self.source / relative, self.source) != relative:
            raise MirrorError('invalid log path: ' + relative)
        path = self.destination / relative
        if any(p.is_symlink() for p in (path, *path.parents)):
            raise MirrorError('symlink in mirrored log path: ' + relative)
        path.parent.mkdir(parents=True, exist_ok=True)
        return path

    def _save(self):
        self._storage()
        temp = self.state_path.with_name(self.state_path.name + '.' + uuid.uuid4().hex + '.tmp')
        with temp.open('x') as stream:
            json.dump(self.state, stream, ensure_ascii=False, indent=2)
            stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
        os.replace(temp, self.state_path)

    def _open(self, reader, relative, expected, source_relative=None):
        fd = os.open(reader, os.O_RDONLY)
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or identity(info) != expected:
            os.close(fd)
            raise MirrorError('source changed while opening: ' + relative)
        self.handles[tuple(expected)] = {
            'fd': fd, 'relative': relative, 'source_relative': source_relative or relative,
            'rotated': source_relative is not None and source_relative != relative}

    def _prefix_equal(self, fd, target, length):
        with target.open('rb') as stream:
            offset = 0
            while offset < length:
                amount = min(BLOCK, length - offset)
                if stream.read(amount) != os.pread(fd, amount, offset):
                    return False
                offset += amount
        return True

    def _tail(self, fd, offset):
        return hashlib.sha256(os.pread(fd, min(TAIL, offset), max(0, offset - TAIL))).hexdigest()

    def _recover_migrated_source(self, relative, previous, busy_destinations):
        """Accept a closed-source cleanup only with its recorded full-content proof."""
        receipt_path = self.destination.parent / 'system_disk_migration_cleanup.json'
        if not receipt_path.is_file() or receipt_path.is_symlink():
            return False
        receipts = json.loads(receipt_path.read_text()).get('files', {})
        source = self.source / previous.get('source_relative', relative)
        receipt = receipts.get(str(source), {})
        target = self._destination(relative)
        if (relative in busy_destinations or receipt.get('state') != 'removed'
                or receipt.get('source_path') != str(source)
                or receipt.get('destination_path') != str(target)
                or receipt.get('source_identity', [])[:2] != previous['source_identity']
                or receipt.get('size_bytes') != previous['offset']
                or not target.is_file()):
            return False
        before = target.stat()
        if not stat.S_ISREG(before.st_mode) or before.st_nlink != 1 or before.st_size != previous['offset']:
            return False
        digest = hashlib.sha256()
        with target.open('rb') as stream:
            for block in iter(lambda: stream.read(BLOCK), b''):
                digest.update(block)
        if (digest.hexdigest() != receipt.get('sha256')
                or fingerprint(target.stat()) != fingerprint(before)):
            return False
        previous['closed'] = True
        previous['destination_fingerprint'] = fingerprint(before)
        previous['recovered_closed_source'] = {
            'receipt_path': str(receipt_path), 'sha256': digest.hexdigest(),
            'verified_unix': time.time()}
        self._save()
        return True

    def _reconcile_destination_identity(self, fd, relative, target, previous):
        """A remount may change inode numbers; rebind only after a complete comparison."""
        before = target.stat()
        source_info = os.fstat(fd)
        if (not stat.S_ISREG(before.st_mode) or before.st_nlink != 1
                or before.st_size != previous['offset']
                or identity(source_info) != previous['source_identity']
                or source_info.st_size < previous['offset']
                or self._tail(fd, previous['offset']) != previous['source_tail_sha256']
                or not self._prefix_equal(fd, target, previous['offset'])
                or fingerprint(target.stat()) != fingerprint(before)):
            return False
        self.state.setdefault('destination_rebindings', []).append({
            'path': relative, 'previous_fingerprint': previous['destination_fingerprint'],
            'verified_fingerprint': fingerprint(before), 'bytes_compared': previous['offset'],
            'verified_unix': time.time()})
        previous['destination_fingerprint'] = fingerprint(before)
        return True

    def _copy(self, handle, relative, busy_destinations, closed):
        self._storage()
        if relative in busy_destinations:
            raise MirrorError('destination is currently being written by another process: ' + relative)
        target = self._destination(relative)
        fd = handle['fd']; info = os.fstat(fd); size = info.st_size
        previous = self.state['files'].get(relative)
        if target.exists() and previous and fingerprint(target.stat()) != previous['destination_fingerprint']:
            if not self._reconcile_destination_identity(fd, relative, target, previous):
                raise MirrorError('destination changed externally: ' + relative)
        if previous and not target.exists():
            raise MirrorError('managed destination disappeared: ' + relative)
        changed = previous and (previous['source_identity'] != identity(info)
                    or size < previous['offset']
                    or self._tail(fd, previous['offset']) != previous['source_tail_sha256'])
        if changed:
            archive = self.destination / '.mirror_rotations' / relative
            archive = archive.with_name(archive.name + '.' + uuid.uuid4().hex)
            archive.parent.mkdir(parents=True, exist_ok=True)
            # Keep every byte from the previous generation; never truncate a log.
            os.rename(target, archive)
            self.state.setdefault('rotations', []).append({
                'path': relative, 'archive': str(archive.relative_to(self.destination)),
                'source_identity': previous['source_identity'], 'bytes': previous['offset']})
            previous = None
        if previous:
            offset = previous['offset']
        elif target.exists():
            target_info = target.stat()
            if not stat.S_ISREG(target_info.st_mode) or target_info.st_nlink != 1:
                raise MirrorError('existing destination is not a private regular file: ' + relative)
            if target_info.st_size > size or not self._prefix_equal(fd, target, target_info.st_size):
                if handle.get('rotated'):
                    alias = handle['source_relative'] + '.generation.' + str(info.st_ino) + '.log'
                    if alias != relative:
                        handle['relative'] = alias
                        self.state.setdefault('aliases', {})[alias] = {
                            'source_path': handle['source_relative'],
                            'reason': 'preserve existing unowned rotated destination'}
                        return self._copy(handle, alias, busy_destinations, closed)
                raise MirrorError('existing destination differs from source: ' + relative)
            if fingerprint(target.stat()) != fingerprint(target_info):
                raise MirrorError('destination changed while adopting rsync copy: ' + relative)
            offset = target_info.st_size
        else:
            offset = 0
        start = offset
        expected_target = fingerprint(target.stat()) if target.exists() else None
        flags = os.O_WRONLY | os.O_CREAT | os.O_APPEND | os.O_NOFOLLOW
        output = os.open(target, flags, 0o600)
        try:
            fcntl.flock(output, fcntl.LOCK_EX | fcntl.LOCK_NB)
            opened_target = os.fstat(output)
            if opened_target.st_size != offset or (expected_target is not None
                    and fingerprint(opened_target) != expected_target):
                raise MirrorError('destination changed before append: ' + relative)
            while offset < size:
                self._storage()
                block = os.pread(fd, min(BLOCK, size - offset), offset)
                if not block:
                    raise MirrorError('source truncated while copying: ' + relative)
                view = memoryview(block)
                while view:
                    written = os.write(output, view); view = view[written:]
                offset += len(block)
            os.fsync(output)
            target_info = os.fstat(output)
            if target_info.st_size != offset:
                raise MirrorError('concurrent destination write detected: ' + relative)
        finally:
            os.close(output)
        if fingerprint(target.stat()) != fingerprint(target_info):
            raise MirrorError('destination path changed during append: ' + relative)
        self.state['files'][relative] = {
            'source_identity': identity(info), 'source_relative': handle['source_relative'], 'offset': offset,
            'source_tail_sha256': self._tail(fd, offset),
            'destination_fingerprint': fingerprint(target_info), 'closed': closed,
            'last_copy_unix': time.time()}
        self._save()
        return offset - start

    def run_once(self, open_files=None):
        self._storage()
        open_files = writable_fds() if open_files is None else open_files
        active = set(); busy = set()
        for item in open_files:
            path = item['path'][:-10] if item['path'].endswith(' (deleted)') else item['path']
            dest_rel = relative_log(path, self.destination)
            if dest_rel is not None:
                busy.add(dest_rel)
            relative = relative_log(path, self.source)
            if relative is None or self._excluded(relative):
                continue
            key = tuple(item['identity']); active.add(key)
            if key not in self.handles:
                known = next(((name, value) for name, value in self.state['files'].items()
                              if value['source_identity'] == list(key) and not value['closed']), None)
                if known:
                    self._open(item['reader'], known[0], item['identity'],
                               known[1].get('source_relative', known[0]))
                else:
                    self._open(item['reader'], relative, item['identity'])
        # Recover unfinished sources after restarting the mirror, including ordinary rename rotation.
        for relative, previous in list(self.state['files'].items()):
            key = tuple(previous['source_identity'])
            if previous['closed'] or key in self.handles or self._excluded(relative):
                continue
            original = self.source / previous.get('source_relative', relative)
            candidates = [original]
            if original.parent.is_dir():
                candidates.extend(original.parent.iterdir())
            for candidate in candidates:
                try:
                    candidate_rel = relative_log(candidate, self.source)
                    if candidate_rel and not candidate.is_symlink() and identity(candidate.stat()) == list(key):
                        self._open(str(candidate), relative, list(key),
                                   previous.get('source_relative', relative))
                        break
                except OSError:
                    continue
            else:
                if not self._recover_migrated_source(relative, previous, busy):
                    raise MirrorError('previous source unavailable for final flush: ' + relative)
        copied = 0; files = 0
        # Finish closed/deleted generations before copying a new file at the same name.
        for key, handle in sorted(list(self.handles.items()), key=lambda pair: (
                pair[0] in active and not os.readlink('/proc/self/fd/' + str(pair[1]['fd'])).endswith(' (deleted)'))):
            link = os.readlink('/proc/self/fd/' + str(handle['fd']))
            old_relative = handle['relative']
            deleted = link.endswith(' (deleted)')
            actual_relative = relative_log(link[:-10] if deleted else link, self.source)
            if actual_relative is not None and (actual_relative != handle['source_relative']
                                                or (deleted and not handle.get('unlinked'))):
                handle['source_relative'] = actual_relative
                handle['relative'] = (actual_relative + '.unlinked.' + str(key[1]) + '.log'
                                      if deleted else actual_relative)
                handle['rotated'] = True
                handle['unlinked'] = deleted
            if old_relative != handle['relative']:
                old_state = self.state['files'].get(old_relative)
                if old_state and old_state['source_identity'] == list(key):
                    old_state['closed'] = True
            relative = handle['relative']; closed = key not in active
            if not self._excluded(relative):
                copied += self._copy(handle, relative, busy, closed); files += 1
            if closed:
                os.close(handle['fd']); del self.handles[key]
        return {'files': files, 'bytes_copied': copied, 'active_files': len(active)}

    def close(self):
        for handle in self.handles.values():
            os.close(handle['fd'])
        self.handles.clear()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--destination', type=Path, default=DESTINATION_ROOT)
    parser.add_argument('--mount', type=Path, default=MOUNT_POINT)
    parser.add_argument('--exclude', action='append', default=[], help='relative file/directory to leave untouched')
    parser.add_argument('--follow', action='store_true')
    parser.add_argument('--interval', type=float, choices=(2.0, 5.0), default=5.0)
    args = parser.parse_args(argv)
    mirror = None
    try:
        mirror = ActiveLogMirror(destination=args.destination, mount=args.mount, excludes=args.exclude)
        lock_path = mirror.destination / '.active_ros_log_mirror.lock'
        with lock_path.open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            while True:
                print(json.dumps(mirror.run_once(), ensure_ascii=False), flush=True)
                if not args.follow:
                    break
                time.sleep(args.interval)
        return 0
    except (OSError, ValueError, MirrorError) as exc:
        print('ACTIVE_LOG_MIRROR_ERROR: ' + str(exc), file=sys.stderr, flush=True)
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        if mirror is not None:
            mirror.close()


if __name__ == '__main__':
    raise SystemExit(main())
