#!/usr/bin/env python3
"""Record an optional lossless RGB-D sidecar within a reserved disk budget.

Only the directly owned rosbag writer receives SIGINT. No controller commands,
record deletion, rollover, overwrite, forced kill, or automatic retry. This
optional sidecar may end before the task; the main keyframe recorder continues.
Stop through output/stop_requested or SIGINT to this observer. Do not use SIGTERM
to the observer and assume that it finalized the separate writer session.
"""
import argparse
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import stat
import struct
import subprocess
import time

MIB = 1 << 20
GIB = 1 << 30
REVIEWED_ARCHIVE_MINIMUM = 9 * GIB // 2
MOUNT = Path('/media/zhuyupei/SanStand')
ROOT = MOUNT / '实验记录/Robot-Controller'
WRITER = '/opt/ros/noetic/lib/rosbag/record'
TOPICS = (
    '/supervisor/camera/color/image_raw', '/supervisor/camera/depth/image_raw',
    '/tf', '/tf_static', '/alicia_d/accepted_joint_states',
    '/alicia_d/control_reference_epoch', '/gui/joint_direct_mode',
    '/grasp_mode/selection', '/grasp_mode/status', '/grasp/state',
    '/grasp/near_field_phase', '/perception/unknown/detector_status',
    '/perception/unknown/object', '/perception/unknown/object_mask',
    '/grasp_mode/observe_return_measurement_request',
    '/grasp_mode/observe_return_measurement_ack', '/grasp_6d/preview_plan_enriched',
    '/alicia_controller/follow_joint_trajectory/status',
    '/alicia_controller/follow_joint_trajectory/goal',
    '/alicia_controller/follow_joint_trajectory/result',
    '/hand_controller/follow_joint_trajectory/status',
    '/hand_controller/follow_joint_trajectory/goal',
    '/hand_controller/follow_joint_trajectory/result',
)


@dataclass(frozen=True)
class Limits:
    duration: int = 600
    max_bytes: int = GIB
    min_free_bytes: int = 6 * GIB
    buffer_mib: int = 256
    closing_reserve_bytes: int = 32 * MIB
    poll_sec: float = 0.2
    graceful_timeout_sec: float = 30.0
    finalize_timeout_sec: float = 30.0

    @property
    def headroom(self):
        return self.buffer_mib * MIB + self.closing_reserve_bytes

    def validate(self, archive_minimum):
        if not 1 <= self.duration <= 600:
            raise ValueError('duration must be in [1, 600] seconds')
        if self.buffer_mib != 256 or self.closing_reserve_bytes < 32 * MIB:
            raise ValueError('reviewed 256 MiB buffer and at least 32 MiB closing reserve required')
        if type(self.max_bytes) is not int or self.max_bytes < self.headroom + MIB:
            raise ValueError('sidecar budget must include buffer, closing reserve and one MiB of data')
        if type(archive_minimum) is not int or archive_minimum < 0:
            raise ValueError('invalid archive minimum free bytes')
        if type(self.min_free_bytes) is not int or self.min_free_bytes < max(
                REVIEWED_ARCHIVE_MINIMUM, archive_minimum):
            raise ValueError('sidecar free-space floor cannot be below archive configuration')
        if not 0 < self.poll_sec <= 0.2:
            raise ValueError('disk polling interval must be in (0, 0.2]')
        if not 0 < self.graceful_timeout_sec <= 60 or not 0 < self.finalize_timeout_sec <= 60:
            raise ValueError('graceful/finalize timeout must be in (0, 60] seconds')


def trigger(limits, free_bytes, written_bytes):
    if written_bytes + limits.headroom >= limits.max_bytes:
        return 'sidecar_bytes'
    if free_bytes <= limits.min_free_bytes + limits.headroom:
        return 'free_space'
    return None


class Storage:
    def __init__(self, output):
        self.output = Path(output)
        if self.output.is_symlink() or self.output.resolve() != self.output.absolute():
            raise ValueError('canonical output path required')
        self.output = self.output.resolve()
        if (ROOT / 'ros_log').resolve() not in self.output.parents:
            raise ValueError('output must be a new SanStand ros_log subdirectory')
        self.device = None

    def check(self):
        if self.output.is_symlink() or self.output.resolve() != self.output:
            raise RuntimeError('canonical output path changed')
        if not MOUNT.is_mount() or os.statvfs(MOUNT).f_flag & os.ST_RDONLY:
            raise RuntimeError('SanStand unavailable or read-only; no fallback')
        device = MOUNT.stat().st_dev
        parent = self.output if self.output.exists() else self.output.parent
        if not parent.is_dir() or parent.stat().st_dev != device:
            raise RuntimeError('output parent is not on SanStand')
        if self.device is not None and self.device != device:
            raise RuntimeError('SanStand device changed')
        self.device = device
        archive = json.loads((ROOT / 'grasp_records/archive_config.json').read_text())
        if not isinstance(archive, dict) or type(archive.get('min_free_bytes')) is not int:
            raise RuntimeError('archive configuration schema invalid')
        if archive.get('upload_enabled') is not False or archive.get('delete_enabled') is not False:
            raise RuntimeError('local-only archive upload/delete policy changed')
        if archive.get('archive_policy') != 'local_only':
            raise RuntimeError('archive policy is not local_only')
        return shutil.disk_usage(ROOT).free, archive['min_free_bytes']


def writer_command(output, limits):
    # Invoke the actual writer, avoiding a rosbag CLI wrapper that can exit
    # before its child completes finalization. No --split/--max-splits deletion.
    # ros_comm 1.17.4 record.cpp parses bare min-space in bytes and size in MiB;
    # recorder.cpp checkSize()/doRecord() close without rollover when split=false.
    return [WRITER, '--lz4', '--duration=' + str(limits.duration),
            '--buffsize=' + str(limits.buffer_mib), '--chunksize=768',
            '--size=' + str((limits.max_bytes - limits.headroom) // MIB),
            '--min-space=' + str(limits.min_free_bytes),
            '-O', str(output / 'full_frames.bag'), *TOPICS]


def measured_size(output):
    total = 0
    for path in (output / 'full_frames.bag', output / 'full_frames.bag.active'):
        try:
            value = path.lstat()
        except FileNotFoundError:
            continue
        if not stat.S_ISREG(value.st_mode) or value.st_nlink != 1:
            raise RuntimeError('recording path is not an ordinary owned file')
        total += value.st_size
    return total


def _record_header(stream):
    def exact(size):
        value = stream.read(size)
        if len(value) != size:
            raise ValueError('incomplete bag index')
        return value
    length = struct.unpack('<I', exact(4))[0]
    if length > 16 * MIB:
        raise ValueError('unreasonable bag header')
    payload = exact(length)
    fields, offset = {}, 0
    while offset < length:
        size = struct.unpack('<I', payload[offset:offset + 4])[0]
        offset += 4
        if size == 0 or offset + size > length:
            raise ValueError('invalid bag header field')
        key, value = payload[offset:offset + size].split(b'=', 1)
        fields[key] = value
        offset += size
    return fields, struct.unpack('<I', exact(4))[0]


def coverage(path):
    """Read only the closed V2 bag index; no ROS imports or message replay."""
    def stamp(value):
        seconds, nanos = struct.unpack('<II', value)
        if nanos >= 1000000000:
            raise ValueError('invalid bag timestamp')
        return seconds * 1000000000 + nanos
    with path.open('rb') as stream:
        if stream.readline() != b'#ROSBAG V2.0\n':
            raise ValueError('not a ROS V2 bag')
        header, _ = _record_header(stream)
        if header.get(b'op') != b'\x03':
            raise ValueError('missing bag file header')
        index = struct.unpack('<Q', header[b'index_pos'])[0]
        count = struct.unpack('<I', header[b'chunk_count'])[0]
        connections = struct.unpack('<I', header[b'conn_count'])[0]
        if not 0 < index <= path.stat().st_size:
            raise ValueError('bag is not indexed/closed')
        stream.seek(index)
        for _ in range(connections):
            item, size = _record_header(stream)
            if item.get(b'op') != b'\x07':
                raise ValueError('invalid connection index')
            stream.seek(size, 1)
        starts, ends, messages = [], [], 0
        for _ in range(count):
            item, size = _record_header(stream)
            if item.get(b'op') != b'\x06':
                raise ValueError('invalid chunk index')
            starts.append(stamp(item[b'start_time']))
            ends.append(stamp(item[b'end_time']))
            entries = struct.unpack('<I', item[b'count'])[0]
            if size != entries * 8:
                raise ValueError('invalid connection message counts')
            for _ in range(entries):
                _, number = struct.unpack('<II', stream.read(8))
                messages += number
        return dict(coverage_start_ns=min(starts) if starts else None,
                    coverage_end_ns=max(ends) if ends else None,
                    chunk_count=count, message_count=messages,
                    coverage_semantics='bag message receipt times; not a claim of complete task coverage')


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(8 * MIB), b''):
            digest.update(chunk)
    return digest.hexdigest()


def record(output, limits, check_storage, *, popen=subprocess.Popen,
           monotonic=time.monotonic, sleep=time.sleep, now_ns=time.time_ns):
    output = Path(output)
    free, archive_floor = check_storage()
    limits.validate(archive_floor)
    if trigger(limits, free, 0):
        raise RuntimeError('insufficient sidecar capacity; keep main recorder independent')
    output.mkdir(exist_ok=False)
    command = writer_command(output, limits)
    result = dict(schema=1, recording_closed=False, full_frame_sidecar_only=True,
                  motion_commands_sent=False, temperature_queried=False,
                  files_deleted=False, main_recorder_controlled=False,
                  command=command, topics=list(TOPICS), compression='lz4',
                  started_ns=now_ns(), duration_limit_sec=limits.duration,
                  max_sidecar_bytes=limits.max_bytes, min_free_bytes=limits.min_free_bytes,
                  archive_min_free_bytes=archive_floor, buffer_reserve_bytes=limits.headroom,
                  stop_file_size_bytes=limits.max_bytes-limits.headroom,
                  stop_free_bytes=limits.min_free_bytes+limits.headroom,
                  initial_free_bytes=free, signals_sent=[],
                  budget_is_filesystem_quota=False,
                  budget_scope='early stop with buffer/closing headroom; actual final size verified',
                  stop_reason=None, coverage_end_ns=None)
    process = None

    def save():
        check_storage()
        temporary = output / ('result.' + str(os.getpid()) + '.tmp')
        with temporary.open('x') as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(output / 'result.json')

    def try_save():
        try:
            save()
        except Exception as exc:
            result['receipt_write_error'] = repr(exc)

    def request_stop(reason):
        if process is None or process.poll() is not None or result.get('signal_attempted'):
            return
        result.update(stop_reason=reason, stop_requested_ns=now_ns(), signal_attempted=True)
        # A failed or unavailable receipt must never delay the owned writer's stop.
        try:
            process.send_signal(signal.SIGINT)
            result['signals_sent'].append('SIGINT')
        except ProcessLookupError:
            result['writer_exited_before_signal'] = True
        except Exception as exc:
            result['stop_signal_error'] = repr(exc)
        try_save()

    save()
    try:
        with (output / 'recorder.log').open('x') as log:
            environment = dict(os.environ, ROS_LOG_DIR=str(output / 'ros'),
                               PYTHONDONTWRITEBYTECODE='1')
            process = popen(command, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True, env=environment)
            result['pid'] = process.pid
            try:
                result['start_ticks'] = Path('/proc/%d/stat' % process.pid).read_text().rsplit(')', 1)[1].split()[19]
            except FileNotFoundError:
                if process.poll() is None:
                    raise
                result['writer_exited_before_identity_read'] = True
            deadline = monotonic() + limits.duration
            save()
            while process.poll() is None:
                free, archive_floor = check_storage()
                limits.validate(archive_floor)
                written = measured_size(output)
                result.update(last_observed_free_bytes=free, last_observed_file_bytes=written,
                              last_probe_ns=now_ns())
                resource = trigger(limits, free, written)
                if resource:
                    result['budget_trigger'] = resource
                    request_stop('budget_exhausted')
                    break
                if (output / 'stop_requested').exists():
                    request_stop('stop_requested')
                    break
                if monotonic() >= deadline:
                    request_stop('duration_elapsed')
                    break
                sleep(limits.poll_sec)
            result['returncode'] = process.wait(timeout=limits.graceful_timeout_sec)
    except BaseException as exc:
        result['error'] = repr(exc)
        request_stop('observer_interrupted_or_failed')
        if process is not None and process.poll() is None:
            # One graceful wait only. Never terminate/kill an unclosed writer.
            if not isinstance(exc, subprocess.TimeoutExpired):
                try:
                    result['returncode'] = process.wait(timeout=limits.graceful_timeout_sec)
                except subprocess.TimeoutExpired:
                    result['graceful_wait_timed_out'] = True
            else:
                result['graceful_wait_timed_out'] = True
        result['writer_may_still_be_running'] = process is not None and process.poll() is None
        if result['writer_may_still_be_running'] or process is None:
            try_save()
            raise
        # A stopped owned writer can still be sealed after an observer error.
    result['recorder_exited_ns'] = now_ns()
    result['returncode'] = process.poll()
    result['writer_may_still_be_running'] = False
    if result['stop_reason'] is None:
        native_limit = ((limits.max_bytes-limits.headroom) // MIB) * MIB
        if measured_size(output) >= native_limit:
            result.update(stop_reason='budget_exhausted', budget_trigger='native_size_limit_observed')
        else:
            result['stop_reason'] = 'recorder_exited'
    try_save()
    bag = output / 'full_frames.bag'
    active = output / 'full_frames.bag.active'
    close_deadline = monotonic() + limits.finalize_timeout_sec
    try:
        while not (bag.is_file() and not active.exists()):
            if monotonic() >= close_deadline:
                raise TimeoutError('owned writer exited without a finalized bag; preserve partial output')
            sleep(.05)
        value = bag.lstat()
        if not stat.S_ISREG(value.st_mode) or value.st_nlink != 1:
            raise RuntimeError('final bag identity changed')
        details = coverage(bag)
        digest = sha(bag)
        after = bag.lstat()
        if (value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns) != (
                after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns):
            raise RuntimeError('final bag changed during verification')
        free, _ = check_storage()
        result.update(details, recording_closed=True, closed_ns=now_ns(),
                      path=str(bag), size_bytes=value.st_size, sha256=digest,
                      final_free_bytes=free, within_sidecar_budget=value.st_size <= limits.max_bytes,
                      free_floor_preserved=free >= limits.min_free_bytes)
        save()
    except BaseException as exc:
        result['close_verification_error'] = repr(exc)
        try_save()
        raise
    print(json.dumps(result, ensure_ascii=False), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=int, default=600)
    parser.add_argument('--max-sidecar-bytes', type=int, default=GIB)
    parser.add_argument('--min-free-bytes', type=int, default=6 * GIB)
    args = parser.parse_args()
    storage = Storage(args.output)
    limits = Limits(duration=args.duration, max_bytes=args.max_sidecar_bytes,
                    min_free_bytes=args.min_free_bytes)
    result = record(storage.output, limits, storage.check)
    return 0 if (result.get('returncode') == 0 and not result.get('error') and
                 not result.get('stop_signal_error') and
                 result.get('within_sidecar_budget') and result.get('free_floor_preserved')) else 2


if __name__ == '__main__':
    raise SystemExit(main())
