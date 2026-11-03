#!/usr/bin/env python3
"""Private offline/diagnostic candidate; never installs or imports ROS itself.

Keep the original read size and message framing. Only avoid materializing
payloads that the original positive queue_size would immediately discard.
"""
import ast
from collections import Counter, deque
import hashlib
import inspect
import io
from pathlib import Path
import struct
import threading

ORIGINAL_SOURCE = Path('/opt/ros/noetic/lib/python3/dist-packages/rospy/msg.py')
ORIGINAL_SOURCE_SHA256 = 'fe4fec18afb214d4836de5500df4bfac432758bc339b93c79c7b30e10f01f7db'
SIGNATURE = ('b', 'msg_queue', 'data_class', 'queue_size', 'max_msgs', 'start')


def _verify_original(original):
    raw = ORIGINAL_SOURCE.read_bytes()
    if hashlib.sha256(raw).hexdigest() != ORIGINAL_SOURCE_SHA256:
        raise ValueError('frozen rospy.msg source changed')
    if (Path(inspect.getsourcefile(original)).resolve() != ORIGINAL_SOURCE or
            tuple(inspect.signature(original).parameters) != SIGNATURE):
        raise ValueError('original deserializer source or signature differs')
    tree = ast.parse(raw, filename=str(ORIGINAL_SOURCE))
    node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'deserialize_messages')
    namespace = {}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])),
                 str(ORIGINAL_SOURCE), 'exec'), namespace)
    expected = namespace['deserialize_messages'].__code__
    actual = original.__code__
    if any(getattr(actual, field) != getattr(expected, field) for field in
           ('co_code', 'co_consts', 'co_names', 'co_varnames')):
        raise ValueError('original deserializer function body differs')
    for name in ('logging', 'traceback', 'genpy'):
        if name not in original.__globals__:
            raise ValueError('original deserializer missing dependency: '+name)


def _eligible(b, msg_queue, data_class, image_class, queue_size, max_msgs, start):
    if not (type(b) is io.BytesIO and type(msg_queue) is list and data_class is image_class and
            type(queue_size) is int and queue_size > 0 and type(start) is int and start >= 0 and
            (max_msgs is None or type(max_msgs) is int)):
        return False
    # Nonstandard cursor/exported buffer states retain original error behavior.
    try:
        end = b.tell()
        view = b.getbuffer()
        try:
            return start <= end <= len(view)
        finally:
            view.release()
    except Exception:
        return False


def _deserialize(b, msg_queue, data_class, queue_size, max_msgs, start, metrics):
    pos = start
    btell = b.tell()
    left = btell - pos
    if left < 4:
        return
    selected = deque()
    count = 0
    cursor = pos
    size = -1
    view = b.getbuffer()
    try:
        # Mirror the original loop, including zero-length and max_msgs behavior.
        while (size < 0 and left >= 4) or (size > -1 and left >= size):
            if size < 0 and left >= 4:
                size = struct.unpack_from('<I', view, cursor)[0]
                cursor += 4
                left -= 4
            if size > -1 and left >= size:
                selected.append((cursor, size))
                if len(selected) > queue_size:
                    selected.popleft()
                metrics['scanned_payload_bytes'] += size
                count += 1
                cursor += size
                pos += size + 4
                left = btell - pos
                size = -1
                if max_msgs and count >= max_msgs:
                    break
    finally:
        # No exported view may survive into reads, custom deserializers or resize.
        view.release()
    metrics['complete_messages_scanned'] += count
    metrics['retained_messages'] += len(selected)
    metrics['retained_payload_bytes'] += sum(length for _, length in selected)
    metrics['skipped_payload_bytes'] += metrics['scanned_payload_bytes'] - metrics['retained_payload_bytes']
    buffs = []
    for offset, length in selected:
        b.seek(offset)
        payload = b.read(length)
        metrics['materialized_payload_bytes'] += len(payload)
        buffs.append(payload)
    # Original builds all payloads before invoking constructors/deserializers;
    # their visible cursor is the end of its scan (possibly a partial header).
    b.seek(cursor)
    for q in buffs:
        data = data_class()
        msg_queue.append(data.deserialize(q))
    del msg_queue[:-queue_size]
    if btell == pos:
        b.seek(start)
        b.truncate(start)
    else:
        if pos != start:
            b.seek(pos)
            leftovers = b.read(btell-pos)
            b.truncate(start + len(leftovers))
            b.seek(start)
            b.write(leftovers)
        else:
            b.seek(btell)


def make_candidate(original, image_class):
    """Return same-signature private callable plus snapshot/deactivate methods.

    No patching occurs. After deactivate, late bound calls delegate directly
    and do not extend the frozen statistics window. In-flight calls still finish.
    """
    _verify_original(original)
    if getattr(image_class, '_type', None) != 'sensor_msgs/Image':
        raise ValueError('exact Image class required')
    lock = threading.Lock()
    active = [True]
    statistics = Counter({key: 0 for key in (
        'calls_started', 'calls_completed', 'calls_failed', 'inflight',
        'optimized_calls', 'fallback_calls', 'scanned_payload_bytes',
        'retained_payload_bytes', 'skipped_payload_bytes', 'materialized_payload_bytes',
        'complete_messages_scanned', 'retained_messages')})
    original_globals = original.__globals__

    def candidate(b, msg_queue, data_class, queue_size=None, max_msgs=None, start=0):
        with lock:
            admitted = active[0]
            if admitted:
                statistics['calls_started'] += 1
                statistics['inflight'] += 1
        if not admitted:
            return original(b, msg_queue, data_class, queue_size, max_msgs, start)
        metrics = Counter()
        failed = True
        try:
            optimized = _eligible(b, msg_queue, data_class, image_class, queue_size, max_msgs, start)
            with lock:
                statistics['optimized_calls' if optimized else 'fallback_calls'] += 1
            if not optimized:
                result = original(b, msg_queue, data_class, queue_size, max_msgs, start)
            else:
                try:
                    result = _deserialize(b, msg_queue, data_class, queue_size, max_msgs, start, metrics)
                except Exception as exc:
                    original_globals['logging'].getLogger('rospy.msg').error(
                        'cannot deserialize message: EXCEPTION %s', original_globals['traceback'].format_exc())
                    raise original_globals['genpy'].DeserializationError('cannot deserialize: %s' % str(exc))
            failed = False
            return result
        finally:
            with lock:
                statistics.update(metrics)
                statistics['calls_failed' if failed else 'calls_completed'] += 1
                statistics['inflight'] -= 1

    def snapshot_statistics():
        with lock:
            return dict(statistics, active=active[0],
                        late_deactivated_calls_delegated_without_statistics=True,
                        original_source_sha256=ORIGINAL_SOURCE_SHA256)

    def deactivate():
        with lock:
            active[0] = False

    candidate.snapshot_statistics = snapshot_statistics
    candidate.deactivate = deactivate
    return candidate
