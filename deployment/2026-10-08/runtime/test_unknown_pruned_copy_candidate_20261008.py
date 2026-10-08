"""Differential local Noetic function-body tests; no ROS node or device."""
import ast
import hashlib
import importlib.util
import io
import logging
from pathlib import Path
import struct
import threading
import traceback

import genpy
import pytest
from sensor_msgs.msg import Image

spec = importlib.util.spec_from_file_location('pruned_candidate', Path(__file__).with_name('unknown_pruned_copy_candidate_20261008.py'))
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def original():
    tree = ast.parse(m.ORIGINAL_SOURCE.read_text())
    node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'deserialize_messages')
    scope = dict(struct=struct, logging=logging, traceback=traceback, genpy=genpy)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])),
                 str(m.ORIGINAL_SOURCE), 'exec'), scope)
    return scope['deserialize_messages']


def packet(seq, size=31):
    msg = Image()
    msg.header.seq = seq
    msg.header.stamp = genpy.Time(123, seq)
    msg.header.frame_id = 'camera_link'
    msg.height = 1
    msg.width = size
    msg.encoding = 'mono8'
    msg.step = size
    msg.data = bytes([seq])*size
    b = io.BytesIO()
    msg.serialize(b)
    payload = b.getvalue()
    return struct.pack('<I', len(payload))+payload


def execute(fn, wire, *, queue_size=2, max_msgs=None, start=0, initial=(), cursor=None, data_class=Image):
    b = io.BytesIO(wire)
    b.seek(len(wire) if cursor is None else cursor)
    q = list(initial)
    try:
        value = fn(b, q, data_class, queue_size, max_msgs, start)
        result = ('return', value)
    except Exception as exc:
        result = ('exception', type(exc), str(exc))
    return result, b.getvalue(), b.tell(), q


@pytest.mark.parametrize('queue_size', [1, 2, 9])
@pytest.mark.parametrize('tail', [b'', b'\x03', b'\x03\0\0', struct.pack('<I', 27), packet(9)[:-1]])
def test_complete_and_partial_tail_all_bytes_queue_cursor_identical(queue_size, tail):
    orig = original(); candidate = m.make_candidate(orig, Image)
    wire = b''.join(packet(i) for i in range(1, 6))+tail
    assert execute(orig, wire, queue_size=queue_size, initial=['older']) == execute(
        candidate, wire, queue_size=queue_size, initial=['older'])
    stat = candidate.snapshot_statistics()
    assert stat['optimized_calls'] == 1 and stat['calls_completed'] == 1 and stat['inflight'] == 0
    assert stat['skipped_payload_bytes'] == sum(len(packet(i))-4 for i in range(1, max(1, 6-queue_size)))


@pytest.mark.parametrize('max_msgs', [None, 0, -1, 1, 3, 9])
def test_nonzero_start_max_msgs_and_existing_queue(max_msgs):
    orig = original(); candidate = m.make_candidate(orig, Image)
    wire = b'prefix'+b''.join(packet(i) for i in range(1, 6))+packet(9)[:7]
    assert execute(orig, wire, start=6, max_msgs=max_msgs, initial=['a', 'b', 'c']) == execute(
        candidate, wire, start=6, max_msgs=max_msgs, initial=['a', 'b', 'c'])


@pytest.mark.parametrize('wire', [b'', b'\0', b'\0\0\0', struct.pack('<I', 2**32-1),
                                struct.pack('<I', 0), packet(1)+struct.pack('<I', 0),
                                struct.pack('<I', 0)+packet(1)+packet(2)])
def test_short_headers_zero_length_and_huge_partial_keep_original_errors(wire):
    orig = original(); candidate = m.make_candidate(orig, Image)
    assert execute(orig, wire, initial=['older']) == execute(candidate, wire, initial=['older'])


def test_original_data_class_exception_and_visible_scan_cursor(monkeypatch):
    orig = original(); candidate = m.make_candidate(orig, Image)
    wire = b''.join(packet(i) for i in range(1, 5))+packet(9)[:15]
    base = Image.deserialize
    histories = []
    for fn in (orig, candidate):
        b = io.BytesIO(wire); b.seek(len(wire)); q = ['older']; observed = []
        def deserialize(self, data):
            observed.append(b.tell())
            if struct.unpack_from('<I', data)[0] == 4:
                raise ValueError('retained Image failure')
            return base(self, data)
        monkeypatch.setattr(Image, 'deserialize', deserialize)
        with pytest.raises(genpy.DeserializationError, match='cannot deserialize: retained Image failure'):
            fn(b, q, Image, 2)
        histories.append((b.getvalue(), b.tell(), q, observed))
    assert histories[0] == histories[1]
    assert histories[0][-1] == [sum(len(packet(i)) for i in range(1,5))+4]*2
    assert candidate.snapshot_statistics()['calls_failed'] == 1


def test_deserializer_can_resize_buffer_no_exported_memoryview_remains(monkeypatch):
    orig = original(); candidate = m.make_candidate(orig, Image)
    wire = packet(1)+packet(2)+packet(3)
    base = Image.deserialize
    histories = []
    for fn in (orig, candidate):
        b = io.BytesIO(wire); b.seek(len(wire)); q = []
        def deserialize(self, data):
            b.write(b'!')
            return base(self, data)
        monkeypatch.setattr(Image, 'deserialize', deserialize)
        fn(b, q, Image, 2)
        histories.append((b.getvalue(), b.tell(), q))
    assert histories[0] == histories[1]


def test_payload_copy_accounting_excludes_pruned_messages():
    orig = original(); candidate = m.make_candidate(orig, Image)
    wire = b''.join(packet(i, size=i*11) for i in range(1, 10))
    assert execute(orig, wire) == execute(candidate, wire)
    s = candidate.snapshot_statistics()
    assert s['complete_messages_scanned'] == 9 and s['retained_messages'] == 2
    assert s['scanned_payload_bytes'] == sum(len(packet(i, i*11))-4 for i in range(1,10))
    assert s['materialized_payload_bytes'] == s['retained_payload_bytes'] == sum(len(packet(i, i*11))-4 for i in (8,9))
    assert s['skipped_payload_bytes'] == s['scanned_payload_bytes']-s['materialized_payload_bytes']


@pytest.mark.parametrize('kwargs', [dict(queue_size=None), dict(queue_size=0), dict(queue_size=-1),
                                   dict(queue_size=True), dict(start=-1), dict(start=3.5),
                                   dict(cursor=10000), dict(max_msgs=2.5)])
def test_outside_specialization_delegates_without_changing_semantics(kwargs):
    orig = original(); candidate = m.make_candidate(orig, Image)
    wire = packet(1)+packet(2)+packet(3)
    assert execute(orig, wire, **kwargs) == execute(candidate, wire, **kwargs)
    assert candidate.snapshot_statistics()['fallback_calls'] == 1


def test_non_image_and_bytesio_subclass_delegate():
    orig = original(); candidate = m.make_candidate(orig, Image)
    class Alternate(Image): pass
    assert execute(orig, packet(1), data_class=Alternate) == execute(candidate, packet(1), data_class=Alternate)
    class Buffer(io.BytesIO): pass
    b = Buffer(packet(1)); b.seek(len(b.getvalue())); q = []
    candidate(b, q, Image, 2)
    assert candidate.snapshot_statistics()['fallback_calls'] == 2


def test_frozen_source_hash_and_original_function_body_are_required(monkeypatch):
    orig = original()
    assert hashlib.sha256(m.ORIGINAL_SOURCE.read_bytes()).hexdigest() == m.ORIGINAL_SOURCE_SHA256
    monkeypatch.setattr(m, 'ORIGINAL_SOURCE_SHA256', '0'*64)
    with pytest.raises(ValueError, match='source changed'): m.make_candidate(orig, Image)


def test_deactivate_inflight_and_late_bound_call_statistics(monkeypatch):
    orig = original(); candidate = m.make_candidate(orig, Image)
    entered, release = threading.Event(), threading.Event()
    base = Image.deserialize
    def paused(self, data):
        entered.set(); assert release.wait(2)
        return base(self, data)
    monkeypatch.setattr(Image, 'deserialize', paused)
    results = []
    t = threading.Thread(target=lambda: results.append(execute(candidate, packet(1))))
    t.start(); assert entered.wait(2)
    candidate.deactivate()
    assert not candidate.snapshot_statistics()['active'] and candidate.snapshot_statistics()['inflight'] == 1
    release.set(); t.join(2); assert not t.is_alive()
    final = candidate.snapshot_statistics()
    assert final['inflight'] == 0 and final['calls_started'] == final['calls_completed'] == 1
    assert execute(candidate, packet(2)) == execute(orig, packet(2))
    assert candidate.snapshot_statistics() == final
