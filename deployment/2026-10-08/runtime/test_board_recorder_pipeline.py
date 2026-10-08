"""Board recorder concurrency contracts; no ROS node, master or hardware."""
import io
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

import pytest
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE/'gen4_root_cause')]
import board_source_capture as board
import collect_single_axis as single

COLOR = '/supervisor/camera/color/image_raw'
DEPTH = '/supervisor/camera/depth/image_raw'


class Clock:
    def __init__(self, ns=10_000_000_000):
        self.ns = ns
        self.calls = []
        self.lock = threading.Lock()

    def now(self):
        with self.lock:
            ns = self.ns
            self.calls.append(ns)
        return rospy.Time(*divmod(ns, 10**9))

    def advance(self, ns):
        with self.lock:
            self.ns += ns


class Writer:
    def __init__(self, recorder):
        self.recorder = recorder
        self.rows = []
        self.failure = None
        self.closed = False
        self.failed_payloads = []
        self.on_submit = None

    def submit(self, topic, payload, stamp, datatype, md5, pytype):
        assert not self.recorder.lock._is_owned(), 'archive admission held state lock'
        if self.failure:
            raise RuntimeError(self.failure)
        if self.on_submit:
            self.on_submit(topic, payload, stamp)
        self.rows.append((topic, payload, stamp, datatype, md5, pytype))

    def latch_failure(self, reason):
        self.failure = str(reason)
        self.recorder.fatal = 'diagnostic archive: '+self.failure

    def summary(self):
        return dict(archive_complete=self.failure is None, failure=self.failure,
                    written=len(self.rows))

    def close(self, timeout=30.):
        self.closed = True
        return self.summary()


@pytest.fixture
def state(tmp_path, monkeypatch):
    r = board.BoardRecorder.__new__(board.BoardRecorder)
    r.out = tmp_path
    r.lock = threading.RLock()
    r._archive_local = threading.local()
    r._callback_condition = threading.Condition()
    r._callback_inflight = 0
    r._accepting_callbacks = True
    r._event_lock = threading.Lock()
    r._receipt_samples = {}
    r._receipt_maxima = {}
    r.board_quality = {}
    r.board_tf = {}
    r.board_snapshots = {}
    r.active = True
    r.frames = {}
    r.counts = {}
    r.states = []
    r.latest_inputs = {}
    r.fatal = None
    r.armed = False
    r.last_joint_ns = 0
    r.latest_task_active = False
    r.epoch_ns = None
    r.allow_epoch_until = 0
    r.subs = []
    r.bag = None
    r.archive_writer = Writer(r)
    clock = Clock()
    monkeypatch.setattr(rospy.Time, 'now', clock.now)
    return r, clock


def image(ns=9_950_000_000, pixels=b'original pixels'):
    msg = Image()
    msg.header.stamp = rospy.Time(*divmod(ns, 10**9))
    msg.header.frame_id = 'camera_link'
    msg.width = len(pixels)
    msg.height = 1
    msg.step = len(pixels)
    msg.encoding = 'mono8'
    msg.data = pixels
    return msg


def thread_call(callable_):
    errors = []
    def run():
        try:
            callable_()
        except BaseException as error:
            errors.append(error)
    worker = threading.Thread(target=run)
    worker.start()
    return worker, errors


def join(worker, errors):
    worker.join(2.)
    assert not worker.is_alive()
    assert not errors


def test_entry_receipt_frozen_and_admission_precedes_one_receive_chain(state, monkeypatch):
    r, clock = state
    msg = image()
    original = Image.serialize
    order = []
    def serialize(self, stream):
        assert not r.lock._is_owned()
        assert COLOR not in r.counts and not r.frames
        order.append('serialize')
        clock.advance(300_000_000)
        original(self, stream)
    monkeypatch.setattr(Image, 'serialize', serialize)
    original_receive = single.Recorder.receive
    def receive(self, topic, message):
        assert r.lock._is_owned()
        assert len(r.archive_writer.rows) == 1
        order.append('receive')
        return original_receive(self, topic, message)
    monkeypatch.setattr(single.Recorder, 'receive', receive)
    r.archive_writer.on_submit = lambda *_: order.append('admission')
    r.callback(COLOR)(msg)
    assert order == ['serialize', 'admission', 'receive']
    assert r.counts == {COLOR: 1}
    assert len(r.archive_writer.rows) == 1
    assert r.archive_writer.rows[0][2] == 10_000_000_000
    assert r.latest_inputs[COLOR] == (msg, 10_000_000_000)
    assert r._receipt_samples[COLOR] == pytest.approx([.05])
    assert r._archive_local.skip_receive_archive is False
    assert r._archive_local.receipt_ns is None
    assert r._callback_inflight == 0 and r.fatal is None


def test_blocked_serialization_keeps_state_lock_and_other_callback_available(state, monkeypatch):
    r, _ = state
    entered, release = threading.Event(), threading.Event()
    original = Image.serialize
    def serialize(self, stream):
        if self.data == b'blocked':
            entered.set()
            assert release.wait(2.)
        original(self, stream)
    monkeypatch.setattr(Image, 'serialize', serialize)
    worker, errors = thread_call(lambda: r.callback(COLOR)(image(pixels=b'blocked')))
    try:
        assert entered.wait(1.)
        assert r.lock.acquire(timeout=.2)
        r.lock.release()
        r.callback('/other')(String(data='ready'))
        assert r.counts == {'/other': 1}
        assert not r.frames
    finally:
        release.set()
        join(worker, errors)
    assert r.counts[COLOR] == 1 and len(r.archive_writer.rows) == 2


def test_blocked_admission_cannot_use_unarchived_source(state):
    r, _ = state
    entered, release = threading.Event(), threading.Event()
    def admission(*_):
        entered.set()
        assert release.wait(2.)
    r.archive_writer.on_submit = admission
    worker, errors = thread_call(lambda: r.callback(COLOR)(image()))
    try:
        assert entered.wait(1.)
        assert not r.frames and not r.latest_inputs and not r.counts
        assert r.lock.acquire(timeout=.2)
        r.lock.release()
    finally:
        release.set()
        join(worker, errors)
    assert r.counts == {COLOR: 1}


def test_concurrent_callback_thread_local_flags_cannot_suppress_other_archive(state, monkeypatch):
    r, _ = state
    entered, release = threading.Event(), threading.Event()
    original = single.Recorder.receive
    def receive(self, topic, msg):
        if topic == COLOR:
            entered.set()
            assert release.wait(2.)
        return original(self, topic, msg)
    monkeypatch.setattr(single.Recorder, 'receive', receive)
    first, first_errors = thread_call(lambda: r.callback(COLOR)(image()))
    try:
        assert entered.wait(1.)
        admitted = threading.Event()
        r.archive_writer.on_submit = lambda topic, *_: admitted.set() if topic == DEPTH else None
        second, second_errors = thread_call(lambda: r.callback(DEPTH)(image()))
        assert admitted.wait(1.), 'archive must precede waiting for the state lock'
        assert len(r.archive_writer.rows) == 2
    finally:
        release.set()
        join(first, first_errors)
    join(second, second_errors)
    assert r.counts == {COLOR: 1, DEPTH: 1}
    assert len(r.frames[9_950_000_000]) == 2


def test_headers_pixels_and_full_raw_bytes_preserved_without_cross_source_pair(state):
    r, _ = state
    color, depth = image(pixels=b'color'), image(ns=9_940_000_000, pixels=b'depth')
    expected = io.BytesIO()
    color.serialize(expected)
    r.callback(COLOR)(color)
    r.callback(DEPTH)(depth)
    assert r.archive_writer.rows[0][1] == expected.getvalue()
    assert set(r.frames[9_950_000_000]) == {'color'}
    assert set(r.frames[9_940_000_000]) == {'depth'}
    restored = Image().deserialize(r.archive_writer.rows[0][1])
    assert restored.header.stamp.to_nsec() == 9_950_000_000
    assert restored.data == b'color'


@pytest.mark.parametrize('stage', ['serialize', 'admission', 'receive'])
def test_callback_exceptions_latch_fatal_and_drain_tls(state, monkeypatch, stage):
    r, _ = state
    def fail(*_):
        raise ValueError('intentional '+stage)
    if stage == 'serialize':
        monkeypatch.setattr(Image, 'serialize', fail)
    elif stage == 'admission':
        r.archive_writer.on_submit = fail
    else:
        monkeypatch.setattr(single.Recorder, 'receive', fail)
    r.callback(COLOR)(image())
    assert 'intentional '+stage in r.fatal
    assert not r.frames and not r.counts
    assert r._callback_inflight == 0
    assert not getattr(r._archive_local, 'skip_receive_archive', False)
    assert getattr(r._archive_local, 'receipt_ns', None) is None


def test_callback_admission_closed_refuses_archive_and_state(state):
    r, _ = state
    r._accepting_callbacks = False
    r.callback(COLOR)(image())
    assert not r.archive_writer.rows and not r.frames and not r.counts
    assert r._callback_inflight == 0


def test_quality_and_board_tf_use_entry_receipt_not_later_receive_time(state):
    r, clock = state
    quality = String(data=json.dumps(dict(source_stamp_ns=9_950_000_000, charuco_count=40)))
    r.archive_writer.on_submit = lambda *_: clock.advance(200_000_000)
    r.callback('/charuco/quality')(quality)
    assert r.board_quality[9_950_000_000][1] == 10_000_000_000
    transform = SimpleNamespace(child_frame_id=board.BOARD_FRAME,
        header=SimpleNamespace(stamp=rospy.Time(9, 950_000_000)))
    class Transforms:
        _type = 'tf2_msgs/TFMessage'
        _md5sum = 'fixture'
        transforms = [transform]
        def serialize(self, stream):
            stream.write(b'transform')
    r.callback('/tf')(Transforms())
    assert r.board_tf[9_950_000_000] == (transform, 10_200_000_000)


def test_mark_large_json_io_and_archive_do_not_take_state_lock(state, monkeypatch, capsys):
    r, _ = state
    original_dumps = json.dumps
    encoded_audits = []
    def dumps(obj, *args, **kwargs):
        assert not r.lock._is_owned()
        if isinstance(obj, dict) and 'audit' in obj:
            encoded_audits.append(obj)
        return original_dumps(obj, *args, **kwargs)
    monkeypatch.setattr(board.json, 'dumps', dumps)
    r.mark('actual_submission_geometry', audit={'proof': list(range(20_000))})
    assert len(encoded_audits) == 1
    saved = json.loads((r.out/'events.jsonl').read_text())
    row = r.archive_writer.rows[0]
    restored = String().deserialize(row[1])
    assert json.loads(restored.data) == saved
    assert row[2] == saved['time_ns'] == 10_000_000_000
    assert 'audit' not in json.loads(capsys.readouterr().out)


def test_blocked_event_io_does_not_block_live_callback(state, monkeypatch, capsys):
    r, _ = state
    entered, release = threading.Event(), threading.Event()
    original_open = Path.open
    class BlockedFile:
        def __enter__(self):
            return self
        def __exit__(self, *_):
            return False
        def write(self, payload):
            assert not r.lock._is_owned()
            entered.set()
            assert release.wait(2.)
    def open_(path, *args, **kwargs):
        return BlockedFile() if path == r.out/'events.jsonl' else original_open(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'open', open_)
    worker, errors = thread_call(lambda: r.mark('event', audit={'proof': 'large'}))
    try:
        assert entered.wait(1.)
        r.callback(COLOR)(image())
        assert r.counts == {COLOR: 1}
        assert r.archive_writer.rows[0][0] == COLOR
    finally:
        release.set()
        join(worker, errors)
    assert r.archive_writer.rows[1][0] == '/support_diagnostic/event'
    capsys.readouterr()


def test_close_drains_inflight_callback_before_archive_close(state, monkeypatch):
    r, _ = state
    entered, release = threading.Event(), threading.Event()
    original = Image.serialize
    def serialize(self, stream):
        entered.set()
        assert release.wait(2.)
        original(self, stream)
    monkeypatch.setattr(Image, 'serialize', serialize)
    callback, callback_errors = thread_call(lambda: r.callback(COLOR)(image()))
    closer = None
    try:
        assert entered.wait(1.)
        closer, close_errors = thread_call(r.close)
        deadline = time.monotonic()+1.
        while r._accepting_callbacks and time.monotonic() < deadline:
            time.sleep(.001)
        assert not r._accepting_callbacks
        assert not r.archive_writer.closed
        release.set()
        join(callback, callback_errors)
        join(closer, close_errors)
    finally:
        release.set()
        if callback.is_alive(): callback.join(2.)
        if closer and closer.is_alive(): closer.join(2.)
    assert json.loads((r.out/'counts.json').read_text())['archive_writer']['archive_complete']
    assert r.counts == {COLOR: 1} and not r.active


def test_close_callback_drain_timeout_is_explicit_incomplete_archive(state, monkeypatch):
    r, _ = state
    r._callback_inflight = 1
    ticks = iter([0., 6.])
    monkeypatch.setattr(board.time, 'monotonic', lambda: next(ticks))
    with pytest.raises(RuntimeError, match='archive incomplete'):
        r.close()
    counts = json.loads((r.out/'counts.json').read_text())
    assert counts['archive_writer']['archive_complete'] is False
    assert 'callback drain' in counts['archive_writer']['failure']
    assert r.archive_writer.closed


def test_close_child_failure_cannot_leave_counts_marked_complete(state):
    r, _ = state
    class Sink:
        def close(self, timeout):
            raise RuntimeError('child close failed')
    r.process_sink = Sink()
    with pytest.raises(RuntimeError, match='child close failed'):
        r.close()
    counts = json.loads((r.out/'counts.json').read_text())
    assert counts['archive_writer']['archive_complete'] is False
    assert 'child close failed' in counts['process_archive_close_error']
    assert json.loads((r.out/'process_archive.json').read_text())['archive_complete'] is False


def test_receipt_report_write_failure_still_closes_child(state, monkeypatch):
    r, _ = state
    closed = []
    class Sink:
        def close(self, timeout):
            closed.append(timeout)
            return dict(bag_close_confirmed=True, child_alive=False)
    r.process_sink = Sink()
    original_write = board.base.write
    def write(path, payload):
        if path.name == 'receipt_latency.json':
            raise OSError('receipt evidence drive error')
        return original_write(path, payload)
    monkeypatch.setattr(board.base, 'write', write)
    with pytest.raises(OSError, match='receipt evidence drive error'):
        r.close()
    assert closed == [30.]
    assert r.archive_writer.closed
    assert json.loads((r.out/'process_archive.json').read_text())['bag_close_confirmed']


def test_parent_counts_write_error_does_not_prevent_child_cleanup(state, monkeypatch):
    r, _ = state
    closed = []
    class Sink:
        def close(self, timeout):
            closed.append(timeout)
            return dict(bag_close_confirmed=True, child_alive=False)
    r.process_sink = Sink()
    original_write_text = Path.write_text
    def write_text(path, *args, **kwargs):
        if path.name == 'counts.json':
            raise OSError('counts evidence drive error')
        return original_write_text(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'write_text', write_text)
    with pytest.raises(OSError, match='counts evidence drive error'):
        r.close()
    assert closed == [30.]
    assert r.archive_writer.closed
    assert json.loads((r.out/'process_archive.json').read_text())['bag_close_confirmed']


def test_entry_receipt_retained_for_validated_feedback(state):
    r, _ = state
    from sensor_msgs.msg import JointState
    msg = JointState()
    msg.header.stamp = rospy.Time(9, 950_000_000)
    msg.header.frame_id = 'sdk_measured'
    msg.name = ['Joint%d'%i for i in range(1, 7)]+['right_finger']
    msg.position = [0.]*7
    r.callback('/alicia_d/accepted_joint_states')(msg)
    assert r.states == [(9_950_000_000, [0.]*7)]
    assert r.latest_inputs['/alicia_d/accepted_joint_states'][1] == 10_000_000_000
    assert r._receipt_samples['/alicia_d/accepted_joint_states'] == pytest.approx([.05])
    assert len(r.archive_writer.rows) == 1 and r._callback_inflight == 0
