"""Spawn/IPC/archive lifecycle tests. Child factories only write tmp test data."""
from functools import partial
import multiprocessing
import os
import io
from pathlib import Path
import pickle
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from bounded_async_bag_writer import BoundedAsyncBagWriter
from gen4_root_cause.process_bag_sink import ProcessBagSink
import gen4_root_cause.process_bag_sink as SINK_MODULE


class RawMessage:
    _type='synthetic/RawMessage'
    _md5sum='0123456789abcdef0123456789abcdef'
    _full_text='uint8[] data\n'


class TestFile:
    __test__=False
    def __init__(self,path,delay=0,fail=False,die=False,close_delay=0,close_fail=False):
        self.path=Path(path);self.delay=delay;self.fail=fail;self.die=die
        self.close_delay=close_delay;self.close_fail=close_fail
        self.file=self.path.open('wb')

    def write(self,m,p,t):
        (self.path.parent/'entered').write_text(str(os.getpid()))
        end=time.monotonic()+self.delay
        # Deliberately CPU/GIL-bound to check isolation,not a releasing sleep.
        while time.monotonic()<end:sum(range(5000))
        if self.die:os._exit(17)
        if self.fail:raise IOError('synthetic disk failure')
        pickle.dump((m,p,t),self.file,protocol=4)
        self.file.flush()

    def close(self):
        time.sleep(self.close_delay)
        if self.close_fail:raise IOError('synthetic bag close failure')
        self.file.close();(self.path.parent/'closed').write_text('closed')


def fail_open(path):raise IOError('synthetic open failure')


def delayed_open(path):time.sleep(1);return TestFile(path)


def mutated_type_factory(path):
    RawMessage._full_text='uint16[] wrong_definition\n'
    return TestFile(path)


def child_dies_without_reading(path,commands,responses,factory):
    responses.send(('ready',0))
    # Parent feeder can fill the OS pipe,but this child never reads it.
    time.sleep(.3);os._exit(19)


def metadata(payload=b'raw',stamp=1791035288690793514):return dict(
    topic='/synthetic/raw',datatype=RawMessage._type,md5sum=RawMessage._md5sum,
    stamp_ns=stamp,serialized_bytes=len(payload))


def wait_file(path,timeout=2):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if path.exists():return True
        time.sleep(.005)
    return False


class ProcessBagSinkTests(unittest.TestCase):
    def test_raw_fidelity_order_receipt_timestamp_and_original_type(self):
        with tempfile.TemporaryDirectory(prefix='bag_sink_',dir='/tmp')as d:
            path=Path(d)/'raw.bin';sink=ProcessBagSink(path,_sink_factory=TestFile)
            self.assertNotEqual(sink.process.pid,os.getpid());self.assertEqual(sink.summary()['start_method'],'spawn')
            expected=[]
            for i in range(8):
                payload=bytes([0,255,i])+b'\x00raw\n'+str(i).encode();m=metadata(payload,1791035288690793514+i)
                expected.append((m.copy(),payload,RawMessage));sink.write(m,payload,RawMessage)
                m['topic']='/mutated_after_ack'
            report=sink.close();self.assertTrue(report['bag_close_confirmed']);self.assertEqual(report['written'],8)
            got=[]
            with path.open('rb')as f:
                while True:
                    try:got.append(pickle.load(f))
                    except EOFError:break
            self.assertEqual(got,expected);self.assertEqual(sink.close(),report)
            self.assertFalse(sink.process.is_alive())
            self.assertFalse(report['ipc_feeder_alive'])

    def test_child_CPU_block_does_not_block_parent_heartbeat_or_archive_admission(self):
        with tempfile.TemporaryDirectory(prefix='bag_sink_',dir='/tmp')as d:
            sink=ProcessBagSink(Path(d)/'raw.bin',_sink_factory=partial(TestFile,delay=.4))
            writer=BoundedAsyncBagWriter(sink.write,lambda reason:None,maximum_bytes=32*1024*1024)
            writer.submit('/synthetic/raw',b'raw',1,RawMessage._type,RawMessage._md5sum,RawMessage)
            self.assertTrue(wait_file(Path(d)/'entered'))
            ticks=[];end=time.monotonic()+.15
            while time.monotonic()<end:ticks.append(time.monotonic());time.sleep(.002)
            self.assertGreater(len(ticks),30)
            self.assertLess(max(b-a for a,b in zip(ticks,ticks[1:])),.05)
            start=time.monotonic();writer.submit('/synthetic/raw',b'two',2,RawMessage._type,RawMessage._md5sum,RawMessage)
            self.assertLess(time.monotonic()-start,.03)
            self.assertTrue(writer.close()['archive_complete']);sink.close()

    def test_child_write_error_retains_all_parent_failed_payloads(self):
        with tempfile.TemporaryDirectory(prefix='bag_sink_',dir='/tmp')as d:
            sink=ProcessBagSink(Path(d)/'raw.bin',_sink_factory=partial(TestFile,delay=.1,fail=True))
            faults=[];writer=BoundedAsyncBagWriter(sink.write,faults.append)
            for p in [b'first',b'second']:writer.submit('/synthetic/raw',p,7,RawMessage._type,RawMessage._md5sum,RawMessage)
            r=writer.close();self.assertFalse(r['archive_complete']);self.assertEqual(r['failed'],2)
            self.assertEqual([x[1]for x in writer.failed_payloads],[b'first',b'second'])
            self.assertEqual(len(faults),1)
            with self.assertRaisesRegex(RuntimeError,'synthetic disk failure'):sink.close()
            self.assertTrue(sink.close_confirmed);self.assertFalse(sink.process.is_alive())

    def test_child_crash_is_explicit_and_cleanup_does_not_touch_unrelated_process(self):
        with tempfile.TemporaryDirectory(prefix='bag_sink_',dir='/tmp')as d:
            ctx=multiprocessing.get_context('spawn');other=ctx.Process(target=time.sleep,args=(3,));other.start()
            try:
                sink=ProcessBagSink(Path(d)/'raw.bin',_sink_factory=partial(TestFile,die=True))
                with self.assertRaisesRegex(RuntimeError,'pipe closed|exited'):sink.write(metadata(),b'raw',RawMessage)
                with self.assertRaises(RuntimeError):sink.close(timeout=1)
                self.assertFalse(sink.process.is_alive());self.assertTrue(other.is_alive())
            finally:other.terminate();other.join(2)

    def test_write_deadline_is_terminal_and_close_reaps_blocked_child(self):
        with tempfile.TemporaryDirectory(prefix='bag_sink_',dir='/tmp')as d:
            sink=ProcessBagSink(Path(d)/'raw.bin',write_timeout=.1,_sink_factory=partial(TestFile,delay=3))
            start=time.monotonic()
            with self.assertRaisesRegex(RuntimeError,'deadline'):sink.write(metadata(),b'raw',RawMessage)
            self.assertLess(time.monotonic()-start,.4)
            with self.assertRaisesRegex(RuntimeError,'deadline'):sink.write(metadata(),b'raw',RawMessage)
            with self.assertRaises(RuntimeError):sink.close(timeout=.3)
            self.assertFalse(sink.process.is_alive());self.assertFalse(sink.close_confirmed)

    def test_ready_failure_and_ready_deadline_are_explicit(self):
        with tempfile.TemporaryDirectory(prefix='bag_sink_',dir='/tmp')as d:
            with self.assertRaisesRegex(RuntimeError,'synthetic open failure'):ProcessBagSink(Path(d)/'a',_sink_factory=fail_open)
            start=time.monotonic()
            with self.assertRaisesRegex(RuntimeError,'deadline'):ProcessBagSink(Path(d)/'b',ready_timeout=.1,_sink_factory=delayed_open)
            self.assertLess(time.monotonic()-start,.7)

    def test_close_confirmation_requires_successful_actual_child_close(self):
        with tempfile.TemporaryDirectory(prefix='bag_sink_',dir='/tmp')as d:
            sink=ProcessBagSink(Path(d)/'raw.bin',_sink_factory=partial(TestFile,close_fail=True))
            sink.write(metadata(),b'raw',RawMessage)
            with self.assertRaisesRegex(RuntimeError,'synthetic bag close failure'):sink.close()
            self.assertFalse(sink.close_confirmed);self.assertFalse(sink.process.is_alive())

    def test_close_deadline_reaps_child_stuck_during_bag_close(self):
        with tempfile.TemporaryDirectory(prefix='bag_sink_',dir='/tmp')as d:
            sink=ProcessBagSink(Path(d)/'raw.bin',_sink_factory=partial(TestFile,close_delay=3))
            with self.assertRaisesRegex(RuntimeError,'deadline'):sink.close(timeout=.3)
            self.assertFalse(sink.close_confirmed);self.assertFalse(sink.process.is_alive())

    def test_mutable_oversize_bad_timestamp_and_unpickleable_type_rejected_before_IPC(self):
        for payload,stamp,pytype,reason in [
            (bytearray(b'raw'),1,RawMessage,'immutable'),
            (b'0123456789',1,RawMessage,'maximum'),
            (b'raw',1.5,RawMessage,'stamp'),
            (b'raw',1,type('NonImportable',(),dict(_type=RawMessage._type,
                 _md5sum=RawMessage._md5sum,_full_text=RawMessage._full_text)),'pickled')]:
            with self.subTest(reason=reason),tempfile.TemporaryDirectory(prefix='bag_sink_',dir='/tmp')as d:
                sink=ProcessBagSink(Path(d)/'raw.bin',maximum_payload_bytes=8,_sink_factory=TestFile)
                with self.assertRaisesRegex(RuntimeError,reason):sink.write(metadata(payload,stamp),payload,pytype)
                self.assertEqual(sink.sequence,0)
                with self.assertRaises(RuntimeError):sink.close()
                self.assertTrue(sink.close_confirmed)

    def test_concurrent_write_does_not_admit_second_inflight_message(self):
        with tempfile.TemporaryDirectory(prefix='bag_sink_',dir='/tmp')as d:
            sink=ProcessBagSink(Path(d)/'raw.bin',_sink_factory=partial(TestFile,delay=.3));errors=[]
            def write():
                try:sink.write(metadata(),b'raw',RawMessage)
                except Exception as exc:errors.append(str(exc))
            thread=threading.Thread(target=write);thread.start();self.assertTrue(wait_file(Path(d)/'entered'))
            with self.assertRaisesRegex(RuntimeError,'concurrent'):sink.write(metadata(),b'raw',RawMessage)
            thread.join(1);self.assertFalse(errors);self.assertEqual(sink.sequence,1);sink.close()

    def test_raw_message_definition_is_verified_in_spawned_child(self):
        with tempfile.TemporaryDirectory(prefix='bag_sink_',dir='/tmp')as d:
            sink=ProcessBagSink(Path(d)/'raw.bin',_sink_factory=mutated_type_factory)
            with self.assertRaisesRegex(RuntimeError,'type/definition mismatch'):sink.write(metadata(),b'raw',RawMessage)
            with self.assertRaises(RuntimeError):sink.close()
            self.assertTrue(sink.close_confirmed);self.assertFalse(sink.summary()['ipc_feeder_alive'])

    def test_wrong_raw_datatype_and_MD5_fail_before_IPC(self):
        for key in ['datatype','md5sum']:
            with self.subTest(key=key),tempfile.TemporaryDirectory(prefix='bag_sink_',dir='/tmp')as d:
                sink=ProcessBagSink(Path(d)/'raw.bin',_sink_factory=TestFile);m=metadata();m[key]='wrong'
                with self.assertRaisesRegex(RuntimeError,'datatype/MD5 mismatch'):sink.write(m,b'raw',RawMessage)
                self.assertEqual(sink.sequence,0)
                with self.assertRaises(RuntimeError):sink.close()

    def test_large_inflight_crash_reaps_feeder_without_retaining_IPC_copy(self):
        with tempfile.TemporaryDirectory(prefix='bag_sink_',dir='/tmp')as d:
            with patch.object(SINK_MODULE,'_child_main',child_dies_without_reading):
                sink=ProcessBagSink(Path(d)/'raw.bin',_sink_factory=TestFile)
            payload=b'x'*(8*1024*1024)
            with self.assertRaises(RuntimeError):sink.write(metadata(payload),payload,RawMessage)
            with self.assertRaises(RuntimeError):sink.close(timeout=1)
            self.assertFalse(sink.process.is_alive());self.assertFalse(sink.summary()['ipc_feeder_alive'])

    def test_real_LZ4_rosbag_raw_bytes_type_and_original_nanosecond_stamps(self):
        import rosbag
        from std_msgs.msg import String
        from sensor_msgs.msg import Image
        with tempfile.TemporaryDirectory(prefix='bag_sink_',dir='/tmp')as d:
            path=Path(d)/'samples.bag';sink=ProcessBagSink(path);expected=[]
            for i,msg in enumerate([String(data='exact \u4e2d\u6587\nreceipt'),
                  Image(height=1,width=3,encoding='mono8',step=3,data=b'\x00\xff\x01'),String(data='final')]):
                stream=io.BytesIO();msg.serialize(stream);payload=stream.getvalue();stamp=1791035288690793514+i
                topic='/image'if isinstance(msg,Image)else '/event'
                m=dict(topic=topic,stamp_ns=stamp,serialized_bytes=len(payload),datatype=msg._type,md5sum=msg._md5sum)
                sink.write(m,payload,type(msg));expected.append((topic,payload,stamp,msg._type,msg._md5sum))
            self.assertTrue(sink.close()['bag_close_confirmed'])
            with rosbag.Bag(str(path),'r')as bag:
                self.assertEqual({x.compression for x in bag._chunk_headers.values()},{'lz4'})
                got=[(topic,raw[1],stamp.to_nsec(),raw[0],raw[2])for topic,raw,stamp in bag.read_messages(raw=True)]
                definitions={c.datatype:(c.md5sum,c.msg_def)for c in bag._connections.values()}
                self.assertEqual(definitions[String._type],(String._md5sum,String._full_text))
                self.assertEqual(definitions[Image._type],(Image._md5sum,Image._full_text))
            self.assertEqual(got,expected)


if __name__=='__main__':unittest.main()
