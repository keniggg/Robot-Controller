import threading
import time
import unittest
from bounded_async_bag_writer import BoundedAsyncBagWriter

class AsyncArchiveTests(unittest.TestCase):
    def submit(self,w,value=b'abc',stamp=123):w.submit('/test',value,stamp,'type','md5',bytes)
    def test_source_receipt_and_order_preserved(self):
        got=[];w=BoundedAsyncBagWriter(lambda m,p,t:got.append((m,p,t)),lambda reason:None)
        for i in range(20):self.submit(w,str(i).encode(),100+i)
        r=w.close();self.assertTrue(r['archive_complete']);self.assertEqual(r['written'],20)
        self.assertEqual([m['stamp_ns'] for m,_,_ in got],list(range(100,120)))
        self.assertEqual(got[3][1],b'3')
    def test_callback_does_not_wait_for_blocked_disk(self):
        entered=threading.Event();release=threading.Event()
        def write(*_):entered.set();release.wait(2)
        w=BoundedAsyncBagWriter(write,lambda reason:None)
        self.submit(w);self.assertTrue(entered.wait(1));start=time.monotonic();self.submit(w)
        self.assertLess(time.monotonic()-start,.05)
        release.set();self.assertTrue(w.close()['archive_complete'])
    def test_inflight_counts_towards_item_bound_and_overflow_latches(self):
        entered=threading.Event();release=threading.Event();fault=[]
        def write(*_):entered.set();release.wait(2)
        w=BoundedAsyncBagWriter(write,fault.append,maximum_items=1)
        self.submit(w);self.assertTrue(entered.wait(1))
        with self.assertRaises(RuntimeError):self.submit(w)
        self.assertEqual(len(fault),1);release.set();r=w.close()
        self.assertEqual(r['written'],1);self.assertEqual(r['rejected'],1);self.assertFalse(r['archive_complete'])
    def test_bytes_bound_includes_payload_and_overhead(self):
        fault=[];w=BoundedAsyncBagWriter(lambda *_:None,fault.append,maximum_bytes=2050)
        with self.assertRaises(RuntimeError):self.submit(w,b'abc')
        r=w.close();self.assertEqual(r['accepted'],0);self.assertEqual(len(fault),1)
    def test_write_failure_keeps_raw_payload_drains_already_admitted(self):
        entered=threading.Event();release=threading.Event();fault=[];calls=[]
        def write(m,p,t):
            calls.append(p)
            if p==b'bad':entered.set();release.wait(2);raise IOError('disk fault')
        w=BoundedAsyncBagWriter(write,fault.append)
        self.submit(w,b'bad');self.assertTrue(entered.wait(1));self.submit(w,b'good');release.set()
        r=w.close();self.assertEqual(calls,[b'bad',b'good']);self.assertEqual(r['failed'],1);self.assertEqual(r['written'],1)
        self.assertEqual(w.failed_payloads[0][1],b'bad');self.assertEqual(len(fault),1);self.assertFalse(r['archive_complete'])
    def test_close_drains_all_accepted_before_return(self):
        got=[]
        def write(m,p,t):time.sleep(.001);got.append(m['stamp_ns'])
        w=BoundedAsyncBagWriter(write,lambda reason:None)
        for i in range(30):self.submit(w,stamp=i)
        r=w.close();self.assertEqual(len(got),30);self.assertEqual(r['pending'],0);self.assertFalse(w.worker.is_alive())
    def test_submit_after_failure_rejected_no_new_payload(self):
        fault=[];w=BoundedAsyncBagWriter(lambda *_:None,fault.append,maximum_bytes=1)
        for _ in range(2):
            with self.assertRaises(RuntimeError):self.submit(w)
        r=w.close();self.assertEqual(r['rejected'],2);self.assertEqual(r['accepted'],0);self.assertEqual(len(fault),1)
    def test_close_deadline_explicit_fault(self):
        release=threading.Event();entered=threading.Event();fault=[]
        def write(*_):entered.set();release.wait(2)
        w=BoundedAsyncBagWriter(write,fault.append);self.submit(w);self.assertTrue(entered.wait(1))
        with self.assertRaises(RuntimeError):w.close(timeout=.01)
        release.set();r=w.close();self.assertIn('deadline',r['failure']);self.assertEqual(r['written'],1)
    def test_explicit_fault_latches_without_abandoning_admitted_items(self):
        entered=threading.Event();release=threading.Event();fault=[]
        def write(*_):entered.set();release.wait(2)
        w=BoundedAsyncBagWriter(write,fault.append);self.submit(w);self.assertTrue(entered.wait(1))
        w.latch_failure('callback termination unavailable')
        with self.assertRaises(RuntimeError):self.submit(w)
        release.set();r=w.close();self.assertEqual(r['written'],1)
        self.assertEqual(r['failure'],'callback termination unavailable');self.assertFalse(r['archive_complete'])
    def test_failure_guard_does_not_wait_for_blocked_notification(self):
        notification_entered=threading.Event();release=threading.Event()
        def failure(_):notification_entered.set();release.wait(2)
        def write(*_):raise IOError('disk failed before callback can take parent lock')
        w=BoundedAsyncBagWriter(write,failure);self.submit(w)
        self.assertTrue(notification_entered.wait(1))
        with self.assertRaisesRegex(RuntimeError,'disk failed'):w.require_healthy()
        release.set();self.assertFalse(w.close()['archive_complete'])
    def test_serializer_failure_latches_client_writer_before_raise(self):
        import types
        import bounded_motor_centering as motor
        diagnostic=motor.make_diagnostic_class(types.SimpleNamespace(Diagnostic=object)).__new__(motor.make_diagnostic_class(types.SimpleNamespace(Diagnostic=object)))
        reasons=[];diagnostic.archive_writer=BoundedAsyncBagWriter(lambda *_:None,reasons.append)
        class BadMessage:
            def serialize(self,_):raise ValueError('invalid message serialization')
        with self.assertRaises(ValueError):diagnostic.write_bag('/bad',BadMessage(),None)
        with self.assertRaisesRegex(RuntimeError,'serialization'):diagnostic.archive_writer.require_healthy()
        self.assertFalse(diagnostic.archive_writer.close()['archive_complete']);self.assertEqual(len(reasons),1)
    def test_mutable_payload_rejected(self):
        w=BoundedAsyncBagWriter(lambda *_:None,lambda reason:None)
        with self.assertRaises(ValueError):self.submit(w,bytearray(b'abc'))
        self.assertTrue(w.close()['archive_complete'])
if __name__=='__main__':unittest.main()
