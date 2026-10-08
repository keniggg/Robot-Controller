"""Finite in-memory archive queue; never waits for disk in producer callbacks."""
from collections import Counter, deque
import threading
import time


class BoundedAsyncBagWriter:
    def __init__(self, write, on_failure, maximum_bytes=64*1024*1024, maximum_items=512):
        if maximum_bytes<=0 or maximum_items<=0:raise ValueError('positive queue bounds required')
        self.write,self.on_failure=write,on_failure
        self.maximum_bytes,self.maximum_items=int(maximum_bytes),int(maximum_items)
        self.condition=threading.Condition();self.queue=deque();self.failed_payloads=[]
        self.failure=None;self.closing=False;self.inflight=0;self.bytes=0
        self.accepted,self.written,self.failed,self.rejected=0,0,0,0
        self.peak_bytes,self.peak_items=0,0;self.maximum_write_seconds=0.
        self.rejected_topics=Counter();self.failures=[]
        self.worker=threading.Thread(target=self._run,name='centering-bag-writer',daemon=True)
        self.worker.start()

    def _latch(self,reason,metadata=None):
        # Caller holds condition. Notify external state ONLY after releasing it.
        first=self.failure is None
        if first:self.failure=str(reason)
        if len(self.failures)<16:self.failures.append(dict(reason=str(reason),metadata=metadata or {}))
        return first

    def submit(self,topic,payload,stamp_ns,datatype,md5sum,pytype):
        if not isinstance(payload,bytes):raise ValueError('serialized immutable bytes required')
        cost=len(payload)+2048
        metadata=dict(topic=topic,stamp_ns=int(stamp_ns),serialized_bytes=len(payload),datatype=datatype,md5sum=md5sum)
        notification=False
        with self.condition:
            total_items=len(self.queue)+self.inflight+len(self.failed_payloads)
            if self.failure is not None or self.closing:
                self.rejected+=1;self.rejected_topics[topic]+=1
                reason=self.failure or 'archive writer is closing'
            elif self.bytes+cost>self.maximum_bytes or total_items+1>self.maximum_items:
                self.rejected+=1;self.rejected_topics[topic]+=1
                reason='bounded archive queue overflow; evidence admission halted'
                notification=self._latch(reason,metadata)
            else:
                self.queue.append((metadata,payload,pytype,cost));self.bytes+=cost;self.accepted+=1
                self.peak_bytes=max(self.peak_bytes,self.bytes);self.peak_items=max(self.peak_items,total_items+1)
                self.condition.notify();return
        if notification:self.on_failure(reason)
        raise RuntimeError(reason)

    def _run(self):
        while True:
            with self.condition:
                while not self.queue and not self.closing:self.condition.wait()
                if not self.queue:return
                item=self.queue.popleft();self.inflight=1
            metadata,payload,pytype,cost=item;start=time.monotonic();failure=None
            try:self.write(metadata,payload,pytype)
            except BaseException as error:failure=str(error)
            with self.condition:
                self.maximum_write_seconds=max(self.maximum_write_seconds,time.monotonic()-start)
                self.inflight=0
                if failure is None:self.bytes-=cost;self.written+=1;notification=False
                else:
                    self.failed+=1;self.failed_payloads.append(item)
                    notification=self._latch('archive write failed: '+failure,metadata)
                self.condition.notify_all()
            if notification:
                try:self.on_failure(self.failure)
                except BaseException:
                    # Failure remains observable through summary and close; a
                    # notification error must never kill queue draining silently.
                    pass

    def require_healthy(self):
        # The owner checks the latch directly even while its state lock delays
        # asynchronous on_failure notification. No disk operation holds this lock.
        with self.condition:
            if self.failure is not None:raise RuntimeError(self.failure)
            if self.closing:raise RuntimeError('archive writer is closing')

    def latch_failure(self,reason):
        with self.condition:notification=self._latch(reason)
        if notification:self.on_failure(self.failure)

    def close(self,timeout=30.):
        with self.condition:self.closing=True;self.condition.notify_all()
        self.worker.join(timeout)
        if self.worker.is_alive():
            with self.condition:notification=self._latch('archive writer drain deadline exhausted')
            if notification:self.on_failure(self.failure)
            raise RuntimeError(self.failure)
        return self.summary()

    def summary(self):
        with self.condition:
            return dict(accepted=self.accepted,written=self.written,failed=self.failed,rejected=self.rejected,
                pending=len(self.queue),inflight=self.inflight,retained_failed_payloads=len(self.failed_payloads),
                maximum_bytes=self.maximum_bytes,maximum_items=self.maximum_items,
                peak_bytes=self.peak_bytes,peak_items=self.peak_items,
                maximum_write_seconds=self.maximum_write_seconds,
                failure=self.failure,failures=list(self.failures),rejected_topics=dict(self.rejected_topics),
                archive_complete=bool(self.failure is None and self.accepted==self.written and self.rejected==0 and not self.queue and not self.inflight))
