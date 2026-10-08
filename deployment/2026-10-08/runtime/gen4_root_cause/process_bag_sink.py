"""One-message IPC sink: a spawned process alone opens/compresses the ROS bag.

The parent BoundedAsyncBagWriter still owns its 32MiB admitted queue,including
the in-flight original payload. This sink adds at most one serialized envelope
(payload <=32MiB,metadata/type overhead <=64KiB) and one child decoded payload.
IPC/pickling can make transient copies of that bounded envelope; this is not a
claim that combined process RSS is 32MiB. No subscriber callback calls write.

The child imports rosbag/genpy for file serialization only; it never initializes
ROS,connects to a master,accesses devices,or publishes commands. A failure is
terminal and raises to the existing parent writer,which retains failed bytes.
"""
import math
import hashlib
import multiprocessing
import os
import pickle
import queue
import threading
import time

MAX_PAYLOAD_BYTES=32*1024*1024
MAX_ENVELOPE_OVERHEAD=64*1024
MAX_STAMP_NS=(2**32)*10**9-1


class _RosbagFile:
    def __init__(self,path):
        import genpy
        import rosbag
        self.Time=genpy.Time
        self.bag=rosbag.Bag(path,'w',compression='lz4')

    def write(self,metadata,payload,pytype):
        stamp=self.Time(*divmod(metadata['stamp_ns'],10**9))
        raw=(metadata['datatype'],payload,metadata['md5sum'],pytype)
        self.bag.write(metadata['topic'],raw,stamp,raw=True)

    def close(self):self.bag.close()


def _open_rosbag(path):return _RosbagFile(path)


def _type_signature(pytype):
    if not isinstance(pytype,type):raise ValueError('original message Python type required')
    values={name:getattr(pytype,name,None)for name in ('_type','_md5sum','_full_text')}
    if any(not isinstance(v,str)for v in values.values()):raise ValueError('ROS message type/MD5/full definition required')
    if any(len(v)>MAX_ENVELOPE_OVERHEAD for v in values.values()):raise ValueError('ROS message type definition exceeds metadata bound')
    return (pytype.__module__,pytype.__qualname__,values['_type'],values['_md5sum'],
            hashlib.sha256(values['_full_text'].encode()).hexdigest())


def _child_main(path,commands,responses,factory):
    sink=None;sequence=0;expected=1
    try:
        sink=factory(path)
        responses.send(('ready',0))
        while True:
            envelope=commands.get()
            if envelope is None:break
            sequence=expected
            row=pickle.loads(envelope)
            if row[0]!=expected:raise RuntimeError('child sequence mismatch')
            _,metadata,payload,pytype,signature=row
            if _type_signature(pytype)!=signature:raise RuntimeError('child original message type/definition mismatch')
            sink.write(metadata,payload,pytype)
            responses.send(('written',sequence))
            expected+=1
    except BaseException as exc:
        # Small control messages fit in the pipe without unbounded buffering.
        try:responses.send(('error',sequence,type(exc).__name__,str(exc)[:1024]))
        except (BrokenPipeError,EOFError,OSError):pass
    finally:
        try:
            if sink is not None:sink.close()
        except BaseException as exc:
            try:responses.send(('close_error',sequence,type(exc).__name__,str(exc)[:1024]))
            except (BrokenPipeError,EOFError,OSError):pass
        else:
            try:responses.send(('closed',sequence))
            except (BrokenPipeError,EOFError,OSError):pass
        responses.close()


def _timeout(value,label,maximum):
    if isinstance(value,bool)or not isinstance(value,(int,float))or not math.isfinite(value)or not 0<value<=maximum:
        raise ValueError('%s must be positive and <=%ss'%(label,maximum))
    return float(value)


class ProcessBagSink:
    """Call write from one existing archive worker,then close after it drains.

    _sink_factory is a private pickleable test seam; production uses rosbag LZ4.
    Deadlines cover queue admission and child acknowledgement together. Once a
    deadline/error occurs,no subsequent message is admitted by this sink.
    """
    def __init__(self,path,*,ready_timeout=5.,write_timeout=5.,
                 maximum_payload_bytes=MAX_PAYLOAD_BYTES,_sink_factory=None):
        self.ready_timeout=_timeout(ready_timeout,'ready_timeout',5.)
        self.write_timeout=_timeout(write_timeout,'write_timeout',5.)
        if (isinstance(maximum_payload_bytes,bool)or not isinstance(maximum_payload_bytes,int)
                or not 0<maximum_payload_bytes<=MAX_PAYLOAD_BYTES):
            raise ValueError('payload bound must be positive and <=32MiB')
        self.maximum_payload_bytes=maximum_payload_bytes
        self.path=os.fspath(path);self.failure=None;self.closed=False
        self.close_confirmed=False;self.sequence=0;self.written=0
        self.operation=threading.Lock()
        ctx=multiprocessing.get_context('spawn')
        self.commands=ctx.Queue(maxsize=1)
        self.commands._ignore_epipe=True
        self.responses,send=ctx.Pipe(duplex=False)
        self.process=ctx.Process(target=_child_main,
             args=(self.path,self.commands,send,_sink_factory or _open_rosbag),
             name='diagnostic-bag-sink',daemon=True)
        deadline=time.monotonic()+self.ready_timeout
        protocol_deadline=deadline-min(.25,self.ready_timeout/4)
        try:
            self.process.start();send.close()
            self._ack(protocol_deadline,'ready',0)
        except BaseException as exc:
            self._fail(exc)
            send.close();self._cleanup(deadline)
            self.closed=True
            raise RuntimeError(self.failure)from exc

    def _fail(self,error):
        if self.failure is None:self.failure='process bag sink: '+str(error)

    def _ack(self,deadline,kind,sequence):
        while True:
            remaining=deadline-time.monotonic()
            if remaining<=0:raise RuntimeError('%s acknowledgement deadline exhausted'%kind)
            if self.responses.poll(min(.05,remaining)):
                try:ack=self.responses.recv()
                except (EOFError,OSError)as exc:
                    raise RuntimeError('child acknowledgement pipe closed')from exc
                if ack[0]in ('error','close_error'):
                    raise RuntimeError('child %s sequence %s: %s: %s'%tuple(ack))
                if ack[:2]!=(kind,sequence):
                    raise RuntimeError('unexpected child acknowledgement: '+repr(ack))
                return ack
            if not self.process.is_alive():
                # Exit can precede parent scheduling; drain a pending small ack.
                if self.responses.poll(0):continue
                raise RuntimeError('child exited without %s ack (exitcode=%s)'%(kind,self.process.exitcode))

    def _put(self,envelope,deadline):
        if not self.process.is_alive():raise RuntimeError('child not alive (exitcode=%s)'%self.process.exitcode)
        remaining=deadline-time.monotonic()
        if remaining<=0:raise RuntimeError('IPC admission deadline exhausted')
        try:self.commands.put(envelope,timeout=remaining)
        except queue.Full as exc:raise RuntimeError('IPC admission deadline exhausted')from exc

    def _envelope(self,metadata,payload,pytype,sequence):
        if type(payload)is not bytes:raise ValueError('serialized immutable bytes required')
        if len(payload)>self.maximum_payload_bytes:raise ValueError('serialized payload exceeds bounded IPC maximum')
        if not isinstance(metadata,dict):raise ValueError('raw message metadata dict required')
        metadata=dict(metadata)
        if set(metadata)!={'topic','datatype','md5sum','stamp_ns','serialized_bytes'}:
            raise ValueError('bounded raw metadata fields required')
        for name in ('topic','datatype','md5sum'):
            if not isinstance(metadata.get(name),str)or not metadata[name]:raise ValueError('invalid raw '+name)
        if sum(len(metadata[k])for k in ('topic','datatype','md5sum'))>MAX_ENVELOPE_OVERHEAD//4:
            raise ValueError('IPC metadata exceeds 64KiB bound')
        stamp=metadata.get('stamp_ns')
        if type(stamp)is not int or not 0<=stamp<=MAX_STAMP_NS:raise ValueError('exact nonnegative ROS receipt stamp_ns required')
        if metadata.get('serialized_bytes')!=len(payload):raise ValueError('serialized payload length mismatch')
        signature=_type_signature(pytype)
        if signature[2:4]!=(metadata['datatype'],metadata['md5sum']):raise ValueError('original raw message datatype/MD5 mismatch')
        # Serialize synchronously in the single archive worker: Queue's feeder
        # must never silently discard an unpickleable generated message type.
        try:envelope=pickle.dumps((sequence,metadata,payload,pytype,signature),protocol=4)
        except Exception as exc:raise ValueError('raw message type/metadata cannot be pickled: '+str(exc))from exc
        if len(envelope)>len(payload)+MAX_ENVELOPE_OVERHEAD:raise ValueError('IPC metadata/type overhead exceeds 64KiB')
        return envelope

    def write(self,metadata,payload,pytype):
        if not self.operation.acquire(blocking=False):raise RuntimeError('concurrent sink operation is unsupported')
        try:
            if self.closed:raise RuntimeError('process bag sink is closed')
            if self.failure is not None:raise RuntimeError(self.failure)
            deadline=time.monotonic()+self.write_timeout
            sequence=self.sequence+1
            envelope=self._envelope(metadata,payload,pytype,sequence)
            self.sequence=sequence
            self._put(envelope,deadline);self._ack(deadline,'written',sequence)
            self.written+=1
        except BaseException as exc:
            self._fail(exc)
            raise RuntimeError(self.failure)from exc
        finally:self.operation.release()

    def _cleanup(self,deadline):
        # The only signal target is the Process instance created by this sink.
        # Closing the unused parent reader also unblocks a feeder if the child
        # died midway through a large IPC payload; no feeder join is unbounded.
        if self.process.pid is not None:
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(min(.1,max(0,deadline-time.monotonic())/2))
            else:self.process.join(0)
            if self.process.is_alive():
                self.process.kill();self.process.join(max(0,deadline-time.monotonic()))
            if self.process.is_alive():self._fail('child cleanup deadline exhausted')
        self.commands.cancel_join_thread();self.commands.close()
        self.commands._reader.close()
        feeder=self.commands._thread
        if feeder is not None:
            feeder.join(max(0,deadline-time.monotonic()))
            if feeder.is_alive():self._fail('IPC feeder cleanup deadline exhausted')
        self.commands._writer.close()
        self.responses.close()

    def close(self,timeout=30.):
        timeout=_timeout(timeout,'close timeout',30.);deadline=time.monotonic()+timeout
        if self.closed:
            if self.failure is not None:raise RuntimeError(self.failure)
            return self.summary()
        protocol_deadline=deadline-min(.25,timeout/4)
        acquired=self.operation.acquire(timeout=max(0,protocol_deadline-time.monotonic()))
        try:
            if not acquired:raise RuntimeError('close operation deadline exhausted')
            # Reserve bounded cleanup time within the requested close budget.
            if self.process.is_alive():self._put(None,protocol_deadline)
            while True:
                remaining=protocol_deadline-time.monotonic()
                if remaining<=0:raise RuntimeError('bag close acknowledgement deadline exhausted')
                if self.responses.poll(min(.05,remaining)):
                    try:ack=self.responses.recv()
                    except (EOFError,OSError)as exc:raise RuntimeError('child exited without bag close confirmation')from exc
                    if ack[0]=='closed':
                        if ack[1]!=self.sequence:raise RuntimeError('bag close sequence mismatch')
                        self.close_confirmed=True;break
                    if ack[0]in ('error','close_error'):
                        self._fail('child %s sequence %s: %s: %s'%tuple(ack))
                        if ack[0]=='close_error':raise RuntimeError(self.failure)
                    elif ack[:2]==('written',self.sequence)and self.failure is not None:
                        # Timed-out write is still an explicit archive failure.
                        # A late ack cannot make its retained parent bytes vanish.
                        continue
                    else:raise RuntimeError('unexpected close acknowledgement: '+repr(ack))
                elif not self.process.is_alive()and not self.responses.poll(0):
                    raise RuntimeError('child exited without bag close confirmation')
            self.process.join(max(0,deadline-time.monotonic()))
            if self.process.is_alive():raise RuntimeError('child exit deadline exhausted')
        except BaseException as exc:self._fail(exc)
        finally:
            self._cleanup(deadline);self.closed=True
            if acquired:self.operation.release()
        if self.failure is not None:raise RuntimeError(self.failure)
        return self.summary()

    def summary(self):
        return dict(child_pid=self.process.pid,child_alive=self.process.is_alive(),
             start_method='spawn',sequence=self.sequence,written=self.written,
             maximum_payload_bytes=self.maximum_payload_bytes,
             maximum_envelope_overhead_bytes=MAX_ENVELOPE_OVERHEAD,
             maximum_inflight_messages=1,closed=self.closed,
             ipc_feeder_alive=bool(self.commands._thread is not None and self.commands._thread.is_alive()),
             bag_close_confirmed=self.close_confirmed,failure=self.failure)
