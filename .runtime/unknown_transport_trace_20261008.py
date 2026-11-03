#!/usr/bin/env python3
"""Bounded metadata-only transport tap. Importing this module never starts ROS.

Install before constructing the original UnknownTabletopPerception class. All
original calls, buffers, queues, return values and exceptions remain authoritative.
No pixels, device queries, publishers, services, freshness changes, or controls.
"""
from collections import Counter
import hashlib
import importlib
import inspect
import json
import os
from pathlib import Path
import queue
import struct
import threading
import time

EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
MOUNT = Path('/media/zhuyupei/SanStand')
MAX_SECONDS = 90.
MAX_BYTES = 64*1024*1024
MAX_EVENTS = 120000
MAX_OPEN_CALLS = 128
MAX_CONNECTIONS = 16
MAX_WIRE_HEADERS = 64
RESERVED_BYTES = 256*1024


def _json(value):
    return (json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(',', ':'), allow_nan=False)+'\n').encode()


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _clock():
    return dict(monotonic_ns=time.monotonic_ns(), thread_cpu_ns=time.thread_time_ns(), wall_ns=time.time_ns())


def message_header(msg):
    """Copy only fixed-size numeric identity; never retain message/pixel references."""
    h = msg.header
    return dict(object_id=id(msg), seq=int(h.seq), source_stamp_ns=int(h.stamp.secs)*10**9+int(h.stamp.nsecs))


def wire_headers(buffer, received_bytes, *, start=0, max_msgs=None):
    """Read length/Header scalars without cursor changes or payload copies.

    Release the memoryview BEFORE the real deserializer resizes its BytesIO.
    A negative byte offset denotes bytes present before the observer saw recv.
    """
    total = buffer.tell()
    result = dict(buffer_bytes=total, complete_count=0, complete_count_exact=True,
                  complete=[], partial=None, stream_buffer_start=received_bytes-total)
    view = buffer.getbuffer()
    try:
        pos = int(start)
        if not 0 <= pos <= total:
            raise ValueError('invalid buffer start')
        for _ in range(MAX_WIRE_HEADERS):
            if total-pos < 4:
                if pos < total:
                    result['partial'] = dict(byte_start=received_bytes-total+pos,
                                             available_bytes=total-pos, length_prefix_incomplete=True)
                break
            size = struct.unpack_from('<I', view, pos)[0]
            end = pos+4+size
            item = dict(byte_start=received_bytes-total+pos, byte_end=received_bytes-total+end,
                        payload_bytes=size, available_bytes=min(total-pos, 4+size))
            if size >= 12 and total-pos >= 16:
                seq, sec, nsec = struct.unpack_from('<III', view, pos+4)
                item.update(seq=seq, source_stamp_ns=sec*10**9+nsec,
                            header_nsec_valid=nsec < 10**9)
            if end > total:
                result['partial'] = item
                break
            result['complete'].append(item); result['complete_count'] += 1
            pos = end
            if max_msgs and result['complete_count'] >= max_msgs:
                break
        else:
            result['complete_count_exact'] = pos >= total
            result['peek_limit_reached'] = pos < total
    finally:
        view.release()
    return result


class TraceSession:
    """Internal engine; public install adds external-storage and binding checks."""
    def __init__(self, output_dir, topics, duration_sec=90., maximum_bytes=MAX_BYTES):
        if not 0 < duration_sec <= MAX_SECONDS or not RESERVED_BYTES+1024 <= maximum_bytes <= MAX_BYTES:
            raise ValueError('trace duration/byte limit invalid')
        if len(topics) != 2 or len(set(topics)) != 2 or any(not isinstance(t,str) or not t.startswith('/') or len(t)>256 for t in topics):
            raise ValueError('exactly two distinct absolute image topics required')
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=False)
        self.topics = frozenset(topics)
        self.maximum_bytes = int(maximum_bytes)
        self.started_monotonic = time.monotonic()
        self.deadline_monotonic = self.started_monotonic+duration_sec
        self.pid = os.getpid()
        self.tls = threading.local()
        self.lock = threading.RLock()
        self.enabled = True
        self.stopped = False
        self.finalized = None
        self.stop_reason = None
        self.stopped_monotonic = None
        self.patches = []
        self.restore_conflicts = []
        self.connections = {}
        self.open_calls = {}
        self.calls_at_stop = []
        self.next_call = 0
        self.rows_admitted = 0
        self.rows_written = 0
        self.bytes_written = 0
        self.counts = Counter()
        self.errors = []
        self.events = queue.Queue(maxsize=1024)
        self.file = (self.output_dir/'events.jsonl').open('xb')
        self.digest = hashlib.sha256()
        self.writer = threading.Thread(target=self._write, name='unknown-transport-trace-writer', daemon=True)
        self.writer.start()
        self.timer = None
        self.sources = {}

    def active(self):
        if not self.enabled:
            return False
        if time.monotonic() >= self.deadline_monotonic:
            self.stop('deadline')
            return False
        return True

    def error(self, kind, exc=None):
        try:
            with self.lock:
                self.counts[kind] += 1
                if len(self.errors) < 16:
                    self.errors.append({'kind':kind, 'error_type':None if exc is None else type(exc).__name__})
        except Exception:
            pass

    def begin(self, kind, context=None, **data):
        try:
            if not self.active():
                return None
            row = dict(kind=kind, pid=self.pid, tid=threading.get_native_id(), start=_clock(), **data)
            if context is not None:
                row.update(connection_id=context['connection_id'], topic=context['topic'], receive_cycle=context.get('cycle'))
            with self.lock:
                if not self.enabled:
                    return None
                if len(self.open_calls) >= MAX_OPEN_CALLS:
                    self.stop('open_call_budget'); return None
                self.next_call += 1
                row['call_id'] = self.next_call
                self.open_calls[row['call_id']] = row
            return row
        except Exception as exc:
            self.error('begin_error', exc)
            return None

    def end(self, row, *, original_exception=None, **data):
        if row is None:
            return
        try:
            end = _clock()
            with self.lock:
                self.open_calls.pop(row['call_id'], None)
                if not self.enabled or end['monotonic_ns'] > int(self.deadline_monotonic*10**9):
                    self.counts['ended_after_admission_closed'] += 1
                    return
                row = dict(row, end=end, original_exception=original_exception, **data)
                row['elapsed_ns'] = end['monotonic_ns']-row['start']['monotonic_ns']
                row['thread_cpu_ns'] = end['thread_cpu_ns']-row['start']['thread_cpu_ns']
                if self.rows_admitted >= MAX_EVENTS:
                    self.stop('event_budget'); return
                try:
                    self.events.put_nowait(row)
                except queue.Full:
                    self.stop('queue_full'); return
                self.rows_admitted += 1
        except Exception as exc:
            self.error('end_error', exc)

    def _write(self):
        try:
            while not self.stopped or not self.events.empty():
                try:
                    row = self.events.get(timeout=.1)
                except queue.Empty:
                    continue
                try:
                    raw = _json(row)
                    if self.bytes_written+len(raw) > self.maximum_bytes-RESERVED_BYTES:
                        self.stop('byte_budget')
                        self.counts['not_written_byte_budget'] += 1
                        continue
                    self.file.write(raw); self.digest.update(raw)
                    self.bytes_written += len(raw); self.rows_written += 1
                finally:
                    self.events.task_done()
            self.file.flush()
        except Exception as exc:
            self.error('writer_error', exc)
            self.stop('writer_error')
        finally:
            try:
                self.file.close()
            except Exception as exc:
                self.error('writer_close_error', exc)

    def patch(self, owner, name, factory):
        original = getattr(owner, name)
        if hasattr(original, '_unknown_transport_trace_owner'):
            raise RuntimeError('another trace owns '+name)
        wrapper = factory(original)
        wrapper._unknown_transport_trace_owner = id(self)
        setattr(owner, name, wrapper)
        self.patches.append((owner, name, original, wrapper))

    def connection(self, transport):
        topic = transport.protocol.resolved_name
        if topic not in self.topics or getattr(transport.protocol.recv_data_class, '_type', None) != 'sensor_msgs/Image':
            return None
        key = (id(transport), id(transport.socket))
        with self.lock:
            if key not in self.connections:
                if len(self.connections) >= MAX_CONNECTIONS:
                    self.stop('connection_budget'); return None
                self.connections[key] = dict(connection_id=len(self.connections)+1, topic=topic,
                    transport_object_id=id(transport), socket_object_id=id(transport.socket),
                    received_bytes=0, cycle=0)
            ctx = self.connections[key]
            ctx['cycle'] += 1
            return ctx

    def stop(self, reason='requested'):
        """No join/network/ROS; identity-check restoration, safe for an in-flight call."""
        with self.lock:
            if self.stopped:
                return
            self.enabled = False
            self.stopped = True
            self.stop_reason = str(reason)[:128]
            self.stopped_monotonic = time.monotonic()
            # Keep only bounded scalar call identity at a capture boundary.
            # A pending deserializer may otherwise duplicate many peek headers.
            keys=('call_id','kind','pid','tid','start','topic','connection_id','receive_cycle')
            self.calls_at_stop = [{k:r[k] for k in keys if k in r} for r in self.open_calls.values()]
            for owner,name,original,wrapper in reversed(self.patches):
                if getattr(owner,name) is wrapper:
                    setattr(owner,name,original)
                else:
                    self.restore_conflicts.append(name)
            if self.timer is not None:
                self.timer.cancel()

    def finalize(self, timeout=3.):
        if self.finalized is not None:
            return self.finalized
        self.stop('finalize')
        self.writer.join(max(0., min(float(timeout), 3.)))
        sealed = not self.writer.is_alive() and self.events.empty() and self.rows_admitted == self.rows_written and not self.errors
        summary = dict(schema=1, kind='unknown_transport_trace', sealed=sealed,
            observer_integrity_passed=bool(sealed and not self.restore_conflicts and self.stop_reason not in
                ('open_call_budget','event_budget','queue_full','byte_budget','connection_budget','writer_error','install_failed')),
            summary_path=str(self.output_dir/'summary.json'), output_dir=str(self.output_dir),
            started_monotonic=self.started_monotonic, deadline_monotonic=self.deadline_monotonic,
            stop_reason=self.stop_reason, stopped_monotonic=self.stopped_monotonic,
            rows_admitted=self.rows_admitted, rows_written=self.rows_written,
            events_bytes=self.bytes_written, events_sha256=self.digest.hexdigest() if sealed else None,
            queued_unwritten=self.events.qsize(), writer_alive=self.writer.is_alive(),
            maximum_bytes=self.maximum_bytes, maximum_events=MAX_EVENTS,
            outstanding_calls=self.calls_at_stop, restore_conflicts=self.restore_conflicts,
            original_method_bindings_restored=not self.restore_conflicts,
            dormant_bound_wrappers_possible=True, counts=dict(self.counts), errors=list(self.errors),
            sources=self.sources, topics=sorted(self.topics), pixels_saved=False,
            limitations=['Existing bound callback references may still call a disabled straight-through wrapper until the mirror process exits.',
                'Socket recv duration includes scheduling/GIL reacquisition; it is not proof of pure network wait.',
                'A mirror process is extra load and cannot establish the original process scheduling/GIL cause.',
                'Queue pruning describes complete buffered messages, not kernel ingress time or publication time.',
                'Trace byte budget does not bound original process RSS or original image buffers.'])
        raw = _json(summary)
        (self.output_dir/'summary.json').write_bytes(raw)
        (self.output_dir/'SHA256SUMS').write_text(hashlib.sha256(raw).hexdigest()+'  summary.json\n'+
            (self.digest.hexdigest()+'  events.jsonl\n' if sealed else ''))
        self.finalized = summary
        return summary


def _safe_end(session, row, **kwargs):
    try:
        session.end(row, **kwargs)
    except Exception as exc:
        try: session.error('end_hook_error',exc)
        except Exception: pass


def _call(session, row, original, args, kwargs, finish=None):
    """Exactly one original call; logger failure cannot replace its exception/return."""
    try:
        value = original(*args, **kwargs)
    except BaseException as exc:
        _safe_end(session,row,original_exception=type(exc).__name__)
        raise
    else:
        extra = {}
        if finish is not None:
            try:
                extra = finish(value)
            except Exception as exc:
                session.error('finish_metadata_error', exc)
        _safe_end(session,row,**extra)
        return value


def _install(session, tcp, topic_module, node_class):
    """Kept separate for isolated tests using fake modules and original functions."""
    def once_factory(original):
        def wrapped(transport, *args, **kwargs):
            if not session.active(): return original(transport,*args,**kwargs)
            previous = getattr(session.tls,'receive',None)
            try:
                ctx = session.connection(transport)
            except Exception as exc:
                session.error('connection_metadata_error',exc);ctx=None
            if ctx is None:return original(transport,*args,**kwargs)
            session.tls.receive = ctx
            row = session.begin('receive_once',ctx)
            try:
                return _call(session,row,original,(transport,)+args,kwargs,
                    lambda msgs:dict(delivered_headers=[message_header(msg) for msg in msgs[:16]], delivered_count=len(msgs)))
            finally:
                session.tls.receive = previous
        return wrapped

    def recv_factory(original):
        def wrapped(sock, buffer, buff_size):
            ctx = getattr(session.tls,'receive',None)
            if ctx is None or not session.active():return original(sock,buffer,buff_size)
            try: before=buffer.tell()
            except Exception as exc: session.error('buffer_size_error',exc);before=None
            row = session.begin('recv_buff',ctx,requested_bytes=buff_size,buffer_bytes_before=before)
            class Proxy:
                def recv(self,*args,**kwargs):
                    call = session.begin('socket_recv',ctx,byte_start=ctx['received_bytes'])
                    def finish(value):
                        size=len(value);ctx['received_bytes']+=size
                        return dict(returned_bytes=size,byte_end=ctx['received_bytes'])
                    return _call(session,call,sock.recv,args,kwargs,finish)
                def __getattr__(self,name):return getattr(sock,name)
            return _call(session,row,original,(Proxy(),buffer,buff_size),{},
                lambda size:dict(returned_bytes=size,buffer_bytes_after=buffer.tell()))
        return wrapped

    def deserialize_factory(original):
        def wrapped(buffer,msg_queue,data_class,*args,**kwargs):
            ctx=getattr(session.tls,'receive',None)
            if ctx is None or not session.active():return original(buffer,msg_queue,data_class,*args,**kwargs)
            qsize=args[0] if args else kwargs.get('queue_size')
            max_msgs=args[1] if len(args)>1 else kwargs.get('max_msgs')
            start=args[2] if len(args)>2 else kwargs.get('start',0)
            try:
                peek=wire_headers(buffer,ctx['received_bytes'],start=start,max_msgs=max_msgs)
            except Exception as exc:
                session.error('wire_peek_error',exc);peek={'unavailable':type(exc).__name__}
            row=session.begin('deserialize_messages',ctx,queue_size=qsize,max_msgs=max_msgs,
                              queue_count_before=len(msg_queue),wire_before=peek)
            def finish(_):
                return dict(buffer_bytes_after=buffer.tell(),queue_count_after=len(msg_queue),
                            actual_headers=[message_header(msg) for msg in msg_queue[-16:]])
            return _call(session,row,original,(buffer,msg_queue,data_class)+args,kwargs,finish)
        return wrapped

    def callback_factory(original):
        def wrapped(subscriber,msgs,connection):
            topic=getattr(getattr(connection,'protocol',None),'resolved_name',None)
            if topic not in session.topics or not session.active():return original(subscriber,msgs,connection)
            try:
                headers=[message_header(msg) for msg in msgs[:16]]
            except Exception as exc:
                session.error('callback_header_error',exc);headers=[]
            try: ctx=session.connections.get((id(connection),id(connection.socket)))
            except Exception as exc:session.error('callback_connection_error',exc);ctx=None
            row=session.begin('receive_callback',ctx,topic=topic,headers=headers,message_count=len(msgs))
            return _call(session,row,original,(subscriber,msgs,connection),{})
        return wrapped

    def frame_factory(original):
        def wrapped(node,color,depth):
            if not session.active():return original(node,color,depth)
            try:headers=[message_header(color),message_header(depth)]
            except Exception as exc:session.error('frame_header_error',exc);headers=[]
            row=session.begin('node_frame',headers=headers)
            return _call(session,row,original,(node,color,depth),{})
        return wrapped

    def process_factory(original):
        def wrapped(node,event):
            if not session.active():return original(node,event)
            try:
                timer={k: None if getattr(event,k,None) is None else
                    int(getattr(event,k).secs)*10**9+int(getattr(event,k).nsecs)
                    for k in ('current_expected','current_real','last_expected','last_real')}
            except Exception as exc:session.error('timer_metadata_error',exc);timer={}
            row=session.begin('node_process',timer_ros_ns=timer)
            return _call(session,row,original,(node,event),{},
                lambda _:dict(last_consumed_source_after=getattr(node,'last_consumed_ns',None)))
        return wrapped

    try:
        session.patch(tcp.TCPROSTransport,'receive_once',once_factory)
        session.patch(tcp,'recv_buff',recv_factory)
        session.patch(tcp,'deserialize_messages',deserialize_factory)
        session.patch(topic_module._SubscriberImpl,'receive_callback',callback_factory)
        session.patch(node_class,'frame',frame_factory)
        session.patch(node_class,'process',process_factory)
    except BaseException:
        session.stop('install_failed')
        raise


def install_transport_trace(output_dir, topics, node_class, duration_sec=90., maximum_bytes=MAX_BYTES):
    """Install before constructing node/subscriptions; does not initialize ROS."""
    output=Path(output_dir).resolve()
    if not MOUNT.is_mount() or EXT.resolve() not in output.parents:
        raise ValueError('SanStand output required; no fallback')
    tcp=importlib.import_module('rospy.impl.tcpros_base')
    tm=importlib.import_module('rospy.topics')
    # Refuse a substantially different ABI instead of rewriting its receive loop.
    expected=[(tcp.recv_buff,('sock','b','buff_size')),
        (tcp.deserialize_messages,('b','msg_queue','data_class','queue_size','max_msgs','start')),
        (tcp.TCPROSTransport.receive_once,('self',)),
        (tm._SubscriberImpl.receive_callback,('self','msgs','connection')),
        (node_class.frame,('self','color','depth')),(node_class.process,('self','_event'))]
    for func,names in expected:
        if tuple(inspect.signature(func).parameters) != names:
            raise RuntimeError('transport tap ABI mismatch: '+str(func))
    session=TraceSession(output,topics,duration_sec,maximum_bytes)
    try:
        paths={Path(__file__).resolve(),Path(inspect.getfile(node_class.frame)).resolve(),Path(tcp.__file__).resolve(),
               Path(tm.__file__).resolve(),Path(inspect.getfile(tcp.deserialize_messages)).resolve()}
        session.sources={str(p):_sha(p) for p in paths}
        _install(session,tcp,tm,node_class)
        session.timer=threading.Timer(max(0.,session.deadline_monotonic-time.monotonic()),session.stop,args=('deadline',))
        session.timer.daemon=True;session.timer.start()
        (output/'ready.json').write_bytes(_json(dict(schema=1,started_monotonic=session.started_monotonic,
            deadline_monotonic=session.deadline_monotonic,topics=sorted(session.topics),sources=session.sources,
            maximum_bytes=maximum_bytes,pixels_saved=False,stage='installed_before_original_node_construction')))
        return session
    except BaseException:
        session.stop('install_failed');session.finalize();raise
