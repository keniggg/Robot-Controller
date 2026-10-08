"""Pure isolated tests using the installed ROS function bodies, never ROS init."""
import ast
import errno
import hashlib
import importlib.util
import io
import json
import logging
from pathlib import Path
import socket
import struct
from types import SimpleNamespace, ModuleType
import traceback
import pytest

PATH=Path(__file__).with_name('unknown_transport_trace_20261008.py')
spec=importlib.util.spec_from_file_location('unknown_trace_tested',PATH)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
ROS=Path('/opt/ros/noetic/lib/python3/dist-packages/rospy')


class OriginalError(Exception): pass
class DeserializationError(Exception): pass
class TransportException(Exception): pass
class TransportTerminated(Exception): pass


def original_function(path,name,class_name=None):
    tree=ast.parse(path.read_text())
    items=tree.body
    if class_name:items=next(x for x in items if isinstance(x,ast.ClassDef) and x.name==class_name).body
    node=next(x for x in items if isinstance(x,ast.FunctionDef) and x.name==name)
    return compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),str(path),'exec')


class Image:
    _type='sensor_msgs/Image'
    def deserialize(self, data):
        seq,secs,nsecs=struct.unpack_from('<III',data)
        self.header=SimpleNamespace(seq=seq,stamp=SimpleNamespace(secs=secs,nsecs=nsecs))
        self.data=data[12:]
        return self


def packet(seq=1,secs=100,nsecs=7,tail=b'pixels'):
    data=struct.pack('<III',seq,secs,nsecs)+tail
    return struct.pack('<I',len(data))+data


def bindings():
    tcp=ModuleType('fake_tcp')
    tcp.__dict__.update(struct=struct,logging=logging,traceback=traceback,socket=socket,
        EAGAIN=errno.EAGAIN,EWOULDBLOCK=errno.EWOULDBLOCK,
        genpy=SimpleNamespace(DeserializationError=DeserializationError),
        DeserializationError=DeserializationError,TransportException=TransportException,
        TransportTerminated=TransportTerminated,ServiceException=OriginalError,
        is_shutdown=lambda:False,rospyerr=lambda *args:None)
    exec(original_function(ROS/'msg.py','deserialize_messages'),tcp.__dict__)
    exec(original_function(ROS/'impl/tcpros_base.py','recv_buff'),tcp.__dict__)
    exec(original_function(ROS/'impl/tcpros_base.py','receive_once','TCPROSTransport'),tcp.__dict__)
    tcp.TCPROSTransport=type('TCPROSTransport',(),{'receive_once':tcp.receive_once})
    class Sub:
        def receive_callback(self,msgs,connection):
            return ('callback',msgs,connection)
    topic=SimpleNamespace(_SubscriberImpl=Sub)
    class Node:
        last_consumed_ns=0
        def frame(self,color,depth):return ('frame',color,depth)
        def process(self,_event):return _event
    return tcp,topic,Node


class Sock:
    def __init__(self,chunks):self.chunks=list(chunks);self.calls=0
    def setblocking(self,value):assert value==1
    def recv(self,size):
        self.calls+=1
        if not self.chunks:raise OriginalError('unexpected extra recv')
        value=self.chunks.pop(0)
        if isinstance(value,BaseException):raise value
        return value


def transport(tcp,chunks,topic='/color'):
    obj=tcp.TCPROSTransport();obj.socket=Sock(chunks);obj.read_buff=io.BytesIO()
    obj.done=False;obj.stat_bytes=0;obj.stat_num_msg=0;obj.header={'callerid':'original_camera'}
    obj.is_latched=False;obj.name=topic
    obj.protocol=SimpleNamespace(resolved_name=topic,recv_data_class=Image,buff_size=2**24)
    obj.protocol.read_messages=lambda b,q,s:tcp.deserialize_messages(b,q,Image,queue_size=2)
    return obj


def session(tmp_path):return m.TraceSession(tmp_path/'trace',('/color','/depth'),5.,2*1024*1024)


def finish(s):
    s.stop('test_complete');summary=s.finalize()
    rows=[json.loads(x) for x in (s.output_dir/'events.jsonl').read_text().splitlines()]
    assert summary['rows_written']==len(rows)
    return summary,rows


def test_actual_receive_framing_queue_pruning_and_restoration(tmp_path):
    tcp,topic,Node=bindings(); originals=(tcp.recv_buff,tcp.deserialize_messages,tcp.TCPROSTransport.receive_once,
        topic._SubscriberImpl.receive_callback,Node.frame,Node.process)
    s=session(tmp_path);m._install(s,tcp,topic,Node)
    # Four complete ROS messages in one recv: original queue_size=2 drops first two.
    obj=transport(tcp,[b''.join(packet(i) for i in range(4))]);result=obj.receive_once()
    assert [x.header.seq for x in result]==[2,3]
    callback=topic._SubscriberImpl().receive_callback; r=callback(result,obj)
    assert r[1] is result and r[2] is obj
    node=Node();assert node.frame(result[0],result[1])[1] is result[0]
    event=object();assert node.process(event) is event
    summary,rows=finish(s)
    assert summary['sealed'] and summary['observer_integrity_passed']
    assert originals==(tcp.recv_buff,tcp.deserialize_messages,tcp.TCPROSTransport.receive_once,
        topic._SubscriberImpl.receive_callback,Node.frame,Node.process)
    d=next(r for r in rows if r['kind']=='deserialize_messages')
    assert [x['seq'] for x in d['wire_before']['complete']]==[0,1,2,3]
    assert [x['seq'] for x in d['actual_headers']]==[2,3]
    assert {x['kind'] for x in rows}=={'receive_once','recv_buff','socket_recv','deserialize_messages',
                                     'receive_callback','node_frame','node_process'}
    assert all(x['elapsed_ns']>=0 and x['thread_cpu_ns']>=0 for x in rows)
    assert 'pixels' not in (s.output_dir/'events.jsonl').read_text()
    # A previously bound callback is dormant after restoration, no late events.
    count=s.rows_admitted;assert callback(result,obj)[1] is result;assert s.rows_admitted==count


def test_fragment_header_and_batch_offsets_without_cursor_mutation(tmp_path):
    tcp,topic,Node=bindings();s=session(tmp_path);m._install(s,tcp,topic,Node)
    p=packet(9);obj=transport(tcp,[p[:2],p[2:8],p[8:19],p[19:]])
    result=obj.receive_once();assert result[0].header.seq==9 and obj.socket.calls==4
    _,rows=finish(s)
    recv=[r for r in rows if r['kind']=='socket_recv']
    assert [(x['byte_start'],x['byte_end']) for x in recv]==[(0,2),(2,8),(8,19),(19,len(p))]
    d=[r for r in rows if r['kind']=='deserialize_messages']
    assert any(x['wire_before']['partial'] is not None for x in d)
    assert d[-1]['wire_before']['complete'][0]['byte_start']==0
    assert d[-1]['wire_before']['complete'][0]['byte_end']==len(p)


def test_wire_peek_releases_memoryview_and_handles_prefix():
    b=io.BytesIO();b.write(packet(3));pos=b.tell();r=m.wire_headers(b,len(b.getvalue()))
    assert b.tell()==pos and r['complete'][0]['seq']==3
    b.truncate(0) # Would BufferError if trace retained the memoryview.
    b.seek(0);b.write(b'\x04');assert m.wire_headers(b,1)['partial']['length_prefix_incomplete']


def test_non_target_is_exact_pass_through(tmp_path):
    tcp,topic,Node=bindings();s=session(tmp_path);m._install(s,tcp,topic,Node)
    obj=transport(tcp,[packet()],'/other');assert len(obj.receive_once())==1
    _,rows=finish(s);assert not rows


def test_original_exception_object_is_preserved(tmp_path):
    tcp,topic,Node=bindings();s=session(tmp_path);m._install(s,tcp,topic,Node)
    error=OriginalError('unchanged');obj=transport(tcp,[error])
    with pytest.raises(OriginalError) as e:obj.receive_once()
    assert e.value is error and obj.socket.calls==1
    _,rows=finish(s);assert any(x['original_exception']=='OriginalError' for x in rows)


def test_logger_failure_cannot_replace_return_or_exception(tmp_path,monkeypatch):
    s=session(tmp_path)
    def fail(*a,**k):raise RuntimeError('diagnostic failure')
    monkeypatch.setattr(s,'end',fail)
    marker=object();calls=[]
    assert m._call(s,{},lambda:(calls.append(1) or marker),(),{}) is marker
    err=OriginalError('original')
    def original():calls.append(2);raise err
    with pytest.raises(OriginalError) as e:m._call(s,{},original,(),{})
    assert e.value is err and calls==[1,2]
    finish(s)


def test_wire_peek_error_does_not_skip_original_deserializer(tmp_path,monkeypatch):
    tcp,topic,Node=bindings();s=session(tmp_path);m._install(s,tcp,topic,Node)
    monkeypatch.setattr(m,'wire_headers',lambda *a,**k:(_ for _ in ()).throw(ValueError('trace only')))
    obj=transport(tcp,[packet(4)]);assert obj.receive_once()[0].header.seq==4
    summary,_=finish(s);assert summary['counts']['wire_peek_error']==1


def test_stop_restores_only_owned_methods(tmp_path):
    tcp,topic,Node=bindings();s=session(tmp_path);m._install(s,tcp,topic,Node)
    other=lambda *a:None;tcp.recv_buff=other
    summary,_=finish(s);assert tcp.recv_buff is other
    assert summary['restore_conflicts']==['recv_buff'] and not summary['observer_integrity_passed']


def test_partial_install_restores_previous_patches(tmp_path):
    tcp,topic,Node=bindings();s=session(tmp_path);original=tcp.recv_buff
    del Node.process
    with pytest.raises(AttributeError):m._install(s,tcp,topic,Node)
    assert tcp.recv_buff is original
    assert s.finalize()['stop_reason']=='install_failed'


def test_deadline_keeps_original_callable(tmp_path):
    tcp,topic,Node=bindings();s=session(tmp_path);m._install(s,tcp,topic,Node)
    cached=Node().process;s.deadline_monotonic=m.time.monotonic()-1
    item=object();assert cached(item) is item
    summary,_=finish(s);assert summary['stop_reason']=='deadline'


def test_byte_budget_fails_closed_diagnostics_only(tmp_path):
    s=m.TraceSession(tmp_path/'trace',('/color','/depth'),5.,m.RESERVED_BYTES+1024)
    token=s.begin('test');s.end(token,diagnostic_text='a'*2048)
    summary,_=finish(s)
    assert not summary['observer_integrity_passed']
    assert summary['events_bytes']<=1024
    assert summary['counts']['not_written_byte_budget']==1


def test_event_budget_records_partial(tmp_path,monkeypatch):
    s=session(tmp_path);monkeypatch.setattr(m,'MAX_EVENTS',1)
    s.end(s.begin('one'));s.end(s.begin('two'))
    summary,rows=finish(s);assert len(rows)==1 and summary['stop_reason']=='event_budget'
    assert not summary['observer_integrity_passed']


def test_inflight_call_at_stop_is_explicit(tmp_path):
    s=session(tmp_path);row=s.begin('waiting_socket');s.stop('requested');s.end(row)
    summary,_=finish(s)
    assert summary['outstanding_calls'][0]['kind']=='waiting_socket'
    assert summary['counts']['ended_after_admission_closed']==1
    assert s.finalize() is summary
    stopped=summary['stopped_monotonic'];s.stop('second_stop')
    assert s.stopped_monotonic==stopped and s.stop_reason=='requested'


def test_limits_rejected_before_output(tmp_path):
    for duration,limit in [(91,1024*1024),(0,1024*1024),(30,65*1024*1024)]:
        with pytest.raises(ValueError):m.TraceSession(tmp_path/'not_created',('/color','/depth'),duration,limit)
    assert not (tmp_path/'not_created').exists()


def test_public_install_rejects_system_disk_without_importing_ros(tmp_path,monkeypatch):
    monkeypatch.setattr(m.importlib,'import_module',lambda _:(_ for _ in ()).throw(AssertionError('no ROS import expected')))
    with pytest.raises(ValueError,match='SanStand'):m.install_transport_trace(tmp_path/'trace',('/color','/depth'),object)


def test_report_hash_matches_exact_events(tmp_path):
    s=session(tmp_path);s.end(s.begin('one'));summary,_=finish(s)
    assert summary['events_sha256']==hashlib.sha256((s.output_dir/'events.jsonl').read_bytes()).hexdigest()
