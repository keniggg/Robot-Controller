#!/usr/bin/env python3
"""Bounded synthetic replay of installed rospy buffer append/deserialize bodies.

No ROS initialization, connections, SDK, devices, or real images. The fake recv
returns prebuilt bytes, so results exclude socket allocation, network and live
memory pressure. Performance is diagnostic, never a quality/acceptance gate.
"""
import argparse
import ast
import errno
import hashlib
import importlib
import importlib.util
import io
import json
import logging
from pathlib import Path
import resource
import socket
import struct
import sys
import time
import tracemalloc
import traceback
from types import SimpleNamespace

ROS=Path('/opt/ros/noetic/lib/python3/dist-packages')
MAX_BYTES=16*1024*1024


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def original_bindings():
    if str(ROS) not in sys.path:sys.path.insert(0,str(ROS))
    Image=importlib.import_module('sensor_msgs.msg').Image
    genpy=importlib.import_module('genpy')
    class TransportTerminated(Exception):pass
    env=dict(struct=struct,socket=socket,logging=logging,traceback=traceback,EAGAIN=errno.EAGAIN,EWOULDBLOCK=errno.EWOULDBLOCK,
             TransportTerminated=TransportTerminated,genpy=genpy,DeserializationError=genpy.DeserializationError)
    paths=[ROS/'rospy/msg.py',ROS/'rospy/impl/tcpros_base.py',ROS/'sensor_msgs/msg/_Image.py']
    for p,name in [(paths[0],'deserialize_messages'),(paths[1],'recv_buff')]:
        tree=ast.parse(p.read_text());node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name==name)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),str(p),'exec'),env)
    return env['recv_buff'],env['deserialize_messages'],Image,{str(p):sha(p) for p in paths}


def synthetic_packet(wire_bytes,seq):
    # 1-row mono8 data makes an honest complete Image of exactly the observed
    # byte count. It does not purport to reconstruct original RGB-D pixels/shape.
    overhead=56 # length prefix + Header(camera_link) + image scalar/string fields
    if not overhead < wire_bytes <= MAX_BYTES:raise ValueError('invalid packet size')
    n=wire_bytes-overhead;frame=b'camera_link';encoding=b'8UC1'
    body=(struct.pack('<III',seq,100,seq)+struct.pack('<I',len(frame))+frame+
          struct.pack('<II',1,n)+struct.pack('<I',len(encoding))+encoding+
          struct.pack('<BII',0,n,n)+bytes(n))
    packet=struct.pack('<I',len(body))+body
    if len(packet)!=wire_bytes:raise AssertionError('packet length differs')
    return packet


def clock():
    r=resource.getrusage(resource.RUSAGE_THREAD)
    return dict(monotonic_ns=time.monotonic_ns(),cpu_ns=time.thread_time_ns(),minor_faults=r.ru_minflt,major_faults=r.ru_majflt)


def change(a,b):return {k:b[k]-a[k] for k in a}


def status_memory():
    wanted={'VmRSS','VmHWM','VmSwap'}
    result={}
    for line in Path('/proc/self/status').read_text().splitlines():
        key,_,value=line.partition(':')
        if key in wanted:result[key]=value.strip()
    return result


def run_once(wire, prefix, count, bindings, residual=b''):
    recv,deserialize,Image,_=bindings
    b=io.BytesIO();b.write(wire[:prefix]);incoming=wire[prefix:]
    sock=SimpleNamespace(recv=lambda n:incoming)
    q=[];start=clock();returned=recv(sock,b,16*1024*1024);after_append=clock()
    deserialize(b,q,Image,queue_size=2);after_deserialize=clock()
    expected=list(range(max(0,count-2),count))
    if returned!=len(incoming) or [v.header.seq for v in q]!=expected or b.tell()!=len(residual) or b.getvalue()!=residual:
        raise AssertionError('original queue/source/residual semantics changed')
    if any(v.height!=1 or v.encoding!='8UC1' or v.step!=len(v.data) or any(v.data) for v in q):
        raise AssertionError('synthetic Image contents changed')
    proof=hashlib.sha256()
    for v in q:
        proof.update(json.dumps([v.header.seq,v.header.stamp.secs,v.header.stamp.nsecs,v.header.frame_id,
            v.height,v.width,v.encoding,v.is_bigendian,v.step],separators=(',',':')).encode())
        proof.update(v.data)
    proof.update(residual)
    return dict(append=change(start,after_append),deserialize=change(after_append,after_deserialize),
        returned_bytes=returned,retained_sequences=expected,residual_bytes=len(residual),output_sha256=proof.hexdigest())


def benchmark(wire_bytes,count,prefix,iterations=5,deadline_sec=3.,tail_bytes=0,candidate_path=None):
    if not 1<=count<=64 or not 1<=iterations<=30 or not 0<deadline_sec<=5:
        raise ValueError('bounded replay arguments required')
    if wire_bytes*count+tail_bytes>MAX_BYTES or not 0<=tail_bytes<wire_bytes or not 0<=prefix<wire_bytes*count+tail_bytes:
        raise ValueError('aggregate <=16MiB and valid prefix required')
    bindings=original_bindings();candidate=None
    source_hashes=dict(bindings[3])
    if candidate_path is not None:
        candidate_path=Path(candidate_path).resolve()
        spec=importlib.util.spec_from_file_location('pruned_replay_candidate',candidate_path)
        candidate_module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=candidate_module;spec.loader.exec_module(candidate_module)
        candidate=candidate_module.make_candidate(bindings[1],bindings[2])
        bindings=(bindings[0],candidate,bindings[2],bindings[3]);source_hashes[str(candidate_path)]=sha(candidate_path)
    residual=synthetic_packet(wire_bytes,count)[:tail_bytes] if tail_bytes else b''
    wire=b''.join(synthetic_packet(wire_bytes,i) for i in range(count))+residual
    memory_before=status_memory()
    deadline=time.monotonic()+deadline_sec;rows=[]
    for i in range(iterations):
        if time.monotonic()>=deadline:break
        rows.append(dict(iteration=i,**run_once(wire,prefix,count,bindings,residual)))
    # Separate from all timed samples: tracemalloc changes allocation cost.
    timed_statistics=None if candidate is None else candidate.snapshot_statistics()
    peak=None
    if time.monotonic()<deadline:
        tracemalloc.start()
        try:
            run_once(wire,prefix,count,bindings,residual);current,peak=tracemalloc.get_traced_memory()
        finally:tracemalloc.stop()
    return dict(schema=1,kind='synthetic_original_rospy_copy_replay',original_source_sha256=source_hashes,
        variant='candidate_skip_pruned' if candidate else 'original',
        candidate_statistics=None if candidate is None else candidate.snapshot_statistics(),
        timed_candidate_statistics=timed_statistics,
        final_statistics_include_separate_allocation_probe=peak is not None,
        process_memory_before=memory_before,process_memory_after=status_memory(),
        process_memory_scope='Current VmRSS/VmSwap and exec-lifetime VmHWM; not whole machine or precise per-function peak.',
        replay_source_sha256=sha(__file__),wire_bytes_per_message=wire_bytes,complete_messages=count,
        prefix_bytes=prefix,tail_bytes=tail_bytes,incoming_bytes=len(wire)-prefix,buffer_after_append_bytes=len(wire),
        queue_size_unchanged=2,requested_iterations=iterations,actual_iterations=len(rows),
        deadline_sec=deadline_sec,all_requested_completed=len(rows)==iterations,rows=rows,
        separate_python_allocation_probe_peak_bytes=peak,
        limits=['Synthetic legitimate mono8 Image messages preserve observed wire lengths, not original geometry/pixels.',
            'Fake recv returns already allocated bytes; socket allocation/USB/network and live memory pressure are not reproduced.',
            'Original recv_buff is unchanged. The reported variant either uses installed deserialize AST or the frozen candidate; both use original Image.deserialize without ROS initialization.',
            'After-append metrics isolate this synthetic buffer case and exclude live tap overhead. They cannot explain the live wall/CPU remainder uniquely.',
            'Thread rusage fault counters and wall/CPU clock calls add small measurement overhead; no production gates or queues are changed.',
            'Tracemalloc peak is a separate untimed Python-allocation probe, not RSS or total native resident memory.'],problem_resolved=False)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--wire-bytes',type=int,required=True);ap.add_argument('--complete-messages',type=int,required=True)
    ap.add_argument('--prefix-bytes',type=int,required=True);ap.add_argument('--iterations',type=int,default=5)
    ap.add_argument('--tail-bytes',type=int,default=0);ap.add_argument('--candidate',type=Path);
    ap.add_argument('--deadline-sec',type=float,default=3.);ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args()
    if a.output.exists():raise ValueError('new output required')
    report=benchmark(a.wire_bytes,a.complete_messages,a.prefix_bytes,a.iterations,a.deadline_sec,a.tail_bytes,a.candidate)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    data=(json.dumps(report,ensure_ascii=False,indent=2)+'\n').encode();a.output.write_bytes(data)
    digest=hashlib.sha256(data).hexdigest();a.output.with_suffix(a.output.suffix+'.sha256').write_text(digest+'  '+a.output.name+'\n')
    print(json.dumps(dict(output=str(a.output),sha256=digest,iterations=report['actual_iterations'])))


if __name__=='__main__':main()
