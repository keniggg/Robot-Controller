#!/usr/bin/env python3
"""Quantify already captured receive-buffer metadata; no ROS imports or pixels."""
import argparse
from collections import defaultdict
import hashlib
import importlib.util
import json
import math
from pathlib import Path

PARENT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('receive_analysis',PARENT/'analyze_unknown_receive_experiment_20261008.py')
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)


def correlation(pairs):
    pairs=[(float(x),float(y)) for x,y in pairs if base.finite(x) and base.finite(y)]
    if len(pairs)<3:return dict(n=len(pairs),pearson=None,reason='fewer_than_three')
    xs,ys=zip(*pairs);mx=sum(xs)/len(xs);my=sum(ys)/len(ys)
    numerator=sum((x-mx)*(y-my) for x,y in pairs)
    divisor=math.sqrt(sum((x-mx)**2 for x in xs)*sum((y-my)**2 for y in ys))
    return dict(n=len(pairs),pearson=numerator/divisor if divisor else None,
                reason='descriptive_not_causal' if divisor else 'constant_input')


def clocks(e,edge):
    c=e[edge];return c['monotonic_ns'],c['thread_cpu_ns']


def byte_bin(size):
    for n in (64,256,512,1024,2048,4096):
        if size<=n*1024:return '<='+str(n)+'KiB'
    return '>4096KiB'


def quantify(events,phases):
    by_cycle=defaultdict(list)
    for e in events:
        by_cycle[(e.get('tid'),e.get('connection_id'),e.get('receive_cycle'))].append(e)
    rows=[];excluded=defaultdict(int);deserial=[]
    for e in events:
        if e['kind']=='deserialize_messages':
            w=e.get('wire_before',{});complete=w.get('complete',[])
            if not w.get('complete_count_exact'):
                excluded['inexact_deserialize_peek']+=1;continue
            full=w.get('complete_count',0);q=e.get('queue_size')
            copied_body_bytes=sum(v['payload_bytes'] for v in complete)
            retained=complete[-q:] if q else complete
            retained_bytes=sum(v['payload_bytes'] for v in retained)
            deserial.append(dict(call_id=e['call_id'],phase=base.phase_of(*base.event_time(e),phases),
                complete_messages=full,queue_size=q,copied_complete_body_bytes=copied_body_bytes,
                kept_complete_body_bytes=retained_bytes,
                discarded_complete_body_bytes=max(0,copied_body_bytes-retained_bytes),
                buffer_bytes=w.get('buffer_bytes'),partial_message=bool(w.get('partial')),
                wall_ms=e['elapsed_ns']*1e-6,cpu_ms=e['thread_cpu_ns']*1e-6))
        if e['kind']!='recv_buff' or e.get('original_exception') is not None:continue
        siblings=by_cycle[(e.get('tid'),e.get('connection_id'),e.get('receive_cycle'))]
        start,end=clocks(e,'start')[0],clocks(e,'end')[0]
        sockets=[v for v in siblings if v['kind']=='socket_recv' and
                 start<=clocks(v,'start')[0]<=clocks(v,'end')[0]<=end]
        if len(sockets)!=1:
            excluded['recv_without_unique_contained_socket']+=1;continue
        s=sockets[0]
        size=s.get('returned_bytes');before=e.get('buffer_bytes_before');after=e.get('buffer_bytes_after')
        if not isinstance(size,int) or not isinstance(before,int) or not isinstance(after,int) or after-before!=size or e.get('returned_bytes')!=size:
            excluded['recv_byte_invariant_failed']+=1;continue
        sw,sc=clocks(s,'end');ew,ec=clocks(e,'end')
        if ec<sc:excluded['cpu_clock_reverse']+=1;continue
        next_ds=sorted((v for v in siblings if v['kind']=='deserialize_messages' and clocks(v,'start')[0]>=end),
                       key=lambda v:clocks(v,'start')[0])
        rows.append(dict(call_id=e['call_id'],socket_call_id=s['call_id'],tid=e['tid'],connection_id=e.get('connection_id'),
            receive_cycle=e.get('receive_cycle'),topic=e.get('topic'),phase=base.phase_of(*base.event_time(e),phases),
            start_monotonic=start*1e-9,end_monotonic=end*1e-9,
            received_bytes=size,buffer_before_bytes=before,buffer_after_bytes=after,byte_bin=byte_bin(size),
            socket_wall_ms=s['elapsed_ns']*1e-6,socket_cpu_ms=s['thread_cpu_ns']*1e-6,
            recv_buff_wall_ms=e['elapsed_ns']*1e-6,recv_buff_cpu_ms=e['thread_cpu_ns']*1e-6,
            after_socket_wall_ms=(ew-sw)*1e-6,after_socket_cpu_ms=(ec-sc)*1e-6,
            after_socket_wall_minus_thread_cpu_ms=((ew-sw)-(ec-sc))*1e-6,
            following_deserialize_call_id=next_ds[0]['call_id'] if next_ds else None))
    bins=defaultdict(list)
    for r in rows:bins[(r['phase'],r['byte_bin'])].append(r)
    measures=('received_bytes','buffer_before_bytes','buffer_after_bytes','socket_wall_ms','socket_cpu_ms',
              'after_socket_wall_ms','after_socket_cpu_ms','after_socket_wall_minus_thread_cpu_ms')
    grouped=[dict(phase=k[0],received_byte_bin=k[1],count=len(v),statistics={f:base.stats([r[f] for r in v]) for f in measures}) for k,v in sorted(bins.items())]
    associations={}
    for phase in set(r['phase'] for r in rows):
        relevant=[r for r in rows if r['phase']==phase]
        associations[phase]={x+'_to_'+y:correlation([(r[x],r[y]) for r in relevant])
                             for x in ('received_bytes','buffer_before_bytes','buffer_after_bytes')
                             for y in ('after_socket_wall_ms','after_socket_cpu_ms')}
    ds_groups=defaultdict(list)
    for d in deserial:ds_groups[(d['phase'],d['complete_messages'])].append(d)
    return dict(matched_recv_count=len(rows),excluded=dict(excluded),groups=grouped,correlations=associations,
        largest_after_socket=sorted(rows,key=lambda r:r['after_socket_wall_ms'],reverse=True)[:30],
        largest_incoming_chunks=sorted(rows,key=lambda r:r['received_bytes'],reverse=True)[:15],
        deserialize_groups=[dict(phase=k[0],complete_messages=k[1],count=len(v),
            wall_ms=base.stats([r['wall_ms'] for r in v]),cpu_ms=base.stats([r['cpu_ms'] for r in v]),
            copied_complete_body_bytes=sum(r['copied_complete_body_bytes'] for r in v),
            discarded_complete_body_bytes=sum(r['discarded_complete_body_bytes'] for r in v)) for k,v in sorted(ds_groups.items())],
        largest_deserialize=sorted(deserial,key=lambda r:r['wall_ms'],reverse=True)[:20])


def analyze(case):
    case=Path(case);p=case/'shadow/transport';hashes=base.manifest(p)
    summary=json.loads(base.read(p/'summary.json'))
    if not summary.get('sealed') or summary.get('events_sha256')!=hashes.get('events.jsonl'):
        raise ValueError('Unsealed or unbound trace')
    events=base.jsonl(p/'events.jsonl')
    if len(events)!=summary['rows_written']:raise ValueError('Trace row count mismatch')
    rpath=case/'raw_load/ready.json';opath=case/'raw_load/outcome.json'
    r=json.loads(base.read(rpath));o=json.loads(base.read(opath))
    phases=base.make_phases(summary['started_monotonic'],min(summary['deadline_monotonic'],summary['stopped_monotonic']),r,o)
    return dict(input=str(case),trace_sha256=hashes,raw_boundaries_sha256={str(p.name):base.digest(p) for p in (rpath,opath)},
        full_trace_window=summary.get('stop_reason')=='deadline' and summary.get('observer_integrity_passed',False),
        raw_capture_valid=o.get('capture_valid'),phases=phases,analysis=quantify(events,phases))


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--input',type=Path,action='append',required=True);ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args()
    if a.output.exists():raise ValueError('New output required')
    report=dict(schema=1,kind='offline_unknown_receive_buffer_cost',source_sha256=base.digest(__file__),
        helper_sha256=base.digest(PARENT/'analyze_unknown_receive_experiment_20261008.py'),cases=[analyze(p) for p in a.input],
        limitations=['After-socket interval includes original BytesIO.write, scheduling, and tap metadata return/queue overhead. It is not measured pure copy time.',
        'Thread CPU includes user and kernel time; wall-minus-CPU is not a direct measurement of scheduler/GIL/network waiting.',
        'Byte correlations are descriptive for this finite mixed workload. Memory pressure and burst backlog co-vary; neither disks nor BytesIO are proved the sole cause.',
        'Copied/discarded bytes follow installed original rospy.deserialize_messages body-read-before-queue-prune semantics, not allocation profiling.',
        'All complete batch bodies are counted once per observed deserialize; incomplete fragments are not claimed copied as full packets.',
        'No pixels were read or decoded; both experiment traces ended early and retain incomplete-window status.'],problem_resolved=False)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    raw=(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode();a.output.write_bytes(raw)
    h=hashlib.sha256(raw).hexdigest();a.output.with_suffix(a.output.suffix+'.sha256').write_text(h+'  '+a.output.name+'\n')
    print(json.dumps(dict(output=str(a.output),sha256=h,cases=len(report['cases']))))


if __name__=='__main__':main()
