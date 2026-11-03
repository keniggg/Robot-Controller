#!/usr/bin/env python3
"""Offline association of sealed receive events with existing passive proc samples."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent


def module(name,filename):
    spec=importlib.util.spec_from_file_location(name,HERE/filename)
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m


def analyze(folder):
    trace=module('trace_analysis','analyze_unknown_receive_experiment_20261008.py')
    memory=module('memory_analysis','analyze_sdk_memory_pressure_20261007.py')
    folder=Path(folder).resolve();transport=folder/'shadow/transport'
    hashes=trace.manifest(transport)
    summary=json.loads(trace.read(transport/'summary.json'))
    if not summary.get('sealed') or summary.get('events_sha256')!=hashes.get('events.jsonl'):
        raise ValueError('Trace is not sealed and hash bound')
    events=trace.jsonl(transport/'events.jsonl')
    ready,outcome,rows,resource_hashes=memory.load_sealed(folder/'resources')
    same_boot=ready.get('boot_id')==outcome.get('boot_id_at_end') and bool(ready.get('boot_id'))
    selected=sorted((v for v in events if v['elapsed_ns']>=200_000_000),key=lambda v:v['elapsed_ns'],reverse=True)[:30]
    links=[]
    for e in selected:
        start,end=trace.event_time(e)
        association=memory.resource_association(rows['resources'],start,end,ready,same_boot)
        links.append(dict(event=e,resources=association))
    # Preserve the actual source/buffer chain around the largest receive_once.
    once=max((v for v in events if v['kind']=='receive_once'),key=lambda v:v['elapsed_ns'],default=None)
    chain=[]
    if once:
        for e in events:
            if e.get('connection_id')==once.get('connection_id') and e.get('receive_cycle')==once.get('receive_cycle'):
                chain.append(e)
        chain.sort(key=lambda v:v['start']['monotonic_ns'])
    return dict(schema=1,kind='offline_receive_resource_association',input=str(folder),
        source_sha256={str(p):trace.digest(p) for p in (Path(__file__),HERE/'analyze_unknown_receive_experiment_20261008.py',HERE/'analyze_sdk_memory_pressure_20261007.py')},
        trace_sha256=hashes,resources_sha256=resource_hashes,
        resources_capture_closed=outcome.get('capture_closed'),
        resources_observer_integrity_passed=outcome.get('observer_integrity_passed'),
        resources_stop_reason=outcome.get('stop_reason'),
        resources_boot_consistent=same_boot,resources_boot_id=ready.get('boot_id'),
        trace_pid=next((v.get('pid') for v in events),None),
        trace_pid_was_resource_watch_target=str(next((v.get('pid') for v in events),None)) in ready['bound_processes'],
        resource_bound_processes=ready['bound_processes'],trace_window_complete=bool(summary.get('observer_integrity_passed') and summary.get('stop_reason')=='deadline'),
        event_links=links,largest_receive_once_chain=chain,
        limitations=[
            'Partial sealed records remain partial. No full 80/90-second integrity is inferred from SHA validity.',
            'Proc counters use actual bracketing samples; gaps over two sampling periods are not bridged, and margins are reported.',
            'Global direct reclaim, compaction, THP and PSI are association evidence, not per-thread causal attribution.',
            'The bound original unknown process is not the shadow trace process. Its faults cannot be assigned to shadow.',
            'The interval after socket event end but before recv_buff end includes original BytesIO.write, scheduling, and tap metadata/queue overhead; it is not separately instrumented memcpy time.',
            'socket_recv wall and CPU include Python allocation/copy and return scheduling, not exclusively network waiting.',
            'The trace does not carry an independent boot_id; it is joined to resources on the same experiment host monotonic clock; resources own start/end boot IDs are verified.',
            'No ROS, images, bags or device controls are read or executed by this offline analysis.'],
        problem_resolved=False)


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--input',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args()
    if a.output.exists():raise ValueError('New output required')
    report=analyze(a.input);a.output.parent.mkdir(parents=True,exist_ok=True)
    raw=(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode();a.output.write_bytes(raw)
    h=hashlib.sha256(raw).hexdigest();a.output.with_suffix(a.output.suffix+'.sha256').write_text(h+'  '+a.output.name+'\n')
    print(json.dumps(dict(output=str(a.output),sha256=h,event_links=len(report['event_links']),same_boot=report['resources_boot_consistent'],shadow_pid_sampled=report['trace_pid_was_resource_watch_target'])))


if __name__=='__main__':main()
