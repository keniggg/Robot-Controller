#!/usr/bin/env python3
"""Offline metadata-only analysis of a sealed isolated unknown receive experiment.

No ROS imports, image decoding, device access, or live operations. Phase boundaries
come from the recorder's actual capture/close observations, never planned timers.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path

MAX_FILE = 70*1024*1024
STAGES = ('receive_once', 'recv_buff', 'socket_recv', 'deserialize_messages',
          'receive_callback', 'node_frame', 'node_process')
PHASES = ('baseline', 'recording', 'closing', 'after', 'cross_phase', 'outside', 'unassigned')


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda: f.read(1024*1024), b''): h.update(b)
    return h.hexdigest()


def read(path):
    path = Path(path)
    if path.is_symlink() or path.stat().st_size > MAX_FILE:
        raise ValueError('Unsafe or oversized metadata: '+str(path))
    return path.read_bytes()


def manifest(directory, filename='SHA256SUMS'):
    directory = Path(directory)
    lines = read(directory/filename).decode().splitlines()
    result = {}
    for line in lines:
        parts = line.split(None, 1)
        if len(parts) != 2: raise ValueError('Malformed checksum manifest')
        h, name = parts[0], parts[1].lstrip('*')
        if len(h) != 64 or any(c not in '0123456789abcdef' for c in h):
            raise ValueError('Invalid checksum')
        if Path(name).name != name or name in result: raise ValueError('Unsafe/duplicate manifest path')
        if hashlib.sha256(read(directory/name)).hexdigest() != h:
            raise ValueError('Checksum mismatch: '+str(directory/name))
        result[name] = h
    return result


def jsonl(path):
    return [json.loads(line) for line in read(path).splitlines() if line.strip()]


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def stats(values):
    values = sorted(float(x) for x in values if finite(x))
    if not values: return dict(available=False, count=0, p50=None, p95=None, maximum=None)
    def percentile(p):
        pos = (len(values)-1)*p
        lo = int(pos); hi = min(lo+1, len(values)-1)
        return values[lo]+(values[hi]-values[lo])*(pos-lo)
    return dict(available=True, count=len(values), p50=percentile(.5), p95=percentile(.95), maximum=values[-1])


def make_phases(start, end, raw_ready, outcome):
    a = raw_ready['capture_start_monotonic']
    b, c = outcome['capture_end_monotonic'], outcome['completed_monotonic']
    if a != outcome['capture_start_monotonic']:
        raise ValueError('Recorder ready/outcome start mismatch')
    if raw_ready['capture_deadline_monotonic'] != outcome['capture_deadline_monotonic']:
        raise ValueError('Recorder ready/outcome deadline mismatch')
    if not all(finite(x) for x in (start,end,a,b,c)) or not start <= a <= b <= c <= end:
        raise ValueError('Actual recorder boundaries outside trace or reversed')
    return [dict(name=n, start_monotonic=s, end_monotonic=e) for n,s,e in
            [('baseline',start,a), ('recording',a,b), ('closing',b,c), ('after',c,end)]]


def phase_of(start, end, phases):
    if not phases or not finite(start) or not finite(end): return 'unassigned'
    if end < start: raise ValueError('Reversed event interval')
    # Point events on a boundary belong to the following phase. Completed
    # intervals ending on the boundary belong to the preceding phase.
    for p in phases:
        a,b = p['start_monotonic'],p['end_monotonic']
        if a <= start < b and end <= b: return p['name']
    if end <= phases[0]['start_monotonic'] or start >= phases[-1]['end_monotonic']: return 'outside'
    return 'cross_phase'


def event_time(event):
    return event['start']['monotonic_ns']*1e-9, event['end']['monotonic_ns']*1e-9


def headers(event):
    return event.get('headers',event.get('actual_headers',event.get('delivered_headers',[])))


def brief(event, phases):
    start,end = event_time(event)
    return {**{k:event.get(k) for k in ('call_id','kind','tid','connection_id','receive_cycle','topic')},
            'start_monotonic':start,'end_monotonic':end,'phase':phase_of(start,end,phases),
            'wall_ms':event['elapsed_ns']*1e-6,'thread_cpu_ms':event['thread_cpu_ns']*1e-6,
            'sources':[h.get('source_stamp_ns') for h in headers(event)],
            'original_exception':event.get('original_exception')}


def analyze_events(events, phases):
    groups = defaultdict(lambda: {'wall':[], 'cpu':[]})
    age_groups = defaultdict(list)
    callback_rows = []
    timer_lateness = defaultdict(list)
    batches = Counter()
    for e in events:
        a,b = event_time(e)
        phase = phase_of(a,b,phases)
        group = groups[(phase,e['kind'])]
        group['wall'].append(e['elapsed_ns']*1e-6); group['cpu'].append(e['thread_cpu_ns']*1e-6)
        if e['kind'] in ('receive_callback','node_frame'):
            for h in headers(e):
                source = h.get('source_stamp_ns')
                if not isinstance(source,int) or source <= 0: continue
                age = (e['start']['wall_ns']-source)*1e-9
                key = (phase, e['kind'], e.get('topic','exact_rgbd_pair'))
                age_groups[key].append(age)
                callback_rows.append(dict(call_id=e['call_id'],kind=e['kind'],topic=e.get('topic'),
                    source_stamp_ns=source,arrival_monotonic=a,source_age_sec=age,phase=phase))
        if e['kind'] == 'node_process':
            timer=e.get('timer_ros_ns',{})
            if all(isinstance(timer.get(k),int) for k in ('current_expected','current_real')):
                timer_lateness[phase].append((timer['current_real']-timer['current_expected'])*1e-6)
        if e['kind']=='deserialize_messages':
            wire=e.get('wire_before',{})
            batches['calls']+=1
            batches['with_partial_message']+=bool(wire.get('partial'))
            batches['peek_inexact']+=not wire.get('complete_count_exact',False)
            if wire.get('complete_count_exact'):
                batches['complete_messages_seen']+=wire.get('complete_count',0)
                batches['delivered_messages_after_queue']+=len(e.get('actual_headers',[]))
                batches['complete_batch_exceeds_configured_queue']+=bool(e.get('queue_size') and wire.get('complete_count',0)>e['queue_size'])
    per_phase = {p:{s:dict(wall_ms=stats(groups[(p,s)]['wall']),thread_cpu_ms=stats(groups[(p,s)]['cpu']))
                   for s in STAGES} for p in PHASES}
    long_events=[]
    for e in sorted((v for v in events if v['elapsed_ns']>=200_000_000),key=lambda v:v['elapsed_ns'],reverse=True)[:30]:
        a,b=event_time(e); related=defaultdict(list)
        for v in events:
            if v is e: continue
            va,vb=event_time(v)
            if vb < a or va > b: continue
            if (v.get('tid')==e.get('tid') or
                (e.get('connection_id') is not None and v.get('connection_id')==e.get('connection_id')) or
                v['kind']=='node_process' or e['kind']=='node_process'):
                related[v['kind']].append(v)
        item=brief(e,phases)
        item['overlapping_related_stages']={k:{'count':len(v),'longest':[brief(x,phases) for x in
            sorted(v,key=lambda z:z['elapsed_ns'],reverse=True)[:3]]} for k,v in related.items()}
        item['association_only_not_causation']=True
        long_events.append(item)
    return dict(per_phase=per_phase,
        callback_source_age_sec=[dict(phase=k[0],stage=k[1],topic=k[2],statistics=stats(v)) for k,v in sorted(age_groups.items())],
        callback_age_largest=sorted(callback_rows,key=lambda v:v['source_age_sec'],reverse=True)[:30],
        timer_reported_lateness_ms={k:stats(v) for k,v in timer_lateness.items()},
        buffered_batch_observations=dict(batches),long_events=long_events,
        outstanding_calls_not_in_completed_stage_statistics=True)


def status_identity(status, declared_kind, ready):
    for kind, key in [('shadow','private_selection'), ('production','production_selection_before')]:
        selected=ready.get(key,{})
        if (str(status.get('generation')) == str(selected.get('generation')) and
                status.get('selection_stamp_ns') is not None and
                str(status['selection_stamp_ns']) == str(selected.get('stamp_ns'))):
            return kind
    return 'unclassified'


def analyze_statuses(rows, phases, events, ready):
    groups=defaultdict(Counter); raw={}; processing=defaultdict(list); diagnostics=[]
    identity_counts=Counter(); mismatches=[]
    source_callbacks=defaultdict(list)
    for e in events:
        if e['kind']=='receive_callback':
            for h in headers(e):
                source_callbacks[h.get('source_stamp_ns')].append(e)
    for row in rows:
        text=row['original_status_data']
        if hashlib.sha256(text.encode()).hexdigest()!=row['original_status_utf8_sha256']:
            raise ValueError('Original status payload checksum mismatch')
        status=json.loads(text); declared=row['kind']; receipt=row['receipt']['monotonic']
        kind=status_identity(status,declared,ready)
        identity_counts[(declared,kind)]+=1
        if kind not in ('unclassified',declared):
            mismatches.append(dict(declared_kind=declared,evidence_kind=kind,
                generation=status.get('generation'),selection_stamp_ns=status.get('selection_stamp_ns'),receipt_monotonic=receipt))
        phase=phase_of(receipt,receipt,phases)
        groups[(kind,phase)][str(status.get('state'))+'|'+str(status.get('detail'))]+=1
        timing=status.get('frame_timing') or {}
        key_source=timing.get('source_stamp_ns')
        if finite(timing.get('processing_wall_sec')):
            begin=timing.get('processing_start_monotonic_sec')
            if finite(begin):
                end=begin+timing['processing_wall_sec']
                processing[(kind,phase_of(begin,end,phases))].append(timing['processing_wall_sec']*1000)
        arrivals=list((timing.get('raw_arrivals') or {}).items())
        arrivals+=list(((status.get('rgbd_ingress_timing') or {}).get('latest_raw_arrivals') or {}).items())
        for stream,v in arrivals:
            if not v or not finite(v.get('arrival_monotonic_sec')): continue
            source=v.get('source_stamp_ns',key_source)
            key=(kind,stream,source,v['arrival_monotonic_sec'])
            if key not in raw:
                at=v['arrival_monotonic_sec']
                raw[key]=dict(kind=kind,stream=stream,source_stamp_ns=source,arrival_monotonic=at,
                    source_age_sec=v.get('source_age_at_arrival_sec'),phase=phase_of(at,at,phases),original=v)
        if status.get('target_lost') or status.get('state') in ('error','lost'):
            diagnostics.append(dict(kind=kind,receipt_monotonic=receipt,phase=phase,
                generation=status.get('generation'),detail=status.get('detail'),
                source_stamp_ns=status.get('source_stamp_ns'),frame_timing=timing,
                rgbd_ingress_timing=status.get('rgbd_ingress_timing')))
    raw_groups=defaultdict(list)
    for v in raw.values():raw_groups[(v['kind'],v['phase'],v['stream'])].append(v['source_age_sec'])
    largest=sorted((v for v in raw.values() if finite(v['source_age_sec'])),key=lambda v:v['source_age_sec'],reverse=True)[:30]
    for row in largest:
        matched=source_callbacks.get(row['source_stamp_ns'],[]) if row['kind']=='shadow' else []
        row['same_source_shadow_callbacks']=[brief(e,phases) for e in matched[:4]]
    return dict(identity_label_mismatch_count=len(mismatches),identity_label_mismatch_examples=mismatches[:3],
        identity_counts=[dict(declared_kind=k[0],evidence_kind=k[1],count=v) for k,v in sorted(identity_counts.items())],
        classification_uses_original_generation_and_selection_stamp=True,
        counts=[dict(kind=k[0],receipt_phase=k[1],states_details=dict(v)) for k,v in sorted(groups.items())],
        raw_arrival_age_sec=[dict(kind=k[0],phase=k[1],stream=k[2],statistics=stats(v)) for k,v in sorted(raw_groups.items())],
        raw_arrival_largest=largest,unique_raw_arrivals=len(raw),
        processing_wall_ms=[dict(kind=k[0],phase=k[1],statistics=stats(v)) for k,v in sorted(processing.items())],
        error_or_lost_status_count=len(diagnostics),first_error_or_lost=diagnostics[:8],
        raw_arrival_phase_uses_original_callback_monotonic=True)


def analyze(directory):
    directory=Path(directory).resolve(); shadow=directory/'shadow'; transport=shadow/'transport'
    errors=[]; warnings=[]; bindings={}
    def load(p):
        raw=read(p);bindings[str(p.relative_to(directory))]=hashlib.sha256(raw).hexdigest()
        return json.loads(raw)
    sm=manifest(shadow); tm=manifest(transport)
    for name in ('summary.json','ready.json','detector_status.jsonl'):
        if name not in sm:raise ValueError('Missing shadow manifest member: '+name)
    for name in ('summary.json','events.jsonl'):
        if name not in tm:raise ValueError('Missing trace manifest member: '+name)
    summary=load(shadow/'summary.json'); ready=load(shadow/'ready.json'); trace=load(transport/'summary.json')
    events=jsonl(transport/'events.jsonl'); statuses=jsonl(shadow/'detector_status.jsonl')
    if summary.get('trace') != trace:errors.append('Wrapper and independent trace summaries differ')
    if trace.get('events_sha256')!=tm['events.jsonl']:errors.append('Trace events hash differs')
    if trace.get('rows_written')!=len(events):errors.append('Trace row count differs')
    if trace.get('events_bytes')!=(transport/'events.jsonl').stat().st_size:errors.append('Trace byte count differs')
    if summary.get('statuses',{}).get('sha256')!=sm['detector_status.jsonl']:errors.append('Status hash differs')
    if summary.get('statuses',{}).get('rows')!=len(statuses):errors.append('Status row count differs')
    if summary.get('statuses',{}).get('bytes')!=(shadow/'detector_status.jsonl').stat().st_size:errors.append('Status byte count differs')
    start,end=trace['started_monotonic'],trace['deadline_monotonic']
    full=bool(summary.get('complete') and summary.get('trace_full_window_passed') and
        trace.get('sealed') and trace.get('observer_integrity_passed') and
        trace.get('original_method_bindings_restored') and not trace.get('restore_conflicts') and
        trace.get('stop_reason') in ('deadline','duration_limit') and
        finite(trace.get('stopped_monotonic')) and trace['stopped_monotonic']>=end)
    if not full:errors.append('Sealed recording is not a verified full, intact trace window')
    if ready.get('started_monotonic')!=start or ready.get('deadline_monotonic')!=end:
        errors.append('Wrapper ready trace boundaries differ')
    phases=[]; outcome={}; raw_ready={}; experiment={}
    if (directory/'experiment.sha256').is_file():
        em=manifest(directory,'experiment.sha256');experiment=load(directory/'experiment.json')
        if not experiment.get('orchestration_completed'):errors.append('Orchestration not completed')
        if not experiment.get('production_unknown_identity_preserved'):errors.append('Production process identity not preserved')
    else:warnings.append('No orchestration checksum available')
    if (directory/'raw_load/outcome.json').is_file() and (directory/'raw_load/ready.json').is_file():
        outcome=load(directory/'raw_load/outcome.json');raw_ready=load(directory/'raw_load/ready.json')
        if not outcome.get('bag_closed') or not outcome.get('capture_valid') or outcome.get('errors'):
            errors.append('Raw recorder not closed/valid')
        child=experiment.get('children',{}).get('raw_load',{})
        if child and child.get('ready')!=raw_ready:errors.append('Observed recorder ready differs from original')
        if child and child.get('returncode')!=0:errors.append('Raw recorder return code is not zero')
        try:phases=make_phases(start,min(end,trace.get('stopped_monotonic',end)),raw_ready,outcome)
        except (KeyError,ValueError) as exc:errors.append('Raw phase boundaries: '+str(exc))
    else:errors.append('Actual raw recorder boundaries unavailable; phases unassigned')
    events.sort(key=lambda e:e['start']['monotonic_ns'])
    status_report=analyze_statuses(statuses,phases,events,ready)
    if status_report['identity_label_mismatch_count']:
        errors.append('Status labels conflict with actual selection identity; original labels preserved, no valid production/shadow comparison')
    result=dict(schema=1,kind='offline_unknown_receive_experiment',input=str(directory),
        analysis_source_sha256=digest(__file__),input_integrity_passed=not errors,
        errors=errors,warnings=warnings,full_trace_window_passed=full,
        input_sha256=bindings,shadow_manifest=sm,trace_manifest=tm,
        trace_window=dict(start_monotonic=start,deadline_monotonic=end,stopped_monotonic=trace.get('stopped_monotonic')),
        phases=phases,event_count=len(events),status_count=len(statuses),
        trace=analyze_events(events,phases),statuses=status_report,
        raw_load=dict(ready=raw_ready,outcome={k:v for k,v in outcome.items() if k not in ('pairs','receive')},
            metadata_pair_count=len(outcome.get('pairs',[])),
            boundaries_hashed_at_analysis=True,original_outcome_has_capture_manifest=False,
            bag_content_not_read_or_independently_verified=True),
        outstanding_trace_calls=trace.get('outstanding_calls',[]),
        limitations=[
            'This traces a separate shadow process. Its added subscriptions and algorithm work alter system load; original-process GIL/network stall cause is not proved.',
            'socket_recv wall time includes blocked receive, scheduling and GIL reacquisition. Low thread CPU alone cannot distinguish those.',
            'Source age uses the SDK host read-completion stamp, not an exposure timestamp. No clocks are shifted or source stamps renewed.',
            'Nested receive/deserialize/callback/Timer durations overlap and must not be added. Temporal overlap is association only.',
            'Only fully phase-contained intervals enter that phase statistics; crossing intervals are listed separately.',
            'Closing spans capture_end through completed_monotonic, including queue drain, bag close and report construction; it is not an exact fsync-only timing.',
            'Raw recording B adds serialization, compression, memory and I/O. It is not a disk-only intervention.',
            'No missing measurements or short clean window prove health or resolution. No images or bag payloads were decoded.',
            'Original raw outcome has no separate capture-time checksum manifest; its unchanged bytes are bound now by this analysis, while ready also matches the sealed orchestrator.'])
    return result


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--input',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args()
    if a.output.exists():raise ValueError('New analysis output required')
    report=analyze(a.input)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    a.output.with_suffix(a.output.suffix+'.sha256').write_text(digest(a.output)+'  '+a.output.name+'\n')
    print(json.dumps(dict(output=str(a.output),sha256=digest(a.output),valid=report['input_integrity_passed'],
                         events=report['event_count'],statuses=report['status_count'],errors=report['errors'])))
    return 0 if report['input_integrity_passed'] else 2


if __name__=='__main__':raise SystemExit(main())
