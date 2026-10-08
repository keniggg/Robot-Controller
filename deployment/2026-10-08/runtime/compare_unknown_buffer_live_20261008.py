#!/usr/bin/env python3
"""Offline comparison of three sealed diagnostic shadow rounds; no live actions."""
import argparse
from collections import Counter,defaultdict
import datetime
import hashlib
import importlib.util
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent


def module(name,filename):
    s=importlib.util.spec_from_file_location(name,HERE/filename);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m


base=module('receive_comparison_base','analyze_unknown_receive_experiment_20261008.py')
mem=module('receive_comparison_memory','analyze_sdk_memory_pressure_20261007.py')


def bound_json(path,expected=None):
    raw=base.read(path);h=hashlib.sha256(raw).hexdigest()
    if expected and h!=expected:raise ValueError('Changed sealed input '+str(path))
    return json.loads(raw),h


def save(path,value):
    if path.exists():raise ValueError('New output required: '+str(path))
    raw=(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode();path.write_bytes(raw)
    h=hashlib.sha256(raw).hexdigest();path.with_suffix(path.suffix+'.sha256').write_text(h+'  '+path.name+'\n');return h


def sdk_statistics(case,phases):
    ready,outcome,rows,hashes=mem.load_sealed(case/'resources')
    entries=[]
    for row in rows['sdk']:
        entry=row['original_entry']
        if mem.sha(mem.canonical(entry))!=row['entry_sha256']:raise ValueError('SDK entry hash differs')
        entries.append(entry)
    entries.sort(key=lambda e:e['read_completed_monotonic'])
    groups=defaultdict(lambda:defaultdict(list));counts=defaultdict(Counter);prior=None
    for e in entries:
        finish=e['read_completed_monotonic'];ev=e['evidence'];wait=ev.get('host_wait',{})
        start=wait.get('enter',{}).get('monotonic_sec');exit=wait.get('exit',{}).get('monotonic_sec')
        phase=base.phase_of(start,finish,phases)
        pointphase=base.phase_of(finish,finish,phases)
        counts[pointphase]['read_completions']+=1
        if base.finite(start) and base.finite(exit):
            wp=base.phase_of(start,exit,phases)
            groups[wp]['host_wait_sec'].append(wait.get('elapsed_sec'))
            groups[wp]['host_wait_thread_cpu_sec'].append(wait.get('thread_cpu_elapsed_sec'))
        groups[phase]['reported_read_total_sec'].append(ev.get('total_sec'))
        for k,v in ev.get('stages_sec',{}).items():groups[phase]['stage_'+k+'_sec'].append(v)
        counts[pointphase]['reset:'+str(ev.get('regional_stationary_evidence',{}).get('global_history_reset_reason'))]+=1
        owner=(e.get('geometry_generation'),e.get('geometry_activation_stamp_ns'))
        if prior and prior[0]==owner:
            old=prior[1];ip=base.phase_of(old['read_completed_monotonic'],finish,phases)
            groups[ip]['visible_successful_read_interval_sec'].append(finish-old['read_completed_monotonic'])
            groups[ip]['visible_source_header_interval_sec'].append((e['source_stamp_ns']-old['source_stamp_ns'])*1e-9)
        prior=(owner,e)
    return dict(sealed_hashes=hashes,resource_capture_complete=outcome.get('observer_integrity_passed'),
        resource_capture_start=outcome.get('capture_start_monotonic'),resource_capture_end=outcome.get('capture_end_monotonic'),
        phases={p:dict(counts=dict(counts[p]),statistics={k:base.stats(v) for k,v in groups[p].items()}) for p in base.PHASES},
        limits=['Only successful camera read history is supplied; no rejected SDK frames or guaranteed image-publication count inferred.',
                'Stage aggregates require the enclosing SDK wait-entry to read-completion interval inside a phase. Crossing intervals are separate.',
                'Visible inter-read intervals do not imply every sensor frame was observed.'])


def status_and_events(case,summary,phases):
    sm=base.manifest(case/'shadow');tm=base.manifest(case/'shadow/transport')
    wrappers=base.jsonl(case/'shadow/detector_status.jsonl');events=base.jsonl(case/'shadow/transport/events.jsonl')
    ready,_=bound_json(case/'shadow/ready.json',sm['ready.json'])
    identities=Counter();source_labels=[];first_losses={};parsed=[]
    for r in wrappers:
        if hashlib.sha256(r['original_status_data'].encode()).hexdigest()!=r['original_status_utf8_sha256']:raise ValueError('Status payload hash differs')
        s=json.loads(r['original_status_data']);kind=base.status_identity(s,r['kind'],ready)
        caller=r['receipt'].get('publisher_callerid');identities[(kind,r['kind'],caller,str(s.get('generation')),str(s.get('selection_stamp_ns')))]+=1
        expected=ready['node_name'] if kind=='shadow' else '/unknown_tabletop_perception' if kind=='production' else None
        if expected and caller!=expected:raise ValueError('Publisher caller differs from selected identity')
        if kind!='unclassified' and kind!=r['kind']:raise ValueError('Declared status label differs')
        loss=s.get('first_loss')
        if kind=='shadow' and isinstance(loss,dict):
            key=(loss.get('generation'),loss.get('selection_stamp_ns'),loss.get('observed_stamp_ns'))
            first_losses.setdefault(key,dict(original_wrapper=r,loss=loss))
        parsed.append((kind,r,s))
    batches=defaultdict(Counter)
    for e in events:
        if e['kind']!='deserialize_messages':continue
        p=base.phase_of(*base.event_time(e),phases);w=e.get('wire_before',{})
        batches[p]['calls']+=1
        if not w.get('complete_count_exact'):
            batches[p]['inexact_peek']+=1;continue
        c=w.get('complete_count',0);q=e.get('queue_size');batches[p]['complete_messages']+=c
        batches[p]['complete_batches_larger_than_queue']+=bool(q and c>q)
        if q and c>q:
            batches[p]['bodies_original_would_prune']+=c-q
            batches[p]['bytes_original_would_prune']+=sum(x['payload_bytes'] for x in w['complete'][:-q])
        batches[p]['partial_fragments']+=bool(w.get('partial'))
    b=next(p for p in phases if p['name']=='recording')
    nodes=[e for e in events if e['kind']=='node_process' and base.phase_of(*base.event_time(e),phases)=='recording']
    longest=max(nodes,key=lambda e:e['elapsed_ns']) if nodes else None
    matched=[]
    if longest:
        for kind,r,s in parsed:
            t=s.get('frame_timing') or {}
            if kind=='shadow' and s.get('source_stamp_ns')==longest.get('last_consumed_source_after') and base.finite(t.get('processing_start_monotonic_sec')):
                if abs(t['processing_start_monotonic_sec']-longest['start']['monotonic_ns']*1e-9)<.01:
                    matched.append(r)
    proof=dict(schema=1,source_status_sha256=sm['detector_status.jsonl'],source_events_sha256=tm['events.jsonl'],
        first_losses=list(first_losses.values()),largest_B_timer_original=longest,matched_original_statuses=matched,
        interval_binding='same original source plus processing-start within 10ms of tap Timer call; raw values retained',
        first_loss_interpretation=[])
    for x in proof['first_losses']:
        l=x['loss'];o=l.get('observed_stamp_ns');g=l.get('last_good_source_stamp_ns');s=l.get('source_stamp_ns')
        if all(isinstance(v,int) for v in (o,g,s)):
            proof['first_loss_interpretation'].append(dict(reason=l.get('reason'),current_source_age_sec=(o-s)*1e-9,last_good_age_sec=(o-g)*1e-9,
                first_loss_observed_ns=o,first_loss_source_ns=s))
    if longest and proof['first_losses']:
        loss_ns=proof['first_losses'][0]['loss'].get('observed_stamp_ns')
        if isinstance(loss_ns,int):proof['largest_timer_start_after_first_loss_sec']=(longest['start']['wall_ns']-loss_ns)*1e-9
    proofhash=save(case/'analysis/first_loss_and_slow_processing.json',proof)
    caller_report=[dict(evidence_kind=k[0],declared_kind=k[1],callerid=k[2],generation=k[3],selection_stamp_ns=k[4],count=v) for k,v in identities.items()]
    return dict(identity_and_callers=caller_report,batch_statistics_by_phase={k:dict(v) for k,v in batches.items()},
        first_loss_count=len(first_losses),first_loss_ages=proof['first_loss_interpretation'],
        largest_B_timer=base.brief(longest,phases) if longest else None,
        largest_B_timer_status_timing=[json.loads(r['original_status_data']).get('frame_timing') for r in matched],
        first_loss_proof_sha256=proofhash,
        full_window_events_over_2sec=sum(e['elapsed_ns']>=2e9 for e in events),
        B_events_over_2sec=sum(e['elapsed_ns']>=2e9 for e in events if base.phase_of(*base.event_time(e),phases)=='recording'),
        events=events)


def analyze(root):
    cases=[];contamination=[]
    for name in ('baseline_original','candidate_skip_pruned','return_original'):
        case=root/name
        report,rh=bound_json(case/'analysis/receive_analysis.json')
        if not report.get('input_integrity_passed'):raise ValueError('Receive report not intact')
        summary,sh=bound_json(case/'shadow/summary.json',report['shadow_manifest']['summary.json'])
        phases=report['phases'];extra=status_and_events(case,summary,phases);events=extra.pop('events')
        if name=='baseline_original':
            # Supplied exact UTC bounds, mapped using nearest contemporaneous
            # trace wall/monotonic pair solely to flag unrelated test overlap.
            date=datetime.datetime(2026,10,8,8,6,8,tzinfo=datetime.timezone.utc)
            wall_start=int(date.timestamp())*10**9+606_000_000;wall_end=wall_start+1_691_000_000
            e=min(events,key=lambda v:abs(v['start']['wall_ns']-wall_start))
            a=(e['start']['monotonic_ns']+wall_start-e['start']['wall_ns'])*1e-9
            z=(e['start']['monotonic_ns']+wall_end-e['start']['wall_ns'])*1e-9
            contamination.append(dict(kind='independent_104_test_pytest',start_utc='2026-10-08T08:06:08.606Z',end_utc='2026-10-08T08:06:10.297Z',
                approximate_start_monotonic=a,approximate_end_monotonic=z,phase=base.phase_of(a,z,phases),
                B_overlap=base.phase_of(a,z,phases) in ('recording','cross_phase'),
                mapping='Nearest captured trace wall/monotonic clock pair; not used to modify any source timestamp.'))
        outcome=report['raw_load']['outcome'];b=next(p for p in phases if p['name']=='recording')
        raw,_=bound_json(case/'raw_load/outcome.json',report['input_sha256']['raw_load/outcome.json'])
        unique={r['source_ns'] for r in raw['pairs']}
        if len(unique)!=outcome['pair_count'] or len(raw['pairs'])!=outcome['pair_count']:raise ValueError('Raw pair source count differs')
        cases.append(dict(name=name,receive_report_sha256=rh,shadow_summary_sha256=sh,all_90s_complete=summary.get('complete'),
            phases=phases,B_duration_sec=b['end_monotonic']-b['start_monotonic'],B_raw_pairs=outcome['pair_count'],
            B_raw_unique_sources=len(unique),raw_closed_valid=bool(outcome.get('bag_closed') and outcome.get('capture_valid')),
            B_receive_stage_statistics=report['trace']['per_phase']['recording'],
            B_callback_source_age=[v for v in report['trace']['callback_source_age_sec'] if v['phase']=='recording'],
            B_raw_arrival_age=[v for v in report['statuses']['raw_arrival_age_sec'] if v['phase']=='recording'],
            B_status_counts=[v for v in report['statuses']['counts'] if v['receipt_phase']=='recording'],
            sdk=sdk_statistics(case,phases),
            variant=summary.get('deserializer_variant'),candidate_statistics=summary.get('candidate_statistics'),
            candidate_restore=summary.get('candidate_restore'),candidate_validation=summary.get('candidate_validation'),
            production_selection_unchanged=summary.get('production_selection_unchanged'),**extra))
    return dict(schema=1,kind='three_round_live_buffer_comparison',source_sha256=base.digest(__file__),
        root=str(root),primary_comparison='Fully contained actual B raw-capture interval in each round; crossing intervals separately retained in per-round reports.',
        cases=cases,unrelated_workload_annotations=contamination,
        prior_benchmark=dict(start_monotonic=192632.731440408,end_monotonic=192636.543897903,
            baseline_capture_start_monotonic=192650.207372691,overlap=False,gap_sec=13.66347478801664),
        conclusions=[
            'All three diagnostic shadow traces complete their90-second windows; each45-second raw recording is valid. No2-second completed trace event is present in these rounds.',
            'The candidate executed and restored completely, but skipped_payload_bytes=0. No backlog larger than queue2 arose in candidate or return-original; avoided-copy benefit was not exercised live.',
            'The original baseline shadow first lost qualification because its previous good support reference aged past0.8s while its current frame remained about12ms fresh. This is distinct from multi-second image transport delay.',
            'A later baseline Timer took about1.032s, dominated by segmentation (~1.008s wall/~0.651s thread CPU), and that frame aged beyond0.8s during processing; it occurred after the original support-gap loss.',
            'Candidate and return-original tracked through most B statuses with transient support pending. Return-original also remaining stable precludes attributing non-recurrence solely to the candidate.',
            'Production selection generation25 remains retired throughout; shadow selection results confer no motion or grasp authority.'],
        limitations=[
            'Three sequential short rounds are not randomized or globally load-isolated; underlying system/camera/support conditions can vary.',
            'The unrelated104-test run overlaps only baseline after phase, not its B capture. The synthetic benchmark ended before all live baseline recording.',
            'SDK history contains successful reads only. Recorded rejected status counts are messages, not a complete independent count of physically invalid sensor frames.',
            'CPU/wall metrics localize time but do not isolate physical camera, VM scheduling, allocator or disk as a single cause.',
            'Raw pair source count/metadata are checked; no pixels or bag contents are decoded by this comparison.',
            'Synthetic deterministic savings and live delay efficacy are distinct. The underlying intermittent problem is not declared resolved.'],
        problem_resolved=False,production_grasp_recovered=False,new_motion_authority=False)


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--input',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args();r=analyze(a.input.resolve());h=save(a.output,r)
    print(json.dumps(dict(output=str(a.output),sha256=h,cases=len(r['cases']))))


if __name__=='__main__':main()
