#!/usr/bin/env python3
"""Read sealed artifacts only; write one new external review, never run ROS."""
from pathlib import Path
import ast
from collections import Counter
import hashlib
import json

W=Path(__file__).resolve().parents[1]
CASE=Path(json.loads((W/'.runtime/current_unknown_buffer_round_20261008.json').read_text())['directory'])

def read(p):return json.loads(p.read_text())
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
def text_sha(raw):return hashlib.sha256(raw).hexdigest()

def main():
    selection=read(CASE/'initial_runtime_state.json')['parameters']['/grasp_mode/selection']
    verified=[];sources={};geometries=[];boots=set();segments={}
    for name,mode in [('baseline_original','original'),('candidate_skip_pruned','skip_pruned_copies'),('return_original','original')]:
        p=CASE/name;e=read(p/'experiment.json');s=read(p/'shadow/summary.json')
        t=read(p/'shadow/transport/summary.json');raw=read(p/'raw_load/outcome.json')
        resource=read(p/'resources/outcome.json');ready=read(p/'resources/ready.json')
        for rel in ['experiment.sha256','shadow/SHA256SUMS','shadow/transport/SHA256SUMS','resources/SHA256SUMS']:
            side=p/rel
            for line in side.read_text().splitlines():
                expected,n=line.split(None,1);f=side.parent/n.strip().lstrip('*')
                assert sha(f)==expected,(name,f)
                verified.append(str(f.relative_to(CASE)))
        assert e['orchestration_completed'] and e['failure'] is None and not e['diagnostic_children_remaining']
        assert all(c['returncode']==0 for c in e['children'].values())
        assert s['complete'] and s['stop_reason']=='duration_limit' and s['trace_full_window_passed'] and s['hooks_restored'] and s['sources_unchanged']
        assert not s['errors'] and not s['cleanup_errors'] and s['private_parameter_cleanup']=='deleted_own_exact_private_selection'
        assert s['production_selection_before']==s['production_selection_after']==selection
        assert e['production_unknown_identity_preserved']
        assert e['production_unknown']['pid']==e['production_unknown_after']['pid']==118128
        assert e['production_unknown']['start_ticks']==e['production_unknown_after']['start_ticks']==2489668
        assert e['deserializer_variant']==s['deserializer_variant']==mode
        assert s['maximum_directory_bytes']==64*1024**2 and s['directory_budget_passed']
        assert t['maximum_bytes']==50*1024**2 and t['sealed'] and t['observer_integrity_passed'] and t['original_method_bindings_restored'] and not t['restore_conflicts']
        assert t['rows_admitted']==t['rows_written'] and not t['queued_unwritten'] and not t['writer_alive']
        assert t['stopped_monotonic']>=t['deadline_monotonic'] and t['deadline_monotonic']-t['started_monotonic']==90
        assert s['capture_start_monotonic']==t['started_monotonic']
        assert s['pid']==e['children']['shadow']['identity']['pid'] and s['start_ticks']==e['children']['shadow']['identity']['start_ticks']
        trace_rows=0
        for line in (p/'shadow/transport/events.jsonl').open():
            event=json.loads(line);assert event['pid']==s['pid'];trace_rows+=1
        assert trace_rows==t['rows_written']
        assert resource['observer_integrity_passed'] and resource['capture_closed'] and resource['stop_reason']=='duration_elapsed' and not resource['errors']
        explicit={pid:v['start_ticks'] for pid,v in ready['bound_processes'].items() if v['explicit_watch']}
        assert explicit=={str(s['pid']):s['start_ticks']}
        boots.add(ready['boot_id']);assert resource['boot_id_at_end']==ready['boot_id']
        for fn,h in e['source_sha256'].items():assert sha(p/'sources'/Path(fn).name)==h,(name,fn)
        for fn,h in s['source_sha256'].items():assert sha(Path(fn))==h,(name,fn);sources[fn]=h
        cp=json.loads(json.dumps(s['camera_parameters_at_start']));cp['runtime_profile'].pop('stationary_temporal',None);geometries.append(cp)
        counts=Counter();callers=Counter();generations=Counter();bounds={}
        for line in (p/'shadow/detector_status.jsonl').open():
            row=json.loads(line);text=row['original_status_data'];assert text_sha(text.encode())==row['original_status_utf8_sha256']
            d=json.loads(text);kind=row['kind'];rc=row['receipt'];caller=rc['publisher_callerid']
            counts[kind]+=1;callers[(kind,caller)]+=1;generations[(kind,d.get('generation'))]+=1
            b=bounds.setdefault(kind,[rc['monotonic'],rc['monotonic']]);b[0]=min(b[0],rc['monotonic']);b[1]=max(b[1],rc['monotonic'])
            if kind=='production':
                assert caller=='/unknown_tabletop_perception' and row['topic']=='/perception/unknown/detector_status'
                assert d['generation']==25 and str(d['selection_stamp_ns'])==selection['stamp_ns']
            else:
                assert kind=='shadow' and caller==s['node_name'] and row['topic']==s['namespace']+'/perception/unknown/detector_status'
                assert (d['generation']==-1 and d['selection_stamp_ns']==0) or (d['generation']==1 and str(d['selection_stamp_ns'])==s['private_selection']['stamp_ns'])
        assert sum(counts.values())==s['statuses']['rows'] and s['statuses']['sealed'] and not s['statuses']['limit_reached']
        assert sha(p/'shadow/detector_status.jsonl')==s['statuses']['sha256']
        owners=set();rpc_count=0
        for line in (p/'resources/camera_rpc.jsonl').open():
            row=json.loads(line);rpc_count+=1;assert row['schema_valid'];meta=row['payload_without_entries']
            owners.add((meta['geometry_generation'],str(meta['geometry_activation_stamp_ns']),meta['frame_id']))
            assert not row.get('owner_changed_since_previous_poll')
        assert owners=={(2,'1791430691426100492','camera_link')}
        assert raw['capture_valid'] and raw['bag_closed'] and raw['stop_reason']=='duration_elapsed' and not raw['errors']
        assert raw['pair_count']==len(raw['pairs'])==len({r['source_ns'] for r in raw['pairs']})
        assert all(set(r['images'])=={'color','depth'} and all(h['source_ns']==r['source_ns'] for h in r['images'].values()) for r in raw['pairs'])
        assert all(raw['capture_start_monotonic']<=h['receipt_monotonic']<raw['capture_deadline_monotonic'] for r in raw['pairs'] for h in r['images'].values())
        assert raw['capture_end_monotonic']-raw['capture_start_monotonic']>=45
        assert (p/'raw_load/raw_rgbd.bag').stat().st_size==raw['bag_bytes'] and not (p/'raw_load/raw_rgbd.bag.active').exists()
        if mode=='original':
            assert s['candidate_source_sha256'] is None and 'candidate_binding' not in s and not s['candidate_validation']['candidate_used'] and not s['candidate_validation']['copy_pruning_observed']
        else:
            st=s['candidate_statistics'];assert st['calls_started']==st['calls_completed']==6608 and st['calls_failed']==st['inflight']==0 and st['active'] is False
            assert st['optimized_calls']==3910 and st['fallback_calls']==2698 and st['complete_messages_scanned']==st['retained_messages']==3568 and st['skipped_payload_bytes']==0
            assert st['scanned_payload_bytes']==st['retained_payload_bytes']==st['materialized_payload_bytes']==2848494960
            assert s['candidate_restore']=={'candidate_deactivated':True,'original_alias_restored':True,'restore_conflicts':[]}
            assert s['candidate_validation']['passed'] and s['candidate_validation']['candidate_used'] and not s['candidate_validation']['copy_pruning_observed']
        segments[name]=dict(deserializer=mode,experiment_sha256=sha(p/'experiment.json'),summary_sha256=sha(p/'shadow/summary.json'),
            trace_sha256=t['events_sha256'],raw_outcome_sha256_at_review=sha(p/'raw_load/outcome.json'),
            trace_observed_seconds=t['stopped_monotonic']-t['started_monotonic'],status_count=dict(counts),
            status_generations={str(k):v for k,v in generations.items()},callerids={str(k):v for k,v in callers.items()},
            status_receipt_bounds_monotonic=bounds,production_identity=dict(pid=118128,start_ticks=2489668),
            shadow_identity=dict(pid=s['pid'],start_ticks=s['start_ticks']),resource_observed_seconds=resource['duration_observed_sec'],
            private_parameter_cleanup=s['private_parameter_cleanup'],rpc_geometry_owner=list(owners),rpc_rows=rpc_count,
            raw_pair_count=raw['pair_count'],raw_capture_valid=True,raw_bag_size=raw['bag_bytes'],
            raw_relative_trace_window_sec=[raw['capture_start_monotonic']-t['started_monotonic'],raw['capture_end_monotonic']-t['started_monotonic']],
            trace_open_calls_at_stop=len(t['outstanding_calls']),candidate_statistics=s.get('candidate_statistics'),candidate_validation=s['candidate_validation'])
    assert all(g==geometries[0] for g in geometries) and len(boots)==1
    tree=ast.parse((W/'.runtime/run_unknown_receive_shadow_20261008.py').read_text())
    arg=next(n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='add_argument' and n.args and isinstance(n.args[0],ast.Constant) and n.args[0].value=='--deserializer')
    assert next(k.value.value for k in arg.keywords if k.arg=='default')=='original'
    report=dict(schema=1,review_kind='Independent sealed artifact and isolation review; no live reads/actions',
        reviewer_source_sha256=sha(Path(__file__)),segments=segments,verified_file_count=len(verified),verified_files=verified,
        source_sha256_current_and_recorded=sources,production_selection_unchanged=selection,
        static_camera_config_equal_excluding_only_runtime_stationary_temporal=True,
        static_camera_config_sha256=text_sha(json.dumps(geometries[0],sort_keys=True,separators=(',',':'),allow_nan=False).encode()),
        observed_geometry_owner_stable=[2,'1791430691426100492','camera_link'],boot_id=next(iter(boots)),
        default_original_retained=True,candidate_copy_pruning_triggered=False,field_speedup_demonstrated=False,
        limitations=[
            'Candidate processed images but skipped zero bytes; field copy-saving and resolution of the original intermittent delay are not established.',
            'Helper counts cover install-to-deactivate; use trace timestamps for windowed performance comparisons.',
            'The mirror adds load and lacks production output consumers; it is not identical to the production process.',
            'Production unknown OS identity is directly bound; camera/driver OS PIDs are absent from these segment manifests. Stable geometry activation is not an independent PID measurement.',
            'Static camera configuration was captured at segment startup; root-diagnostic geometry owner was checked at captured polls. Unobserved intervals are not independently proven.',
            'Raw bags were not reread or rehashed: the independent recorder supplies no preexisting bag SHA. Here validity checks sealed outcome, unique exact pairs/receipt windows, closed filename and byte size; recorded metadata/trace checksums are verified separately.'])
    out=CASE/'independent_segment_review.json'
    with out.open('x') as f:f.write(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(path=str(out),sha256=sha(out),verified_files=len(verified),
        segments={n:{k:v for k,v in r.items() if k in ('trace_observed_seconds','status_count','raw_pair_count','resource_observed_seconds','candidate_validation')} for n,r in segments.items()}),ensure_ascii=False,indent=2))

if __name__=='__main__':main()
