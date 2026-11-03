"""Pure sealed fixtures: no live sampling, ROS, SDK, or /proc access."""
import copy
import importlib.util
import json
from pathlib import Path

import pytest

PATH = Path(__file__).with_name('analyze_sdk_memory_pressure_20261007.py')
spec = importlib.util.spec_from_file_location('offline_sdk_memory_analysis', PATH)
a = importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)


def stat(start=50, ticks=0):
    return dict(start_ticks=start,state='S',minflt=ticks,majflt=0,utime_ticks=ticks,stime_ticks=0)


def resource(t,index):
    return dict(started_monotonic=t,finished_monotonic=t+.01,quality_gaps=[],
        vmstat={'allocstall_movable':index,'balloon_inflate':0,'pswpout':index*10,'nr_free_pages':100-index},
        pressure={name:f'some avg10=0 avg60=0 avg300=0 total={index*1000}\nfull avg10=0 avg60=0 avg300=0 total={index*500}' for name in ('cpu','memory','io')},
        meminfo={'MemAvailable':'7000000 kB'},diskstats=f'8 0 sda {index} 0 {index*2} 0 {index} 0 {index*3} 0 0 {index} {index}\n',
        processes={'123':{'stat':stat(ticks=index),'tasks':{'124':{'stat':stat(51,index),
            'schedstat_raw':f'{index*100} 0 {index}', 'wchan_raw':'futex_wait_queue_me'}}}})


def entry(source,completed,duration=.4):
    return dict(geometry_generation=1,geometry_activation_stamp_ns=999,source_stamp_ns=source,
        frame_id='camera',read_completed_monotonic=completed,
        publication_outcome_unknown=True,exposure_clock_calibrated=False,
        evidence={'result':'complete','host_wait':{'enter':{'monotonic_sec':completed-duration-.02},
            'exit':{'monotonic_sec':completed-.02},'elapsed_sec':duration,'thread_cpu_elapsed_sec':.00005},
            'stages_sec':{'wait_for_frames':duration,'align':.002},
            'regional_stationary_evidence':{'global_history_reset_reason':'sdk_source_discontinuity'},
            'raw_frames':{s:{'frame_number':source,'device_timestamp_ms':completed*1000,
                'timestamp_domain':'global_time','metadata_raw':{'frame_timestamp':int(completed*1e6),
                'actual_fps':704},'unavailable':{'frame_counter':'unsupported'}} for s in ('color','depth')}})


def fixture():
    ready=dict(schema=1,source_sha256='f'*64,boot_id='boot-a',resource_period_sec=.5,sched_schedstats='0',
        bound_processes={'123':{'start_ticks':50,'comm':'python3','explicit_watch':True,
                              'tasks':{'124':{'start_ticks':51,'comm':'python3'}}}})
    values=[entry(10,-.1,.02),entry(11,.6,.02),entry(12,1.22),entry(13,2,.02),entry(14,3.1,.02)]
    sdk=[dict(entry_sha256=a.sha(a.canonical(e)),original_entry=e,first_observed_monotonic=e['read_completed_monotonic']+.3) for e in values]
    rpc=[]
    for group in (sdk[:3],sdk[2:]):
        rpc.append(dict(schema_valid=True,payload_without_entries={'geometry_generation':1,'geometry_activation_stamp_ns':999,'copy_errors':0},
            entry_references=[{'key':list(a.entry_key(r['original_entry'])),'entry_sha256':r['entry_sha256']} for r in group],elapsed_sec=.003))
    outcome=dict(schema=1,source_sha256='f'*64,capture_closed=True,capture_start_monotonic=0,
        capture_end_monotonic=3,boot_id_at_end='boot-a',observer_integrity_passed=True,
        record_state={},errors=[],rpc_errors={},proc_quality_gaps={},monitor_self_usage={})
    return ready,outcome,dict(resources=[resource(i*.5,i) for i in range(6)],sdk=sdk,rpc=rpc)


def seal(folder,parts):
    ready,outcome,rows=copy.deepcopy(parts)
    raw={'ready.json':a.canonical(ready)}
    for kind,name in a.FILES.items():raw[name]=b''.join(a.canonical(r) for r in rows[kind])
    outcome['record_state']['counts']={k:len(v) for k,v in rows.items()}
    outcome['files']={k:{'sha256':a.sha(v),'bytes':len(v)} for k,v in raw.items()}
    raw['outcome.json']=a.canonical(outcome)
    for name,value in raw.items():(folder/name).write_bytes(value)
    (folder/'SHA256SUMS').write_text(''.join(a.sha(value)+'  '+name+'\n' for name,value in sorted(raw.items())))


def test_exact_sealed_interval_and_resource_association(tmp_path):
    seal(tmp_path,fixture());r=a.analyze(tmp_path,123)
    assert r['window']['unique_entries_total']==5
    assert r['window']['completed_inside']==3 and r['window']['excluded_outside']==2
    assert r['summary']['long_wait_count']==1
    event=r['long_wait_events'][0]
    assert event['source_stamp_ns']==12 and event['resources']['available']
    assert event['resources']['resource_row_indices']==[1,3]
    assert event['resources']['vmstat']['deltas']['allocstall_movable']==2
    assert 'nr_free_pages' not in event['resources']['vmstat']['deltas']
    assert event['resources']['vmstat_gauges_after']['nr_free_pages']==97
    assert event['stream_changes_from_previous_visible_success']['color']['current_raw']['metadata_raw']['actual_fps']==704
    assert r['rpc_history_coverage'][1]['previous_same_owner_overlap_count']==1
    assert r['problem_resolved'] is False


def test_tamper_is_rejected_before_analysis(tmp_path):
    seal(tmp_path,fixture());(tmp_path/'resources.jsonl').write_text('{}\n')
    with pytest.raises(ValueError,match='checksum mismatch'):a.analyze(tmp_path)


@pytest.mark.parametrize('change', ['unsealed','observer_sha','temporary','counts','unsafe_manifest'])
def test_seal_contract(tmp_path,change):
    p=fixture()
    if change=='unsealed':p[1]['capture_closed']=False
    if change=='observer_sha':p[1]['source_sha256']='0'*64
    seal(tmp_path,p)
    if change=='temporary':(tmp_path/'outcome.json.tmp').write_text('unfinished')
    if change=='counts':
        d=json.loads((tmp_path/'outcome.json').read_text());d['record_state']['counts']['sdk']=99
        raw=a.canonical(d);(tmp_path/'outcome.json').write_bytes(raw)
        lines=(tmp_path/'SHA256SUMS').read_text().splitlines()
        (tmp_path/'SHA256SUMS').write_text('\n'.join(a.sha(raw)+'  outcome.json' if x.endswith('  outcome.json') else x for x in lines)+'\n')
    if change=='unsafe_manifest':(tmp_path/'SHA256SUMS').write_text('0'*64+'  ../ready.json\n')
    with pytest.raises(ValueError):a.analyze(tmp_path)


def test_boot_change_blocks_resource_deltas(tmp_path):
    p=fixture();p[1]['boot_id_at_end']='boot-b';seal(tmp_path,p);r=a.analyze(tmp_path)
    assert not r['integrity']['same_boot']
    assert r['long_wait_events'][0]['resources']==dict(available=False,event_start_monotonic=.7999999999999999,
        event_end_monotonic=1.2,causality_established=False,reason='boot_identity_mismatch')


def test_missing_rows_are_not_bridged(tmp_path):
    p=fixture();p[2]['resources']=[resource(0,0),resource(1.5,3),resource(2.5,5)]
    seal(tmp_path,p);association=a.analyze(tmp_path)['long_wait_events'][0]['resources']
    assert not association['available'] and association['reason']=='resource_sampling_gap_no_counter_bridge'
    assert 'vmstat' not in association


@pytest.mark.parametrize('where', ['process','task'])
def test_identity_change_prevents_false_counter_attribution(tmp_path,where):
    p=fixture();row=p[2]['resources'][2]['processes']['123']
    (row['stat'] if where=='process' else row['tasks']['124']['stat'])['start_ticks']=777
    seal(tmp_path,p);process=a.analyze(tmp_path)['long_wait_events'][0]['resources']['processes']['123']
    checked=process if where=='process' else process['tasks']['124']
    assert not checked['identity_valid'] and 'schedstat_delta' not in checked


def test_disabled_schedstats_zero_is_not_evidence_of_no_wait(tmp_path):
    seal(tmp_path,fixture());task=a.analyze(tmp_path)['long_wait_events'][0]['resources']['processes']['123']['tasks']['124']
    assert task['schedstat_delta']['deltas']['runqueue_wait_ns']==0
    assert not task['runqueue_wait_counter_interpretable']
    assert task['state_observations'][1]['start_monotonic']==1


def test_counter_decrease_is_unavailable_not_negative_activity(tmp_path):
    p=fixture();p[2]['resources'][3]['vmstat']['allocstall_movable']=0;seal(tmp_path,p)
    delta=a.analyze(tmp_path)['long_wait_events'][0]['resources']['vmstat']
    assert 'allocstall_movable' in delta['unavailable_or_decreased']
    assert 'allocstall_movable' not in delta['deltas']


@pytest.mark.parametrize('kind',['entry_digest','duplicate','rpc_owner','rpc_hash'])
def test_source_provenance_cannot_be_rebound(tmp_path,kind):
    p=fixture()
    if kind=='entry_digest':p[2]['sdk'][2]['original_entry']['evidence']['host_wait']['elapsed_sec']=5
    if kind=='duplicate':p[2]['sdk'].append(copy.deepcopy(p[2]['sdk'][0]))
    if kind=='rpc_owner':p[2]['rpc'][0]['payload_without_entries']['geometry_generation']=2
    if kind=='rpc_hash':p[2]['rpc'][0]['entry_references'][0]['entry_sha256']='0'*64
    seal(tmp_path,p)
    with pytest.raises(ValueError):a.analyze(tmp_path)


def test_rpc_error_and_no_overlap_are_explicit(tmp_path):
    p=fixture();p[2]['rpc'][1]['entry_references']=p[2]['rpc'][1]['entry_references'][1:]
    p[2]['rpc'].append(dict(schema_valid=False,error='timeout'))
    seal(tmp_path,p);r=a.analyze(tmp_path)
    assert r['rpc_history_coverage'][1]['previous_same_owner_overlap_count']==0
    assert not r['rpc_history_coverage'][1]['no_successful_history_gap_proven']
    assert not r['rpc_history_coverage'][2]['valid']


def test_unknown_camera_pid_is_not_silently_labelled(tmp_path):
    seal(tmp_path,fixture())
    with pytest.raises(ValueError,match='camera PID'):a.analyze(tmp_path,999)


def test_wait_crossing_window_is_retained_but_not_correlated(tmp_path):
    p=fixture();e=p[2]['sdk'][1]['original_entry'];e['evidence']['host_wait']=dict(enter={'monotonic_sec':-.2},exit={'monotonic_sec':.58},elapsed_sec=.78)
    p[2]['sdk'][1]['entry_sha256']=a.sha(a.canonical(e))
    p[2]['rpc'][0]['entry_references'][1]['entry_sha256']=p[2]['sdk'][1]['entry_sha256']
    seal(tmp_path,p);report=a.analyze(tmp_path);event=report['long_wait_events'][0]
    assert not event['wholly_within_capture_window'] and not event['resources']['available']
    assert report['summary']['wait_seconds']['count']==3
    assert report['summary']['wait_seconds_wholly_inside_capture']['count']==2
    assert report['summary']['wait_seconds']['max']==.78
    assert report['summary']['wait_seconds_wholly_inside_capture']['max']==.4
    assert report['summary']['long_wait_count']==2
    assert report['summary']['long_wait_count_wholly_inside_capture']==1


@pytest.mark.parametrize('position,value', [(0,1),(1,1),(0,None),(1,None),(0,False)])
def test_overlap_cannot_prove_completeness_after_unknown_or_failed_copy(tmp_path,position,value):
    p=fixture();p[2]['rpc'][position]['payload_without_entries']['copy_errors']=value
    seal(tmp_path,p);coverage=a.analyze(tmp_path)['rpc_history_coverage'][1]
    assert coverage['previous_same_owner_overlap_count']==1
    assert not coverage['no_successful_history_gap_proven']
