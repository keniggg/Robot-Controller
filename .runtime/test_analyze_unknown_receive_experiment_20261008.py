import hashlib
import importlib.util
import json
from pathlib import Path
import pytest

spec=importlib.util.spec_from_file_location('receive_analysis',Path(__file__).with_name('analyze_unknown_receive_experiment_20261008.py'))
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)


def test_phase_boundaries_and_crossing():
    phases=a.make_phases(10,100,dict(capture_start_monotonic=30,capture_deadline_monotonic=75),
        dict(capture_start_monotonic=30,capture_deadline_monotonic=75,capture_end_monotonic=75.1,completed_monotonic=77))
    assert a.phase_of(29,30,phases)=='baseline'
    assert a.phase_of(30,30,phases)=='recording'
    assert a.phase_of(29,31,phases)=='cross_phase'
    assert a.phase_of(75.1,76,phases)=='closing'
    assert a.phase_of(77,99,phases)=='after'
    assert a.phase_of(100,101,phases)=='outside'
    assert a.phase_of(10,11,[])=='unassigned'


def test_no_data_is_not_healthy():
    assert a.stats([])==dict(available=False,count=0,p50=None,p95=None,maximum=None)
    assert not a.analyze_events([],[])['per_phase']['baseline']['socket_recv']['wall_ms']['available']


def test_rejects_mismatched_real_boundaries():
    with pytest.raises(ValueError,match='start mismatch'):
        a.make_phases(0,100,dict(capture_start_monotonic=10),dict(capture_start_monotonic=11,capture_end_monotonic=20,completed_monotonic=21))


def test_checksum_tamper_and_symlink(tmp_path):
    f=tmp_path/'events.jsonl';f.write_text('{}\n')
    (tmp_path/'SHA256SUMS').write_text(hashlib.sha256(f.read_bytes()).hexdigest()+'  events.jsonl\n')
    assert a.manifest(tmp_path)
    f.write_text('{"changed":true}\n')
    with pytest.raises(ValueError,match='Checksum mismatch'):a.manifest(tmp_path)
    f.unlink();f.symlink_to('/dev/null')
    with pytest.raises(ValueError,match='Unsafe'):a.manifest(tmp_path)


def event(kind='receive_callback',start=20,end=20.1):
    return dict(kind=kind,call_id=1,tid=4,connection_id=3,topic='color',
        start=dict(monotonic_ns=round(start*1e9),wall_ns=1002_000_000_000),
        end=dict(monotonic_ns=round(end*1e9),wall_ns=1002_100_000_000),
        elapsed_ns=round((end-start)*1e9),thread_cpu_ns=2_000_000,
        headers=[dict(source_stamp_ns=1000_000_000_000)])


def test_source_age_uses_actual_event_not_wall_duration():
    e=event();r=a.analyze_events([e],[])
    assert r['callback_source_age_sec'][0]['statistics']['maximum']==2
    assert r['per_phase']['unassigned']['receive_callback']['wall_ms']['maximum']==100
    assert r['per_phase']['unassigned']['receive_callback']['thread_cpu_ms']['maximum']==2


def status_row(kind='production',generation=1,stamp=123):
    payload=json.dumps(dict(generation=generation,selection_stamp_ns=stamp,state='unavailable',detail='waiting',
        frame_timing={'raw_arrivals':{'color':dict(source_stamp_ns=1000_000_000_000,
        arrival_monotonic_sec=19,source_age_at_arrival_sec=1.5)}}))
    return dict(kind=kind,receipt={'monotonic':21},original_status_data=payload,
        original_status_utf8_sha256=hashlib.sha256(payload.encode()).hexdigest())


def test_mislabeled_production_is_never_counted_as_production():
    ready=dict(private_selection={'generation':1,'stamp_ns':'123'},production_selection_before={'generation':25,'stamp_ns':'124'})
    row=status_row();r=a.analyze_statuses([row,row],[],[event()],ready)
    assert r['identity_label_mismatch_count']==2
    assert r['counts'][0]['kind']=='shadow'
    assert r['unique_raw_arrivals']==1
    assert r['raw_arrival_age_sec'][0]['statistics']['maximum']==1.5
    assert r['raw_arrival_largest'][0]['arrival_monotonic']==19
    assert len(r['raw_arrival_largest'][0]['same_source_shadow_callbacks'])==1


def test_generation_alone_does_not_bind_selection():
    ready=dict(private_selection={'generation':1,'stamp_ns':'123'})
    assert a.status_identity({'generation':1,'selection_stamp_ns':124},'shadow',ready)=='unclassified'
    assert a.status_identity({'generation':1},'shadow',ready)=='unclassified'


def test_status_payload_hash_required():
    row=status_row();row['original_status_data']='{}'
    with pytest.raises(ValueError,match='payload checksum'):a.analyze_statuses([row],[],[],{})


def test_nested_intervals_not_summed():
    r=a.analyze_events([event('receive_once',20,23),event('socket_recv',21,22)],[])
    assert r['long_events'][0]['wall_ms']==3000
    assert r['long_events'][0]['overlapping_related_stages']['socket_recv']['longest'][0]['wall_ms']==1000
    assert r['long_events'][0]['association_only_not_causation']


def test_long_timer_lists_other_thread_transport_overlap():
    timer=event('node_process',20,23);timer['tid']=1
    socket=event('socket_recv',21,22);socket['tid']=2
    r=a.analyze_events([timer,socket],[])
    assert r['long_events'][0]['overlapping_related_stages']['socket_recv']['count']==1
