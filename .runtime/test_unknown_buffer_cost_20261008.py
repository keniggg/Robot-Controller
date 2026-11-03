import importlib.util
from pathlib import Path
import pytest


def mod(name):
    spec=importlib.util.spec_from_file_location(name,Path(__file__).with_name(name+'.py'))
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m


q=mod('analyze_unknown_buffer_cost_20261008');r=mod('replay_unknown_receive_copy_20261008')


def event(kind,call_id,start,end,cpu_start,cpu_end,**kw):
    return dict(kind=kind,call_id=call_id,tid=4,connection_id=1,receive_cycle=3,
        start={'monotonic_ns':start,'thread_cpu_ns':cpu_start},
        end={'monotonic_ns':end,'thread_cpu_ns':cpu_end},elapsed_ns=end-start,thread_cpu_ns=cpu_end-cpu_start,**kw)


def test_exact_nested_after_socket_cpu_and_byte_invariants():
    b=event('recv_buff',1,0,1000,0,400,returned_bytes=60,buffer_bytes_before=40,buffer_bytes_after=100)
    s=event('socket_recv',2,10,100,10,30,returned_bytes=60)
    result=q.quantify([b,s],[]);row=result['largest_after_socket'][0]
    assert row['after_socket_wall_ms']==pytest.approx(.0009)
    assert row['after_socket_cpu_ms']==pytest.approx(.00037)
    assert row['after_socket_wall_minus_thread_cpu_ms']==pytest.approx(.00053)
    b['buffer_bytes_after']=99
    assert q.quantify([b,s],[])['excluded']['recv_byte_invariant_failed']==1


def test_nonunique_socket_is_excluded_not_guessed():
    b=event('recv_buff',1,0,1000,0,400)
    s=event('socket_recv',2,10,100,10,30)
    assert q.quantify([b,s,dict(s,call_id=3)],[])['excluded']['recv_without_unique_contained_socket']==1


def test_batch_prune_copy_counts_and_missing_peek():
    d=event('deserialize_messages',1,1,100,1,30,queue_size=2,
       wire_before={'complete_count_exact':True,'complete_count':3,'buffer_bytes':40,
       'complete':[{'payload_bytes':5},{'payload_bytes':6},{'payload_bytes':7}]})
    v=q.quantify([d],[])['largest_deserialize'][0]
    assert v['copied_complete_body_bytes']==18 and v['discarded_complete_body_bytes']==5
    d['wire_before']['complete_count_exact']=False
    assert not q.quantify([d],[])['largest_deserialize']


def test_constant_input_cannot_supply_correlation():
    assert q.correlation([(1,2)]*3)['pearson'] is None
    assert q.correlation([(1,2)])['reason']=='fewer_than_three'


def test_real_original_deserialize_keeps_final_two_complete_messages():
    bindings=r.original_bindings()
    wire=b''.join(r.synthetic_packet(2048,i) for i in range(3))
    out=r.run_once(wire,100,3,bindings)
    assert out['retained_sequences']==[1,2] and out['returned_bytes']==6044
    assert out['append']['cpu_ns']>=0 and out['deserialize']['cpu_ns']>=0


def test_replay_argument_memory_and_runtime_limits():
    with pytest.raises(ValueError,match='16MiB'):r.benchmark(1024*1024,17,0)
    with pytest.raises(ValueError,match='bounded'):r.benchmark(4096,1,0,100)
    with pytest.raises(ValueError,match='bounded'):r.benchmark(4096,1,0,1,6)
    with pytest.raises(ValueError,match='packet'):r.synthetic_packet(55,0)


def test_partial_tail_preserves_exact_original_residual():
    bindings=r.original_bindings();tail=r.synthetic_packet(2048,3)[:300]
    wire=b''.join(r.synthetic_packet(2048,i) for i in range(3))+tail
    out=r.run_once(wire,117,3,bindings,tail)
    assert out['retained_sequences']==[1,2] and out['residual_bytes']==300
