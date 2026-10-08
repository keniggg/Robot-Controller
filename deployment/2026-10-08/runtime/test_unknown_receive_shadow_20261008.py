import ast
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

path=Path(__file__).with_name('run_unknown_receive_shadow_20261008.py')
spec=importlib.util.spec_from_file_location('unknown_shadow_wrapper',path)
w=importlib.util.module_from_spec(spec);spec.loader.exec_module(w)


def test_complete_explicit_remaps_and_no_production_output_target():
    ns=w.private_namespace(123,456);maps=w.remappings(ns,True)
    assert set(maps)==set(w.PRIVATE_INPUTS)
    assert all(maps[k]==ns+k for k in w.PRIVATE_INPUTS)
    assert all(k not in maps for k in w.OUTPUTS)
    assert '/camera' not in maps and '/unknown_perception' not in maps
    assert len(w.OUTPUTS)==8 and len(w.PRIVATE_INPUTS)==3


def test_deserializer_default_and_explicit_frozen_candidate_cli_contract():
    w.validate_deserializer_option('original',None)
    w.validate_deserializer_option('skip_pruned_copies','a'*64)
    for mode,sha in [('original','a'*64),('skip_pruned_copies',None),
                     ('skip_pruned_copies','bad'),('other',None)]:
        with pytest.raises(ValueError):w.validate_deserializer_option(mode,sha)
    tree=ast.parse(path.read_text())
    calls=[node for node in ast.walk(tree) if isinstance(node,ast.Call)
           and isinstance(node.func,ast.Attribute) and node.func.attr=='add_argument']
    argument=next(node for node in calls if node.args and isinstance(node.args[0],ast.Constant)
                  and node.args[0].value=='--deserializer')
    assert next(k.value.value for k in argument.keywords if k.arg=='default')=='original'


def candidate_fixture():
    calls=[]
    class Image:pass
    def original(b,msg_queue,data_class,queue_size=None,max_msgs=None,start=0):
        calls.append(('original',b,msg_queue,data_class,queue_size,max_msgs,start))
        if b=='raise':raise RuntimeError('original_failure')
        return msg_queue
    def make_candidate(fn,image_class):
        def candidate(b,msg_queue,data_class,queue_size=None,max_msgs=None,start=0):
            calls.append(('candidate',data_class is image_class))
            return fn(b,msg_queue,data_class,queue_size,max_msgs,start)
        stats={'active':True,'optimized_calls':1,'inflight':0,'skipped_payload_bytes':4}
        candidate.snapshot_statistics=lambda:stats
        candidate.deactivate=lambda:stats.update(active=False)
        return candidate
    tcp=SimpleNamespace(deserialize_messages=original)
    return tcp,SimpleNamespace(make_candidate=make_candidate),Image,original,calls


def test_candidate_alias_composes_inside_trace_and_restores_in_reverse_order():
    tcp,helper,image_class,original,calls=candidate_fixture()
    global_msg=SimpleNamespace(deserialize_messages=original)
    candidate=w.CandidateAlias(tcp,helper,image_class,original)
    inner=tcp.deserialize_messages
    def trace(b,msg_queue,data_class,queue_size=None,max_msgs=None,start=0):
        calls.append(('trace',))
        return inner(b,msg_queue,data_class,queue_size,max_msgs,start)
    tcp.deserialize_messages=trace
    queue=[]
    assert tcp.deserialize_messages('buffer',queue,image_class,2,3,4) is queue
    assert [r[0] for r in calls]==['trace','candidate','original']
    assert calls[-1][-3:]==(2,3,4)
    assert global_msg.deserialize_messages is original
    tcp.deserialize_messages=inner  # Original trace.stop ownership restoration.
    assert candidate.restore()==dict(original_alias_restored=True,candidate_deactivated=True,restore_conflicts=[])
    assert tcp.deserialize_messages is original
    assert candidate.restore()==dict(original_alias_restored=True,candidate_deactivated=True,restore_conflicts=[])
    frozen=candidate.statistics();frozen['optimized_calls']=999
    assert candidate.statistics()['optimized_calls']==1


def test_candidate_alias_never_overwrites_another_hook_or_accepts_changed_original():
    tcp,helper,image_class,original,_=candidate_fixture()
    candidate=w.CandidateAlias(tcp,helper,image_class,original)
    foreign=lambda *a:None
    tcp.deserialize_messages=foreign
    result=candidate.restore()
    assert result['original_alias_restored'] is False and result['restore_conflicts']
    assert tcp.deserialize_messages is foreign
    with pytest.raises(ValueError,match='original_deserializer_alias_changed'):
        w.CandidateAlias(tcp,helper,image_class,original)
    assert tcp.deserialize_messages is foreign


def test_candidate_alias_preserves_exception_and_installs_nothing_on_bad_factory():
    tcp,helper,image_class,original,_=candidate_fixture()
    candidate=w.CandidateAlias(tcp,helper,image_class,original)
    with pytest.raises(RuntimeError,match='original_failure'):
        tcp.deserialize_messages('raise',[],image_class)
    candidate.restore();assert tcp.deserialize_messages is original
    bad=SimpleNamespace(make_candidate=lambda *a:lambda *a:None)
    with pytest.raises(ValueError,match='ABI_mismatch'):
        w.CandidateAlias(tcp,bad,image_class,original)
    assert tcp.deserialize_messages is original


def test_candidate_deactivation_failure_still_restores_owned_alias_but_rejects_validation():
    tcp,helper,image_class,original,_=candidate_fixture()
    candidate=w.CandidateAlias(tcp,helper,image_class,original)
    def broken():raise RuntimeError('deactivation_failed')
    candidate.candidate.deactivate=broken
    result=candidate.restore()
    assert tcp.deserialize_messages is original and result['original_alias_restored']
    assert result['candidate_deactivated'] is False and result['restore_conflicts']
    assert not w.candidate_statistics_validation(candidate_statistics(),result)['passed']


def candidate_statistics():
    return dict(active=False,calls_started=3,calls_completed=3,calls_failed=0,inflight=0,optimized_calls=2,fallback_calls=1,
        scanned_payload_bytes=100,retained_payload_bytes=40,skipped_payload_bytes=60,
        materialized_payload_bytes=40,complete_messages_scanned=5,retained_messages=2)


def test_candidate_must_actually_run_but_no_backlog_is_not_copy_saving_evidence():
    restored=dict(original_alias_restored=True,candidate_deactivated=True,restore_conflicts=[])
    stats=candidate_statistics()
    valid=w.candidate_statistics_validation(stats,restored)
    assert valid['passed'] and valid['candidate_used'] and valid['copy_pruning_observed']
    no_skip=dict(stats,skipped_payload_bytes=0,retained_payload_bytes=100,materialized_payload_bytes=100)
    valid=w.candidate_statistics_validation(no_skip,restored)
    assert valid['passed'] and valid['candidate_used'] and not valid['copy_pruning_observed']
    unused=dict(stats,optimized_calls=0,fallback_calls=3,skipped_payload_bytes=0)
    valid=w.candidate_statistics_validation(unused,restored)
    assert not valid['passed'] and not valid['candidate_used']
    assert 'candidate_not_exercised' in valid['reasons']


@pytest.mark.parametrize('missing,reason',[
    ('complete_messages_scanned','candidate_no_complete_image_scanned'),
    ('retained_messages','candidate_no_complete_image_retained'),
])
def test_partial_only_candidate_calls_do_not_prove_image_decode_wiring(missing,reason):
    stats=candidate_statistics();stats[missing]=0
    restored=dict(original_alias_restored=True,candidate_deactivated=True,restore_conflicts=[])
    result=w.candidate_statistics_validation(stats,restored)
    assert not result['passed'] and reason in result['reasons']
    assert not result['candidate_used'] and not result['copy_pruning_observed']


@pytest.mark.parametrize('change',[
    {'inflight':1,'calls_completed':2},{'calls_failed':1,'calls_completed':2},
    {'calls_started':9},{'optimized_calls':True},{'skipped_payload_bytes':-1},
    {'skipped_payload_bytes':float('nan')},{'active':True},
])
def test_candidate_partial_or_invalid_statistics_fail_validation(change):
    stats=candidate_statistics();stats.update(change)
    result=w.candidate_statistics_validation(stats,dict(original_alias_restored=True,candidate_deactivated=True,restore_conflicts=[]))
    assert not result['passed']


def test_candidate_restore_conflict_prevents_complete_even_when_statistics_pass():
    result=w.candidate_statistics_validation(candidate_statistics(),
        dict(original_alias_restored=False,restore_conflicts=['foreign_hook']))
    assert not result['passed'] and 'candidate_alias_not_restored' in result['reasons']


def test_real_image_candidate_inside_original_trace_preserves_stage_and_late_call_cleanup(tmp_path,monkeypatch):
    """Installed Noetic framing/Image, no node, master, socket or device calls."""
    import io
    import struct
    monkeypatch.syspath_prepend('/opt/ros/noetic/lib/python3/dist-packages')
    import rospy
    import rospy.msg
    import rospy.impl.tcpros_base as tcp
    import rospy.topics as topic_module
    from sensor_msgs.msg import Image
    import genpy
    assert not rospy.core.is_initialized()
    candidate_spec=importlib.util.spec_from_file_location('wrapper_candidate_integration',w.CANDIDATE)
    helper=importlib.util.module_from_spec(candidate_spec);candidate_spec.loader.exec_module(helper)
    trace_spec=importlib.util.spec_from_file_location('wrapper_trace_integration',w.TRACE)
    trace_module=importlib.util.module_from_spec(trace_spec);trace_spec.loader.exec_module(trace_module)
    class Node:
        def frame(self,color,depth):return color,depth
        def process(self,_event):return _event
    payloads=[]
    for seq in range(5):
        message=Image();message.header.seq=seq;message.header.stamp=genpy.Time(100,seq+1)
        message.header.frame_id='original_optical';message.height=1;message.width=4
        message.encoding='mono8';message.step=4;message.data=bytes([seq]*4)
        stream=io.BytesIO();message.serialize(stream);raw=stream.getvalue()
        payloads.append(struct.pack('<I',len(raw))+raw)
    wire=b''.join(payloads)+struct.pack('<I',999)+b'incomplete'
    def buffer():
        b=io.BytesIO(wire);b.seek(0,2);return b
    reference=buffer();expected=[]
    original=rospy.msg.deserialize_messages
    original(reference,expected,Image,queue_size=2)
    binding=w.CandidateAlias(tcp,helper,Image,original)
    session=trace_module.TraceSession(tmp_path/'trace',('/color','/depth'),5.,2*1024*1024)
    try:
        trace_module._install(session,tcp,topic_module,Node)
        session.tls.receive=dict(connection_id=1,topic='/color',cycle=1,received_bytes=len(wire))
        traced_alias=tcp.deserialize_messages
        current=buffer();actual=[]
        protocol=tcp.TCPROSTransportProtocol('/color',Image,queue_size=2,buff_size=16*1024*1024)
        assert protocol.read_messages(current,actual,None) is None
        assert actual==expected and [m.header.seq for m in actual]==[3,4]
        assert current.getvalue()==reference.getvalue() and current.tell()==reference.tell()
        assert protocol.buff_size==16*1024*1024 and protocol.queue_size==2
    finally:
        session.stop('test_complete')
        restored=binding.restore()
        trace_result=session.finalize()
    statistics=binding.statistics()
    assert w.candidate_statistics_validation(statistics,restored)['passed']
    assert statistics['optimized_calls']==1 and statistics['skipped_payload_bytes']>0
    assert statistics['active'] is False and statistics['inflight']==0
    assert tcp.deserialize_messages is original and rospy.msg.deserialize_messages is original
    late=buffer();late_queue=[]
    assert traced_alias(late,late_queue,Image,queue_size=2) is None
    assert late_queue==expected and binding.statistics()==statistics
    rows=[json.loads(line) for line in (tmp_path/'trace/events.jsonl').read_text().splitlines()]
    row=next(r for r in rows if r['kind']=='deserialize_messages')
    assert [h['seq'] for h in row['wire_before']['complete']]==[0,1,2,3,4]
    assert [h['seq'] for h in row['actual_headers']]==[3,4]
    assert trace_result['original_method_bindings_restored'] and trace_result['observer_integrity_passed']
    assert not rospy.core.is_initialized()


@pytest.mark.parametrize('ns',['/','/perception/unknown','/diagnostics/unknown_receive_shadow_1_2/extra',
                               '/diagnostics/unknown_receive_shadow_1_2/../grasp_mode'])
def test_arbitrary_or_parent_escape_namespaces_rejected(ns):
    with pytest.raises(ValueError):w.remappings(ns)


def test_actual_rospy_parameter_and_topic_resolution_without_node_or_network(monkeypatch):
    """Exercise installed Noetic MasterProxy, replacing only its remote target."""
    monkeypatch.syspath_prepend('/opt/ros/noetic/lib/python3/dist-packages')
    import rospy
    import rospy.msproxy
    import rospy.impl.paramserver
    names=rospy.names
    for key in ('_mappings','_resolved_mappings','_caller_id','_caller_namespace'):
        monkeypatch.setattr(names,key,getattr(names,key))
    monkeypatch.setattr(rospy.impl.paramserver,'_param_server_cache',None)
    ns=w.private_namespace(987,123456);maps=w.remappings(ns,True)
    names.reload_mappings(['offline']+[k+':='+v for k,v in maps.items()])
    names._set_caller_id(ns+'/unknown_receive_shadow');names.initialize_mappings(names.get_name())
    assert not rospy.core.is_initialized()
    w.verify_resolutions(names.resolve_name,ns,maps,('/camera_test/color','/camera_test/depth'))
    seen=[];private=w.private_selection(1791400000000000001)
    class Target:
        def getParam(self,caller,key):seen.append(('get',key));return 1,'ok',private
        def subscribeParam(self,caller,uri,key):seen.append(('subscribe',key));return 1,'ok',private
    proxy=rospy.msproxy.MasterProxy.__new__(rospy.msproxy.MasterProxy);proxy.target=Target()
    assert proxy['/grasp_mode/selection']==private
    assert proxy.get_param_cached('/grasp_mode/selection')==private
    assert seen==[('get',ns+'/grasp_mode/selection'),('subscribe',ns+'/grasp_mode/selection')]
    assert names.resolve_name(w.PRODUCTION_STATUS)==w.PRODUCTION_STATUS
    assert names.resolve_name(ns+w.PRODUCTION_STATUS)==ns+w.PRODUCTION_STATUS
    assert not rospy.core.is_initialized()


def test_resolution_check_rejects_unmapped_output_or_camera_alias():
    ns=w.private_namespace(1,2);maps=w.remappings(ns)
    for badkey,badvalue in [(w.OUTPUTS[0],ns+w.OUTPUTS[0]),('/c','/other')]:
        changed=dict(maps);changed[badkey]=badvalue
        with pytest.raises(ValueError):w.verify_resolutions(lambda k:changed.get(k,k),ns,maps,('/c','/d'))


def test_selection_only_private_generation_one_exact_xmlrpc_safe_stamp():
    value=w.private_selection(1791400000000000001)
    assert value=={'mode':'unknown','strategy':'observe_return','generation':1,'stamp_ns':'1791400000000000001'}
    import xmlrpc.client
    xmlrpc.client.dumps((value,))
    with pytest.raises(ValueError):w.private_selection(True)


def test_parameter_cleanup_never_removes_parent_or_changed_value():
    ns=w.private_namespace(1,2);key=ns+'/grasp_mode/selection';own=w.private_selection(123)
    deleted=[];ros=SimpleNamespace(get_param=lambda *a:own,delete_param=deleted.append)
    assert w.cleanup_private_selection(ros,key,own,ns)=='deleted_own_exact_private_selection'
    assert deleted==[key]
    ros.get_param=lambda *a:{'other':'changed'}
    assert w.cleanup_private_selection(ros,key,own,ns)=='changed_private_value_preserved'
    with pytest.raises(ValueError):w.cleanup_private_selection(ros,'/grasp_mode/selection',own,ns)
    with pytest.raises(ValueError):w.cleanup_private_selection(ros,ns,own,ns)
    assert deleted==[key]


def test_runtime_node_pub_sub_verification_catches_production_publisher():
    ns=w.private_namespace(1,2)
    def topic(name):return SimpleNamespace(resolved_name=name,impl=SimpleNamespace(resolved_name=name))
    node=SimpleNamespace(pubs={str(i):topic(ns+name) for i,name in enumerate(w.OUTPUTS)},
        selection_sub=topic(ns+w.PRIVATE_INPUTS[0]),
        rgb_reference_sub=topic(ns+w.PRIVATE_INPUTS[1]),
        rgb_acquisition_sub=topic(ns+w.PRIVATE_INPUTS[2]),
        color_sub=SimpleNamespace(sub=SimpleNamespace(resolved_name='/c')),
        depth_sub=SimpleNamespace(sub=SimpleNamespace(resolved_name='/d')))
    w.verify_node_topics(node,ns,('/c','/d'))
    node.pubs['0'].resolved_name=w.OUTPUTS[0]
    with pytest.raises(ValueError):w.verify_node_topics(node,ns,('/c','/d'))


def test_status_recorder_preserves_both_raw_json_strings_and_source_receipts(tmp_path):
    recorder=w.StatusRecorder(tmp_path)
    original='{"generation":25, "detail":"target lost", "source_stamp_ns":1791400000000000001}'
    recorder.offer(original,'production',w.PRODUCTION_STATUS,dict(monotonic=10.))
    recorder.offer('{"generation":1}','shadow','/diagnostics/shadow/status',dict(monotonic=11.))
    result=recorder.finish();assert result['sealed'] and result['rows']==2
    raw=(tmp_path/'detector_status.jsonl').read_bytes();rows=[json.loads(line) for line in raw.splitlines()]
    assert rows[0]['original_status_data']==original
    assert rows[0]['original_status_utf8_sha256']==hashlib.sha256(original.encode()).hexdigest()
    assert rows[0]['receipt']['monotonic']==10.
    assert rows[1]['kind']=='shadow'
    assert result['sha256']==hashlib.sha256(raw).hexdigest()
    assert not result['contains_pixels']


def test_status_budget_and_repeated_finish_are_bounded(tmp_path):
    recorder=w.StatusRecorder(tmp_path,maximum_bytes=100)
    recorder.offer('{}','shadow','/diagnostics/a',{})
    result=recorder.finish();again=recorder.finish()
    assert result['sealed'] and result['rows']==0 and result['limit_reached']
    assert result==again
    assert (tmp_path/'detector_status.jsonl').stat().st_size<=100


def test_dual_status_budget_reallocation_keeps_total_and_reserved_overhead(tmp_path):
    mib=1024*1024
    assert (w.TRACE_BYTES,w.STATUS_BYTES,w.TOTAL_BYTES)==(50*mib,12*mib,64*mib)
    assert w.TRACE_BYTES+w.STATUS_BYTES+2*mib==w.TOTAL_BYTES
    recorder=w.StatusRecorder(tmp_path)
    try:
        assert recorder.maximum_bytes==w.STATUS_BYTES
    finally:
        assert recorder.finish()['sealed']


def test_status_row_limit_and_late_callback_cannot_append(tmp_path):
    recorder=w.StatusRecorder(tmp_path,maximum_rows=1)
    recorder.offer('{}','shadow','/diagnostics/a',{})
    recorder.offer('{}','production',w.PRODUCTION_STATUS,{})
    result=recorder.finish();before=(tmp_path/'detector_status.jsonl').read_bytes()
    recorder.offer('{}','shadow','/diagnostics/a',{})
    assert (tmp_path/'detector_status.jsonl').read_bytes()==before
    assert result['rows']==1 and result['counts']['budget_rejected']==1


def test_status_oversize_does_not_enter_queue(tmp_path):
    recorder=w.StatusRecorder(tmp_path)
    recorder.offer('x'*(128*1024+1),'shadow','/diagnostics/a',{})
    result=recorder.finish()
    assert result['rows']==0 and result['limit_reached']


def test_live_entry_has_no_production_service_or_subprocess_control():
    tree=ast.parse(path.read_text())
    calls=[node.func.attr for node in ast.walk(tree) if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute)]
    assert 'ServiceProxy' not in calls and 'Popen' not in calls and 'kill' not in calls
    assert calls.count('set_param')==1 and calls.count('delete_param')==1
    run_calls=[node for node in ast.walk(tree) if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute) and node.func.attr=='run_path']
    assert len(run_calls)==1
    assert next(kw.value.value for kw in run_calls[0].keywords if kw.arg=='run_name')!='__main__'


def test_frozen_original_source_still_matches_and_no_live_import_during_wrapper_import():
    assert hashlib.sha256(w.SOURCE.read_bytes()).hexdigest()==w.SOURCE_SHA
    assert w.TRACE_BYTES+w.STATUS_BYTES< w.TOTAL_BYTES
    assert w.own_start_ticks()>0


@pytest.mark.parametrize('reason',['event_budget','connection_budget','byte_budget','queue_full','writer_error','requested',
                                  'status_writer_error_or_budget'])
def test_sealed_early_budget_or_stop_is_never_complete(reason):
    summary=dict(sealed=True,observer_integrity_passed=True,original_method_bindings_restored=True,
        restore_conflicts=[],stop_reason=reason,started_monotonic=10.,deadline_monotonic=100.,stopped_monotonic=30.)
    assert not w.trace_window_complete(summary,10.,100.)


def test_trace_must_reach_original_deadline_and_pass_integrity():
    summary=dict(sealed=True,observer_integrity_passed=True,original_method_bindings_restored=True,
        restore_conflicts=[],stop_reason='deadline',started_monotonic=10.,deadline_monotonic=100.,stopped_monotonic=100.)
    assert w.trace_window_complete(summary,10.,100.)
    for key,value in [('stopped_monotonic',99.9),('stopped_monotonic',float('nan')),
                      ('observer_integrity_passed',False),('deadline_monotonic',80.),
                      ('restore_conflicts',['frame'])]:
        bad=dict(summary);bad[key]=value
        assert not w.trace_window_complete(bad,10.,100.)


@pytest.fixture
def actual_noetic_topics(monkeypatch):
    """Original Topic/TopicImpl/TopicManager bodies; no registration or sockets."""
    monkeypatch.syspath_prepend('/opt/ros/noetic/lib/python3/dist-packages')
    import rospy
    import rospy.topics as topics
    from std_msgs.msg import String
    names=rospy.names
    for key in ('_mappings','_resolved_mappings','_caller_id','_caller_namespace'):
        monkeypatch.setattr(names,key,getattr(names,key))
    ns=w.private_namespace(321,654)
    names._set_caller_id(ns+'/unknown_receive_shadow')
    def set_maps(maps):
        names.reload_mappings(['offline']+[k+':='+v for k,v in maps.items()])
        names.initialize_mappings(names.get_name())
    set_maps(w.remappings(ns,True))
    assert not rospy.core.is_initialized()
    # Force post-init name resolution without calling init_node or any master.
    monkeypatch.setattr(rospy.core,'is_initialized',lambda:True)
    monkeypatch.setattr(topics.SubscriberStatisticsLogger,'is_enabled',staticmethod(lambda:False))
    listeners=SimpleNamespace(notify_added=lambda *a:None,notify_removed=lambda *a:None)
    monkeypatch.setattr(topics,'get_registration_listeners',lambda:listeners)
    manager=topics._TopicManager()
    monkeypatch.setattr(topics,'get_topic_manager',lambda:manager)
    yield rospy,topics,String,ns,manager,set_maps
    manager.close_all()


def test_actual_noetic_body_reproduces_rejected_old_double_remap(actual_noetic_topics):
    ros,topics,String,ns,manager,set_maps=actual_noetic_topics
    alias=ns+'/production_detector_input'
    set_maps({w.PRODUCTION_STATUS:ns+w.PRODUCTION_STATUS,alias:w.PRODUCTION_STATUS})
    old_shadow=topics.Subscriber(ns+w.PRODUCTION_STATUS,String,lambda _:None,queue_size=4)
    old_alias=topics.Subscriber(alias,String,lambda _:None,queue_size=4)
    assert old_alias.resolved_name==w.PRODUCTION_STATUS
    assert old_alias.impl.resolved_name==ns+w.PRODUCTION_STATUS
    assert manager.subs[ns+w.PRODUCTION_STATUS] is old_alias.impl
    assert old_shadow.impl is not old_alias.impl
    old_shadow.unregister()
    with pytest.raises(AssertionError,match='cannot release topic impl'):
        old_alias.unregister()
    old_alias.impl.close()


def test_actual_noetic_status_observers_are_distinct_deliver_and_unregister(actual_noetic_topics):
    ros,topics,String,ns,manager,set_maps=actual_noetic_topics
    got=[]
    shadow=topics.Subscriber(ns+w.PRODUCTION_STATUS,String,lambda msg:got.append(('shadow',json.loads(msg.data)['generation'])),queue_size=4)
    production=topics.Subscriber(w.PRODUCTION_STATUS,String,lambda msg:got.append(('production',json.loads(msg.data)['generation'])),queue_size=4)
    assert shadow.resolved_name==shadow.impl.resolved_name==ns+w.PRODUCTION_STATUS
    assert production.resolved_name==production.impl.resolved_name==w.PRODUCTION_STATUS
    assert shadow.impl is not production.impl
    shadow.impl.receive_callback([String(data='{"generation":1}')],None)
    production.impl.receive_callback([String(data='{"generation":25}')],None)
    assert got==[('shadow',1),('production',25)]
    shadow.unregister();production.unregister()
    assert manager.subs=={}


def test_actual_original_publishers_construct_only_private_ros_objects(actual_noetic_topics):
    ros,topics,String,ns,manager,set_maps=actual_noetic_topics
    proxy=w.PrivatePublisherRos(ros,ns)
    publishers=[proxy.Publisher(name,String,queue_size=2,latch=True) for name in w.OUTPUTS]
    assert set(manager.pubs)=={ns+name for name in w.OUTPUTS}
    assert all(pub.resolved_name==pub.impl.resolved_name and pub.impl.queue_size==2 and pub.impl.is_latch for pub in publishers)
    with pytest.raises(ValueError):proxy.Publisher('/robot/action',String,queue_size=1)
    for pub in publishers:pub.unregister()
    assert manager.pubs=={}


def test_constructor_proxy_restores_globals_on_success_and_failure():
    calls=[]
    ros=SimpleNamespace(Publisher=lambda *a,**k:calls.append((a,k)),marker='unchanged')
    scope={'rospy':ros}
    exec('class Example:\n def __init__(self):\n  rospy.Publisher("/perception/unknown/detector_status", object, queue_size=2)\n  self.marker=rospy.marker\n',scope)
    ns=w.private_namespace(1,2)
    node=w.construct_isolated_node(scope['Example'],ros,ns)
    assert scope['rospy'] is ros and node.marker=='unchanged'
    assert calls[0][0][0]==ns+w.PRODUCTION_STATUS
    exec('class Failing:\n def __init__(self):\n  raise RuntimeError("original failure")\n',scope)
    with pytest.raises(RuntimeError,match='original failure'):
        w.construct_isolated_node(scope['Failing'],ros,ns)
    assert scope['rospy'] is ros
