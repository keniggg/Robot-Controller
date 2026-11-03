import argparse
import copy
import importlib.util
import json
from pathlib import Path
import signal
import threading
import time
from xmlrpc.server import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler

import pytest

PATH=Path(__file__).with_name('observe_sdk_memory_pressure_20261007.py')
spec=importlib.util.spec_from_file_location('sdk_memory_observer',PATH)
M=importlib.util.module_from_spec(spec);spec.loader.exec_module(M)


def stat(ticks=10):
    fields=['0']*24
    for index,value in [(0,'S'),(7,'2'),(9,'3'),(11,'4'),(12,'5'),(19,str(ticks)),(21,'6')]: fields[index]=value
    return '123 (a name (with brackets)) '+' '.join(fields)


def envelope(source=1791382110373659849):
    entry={'source_stamp_ns':source,'read_completed_monotonic':time.monotonic(),'geometry_generation':2,
        'geometry_activation_stamp_ns':'1791345142349426984','evidence':{'raw_frames':{'color':{
            'frame_number':100,'metadata_raw':{'frame_timestamp':2097739822,'actual_fps':704}}}}}
    payload={'schema':1,'capacity':32,'geometry_generation':2,'geometry_activation_stamp_ns':'1791345142349426984',
        'diagnostic_only':True,'motion_authorized':False,'entries':[entry]}
    return {'schema':1,'encoding':'json','payload_json':json.dumps(payload)}


class Proxy:
    def __init__(self,value=None): self.value=value if value is not None else envelope();self.calls=[]
    def getParam(self,*args): self.calls.append(args);return 1,'ok',self.value
    def __exit__(self,*args): pass


def fixed_proc(tmp_path):
    proc=tmp_path/'proc';sysroot=tmp_path/'sys';proc.mkdir();sysroot.mkdir()
    def put(rel,value):
        p=proc/rel;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(value)
    put('123/comm','camera');put('123/stat',stat());put('123/task/123/stat',stat());put('123/task/123/comm','camera')
    put('123/task/123/schedstat','1000 2000 3');put('123/task/123/wchan','0')
    put('123/status','VmRSS: 1024 kB\nVmSwap: 4 kB\nThreads: 1\n')
    put('vmstat','pswpout 3\npgscan_kswapd 12\npgsteal_direct 4\ncompact_stall 0\nballoon_inflate 0\nnr_dirty 5\n')
    put('meminfo','MemAvailable: 123 kB\nSwapFree: 100 kB\n')
    put('diskstats','8 0 sda 1 0 4 0 2 0 8 1 0 1 1\n');put('stat','cpu 1 2 3 4 5 6 7 8\n')
    for kind in ('cpu','memory','io'): put('pressure/'+kind,'some avg10=0.00 total=2\nfull avg10=0.00 total=0\n')
    for key in ('full_scans','pages_collapsed'):
        p=sysroot/'kernel/mm/transparent_hugepage/khugepaged'/key;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('4')
    return proc,sysroot,put


def test_stat_parenthesis_and_missing():
    assert M.parse_stat(stat())==dict(state='S',minflt=2,majflt=3,utime_ticks=4,stime_ticks=5,start_ticks=10,rss_pages=6)
    assert M.parse_stat({'error':'FileNotFound'})['error']=='stat_unavailable'
    assert M.parse_stat('truncated')['error']=='malformed_stat'


@pytest.mark.parametrize('uri',['https://127.0.0.1:11311','http://some-host:11311','http://u:p@127.0.0.1:11311','http://127.0.0.1/?x=1'])
def test_master_validation_prevents_dns_or_auth(uri):
    with pytest.raises(ValueError): M.validate_master_uri(uri)


def test_localhost_normalized():
    assert M.validate_master_uri('http://localhost:11311')=='http://127.0.0.1:11311/'


def test_original_source_and_native_integer_preserved():
    original=envelope();before=copy.deepcopy(original);payload=M.decode_parameter(original)
    assert payload['entries'][0]['source_stamp_ns']==1791382110373659849
    assert payload['entries'][0]['evidence']['raw_frames']['color']['metadata_raw']['frame_timestamp']==2097739822
    assert original==before


@pytest.mark.parametrize('mutation',[lambda d:d.update(capacity=33),lambda d:d.update(entries=[{}]),
    lambda d:d.update(motion_authorized=True),lambda d:d['entries'][0].update(geometry_generation=True),
    lambda d:d['entries'][0].update(read_completed_monotonic=True)])
def test_invalid_schema_cannot_claim_valid(mutation):
    e=envelope();p=json.loads(e['payload_json']);mutation(p);e['payload_json']=json.dumps(p)
    with pytest.raises(ValueError): M.decode_parameter(e)


def test_record_budget_closes_capture_and_keeps_prior():
    stop=threading.Event();r=M.Records(stop,budget=20)
    assert r.append('rpc',{'a':1})
    assert not r.append('rpc',{'too':'long for remaining budget'})
    assert stop.is_set() and r.close()['counts']['rpc']==1
    assert not r.append('rpc',{})
    assert r.bytes<=20


def test_record_row_bound(monkeypatch):
    monkeypatch.setattr(M,'MAX_ROWS',1);stop=threading.Event();r=M.Records(stop)
    assert r.append('resources',{})
    assert not r.append('resources',{})
    assert r.reason=='record_count_limit:resources'


def test_rpc_exact_entries_deduplicated_and_change_rejected():
    stop=threading.Event();r=M.Records(stop);w=M.RpcWorker('http://127.0.0.1',r,stop,1)
    p=Proxy();w.poll(p);w.poll(p)
    assert len(r.lines['sdk'])==1 and len(r.lines['rpc'])==2
    assert p.calls==[('/passive_sdk_memory_observer','/camera_read_diagnostics')]*2
    body=json.loads(p.value['payload_json']);body['entries'][0]['evidence']['raw_frames']['color']['frame_number']=101
    p.value['payload_json']=json.dumps(body);w.poll(p)
    last=json.loads(r.lines['rpc'][-1]);assert not last['schema_valid']
    assert 'same_owner_source_changed_evidence' in last['error']
    assert len(r.lines['sdk'])==1


def test_missing_parameter_explicit():
    p=Proxy();p.getParam=lambda *a:(-1,'not set',0)
    stop=threading.Event();r=M.Records(stop);M.RpcWorker('http://127.0.0.1',r,stop,1).poll(p)
    assert not json.loads(r.lines['rpc'][0])['schema_valid']
    assert not r.lines['sdk']


def test_bind_wrong_pid_start_rejected(tmp_path):
    proc,_,_=fixed_proc(tmp_path)
    with pytest.raises(ValueError,match='identity_mismatch'): M.bind_processes(['123:11'],proc)


def test_fixed_threads_not_rediscovered_and_status_1hz(tmp_path):
    proc,sysroot,put=fixed_proc(tmp_path);bound=M.bind_processes(['123:10'],proc)
    put('123/task/124/stat',stat(11));put('123/task/124/comm','late')
    sampler=M.ProcSampler(bound,proc,sysroot);a=sampler.sample();b=sampler.sample()
    assert list(a['processes']['123']['tasks'])==['123']
    assert 'status' in a['processes']['123'] and 'status' not in b['processes']['123']
    assert a['vmstat']['balloon_inflate']==0 and a['vmstat']['pgscan_kswapd']==12
    assert a['quality_gaps']==[]


@pytest.mark.parametrize('changed',['process','task'])
def test_identity_changed_during_proc_read_discards_details(tmp_path,monkeypatch,changed):
    proc,sysroot,_=fixed_proc(tmp_path);bound=M.bind_processes(['123:10'],proc);real=M.read_text;counts=CounterForTest()
    path=proc/'123'/('stat' if changed=='process' else 'task/123/stat')
    def read(p,*args):
        if Path(p)==path:
            counts.value+=1
            return stat(10 if counts.value==1 else 20)
        return real(p,*args)
    monkeypatch.setattr(M,'read_text',read)
    row=M.ProcSampler(bound,proc,sysroot).sample();item=row['processes']['123']
    if changed=='task': item=item['tasks']['123']
    assert item['mixed_interval_details_discarded'] and 'stat' not in item
    assert row['quality_gaps']


class CounterForTest:
    value=0


def test_blocked_rpc_does_not_block_proc_sampling(tmp_path):
    proc,sysroot,_=fixed_proc(tmp_path);bound=M.bind_processes(['123:10'],proc)
    release=threading.Event();entered=threading.Event();p=Proxy()
    def get(*args): entered.set();release.wait(2);return 1,'ok',envelope()
    p.getParam=get;stop=threading.Event();r=M.Records(stop);w=M.RpcWorker('http://127.0.0.1',r,stop,time.monotonic()+2,lambda:p)
    w.thread.start();assert entered.wait(.5)
    start=time.monotonic();row=M.ProcSampler(bound,proc,sysroot).sample()
    assert time.monotonic()-start<.2 and not row['quality_gaps']
    stop.set();release.set();w.thread.join(1);assert not w.thread.is_alive()


def test_real_loopback_rpc_timeout_is_bounded():
    server=SimpleXMLRPCServer(('127.0.0.1',0),logRequests=False,allow_none=True)
    def get(*args): time.sleep(1.3);return 1,'late',envelope()
    server.register_function(get,'getParam');thread=threading.Thread(target=server.handle_request,daemon=True);thread.start()
    stop=threading.Event();r=M.Records(stop);uri='http://127.0.0.1:'+str(server.server_address[1])
    w=M.RpcWorker(uri,r,stop,time.monotonic()+2);proxy=w.proxy_factory();start=time.monotonic();w.poll(proxy)
    assert .8<time.monotonic()-start<1.5
    assert not json.loads(r.lines['rpc'][0])['schema_valid']
    proxy.__exit__(None,None,None);thread.join(1);server.server_close()


def test_loopback_negotiates_identity_with_gzip_capable_server():
    headers=[]
    class Handler(SimpleXMLRPCRequestHandler):
        encode_threshold=0
        def do_POST(self):
            headers.append(self.headers.get('Accept-Encoding'))
            return super().do_POST()
    server=SimpleXMLRPCServer(('127.0.0.1',0),requestHandler=Handler,logRequests=False,allow_none=True)
    server.register_function(lambda *a:(1,'ok',envelope()),'getParam')
    thread=threading.Thread(target=server.handle_request,daemon=True);thread.start()
    stop=threading.Event();records=M.Records(stop)
    worker=M.RpcWorker('http://127.0.0.1:'+str(server.server_address[1]),records,stop,time.monotonic()+2)
    proxy=worker.proxy_factory();worker.poll(proxy);proxy.__exit__(None,None,None);thread.join(1);server.server_close()
    assert headers==['identity']
    assert json.loads(records.lines['rpc'][0])['schema_valid']
    assert len(records.lines['sdk'])==1


def test_unsolicited_gzip_still_rejected():
    class Response:
        def getheader(self,*a): return 'gzip'
    with pytest.raises(ValueError,match='Compressed RPC response'):
        M.LimitedTransport().parse_response(Response())


def test_five_consecutive_rpc_errors_stop_with_visible_first_error(capsys):
    proxy=Proxy();proxy.getParam=lambda *a:(-1,'not set',0)
    stop=threading.Event();records=M.Records(stop);worker=M.RpcWorker('http://127.0.0.1',records,stop,1)
    for _ in range(5): worker.poll(proxy)
    assert stop.is_set() and records.reason=='rpc_consecutive_failures'
    assert len(records.lines['rpc'])==5 and not records.lines['sdk']
    assert capsys.readouterr().out.count('CAMERA_METADATA_ERROR:')==1


def test_saved_real_ros_envelope_decode_without_live_calls():
    path=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/memory_latency_round_20261008T011102Z/uncompressed_parameter_probe.json')
    if not path.exists(): pytest.skip('sealed local real-ROS fixture is external and optional elsewhere')
    raw=path.read_bytes()
    assert M.digest(raw)=='cc6f694048d83508d17332d7533df9095bb23717a73ace6d76eb00e453bfa119'
    result=json.loads(raw)['response'];assert result[0]==1
    payload=M.decode_parameter(result[2]);assert len(payload['entries'])==32
    assert payload['geometry_generation']==2


def fake_capture(tmp_path,monkeypatch,sampler_kind='normal',worker_kind='normal'):
    out=tmp_path/'capture'
    def prepare(_): out.mkdir();return out
    monkeypatch.setattr(M,'prepare_output',prepare)
    class Mounted:
        def is_mount(self): return True
    monkeypatch.setattr(M,'MOUNT',Mounted())
    monkeypatch.setattr(M,'bind_processes',lambda *_:{})
    original_read=M.read_text
    monkeypatch.setattr(M,'read_text',lambda path,*a:'fixed-boot' if str(path).endswith('boot_id') else '0' if str(path).endswith('sched_schedstats') else original_read(path,*a))
    class Sampler:
        def __init__(self,*args): pass
        def sample(self):
            if sampler_kind=='raise': raise RuntimeError('synthetic proc failure')
            if sampler_kind=='signal': signal.raise_signal(signal.SIGTERM)
            now=time.monotonic()
            return {'started_monotonic':now,'finished_monotonic':now,'read_cost_sec':0,'quality_gaps':[]}
    monkeypatch.setattr(M,'ProcSampler',Sampler)
    original_worker=M.RpcWorker
    if worker_kind in ('alive','empty'):
        class LateThread:
            def start(self): pass
            def join(self,seconds): pass
            def is_alive(self): return worker_kind=='alive'
        class LateWorker:
            def __init__(self,*args): self.thread=LateThread();self.errors={};self.seen={};self.read_times={}
        monkeypatch.setattr(M,'RpcWorker',LateWorker)
    else:
        monkeypatch.setattr(M,'RpcWorker',lambda u,r,s,d:original_worker(u,r,s,d,lambda:Proxy()))
    args=argparse.Namespace(seconds=.03,output=str(out),master_uri='http://127.0.0.1:11311',watch_pid=[])
    rc=M.run(args)
    return rc,out,json.loads((out/'outcome.json').read_text())


@pytest.mark.parametrize('kind,reason',[('raise','exception'),('signal','signal')])
def test_failure_or_signal_seals_partial(tmp_path,monkeypatch,kind,reason):
    previous=signal.getsignal(signal.SIGTERM)
    rc,outcome_dir,result=fake_capture(tmp_path,monkeypatch,kind)
    assert rc==2 and result['stop_reason']==reason and result['capture_closed']
    assert not result['observer_integrity_passed']
    assert (outcome_dir/'SHA256SUMS').exists()
    assert signal.getsignal(signal.SIGTERM)==previous


def test_late_rpc_never_claims_metadata_complete(tmp_path,monkeypatch):
    rc,_,result=fake_capture(tmp_path,monkeypatch,worker_kind='alive')
    assert rc==2 and result['rpc_thread_alive_at_seal']
    assert not result['observer_integrity_passed'] and not result['sdk_evidence_available']


def test_completed_window_without_sdk_is_nonzero_exit(tmp_path,monkeypatch):
    rc,_,result=fake_capture(tmp_path,monkeypatch,worker_kind='empty')
    assert rc==2 and result['stop_reason']=='duration_elapsed'
    assert not result['observer_integrity_passed'] and not result['sdk_evidence_available']


def test_complete_local_capture_hashes_and_closed_schedstats_limit(tmp_path,monkeypatch):
    rc,out,result=fake_capture(tmp_path,monkeypatch)
    assert rc==0 and result['observer_integrity_passed']
    ready=json.loads((out/'ready.json').read_text())
    assert ready['schedstat_zero_does_not_prove_no_delay'] is True
    assert result['monitor_self_usage']['cpu_seconds_over_capture']>=0
    assert 'VmRSS' in result['monitor_self_usage']['capture_end_before_file_writes']['current_proc_status']
    for line in (out/'SHA256SUMS').read_text().splitlines():
        expected,name=line.split('  ',1);assert M.digest((out/name).read_bytes())==expected
    assert sum(p.stat().st_size for p in out.iterdir())<=M.TOTAL_BYTES


def test_partial_startup_identity_error_has_outcome(tmp_path,monkeypatch):
    out=tmp_path/'x';monkeypatch.setattr(M,'prepare_output',lambda _: (out.mkdir() or out))
    class Mounted:
        def is_mount(self): return True
    monkeypatch.setattr(M,'MOUNT',Mounted())
    monkeypatch.setattr(M,'read_text',lambda *a:'fixed-boot')
    def fail(*args): raise ValueError('watch_process_identity_mismatch:123')
    monkeypatch.setattr(M,'bind_processes',fail)
    rc=M.run(argparse.Namespace(seconds=.1,output=str(out),master_uri='http://127.0.0.1',watch_pid=['123:1']))
    result=json.loads((out/'outcome.json').read_text())
    assert rc==2 and result['capture_start_monotonic'] is None
    assert result['record_state']['counts']==dict(resources=0,rpc=0,sdk=0)


def test_signal_during_sealing_keeps_handler_until_files_saved(tmp_path,monkeypatch):
    original=M.write_atomic;seen=[]
    def write(path,parts):
        if path.name=='resources.jsonl':
            seen.append(signal.getsignal(signal.SIGTERM));signal.raise_signal(signal.SIGTERM)
        return original(path,parts)
    monkeypatch.setattr(M,'write_atomic',write)
    rc,out,result=fake_capture(tmp_path,monkeypatch)
    assert (out/'SHA256SUMS').exists() and result['signals']
    assert rc==2 and not result['observer_integrity_passed']
    assert all(callable(handler) for handler in seen)


@pytest.mark.parametrize('seconds',[0,301,float('nan')])
def test_duration_bounds_before_output(seconds):
    with pytest.raises(ValueError): M.run(argparse.Namespace(seconds=seconds))
