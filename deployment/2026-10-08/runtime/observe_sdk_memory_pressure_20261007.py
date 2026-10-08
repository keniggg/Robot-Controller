#!/usr/bin/env python3
"""Bounded passive SDK/proc observation. No collection without --run.

Only ROS master getParam('/camera_read_diagnostics') is used, on its own thread.
No ROS node, services, publications, camera SDK/device queries, or sysctl writes.
"""
import argparse
from collections import Counter
import hashlib
import http.client
import ipaddress
import json
import math
import os
from pathlib import Path
import resource
import signal
import threading
import time
from urllib.parse import urlsplit
import xmlrpc.client

EXTERNAL = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')
MOUNT = Path('/media/zhuyupei/SanStand')
PARAMETER = '/camera_read_diagnostics'
PERIOD = .5
RPC_TIMEOUT = 1.0
TOTAL_BYTES = 32 * 1024**2
RESERVE_BYTES = 256 * 1024
MAX_RESPONSE = 2 * 1024**2
MAX_ITEM = 1024**2
MAX_ROWS = 620
MAX_SDK = 10000
MAX_PIDS = 12
MAX_TASKS = 64
FILES = {'resources':'resources.jsonl', 'rpc':'camera_rpc.jsonl', 'sdk':'sdk_entries.jsonl'}


def encode(value):
    return (json.dumps(value, ensure_ascii=False, separators=(',', ':'), allow_nan=False)+'\n').encode('utf8')


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def read_text(path, maximum=256*1024):
    try:
        with Path(path).open() as stream:
            raw = stream.read(maximum+1)
        if len(raw)>maximum:
            return {'error':'read_size_limit', 'path':str(path)}
        return raw.strip()
    except OSError as exc:
        return {'error':type(exc).__name__, 'errno':exc.errno, 'path':str(path)}


def parse_stat(raw):
    if not isinstance(raw,str):
        return {'error':'stat_unavailable', 'detail':raw}
    try:
        f=raw.rsplit(')',1)[1].split()
        return {'state':f[0], 'minflt':int(f[7]), 'majflt':int(f[9]),
                'utime_ticks':int(f[11]), 'stime_ticks':int(f[12]),
                'start_ticks':int(f[19]), 'rss_pages':int(f[21])}
    except (ValueError,IndexError):
        return {'error':'malformed_stat', 'raw':raw}


def numeric_lines(raw, prefixes=(), keys=()):
    if not isinstance(raw,str):
        return {'error':'unavailable', 'detail':raw}
    result={}
    try:
        for line in raw.splitlines():
            key,value=line.split()
            if key in keys or key.startswith(prefixes): result[key]=int(value)
    except ValueError:
        return {'error':'malformed_numeric_lines', 'raw':raw}
    return result


def selected_fields(raw, keys):
    if not isinstance(raw,str): return {'error':'unavailable', 'detail':raw}
    return {k:v.strip() for k,v in (line.split(':',1) for line in raw.splitlines() if ':' in line) if k in keys}


def diagnostic_gaps(value,prefix=''):
    result=[]
    if isinstance(value,dict):
        for key,item in value.items():
            path=prefix+'.'+key if prefix else key
            if key=='error' or key.startswith('identity_error') or key=='binding_error': result.append(path)
            elif isinstance(item,dict): result.extend(diagnostic_gaps(item,path))
    return result


def validate_master_uri(uri):
    value=urlsplit(uri)
    if value.scheme!='http' or not value.hostname or value.username or value.password or value.query or value.fragment:
        raise ValueError('Only HTTP ROS master URI without credentials/query is supported')
    host=value.hostname
    if host=='localhost': host='127.0.0.1'
    ipaddress.ip_address(host)  # Avoid an unbounded DNS resolution in the observer.
    port=value.port or 80
    return 'http://'+('['+host+']' if ':' in host else host)+':'+str(port)+(value.path or '/')


class LimitedTransport(xmlrpc.client.Transport):
    accept_gzip_encoding = False

    def make_connection(self,host):
        chost,self._extra_headers,_=self.get_host_info(host)
        return http.client.HTTPConnection(chost,timeout=RPC_TIMEOUT)

    def parse_response(self,response):
        if response.getheader('Content-Encoding',''):
            raise ValueError('Compressed RPC response not accepted')
        parser,unmarshaller=self.getparser()
        used=0
        try:
            while True:
                part=response.read(min(65536,MAX_RESPONSE-used+1))
                if not part: break
                used+=len(part)
                if used>MAX_RESPONSE: raise ValueError('RPC response size limit')
                parser.feed(part)
            parser.close()
            return unmarshaller.close()
        finally:
            response.close()


class Records:
    """Only encoded bounded metadata is retained; no periodic disk writes."""
    def __init__(self,stop,budget=TOTAL_BYTES-RESERVE_BYTES):
        self.stop,self.budget=stop,budget
        self.lock=threading.Lock()
        self.lines={k:[] for k in FILES}
        self.bytes=0
        self.closed=False
        self.reason=None
        self.rejected=Counter()

    def append(self,kind,value):
        raw=encode(value)
        with self.lock:
            if self.closed:
                self.rejected['after_close']+=1
                return False
            limit=MAX_SDK if kind=='sdk' else MAX_ROWS
            reason=('item_byte_limit' if len(raw)>MAX_ITEM else
                    'record_count_limit:'+kind if len(self.lines[kind])>=limit else
                    'metadata_byte_limit' if self.bytes+len(raw)>self.budget else None)
            if reason:
                self.rejected[reason]+=1
                if self.reason is None: self.reason=reason
                self.stop.set()
                return False
            self.lines[kind].append(raw)
            self.bytes+=len(raw)
            return True

    def close(self):
        with self.lock:
            self.closed=True
            return {'record_bytes':self.bytes,'counts':{k:len(v) for k,v in self.lines.items()},
                    'stop_reason':self.reason,'rejected':dict(self.rejected)}


def decode_parameter(value):
    if not isinstance(value,dict) or type(value.get('schema')) is not int or value['schema']!=1 or value.get('encoding')!='json':
        raise ValueError('invalid_parameter_envelope')
    raw=value.get('payload_json')
    if not isinstance(raw,str) or len(raw.encode('utf8'))>MAX_ITEM:
        raise ValueError('invalid_or_oversize_payload_json')
    payload=json.loads(raw)
    if (not isinstance(payload,dict) or type(payload.get('schema')) is not int or payload['schema']!=1
            or type(payload.get('capacity')) is not int or payload['capacity']!=32
            or type(payload.get('geometry_generation')) is not int or payload['geometry_generation']<0
            or payload.get('diagnostic_only') is not True or payload.get('motion_authorized') is not False
            or not isinstance(payload.get('entries'),list) or len(payload['entries'])>32):
        raise ValueError('invalid_diagnostic_schema')
    for entry in payload['entries']:
        if (not isinstance(entry,dict) or type(entry.get('source_stamp_ns')) is not int or entry['source_stamp_ns']<=0
                or type(entry.get('geometry_generation')) is not int or entry['geometry_generation']!=payload['geometry_generation']
                or not isinstance(entry.get('evidence'),dict)
                or type(entry.get('read_completed_monotonic')) not in (int,float)
                or not math.isfinite(entry['read_completed_monotonic'])):
            raise ValueError('invalid_exact_source_entry')
    return payload


class RpcWorker:
    def __init__(self,uri,records,stop,deadline,proxy_factory=None):
        self.uri,self.records,self.stop,self.deadline=uri,records,stop,deadline
        self.proxy_factory=proxy_factory or (lambda:xmlrpc.client.ServerProxy(uri,transport=LimitedTransport(),allow_none=True))
        self.thread=threading.Thread(target=self.run,name='passive_camera_metadata_rpc',daemon=True)
        self.seen={}
        self.read_times={}
        self.errors=Counter()
        self.last_owner=None
        self.consecutive_failures=0

    def poll(self,proxy):
        start=time.monotonic()
        row={'started_monotonic':start,'started_wall_ns':time.time_ns(), 'parameter':PARAMETER,
             'receipt_clock_is_not_exposure':True}
        try:
            code,message,value=proxy.getParam('/passive_sdk_memory_observer',PARAMETER)
            row.update(original_rpc_code=code,original_rpc_message=message)
            if code!=1: raise ValueError('parameter_unavailable')
            raw=value.get('payload_json') if isinstance(value,dict) else None
            row['envelope_without_payload']={k:v for k,v in value.items() if k!='payload_json'} if isinstance(value,dict) else value
            if isinstance(raw,str): row.update(payload_json_sha256=digest(raw.encode()),payload_json_bytes=len(raw.encode()))
            payload=decode_parameter(value)
            row['payload_without_entries']={k:v for k,v in payload.items() if k!='entries'}
            owner=(payload.get('geometry_generation'),payload.get('geometry_activation_stamp_ns'))
            row['owner_changed_since_previous_poll']=self.last_owner is not None and owner!=self.last_owner
            self.last_owner=owner
            row['entry_references']=[]
            for entry in payload['entries']:
                key=(entry['geometry_generation'],entry.get('geometry_activation_stamp_ns'),entry['source_stamp_ns'])
                encoded=encode(entry); entry_hash=digest(encoded)
                row['entry_references'].append({'key':list(key),'entry_sha256':entry_hash})
                if key in self.seen:
                    if self.seen[key]!=entry_hash:
                        raise ValueError('same_owner_source_changed_evidence')
                    continue
                if not self.records.append('sdk',{'first_observed_monotonic':time.monotonic(),
                    'entry_sha256':entry_hash,'original_entry':entry}): break
                self.seen[key]=entry_hash
                self.read_times[key]=entry['read_completed_monotonic']
            row['schema_valid']=True
        except Exception as exc:
            error=type(exc).__name__+': '+str(exc)
            self.errors[error[:240]]+=1
            row.update(schema_valid=False,error=error[:512])
        row.update(finished_monotonic=time.monotonic(),finished_wall_ns=time.time_ns())
        row['elapsed_sec']=row['finished_monotonic']-start
        self.records.append('rpc',row)
        if row.get('schema_valid'):
            self.consecutive_failures=0
        else:
            self.consecutive_failures+=1
            if self.consecutive_failures==1:
                print('CAMERA_METADATA_ERROR: '+row.get('error','unknown'),flush=True)
            if self.consecutive_failures>=5:
                with self.records.lock:
                    if self.records.reason is None: self.records.reason='rpc_consecutive_failures'
                self.stop.set()

    def run(self):
        while not self.stop.is_set() and time.monotonic()<self.deadline:
            start=time.monotonic()
            proxy=None
            try:
                proxy=self.proxy_factory()
                self.poll(proxy)
            except Exception as exc:
                self.errors['worker:'+type(exc).__name__]+=1
                self.records.append('rpc',{'started_monotonic':start,'finished_monotonic':time.monotonic(),
                    'schema_valid':False,'error':'worker:'+repr(exc)[:512]})
            finally:
                if proxy is not None:
                    try:
                        # Local transport close, never remote XML-RPC "close".
                        proxy.__exit__(None,None,None)
                    except Exception: pass
            self.stop.wait(max(0,min(self.deadline-time.monotonic(),PERIOD-(time.monotonic()-start))))


def bind_processes(watch,proc=Path('/proc')):
    requested={}
    for value in watch:
        fields=value.split(':')
        if len(fields)!=2: raise ValueError('watch-pid must be PID:start_ticks')
        pid,ticks=map(int,fields)
        if pid<=0 or ticks<=0 or (pid in requested and requested[pid]!=ticks): raise ValueError('invalid/conflicting watch identity')
        requested[pid]=ticks
    selected={}
    for path in proc.iterdir():
        if not path.name.isdigit(): continue
        name=read_text(path/'comm',256)
        if isinstance(name,str) and name.startswith(('kswapd','kcompactd','khugepaged','vmtools','vmware-vmblock')):
            selected[int(path.name)]=None
    selected.update(requested)
    if len(selected)>MAX_PIDS: raise ValueError('too_many_bound_processes')
    result={};total_tasks=0
    for pid,expected in sorted(selected.items()):
        base=proc/str(pid);stat=parse_stat(read_text(base/'stat'))
        if 'start_ticks' not in stat or (expected is not None and stat['start_ticks']!=expected):
            if pid in requested: raise ValueError('watch_process_identity_mismatch:'+str(pid))
            result[str(pid)]={'binding_error':stat};continue
        tasks={}
        try:
            for task in sorted((base/'task').iterdir(),key=lambda p:int(p.name)):
                value=parse_stat(read_text(task/'stat'))
                if 'start_ticks' not in value: continue
                tasks[task.name]={'start_ticks':value['start_ticks'],'comm':read_text(task/'comm',256)}
        except OSError as exc:
            if pid in requested: raise ValueError('watch_tasks_unavailable:'+str(pid)) from exc
        total_tasks+=len(tasks)
        if total_tasks>MAX_TASKS: raise ValueError('too_many_bound_tasks')
        result[str(pid)]={'start_ticks':stat['start_ticks'],'comm':read_text(base/'comm',256),
                          'explicit_watch':pid in requested,'tasks':tasks}
    return result


class ProcSampler:
    def __init__(self,bound,proc=Path('/proc'),sysroot=Path('/sys')):
        self.bound,self.proc,self.sysroot=bound,proc,sysroot
        self.last_status=-math.inf

    def sample(self):
        start=time.monotonic();p=self.proc
        row={'started_monotonic':start,'started_wall_ns':time.time_ns(),'processes':{}}
        row['vmstat']=numeric_lines(read_text(p/'vmstat'),
            ('pswp','pgscan','pgsteal','allocstall','compact_','thp_','balloon_'),
            ('pgmajfault','nr_free_pages','nr_dirty','nr_writeback','nr_anon_transparent_hugepages'))
        row['meminfo']=selected_fields(read_text(p/'meminfo'),('MemTotal','MemAvailable','MemFree','Cached','SwapTotal','SwapFree',
            'SwapCached','AnonHugePages','AnonPages','Slab','SReclaimable','SUnreclaim','Dirty','Writeback'))
        row['pressure']={k:read_text(p/'pressure'/k) for k in ('cpu','memory','io')}
        row['diskstats']=read_text(p/'diskstats')
        rawcpu=read_text(p/'stat')
        row['cpu_stat']=rawcpu.splitlines()[0] if isinstance(rawcpu,str) and rawcpu else rawcpu
        thp=self.sysroot/'kernel/mm/transparent_hugepage/khugepaged'
        row['khugepaged']={k:read_text(thp/k) for k in ('full_scans','pages_collapsed')}
        status_due=start-self.last_status>=1
        for pid,identity in self.bound.items():
            if 'start_ticks' not in identity:
                row['processes'][pid]={'error':'unbound_process','binding':identity};continue
            base=p/pid;stat=parse_stat(read_text(base/'stat'))
            if stat.get('start_ticks')!=identity['start_ticks']:
                row['processes'][pid]={'identity_error':stat,'expected_start_ticks':identity['start_ticks']};continue
            observed={'stat':stat,'tasks':{}}
            if status_due:
                observed['status']=selected_fields(read_text(base/'status'),('Name','State','VmRSS','RssAnon','RssFile','RssShmem','VmSwap','Threads'))
                observed['status_sampled_monotonic']=time.monotonic()
            for tid,task_id in identity['tasks'].items():
                tpath=base/'task'/tid;tstat=parse_stat(read_text(tpath/'stat'))
                if tstat.get('start_ticks')!=task_id['start_ticks']:
                    observed['tasks'][tid]={'identity_error':tstat,'expected_start_ticks':task_id['start_ticks']};continue
                observed['tasks'][tid]={'stat':tstat,'schedstat_raw':read_text(tpath/'schedstat'), 'wchan_raw':read_text(tpath/'wchan',512)}
                endstat=parse_stat(read_text(tpath/'stat'))
                if endstat.get('start_ticks')!=task_id['start_ticks']:
                    observed['tasks'][tid]={'identity_error_after_read':endstat,'expected_start_ticks':task_id['start_ticks'],
                                            'mixed_interval_details_discarded':True}
            endstat=parse_stat(read_text(base/'stat'))
            if endstat.get('start_ticks')!=identity['start_ticks']:
                observed={'identity_error_after_read':endstat,'expected_start_ticks':identity['start_ticks'],
                          'mixed_interval_details_discarded':True}
            row['processes'][pid]=observed
        if status_due: self.last_status=start
        row.update(finished_monotonic=time.monotonic(),finished_wall_ns=time.time_ns())
        row['read_cost_sec']=row['finished_monotonic']-start
        row['quality_gaps']=diagnostic_gaps(row)
        return row


def prepare_output(output):
    original=Path(output);out=original.resolve()
    if not MOUNT.is_mount(): raise ValueError('SanStand is not mounted; no local fallback')
    if original.is_symlink() or EXTERNAL not in out.parents: raise ValueError('New output under external ros_log required')
    if out.exists(): raise FileExistsError(out)
    out.mkdir(parents=False)
    return out


def write_atomic(path,parts):
    temporary=path.with_suffix(path.suffix+'.tmp');h=hashlib.sha256();size=0
    with temporary.open('xb') as stream:
        for raw in parts:
            if stream.write(raw)!=len(raw): raise OSError('short write')
            h.update(raw);size+=len(raw)
        stream.flush();os.fsync(stream.fileno())
    temporary.replace(path)
    return {'sha256':h.hexdigest(),'bytes':size}


def self_usage():
    value=resource.getrusage(resource.RUSAGE_SELF)
    return {'monotonic':time.monotonic(),'user_cpu_sec':value.ru_utime,'system_cpu_sec':value.ru_stime,
            'lifetime_max_rss_KiB':value.ru_maxrss,'minor_faults':value.ru_minflt,'major_faults':value.ru_majflt,
            'voluntary_context_switches':value.ru_nvcsw,'involuntary_context_switches':value.ru_nivcsw,
            'current_proc_status':selected_fields(read_text('/proc/self/status'),('VmRSS','VmHWM','VmSize')),
            'maxrss_note':'ru_maxrss is a process-lifetime high-water value and can retain inherited startup/fork history; use current VmRSS for current resident memory, not ru_maxrss.'}


def run(args):
    if not math.isfinite(args.seconds) or not 0<args.seconds<=300: raise ValueError('seconds must be >0 and <=300')
    uri=validate_master_uri(args.master_uri)
    out=prepare_output(args.output)
    stop=threading.Event();records=Records(stop);worker=None;old_handlers={};reason='startup_error';errors=[]
    ready=None;capture_start=None;capture_end=None;hashes={};signals=[];usage_start=None;usage_end=None;costs=[];proc_gaps=Counter()
    source_sha=digest(Path(__file__).read_bytes())
    def interrupted(signum,_frame):
        if len(signals)<16: signals.append({'signal':signum,'monotonic':time.monotonic(),'wall_ns':time.time_ns()})
        stop.set()
    try:
        for sig in (signal.SIGINT,signal.SIGTERM): old_handlers[sig]=signal.signal(sig,interrupted)
        boot=read_text('/proc/sys/kernel/random/boot_id',256)
        if not isinstance(boot,str) or not boot: raise ValueError('boot_id_unavailable')
        bound=bind_processes(args.watch_pid)
        sched=read_text('/proc/sys/kernel/sched_schedstats',32)
        ready={'schema':1,'pid':os.getpid(),'boot_id':boot,'created_monotonic':time.monotonic(),'created_wall_ns':time.time_ns(),
            'duration_sec':args.seconds,'resource_period_sec':PERIOD,'rpc_period_sec':PERIOD,'rpc_socket_timeout_sec':RPC_TIMEOUT,
            'bound_processes':bound,'maximum_fixed_tasks':MAX_TASKS,'source_sha256':source_sha,
            'sched_schedstats':sched,'schedstat_zero_does_not_prove_no_delay':sched!='1',
            'master_uri':uri,'only_rpc_parameter':PARAMETER,'diagnostic_only':True,'motion_authorized':False,
            'sdk_device_queries':False,'hardware_commands_sent':False,'metadata_budget_bytes':TOTAL_BYTES,
            'sdk_entry_limit':MAX_SDK,'per_stream_row_limit':MAX_ROWS,
            'sampling_starts_after_ready_file_is_written':True}
        hashes['ready.json']=write_atomic(out/'ready.json',[encode(ready)])
        capture_start=time.monotonic();deadline=capture_start+args.seconds
        usage_start=self_usage()
        sampler=ProcSampler(bound);worker=RpcWorker(uri,records,stop,deadline);worker.thread.start()
        print(json.dumps({'ready':str(out/'ready.json'),'capture_start_monotonic':capture_start,'deadline_monotonic':deadline}),flush=True)
        reason='duration_elapsed';next_sample=capture_start;previous=None
        while not stop.is_set() and time.monotonic()<deadline:
            delay=min(next_sample,deadline)-time.monotonic()
            if delay>0: stop.wait(delay)
            if stop.is_set() or time.monotonic()>=deadline: break
            row=sampler.sample()
            costs.append(row['read_cost_sec'])
            proc_gaps.update(row.get('quality_gaps',[]))
            row['actual_start_interval_sec']=None if previous is None else row['started_monotonic']-previous
            if not records.append('resources',row): break
            previous=row['started_monotonic'];next_sample=previous+PERIOD
        capture_end=time.monotonic()
        if signals: reason='signal'
        elif records.reason: reason=records.reason
    except BaseException as exc:
        errors.append(type(exc).__name__+': '+str(exc)[:512]);reason='exception'
    finally:
        if capture_end is None: capture_end=time.monotonic()
        usage_end=self_usage()
        stop.set()
        if worker is not None: worker.thread.join(1.25)
        state=records.close()
        rpc_alive=bool(worker and worker.thread.is_alive())
        if rpc_alive: errors.append('RPC thread did not return within bounded join; late observations rejected')
        try:
            if not MOUNT.is_mount(): raise OSError('SanStand mount lost; no fallback')
            for kind,name in FILES.items(): hashes[name]=write_atomic(out/name,records.lines[kind])
            if signals and reason=='duration_elapsed': reason='signal_during_seal'
            in_window=sum(capture_start<=value<capture_end for value in list(worker.read_times.values())) if worker and capture_start is not None else 0
            outcome={'schema':1,'diagnostic_only':True,'motion_authorized':False,'configuration_changed':False,
                'hardware_commands_sent':False,'temperature_queried':False,'source_sha256':source_sha,
                'stop_reason':reason,'signals':signals,'errors':errors,'capture_start_monotonic':capture_start,
                'capture_end_monotonic':capture_end,'duration_requested_sec':args.seconds,
                'duration_observed_sec':None if capture_start is None else capture_end-capture_start,
                'record_state':state,'rpc_thread_alive_at_seal':rpc_alive,'rpc_errors':dict(worker.errors) if worker else {},
                'unique_sdk_entries':len(worker.seen) if worker else 0,'files':hashes,
                'sdk_evidence_available':bool(worker and worker.seen),
                'sdk_entries_completed_inside_capture_window':in_window,
                'observer_integrity_passed':bool(reason=='duration_elapsed' and not errors and not state['stop_reason']
                    and worker and in_window>0 and not worker.errors and not proc_gaps and state['counts']['resources']>0),
                'proc_quality_gaps':dict(proc_gaps),
                'monitor_self_usage':{'capture_start':usage_start,'capture_end_before_file_writes':usage_end,
                    'cpu_seconds_over_capture':None if usage_start is None else
                    usage_end['user_cpu_sec']+usage_end['system_cpu_sec']-usage_start['user_cpu_sec']-usage_start['system_cpu_sec'],
                    'proc_read_cost_mean_sec':sum(costs)/len(costs) if costs else None,
                    'proc_read_cost_max_sec':max(costs) if costs else None},
                'boot_id_at_end':read_text('/proc/sys/kernel/random/boot_id',256),
                'capture_closed':True,'problem_resolved':False,'physical_validation_passed':False,
                'limits':['SDK entries retain original JSON values and are deduplicated by owner/source; each RPC retains complete payload SHA, owner fields and ordered entry references, not repeated payload text.',
                    'SDK history covers successful reads, not guaranteed publication or calibrated exposure; entries read before or after the capture window remain labelled by original read timestamps and must be excluded from window statistics. An RPC begun inside the window may return after capture end.',
                    'Proc reads are not atomic across fields. Actual row start/end and read costs are saved; do not bridge missing rows or reuse timestamps across boots.',
                    'TIDs are enumerated only at startup and pinned by start_ticks. New threads are not silently included; disappeared/reused identities are recorded as unavailable.',
                    'schedstat delay can be disabled and wchan0 is ambiguous. Neither a zero counter nor absence of a field proves no stall.',
                    'The RPC uses a1s socket timeout and bounded response, not a hard real-time deadline under arbitrary OS suspension/trickling peer; the daemon worker has a bounded join and late records are rejected.',
                    'Global PSI/reclaim/balloon counters show association only; guest observations cannot identify host memory pressure or determine the unique cause.',
                    'The32MiB archive budget is a serialized-record limit, not a process RSS guarantee. Records are buffered and written after observation to reduce disk perturbation.']}
            total=sum(v['bytes'] for v in hashes.values());raw=encode(outcome)
            if total+len(raw)+4096>TOTAL_BYTES: raise OSError('final archive budget exceeded')
            hashes['outcome.json']=write_atomic(out/'outcome.json',[raw])
            sums=''.join(v['sha256']+'  '+name+'\n' for name,v in sorted(hashes.items())).encode()
            write_atomic(out/'SHA256SUMS',[sums])
            print(json.dumps({'outcome':str(out/'outcome.json'),'stop_reason':reason,'record_state':state,'errors':errors}),flush=True)
        except BaseException as exc:
            print('PARTIAL_CAPTURE_SEAL_ERROR: '+repr(exc),flush=True)
            return 2
        finally:
            for sig,handler in old_handlers.items(): signal.signal(sig,handler)
    return 0 if outcome.get('observer_integrity_passed') else 2


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',action='store_true')
    parser.add_argument('--output')
    parser.add_argument('--seconds',type=float,default=180)
    parser.add_argument('--watch-pid',action='append',default=[])
    parser.add_argument('--master-uri',default=os.environ.get('ROS_MASTER_URI','http://127.0.0.1:11311'))
    args=parser.parse_args()
    if not args.run:
        print('Prepared only; no sampling started. --run and a new external --output are required.')
        return 0
    if not args.output: parser.error('--output is required with --run')
    return run(args)


if __name__=='__main__':
    raise SystemExit(main())
