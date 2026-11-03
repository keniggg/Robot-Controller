"""Offline bounded A/B/A evidence summary; no live ROS or hardware access."""
import argparse
from collections import Counter
import hashlib,json,re
from pathlib import Path

def sha(b):return hashlib.sha256(b).hexdigest()
def encode(v):return (json.dumps(v,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
def distribution(v,scale=1.):
    v=sorted(float(x)*scale for x in v if x is not None)
    if not v:return {'count':0}
    def q(p):
        x=p*(len(v)-1);i=int(x);return v[i]+(v[min(i+1,len(v)-1)]-v[i])*(x-i)
    return dict(count=len(v),minimum=v[0],p50=q(.5),p95=q(.95),maximum=v[-1])
def delta_summary(values,mono_key,value_key,lo,hi,scale=1.):
    ordered=sorted(values,key=lambda r:r[mono_key]);inside=[];cross=[]
    for a,b in zip(ordered,ordered[1:]):
        aa,bb=a[mono_key],b[mono_key]
        if lo<=aa<hi and lo<=bb<hi:inside.append((b[value_key]-a[value_key])*scale)
        elif aa<hi and bb>=lo and (aa<lo<=bb or aa<hi<=bb):
            cross.append(dict(start_monotonic=aa,end_monotonic=bb,value=(b[value_key]-a[value_key])*scale,assigned_to_phase=False))
    return dict(fully_inside=distribution(inside),cross_boundary_excluded=cross)
def psi_parse(s):
    return {line.split()[0]:{k:float(v) for k,v in (t.split('=') for t in line.split()[1:])} for line in s.splitlines()}
def disk_delta(a,b,seconds):
    result={};aa={r[2]:r for r in a};bb={r[2]:r for r in b}
    for name in sorted(set(aa)&set(bb)):
        x,y=aa[name],bb[name];z={}
        for key,i in [('reads_completed',3),('sectors_read_512_bytes',5),('read_ms',6),('writes_completed',7),('sectors_written_512_bytes',9),('write_ms',10),('io_ms',12),('weighted_io_ms',13)]:z[key]=int(y[i])-int(x[i])
        z['read_MiB']=z['sectors_read_512_bytes']*512/2**20;z['write_MiB']=z['sectors_written_512_bytes']*512/2**20
        z['read_MiB_per_sec']=z['read_MiB']/seconds;z['write_MiB_per_sec']=z['write_MiB']/seconds
        z['io_busy_percent']=100*z['io_ms']/(seconds*1000);z['inflight_first_last']=[int(x[11]),int(y[11])]
        result[name]=z
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('--capture',type=Path,required=True);args=p.parse_args();capture=args.capture
    raw=(capture/'observations.json').read_bytes();frozen=(capture/'SHA256SUMS').read_text().split()[0]
    if sha(raw)!=frozen:raise ValueError('observations_hash_mismatch')
    d=json.loads(raw);report=dict(diagnostic_only=True,motion_authorized=False,configuration_changed=False,
        observations_sha256=sha(raw),script_sha256=sha(Path(__file__).read_bytes()),error=d['error'],record_dir=d['record_dir'],recorder_returncode=d['recorder_returncode'],phases=[])
    for phase in d['phases']:
        lo,hi=phase['start'],phase['end'];read=[r for r in d['sdk_reads'] if lo<=r['monotonic']<hi];ss=[r for r in d['samples'] if lo<=r['monotonic']<hi]
        wait=[r for r in read if lo<=r['host_wait']['enter']['monotonic_sec']<=r['host_wait']['exit']['monotonic_sec']<hi]
        row=dict(name=phase['name'],monotonic_start=lo,monotonic_end=hi,duration_sec=hi-lo,
            sdk_read_count=len(read),sdk_result_counts=dict(Counter(r['result'] for r in read)),
            sdk_read_completed_first_last=[read[0]['monotonic'],read[-1]['monotonic']] if read else None,
            sdk_wait_ms=distribution([r['host_wait']['elapsed_sec'] for r in wait],1000),
            sdk_wait_thread_cpu_ms=distribution([r['host_wait']['thread_cpu_elapsed_sec'] for r in wait],1000),
            wait_spanning_boundary_excluded=len(read)-len(wait),
            sdk_source_interval_ms=delta_summary(d['sdk_reads'],'monotonic','source_ns',lo,hi,1e-6),
            sdk_read_completed_interval_ms=delta_summary(d['sdk_reads'],'monotonic','monotonic',lo,hi,1000),
            sdk_raw_streams={},feedback={})
        for stream in ('color','depth'):
            ff=[]
            for r in d['sdk_reads']:
                t=r.get('raw_frames',{}).get(stream) or {}
                if t.get('present') is True:ff.append(dict(monotonic=r['monotonic'],device_timestamp_ms=t['device_timestamp_ms'],frame_number=t['frame_number'],actual_fps=t.get('metadata_raw',{}).get('actual_fps')))
            row['sdk_raw_streams'][stream]=dict(device_interval_ms=delta_summary(ff,'monotonic','device_timestamp_ms',lo,hi),
                frame_number_step=delta_summary(ff,'monotonic','frame_number',lo,hi),actual_fps_raw_counts=dict(Counter(str(r['actual_fps']) for r in ff if lo<=r['monotonic']<hi)))
        for topic,values in d['feedback'].items():
            ff=[r for r in values if lo<=r['monotonic']<hi]
            row['feedback'][topic]=dict(count=len(ff),caller_counts=dict(Counter(r['caller'] for r in ff)),
                source_interval_ms=delta_summary(values,'monotonic','source_ns',lo,hi,1e-6),
                receipt_interval_ms=delta_summary(values,'monotonic','monotonic',lo,hi,1000),
                source_age_at_receipt_ms=distribution([(r['receipt_ns']-r['source_ns'])/1e6 for r in ff]),
                scope='Phase assignment uses receipt monotonic times; source gap is original ROS header delta, excluding intervals whose receipt endpoints span a boundary.')
        if len(ss)>1:
            first,last=ss[0],ss[-1];span=last['monotonic']-first['monotonic']
            psi={}
            for kind in ('cpu','memory','io'):
                a,b=psi_parse(first['psi_'+kind]),psi_parse(last['psi_'+kind]);psi[kind]={}
                for key in a:
                    delta=b[key]['total']-a[key]['total'];psi[kind][key]=dict(total_delta_microseconds=delta,elapsed_fraction_percent=100*delta/(span*1e6),first_avg10=a[key]['avg10'],last_avg10=b[key]['avg10'])
            row['system']=dict(sample_count=len(ss),first_sample_monotonic=first['monotonic'],last_sample_monotonic=last['monotonic'],sample_span_sec=span,
                phase_start_unsampled_sec=first['monotonic']-lo,phase_end_unsampled_sec=hi-last['monotonic'],psi=psi,
                counter_deltas={k:last['vmstat'][k]-v for k,v in first['vmstat'].items() if not k.startswith('nr_')},
                vm_gauges={k:dict(first=first['vmstat'][k],last=last['vmstat'][k],minimum=min(v['vmstat'][k] for v in ss),maximum=max(v['vmstat'][k] for v in ss)) for k in first['vmstat'] if k.startswith('nr_')},
                meminfo_first=first['meminfo'],meminfo_last=last['meminfo'],diskstats_delta=disk_delta(first['diskstats'],last['diskstats'],span),
                metadata_read_ms=distribution([r['metadata_read_seconds'] for r in ss],1000),metadata_error_count=sum('camera_metadata_error' in r for r in ss))
        report['phases'].append(row)
    names=[r['name'] for r in report['phases']]
    report['aba_complete']=all(x in names for x in ['A1_without_extra_keyframe_recording','B_keyframe_recording','A2_without_extra_keyframe_recording']) and d['error'] is None
    report['sdk_history_before_first_phase_excluded']=sum(r['monotonic']<d['phases'][0]['start'] for r in d['sdk_reads'])
    report['sdk_history_after_last_phase_excluded']=sum(r['monotonic']>=d['phases'][-1]['end'] for r in d['sdk_reads'])
    report['sanstand_mapping_observed_readonly']=dict(source='/dev/sdb2',target='/media/zhuyupei/SanStand',filesystem='exfat',method='findmnt -T user external storage; no mount changes')
    report['limits']=['Only intervals wholly within phase monotonic bounds enter per-phase interval distributions; crossings are separately retained.',
        'System counter deltas cover the first-to-last sample span, not unmeasured exact phase edges. Proc reads occur after metadata RPC in each observation.',
        'Disk devices and their partitions must not be summed together; 512-byte sectors are used for diskstats regardless of filesystem block size.',
        'The B intervention includes recorder subscriptions, message copying, compression, memory and external writes; it is not a pure disk experiment.',
        'A2 may retain dirty cache/writeback and is not guaranteed to equal A1.',
        'A short clean A interval alone cannot prove recording caused an earlier stall or that the root problem is solved.',
        'SDK read history is a bounded diagnostic history; no old pre-phase entries are counted in A1.',
        'No missing B/A2 observation is imputed.']
    if not report['aba_complete']:report['conclusion']='A/B/A incomplete: only actual observed phases can be summarized. No recording-on contrast or recording causal conclusion is available.'
    out=capture/'analysis';out.mkdir(exist_ok=False);b=encode(report);(out/'report.json').write_bytes(b);(out/'SHA256SUMS').write_text(sha(b)+'  report.json\n')
    print(json.dumps(dict(output=str(out),sha256=sha(b),aba_complete=report['aba_complete'],phases=[dict(name=r['name'],sdk_wait_ms=r['sdk_wait_ms'],sdk_interval_ms=r['sdk_source_interval_ms']['fully_inside'],feedback={k:dict(callers=v['caller_counts'],interval_ms=v['source_interval_ms']['fully_inside']) for k,v in r['feedback'].items()},system=r.get('system')) for r in report['phases']]),ensure_ascii=False))

if __name__=='__main__':main()
