"""Offline summaries of sealed passive disk/raw-RGBD load observations.

No ROS imports, hardware access or pixel decoding. No missing datum implies health.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
MAX_OBSERVATIONS=32*1024**2

def finite(v):return isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)
def digest(b):return hashlib.sha256(b).hexdigest()
def encoded(v):return (json.dumps(v,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
def stats(values):
    v=sorted(x for x in values if finite(x));bad=len(values)-len(v)
    if not v:return dict(available=False,count=0,invalid_values=bad,p50=None,p95=None,maximum=None)
    def q(f):
        x=f*(len(v)-1);i=int(x);return v[i]+(v[min(i+1,len(v)-1)]-v[i])*(x-i)
    return dict(available=True,count=len(v),invalid_values=bad,minimum=v[0],p50=q(.5),p95=q(.95),maximum=v[-1])
def ordered(rows,key='monotonic'):
    return sorted((r for r in rows if isinstance(r,dict) and finite(r.get(key))),key=lambda r:r[key])
def interval_stats(rows,lo,hi,value='source_ns',scale=1e-6,timekey='monotonic'):
    rr=ordered(rows,timekey);inside=[];cross=[];bad=[]
    for a,b in zip(rr,rr[1:]):
        x,y=a[timekey],b[timekey]
        within=lo<=x<hi and lo<=y<hi
        touching=(x<lo<=y) or (x<hi<=y)
        if not within and not touching:continue
        av,bv=a.get(value),b.get(value)
        if not finite(av) or not finite(bv):
            bad.append(dict(start_monotonic=x,end_monotonic=y,reason='missing_or_nonfinite_value'));continue
        dt=(bv-av)*scale
        if dt<0:
            bad.append(dict(start_monotonic=x,end_monotonic=y,reason='negative_value_delta',value=dt));continue
        if within:inside.append(dt)
        else:cross.append(dict(start_monotonic=x,end_monotonic=y,value=dt,assigned_to_phase=False))
    return dict(fully_inside=stats(inside),cross_boundary_excluded=cross,invalid_intervals=bad,
                invalid_monotonic_rows=len(rows)-len(rr),missing_values_not_bridged=True)
def psi(s):
    if not isinstance(s,str):raise ValueError('missing_psi')
    d={line.split()[0]:{k:float(v) for k,v in (t.split('=',1) for t in line.split()[1:])} for line in s.splitlines()}
    if not d or any(not finite(v['total']) for v in d.values()):raise ValueError('invalid_psi')
    return d

def system_summary(rows,lo,hi):
    ss=[r for r in ordered(rows) if lo<=r['monotonic']<hi]
    out=dict(available=len(ss)>=2,sample_count=len(ss),errors=[],metadata_read_ms=stats([r.get('metadata_read_seconds',float('nan'))*1000 if finite(r.get('metadata_read_seconds')) else None for r in ss]),
             camera_metadata_errors=[r.get('camera_metadata_error') for r in ss if 'camera_metadata_error' in r])
    if len(ss)<2:return out
    a,b=ss[0],ss[-1];dt=b['monotonic']-a['monotonic']
    if dt<=0:out.update(available=False,errors=['nonpositive_sample_span']);return out
    out.update(first_sample_monotonic=a['monotonic'],last_sample_monotonic=b['monotonic'],sample_span_sec=dt,
               start_unsampled_sec=a['monotonic']-lo,end_unsampled_sec=hi-b['monotonic'],psi={},vmstat={},diskstats={},
               meminfo_first=a.get('meminfo'),meminfo_last=b.get('meminfo'))
    for kind in ('cpu','memory','io'):
        try:
            x,y=psi(a.get('psi_'+kind)),psi(b.get('psi_'+kind));v={}
            for k in set(x)&set(y):
                delta=y[k]['total']-x[k]['total']
                if delta<0:raise ValueError('counter_decreased')
                v[k]=dict(delta_microseconds=delta,elapsed_fraction_percent=delta/dt/1e4)
            out['psi'][kind]=v
        except (ValueError,KeyError,TypeError) as exc:out['errors'].append('psi_'+kind+':'+str(exc))
    x,y=a.get('vmstat') or {},b.get('vmstat') or {}
    for key in set(x)|set(y):
        if not finite(x.get(key)) or not finite(y.get(key)):out['errors'].append('vmstat_missing:'+key);continue
        if key.startswith('nr_'):
            vv=[s.get('vmstat',{}).get(key) for s in ss];out['vmstat'][key]=dict(kind='gauge',first=x[key],last=y[key],distribution=stats(vv));continue
        delta=y[key]-x[key]
        if delta<0:out['errors'].append('vmstat_counter_decreased:'+key);continue
        out['vmstat'][key]=dict(kind='counter',delta=delta)
    if not x or not y:out['errors'].append('vmstat_missing')
    def disks(v):
        result={}
        for r in v or []:
            if not isinstance(r,list) or len(r)<14:out['errors'].append('diskstats_invalid_row');continue
            result[r[2]]=r
        return result
    x,y=disks(a.get('diskstats')),disks(b.get('diskstats'))
    for name in set(x)|set(y):
        if name not in x or name not in y:out['errors'].append('diskstats_missing_endpoint:'+name);continue
        try:
            z={k:int(y[name][i])-int(x[name][i]) for k,i in [('read_operations',3),('read_sectors_512B',5),('read_ms',6),('write_operations',7),('write_sectors_512B',9),('write_ms',10),('busy_ms',12),('weighted_io_ms',13)]}
            if any(v<0 for v in z.values()):raise ValueError('counter_decreased')
            z.update(read_MiB=z['read_sectors_512B']/2048,write_MiB=z['write_sectors_512B']/2048,
                read_MiB_s=z['read_sectors_512B']/2048/dt,write_MiB_s=z['write_sectors_512B']/2048/dt,busy_percent=z['busy_ms']/dt/10)
            out['diskstats'][name]=z
        except (ValueError,TypeError) as exc:out['errors'].append('diskstats_'+name+':'+str(exc))
    if not x or not y:out['errors'].append('diskstats_missing')
    return out

def classify_schema(d):
    if d.get('experiment')=='raw_exact_rgbd_recording_load_ABA' and 'recorder_outcome' in d:return 'raw_rgbd_recording'
    if 'write_timings' in d and 'bytes_written' in d:return 'buffered_disk_write'
    raise ValueError('unsupported_observation_schema')

def analyze(d):
    schema=classify_schema(d);sdk=d.get('sdk_reads') or [];samples=d.get('samples') or [];phases=d.get('phases') or []
    out=dict(schema=1,input_schema=schema,diagnostic_only=True,hardware_or_ros_calls=False,phase_summaries=[],anomalies=[],
        arm_powered_off=True,feedback_interpretation='Count only; missing arm feedback is expected while powered off and is not classified as a stall.',
        original_errors=d.get('errors',[]) if schema=='raw_rgbd_recording' else ([d['error']] if d.get('error') else []))
    for i,p in enumerate(phases):
        if not isinstance(p,dict) or not finite(p.get('start')) or not finite(p.get('end')) or p['end']<=p['start']:
            out['anomalies'].append(dict(kind='invalid_phase',index=i));continue
        lo,hi=p['start'],p['end'];read=[r for r in ordered(sdk) if lo<=r['monotonic']<hi]
        row=dict(name=p.get('name'),start_monotonic=lo,end_monotonic=hi,duration_sec=hi-lo,
                 aggregate_overlaps_other_phases=p.get('name')=='B_owned_process_lifetime',sdk_read_count=len(read),
                 sdk_available=bool(read),sdk_result_counts=dict(Counter(str(r.get('result')) for r in read)),
                 sdk_read_interval_ms=interval_stats(sdk,lo,hi,'monotonic',1000),
                 sdk_source_interval_ms=interval_stats(sdk,lo,hi),sdk_wait_issues=[],native={},feedback={},
                 sdk_first_last_completed_monotonic=[read[0]['monotonic'],read[-1]['monotonic']] if read else None)
        waits=[];waitcpu=[];complete=[];stages={};owners=Counter()
        for r in read:
            hw=r.get('host_wait') or {};enter=(hw.get('enter') or {}).get('monotonic_sec');exit=(hw.get('exit') or {}).get('monotonic_sec')
            if not finite(enter) or not finite(exit) or enter>exit or exit>r['monotonic']:
                row['sdk_wait_issues'].append(dict(source_ns=r.get('source_ns'),reason='missing_or_invalid_wait_bounds'));continue
            if not lo<=enter<=exit<hi:
                row['sdk_wait_issues'].append(dict(source_ns=r.get('source_ns'),reason='cross_boundary_wait_excluded',enter=enter,exit=exit));continue
            elapsed=hw.get('elapsed_sec')
            if not finite(elapsed) or elapsed<0:
                row['sdk_wait_issues'].append(dict(source_ns=r.get('source_ns'),reason='missing_or_invalid_wait_elapsed'));continue
            if abs(elapsed-(exit-enter))>1e-6:row['sdk_wait_issues'].append(dict(source_ns=r.get('source_ns'),reason='reported_elapsed_differs_from_bounds'))
            waits.append(elapsed*1000);waitcpu.append(hw['thread_cpu_elapsed_sec']*1000 if finite(hw.get('thread_cpu_elapsed_sec')) else None)
            complete.append((r['monotonic']-enter)*1000)
        for r in read:
            if not isinstance(r.get('stages_sec'),dict):row['sdk_wait_issues'].append(dict(source_ns=r.get('source_ns'),reason='stage_timings_missing'))
            else:
                for k,v in r['stages_sec'].items():stages.setdefault(k,[]).append(v*1000 if finite(v) and v>=0 else None)
            if 'geometry_generation' in r:owners[str((r.get('geometry_generation'),r.get('geometry_activation_stamp_ns')))]+=1
        row.update(sdk_wait_ms=stats(waits),sdk_wait_cpu_ms=stats(waitcpu),wait_entry_to_read_completed_ms=stats(complete),
            stage_ms={k:stats(v) for k,v in stages.items()},geometry_owner_counts=dict(owners),
            regional_reset_counts=dict(Counter(str(r.get('regional_reset_reason')) for r in read if r.get('regional_reset_reason'))))
        if len(owners)>1:row['sdk_wait_issues'].append(dict(reason='geometry_owner_changed_during_phase'))
        for stream in ('color','depth'):
            ff=[];missing=0;fps=[]
            for r in sdk:
                if not isinstance(r,dict) or not finite(r.get('monotonic')):continue
                native=(r.get('raw_frames') or {}).get(stream) or {}
                present=native.get('present') is True
                if lo<=r['monotonic']<hi and not present:missing+=1
                metadata=native.get('metadata_raw') or {}
                ff.append(dict(monotonic=r['monotonic'],device_timestamp_ms=native.get('device_timestamp_ms') if present else None,
                    native_frame_timestamp_us=metadata.get('frame_timestamp') if present else None,frame_number=native.get('frame_number') if present else None))
                if lo<=r['monotonic']<hi:fps.append(metadata.get('actual_fps')/1000 if finite(metadata.get('actual_fps')) else None)
            row['native'][stream]=dict(missing_frames=missing,device_interval_ms=interval_stats(ff,lo,hi,'device_timestamp_ms',1),
                native_frame_timestamp_interval_ms=interval_stats(ff,lo,hi,'native_frame_timestamp_us',.001),
                frame_number_step=interval_stats(ff,lo,hi,'frame_number',1),actual_fps=stats(fps),
                note='Missing native rows remain barriers; no interval is bridged over missing frame metadata. Counter rollover is reported, never silently corrected.')
        for name,values in (d.get('feedback') or {}).items():
            vv=[r for r in ordered(values) if lo<=r['monotonic']<hi]
            row['feedback'][name]=dict(count=len(vv),callers=dict(Counter(str(r.get('caller')) for r in vv)),
                source_interval_ms=interval_stats(values,lo,hi),absence_is_arm_poweroff_not_stall=True)
        row['system']=system_summary(samples,lo,hi)
        if schema=='buffered_disk_write':
            contained=[];cross=[];invalid=[]
            for event in d.get('write_timings') or []:
                a,t=event.get('monotonic'),event.get('seconds')
                if not finite(a) or not finite(t) or t<0:invalid.append('invalid_write_timing');continue
                b=a+t
                if lo<=a<=b<hi:contained.append(event)
                elif a<hi and b>=lo and (a<lo<=b or a<hi<=b):cross.append(event)
            normal=[e for e in contained if e.get('phase')!='final_fsync'];bb=sum(e.get('bytes',0) for e in normal if finite(e.get('bytes')))
            row['actual_write']=dict(events=len(normal),bytes=bb,MiB=bb/2**20,MiB_s_over_phase=bb/2**20/(hi-lo),
                write_ms=stats([e['seconds']*1000 for e in normal]),fsync_ms=stats([e['seconds']*1000 for e in contained if e.get('phase')=='final_fsync']),
                cross_boundary_excluded=cross,invalid_events=invalid)
        out['phase_summaries'].append(row)
    if phases and out['phase_summaries']:
        lo=min(r['start_monotonic'] for r in out['phase_summaries']);hi=max(r['end_monotonic'] for r in out['phase_summaries'])
        out['sdk_history_outside_all_phases']=dict(before=sum(finite(r.get('monotonic')) and r['monotonic']<lo for r in sdk),
            after=sum(finite(r.get('monotonic')) and r['monotonic']>=hi for r in sdk))
    else:out['anomalies'].append(dict(kind='no_valid_phases'))
    if schema=='raw_rgbd_recording':
        outcome=d.get('recorder_outcome') or {};pairs=outcome.get('pairs') or [];issues=[];valid=[]
        start,end=outcome.get('capture_start_monotonic'),outcome.get('capture_end_monotonic');duration=end-start if finite(start) and finite(end) and end>start else None
        seen=set()
        if duration is None:issues.append(dict(reason='invalid_capture_window'))
        requested=outcome.get('duration_requested_sec')
        if finite(requested) and (duration is None or duration<requested):issues.append(dict(reason='short_capture_window'))
        for i,pair in enumerate(pairs):
            images=pair.get('images') or {};source=pair.get('source_ns');cc,dd=images.get('color') or {},images.get('depth') or {}
            receipts=[cc.get('receipt_monotonic'),dd.get('receipt_monotonic')];written=pair.get('write_completed_monotonic')
            if not finite(source) or source<=0 or cc.get('source_ns')!=source or dd.get('source_ns')!=source:
                issues.append(dict(index=i,reason='pair_source_mismatch'));continue
            if not all(finite(x) for x in receipts) or not finite(written) or written<max(receipts):issues.append(dict(index=i,reason='invalid_pair_timing'));continue
            if source in seen:issues.append(dict(index=i,reason='duplicate_pair_source'));continue
            seen.add(source)
            if duration is None or not all(start<=x<end for x in receipts):issues.append(dict(index=i,reason='pair_receipt_outside_capture'));continue
            valid.append(dict(source_ns=source,monotonic=max(receipts),write_completed=written,
                latest_receipt_to_write_end_ms=(written-max(receipts))*1000,earliest_receipt_to_write_end_ms=(written-min(receipts))*1000))
        if outcome.get('pair_count')!=len(pairs):issues.append(dict(reason='reported_pair_count_mismatch'))
        if not pairs:issues.append(dict(reason='no_recorded_pairs'))
        good=(outcome.get('capture_valid') is True and outcome.get('bag_closed') is True and not outcome.get('errors') and
              d.get('recorder_returncode')==0 and not d.get('owned_recorder_still_running') and not issues and duration is not None)
        bagbytes=outcome.get('bag_bytes');pixelbytes=outcome.get('raw_pixel_bytes')
        out['recorder']=dict(valid_artifact=good,reported_capture_valid=outcome.get('capture_valid'),pair_count=len(pairs),validated_pair_count=len(valid),
            errors=outcome.get('errors'),pair_issues=issues,receive_counts=(outcome.get('receive') or {}).get('counts'),capture_duration_sec=duration,
            stop_reason=outcome.get('stop_reason'),bag_bytes=bagbytes,raw_pixel_bytes=pixelbytes,
            compressed_bag_MiB_s=(bagbytes/2**20/duration if finite(bagbytes) and duration else None),
            input_pixels_MiB_s=(pixelbytes/2**20/duration if finite(pixelbytes) and duration else None),
            completion_tail_sec=(outcome['completed_monotonic']-end if finite(outcome.get('completed_monotonic')) and finite(end) else None),
            latest_receipt_to_write_end_ms=stats([r['latest_receipt_to_write_end_ms'] for r in valid]),
            earliest_receipt_to_write_end_ms=stats([r['earliest_receipt_to_write_end_ms'] for r in valid]),
            pair_timing_note='No write-start timestamp was recorded: latency includes queue, serialization, compression and write; it is not isolated disk or per-pair CPU duration.',phase_pairs={})
        for phase in out['phase_summaries']:
            lo,hi=phase['start_monotonic'],phase['end_monotonic']
            out['recorder']['phase_pairs'][phase['name']]=dict(received_complete_pairs=sum(lo<=r['monotonic']<hi for r in valid),
                write_completed_pairs=sum(lo<=r['write_completed']<hi for r in valid),source_interval_ms=interval_stats(valid,lo,hi),
                write_completion_interval_ms=interval_stats(valid,lo,hi,'write_completed',1000,timekey='write_completed'))
    else:
        counted=sum(e.get('bytes',0) for e in d.get('write_timings',[]) if finite(e.get('bytes')) and e.get('bytes')>0)
        out['actual_bytes_written_reported']=d.get('bytes_written')
        out['actual_bytes_written_counted']=counted
        out['write_accounting_valid']=finite(d.get('bytes_written')) and d['bytes_written']==counted and counted>0
        if not out['write_accounting_valid']:out['anomalies'].append(dict(kind='missing_or_inconsistent_write_accounting'))
    names=[r['name'] for r in out['phase_summaries']];hascontrast=any(str(n).startswith('A1_') for n in names) and any(str(n).startswith('A2_') for n in names) and ('B_raw_rgbd_recording' in names if schema=='raw_rgbd_recording' else 'B_bounded_buffered_disk_write' in names)
    out['complete_phase_contrast']=hascontrast and not out['original_errors'] and (out.get('recorder',{}).get('valid_artifact',False) if schema=='raw_rgbd_recording' else out['write_accounting_valid'])
    out['sdk_evidence_available_in_all_main_phases']=all(r['sdk_available'] and r['sdk_wait_ms']['available'] for r in out['phase_summaries'] if str(r['name']).startswith(('A1_','A2_')) or r['name'] in ('B_raw_rgbd_recording','B_bounded_buffered_disk_write')) and any(str(r['name']).startswith(('A1_','A2_')) or r['name'] in ('B_raw_rgbd_recording','B_bounded_buffered_disk_write') for r in out['phase_summaries'])
    out['limits']=['All phase metrics use monotonic bounds; endpoint-crossing intervals are retained separately, not attributed to A or B. B_owned_process_lifetime overlaps subphases and must not be summed.',
        'No data or invalid data is unavailable, never a healthy zero. A complete phase schedule is not proof of complete SDK diagnostic history.',
        'System deltas cover actual first/last sample spans; proc reads follow the metadata RPC, so submillisecond phase-edge attribution is unavailable.',
        'Device and partition diskstats must not be summed; sector counters use512-byte units. Counter decreases/missing fields remain explicit.',
        'Arm is powered off. Feedback absence is not a stall and no powered motion behavior is validated.',
        'Pure disk B adds bounded buffered file writes; raw B additionally changes subscriptions, copying, LZ4 compression and memory, and may exceed historical5.5MiB/s recording.',
        'A2 may include writeback. Short clean windows cannot prove the historical failure is resolved or uniquely attribute USB/VM/SDK causality.',
        'SDK history denotes completed reads, not guaranteed ROS publications; native frame counters are distinct from ROS source stamps.',
        'Pair completion latency is not isolated write duration. No image data, robot geometry, selection or controller state is changed.']
    return out

def run(capture,output=None):
    capture=Path(capture).absolute()
    if capture.is_symlink() or capture.resolve()!=capture or EXT not in capture.parents:raise ValueError('canonical_external_capture_required')
    path=capture/'observations.json'
    if path.is_symlink() or path.stat().st_size>MAX_OBSERVATIONS:raise ValueError('invalid_observations_path_or_size')
    hashes={}
    for line in (capture/'SHA256SUMS').read_text().splitlines():
        h,name=line.split(None,1);name=name.strip()
        if name in hashes:raise ValueError('duplicate_hash_entry')
        hashes[name]=h
    raw=path.read_bytes()
    if hashes.get('observations.json')!=digest(raw):raise ValueError('observations_frozen_hash_mismatch')
    report=analyze(json.loads(raw));report.update(observations_path=str(path),observations_sha256=digest(raw),source_sha256=digest(Path(__file__).read_bytes()),pixel_or_bag_data_read=False)
    output=Path(output) if output else capture/'analysis'
    if output.absolute().resolve()!=output.absolute() or EXT not in output.absolute().parents:raise ValueError('canonical_external_output_required')
    output.mkdir(exist_ok=False);b=encoded(report);(output/'report.json').write_bytes(b);(output/'SHA256SUMS').write_text(digest(b)+'  report.json\n')
    short=dict(output=str(output),sha256=digest(b),schema=report['input_schema'],complete=report['complete_phase_contrast'],sdk_available=report['sdk_evidence_available_in_all_main_phases'],
        phases=[dict(name=r['name'],duration_sec=r['duration_sec'],sdk_count=r['sdk_read_count'],wait_ms=r['sdk_wait_ms'],source_ms=r['sdk_source_interval_ms']['fully_inside'],write=r.get('actual_write')) for r in report['phase_summaries']],recorder=report.get('recorder'))
    print(json.dumps(short,ensure_ascii=False))
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--capture',required=True,type=Path);p.add_argument('--output',type=Path);a=p.parse_args();run(a.capture,a.output)
