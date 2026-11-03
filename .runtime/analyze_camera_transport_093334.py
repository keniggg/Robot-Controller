#!/usr/bin/python3
"""Offline analysis only: reads recorded data and writes one analysis JSON."""
import json, hashlib, statistics
from pathlib import Path
P=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/camera_transport_20260929T093334Z')
d=json.loads((P/'raw.json').read_text())
E=d['events']; R=d['resources']; tasks=[e for e in E if e['kind']=='task']
start=next(e['wall_ns'] for e in tasks if e.get('active'))
end=next(e['wall_ns'] for e in tasks if e['wall_ns']>start and not e.get('active'))

def stream(kind,a,b,key='wall_ns'):
    es=[e for e in E if e['kind']==kind]
    chosen=[e for e in es if a<=e[key]<=b]
    pairs=[(x,y) for x,y in zip(es,es[1:]) if a<=y[key]<=b]
    gaps=[(y['source_ns']-x['source_ns'])/1e9 for x,y in pairs]
    receipt=[(y['monotonic_ns']-x['monotonic_ns'])/1e9 for x,y in pairs]
    ages=[(e['wall_ns']-e['source_ns'])/1e9 for e in chosen]
    return dict(samples=len(chosen),max_source_gap_sec=max(gaps,default=None),source_gaps_over_800ms=sum(v>.8 for v in gaps),source_gaps_over_200ms=sum(v>.2 for v in gaps),max_callback_monotonic_gap_sec=max(receipt,default=None),max_callback_source_age_sec=max(ages,default=None),callback_source_ages_over_800ms=sum(v>.8 for v in ages),negative_callback_source_ages=sum(v<0 for v in ages),observed_sequence_skips=sum(max(0,y['seq']-x['seq']-1) for x,y in pairs))

def pressure(raw):
    ans={}
    for line in raw.splitlines():
        f=line.split(); ans[f[0]]={k:float(v) for k,v in (i.split('=') for i in f[1:])}
    return ans

def disks(raw):
    ans={}
    for line in raw.splitlines():
        v=line.split()
        if v[2] in ('sda','sdb'):
            nums=list(map(int,v[3:])); ans[v[2]]={'read_sectors':nums[2],'write_sectors':nums[6],'io_ms':nums[9],'weighted_io_ms':nums[10]}
    return ans

def resource(a,b):
    rs=[r for r in R if a<=r['wall_ns']<=b]
    if not rs: return {'samples':0}
    duration=(rs[-1]['monotonic_ns']-rs[0]['monotonic_ns'])/1e9
    out={'samples':len(rs),'covered_sec':duration,'max_sample_monotonic_gap_sec':max(((y['monotonic_ns']-x['monotonic_ns'])/1e9 for x,y in zip(rs,rs[1:])),default=None),'pressure':{},'disk':{}}
    for k in ('cpu','io','memory'):
        ps=[pressure(r['values']['/proc/pressure/'+k]) for r in rs]
        out['pressure'][k]={}
        for kind in ('some','full'):
            us=ps[-1][kind]['total']-ps[0][kind]['total']
            out['pressure'][k][kind]={'max_avg10_percent':max(p[kind]['avg10'] for p in ps),'observed_interval_stall_percent':us/1e6/duration*100 if duration else None}
    for disk in ('sda','sdb'):
        first=disks(rs[0]['values']['/proc/diskstats']).get(disk)
        last=disks(rs[-1]['values']['/proc/diskstats']).get(disk)
        if first and last:
            out['disk'][disk]={'read_MB':(last['read_sectors']-first['read_sectors'])*512/1e6,'write_MB':(last['write_sectors']-first['write_sectors'])*512/1e6,'busy_percent':(last['io_ms']-first['io_ms'])/1000/duration*100 if duration else None}
    cpus=[list(map(int,r['values']['/proc/stat'].split()[1:])) for r in rs]
    delta=[b-a for a,b in zip(cpus[0][:8],cpus[-1][:8])]
    out['guest_cpu']={'aggregate_busy_percent':(sum(delta)-delta[3]-delta[4])/sum(delta)*100 if sum(delta) else None,'iowait_percent':delta[4]/sum(delta)*100 if sum(delta) else None,'steal_percent':delta[7]/sum(delta)*100 if sum(delta) else None,'note':'Guest-visible counters cannot exclude host scheduling stalls.'}
    out['camera_usb_runtime_status_values']=sorted({r['values'].get('/sys/bus/usb/devices/4-1/power/runtime_status','missing').strip() for r in rs})
    return out

episodes=[]
for kind in ('color','depth'):
    es=[e for e in E if e['kind']==kind]
    for x,y in zip(es,es[1:]):
        gap=(y['source_ns']-x['source_ns'])/1e9
        if gap>.2:
            episodes.append({'kind':kind,'source_ns':y['source_ns'],'wall_ns':y['wall_ns'],'inside_task_receipt_window':start<=y['wall_ns']<=end,'source_gap_sec':gap,'callback_monotonic_gap_sec':(y['monotonic_ns']-x['monotonic_ns'])/1e9,'callback_source_age_sec':(y['wall_ns']-y['source_ns'])/1e9,'sequence_delta':y['seq']-x['seq'],'resources_plus_minus_1s':resource(y['wall_ns']-int(1e9),y['wall_ns']+int(1e9))})
slow=[]
for e in E:
    if e['kind']=='log' and e.get('node')=='/supervisor_camera' and e['message'].startswith('Camera backend read stages: '):
        z=json.loads(e['message'].split(': ',1)[1]); wait=z['host_wait']; prev=z['previous_completed_raw_frames']['raw_frames']; now=z['raw_frames']
        slow.append({'wall_ns':e['wall_ns'],'inside_task_receipt_window':start<=e['wall_ns']<=end,'SDK_wait_wall_sec':wait['elapsed_sec'],'SDK_wait_thread_cpu_sec':wait['thread_cpu_elapsed_sec'],'read_total_sec':z['total_sec'],'raw_color_counter_delta':now['color']['frame_number']-prev['color']['frame_number'],'raw_depth_counter_delta':now['depth']['frame_number']-prev['depth']['frame_number'],'raw_color_global_time_gap_sec':(now['color']['device_timestamp_ms']-prev['color']['device_timestamp_ms'])/1000,'raw_depth_global_time_gap_sec':(now['depth']['device_timestamp_ms']-prev['depth']['device_timestamp_ms'])/1000,'resources_plus_minus_1s':resource(e['wall_ns']-int(1e9),e['wall_ns']+int(1e9))})
joint=[e for e in E if e['kind']=='joint']; x,y=max(zip(joint,joint[1:]),key=lambda xy:xy[1]['source_ns']-xy[0]['source_ns'])
report={'schema':1,'source_raw_sha256':hashlib.sha256((P/'raw.json').read_bytes()).hexdigest(),'hardware_commands_sent':0,'service_actions':0,'camera_setting_changes':0,'analysis_is_offline':True,'task_generation':16,'task_window':{'start_ns':start,'end_ns':end,'duration_sec':(end-start)/1e9,'definition':'First active task callback through first inactive callback following it. Gap pairs are selected by later sample; first sample may straddle boundary.'},'task_by_callback_time':{k:stream(k,start,end) for k in ('color','depth','joint')},'task_by_source_time':{k:stream(k,start,end,'source_ns') for k in ('color','depth','joint')},'whole_record':{k:stream(k,d['started_ns'],d['ended_ns']) for k in ('color','depth','joint')},'task_resources':resource(start,end),'whole_record_resources':resource(d['started_ns'],d['ended_ns']),'camera_gap_episodes_over_200ms':episodes,'camera_slow_backend_records':slow,'largest_joint_source_gap':{'source_gap_sec':(y['source_ns']-x['source_ns'])/1e9,'later_source_ns':y['source_ns'],'later_wall_ns':y['wall_ns'],'sequence_delta':y['seq']-x['seq'],'inside_task_receipt_window':start<=y['wall_ns']<=end},'terminal_task_message':next(e['message'] for e in reversed(tasks) if e['state']=='FAILED' and e['wall_ns']>=start),'conclusions':['Generation 16 recorded camera source gaps and callback source ages stayed below 0.8 s; neither source-time nor callback-time window shows a camera freshness threshold breach.','Generation 16 ended on FINAL_REFINE_TIMEOUT with fresh camera-target distance 0.1564 m outside observation interval [0.22, 0.30] m; this record does not attribute that failure to camera frame loss.','Camera gap episodes cluster around task startup. Their +/-1 s sampled windows show elevated guest CPU waiting, I/O pressure, and approximately 93-100% sdb busy time. This temporal association is consistent with shared resource contention, but cannot isolate disk writes, USB transport, VM scheduling, or sensor behavior as the cause.',
'Two SDK waits of about 0.312 s and 0.273 s occurred just before generation 16. Their SDK wait CPU times were small, but this alone does not identify USB, sensor, host, or VM scheduling as cause.','Sequence skips at passive subscribers mean observed message source gaps can include lost subscriber deliveries; do not equate these gaps with physical sensor exposure gaps.','A 300 s window without camera gaps over 0.8 s does not establish that intermittent camera stalls are fixed.'],'limitations':['Source timestamps here are ROS image acquisition metadata/global-time stamps and SDK accepted joint feedback receipts, not independently calibrated physical camera exposure or MCU sampling times.','Callback source ages apply to this passive observer, not all downstream perception processing completion times.','Resource sampling is approximately 0.5 s and can miss sub-interval spikes; PSI avg10 is smoothed and overlaps previous activity.','Disk busy percentages use non-atomic proc counter reads and short intervals; slight values over 100% are sampling/counter granularity artifacts, not a physical utilization measurement exceeding capacity.',
'No controlled host/USB intervention was performed; temporal association is not causation, and guest telemetry does not expose complete host USB routing or scheduling.','Hardware_commands_sent=0 describes this analyzer and passive recorder; the separately authorized grasp task did move the arm during this window.']}
(P/'analysis.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({k:report[k] for k in ('task_window','task_by_callback_time','task_resources','largest_joint_source_gap')},indent=2))
print(P/'analysis.json')
