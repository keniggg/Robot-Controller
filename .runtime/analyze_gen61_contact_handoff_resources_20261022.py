"""Offline analysis of a sealed GEN61 task; consumes shared non-temperature extraction."""
from pathlib import Path
from collections import Counter, defaultdict
import csv
import datetime as dt
import gzip
import hashlib
import json
import os
import statistics
import time

W = Path(__file__).resolve().parents[1]
N = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z')
E = N / 'gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1'
R = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261022T092802.096230Z_567d85bbe8f54a56af71145548b078f8')
S = E / 'actual_attempt_r1/resource_samples_r3'
D = E / 'contact_handoff_diagnosis_r1'
O = E / 'contact_handoff_resources_r1'

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def read(path):
    return json.loads(Path(path).read_bytes())

def save(path, value):
    raw = (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n').encode()
    with path.open('xb') as stream:
        stream.write(raw)
        stream.flush()
        os.fsync(stream.fileno())
    assert path.read_bytes() == raw

def iso_ns(text):
    value = dt.datetime.fromisoformat(text)
    seconds = int(value.timestamp())
    return seconds * 10**9 + value.microsecond * 1000

def main():
    started, cpu_started = time.monotonic(), time.process_time()
    assert Path('/media/zhuyupei/SanStand').is_mount()
    frozen = read(O / 'frozen_inputs.json')
    assert all(sha(p) == h for p, h in frozen['pins'].items())
    raw = gzip.decompress((O / 'resource_prefix.jsonl.gz').read_bytes())
    assert hashlib.sha256(raw).hexdigest() == frozen['complete_line_prefix_sha256']
    sampler = read(S / 'result.json')
    assert sampler['sample_count'] == 900 and sampler['stop_reason'] == 'duration_limit'
    assert sha(S / 'samples.jsonl') == sampler['samples_sha256'] == '05c6a9690b2c44c1c64670dbba9710914e155e8177c00bba9e3cc61ab031bbcf'
    assert (S / 'samples.jsonl').read_bytes().startswith(raw)
    meta = read(S / 'metadata.json')
    rows = [json.loads(line) for line in raw.splitlines()]
    assert len(rows) == frozen['sample_count'] == 191
    assert not any(row['identity_errors'] for row in rows)
    hz = meta['clock_ticks_per_second']
    wire = read(D / 'wire_result.json')
    assert wire['passed'] and wire['source_record'] == str(R)
    assert all(sha(p) == h for p, h in wire['artifact_sha256'].items())
    manifest = read(R / 'manifest.json')
    assert manifest['state'] == 'complete' and manifest['recorder_closed'] and manifest['result'] == 'failure'
    record_start, record_end = iso_ns(manifest['started_at']), iso_ns(manifest['completed_at'])
    events = [dict(row['message']['parsed'], bag_receipt_ns=row['bag_receipt_ns'])
              for row in read(D / 'task_events.json') if row['topic'] == '/grasp/task_event']
    by_seq = {row['event_seq']: row for row in events}
    assert set(by_seq) == set(range(1, 15))
    requests = read(D / 'contact_requests.json')
    assert len(requests) == 1
    prepare = requests[0]['message']['parsed']
    failure = by_seq[13]['stamp_ns']
    timing = None
    for line in (R / 'runner.log').read_text().splitlines():
        if line.startswith('USER_REQUESTED_GRASP '):
            timing = json.loads(line.partition(' ')[2])['runner_startup_timing']
    assert timing is not None
    actions, profiles, deliveries = [], [], []
    topics = {'/alicia_controller/follow_joint_trajectory/goal', '/alicia_controller/follow_joint_trajectory/result'}
    for line in (D / 'selected_wire.jsonl').open('rb'):
        if not any(k in line for k in (b'"/rosout_agg"', b'"/alicia_controller/follow_joint_trajectory/goal"', b'"/alicia_controller/follow_joint_trajectory/result"')):
            continue
        row = json.loads(line)
        message = row['message']
        if row['topic'] in topics:
            actions.append(dict(topic=row['topic'], bag_receipt_ns=row['bag_receipt_ns'], source_ns=message['header']['stamp']['ns'],
                                goal_id=message.get('goal_id', message.get('status', {}).get('goal_id')),
                                result=message.get('result')))
        if row['topic'] != '/rosout_agg':
            continue
        text = message.get('msg', '')
        if text.startswith('remote 6D pipeline metrics: '):
            profile = json.loads(text.split(': ', 1)[1])
            if profile.get('event') != 'request_completed' or profile.get('drop_reason'):
                continue
            allowed = ('request_id','generation','end_to_end_ms','result_age_ms','ros_prepare_ms','ros_prepare_stages_ms',
                       'transport_ms','wsl_inference_ms','wsl_preprocess_ms','wsl_postprocess_ms','wsl_total_ms',
                       'request_preparation_profile','pending_replacements','started','completed','submitted')
            profiles.append(dict(log_source_ns=message['header']['stamp']['ns'], **{k:profile[k] for k in allowed if k in profile}))
        if text.startswith('remote 6D contact precheck delivery: '):
            value = json.loads(text.split(': ', 1)[1])
            response = value.get('response', {})
            precheck = value.get('precheck', {})
            start, end = value.get('request_monotonic_ns'), response.get('received_monotonic_ns')
            deliveries.append(dict(source_ns=message['header']['stamp']['ns'],operation=value['operation'],
                consumer_outcome=value['consumer_outcome'],request_wall_ns=value.get('request_wall_ns'),response_wall_ns=response.get('received_wall_ns'),
                measured_roundtrip_sec=(end-start)/1e9 if start is not None and end is not None else None,
                following_sample_acquisition=precheck.get('following_sample_acquisition'),
                prediction_cache_hit=precheck.get('prediction_cache_hit'),
                failure_code=precheck.get('failure_code'),execution_authorized=False))
    assert len(actions) == 2 and actions[0]['goal_id'] == actions[1]['goal_id']
    action_start, action_end = actions[0]['source_ns'], actions[1]['source_ns']
    bounds = [
        ('pre_record_idle', record_start-30*10**9, record_start, 'Idle baseline; no active task'),
        ('record_setup_and_mode_switch', record_start, timing['selection_commit_ros_ns'], 'Recorded setup and selection commit'),
        ('initial_candidate_until_task_entry', timing['selection_commit_ros_ns'], by_seq[1]['stamp_ns'], 'Initial candidate computation and plan admission'),
        ('observation_preflight_and_submission', by_seq[1]['stamp_ns'], action_start, 'Observation preflight, handoff and trajectory submission'),
        ('observation_controller_execution', action_start, action_end, 'Recorded controller action goal to successful result; includes feedback waiting'),
        ('observation_endpoint_and_qualification', action_end, by_seq[8]['stamp_ns'], 'Endpoint correction/measurement qualification'),
        ('contact_candidate_and_rebind', by_seq[8]['stamp_ns'], by_seq[10]['stamp_ns'], 'Near-field candidate/strict checks and task rebind'),
        ('contact_preflight_before_prepare', by_seq[10]['stamp_ns'], prepare['stamp_ns'], 'Strict preflight before contact prepare'),
        ('contact_prepare_to_failure', prepare['stamp_ns'], failure, 'Prepare published; no retire or contact controller goal observed'),
        ('failure_closure', failure, record_end, 'Failure, stop and record sealing'),
        ('post_record_idle', record_end, record_end+29*10**9, 'Post-task baseline; keep separate from active task'),
    ]
    windows = bounds + [('record_to_failure',record_start,failure,'Aggregate, overlaps detailed phases'),
                       ('contact_phase_total',by_seq[8]['stamp_ns'],failure,'Aggregate, overlaps detailed phases')]
    intervals = []
    for previous, current in zip(rows, rows[1:]):
        span=(current['monotonic_ns']-previous['monotonic_ns'])/1e9
        assert span > 0 and current['recorded_ns'] > previous['recorded_ns']
        prior={(p['pid'],p['start_ticks']):p for p in previous['processes']}
        assert len(prior)==len(previous['processes'])
        owners=defaultdict(lambda: dict(cpu_sec=0.,read_bytes=0,write_bytes=0,logical_read_bytes=0,logical_write_bytes=0,rss_mib=0.,process_count=0))
        for process in current['processes']:
            old=prior.get((process['pid'],process['start_ticks']))
            owner=owners[process['attribution']['owner']]
            owner['rss_mib']+=process['rss_bytes']/2**20;owner['process_count']+=1
            if old is None:continue
            delta=(process['cpu_ticks']-old['cpu_ticks'])/hz
            assert delta>=0;owner['cpu_sec']+=delta
            for target,source in [('read_bytes','read_bytes'),('write_bytes','write_bytes'),('logical_read_bytes','rchar'),('logical_write_bytes','wchar')]:
                owner[target]+=process['io'].get(source,0)-old['io'].get(source,0)
        device=current['external_filesystem_device']['counters'];old_device=previous['external_filesystem_device']['counters']
        intervals.append(dict(start_ns=previous['recorded_ns'],end_ns=current['recorded_ns'],seconds=span,owners=dict(owners),
            host_busy_percent=current['interval_summary']['host_busy_percent'],host_iowait_percent=current['interval_summary']['host_iowait_percent'],
            min_available_mib=min(previous['memory_kib']['MemAvailable'],current['memory_kib']['MemAvailable'])/1024,
            external_read_bytes=device['read_bytes']-old_device['read_bytes'],external_write_bytes=device['write_bytes']-old_device['write_bytes'],
            external_busy_ms=device['busy_ms']-old_device['busy_ms'],external_weighted_busy_ms=device['weighted_busy_ms']-old_device['weighted_busy_ms']))
    phases=[]
    for name,start,end,note in windows:
        selected=[row for row in intervals if row['start_ns']>=start and row['end_ns']<=end]
        coverage=sum(row['seconds'] for row in selected)
        owners=defaultdict(lambda: dict(cpu_sec=0.,read_bytes=0,write_bytes=0,logical_read_bytes=0,logical_write_bytes=0,peak_sampled_rss_mib=0.,peak_interval_cores=0.))
        for interval in selected:
            for owner,values in interval['owners'].items():
                target=owners[owner]
                for key in ('cpu_sec','read_bytes','write_bytes','logical_read_bytes','logical_write_bytes'):target[key]+=values[key]
                target['peak_sampled_rss_mib']=max(target['peak_sampled_rss_mib'],values['rss_mib'])
                target['peak_interval_cores']=max(target['peak_interval_cores'],values['cpu_sec']/interval['seconds'])
        for values in owners.values():values['mean_cores']=values['cpu_sec']/coverage if coverage else None
        phases.append(dict(name=name,start_ns=start,end_ns=end,exact_event_duration_sec=(end-start)/1e9,note=note,
            complete_sample_interval_count=len(selected),sampled_coverage_sec=coverage,owners=dict(owners),
            tracked_cpu_sec=sum(v['cpu_sec'] for v in owners.values()),
            host_busy_mean_percent=sum(i['host_busy_percent']*i['seconds'] for i in selected)/coverage if coverage else None,
            host_busy_peak_percent=max((i['host_busy_percent'] for i in selected),default=None),
            host_iowait_mean_percent=sum(i['host_iowait_percent']*i['seconds'] for i in selected)/coverage if coverage else None,
            minimum_available_mib=min((i['min_available_mib'] for i in selected),default=None),
            external_write_mib=sum(i['external_write_bytes'] for i in selected)/2**20,
            external_read_mib=sum(i['external_read_bytes'] for i in selected)/2**20,
            external_busy_percent=sum(i['external_busy_ms'] for i in selected)/(coverage*10) if coverage else None))
    handoff=[i for i in intervals if i['end_ns']>=prepare['stamp_ns']-5*10**9 and i['start_ns']<=failure+10**9]
    thread_rows=[]
    for row in rows:
        if not prepare['stamp_ns']-10*10**9<=row['recorded_ns']<=failure+5*10**9:continue
        process=next(p for p in row['processes'] if p['name']=='/grasp_task_node')
        if not process.get('thread_samples'):continue
        thread_rows.append(dict(recorded_ns=row['recorded_ns'],state_counts=dict(Counter(t['state'] for t in process['thread_samples'])),
            wchan_counts=dict(Counter(t['wchan'] for t in process['thread_samples'])),thread_samples=process['thread_samples']))
    thread_deltas=[]
    for a,b in zip(thread_rows,thread_rows[1:]):
        old={(t['tid'],t['start_ticks']):t for t in a['thread_samples']}
        for row in b['thread_samples']:
            prior=old.get((row['tid'],row['start_ticks']))
            if prior is None:continue
            old_stat=list(map(int,prior['schedstat'].split()));new_stat=list(map(int,row['schedstat'].split()))
            thread_deltas.append(dict(tid=row['tid'],start_ns=a['recorded_ns'],end_ns=b['recorded_ns'],
                cpu_sec=(row['cpu_ticks']-prior['cpu_ticks'])/hz,scheduler_run_sec=(new_stat[0]-old_stat[0])/1e9,
                scheduler_runnable_wait_sec=(new_stat[1]-old_stat[1])/1e9))
    cpu_hot=sorted(thread_deltas,key=lambda row:row['cpu_sec'],reverse=True)[:10]
    phase_path=O/'phase_resources.json';save(phase_path,dict(phases=phases,events=events,actions=actions,prepare=prepare))
    handoff_path=O/'handoff_window.json';save(handoff_path,dict(intervals=handoff,thread_samples=thread_rows,highest_cpu_thread_intervals=cpu_hot,feedback=wire['sdk_failure_window']))
    profile_path=O/'request_timing.json';save(profile_path,dict(profiles=profiles,contact_precheck_deliveries=deliveries,runner_startup_timing=timing,
        notes=['Mode_prepare elapsed includes base_prepare; do not add overlapping scope totals.','Service roundtrip and process CPU are different scopes.','A request latency percentile containing dropped requests is not successful-plan latency.']))
    csv_path=O/'resource_timeline.csv'
    owner_names=sorted({name for i in intervals for name in i['owners']})
    with csv_path.open('x',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['start_ns','end_ns','seconds','host_busy_percent','host_iowait_percent','available_mib','external_write_mib']+[n+'_cores' for n in owner_names])
        for i in intervals:writer.writerow([i['start_ns'],i['end_ns'],i['seconds'],i['host_busy_percent'],i['host_iowait_percent'],i['min_available_mib'],i['external_write_bytes']/2**20]+[i['owners'].get(n,{}).get('cpu_sec',0)/i['seconds'] for n in owner_names])
    resources={p['name']:p for p in phases}
    report=['# GEN61 实际接触交接失败：阶段资源分析','',
        '原任务结果为失败，未发生接触阶段控制器提交。仅分析已封口记录；未调用 ROS、硬件或重解 bag。',
        '', '|阶段|事件时长 s|完整采样覆盖 s|宿主 busy %|任务 CPU 核|remote CPU 核|感知 CPU 核|最低可用 MiB|',
        '|---|---:|---:|---:|---:|---:|---:|---:|']
    def fmt(value):return '—' if value is None else '%.3f'%value
    for p in phases[:len(bounds)]:
        means=[p['owners'].get(n,{}).get('mean_cores') for n in ['/grasp_task_node','/remote_grasp6d_node','/unknown_tabletop_perception']]
        report.append('|'+ '|'.join([p['name'],fmt(p['exact_event_duration_sec']),fmt(p['sampled_coverage_sec']),fmt(p['host_busy_mean_percent'])]+[fmt(x) for x in means]+[fmt(p['minimum_available_mib'])])+'|')
    report += ['',
        '阶段时长来自实际 task event / controller goal-result / prepare。CPU 与 IO 仅累计完全落在阶段内的相邻样本区间，未对边界线性分摊；短阶段可能没有完整区间。100% 进程 CPU 表示一个逻辑核心。',
        '',
        '观察动作的控制器 goal→成功 result 为 %.3f s；这是动作与反馈等待窗口，不能归为纯计算时间。接触候选至任务 rebind 为 %.3f s；此时 task/remote/gateway 各有计算和服务等待，不能由低单个进程 CPU 推断空闲。'%( (action_end-action_start)/1e9,resources['contact_candidate_and_rebind']['exact_event_duration_sec']),
        '',
        'prepare→首个 FAILED 为 %.3f s，原 prepare 绝对窗口仅 %.3f s。最后两个失败前区间 task CPU 接近一个核心，宿主仍有余量；这支持进一步定位任务内部同步工作/锁或 GIL 调度，但 /proc 不能确认具体 Python 栈或回调锁。'%((failure-prepare['stamp_ns'])/1e9,(prepare['deadline_ns']-prepare['stamp_ns'])/1e9),
        '',
        '独立录制器在失败附近收到 80 条 sdk_measured，最大源间隔 %.3f ms、最大录制到达延迟 %.3f ms，失败前最新源年龄 %.3f ms。该证据反对驱动停止产生反馈；它不能证明 task 本地回调及时入缓存。'%(wire['sdk_failure_window']['maximum_source_gap_ns']/1e6,wire['sdk_failure_window']['maximum_receipt_age_ns']/1e6,wire['sdk_failure_window']['source_age_at_failure_ns']/1e6),
        '',
        '近场请求 34 的 end_to_end 为 27.972 s；记录的预测 RPC transport 为 1.124 s，服务自身 total 为 1.083 s，其中 inference 0.626 s。base_prepare 为 3.733 s / 当前线程 CPU 2.281 s，geometry_registration 1.689 s / 1.534 s，tabletop_candidates 0.853 s / 0.696 s，predict 1.141 s / 0.017 s。这些嵌套范围不能相加为端到端总耗时；剩余阶段包含本地几何、严格服务检查、发布等，应按具体事件进一步拆分。',
        '',
        '回归建议：以实录 prepare/header/feedback 顺序复现，同时区分驱动源时间、录制器到达、task callback 入缓存与读取快照时刻；覆盖检查期间新帧到达、锁等待、乱序/未来时间、epoch 改变、原绝对截止超时、姿态变化及 ACK 缺失。保持 0.8 s/位移/计划/控制所有权门限，先定位并减少不可中断的局部工作，不能靠重置截止或放宽 freshness 通过。',
        '',
        '采样器共 900 样本、900.063 s，自然退出；观察器 CPU 68.078 s（约 0.0756 核），最大单次采集 0.180 s。资源分析仅使用事先冻结的前 191 样本，覆盖任务前基线至封口后约 30 s，后续长空闲不混入任务均值。',
        '',
        '限制：1 s 采样及每五次线程采样不能直接诊断 800 ms 瞬态。线程 wchan 是瞬时等待点；schedstat 等待指 runnable 调度等待而非回调队列。进程 RSS 可能共享页面，设备 IO 包含所有 SanStand 写入者。未读取温度、未修改源码或门限，物理成功仍未成立。', '']
    report_path=O/'analysis.md';report_path.write_text('\n'.join(report))
    artifacts=[phase_path,handoff_path,profile_path,csv_path,report_path]
    pins={str(p):sha(p) for p in [S/'metadata.json',S/'result.json',D/'wire_result.json',O/'frozen_inputs.json',Path(__file__)]}
    result=dict(kind='GEN61_actual_contact_handoff_resource_analysis',passed=True,original_task_result='failure',physical_grasp_verified=False,
        source_record=str(R),source_bag_sha256=wire['bag_sha256'],whole_sampler_sha256=sampler['samples_sha256'],
        frozen_prefix_sha256=frozen['complete_line_prefix_sha256'],sample_prefix_count=len(rows),sampler_closed=True,
        sampler_result_sha256=sha(S/'result.json'),sampler_identity_errors=0,pins=pins,shared_wire_artifacts=wire['artifact_sha256'],
        artifacts={str(p):sha(p) for p in artifacts},stage_count=len(bounds),original_task_duration_sec=(record_end-record_start)/1e9,
        preparation_deadline_sec=(prepare['deadline_ns']-prepare['stamp_ns'])/1e9,prepare_to_failure_sec=(failure-prepare['stamp_ns'])/1e9,
        task_local_callback_timeliness_proven=False,resource_exhaustion_supported=False,cause_status='local synchronous work/cache-lock candidate; not a reproduced root cause',
        bag_decoded_here=False,ROS_contacted=False,motion_commands_sent=False,temperature_read=False,source_modified=False,
        analysis_elapsed_sec=time.monotonic()-started,analysis_cpu_sec=time.process_time()-cpu_started)
    save(O/'result.json',result)
    print(json.dumps(dict(result_sha256=sha(O/'result.json'),phases=[{k:p[k] for k in ['name','exact_event_duration_sec','sampled_coverage_sec','host_busy_mean_percent','minimum_available_mib','external_write_mib']} for p in phases],hot_thread_intervals=cpu_hot[:3],profiles=[{k:v for k,v in p.items() if k!='request_preparation_profile'} for p in profiles]),ensure_ascii=False,indent=2))

if __name__ == '__main__':
    main()
