#!/usr/bin/env python3
"""Analyze one closed GEN60 sampling interval and source-bound event windows."""
import argparse
import ast
from collections import defaultdict, Counter
import hashlib
import json
import math
import os
from pathlib import Path
import time

W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z')
S=N/'gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/actual_attempt_r1'
R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261022T060615.487250Z_8db865e4c1094eb1bbdc72457cb35e2e')
OLD=W/'.runtime/analyze_gen59_actual_resources_20261022.py'
OLD_SHA='eb85a6b9ed667d8bcb4ac298d2d10578df038896a646eb1b671270195f4100e9'

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def read(path):
    return json.loads(Path(path).read_bytes())

def write(path,value):
    raw=(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
    with Path(path).open('xb') as stream:
        stream.write(raw);stream.flush();os.fsync(stream.fileno())
    assert Path(path).read_bytes()==raw

def pressure(text):
    return {line.split()[0]:{k:float(v) for k,v in (word.split('=',1) for word in line.split()[1:])}
            for line in (text or '').splitlines()}

def main():
    parser=argparse.ArgumentParser(__doc__)
    parser.add_argument('--boundaries',type=Path,required=True)
    parser.add_argument('--boundaries-sha256',required=True)
    args=parser.parse_args()
    assert Path('/media/zhuyupei/SanStand').is_mount()
    assert sha(args.boundaries)==args.boundaries_sha256 and sha(OLD)==OLD_SHA
    boundary=read(args.boundaries)
    assert boundary['kind']=='GEN60_actual_resource_event_windows' and boundary['record_root']==str(R)
    pins={str(args.boundaries.resolve()):args.boundaries_sha256,str(OLD):OLD_SHA,str(Path(__file__).resolve()):sha(__file__)}
    pins.update(boundary['input_sha256'])
    for path,digest in pins.items():assert sha(path)==digest,path
    for path in [S/'resources_r1'/name for name in ('metadata.json','samples.jsonl','STOP','result.json')]+[
            S/'terminal_process_receipt_r1/result.json',S/'post_attempt_idle_r1/result.json',R/'result_summary.json',R/'manifest.json']:
        pins[str(path)]=sha(path)
    meta=read(S/'resources_r1/metadata.json');closed=read(S/'resources_r1/result.json')
    assert closed['stop_reason']=='STOP_file' and closed['deployment_unchanged'] and closed['identity_error_count']==0
    assert closed['metadata_sha256']==pins[str(S/'resources_r1/metadata.json')]
    assert closed['samples_sha256']==pins[str(S/'resources_r1/samples.jsonl')]
    closure=read(S/'terminal_process_receipt_r1/result.json')
    assert pins[str(S/'terminal_process_receipt_r1/result.json')]=='575970abd0df0b43db98fd3fffbd0e4442678fe9a8be7361ce298878e3d4c398'
    assert closure['passed'] and closure['record_closed'] and closure['owned_pid_directories_absent']
    unit=closure['units']['alicia-gen60-task-sampler-r1.service']
    assert unit['MainPID']=='0' and unit['ExecMainStatus']=='0' and unit['SubState']=='exited'
    for path,digest in closure['input_sha256'].items():assert sha(path)==digest; pins[path]=digest
    assert pins[str(S/'post_attempt_idle_r1/result.json')]=='0886f5c69ed7dce3f0d57029cc0fd3a2081568dfab76d86d87357e7dbd3400bf'
    post=read(S/'post_attempt_idle_r1/result.json')
    assert post['passed'] and post['task_active'] is False and post['actions_active'] is False
    for key in ('helper','primitives'):
        assert sha(meta[key+'_path'])==meta[key+'_sha256'];pins[meta[key+'_path']]=meta[key+'_sha256']
    assert sha(meta['deployment'])==meta['deployment_sha256'];pins[meta['deployment']]=meta['deployment_sha256']
    samples=[json.loads(line) for line in (S/'resources_r1/samples.jsonl').read_text().splitlines()]
    assert len(samples)==closed['sample_count']==230
    assert all(not row['identity_errors'] for row in samples)
    assert all(b['recorded_ns']>a['recorded_ns'] and b['monotonic_ns']>a['monotonic_ns'] for a,b in zip(samples,samples[1:]))
    targets={name:(p['pid'],p['start_ticks']) for name,p in meta['target_process_identities'].items()}
    assert len(targets)==17
    for row in samples:
        row['_proc']={(p['pid'],p['start_ticks']):p for p in row['processes']}
        assert len(row['_proc'])==len(row['processes']) and all(key in row['_proc'] for key in targets.values())
    # Reuse the exact reviewed GEN59 arithmetic without importing/executing its
    # hard-coded main, record assumptions, or old task boundaries.
    tree=ast.parse(OLD.read_bytes())
    selected=[]
    for name in ('percentile','window_report'):
        matches=[n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name==name]
        assert len(matches)==1;selected.append(matches[0])
    ns=dict(defaultdict=defaultdict,Counter=Counter,math=math,samples=samples,hz=meta['clock_ticks_per_second'])
    exec(compile(ast.Module(body=selected,type_ignores=[]),str(OLD),'exec'),ns)
    windows=boundary['phases'];active=windows['task_active'];whole=windows['whole_automatic_attempt']
    assert samples[0]['recorded_ns']<=whole['start_ns']<active['start_ns']<active['end_ns']==whole['end_ns']<=samples[-1]['recorded_ns']
    phases={'pre_selection_gate_and_preparation':dict(start_ns=samples[0]['recorded_ns'],end_ns=whole['start_ns']),
            **windows,'post_attempt_tail':dict(start_ns=whole['end_ns'],end_ns=samples[-1]['recorded_ns'])}
    reports={name:ns['window_report'](name,row['start_ns'],row['end_ns']) for name,row in phases.items()}
    pressures={}
    for name,phase in phases.items():
        rows=[row for row in samples if phase['start_ns']<=row['recorded_ns']<=phase['end_ns']]
        if len(rows)<2:
            pressures[name]=dict(available=False,reason='fewer than two samples in exact event window');continue
        dt=(rows[-1]['monotonic_ns']-rows[0]['monotonic_ns'])/1e9
        metrics={}
        for resource in ('cpu','memory','io'):
            a,b=pressure(rows[0]['pressure'][resource]),pressure(rows[-1]['pressure'][resource])
            metrics[resource]={kind:dict(stall_delta_sec=(b[kind]['total']-a[kind]['total'])/1e6,
                fraction_percent=100*(b[kind]['total']-a[kind]['total'])/1e6/dt,
                peak_reported_avg10_percent=max(pressure(row['pressure'][resource])[kind]['avg10'] for row in rows)) for kind in a}
        pressures[name]=dict(available=True,counter_coverage_sec=dt,pressure=metrics,
            swap_used_delta_kib=rows[0]['memory_kib']['SwapFree']-rows[-1]['memory_kib']['SwapFree'],
            dirty_peak_kib=max(row['memory_kib']['Dirty'] for row in rows),writeback_peak_kib=max(row['memory_kib']['Writeback'] for row in rows))
    terminal=read(R/'result_summary.json')
    assert terminal['state']=='complete' and terminal['result']=='failure'
    result=dict(kind='GEN60_closed_actual_resource_analysis',passed=True,recorded_ns=time.time_ns(),input_sha256=pins,
        source_record=str(R),terminal_result='failure',failure_reason=terminal['failure_reason'],
        task_boundary_method=boundary['method'],event_windows=windows,phases=reports,pressure=pressures,
        planning_and_rpc=boundary['planning_and_rpc'],
        sampling_closure=dict(sample_count=len(samples),elapsed_sec=closed['elapsed_sec'],stop_reason=closed['stop_reason'],
            deployment_unchanged=True,identity_error_count=0,observer_cpu_sec=closed['observer_cpu_sec'],
            maximum_collection_sec=closed['maximum_collection_sec'],accepted_process_count=17,
            tracked_identity_count=closed['tracked_identity_count'],main_pid_zero_and_exit0=True,
            observed_owned_processes_absent=closure['owned_pid_directories_absent']),
        sampling_quality=dict(first_sample_ns=samples[0]['recorded_ns'],last_sample_ns=samples[-1]['recorded_ns'],
            min_interval_sec=min((b['monotonic_ns']-a['monotonic_ns'])/1e9 for a,b in zip(samples,samples[1:])),
            max_interval_sec=max((b['monotonic_ns']-a['monotonic_ns'])/1e9 for a,b in zip(samples,samples[1:])),
            wall_minus_monotonic_duration_sec=((samples[-1]['recorded_ns']-samples[0]['recorded_ns'])-(samples[-1]['monotonic_ns']-samples[0]['monotonic_ns']))/1e9),
        limitations=['Only adjacent samples fully contained within each event window count; no boundary interpolation. Short windows can have no resource pair.',
            'CPU 100 percent means one logical core. Logical CPU count is capacity, not measured utilization. Host counters include unrelated work.',
            'RSS sums may count shared pages more than once. Sampling may miss short-lived processes. Logical rchar/wchar differ from physical storage I/O.',
            'schedstat and wchan concern the sampled main thread and do not measure ROS callback queues, GUI frame latency or application/RPC waiting.',
            'RPC wall time includes compute, transport and waiting; no decomposition is invented. No remote-server CPU measurement is available.',
            'Host PSI intervals overlap across resources; they must not be added or attributed to an individual process.',
            'Pre-active computation and handoff after selection are included in whole-attempt load. Pre-selection preparation and post-terminal tail are separate; phase counters need not sum due to boundary loss and overlapping windows.',
            'No speedup, path safety, or physical grasp success is established.'],
        ROS_contacted=False,network_contacted=False,hardware_contacted=False,tests_run=False,production_modified=False)
    for path,digest in pins.items():assert sha(path)==digest,path
    out=S/'resource_review_r1';out.mkdir()
    write(out/'result.json',result)
    a=reports['task_active'];whole_report=reports['whole_automatic_attempt']
    lines=['# GEN60 实际任务资源分析','',
        '本报告仅分析已封口采样、状态事件和日志。本次任务失败，原因见末尾。整次自动尝试从selection4提交开始，包含task-active前的远端推理和观察候选规划；selection前准备及terminal后采样单列。',
        '',f"采样 {len(samples)} 次，{closed['elapsed_sec']:.3f} 秒；STOP 正常退出，MainPID=0、退出码0，17 个验收进程身份错误为0。",'',
        '| 阶段 | 事件跨度 秒 | 计数覆盖 秒 | 跟踪 CPU 秒 | 平均核等价值 |',
        '|---|---:|---:|---:|---:|']
    for name,q in reports.items():
        cores=q['all_tracked_mean_cpu_percent_one_core_100']
        lines.append('| {} | {:.3f} | {:.3f} | {:.2f} | {} |'.format(name,q['exact_window_sec'],q['counter_pair_coverage_sec'],q['all_tracked_cpu_sec'],'无成对采样' if cores is None else '%.3f'%(cores/100)))
    lines += ['',f"整次自动尝试：验收节点 CPU {whole_report['accepted_process_cpu_sec']:.2f} 秒，含本次录制等已归属子进程共 {whole_report['all_tracked_cpu_sec']:.2f} 秒；主机忙碌 {whole_report['host_cpu_busy_percent']:.2f}%，I/O wait {whole_report['host_cpu_iowait_percent']:.2f}%。RSS相加峰值 {whole_report['tracked_rss_sum_peak_bytes']/2**30:.3f} GiB（共享页可能重复），MemAvailable最低 {whole_report['mem_available_min_kib']/2**20:.3f} GiB。",'',
        f"其中task-active窗口：验收节点 CPU {a['accepted_process_cpu_sec']:.2f} 秒，全部跟踪进程共 {a['all_tracked_cpu_sec']:.2f} 秒；主机忙碌 {a['host_cpu_busy_percent']:.2f}%，I/O wait {a['host_cpu_iowait_percent']:.2f}%。",'',
        '以下每进程统计覆盖整次自动尝试：','',
        '| 进程 | PID | CPU秒 | 平均CPU%，100%=1核 | RSS峰MiB | 存储写MiB |','|---|---:|---:|---:|---:|---:|']
    for p in whole_report['processes']:
        lines.append('| {} | {} | {:.2f} | {} | {:.2f} | {:.3f} |'.format(p['name'],p['pid'],p['cpu_sec'],'无成对采样' if p['mean_cpu_percent_one_core_100'] is None else '%.2f'%p['mean_cpu_percent_one_core_100'],(p['rss_peak_bytes'] or 0)/2**20,p['io_delta'].get('write_bytes',0)/2**20))
    lines += ['',boundary['method'],'',*boundary['readable_stage_notes'],'',
        '终态：'+terminal['failure_reason'],'',
        '应用队列长度、图像到达延迟、GUI帧时间、RPC内部计算/传输/等待拆分均无直接测量。主线程调度计数不能当成应用队列或全进程等待。未作加速或实物成功声明。',
        '', '完整每进程 CPU/RSS/IO、主机压力、窗口覆盖与源 SHA 见 result.json。']
    with (out/'REPORT.md').open('x') as stream:stream.write('\n'.join(lines)+'\n')
    write(out/'manifest.json',dict(passed=True,kind='GEN60_actual_resource_analysis_closed',artifact_sha256={str(p):sha(p) for p in (out/'result.json',out/'REPORT.md')},all_inputs_rechecked=True))
    print(json.dumps(dict(passed=True,result=str(out/'result.json'),sha256=sha(out/'result.json'),report_sha256=sha(out/'REPORT.md'),active_cpu_sec=a['all_tracked_cpu_sec'],active_wall_sec=a['exact_window_sec'])))

if __name__=='__main__':main()
