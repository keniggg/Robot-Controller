#!/usr/bin/env python3
"""Summarize closed offline resource analysis and pressure counters."""
from pathlib import Path
import hashlib
import json
import time

S=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/actual_attempt_r1')
O=S/'resource_review_r1'

def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p): return json.loads(Path(p).read_text())

def pressure(text):
    result={}
    for line in (text or '').splitlines():
        words=line.split();result[words[0]]={k:float(v) for k,v in (w.split('=',1) for w in words[1:])}
    return result

def main():
    source=O/'result.json'
    assert sha(source)=='95c616e77e7df6b93624b2981d1707753b72d4fcf3a8ffada43f3a2f2e61da6b'
    d=read(source);samples=[json.loads(l) for l in (S/'resource_samples_r1/samples.jsonl').read_text().splitlines()]
    assert sha(S/'resource_samples_r1/samples.jsonl')==d['input_sha256'][str(S/'resource_samples_r1/samples.jsonl')]
    data={}
    for name,q in d['phases'].items():
        rows=[s for s in samples if q['start_ns']<=s['recorded_ns']<=q['end_ns']]
        if len(rows)<2:continue
        covered=(rows[-1]['monotonic_ns']-rows[0]['monotonic_ns'])/1e9
        metrics={}
        for resource in ('cpu','memory','io'):
            x,y=pressure(rows[0]['pressure'][resource]),pressure(rows[-1]['pressure'][resource]);m={}
            for kind in x:
                stall=(y[kind]['total']-x[kind]['total'])/1e6
                m[kind]=dict(stall_total_delta_sec=stall,stall_fraction_percent=100*stall/covered,
                    maximum_reported_avg10_percent=max(pressure(s['pressure'][resource])[kind]['avg10'] for s in rows))
            metrics[resource]=m
        data[name]=dict(counter_covered_sec=covered,pressure=metrics,
            swap_used_delta_kib=rows[0]['memory_kib']['SwapFree']-rows[-1]['memory_kib']['SwapFree'],
            dirty_peak_kib=max(s['memory_kib']['Dirty'] for s in rows),writeback_peak_kib=max(s['memory_kib']['Writeback'] for s in rows))
    supplemental=dict(kind='GEN59_resource_pressure_and_readable_summary',recorded_ns=time.time_ns(),helper_sha256=sha(__file__),
        input_sha256={str(source):sha(source),str(S/'resource_samples_r1/samples.jsonl'):sha(S/'resource_samples_r1/samples.jsonl')},
        phases=data,semantics='Host PSI stalls overlap; neither add across resources nor attribute to a single process. No callback/application queue occupancy is available.',
        no_live_calls=True)
    with (O/'pressure.json').open('x') as f:json.dump(supplemental,f,indent=2);f.write('\n')
    active=d['phases']['task_active_observed'];near=d['phases']['near_field'];motion=d['phases']['observation_motion_stage_including_setup']
    lines=[
        '# GEN59 实际任务资源分析', '',
        '本报告仅分析已封口采样、任务文本和审计。任务结果为失败：近场共用截止时间耗尽，未选出通过严格检查的接触候选。没有再次访问 ROS、网络或硬件。', '',
        f"756 次采样，记录区间 {d['interval_quality']['first_sample_ns']}—{d['interval_quality']['last_sample_ns']}，周期实测 {d['interval_quality']['min_interval_sec']:.3f}—{d['interval_quality']['max_interval_sec']:.3f} 秒。STOP 文件、结果 SHA 和父代理进程退出回执一致；采样器 MainPID=0、退出码 0、RemainAfterExit 的 active/exited，无运行进程。17 个验收节点全程存在，身份错误 0，另跟踪 4 个本次录制/运行子进程。",
        '',
        '活动起点采用已保存的首条明确活动状态日志 1792644293.514122；精确消息头起点未导出，后续状态日志晚 36.057 ms。终点为实际 inactive 状态消息头 1792644421730645179。下列计数仅使用完全位于窗口内的相邻采样，边界不足一秒的部分不插值、不混入。',
        '',
        '| 区间 | 事件窗口 秒 | 计数覆盖 秒 | 已跟踪 CPU 秒 | 平均核等价值 |',
        '|---|---:|---:|---:|---:|',
    ]
    labels={'pre_task_not_task_load':'前置准备/空闲（排除任务负载）','task_active_observed':'已证活动窗口','observation_motion_stage_including_setup':'观测运动阶段，含提交准备','arrival_qualification':'运动完成至近场开始','near_field':'近场处理','post_task_tail':'结束后尾部（排除任务负载）'}
    for name,q in d['phases'].items():
        lines.append(f"| {labels[name]} | {q['exact_window_sec']:.3f} | {q['counter_pair_coverage_sec']:.3f} | {q['all_tracked_cpu_sec']:.2f} | {q['all_tracked_mean_cpu_percent_one_core_100']/100:.3f} |")
    lines += ['', '分段与总窗口存在边界采样损失，不能把各分段计数机械相加。前置窗口包含清场、采集和初始选视角，不能称为纯空闲；它没有计入任务活动负载。', '',
        f"活动窗口内：17 个验收进程 CPU {active['accepted_process_cpu_sec']:.2f} 秒；加上本次子进程共 {active['all_tracked_cpu_sec']:.2f} 秒。主机全部工作忙碌比例 {active['host_cpu_busy_percent']:.2f}%，I/O wait {active['host_cpu_iowait_percent']:.2f}%。12 个逻辑 CPU 只是可用容量。RSS 相加采样峰值 {active['tracked_rss_sum_peak_bytes']/2**30:.3f} GiB（共享页可能重复）；MemAvailable 最低 {active['mem_available_min_kib']/2**20:.3f} GiB。",
        '',
        '| 进程/本次子进程 | PID | 活动 CPU 秒 | 平均 CPU %，100%=1核 | RSS 峰 MiB | 存储写入 MiB |',
        '|---|---:|---:|---:|---:|---:|']
    for r in active['processes']:
        lines.append(f"| {r['name']} | {r['pid']} | {r['cpu_sec']:.2f} | {r['mean_cpu_percent_one_core_100']:.2f} | {r['rss_peak_bytes']/2**20:.2f} | {r['io_delta'].get('write_bytes',0)/2**20:.3f} |")
    lines += ['',
        f"观测运动阶段日志跨度 {motion['exact_window_sec']:.3f} 秒，其中提交的重定时轨迹为 33.048 秒；其余包含准备、交接和结果返回，不能当作多出的纯计算时间。此阶段采样平均使用 {motion['all_tracked_mean_cpu_percent_one_core_100']/100:.3f} 核。到位后端点补偿报告 no_motion=true，没有另发补偿运动。",
        '',
        f"近场事件窗口 {near['exact_window_sec']:.3f} 秒。17 次接触预检的请求至返回区间互不重叠，合计 {d['contact_precheck_wall_sum_sec']:.3f} 秒，占近场窗口 {100*d['contact_precheck_wall_fraction_of_near_field']:.2f}%；其中包括计算、服务传输和等待，不能叫做 CPU 时间。这里是 8 个候选级检查、9 次高度窗口重试；审计保存 16 个预检 plan ID，另有 1 次截止时间拒绝返回。",
        '',
        '近场主要进程 CPU（同一窗口，彼此可并发）为：']
    for r in near['processes'][:8]:lines.append(f"- {r['name']}：{r['cpu_sec']:.2f} 秒 CPU，平均 {r['mean_cpu_percent_one_core_100']:.2f}%。")
    stages=d['saved_prepare_stage_wall_sec']
    lines += ['',
        f"本次保存的算法准备阶段耗时：几何配准 {stages['geometry_registration']:.3f} 秒，桌面候选生成 {stages['tabletop_candidates']:.3f} 秒，参数读取 {stages['parameters']:.3f} 秒，远端输入准备 {stages['remote_input']:.3f} 秒。这些字段不足以构成完整关键路径，但明确不能把整个约 89 秒近场阶段归给配准。主要可定位的墙钟时间段是接触预检请求/返回；CPU 热点还包括未知目标感知、运动交接和任务状态进程，需各自内部剖析才能进一步定位具体函数。",
        '',
        f"活动窗口 SanStand 设备写入 {active['external_disk_delta']['write_bytes']/2**20:.2f} MiB；录制进程写入约 {next(r for r in active['processes'] if r['name']=='record_grasp_keyframes_readonly.py')['io_delta']['write_bytes']/2**20:.2f} MiB。设备数值包含其他写入者；rchar/wchar 是逻辑 I/O，不能等同磁盘流量。低的整体 I/O wait 不能证明所有请求都没有局部等待。",
        '',
        f"活动窗口 host PSI：CPU some 累积 {data['task_active_observed']['pressure']['cpu']['some']['stall_total_delta_sec']:.3f} 秒，memory some {data['task_active_observed']['pressure']['memory']['some']['stall_total_delta_sec']:.3f} 秒，I/O some {data['task_active_observed']['pressure']['io']['some']['stall_total_delta_sec']:.3f} 秒；不同压力项可以重叠，不能相加或归因给单个进程。SwapFree 净变化 {data['task_active_observed']['swap_used_delta_kib']} KiB。",
        '',
        '现有数据没有 ROS 回调队列长度、图像到达延迟、GUI 帧时间或远端服务端 CPU。schedstat 是调度运行/等待计数，wchan 仅主线程抽样；不能把它们命名为应用队列。结果不包含加速声明。',
        '',
        '术语补充：track 17/18/19 的三个原始候选由预测命令曲线终点筛查拒绝。逐个 gzip/object SHA 核验的回放都写明 validation_kind=terminal_endpoint_screen、checked_point_count=1、continuous_validation_executed=false、trajectory_point_index=16。日志旧前缀中的 continuous geometry 不表示本次已经扫完连续轨迹；这只否定该候选分支的终点，不否定其他 IK 分支。',
        '',
        '来源与计算方法见 result.json；压力计数见 pressure.json；每个进程的 CPU、RSS、物理/逻辑 I/O、主线程调度计数均保留在结构化结果中。',
    ]
    with (O/'REPORT.md').open('x') as f:f.write('\n'.join(lines)+'\n')
    print(json.dumps({'report':str(O/'REPORT.md'),'report_sha256':sha(O/'REPORT.md'),'pressure_sha256':sha(O/'pressure.json')}))

if __name__=='__main__':main()
