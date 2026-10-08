import contextlib,io,json,pathlib,runpy,sys
root=pathlib.Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/passive_camera_feedback_20260929T142715Z')
sys.argv=['analyze_passive_camera_feedback_observer.py',str(root)]
with contextlib.redirect_stdout(io.StringIO()):ns=runpy.run_path('.runtime/analyze_passive_camera_feedback_observer.py',run_name='__main__')
summary=ns['summary'];detail=ns['gaps'];res=ns['res_deltas'];task=ns['task_deltas'];stats=ns['stream_stats'];streams=ns['streams'];start=ns['start'];end=ns['end']
# Exact existing resource-sample boundaries surrounding the cluster; no new sampling.
cluster=[1790692094.1223135,1790692097.3978937]
cd=res(*cluster);td=task(*cluster);whole=res(start,end)
large=next(g for g in detail if g['kind']=='source' and g['topic']=='/supervisor/camera/color/image_raw')
other=[1790692141.019936,1790692152.8356059]
otherdata={'interval_wall':other,'resources':res(*other),'streams':{k:stats(v,*other) for k,v in streams.items() if k in ['/supervisor/camera/color/image_raw','/alicia_d/accepted_joint_states','/alicia_d/sdk_diagnostics:joint_06_00']}}
psi=cd['psi_total_delta_us'];duration=cd['bracket_wall'][1]-cd['bracket_wall'][0]
findings={
 'classification':'pre-fix baseline only; parent confirmed perception hot reload had not occurred during these 180 seconds',
 'observation_wall':[start,end],
 'slow_cluster':{'interval_wall':cluster,'resource_bracket':cd,'threads':td,'memory_full_psi_fraction_of_bracket_time':psi.get('memory/full',0)/1e6/duration,'swapout_fraction_of_whole':cd['vmstat_deltas']['pswpout']/whole['vmstat_deltas']['pswpout'],'kswapd_scan_fraction_of_whole':cd['vmstat_deltas']['pgscan_kswapd']/whole['vmstat_deltas']['pgscan_kswapd']},
 'largest_rgb_gap':large,
 'separate_majorfault_swapin_cluster':otherdata,
 'conclusions':[
  'The largest camera source gap is 0.958735 s; its logged SDK wait_for_frames took 0.929738 s with only 63.673 microseconds of camera-main thread CPU. This localizes the measured camera delay to waiting in the backend stage, not image postprocessing.',
  'Across 19 bracketing samples during that gap, camera native candidates 606061/606062 were always S/do_select, with zero additional runnable-queue wait and only 0.252/0.640 ms runtime. Camera main runnable wait was 0.827 ms. Sustained CPU runnable starvation of these sampled threads is not supported; select/futex snapshots cannot identify whether the dependency is device, USB, kernel or SDK internal.',
  'Driver source feedback gaps in this disturbance reached 0.292907 s and raw-joint parser gaps 0.292880 s. Driver threads accumulated only 0–2.270 ms runnable-queue wait over the maximum camera gap. Two spinner candidates ran for 227/205 ms; the driver was not globally unscheduled for the whole camera wait. Read/queue/poll timestamps remain unavailable.',
  'Measured Linux memory pressure is concentrated in the camera disturbance: swap-out and direct reclaim overlap, whereas major faults and swap-in predominantly occur later. This supports memory/reclaim pressure as a co-occurring condition without proving it caused USB/SDK waiting. Global counters do not identify the allocating or blocked process.',
  'USB device 4-1 runtime status was active in all 358 resource observations and suspended-time counter delta was zero for the full 179.5 s counter interval. These counters provide no evidence of device runtime autosuspend causing the gaps; they do not exclude other USB/host transport problems.',
  'The later interval with heavy swap-in/major faults must be compared independently. Memory-pressure markers do not imply a one-to-one camera stall relationship.',
  'The user-reported host screenshot (29.5/31.7 GB, 93%) is current context supplied after this baseline, not synchronized telemetry for historical gen17/gen18 or proof of WSL memory ceiling. No WSL limit or Windows host-memory time series was captured. nr_free_pages is Linux free pages, not MemAvailable.'
 ],
 'method_limits':['Observation contains possible manual arm movement and observer traffic.','Thread wchan and stat are sequential reads, not an atomic kernel snapshot.','50 ms changed to 100 ms at 1790692152.473534 after 20-sample mean proc-read cost exceeded 10 ms. Actual maximum sample interval was 190.454 ms and was not interpolated.','Resource timing is about 500 ms and intervals are bracketed by real samples. Compact daemon wakes differ from scans or direct compaction stalls.'],
 'host_memory_screenshot':{'source':'parent/user report, not sampled by this script','used_gb':29.5,'total_gb':31.7,'percent':93,'synchronized_to_observation':False,'wsl_memory_limit':'unknown'}
}
(root/'baseline_findings.json').write_text(json.dumps(findings,indent=2)+'\n')
vm=cd['vmstat_deltas'];ov=otherdata['resources']['vmstat_deltas']
lines=['# 修复前180秒被动观测结论','',f'窗口：{start:.6f}–{end:.6f}（2026-09-29 14:27:15.479–14:30:15.479 UTC）。父任务确认热换尚未发生；本数据仅作修复前基线。用户可能手动调姿。没有追加采样。','',
 '相机最大源缺口 0.958735 s，日志对应 wait_for_frames 0.929738 s；该等待内相机主线程 CPU 仅 63.673 μs。该缺口的19个线程采样中，候选原生线程606061/606062始终 S/do_select，runqueue等待增量均0、运行时间仅0.252/0.640 ms；主线程runqueue等待0.827 ms。证据不支持这些线程在可运行队列中持续遭CPU饥饿，符合阻塞等待；但不能仅凭select确定是设备、USB、内核或SDK内部依赖。','',
 '同窗 accepted 最大源间隔0.292907 s，raw-joint主机解析间隔0.292880 s。最大相机缺口内driver各线程runqueue等待仅0–2.270 ms，两个spinner候选运行227/205 ms；驱动并非整段未被调度。仍缺串口read/enqueue/poll时戳，不能断言线上无回包。','',
 f"慢等待资源括区 {cd['bracket_wall'][0]:.6f}–{cd['bracket_wall'][1]:.6f}（{duration:.3f}s）发生 swap-out {vm['pswpout']}页（全程{findings['slow_cluster']['swapout_fraction_of_whole']:.1%}）、kswapd扫描{vm['pgscan_kswapd']}页、direct扫描{vm['pgscan_direct']}页，allocstall_normal={vm['allocstall_normal']}、movable={vm['allocstall_movable']}；memory PSI some/full分别{psi['memory/some']/1000:.3f}/{psi['memory/full']/1000:.3f}ms。该括区major fault仅{vm['pgmajfault']}、swap-in {vm['pswpin']}；compact_migrate/free_scanned均{vm['compact_migrate_scanned']}/{vm['compact_free_scanned']}，compact_stall={vm['compact_stall']}，daemon_wake={vm['compact_daemon_wake']}。因此慢等待同窗有明确回收/换出压力，不能将其说成已确认的主缺页风暴或直接压缩阻塞。",'',
 f"另一个后续括区 {otherdata['resources']['bracket_wall'][0]:.6f}–{otherdata['resources']['bracket_wall'][1]:.6f} 有major fault {ov['pgmajfault']}、swap-in {ov['pswpin']}，同期RGB最大源间隔{otherdata['streams']['/supervisor/camera/color/image_raw']['source_max_sec']:.6f}s、accepted最大源间隔{otherdata['streams']['/alicia_d/accepted_joint_states']['source_max_sec']:.6f}s。换页/缺页与摄像头停顿并非一一对应，不能仅凭内存占用或相关计数锁定根因。",'',
 'USB4-1在358次资源采样中均为active，整个计数区间runtime_suspended_time增量0、runtime_active_time增量179500ms。本窗口不支持该设备runtime autosuspend导致停顿；其他USB/宿主传输问题仍未排除。','',
 '用户新截图29.5/31.7GB、93%是后续报告的当前宿主状态，不是gen17/18或本窗口的同步主机内存时间序列；WSL内存上限未知。采样的nr_free_pages仅是Linux空闲页，不是MemAvailable。','',
 '量化结论：阻塞等待已定位；持续CPU可运行队列饥饿与runtime autosuspend缺乏支持；Linux回收/换出压力确实与慢等待重合，但仍需独立验证因果。无需再次采样即可复核的逐线程与计数证据见baseline_findings.json。','',
 '采样实际median/p99/max=50.110/100.284/190.454ms；14:29:12.474因proc读取连续20次平均10.713ms而切至100ms。全部缺测间隔原样保留，未插补；资源采样约500ms，相关区间使用真实采样点括定。']
(root/'baseline_findings.md').write_text('\n'.join(lines)+'\n')
# Make baseline status explicit in the overview without creating a fictional reload event.
overview=root/'observation_report.md';overview.write_text('**Classification: pre-fix baseline only. Parent confirmed no hot reload occurred during this window.**\n\n'+overview.read_text())
print(json.dumps({'baseline_report':str(root/'baseline_findings.md'),'slow_cluster_resources':cd,'separate_majorfault_cluster':otherdata,'largest_gap':{k:v for k,v in large.items() if k not in ['thread_deltas','resource_deltas']}},ensure_ascii=False))
