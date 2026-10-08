#!/usr/bin/env python3
import argparse,json,pathlib,statistics
p=argparse.ArgumentParser();p.add_argument('before');p.add_argument('after');a=p.parse_args();before=pathlib.Path(a.before);after=pathlib.Path(a.after)
b=json.loads((before/'correlation_summary.json').read_text());n=json.loads((after/'correlation_summary.json').read_text());bs=json.loads((before/'summary.json').read_text());ns=json.loads((after/'summary.json').read_text())
comparisons={}
for topic,old in b['segments']['whole']['streams'].items():
 new=n['segments']['whole']['streams'].get(topic,{})
 comparisons[topic]={'before':old,'after':new,'source_gap_max_change_sec':new.get('source_max_sec',0)-old.get('source_max_sec',0),'source_gaps_ge250ms_change':new.get('source_ge250ms',0)-old.get('source_ge250ms',0)}
identities={}
for role in ['camera','driver']:
 pid=ns['ready'][role+'_pid'];key='driver_start_ticks' if role=='driver' else None;expected=ns['ready'].get(key) if key else ns['ready']['camera_identity']['start_ticks']
 events=[c for c in ns['changes'] if c.get('event')=='process_identity' and c.get('role')==role]
 identities[role]={'pid':pid,'start_ticks':expected,'events':events,'identity_discontinuity_seen':any(c.get('error') or c.get('identity_matches') is False for c in events)}
pair=[]
for log in n.get('pair_unavailable_diagnostics_lower_bound',[]):
 text=log['message'];reason={}
 try:
  d=json.loads(text.split('Camera RGB-D pair unavailable:',1)[1].strip());reason={'pair_timing':d.get('pair_timing'),'host_wait':d.get('host_wait'),'result':d.get('result')}
 except (ValueError,IndexError):reason={'unparsed_message':text}
 pair.append({'source_stamp':log['source_stamp'],'arrival_wall':log['arrival_wall'],**reason})
resb=b['segments']['whole']['resource_deltas'];resn=n['segments']['whole']['resource_deltas']
native_gap_observations={}
for label,directory,run_summary in [('before',before,bs),('after',after,ns)]:
 camera_pid=run_summary['ready']['camera_pid'];gs=json.loads((directory/'correlation_detail.json').read_text());found=[]
 for gap in gs:
  if gap['kind']!='source' or gap['topic'] not in ['/supervisor/camera/color/image_raw','/alicia_d/accepted_joint_states']:continue
  found.append({'topic':gap['topic'],'gap_sec':gap['gap_sec'],'interval_wall':gap['wall_interval'],'camera_thread_samples':{key:{name:value for name,value in data.items() if name in ['sample_count','bracket_wall','runtime_ns_delta','runqueue_wait_ns_delta','timeslices_delta','state_counts','wchan_counts']} for key,data in gap['thread_deltas']['tasks'].items() if key.startswith(str(camera_pid)+'/')}})
 native_gap_observations[label]=found
host_evidence=json.loads((after/'host_memory_user_evidence.json').read_text()) if (after/'host_memory_user_evidence.json').exists() else None
comparison={'native_gap_observations':native_gap_observations,'host_memory_user_evidence':host_evidence,'select_interpretation':'wchan samples are instantaneous; nonzero timeslices show wakeups between samples. Backend waiting below one second is not evidence of a select timeout; the separate SDK source audit reports a fixed 5 s select timeout.','classification':'180 s pre-fix vs 180 s after background perception cache and camera pair-timing guard reload; observational comparison only','before_directory':str(before),'after_directory':str(after),'before_wall':[bs['ready']['ready_wall'],bs['ended_wall']],'after_wall':[ns['ready']['ready_wall'],ns['ended_wall']],'identities':identities,'stream_comparisons':comparisons,'resources':{'before':resb,'after':resn},'sampling':{'before':bs['sampling'],'after':ns['sampling']},'pair_unavailable_observed_log_count_lower_bound':len(pair),'pair_unavailable_diagnostics_lower_bound':pair,'pair_count_warning':'These warning logs are throttled and count only observed messages. Their count is a lower bound, not the total number of rejected pairs. Published-image gaps can include deliberate guard rejections.','limitations':['Two different time windows and a camera process restart; changes in workload, manual arm movement, memory pressure, observer scheduling and restart effects confound a causal interpretation.','Background perception cache and camera pair-timing guard both changed; their individual effects cannot be separated by this observation.','Source timestamps remain host-camera/host-parser timestamps, not measured wire receipt.','Native camera threads are do_select heuristic candidates with pinned start_ticks; this is not debugger-confirmed thread function identity.','No intervention into protected processes occurred during the observer run.'], 'human_reload_interval_utc':['2026-09-29T14:35:18.953Z','2026-09-29T14:35:27.740Z'],'deployment_artifacts':['/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/idle_detector_repair_20260929T142135Z/load_result.json','/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/rgbd_pair_time_repair_20260929T142631Z/load_result.json']}
(after/'baseline_comparison.json').write_text(json.dumps(comparison,indent=2)+'\n')
ns['observation_classification']='post_fix_passive_observation';ns['baseline_directory']=str(before);(after/'summary.json').write_text(json.dumps(ns,indent=2)+'\n')
lines=['# 修复后180秒被动观测与基线对照','',f"相机PID {identities['camera']['pid']} / start_ticks {identities['camera']['start_ticks']}；driver PID {identities['driver']['pid']} / start_ticks {identities['driver']['start_ticks']}。观测期间只读，两者身份异常：camera={identities['camera']['identity_discontinuity_seen']}，driver={identities['driver']['identity_discontinuity_seen']}。",'', '| 数据流 | 基线/修复后样本 | 最大源缺口：基线→修复后(s) | ≥250ms源缺口：基线→修复后 | 最大到达缺口：基线→修复后(s) |','|---|---:|---:|---:|---:|']
for topic,c in comparisons.items():
 o,v=c['before'],c['after'];lines.append(f"| {topic} | {o['samples']}/{v['samples']} | {o.get('source_max_sec',0):.6f} → {v.get('source_max_sec',0):.6f} | {o.get('source_ge250ms',0)} → {v.get('source_ge250ms',0)} | {o.get('receipt_max_sec',0):.6f} → {v.get('receipt_max_sec',0):.6f} |")
lines+=['',f'观测到pair unavailable限频日志{len(pair)}条，仅为拒绝事件下界，不能当总拒绝帧数；拒绝错配本身也可能增大图像发布间隔。','',f"资源计数基线→修复后：swap-in {resb['vmstat_deltas']['pswpin']}→{resn['vmstat_deltas']['pswpin']}页，swap-out {resb['vmstat_deltas']['pswpout']}→{resn['vmstat_deltas']['pswpout']}页，major fault {resb['vmstat_deltas']['pgmajfault']}→{resn['vmstat_deltas']['pgmajfault']}，direct扫描 {resb['vmstat_deltas']['pgscan_direct']}→{resn['vmstat_deltas']['pgscan_direct']}页。USB suspended计数增量 {resb['usb_power']['runtime_suspended_time_delta_ms']}→{resn['usb_power']['runtime_suspended_time_delta_ms']}ms。",'','这只是两个时间窗口的观察对照。后台感知缓存、相机pair防护及相机进程重载均已发生，其他负载/手动调姿/内存压力也可变化；不能据相关变化分离修复效果或锁定停顿根因。完整源/到达时间、线程身份/调度和资源计数见baseline_comparison.json及原始JSON。']
lines += ['', '原生线程采样需要结合timeslices：即便瞬时样本均为S/do_select，非零timeslices仍表示采样间曾获得CPU，不能宣称整段连续同一次select睡眠。Backend等待不足1秒不能称select超时；独立SDK源码审计报告select超时固定5秒。']
for label,observations in native_gap_observations.items():
 if not observations:continue
 gap=observations[0];lines += ['',f"{label}最大相关源缺口 {gap['gap_sec']:.6f}s 原生/主线程括区统计："]
 for key,data in gap['camera_thread_samples'].items():
  lines.append(f"- {key}: timeslices +{data.get('timeslices_delta','unknown')}, runtime {data.get('runtime_ns_delta',0)/1e6:.3f}ms, runqueue {data.get('runqueue_wait_ns_delta',0)/1e6:.3f}ms, wchan {data.get('wchan_counts',{})}")
if host_evidence:
 lines += ['', '用户Windows工作集快照：VMware(vmware-vmx)15.63GB，vmmemWSL0.69GB；当前已知主要占用是VMware，不能归因WSL吃满。用户快照并非自动同步观测，.wslconfig无输出也未确认实际上限；top进程工作集不能完整解释宿主物理内存。']
(after/'baseline_comparison.md').write_text('\n'.join(lines)+'\n')
print(json.dumps({'report':str(after/'baseline_comparison.md'),'streams':comparisons,'identities':identities,'pair_unavailable_observed_log_count_lower_bound':len(pair),'resources':{'before':resb,'after':resn},'sampling':comparison['sampling']},ensure_ascii=False))
