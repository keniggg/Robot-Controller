import json,pathlib,statistics
p=pathlib.Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/passive_camera_feedback_20260929T143719Z')
topics=json.loads((p/'topics.json').read_text());summary=json.loads((p/'summary.json').read_text());logs=json.loads((p/'camera_logs.json').read_text());details=json.loads((p/'correlation_detail.json').read_text());ready=summary['ready']['ready_wall']
coverage={k:{'count':len(rows),'first_source_stamp':rows[0]['source_stamp'],'first_arrival_wall':rows[0]['arrival_wall'],'first_arrival_after_ready_sec':rows[0]['arrival_wall']-ready,'last_source_stamp':rows[-1]['source_stamp'],'last_arrival_wall':rows[-1]['arrival_wall'],'first_to_last_source_span_sec':rows[-1]['source_stamp']-rows[0]['source_stamp']} for k,rows in topics.items()}
accepted=topics['/alicia_d/accepted_joint_states'];raw=[r for r in topics['/alicia_d/sdk_diagnostics'] if any(x.get('command_function_hex')=='06 00' for x in r['diagnostics'])]
pairs=[]
for i,r in enumerate(raw):
 next_stamp=raw[i+1]['source_stamp'] if i+1<len(raw) else float('inf');matches=[v for v in accepted if r['source_stamp']<=v['source_stamp']<next_stamp]
 if len(matches)==1:
  a=matches[0];pairs.append({'raw':r,'accepted':a,'raw_to_accepted_stamp_sec':a['source_stamp']-r['source_stamp'],'raw_stamp_to_arrival_sec':r['arrival_wall']-r['source_stamp'],'accepted_stamp_to_arrival_sec':a['arrival_wall']-a['source_stamp']})
gap=next(g for g in details if g['topic']=='/alicia_d/accepted_joint_states' and g['kind']=='source')
gap_pairs=[v for v in pairs if gap['wall_interval'][0]-.01<=v['accepted']['source_stamp']<=gap['wall_interval'][1]+.01]
journal=[]
for line in (p/'camera_journal_1790692637_2643.jsonl').read_text().splitlines():
 r=json.loads(line);journal.append({'journal_realtime_stamp':int(r['__REALTIME_TIMESTAMP'])/1e6,'journal_monotonic_usec':r.get('__MONOTONIC_TIMESTAMP'),'pid':r.get('_PID'),'unit':r.get('_SYSTEMD_USER_UNIT'),'message':r.get('MESSAGE')})
backend=[]
for row in logs:
 if 'Camera backend read stages:' not in row['message']:continue
 d=json.loads(row['message'].split('Camera backend read stages:',1)[1]);backend.append({'source':'ROS /rosout_agg','log_source_stamp':row['source_stamp'],'log_arrival_wall':row['arrival_wall'],'diagnostic':d})
first_image=coverage['/supervisor/camera/color/image_raw']['first_source_stamp'];no_long_after={}
for topic,rows in topics.items():
 ss=[r for r in rows if r['source_stamp']>=first_image]
 if len(ss)>1:no_long_after[topic]={'samples':len(ss),'maximum_source_gap_sec':max(b['source_stamp']-a['source_stamp'] for a,b in zip(ss,ss[1:])),'source_gaps_ge250ms':sum(b['source_stamp']-a['source_stamp']>=.25 for a,b in zip(ss,ss[1:]))}
result={'outcome':'Pair-time guard rejected stale/mismatched RGB-D; SDK stall not resolved. Published-image max gap 102 ms applies only after first received image, not the entire 180 seconds.','coverage':coverage,'largest_accepted_gap':gap,'raw_accepted_pairs_near_gap':gap_pairs,'all_raw_accepted_pairs':{'count':len(pairs),'raw_joint_count':len(raw),'accepted_count':len(accepted),'raw_to_accepted_stamp_max_sec':max(x['raw_to_accepted_stamp_sec'] for x in pairs)},'camera_backend_diagnostics':backend,'journal_evidence':journal,'journal_source_note':'Existing systemd user journal for _PID645959 from epoch1790692637 through2643. Journal reception time and embedded ROS console time differ from ROS message header/observer arrival; duplicated diagnostics are corroboration, not extra events.','after_first_image_metrics':no_long_after,'poststamp_latency_interpretation':'raw at2640.7772326 -> accepted source stamp2640.7772534 is only20.7 us. Accepted observation at2640.9382539 is161.000 ms later. This is after the accepted source stamp and may include remaining publisher path, ROS transport or subscriber scheduling; not proven parse-to-acceptance/data_mutex delay.','causal_limits':['The camera backend wait overlaps the joint gap; it is not a driver-only disturbance with a proven-normal camera.','Initial 5.001877 s wait entered before observer readiness and exited inside it; the subsequent0.951308 s wait lies within the window.','Camera main/native runtime and timeslices are zero across the full largest joint-gap bracket. Instantaneous sleep snapshots and zero runqueue do not identify a device/host/SDK dependency.','Global memory/swap/reclaim counters overlap but do not identify the initiating process or prove causation.','USB runtime autosuspend remains unsupported by status/counters; other shared host/USB effects are not excluded.'], 'host_memory_user_evidence':json.loads((p/'host_memory_user_evidence.json').read_text())}
(p/'post_observation_boundary_audit.json').write_text(json.dumps(result,indent=2)+'\n')
md='''# 后观测启动边界与589ms反馈缺口复核

**不能宣称相机卡顿已解决，也不能把589ms缺口称为驱动独有。** 图像首样本是1790692641.429159（14:37:21.429Z），首次到达比观测ready晚1.741秒。102.244ms最大图像源间隔只覆盖这之后约178.205秒，不包含启动前后缺帧区间。

ROS与已有user journal相互印证：

- `wait_for_frames`从1790692635.460167至2640.462046，耗时5.001877秒，报`Frame didn't arrive within5000`。进入时间早于观测ready，退出在180秒窗口内。这里确有SDK等待超时；不要把它与基线0.93秒普通等待混称。
- 随后的2640.466406至2641.417716再等待0.951308秒，之后返回RGB/depth相差6204.032ms的帧对。防护以16.667ms阈值拒绝，随后2641.429159出现有效图像。防护有效阻止坏配对，但并未消除底层SDK等待。
- 180秒内观察到34条限频pair unavailable日志，只是拒绝事件下界，不是拒绝帧总数。

关节时间轴：accepted源2640.777253至2641.366791间隔589.537ms；对应raw 06/00解析间隔589.254ms，中间有温度raw，所以全部sdk诊断最大间隔只有428.652ms。前一个关节样本raw2640.777233→accepted源2640.777253仅20.7μs，raw到达延迟0.517ms；accepted却到2640.938254才到达，延迟161.000ms。这个161ms位于accepted源时戳之后，无法从现有数据区分剩余publisher代码、传输与subscriber调度；不是已证明的parse→acceptance/data_mutex等待。全程1619对raw/accepted的源时戳差最大1.750ms。

最大关节缺口的真实采样括区2640.773984–2641.396504共12点：camera main645959和native候选645993/645994全为睡眠，runtime、timeslices、runqueue增量均0。driver8790运行0.210ms/timeslices+4/runqueue0.359ms；8791运行2.393ms/+31/0.005ms；8792运行160.961ms/+30/0.154ms。其他driver线程同样继续有timeslices，因此不支持整个驱动CPU失调度，但也未定位串口、poll、队列或锁。

资源括区2640.216499–2641.801039（1.585秒）：swap-out2172页、kswapd扫描4573页、major fault1、swap-in0、direct reclaim/compact_stall0；memory PSI full185.496ms、IO PSI full112.482ms。USB4-1全active、suspended计数增量0。回收/换出压力同窗存在，但不是确定因果或USB传输无故障证明。

`camera_journal_1790692637_2643.jsonl`是读取已有systemd user journal的补证；`camera_logs.json`是ROS订阅消息。两者时戳口径不同，同一诊断不重复计数。没有追加实时采样或控制节点。

Windows用户快照主要已知工作集为VMware15.63GB，WSL0.69GB；补充WSL free-h为total15Gi、used2.7Gi、available12Gi、swap used0B。它们不是该缺口的同步自动遥测，不支持WSL吃满的归因；配置上限仍未确认。
'''
(p/'post_observation_boundary_audit.md').write_text(md)
# Correct every top-level overview without altering raw samples.
notice='**覆盖/结论修正：相机首样本前仍发生5.001877秒SDK超时及0.951308秒等待；102ms只描述首个有效图像之后的已采样间隔。不能据此宣称全部卡顿已解决或589ms反馈缺口为驱动独有。详见post_observation_boundary_audit.md。**\n\n'
for name in ['baseline_comparison.md','observation_report.md']:
 f=p/name;s=f.read_text();f.write_text(notice+s)
for name in ['summary.json','correlation_summary.json','baseline_comparison.json']:
 f=p/name;d=json.loads(f.read_text());d['coverage']=coverage;d['boundary_audit']='post_observation_boundary_audit.json';d['whole_window_stall_resolved']=False;d['coverage_warning']=notice.strip();f.write_text(json.dumps(d,indent=2)+'\n')
print(json.dumps({'report':str(p/'post_observation_boundary_audit.md'),'camera_coverage':coverage['/supervisor/camera/color/image_raw'],'raw_accepted':result['all_raw_accepted_pairs'],'after_first_image_metrics':no_long_after,'journal_entries':len(journal)},ensure_ascii=False))
