import hashlib,json,math,pathlib,statistics
root=pathlib.Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
out=pathlib.Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/feedforward_speed_grasp_20260929T140844Z')
detail=json.loads((out/'feedback_stall_audit_gen17_gen18.json').read_text())
summary={'scope':'Offline read-only closed gen17/gen18 bag audit; no ROS calls, driver changes, restarts, motion, or hardware-port access.','source_timestamp_semantics':{'sdk_diagnostics':'ros::Time::now after CRC-valid packet dequeued, before acceptance filters; host_parse_time_not_device_time','accepted_joint_states':'ros::Time::now in joint parser after acquiring data_mutex, later used unchanged as accepted header.stamp','tf':'robot_state_publisher subscribes accepted_joint_states and carries accepted sample stamps; rate limiting can omit close samples','unknown':'No serial byte-read/frame-enqueue timestamps, queue depth, poll-write timing, callback start delay, or lock-wait timing are recorded. Actual lack of wire response versus read-thread starvation versus poll/parser delay is not identifiable from these bags.'},'cases':{}}
for label,d in detail.items():
 window=d['window']; raw=d['raw_joint_window']; acc=d['accepted_window']; lat=[]; mismatch=[]
 for i,r in enumerate(raw):
  ns=raw[i+1]['stamp'] if i+1<len(raw) else float('inf')
  matches=[a for a in acc if r['stamp']<=a['stamp']<ns]
  bb=bytes.fromhex(r['values']['payload_hex']);ws=[bb[i]+256*bb[i+1] for i in range(0,12,2)];pos=[(-180+min(4095,w)/4096*360)*math.pi/180 for w in ws]
  if len(matches)!=1 or max(abs(x-y) for x,y in zip(matches[0]['position'],pos))>1e-12: mismatch.append(r['stamp'])
  else:lat.append(matches[0]['stamp']-r['stamp'])
 ts={}
 for topic,v in d['window_summary'].items():
  ts[topic]={'count':v['n'],'largest_source_gaps':[{'seconds':g['gap_s'],'from':g['from']['stamp'],'to':g['to']['stamp']} for g in v['largest_gaps'][:3]],'max_source_to_bag_s':v['delivery_delay_max_s'],'sequence_gaps':len(v['seq_gaps'])}
 rejects=[r for r in d['window_logs'] if 'alicia_d_driver' in r['node'] and any(s in r['msg'].lower() for s in ['rejected','invalid','corrupt','serial read'])]
 summary['cases'][label]={'bag':d['path'],'window':window,'window_raw_joint_frames':len(raw),'window_accepted_joint_frames':len(acc),'decoded_pair_mismatches':mismatch,'raw_to_accepted_max_s':max(lat),'raw_to_accepted_median_s':statistics.median(lat),'whole_bag_raw_diagnostic_sequence_gaps':len(d['summary']['/alicia_d/sdk_diagnostics']['seq_gaps']),'whole_bag_accepted_sequence_gaps':len(d['summary']['/alicia_d/accepted_joint_states']['seq_gaps']),'whole_bag_driver_rejection_invalid_corrupt_logs':rejects,'topics':ts}
summary['findings']=[
 'Main accepted gaps are already present in raw host-parser diagnostics. Every raw joint frame in the selected windows is immediately accepted with identical decoded six-axis positions; feedback validation does not cause those gaps.',
 'Publisher sequence continuity excludes missing already-published raw/accepted messages within the bag stream as the explanation for the main source gaps; it does not establish serial wire receipt.',
 'Gen18 1790690987.070650..1790690987.699090 retains progressing raw/accepted source timestamps while both topics arrive at the recorder in a burst around 1790690987.72. This is additional downstream delivery/recording backlog distinct from source gaps.',
 'Gen17 /alicia_controller/state, a separate process, has a 0.566270 s source gap in the same disturbance; gen18 has a 0.228439 s source gap and >1 s delivery delay. The evidence is broader than a driver-only acceptance lock.',
 'Read thread does not acquire serial_mutex_ around normal reads; it takes queue_mutex_ briefly only after validating a frame. A direct write-mutex/read-thread deadlock is unsupported.',
 'Driver uses two shared spinner threads for polling, parser, heartbeat and command callbacks. pnh_.param(gripper_stroke_m) in three hot paths performs hasParam/getParam synchronously, including while publication holds data_mutex_ plus topic_mutex_. This is a concrete avoidable blocking risk, but its causal contribution to these historical stalls is unmeasured.',
 'Raw-to-accepted sub-1.5 ms intervals weaken a stall after dequeue awaiting data_mutex for these samples, but do not exclude prior callbacks delaying the parser/poller before dequeue. Accepted timestamp precedes publish_joint_state parameter access, so raw-to-accepted latency alone does not measure that call.'
]
summary['minimal_fix_candidate']={'locations':['alicia_d_driver_node.cpp:1405','alicia_d_driver_node.cpp:2271','alicia_d_driver_node.cpp:3205'],'change':'Remove synchronous parameter-server reads from locked hot paths. Preserve gripper_stroke_m units/default, private namespace and runtime-update behavior; a startup-only frozen member changes current semantics. Prewarm a cached subscription outside hot locks or refresh an atomic snapshot in a dedicated configuration context. Avoid assuming getParamCached never blocks on first access or unavailable value.','offline_validation':'Use an isolated ROS master and a fake/PTY serial responder, never the live arm. Exercise valid, missing, changed and deleted gripper_stroke_m values and meter/radian conversions; deliberately delay parameter service, then compare joint parser, heartbeat and poll cadence before/after. Existing provenance tests should still enforce accepted-only feedback and unchanged host-parse timestamp meaning.'}
summary['minimal_next_experiment']={'now_readonly':'Collect existing raw/accepted/TF header timestamps plus local subscriber arrival time; sample driver, controller, camera and ROS-master task schedstat and wchan at bounded low rate. Do not issue SDK queries, access serial, attach ptrace, or restart while manual adjustment continues. This can test scheduling/arrival backlog without claiming wire timing.','next_safe_driver_deployment':'Instrument a bounded monotonic record for completed CRC-valid frame enqueue, dequeue, queue depth/high-water, poll write start/end, timer expected/actual, parser lock wait, and parameter-call duration. Host read completion is still not MCU acquisition or wire arrival. A read/enqueue gap with ongoing polls differs from continuously enqueued frames waiting for callback drain. Schedule only after manual adjustment ends and root authorizes deployment.'}
summary['source_sha256']={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [root/'src/real-arm/alicia_d_driver/src/alicia_d_driver_node.cpp',root/'src/real-arm/alicia_d_driver/src/serial_communicator.cpp']}
(out/'feedback_stall_audit_gen17_gen18_summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
md='''# Gen17 / Gen18 feedback stall audit

离线只读审核；未连接 ROS、串口或调试器，未修改/重启驱动，未发送机械臂命令。

主反馈缺口已出现在原始 SDK **主机解析**时间轴，不能称为“线上没有回包”。Gen17 raw 缺口 0.645168 / 0.717393 秒，对应 accepted 0.645147 / 0.717495 秒；Gen18 raw 0.843356 秒，对应 accepted 0.842600 秒。两份正式 bag 的 raw diagnostic / accepted 发布序号全程连续。

选定窗口 Gen17 38/38、Gen18 53/53 个 raw 关节帧均找到六轴数值完全一致的 accepted 样本；raw→accepted 最大分别 1.440 ms / 0.993 ms。整个录制未见驱动 rejection / invalid / corrupt / serial-read 错误日志。主缺口不是过滤器连续拒绝；Gen18 的 E1 事件在 1790690987.726，晚于主缺口，并且该帧仍被接受。

另有交付积压：Gen18 1790690987.070650 至 1790690987.699090 的 raw/accepted 源时戳持续推进，却在约 1790690987.72 批量进入正式 bag；首帧 raw 延迟 645.6 ms，accepted 651.8 ms。因此“源未产生新解析样本”和“已带时戳消息晚到 recorder”需分开分析。独立 controller state 进程也同期发生源时戳缺口（Gen17 566.3 ms；Gen18 228.4 ms），还出现超过 1 秒的交付延迟，支持系统范围迟滞，不足以只归因于驱动。

时间语义：串口线程只把 CRC-valid payload 放入无时间戳队列；`sdk_diagnostics` 在出队后取时间，`accepted_joint_states` 在随后的 data_mutex 内取时间；TF 继承 accepted 时间戳，近邻样本可能受发布频率限制。现存 bag 缺少读完成、入队、poll 写入、队列深度、回调调度及锁等待指标，无法区分固件/USB未回复、读线程调度停顿、poll 停顿和排队未解析。

通用代码风险：普通串口读取不锁 serial_mutex，因此无证据支持写锁直接阻塞读线程；但全部 ROS timer 共用两个 spinner 线程。`alicia_d_driver_node.cpp` 原始审计行 1405、2271、3205 每次 `pnh_.param("gripper_stroke_m")`，本机模板执行 `hasParam` + `getParam`；其中关节发布持 data_mutex 与 topic_mutex。同步 master RPC 可让一个回调阻塞并拖住另一线程。这是可修缺陷，但历史 bag 未测量该调用耗时，不能宣称已确证为本次根因。raw→accepted 短延迟也不覆盖该时间戳之后的参数读取，更不覆盖出队之前的回调延后。

最小修复建议：把三处参数访问移出持锁热路径，保持私有参数名、默认 0.05 m、单位转换与运行时更新/删除语义。启动时永久冻结值会改变现有语义。可在热路径外预热缓存订阅或用专用配置上下文更新内存快照；首次/未命中的 getParamCached 仍需考虑阻塞。离线用隔离 master + 假串口/PTY 验证配置变化、缺失/删除默认值和转换，并对参数服务注入延迟，检查解析/heartbeat/poll cadence 及原有反馈来源测试。

最小后续观测：现在仅被动订阅现有 raw/accepted/TF，记录 source 与本机 arrival，并低频采样 driver/controller/camera/master 的 task schedstat/wchan；不主动查询 SDK、不打开串口、不 ptrace、不重启。要严格区分串口无响应和队列/调度，下一次安全部署再加入单调时钟的 CRC-valid frame enqueue/dequeue、队列深度、poll write、callback expected/actual 与参数/锁耗时。读完成时间仍不得标成 MCU 或线上采样时间。

详细数据：`feedback_stall_audit_gen17_gen18.json`；简要结构化结论：`feedback_stall_audit_gen17_gen18_summary.json`。正式 keyframe recorder 对图像采样，摄像头完整源缺口请以 scene bag / camera stage diagnostics 为准，不将正式 bag 的图像子采样间隔当完整摄像头停顿。
'''
(out/'feedback_stall_audit_gen17_gen18.md').write_text(md)
print(json.dumps({'summary':str(out/'feedback_stall_audit_gen17_gen18_summary.json'),'report':str(out/'feedback_stall_audit_gen17_gen18.md'),'cases':{k:{'frames':v['window_raw_joint_frames'],'max_raw_to_accepted_s':v['raw_to_accepted_max_s'],'mismatches':v['decoded_pair_mismatches']} for k,v in summary['cases'].items()}},ensure_ascii=False))
