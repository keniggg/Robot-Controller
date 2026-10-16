# GEN21：执行确认前缀的感知交接

本次用户请求按当前已对准目标执行抓取，持续授权仍有效。当前现场确认来自
`call_99d156ff3dc44ab59f1565c7c6ef4263:0`，原文为“已停止操作，双手及前臂均已移到活动范围之外”。
所有实验与测试输出保存在SanStand，不上传GitHub。

## 原问题与证据

GEN20 已完成6024项回归并部署。录制
`grasp_records/20261014T083439.833382Z_f726b19891e949bb922afb59200ffc8e`
结果为失败，未夹持目标。唯一动作是3秒执行确认前缀，不是完整第一观察位轨迹。

控制参考同步开始于1791966904.019秒。随后第3轴相对初始静止标记变化约0.00767rad，
超过0.004rad静止判据。相机源1791966903941551923在post_refine阶段被判
`initial_station_binding_changed`，1791966904.162秒首次宣布target_lost；
该帧年龄约0.220秒，未超过0.8秒期限。前缀目标1791966904.778秒才发送，
1791966907.829秒成功返回。任务原计划锁延迟了撤权回调；进入正式session的ACK等待前，
owner检查发现已撤权，返回`OBSERVE_RETURN_ACK_OWNER_CHANGED_OR_REVOKED`，不是ACK超时。

因此首因是bootstrap绕过普通观察的量测交接，不能用延长ACK或来源期限解决。
既有origin捕获要求执行已确认、SDK/accepted配对及300ms静止窗口，无法直接挪到
建立这些条件的bootstrap之前。

## 修复设计

在原参考同步和唯一前缀服务之前，建立独立`actuation_bootstrap`身份交接，
复用原executing→requalifying→qualified协议、双参与者ACK和三帧同目标量测。
它绑定当前selection、已冻结计划及摘要、目标身份、控制epoch、原始来源与原计划绝对期限；
不创建正式观察session、origin证明或接触权限，不计观察次数，也不续期。

感知端在ACK之前验证原静止标记和新鲜来源，然后退休旧站点及处理中旧帧。
小动作后任务重新等待实际静止窗口；感知端取得三帧，任务与规划端核验同一来源列表和最终实际对象。
新到位标记接替旧站点，之后再走原origin捕获及正式第一观察路径。
原始来源的120秒绝对期限不续期。qualified后保留新到位标记，规划端保持暂停直至正式session接续。

真实目标退休时，仅撤销这次bootstrap所属的执行计划，使原rich-plan撤销消息到达
运动网关的独立提交守卫。保留原计划锁、路径/碰撞检查、0.004rad静止判据和来源期限。

离线真实回调及双锁时序测试还复现了旧positive在模式锁前及时到达、前缀返回后才处理而被误判过期
的情况。该边界与主修复共同验证：只允许退休已被交接取代且入口及时的同目标旧帧，
不刷新旧帧时间、不忽略真实失效或新源变化。

## 当前状态

GEN21实际工作树组合回归完成：6149项通过，0失败、0错误、0跳过；937份源码前后哈希一致。
证据位于`ros_log/full_latest_gen18_restart_20261014T012216Z/verification/gen21_combined_final_r2/`。
独立复核`verification/gen21_independent_revocation/review.json`无阻断项，包含真实撤权到网关、
正式session接续、延迟旧帧隔离的验证。被中断的组合r1保留为未通过记录，不计入成功结论。

最终发布为同一会话`verification/gen21_release_manifest.json`，SHA256
`10dc46f8ed511cf8c96ac747c42906b82aa4ec8b280d9724ac38bb3dfc9d2649`。
`gen21_deployment/result.json`确认三个计算节点重载成功，八个硬件/网关/相机/上位机进程身份保持。
原轮失败及手动恢复回执保留在
`ros_log/full_latest_gen18_restart_20261014T012216Z/gen20_attempt_1_diagnosis.json`。
最终发布、部署、录制和实物结果须分别更新，不能以测试通过代替抓取成功。

## GEN21部署后首轮实测

新现场回复`call_82e8f7daa8da493caab9c4d0f2bbb578:0`确认“双手及前臂已移到整个活动范围之外”。
录制`grasp_records/20261014T092930.740068Z_ded6108f8ba34a848a203c77f2c88628`已正常封口，结果为失败，未夹持。
1791970213.688741秒第一观察位动作返回成功；1791970215.923010秒三帧重确认与qualified双ACK完成。
本轮开始时执行状态已CONFIRMED，因此不能把本轮通过第一观察位当作新bootstrap路径实机覆盖。

1791970216.432764秒规划端报告指定到位帧RGB-D不可用；1791970220.539511秒感知端首先报告
`exact_rgbd_stream_stale`，随后`source_frame_stale_or_future`；1791970221.454699秒任务以
`TARGET_LOST: station_task_owner_changed`结束。后者是下游结果，尚不能作为首因。
正在核对到位后精确帧缓存、回调延迟与站点等待协议；未改变帧有效期或碰撞约束。
`gen21_attempt_1_after_manual/result.json`已核验任务空闲和上位机自身手动确认，机械臂保持到位姿态。
