# GEN22：到位后输入延迟与精确帧交接

当前任务仍是按持续授权完成实物抓取。GEN21首轮到达第一观察位后失败，未夹持。
本文件记录后续修复，不能把离线测试或机械臂到位记作实物抓取成功。

## 原始证据

录制：`grasp_records/20261014T092930.740068Z_ded6108f8ba34a848a203c77f2c88628`。
会话：`ros_log/full_latest_gen18_restart_20261014T012216Z`（下称N）。
完整结果及阶段回执：`N/gen21_attempt_1_diagnosis.json`。
相机与回调时间线：`N/verification/gen22_camera_gap_evidence/findings.json`、
`qualified_final_source_receipts.json`与`statuses_full.json`。

- 1791970213.688741秒第一观察位动作返回成功；215.923010秒qualified双ACK完成。
- 指定源1791970215573863267的RGB、深度、掩膜和对象均在bag中有记录，但不能用录制器接收时间推定规划端接收时间。
- 216.432764秒规划端报该精确源RGB-D不可用；220.539370298秒感知端首次报`exact_rgbd_stream_stale`。
- 在首次失锁前2.463毫秒，源1791970218329827070已经进入感知输入槽，其实际源龄为2.207秒，不能作为新鲜几何。
- 相机SDK同时存在0.868秒连续彩色帧间隔；应用侧另有更长的到达延迟，不能只归咎于相机或USB。
- qualified至失败期间165条接受/控制反馈各轴峰峰值为0；新鲜RGB-D在首次失锁后约0.645秒恢复，但旧流程已永久锁死该generation。
- 221.454699秒`station_task_owner_changed`是后续终态，不是已证实的首次故障。

## 已复现的问题及修复边界

1. 无输入检查先读取`source=None`，经过站点审核后仍使用旧快照，能在新帧已进入输入槽时永久撤销目标。最终判定须在输入锁内重新核对当前槽与接收时间，保持原源时间和期限。
2. 独立反例发现SDK反馈也可能在第一次站点审核失败后更新。仅该次真实反馈新鲜度失败允许在原身份、站点及全部原期限内撤销旧的失败判据；下一tick必须重新走完整审核，不能因此发布几何或动作。
3. 真实远端回调实验确认：量测qualified ACK不要求RGB-D已在其缓存齐全。首次精确帧重建失败后，原流程没有晚到同源帧的接续入口。已实现有界、同源、同session的接续；保留原精确样本准入与注册规则，不改源戳，不换帧，不因此生成执行权限。到位图像等待期间，三topic READY登记仅保留原站点身份；typed pending仍阻止规划，原帧完成历史表面注册后也必须等新鲜量测才能恢复规划。
4. 恢复后的独立复核又复现了最终提交竞态：context审核可能等待，其间phase、任务、session或配置已改变，旧实现只重查时钟，仍能登记旧阶段。真实线程反例同时证实phase及任务回调有锁外写入。此问题须统一失效入口与最终提交边界，并把耗时context审核放在提交锁外；不能只增加一次不具原子性的检查。

输入旧帧仍按原0.8秒几何新鲜度拒绝。既有站点身份等待的5秒上界以及phase、session和monotonic截止不续期。
真实姿态变化、控制epoch变化、手动模式、任务终止、身份失效和到期不得靠等待分支恢复旧权限。

## 验证与当前状态

输入竞态原反例先红后绿，保存在`N/verification/gen22_ingress_recheck/`；SDK-only独立红证据保存在
`N/verification/gen22_independent_scope_diagnostic/`。修复的阶段性回归139项通过、SDK独立反例12项通过，
后续补充及最终冻结结果另行记录，不能混用不同源码哈希的阶段结果作为发布回执。
远端精确RGB-D实验在`N/verification/gen22_qualified_rgbd_remote/`，初次夹具错误也保留并说明。

用户暂停时，`r10_related`完成738项后被SIGINT中断，不能记为全套通过。恢复后补充的RGB-D缓存与等待身份测试98项通过。
最终提交独立红版在`N/verification/gen22_independent_commit_boundary_red/`，8项中6失败、2通过；
组合`N/verification/gen22_combined_final_r1/`因此在完成1756项后主动中止，发布验收不使用该中断结果。
修复后独立15项全部通过，证据为`N/verification/gen22_independent_commit_boundary_green_r1/receipt.json`。
新增retry的context检查移出最终提交锁；原有selection查询仍保留，不能称整段提交完全没有RPC。
phase、任务及手动接管入口与最终本地提交审核使用相同锁序，session、配置、epoch、selection以及全部原期限均须保持有效。
最终组合`N/verification/gen22_combined_final_r2/`覆盖941份源码/资源清单、204个测试文件，6341项全部通过，零失败、错误或跳过；前后哈希一致，耗时473.96秒。
独立最终复核为`N/verification/gen22_final_independent_review/review.json`，阻断项为空。
发布清单为`N/verification/gen22_release_manifest.json`，SHA256为`a99e400819346d3552f37eb2923fae870e6a3826d57451657a52ea25cc3c6da2`。

GEN22已完成全量冷启动部署。用户明确要求重新启动各节点及上位机，启动前主机核验没有旧ROS/GUI进程；
新冷启动会话为`ros_log/full_latest_gen22_restart_20261015T003700Z`（UTC目录时间，用户当地仍为10月14日）。
串口、驱动二进制、桌面和冻结仿真依赖已核验。WSL初次失联，用户重新连接后health返回200、模型loaded=true、CUDA可用。
新会话`cold_restart_acceptance/result.json`核验11个核心节点全部为新进程、源码与发布一致，
驱动、关节反馈、控制器、GUI窗口、配套本地仿真及手眼配置通过；`idle_gui_camera/result.json`核验新鲜RGB-D及GUI自身的手动确认。
使用重启后的现场与真实反馈，没有沿用旧姿态或旧控制epoch。软件温度查询和关节失能始终未执行。

## 部署后首轮实测

录制为`grasp_records/20261015T005512.865050Z_bd6331d2d17f45d199fd3fa9a25dacbd`。
用户重新对准并确认手臂移开后，当前图像目标可见、20份关节反馈无变化，本轮在持续授权下执行。
第一观察轨迹成功，到位量测qualified；后续生成接触方案`26ec14e175bb95b3369dd05c`。
这证明实际流程进入了接触规划，不能据此声称所有新重试分支均已在实机触发。

接近轨迹发送前，任务因`CONTACT_STATION_HANDOFF_FAILED: CONTACT_STATION_ACK_TIMEOUT`失败；
只有第一条观察轨迹，没有接近轨迹或闭爪。录制完整封口，并恢复手动空闲。
新会话`gen22_attempt_1_diagnosis.json`保存详细结果和证据哈希。

接触交接的prepare有remote正ACK，却没有unknown ACK；整个等待窗口仍有正常感知输出。
实际ROS参数`selection.stamp_ns`为十进制字符串。`contact_station_retirement.parse_request`只在等价比较时归一化，
随后仍以原字符串与整数做时间比较，触发TypeError；感知回调此时尚未赋值request，捕获后静默返回。
相同实录请求由remote/task的整数形式传入可通过；原参数形式可稳定复现0 ACK。
随后仅修正该本地参数类型边界，并验证完整prepare/retire交接，没有延长超时。

## GEN23：接触交接参数类型修复

`contact_station_retirement.py`统一解析外部selection后再执行后续比较；请求值和摘要不变，800毫秒时限、身份、阶段和双ACK规则不变。
本轮实际输入回放原先producer抛TypeError、callback无ACK，修复后producer、remote、task均通过，producer回调产生正ACK。
独立18项测试原先4失败、14通过，修复后全绿；覆盖真实producer上下文的prepare/retire、参数整数/字符串等价、非法类型及所有权变化拒绝。

以GEN22的941份清单为基线，证明仅该生产文件变化，另加1个独立测试文件；本次6文件差量共230项全部通过，零失败、错误或跳过，942份清单前后哈希一致。
6341项是此前GEN22完整回归，本次只重新运行230项相关回归，两者分列记录，不混称整套重复通过。
新会话`verification/gen23_contact_station_delta_r1/result.json`、`verification/gen23_final_independent_review/review.json`保存验证与复核。
发布清单`verification/gen23_release_manifest.json`的SHA256为`e773a0ad15a60febfd57c9eec572119b66782a8aac953066deeab7b1b58fe214`。
`gen23_deployment/result.json`确认三个计算节点完成重载、八个硬件及GUI进程身份保持，未发运动或失能命令。

重载后当前图像中的纸盒位于画面下方，新的初始目标识别返回`no_center_instance`，未发起新一轮。
已请求用户重新对准画面中央并确认停止操作、双手前臂移开；后续须重新采集现场与姿态，不能沿用此次已经过期的抓取前快照。


GEN23 第 1 轮已在用户重新对准中央并确认停止操作、双手前臂移开后启动（确认 `call_6dc475c2dd504a0f805de7a45a61ad91:0`）。本轮前检查 20 帧实测姿态无变化，942 份源文件及已部署进程核验一致；使用 `gen23_attempt_1_launch_scene` 新图像与原持续授权 revision 18。运行记录：`/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261015T011001.427571Z_410f5bbeb21b457d8baae7748221688b`。正在执行，结果及实物抓取状态尚未确认。


GEN23 第 1 轮最终失败，已封口并恢复手动空闲。第一观察动作成功到位；尚无接触交接、接近轨迹或闭爪，不能据此验证 GEN23 交接修复的实机通过。新失败是 `station_support_gap_expired_or_future`：三张连续帧的支持面空间覆盖不足，原父 READY source 为 `1792026650231112718`，gap 截止为 `1792026651031112718`；下一帧 source 为 `1792026650963288545`，到位资格检查结束于 `1792026651031848669`，只越过父截止 `735951 ns`，该帧自身仍新鲜且原固定 5 秒身份等待未过期。实测六关节无变化。正在修复旧几何有效期与等待独立新测量期限混用；不沿用旧几何执行动作。精确证据位于新会话 `verification/gen23_attempt1_support_gap_evidence/findings.json`；失败摘要 `gen23_attempt_1_diagnosis.json`。


用户随后明确「先暂停」。已停止后续修复/验证与抓取启动，持续控制为 paused revision 19；GEN24 尚未组合验证或部署。已保留当前源码、已有离线证据及未完成事项，暂停检查点位于新会话 `gen24_user_pause_checkpoint/checkpoint.json`；节点与上位机保留。此前新对准确认被后续暂停指令覆盖，不据此启动抓取，等待用户唤醒。


用户发出「继续处理」后，已恢复至 control active revision 20。暂停检查点的10份文件校验一致；此前独立 green_r1 实际为12项中3失败、9通过，未将其记作通过。两项为空/陈旧输入早退时支持面 gap 的消息来源与消费者校验不匹配；另一项测试用篡改摘要模拟矛盾无效，改用真实8mm支持面偏移验证拒绝。正在完成这些边界修复及旧测试合同更新后，再运行全部组合范围。恢复时图像目标中央可见、任务空闲且手动模式，尚未启动抓取。


## GEN24：当前支持面恢复与固定身份等待

本轮同时处理两处导致同一观察位失败的原因。第一处在 `segment_with_support_fallback`：当前独立桌面审计已合格，但旧参考平面的2mm带仅覆盖窄条，`insufficient_support_spatial_coverage`没有进入既有当前面重算分支。只对当前审计明确合格的这种错误开放原重算路径；仍使用原支持面一致性、目标关联与歧义检查。上一轮28帧的原深度/原TF回放中，当前审计28帧全合格，原实际分割17帧通过、11帧误拒，修复后28帧均通过。此结果仅证明分割与原几何关联通过，不等同完整RGB验证、可执行规划或实物夹持成功。

第二处将已经注册的站位支持面缺帧等待限制在父几何的0.8秒期限内。修复后等待使用原固定身份5秒及更短phase/session/monotonic上限；旧几何0.8秒过期后仍不允许执行，必须等到新鲜完整测量，并通过真实object/mask/status交接才恢复。新帧自身0.8秒期限不变，连续pending不续期。无新帧或旧帧早退时只发布对应原来源的计算等待，不能把另一帧的支持面回执贴在其上。

涉及4生产文件，4份旧测试按新等待合同更新，新增3份测试与1份精确录制时间fixture。针对性7项、99项、24项以及几何5项已分别通过；这些计数有交叠，不相加作为完整覆盖总数。完整组合验证正在 `verification/gen24_combined_r1` 运行，946份源码/资源清单、211个测试文件，尚未部署。温度查询与失能命令均未执行。


用户最新指令为「继续处理，但是修复完成后先不执行，等我唤醒你」。持续抓取控制已设为 paused revision 21，仅继续修复、离线验证与空闲部署，不启动抓取。

GEN24 全组合 `verification/gen24_combined_r1` 实际结果为 6449 项通过、零失败/错误、1 项旧录制 fixture 路径缺失而跳过；946份源码/资源前后哈希一致。原完整回归结果不改写。该fixture的原 `source.json` 由本地trace库按原路径及SHA256核验取出，原 `depth.npy` 在SanStand的SHA与历史迁移清单一致；通过仅指向外盘的兼容软链接补齐旧路径，没有制造新样本或下载云端。`verification/gen24_recorded_fixture_completion_r3` 精确补跑原唯一节点通过，因此共6450个独立用例已完成，完整与补充结果分列保存。此前补充入口的测试rootdir元数据不一致和Python3.8软链接操作错误也保留原记录，未改写为通过。尚未实机验证GEN24整轮抓取。


GEN24 已完成发布和空闲部署。发布清单SHA256 `0e866816517069818540086ff900682115074a9eeaa459a990d7d0e4a4d21a8d`；`gen24_deployment/result.json` 确认3计算节点新进程、8硬件及GUI进程保持；部署后20份实测关节反馈最大变化0，IDLE/inactive/manual=true，目标ready:tracked。原修复后暂停要求已落实，未启动抓取。随后用户明确「现在授权继续进行抓取任务」，控制恢复active revision 22，正重新采集画面与姿态；不能沿用过期检查直接执行。


## GEN24 实测结果与 GEN25 执行期限问题

用户在 `call_9d4b28f1bc374048aa674ac4a0607ba8:0` 确认已停止操作、双手及前臂均已移出活动范围；新的清场图像及姿态检查通过后，active revision 22 下启动录制 `grasp_records/20261015T030730.930818Z_2ff7836aaadc4172980c58ba2741d935`。本轮已失败封口：唯一实际观察动作成功，随后150帧持续READY，当前支持面审计全通过。prepare/retire两阶段四个ACK均成功且摘要匹配，最大延迟6.163毫秒。尚无第二动作或闭爪；没有实物抓取成功证据。独立证据位于 `verification/gen24_attempt1_independent_evidence/findings.json`。此轮未出现support gap，不能将它表述为实机覆盖了所有gap等待边界。

选中接触方案 `ebf0466d14712c960fefe190` 已通过几何、可达性与路径检查。实际终止为 `CONTACT_PREGRASP_EXECUTION_REJECTED / CONTACT_PREGRASP_INSUFFICIENT_EXECUTION_TIME`。最终重定时轨迹135.853162666秒，控制器到位宽限8秒，附加余量2秒；Joint6约1.793弧度净行程在0.02rad/s上限下仅最低行程就需89.66秒。名义预抓取服务错误复用了局部补偿的60秒总期限，原600秒观察会话当时仍剩约520秒。问题不是再次缺少高度/深度。将修复此服务计算期限与已认证原会话运动期限混用，不提高限速，不改变碰撞、实际反馈与局部补偿约束。精确取证在 `verification/gen25_execution_budget_evidence/finding.json`。

用户质疑初始视角和算法合理性后，另做只读算法审计：初始已估计57.18×36.60×20.90毫米目标、1264个测点、99.73%桌面内点率；首个补视触发来自当前未知物体模型的双侧实测接触带不足，并非完全没有深度或距离硬门。该模型不根据盒顶推断隐藏侧面，而且在完成全部接触候选搜索前即可进入补视，因此不能声称初始所有直接抓取方案已证实不可行。近场选中方案需要34.29毫米开口、最终桌面余隙13.47毫米；被拒备选的众数原因不能冒充最终任务失败原因。完整审计 `verification/gen25_algorithm_information_review/findings.json`。

失败后 `gen24_attempt_1_after_manual/result.json` 确认任务/动作空闲及GUI自身手动ACK。后续只读采集 `gen24_attempt_1_after_scene` 发现实测姿态已不同于本轮观察终点；下一轮不得沿用旧对准回执。当前继续修复，持续授权仍有效，未自动再次启动。


GEN25 修复已冻结：仅修改 `contact_pregrasp_gateway.py`，SHA256 `3b99a22d4087bb7b65854ccc2dfc2443589c4e07f5f4ec1772a9ded0c042cfb5`。计算与本地补偿仍为60秒；完整名义轨迹仅在真实原观察会话内使用固定执行截止。完整路径校验、before_execute与最终before_send分别核对最终轨迹耗时、控制器宽限及余量；计算期间共享context仍受原计算期限限制，真正执行后使用原会话期限持续校验。真实手动/epoch/撤销、期限不足和时钟回退均拒绝，不续期、不改变速度。

85项定向回归与19项独立回归分别通过。独立红版16项中12失败、4通过；实现者最初own测试发生helper命名setup错误，不当作修复因果证据。完整组合 `verification/gen25_combined_r1` 正在运行，949份源码/资源、216个测试文件。交接 `verification/gen25_contact_budget_delivery/delivery.json`。所有离线执行均隔离真实ROS，未发运动命令。

后续流程审查未发现另一个必然使135秒名义轨迹超时的任务层60秒限制；原phase只约束首次接触admission，已冻结任务仍由原600秒会话约束。接近与接触的局部60秒属于新短段，其最终时长仍须在实际规划后核对。无触觉且禁止抬升的当前配置中，闭爪后可能终态HOLDING/GRASP_HOLD_UNVERIFIED，需要另外确认真实夹持，不能把闭合命令完成当实物成功，也不自动抬升验证。详见 `verification/gen25_algorithm_information_review/downstream_timing_review.json`。


GEN25 完整组合实际6586项全部通过、0失败/错误/跳过；949份源码/资源哈希一致，216个测试文件完整覆盖。发布清单 `verification/gen25_release_manifest.json` SHA256 `233ddfcb73475b22b1528f847eaa9c1b283b97aeef1e5abbae7966b8df6c441e`，独立审查SHA `0ac0e7b892d769c7c69265ab37a514838f45d0607f8cd01ea57782ee74fc2d6e`。空闲部署 `gen25_deployment/result.json` 通过，3个计算节点及原core自有motion_gateway更新，7个硬件/GUI进程保持；没有启动新抓取。

## GEN26：用户最新策略授权（实现中，尚未部署）

用户明确：规则盒顶轮廓足以推断完整盒形时应直接抓取；只有看见一侧、轮廓不完整、无法可靠推断、不规则目标或接近夹爪物理开口极限时补视。随后补充：只有窄边能放入夹爪且需要大幅转腕时，应针对姿态导致的定位/轮廓误差补充视角；最后明确当前必须有双侧实测接触带的条件可以先关闭，后续需要时再开启。

按此修改 `unknown_require_measured_bilateral_contact` 为可配置且当前默认false；true恢复实测双侧途径。合格完整盒顶采用明确的规则直棱柱模型，保留同源深度/轮廓/标定/桌面/身份的来源哈希与经验边缘、角度、高度误差，不把推断侧面冒充实测点。夹窄边时按角度不确定区间的最大宽度投影加入开口预算；转腕角度记入诊断与目标视角选择，不虚构未经标定的角度到毫米误差模型。物理开口、碰撞、桌面间隙和真实执行反馈继续验证。当前在联合实现模型、候选、最终接触证据及直接任务固定期限，尚未允许下一轮启动。

原始初始RGB-D与mask精确导出在 `verification/gen26_box_inference_design`，source_manifest SHA `655674b48c47c3b00ad852ad8fa8457ccad0ebd731163767531bee0b4642bf21`。顶面视觉完整而深度角点存在小片缺测，须分别处理，不以深度缺角直接断言可见轮廓不完整，也不凭视觉完整制造深度测量。


用户最新要求「先暂停下」。已暂停修复、验证、部署和下一轮抓取，持续控制paused revision 23。GEN26仍为未部署源码；本轮未发新运动。已完成的有限回归分列保存在外盘，包括199项几何、112项remote、87项独立及57项task/close（有重叠，不合计为完整覆盖）。完整组合尚未运行、尚未封版。真实初始RGB-D已能推断完整盒顶；窄边38.517mm加经验宽度不确定度12.493mm和实际fit 4mm合计55.010mm，仍需精度补视，未声称实物成功。暂停源码哈希、现场空闲核验及未完成项见新会话gen26_user_pause_checkpoint/checkpoint.json。等待用户唤醒后继续。

用户随后明确「你接着继续执行」。恢复核查中，暂停清单27份源码与15份证据回执哈希均匹配；SanStand宿主机挂载可写，沙箱看到的只读属性不是磁盘故障。新的只读现场采集`gen26_resume_scene_r1`确认IDLE/inactive/manual=true；发现画面下沿有手部后，用户在`call_f699a84ac634494d9ae4076d619ebdaa:0`明确回复「已停止调整，双手及前臂已移开」。该确认不能代替最终启动前的新图像和实测关节检查。

恢复后的独立审查发现两个待完成边界。其一，已接受的多视图配准可有小幅支持面法向/位移差异；原推断接触分支把模型的配准后支持面与原始实测碰撞平面混同，0.1°法向差和0.5毫米位移的真实注册用例均能复现误拒。修复须保持模型变换证明，并用原始实测面约束物理桌面碰撞，不能简单放宽平面比较。其二，直接任务的固定总期限只在名义预抓取中绑定，后续局部接近/闭爪仍需继承原截止时间；保留每段60秒局部预算，不得以新服务调用续期。两处先保存独立失败用例，再进行针对验证和完整组合回归；此处仍未声称部署或实物夹持成功。

用户进一步明确「我接受所有授权，你不用再等我给出明确的指令」。后续在既有修复、验证、部署与持续抓取范围内直接推进，不再逐项请求确认；现场新状态、原动作检查及可回溯记录仍按实际结果核验。

## GEN26 完整验证、封存与部署（2026-10-15 UTC）

本节及下节的 `N` 均指当前会话 `ros_log/full_latest_gen22_restart_20261015T003700Z`，外盘根为 `/media/zhuyupei/SanStand/实验记录/Robot-Controller`。前文「尚未部署」为当时检查点状态，现已完成下述独立回归、封存与空闲部署；仍未确认实物夹持成功。

配准支持面的独立原始反例3项均失败，修复后8项全部通过，证据为 `N/verification/gen26_registered_inference_independent_{red,green_r1}`。推断模型保留已注册坐标与来源变换，物理碰撞仍绑定原始当前实测支持面；没有用放宽阈值代替坐标/来源区分。直接任务期限的原始20项失败中，18项复现真实期限缺口，另2项原先已经因手动接管拒绝、只是返回文字不符，不算新增缺陷。修复将任务开始时的固定600秒截止传递至后续运动与闭爪，每段局部预算仍受其自身60秒及原总截止共同限制；首次active时间保持真实任务开始时刻，heartbeat、重试、撤销后新局部调用均不续期。仅无匹配观察来源的初始直接接触继承该直接任务期限，既有观察路径保持原会话规则。相关最终47项独立边界测试及此前相同生产源码的420项相关回归均通过；阶段性浮点时间夹具及旧冻结夹具失败按原结果保留，不合计为完整覆盖总数。

完整组合第一轮 `N/verification/gen26_combined_r1/result.json` 为6993项、4失败、0错误/跳过。四处旧remote测试没有隔离新增策略参数读取，离线测试误连ROS参数服务；只补对应 `get_param=False` 夹具及精确调用断言，真实推断与下游断言保留，没有再次修改生产源码。相关166项通过后，完整第二轮 `N/verification/gen26_combined_r2/result.json` **6993项全部通过，0失败、错误、跳过**，耗时545.134秒；960份源码/资源前后SHA256一致，226个测试文件完整覆盖。范围保留GEN25的216份测试，新增9份GEN26测试，并完整执行此前仅被源码清单绑定、未列入完整运行范围的 `test_grasp_task_sequence.py`。两轮原日志/XML/结果均保留，不将首轮失败改写为通过。

| 回执 | SHA256 |
|---|---|
| `N/verification/gen26_combined_r2/result.json` | `d5c9c7d11a43ad08b3bc23bd6903679a45866428826caa882063c10603dbc599` |
| `N/verification/gen26_final_independent_review/review.json` | `0c41ab80f451cb3b4c730b3eef757c3e4779caf196ae3bdd6b4e4b2638839071` |
| `N/verification/gen26_release_manifest.json` | `db336d8a2f2ea3c8f8ccfa35f3735fc34fd62c47f3b84abf85c03e1a00a52bd0` |
| `N/gen26_deployment/result.json` | `871be69abbd6727f8a9b50dbdc8b0a4570fe6eee6c5d8e73547a0a5dab5686f6` |

独立最终审查绑定46份改动/新增源码、运行辅助入口及101份证据，`reviewed=true`、阻断项为空。发布清单封存全部960份源码/资源。审查与发布清单中的 `runtime_deployment_pending=true` 是封存当时事实，原件不追改；随后独立部署回执确认 `deployment_passed=true`、`whole_patch_deployed=true`。3个计算节点和现有core所属motion_gateway已重载，7个硬件/GUI进程身份保持；部署未发运动、扭矩或温度查询命令。当前显式关闭 `unknown_require_measured_bilateral_contact`，可用true恢复；未整体重载YAML覆盖既有运行参数，最大补视次数仍为2、会话期限600秒、闭爪后抬升false，温度监测false/poll=0保持。

持续控制通过 `.runtime/resume_gen26_control_20261015.py` 在锁内从paused revision 23恢复为active revision 24；它核对暂停证据以及经独立审查的后续修复，恢复控制本身不触发运动。回执 `N/gen26_resume_request.json` SHA256为 `cab14d2808551908af73153024190c6c100f9bc37337f2556e2e591dd4cf0c6a`，结果 `N/gen26_resume_request_result.json` SHA256为 `b61e20ca007fbbb3f775440011bdb6499990d17865b2eaae4a1ba79889e884bc`。原持续授权仍为「你一直进行处理，直到成功抓取」，本次恢复请求「你接着继续执行」单独记录，未伪造新的原始授权。

## GEN26 第1轮：初始支持面失效，尚未运动（05:20 UTC状态快照）

新的现场资料为 `N/gen26_attempt_1_scene/`，包含实际查看的 `current_scene.png`、同源深度、`preflight.json`、由该次真实SDK反馈派生的 `preflight_joint_reference.json`、`readiness/result.json`、清场与观察回执。readiness核验960份源码与GEN26发布一致、部署进程身份一致、20份真实关节反馈最大变化0；当时任务IDLE/inactive、GUI手动，启动前selection为generation 4。当前图像中同一白绿纸盒位于红桌中央，未见手部/前臂；沿用用户 `call_f699a84ac634494d9ae4076d619ebdaa:0` 的「已停止调整，双手及前臂已移开」回复，未复用早先已过期图像。

05:17:44 UTC启动正式录制 `grasp_records/20261015T051744.120111Z_0ffc956cd2e54398a35ebff62282e999`；场景内 `active_record.json` 绑定该录制。launcher按既有 `unknown/observe_return` 策略选出generation 5，原固定任务期限仍为600秒；本次recorder上限900秒仅覆盖启动与任务尾部，不延长执行权限。

约8秒后，规划在任何任务运动之前终止：`UNKNOWN_PLANNING_TERMINAL TARGET_LOST: perception_failure: initial_station_current_support_invalidated`，随后 `NO_FRESH_MODE_PREVIEW`，候选计算已正常停止。`user_request_runner_exit.json` 记录returncode=3、physical_success=unverified。此次没有进入抓取任务active，没有观察轨迹、接近或闭爪；不能把软件回归通过记作实物成功，也不能将此原因冒充已经确定的根因。当前支持面为何失效正在独立只读取证，不能仅从终态消息判断是新盒形推断错误或桌面实物变化。

`N/gen26_attempt_1_after_manual/result.json` 已确认generation 5、任务/路由/两动作空闲，并收到GUI自身持久手动ACK；SHA256为 `1c46b4807ec90c3be079103eacb5872ab103e8e34572e13e34897462074cc779`。恢复手动没有发送运动或温度查询命令。只读监督记录为 `N/gen26_attempt_1_watch.log`。

本次检查时canonical `manifest.json` 仍为 **state=recording、result=unknown、recorder_closed=false**，`recorder_outcome.json` 与 `command_result.json` 尚未生成。recorder只有看见task active→inactive才会在终态后3秒收尾；此次在规划阶段结束，因此仍按900秒上限录制，不提前切断。该目录仍属打开/当前定位保护范围；上述规划失败事实不等于已封口的最终分类。等待录制自然关闭后，另据真实终态、日志和关闭回执补记结果，不改写当前快照。此轮尚未取得实物夹持成功证据，后续在原持续授权下处理当前失效原因。

随后只读实录分析已定位本轮失效路径，证据为 `N/verification/gen27_recorded_feedback_gap_analysis/findings.json` 及 `accepted_feedback.json`。分析只读取独立复制的64MiB前缀，原正在写入bag未修改或重建索引。16秒窗口中107份accepted反馈的六轴最大变化均为0，未见task active或臂/手轨迹goal。先前两张图像缺少合格反馈括号：前后accepted样本间隔400.006118毫秒，超过原200毫秒上限，正确进入 `accepted_feedback_bracket_gap`；不能把静止看作括号已经合格。

随后恢复帧及时进入实测重验证：入口时旧reference年龄为0.663007736秒，当前恢复帧年龄仅0.008682966秒。计算结束后失败状态构造时旧reference年龄0.826196909秒，超过原0.8秒期限；当前恢复帧自身年龄仍仅0.171872139秒。这里0.826196909秒明确是 `failure_status_build_ns` 的年龄，状态构造比内部失败稍晚，不能把其26毫秒越界与内部失败时刻混为同一个时间。真实失败阶段为 `support_gap_measured_revalidation`，原错误 `source_exceeds_existing_frame_limit`。后续GEN27将修复及时恢复帧完成审核与旧reference过期混用，同时检查producer、pending身份消费者及initial preflight等待入口；只允许原期限内真实收到的、绑定唯一恢复帧的非运动processing通知，不能事后用晚到通知续期。此处记录根因与修复方向，尚未声称GEN27验证或部署通过。

GEN26第1轮随后已自然封口。recorder在05:32:45 UTC到达900秒上限后正常关闭，`bag_closed=true`、`active_seen=false`、`interrupted=false`、returncode=0；wrapper等待日志结束和清单生成，最终退出3并输出 `GRASP_RECORD_QUEUED result=failure`。canonical manifest于05:36:08 UTC完成，现为 **complete/failure、recorder_closed=true**，明确失败依据是 `NO_FRESH_MODE_PREVIEW ... TARGET_LOST: perception_failure: initial_station_current_support_invalidated`，不是仅按退出码分类。此前05:20的录制中快照保持原样；此段为后续实际终态。

封口小文件SHA256：`manifest.json` 为 `3c122fc07fdaf52890a0deba88fdd1ae4375a83febf9a0ca8a6d7f27978b85d9`，`command_result.json` 为 `991a49552c2cc99dc33e10803ab841cb6b1d41dba86e39e502f9b892cbe3e93e`，`recorder_outcome.json` 为 `f38adcb31da138dff6ab172b6c5247fb9f7ba659887ad055a98561a68780f188`，`user_request_runner_exit.json` 为 `ce6a3a2ef4dfe06beed35ff063a12dd4272470dadc0a75e234ef6100ac7e72a1`，`runner.log` 为 `b89b57d931a98341b8e49f9ef375f9873a51d3220ace7b3346f5acd8bb199bf8`。这些小文件纳入GEN27精确追溯范围。原 `keyframes.bag` 大小为5,931,031,945字节，封口manifest登记SHA256 `5c99808c37fc49d3685af2bae82d164cf7b3ba3ba0d8fcf362ca757b53e6c576`；此次仅核现有大小及与前缀导出一致的inode 94206，不重复计算整个大bag的SHA，不将清单登记哈希冒充此次重新逐字节核验。当前问题仍需该录制定位，继续保留；本轮没有实物夹持成功。

## GEN27：反馈缺帧后的同源完成契约，已验证和部署

修复范围为4份既有生产文件：`unknown_tabletop_perception.py`、`unknown_pending_identity.py`、`initial_preflight_recovery.py`、`initial_preflight_task.py`，新增共享模块 `feedback_recovery_completion.py`，另新增2份测试，未修改旧测试。GEN26原始4份源码按发布SHA保存并逐份核验，便于复查最小差异；GEN26发布、验证与实测失败证据均保持原样。

发生真实反馈括号缺测后，仅当唯一恢复帧在旧reference期限内进入处理、重新取得真实合格反馈括号，并通过原站位/身份检查，producer才发布不含几何或运动权限的processing通知。remote和task collector必须在旧期限内实际收到该通知；随后只允许同一帧完成完整object/mask/status交接，当前源0.8秒新鲜度保持，完成截止取当前源+0.8秒与旧reference+1.6秒中更短者。通知不能伪造支持面、替代当前RGB/深度审核，或授权运动；200毫秒反馈括号上限、姿态变化和各阶段执行检查不变。

同时补齐本地时钟边界：producer、remote和collector在首次READY时固定原reference的monotonic期限及ROS时间高水位；首次通知须仍在原本地期限内，已接纳的完成窗口同样固定，ROS冻结、回跳及monotonic到期均拒绝。重复READY、旧gap或processing回调不能重新计时；恢复完成后完全相同的迟到通知仅作无操作处理，不重新打开等待，内容改变则拒绝。旧内存reference缺少这些字段时保守拒绝，因此部署必须同版重载三端并建立新generation，不能让新旧消费者混用。

独立最终 `N/verification/gen27_feedback_completion_independent_final_r2/receipt.json` 27项全部通过；消费者及相关旧回归 `N/verification/gen27_feedback_consumers_green_r2/receipt.json` 164项全部通过。两者均零失败、错误、跳过，测试期间源码哈希一致。原始独立红版17项中9失败、8通过，其中8项是有效producer/consumer误拒反例，`late_entry`的+1ns浮点取整失败属于夹具问题，不计生产缺陷；后改用明确的+0.801秒边界。`clock_r1`的23项中4失败是实际时钟缺口。`first_admission_red_r2`虽为27项中3失败，只有remote/collector两项有有效因果证据；producer夹具在scene初始化时覆盖monotonic域，其红例不计因果，`independent_final`首轮唯一失败也来自此夹具。最终r2在acquisition后保留原时钟域并单独推进时钟，验证producer真实原reference过期拒绝。上述历史失败、夹具问题及修正均保留，不将目录名中的green当作通过证明，不相加有交叠的定向计数。

完整组合 `N/verification/gen27_combined_r1/result.json` **7033项全部通过，0失败、错误、跳过，退出0**；回执总耗时600.802秒，963份源码/资源前后SHA一致，228个测试文件完整执行。范围完整继承GEN26的960份源码/资源、226个测试文件和6993项基线，再加入1个生产模块与2个新测试文件的40项用例。最终独立审查绑定25份审查/继承核验源码入口及89份证据，`reviewed=true`、阻断项为空；包括最小源码差异、全部定向红绿历史、完整组合、GEN26部署/恢复/手动/场景资料与6份封口小证据。大bag未因本轮审查重复计算SHA。

| 回执 | SHA256 |
|---|---|
| `N/verification/gen27_combined_r1/result.json` | `db27939630ee9ad5337adf5d93a35227a17ccd748c6e89e330ddecc70542e2ce` |
| `N/verification/gen27_final_independent_review/review.json` | `4a56227024e31d4855dd3d780900aedb2710e610e434a12d0888d64c79ab2570` |
| `N/verification/gen27_release_manifest.json` | `3f041d7644e0de907ed97fc90ddffaa47d6c93a8f858c1c4763140ddc123e789` |
| `N/gen27_deployment/result.json` | `d824870e04db8227d797c563d1b308051df90d92bfae21092d468fa8dd9f5bb8` |

GEN27已按封存源码完成空闲部署，独立部署回执确认 `deployment_passed=true`、`whole_patch_deployed=true`。`unknown_tabletop_perception`、`remote_grasp6d_node`、`grasp_task_node`三计算节点以及现有core自有motion_gateway完成重载，7个硬件/GUI进程保持；未发运动、扭矩或温度查询命令。审查与发布清单中的 `runtime_deployment_pending=true` 仍保留其封存当时事实，后续部署成功由独立部署回执证明，不追改旧回执。

部署验收时selection仍为generation 5，尚未启动GEN27第1轮。新一轮须在当前active持续授权下重新采集实际现场图像和SDK关节反馈、核验GEN27源码/部署及同epoch/selection的readiness，再建立新的generation 6；启动与结果以后续新录制为准。离线通过和部署完成不等于实物夹持成功，当前仍无本轮实物成功证据。

## GEN27 第1轮：观察动作路径误差，已失败封口，执行问题未解决

随后采用 `N/gen27_attempt_1_scene_r2` 的新现场与readiness，在active revision 24持续授权下启动generation 6，录制为 `grasp_records/20261015T055831.798558Z_dc3b3988bcb345e19e8ca3fae540340e`。本轮通过初始感知和新鲜预览，方案 `b88c9cb288b982a7325f6d46` 完成执行/预览绑定并调用抓取任务，进入第一条观察动作；这证明流程越过此前的初始规划终止，但不证明GEN27所有缺帧边界均已在实机触发。

05:59:13 UTC，已提交的观察轨迹由控制器返回state=4、code=-4、`Joint3 path error 0.090021`。记录中的真实反馈变化 `actual_delta=0.000000rad`，期望命令变化 `desired_delta=0.090021rad`；动作已发出不能当作机械臂已按轨迹运动或成功到位。现有失败处理安装了连续desired-command hold，并记录正向关节使能重请求；这些命令与回执也不能替代编码器确认。任务随后FAILED并释放执行槽，没有进入接近、闭爪或抬升。当前未解决项为真实关节反馈与已提交观察轨迹不一致，正独立核查驱动/控制器执行链；尚未将原因归结为某一硬件故障、算法恢复错误或温度，也未改变温度禁查和禁止失能约定。

recorder实际看见task active→inactive，于终态后约3秒自然收尾：`active_seen=true`、`bag_closed=true`、`interrupted=false`、`stop_reason=observed_task_became_inactive`。wrapper退出6，`manifest.json` 于05:59:17 UTC完成，明确为 **complete/failure、recorder_closed=true**；失败由任务状态与StartGrasp响应共同证明。原bag大小269,722,638字节，封口manifest登记SHA256 `087c25ed5f534e7edc1fbd890171fa8d329409164aa88bd5ddf6cfdf22db2cc7`，本次文档更新未重新计算bag哈希。

| 本轮证据 | SHA256 |
|---|---|
| 该录制 `manifest.json` | `53a298d2e067d68d01cd46b0316cb2800c014def49a960cdb172e11e18862035` |
| 该录制 `command_result.json` | `09178a3cbff1e1794a47284933474013df7ddf97dd467427cfebb4d93ff822f2` |
| 该录制 `recorder_outcome.json` | `36ea0d6fe72e1289e18bfdaf3c0e512aaa204fc0a8db7cc71204274f056075e6` |
| `N/gen27_attempt_1_after_manual/result.json` | `d6dfa9a6a0b34e2cfb561ba72525b585f4ab3319ccee3279057ba7e5a3938201` |

失败后已在generation 6核实任务、路由及动作空闲，并收到GUI自身持久手动ACK。该手动恢复回执 `motion_commands_sent=false`、`temperature_queried=false`；本轮动作时间线另见 `N/gen27_attempt_1_watch.log` 和录制 `runner.log`。本次记录仍是当前未解决执行问题的必要证据，继续保留；GEN27软件验证/部署成功不改变本轮实测失败，更不代表实物夹持成功。

## GEN27 恢复诊断、主机控制器对齐与第2轮启动（2026-10-15 07:26–07:45 UTC）

用户暂停后再次明确「继续处理」，07:26:09 UTC将持续控制从paused revision 25恢复为active revision 26，回执为 `N/gen27_resume_diagnosis_20261015T0725Z/resume_receipt.json`。恢复时generation 6、任务FAILED/inactive、GUI手动，真实反馈可用，actuation仍为 `PENDING:POSITIVE_ENABLE_REQUESTED`，epoch为 `1792043953113863243`。用户现场回复「没有看到机械臂移动」「主电源已开，急停已释放」，原回复ID与文本已存同目录 `site_replies.json`；这些事实不等同设备已完成运动，也不是新的清场证明。

既有 `N/gen27_attempt_1_motion_diagnostics/result.json` 保留实际只读响应：原始速度通道均为0，自检mask=1023、abnormal_channel_bits=0；通道映射和速度单位未验证，不能据此宣布机械执行正常。恢复后的 `N/gen27_resume_device_identity/result.json` 收到新鲜CRC有效版本响应，设备ID为 `ADFX260604ITKTXC`，原始硬件/固件版本为110/613；这证明串口响应及设备身份，不证明固件兼容性、主动力输出或关节跟随。未查询温度，未发关节失能命令，未将硬件根因记为解决。

`N/gen27_resume_reference_snapshot/result.json` 的5秒被动窗口未收到control_reference，不能声称新epoch已建立SDK参考。主机controller仍保持上次失败的desired，J3为−0.2934741145 rad，而accepted为−0.3834951970 rad，相差约0.090021 rad，desired速度与加速度均为0。为消除这段旧主机参考桥，采用 `.runtime/align_gen27_host_controller_in_manual.py` 单次主机预同步：全程保持GUI manual=true、任务/路由/动作空闲，只向 `/alicia_controller/command` 发布一次0.2秒六轴实测位置hold。driver在手动模式入口拒绝非gui_direct/gui_direct_sync来源，因而此操作允许主机desired更新，不授权SDK运动。它不是新的抓取，也不能替代正常runner随后取得SDK成功写回、真实响应及路径检查。

该helper已实际发布hold，但 `N/gen27_resume_host_controller_alignment/result.json` **仍为passed=false**，原回执不改写。失败来自发布后检查把task接收时间限制为2.5秒：`/grasp/state`实际上是仅状态切换发布的latched消息，不是周期心跳；空闲task的单次latch自然超时。原回执没有逐topic接收年龄，不能据其单独排除其他项同时超时。该次窗口保存26帧accepted，覆盖2.500015226秒，关节变化0，SDK写入观测0、busy事件为空，manual/epoch/actuation均无变化；未因这个检查失败重发hold。helper源码SHA256为 `ccd33af1d943fca6b3f4006d36b4e40811a43aba4c2f5ab43dd4127377ed79b1`。

随后独立只读 `N/gen27_resume_host_controller_postcheck/result.json` **passed=true**：60帧accepted各关节范围均为0，SDK消息0，desired与实测最大差 `2.220446049250313e-16 rad`，desired静止，manual、epoch、PENDING状态保持，未见任务或动作活动。实际约6秒窗口中controller302帧、arm/hand各32帧、task仅1条latch，进一步确认task心跳假设不适用。此结果独立证明主机对齐已完成；不追改helper失败，不证明此前无运动的硬件原因已排除，也不构成实物抓取成功。

本检查点的 `N/gen27_attempt_2_scene_r2/visual_review.json` 记录了实际图像审核：同一目标可见，但左上方有拿手机的手及前臂，`conflicting_hand_or_forearm_visible=true`、`launch_authorized_by_this_image=false`。因此该图不能作为清场证据，已请求新的清场确认；尚未生成本轮scene receipt、未启动GEN27第2轮抓取。此时仍为active revision 26、generation 6、manual=true且任务inactive。GEN27封存源码与发布证据未修改；后续须依据新的实际清场图像、同epoch姿态/readiness和独立新回执决定启动，不能沿用这张有冲突的图像。

| 本次关键证据 | SHA256 |
|---|---|
| `N/gen27_resume_diagnosis_20261015T0725Z/resume_receipt.json` | `dd451a78be908a8efae50ebc8195c021545560d3fc700c4f7abf589abb14894f` |
| 同目录 `site_replies.json` | `9ad966b29d0f008078766a40487a1c9a714dd06c342faba82ecd0623e2e43693` |
| `N/gen27_resume_device_identity/result.json` | `c2ac0fe8b8cd081a36423ca6248398e8490bf11f749559920938b03f9caa051f` |
| `N/gen27_resume_reference_snapshot/result.json` | `9830525c7a34daec6d58d1fbd28fdb5d917ba2f693fcb4c0fb1004b0f6c8a8ed` |
| `N/gen27_resume_host_controller_alignment/result.json`（失败原件） | `13a1c709ae257636cae764391ccf7702fa5b1daf5627942fb2c512b188a80057` |
| `N/gen27_resume_host_controller_postcheck/result.json`（独立后验） | `aac0831b2817b17fd07589c1290c72d3e2f81b9608eca6fc1b4e11cc0698fdf3` |
| `N/gen27_attempt_2_scene_r2/visual_review.json` | `9afffa29f656acdaa2d1bdcfe2e92681843d70db5e52d1daa0819ea63d6c4576` |

上述清场等待随后已解决。用户在 `call_db4ef3feff6b49f5bdbd6a30bf8454b0:0` 明确回复「双手和前臂已移开，已停止调整」。r3图像未见手，但采集时manual=false，因此恢复手动后重新取得 `N/gen27_attempt_2_scene_r4`，未将r3作为通过的启动快照。r4实际图像审核确认同一目标、未见冲突手部；20帧反馈最大变化0，epoch一致，963份源码与GEN27发布一致，部署进程身份保持。r4的 `scene_clear_receipt.json` SHA为 `1cb1c8d01ebb581ca67facd22a8b60bffcd9ccd1593551f904e96653a7362f8f`，`operator_observation.json` SHA为 `9a3dee07a5de75e255e9b629b6e983428d16b67610290ad53cb8a24e569d808f`，`readiness/result.json` SHA为 `de26f4ef7a744b8f89dbc11fa295645323badd47861c13bf2105a4354af5a9b6`。

07:45:06 UTC在active revision 26下启动GEN27第2轮带录制尝试：`grasp_records/20261015T074505.997672Z_ab5fc4f19b2a4ba7b704e6e2d7ffa05d`。r4 `active_record.json`已绑定该目录，录制内authorization确认selection更新至generation 7、unknown/observe_return，physical_success仍为unverified。本段仅记录实际启动，最终任务及封口结果尚待新证据；不能把主机对齐、清场或启动成功写成机械执行/实物抓取成功。

## GEN27 第2轮结果：受限前缀无实测响应，失败封口

第2轮随后已失败，原启动时的未知结果由本段补记。`N/gen27_attempt_2_watch_r2.log`记录任务依次进入 `admitting bounded actuation prefix`、`confirming actuation along bounded observation prefix`，状态由PENDING:COMMAND_SYNCHRONIZED进入PENDING:AWAITING_ENCODER_RESPONSE。07:45:19 UTC提交的是3秒受限前缀，最大关节目标变化约0.025 rad，没有提交上轮完整观察轨迹。控制器虽返回action state=3、error_code=0，但未得到真实编码器方向响应，不能将此响应当作实机到位。

07:45:23 UTC驱动进入 `UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT` 并记录 `INITIAL_PROBE_TARGET_RETIRED`；该机制退役主机探测目标和重放缓存，不表示清除了设备内部保持目标。任务以 `ACTUATION_BOOTSTRAP_FAILED: ACTUATION_PREFIX_NO_RESPONSE` 结束，释放执行槽，StartGrasp返回 `actuation bootstrap failed`。新正向enable epoch及主机对齐后仍未取得真实关节运动响应，未进入完整观察、接近、闭爪或抬升；此轮不能记作bootstrap恢复成功，更不能记作实物抓取成功。硬件/执行链无响应原因仍未解决。

录制正常看见active→inactive，recorder在终态后约3秒结束，`bag_closed=true`、`interrupted=false`；wrapper退出6。07:45:27 UTC manifest已为 **complete/failure、recorder_closed=true**，依据为真实任务终态及StartGrasp响应，而不是单独按退出码分类。bag为119,498,411字节，manifest登记SHA256 `a967c95c7aec82e04475748067d18fd2e162e74e217e9df26c1a1ccc0e52e128`；此次文档更新只读封口小文件，未重新计算整个bag哈希。

随后 `N/gen27_attempt_2_after_manual/result.json` 已确认generation 7、任务/路由/两动作空闲，GUI自身latched手动ACK于 `1792050372501105070` 到达。持续授权保持active revision 26；当前等待用户仅观察并反馈控制盒/电源指示灯、报警文字或错误码，以补足设备端事实。该询问不是重复索要动作授权；等待期间不发新动作，不查询温度，不发失能命令。当前记录继续作为未解决问题必要证据保留。

| 第2轮终态证据 | SHA256 |
|---|---|
| 该录制 `manifest.json` | `8433bbdf94315e2673381fb40fc3179936e13ba6f682b7844beaf5d08f703552` |
| 该录制 `command_result.json` | `655125a0614212919435d15f9e2bed2856d98fbad217c521bc4813b9a45823e9` |
| 该录制 `recorder_outcome.json` | `f6ae908a091cd0251900f08841dc7459070e4a1ccce55bd635043f4d3bcdd77c` |
| `N/gen27_attempt_2_after_manual/result.json` | `6f111db64ab4a76224ecf4bcb052aba366b19b8055b4758b86691d27ebc4824f` |

## 重启后真实响应恢复与 GEN27 第3轮接触规划失败

用户随后报告已完成重启与清场。`N/gen27_after_power_restart_readonly/result.json`确认新epoch `1792050457131485178`、`CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE`及与上一失败快照不同的真实关节姿态；当时任务inactive。独立归因 `N/verification/gen27_power_restart_recovery/findings.json`从原日志按时间与SHA追溯：新epoch来自 `demonstration_false` 正向enable分支，日志没有记录该 `/demonstration` 消息的caller身份；之后明确出现 `gui_direct_sync` 与GUI关节指令，首个CONFIRMED时间为 `1792050460777042731`，后续还有多轴手动调整。两快照之间最大关节差约0.092039 rad包含这些手动调整，不能归为单独重启位移或自动抓取验证。该证据证明真实响应已恢复，不确证之前无响应的硬件根因，也不将旧失败改为成功。

重启后的初次采集manual=false，按既有流程恢复手动后重新检查。`N/gen27_attempt_3_scene/visual_review.json`又发现左上手部，未据此启动；用户在 `call_aec8e6b0ec9743df8e0b53e44aaccc2a:0`确认「已停止操作，双手及前臂已移开」后，重新采集 `N/gen27_attempt_3_scene_r2`并实际审核清场。启动前20帧accepted最大变化0、同新epoch、manual=true且任务inactive；963份源码及部署进程与GEN27发布一致，沿用已完成的7033项组合验证，没有声称本轮重新跑过测试。

07:55:03 UTC在active revision 26下启动generation 8，录制为 `grasp_records/20261015T075503.791411Z_24849e8304a44341adebca3867d29fdd`。新方案 `8d9f3fc0f3f03f4db5059811`提交34.175689425秒的完整观测轨迹，控制器返回state=3、error_code=0，随后通过真实测量与station资格检查，进入 `qualified observation station; checking range before contact planning`。与第2轮只有命令完成而无编码器响应不同，本轮自动补视已有实际运动并到位；这仅验证本轮观察阶段，不能外推为全部执行问题永久解决。

后续3个接触候选的检查记录分别涉及 `PREGRASP_HELD_AXES_GUIDE_DOES_NOT_REACH_FIXED_GOAL`、预测命令连续几何的 `target OBB collision at 14-end/right_finger`，以及计算截止导致的 `MOVEIT_TIMEOUT`。这些是候选规划拒绝，未获接触执行权限；不把备选错误改写为已发生的碰撞。最终在 `1792051046156390666` 进入FAILED，任务原因是 `TARGET_LOST: station_compute_wait_expired_or_future`，随后释放执行槽；producer另记录 `station_compute_wait_station_changed`。没有执行接触接近、闭爪或抬升，接触规划与等待终止问题仍需后续离线诊断/修复，尚无实物抓取成功。原文的 `13-end` 为文档转录错误，按原候选审计及日志更正为 `14-end`，实验原始证据未改。

07:57:30 UTC录制已封口为 **complete/failure、recorder_closed=true**，wrapper退出6，recorder正常看见active→inactive并在约3秒后关闭，未中断。bag大小612,889,902字节，manifest登记SHA256 `2f689e54357e1814d071b873585174261f030d40c6aae07248fd4b9e071d9660`；本次文档更新未重复计算整个bag哈希。`N/gen27_attempt_3_after_manual/result.json`确认generation 8、任务/路由/动作空闲及GUI自身手动ACK；持续授权仍为active revision 26。封存源码和既有发布/实验原回执保持原样，当前继续处理接触规划失败，不自动把本轮运动恢复记成抓取完成。

| 重启恢复及第3轮证据 | SHA256 |
|---|---|
| `N/verification/gen27_power_restart_recovery/findings.json` | `79e31fa6f5f1bf456a6e8679028cbf1f0cf9ef93a6a38bdf75051ab01dfe9df2` |
| `N/gen27_after_power_restart_readonly/result.json` | `8087fc165fdeddf448368e5d3945598a26190c7b177f199afc71102396ac2399` |
| `N/gen27_attempt_3_scene_r2/operator_observation.json` | `ae8e13ae78932d4ccf55c6b25e9cfb6bd091e6ec2c7180cbced873e92c9d24ff` |
| `N/gen27_attempt_3_scene_r2/readiness/result.json` | `70526886773ccd1d19d9a2eef9b9f183f6870f7cf63496dea73c666feda1ebc4` |
| 第3轮录制 `manifest.json` | `068b4f2140bc64e868655f52369ca92354f2da3d259f0c0cd2a83a1df8a1ae7f` |
| 第3轮录制 `command_result.json` | `08e8da7815816b74b213a3d90161388c72ce2f99d8a80e12a7b015dd434b3797` |
| 第3轮录制 `recorder_outcome.json` | `feb5a637dbdb38f31747b04897fc5ffa625747ec3ea97b5dd966b4413723e9c6` |
| `N/gen27_attempt_3_after_manual/result.json` | `acb1a251b4d0842dad7f39117600d8206ee77e19bb03d262ca8423ef991d27b4` |

## GEN27 第3轮补充审查：初始推断、融合输入与补视计算终止

用户追问初始轮廓和深度大部可见为何仍补视。实际初始触发帧为 `1792050907106353521`，保存的观察审计 `17d4f9d7…` 明确记录 `BILATERAL_SURFACE_EVIDENCE_MISSING`，`contact_search_completed=false`、`learned_candidates_requested=false`、`materialized_count=0`。第一窄向 OBB 宽35.897毫米，含开口适配余量为36.897毫米，小于夹爪最大50毫米；负侧独立实测高度仅1.526毫米，正侧8.662毫米，公共接触带未成立，7个方向均有缺侧证据。因此此次确实在直接接触候选物化为0后进入补视，不能说所有直接路径都已完整试过，也不是 `observe_return` 策略无条件要求先观察。当前 `unknown_require_measured_bilateral_contact=false`；完整合格盒形仍可走初始直接接触路线。

原初始观察审计没有保留该次 `regular_box_inference` 资格结果，不能凭缺字段断言推断函数未执行。离线使用 acquisition 到触发帧的三份封口 RGB-D（`1792050906773036718`、`1792050906972096443`、`1792050907106353521`），调用生产融合、几何估计和盒顶资格函数重建，得到 `BOX_TOP_NOT_RECTANGULAR`：顶面RMS为0.553毫米、矩形填充率0.9543，边覆盖率 `[1,1,1,0.7]`，角覆盖率 `[0.427,0.420,0.380,0.257]`，最远角缺口2.844毫米。主体深度足够而四角支持不足，是该重建的明确拒绝条件；不能以“完全没有深度”解释。重建 OBB 为56.587×35.965×21.065毫米，与原触发的窄边/高度分别差0.068/0.014毫米，但原完整有效几何参数、previous axes与准确缓冲配对未保留；此次使用生产YAML/默认值、最近封口关节样本并无previous axes，结果按**有明确限制的离线重建**保存，不写成原运行逐位复现。

这里必须区分输入：生产规划的 `snapshot.depth_raw` 是多帧逐像素中位数；`snapshot.object_mask` 是多帧多数表决后再侵蚀2像素，RGB仍取最后一张原图。保留的 `original_sample` 则是同一最后时间戳的原始深度和原始mask。即使时间戳相同，两套像素与几何处理用途也不同；不能将单帧原始NPZ配同时间戳的融合几何审计称为原融合资格的精确重放。初始三帧彩色图中前侧白色侧面的下缘与红底有明显边界，Hough确实检测到两段；后侧下缘被盒体遮挡，右侧窄面像素较少。整物体mask包含可见侧面且下角有不规则突起，可作为搜索约束，不能直接当成精确盒顶或四条底边的实测证明。

局部离线原型 `.runtime/prototype_gen27_rgb_direction_box.py` 只将同源RGB实际Hough线段提供的方向作为额外盒形方向，最多48条线、0.5度去重、至多12个方向加原深度方向；没有任意角度扫描。每个方向均在重算顶面坐标与尺寸后重新经过原四边/四角、占据率、空洞、内外侧深度、实测点包容、误差与模型校验，阈值未改。独立数值审查确认支持面法向完全不变、轴重构误差0、正交/行列式误差小于9e-16。原型仅使**稍后同姿态的单帧** `1792050907639824151` 在−2.611度方向下通过，尺寸56.944×38.826×19.627毫米；真正初始三帧重建10个方向及近场单帧9个方向仍全部失败。初始融合输入的有效方向均缺少lower-y边候选，说明轴方向变化本身不能补回受mask搜索范围影响的候选。原型尚未部署，也未证明实际初始直接抓取可执行或实物夹持成功；后续仅继续检查原始/融合mask与RGB候选搜索域的区别，不能把原型单帧通过写成整条路线已修复。

近场原审计 `bc7ee7f0…` 对 `1792050956912783622` 明确记录盒形资格失败 `BOX_TOP_NOT_RECTANGULAR`、推断enclosure为null。该阶段没有用合格推断盒形计算开口/碰撞包络，因此 `14-end/right_finger` 拒绝不能归因于推断不确定量在开口与OBB中重复叠加。本节不扩大对此候选碰撞的根因判断。

对终止过程的进一步只读取证还纠正了“接触搜索一直卡住”的解释：request 74已经以 `CONTACT_SEARCH_YIELDED_FOR_MEASURED_VIEW` 成功让出接触搜索并生成补视意图。随后request 75的局部观察规划持续计算约46.4秒；从 `1792051014.221012` 至 `1792051046.410482` 共返回20次 `OBSERVATION_PLANNING_TIMEOUT / moveit_error_code=-6`，MoveGroup反复报告无法采样有效目标状态。原station约90秒阶段截止随后仍按既有规则触发任务失败；最后一条在途服务结果在任务失败后返回。不据这些单次规划超时宣称全局不可达，也没有证据把这段归为图像等待或死锁；未延长时间限制。接触候选的局部拒绝、补视搜索失败与最终station期限是三个不同层次，不再混为同一原因。

| 本次离线审查证据 | SHA256 |
|---|---|
| `N/verification/gen27_attempt3_initial_inference_review/initial_three_frame_replay.json` | `e6dd2d99a578a012dfd73c5b656613bf87e00473db50d5c2cc19ed2514152af4` |
| 同目录 `handoff_and_gate_findings.json` | `64abd9b92effabdcc79cefc6f7d2631ce7973ee130f14593543a8f3d55e40d6f` |
| `N/verification/gen27_rgb_direction_prototype_r1/receipt.json` | `8989ce5dd783f76f4b179f76a660e282c43169489dc70d17ce5a6f54d275b574` |
| `N/verification/gen27_rgb_direction_independent_review/findings.json` | `03de7f81fc73fe1b599fa4a16745ab98a132a230dec2210b0c7c27acb29ac86b` |
| `N/verification/gen27_attempt3_compute_wait_trace/findings.json` | `d041d5acf4bd7a0998c2ff1a604657e921cfa87def53b8435c20ab8dd6cecad9` |

上述均为封口数据与源码的局部离线审查；没有生产源码修改、ROS/设备调用或温度读取。原实验失败分类与全部原始回执保持，尚无闭爪或实物抓取成功。

## GEN28 追加更正：重建 bbox、原始同帧来源与验证边界

前节初始三帧离线重建后来发现一个诊断脚本错误：`RgbdSample.bbox` 的生产约定为 `(x,y,width,height)`，旧脚本传入了 `(x,y,xmax,ymax)`。这改变支持面估计的外围区域，旧几何及其数值不能继续当作准确生产重放；旧段和原始证据保留，由本段追加更正。新目录 `N/verification/gen28_initial_corrected_bbox_replay` 已按正确xywh重建，所有融合NPZ数组及文件SHA均与旧重建相同，但几何JSON已改变。更正后的中心、窄边和高度与原触发审计的差分别为 `5.55e-17`、`1.39e-17`、`6.94e-18 m`。融合像素资格仍拒绝 `BOX_TOP_NOT_RECTANGULAR`，角覆盖率更正为 `[0.430,0.419,0.395,0.271]`。原有效参数、previous axes及原缓冲成员关系未完整留存的限制仍适用；数值吻合不等于异步运行逐位复现。

GEN28的最小生产修复只改变盒形资格的输入归属：`qualify_regular_box_snapshot` 在 `regular_box_inference.py` 中校验保留的最后原始帧，`remote_grasp6d_node.py` 改用该入口。RGB、depth和mask必须来自同一原始样本，并绑定精确时间戳、目标身份/epoch、frame、RGB数组及融合lineage末项规范化0/1 mask SHA。原始证据缺失或不匹配时，拒绝本次盒形推断并说明原因，不借用融合像素或较新的缓存。规划几何仍使用原请求的融合实测点及独立 `geometry_sha256`，不混淆两种来源。原像素资格函数、阈值、误差公式、碰撞包络、执行与闭爪检查保持原样；没有纳入RGB方向扩展、角点新规则或扣除边界误差的原型。

验证已经完成：6文件定向回归94项通过，独立来源绑定测试19项通过；完整组合覆盖230个测试文件、965份源码，共 **7067项通过，0失败/错误/跳过**，wrapper退出0，运行约588.84秒，来源在测试前后未变。文档更新时再次逐份核对965份源码与该组合结果一致，并核对pytest日志/XML哈希。94项初轮的一项失败来自新夹具错误假设“侵蚀2像素必然不合格”；19项独立初轮来自edge-clearance夹具构造错误，均保留原失败记录，修正夹具后重新通过，不把这些失败记为生产缺口。独立最终review已记录空阻塞项，同时明确软件验证不证明实物抓取。

真实历史输入的r3回放确实调用了生产盒形入口和候选生成器：正确xywh三帧经实际fuser融合，最后原始帧的完整RGB/depth/mask使盒形通过，融合实测几何未改。然而候选数仍为 **0**，原因是 `BOX_INFERENCE_PRECISION_VIEW_REQUIRED`。推断窄边38.867820毫米，edge经验误差7.936185毫米，其中未观测边界扩展为6.273838毫米，yaw经验误差0.056081 rad；按原审计每侧适配间隙0.5毫米，所需开口上界为 **58.854761毫米，大于物理50毫米**。这是修复后的真实限制，不是实物成功。该回放没有调用MoveIt、控制器或任务执行，其他候选参数仍有离线fixture限制，不能称完整直接路径已全部试过。

GEN28随后已封存并完成计算节点及既有gateway重载。封存文件的 `runtime_deployment_pending=true` 是封存时状态，后续独立部署回执才证明 `deployment_passed=true`、`whole_patch_deployed=true`；不追改封存原件。7个保留进程的PID/start_ticks前后相同，包括driver、硬件接口、MoveGroup、相机、夹爪控制、路由和GUI。部署未发送运动或力矩指令，未查询温度；部署成功不表示新抓取已执行。

本轮当前视角快照 `N/gen28_readonly_current_scene/preflight.json` 为generation 8、任务inactive、manual=false；其六轴姿态与GEN27第3轮启动前快照最大差约0.150330 rad。两次快照跨越既有观察运动及后续间隔，不能单凭差值归因于某次用户操作或新自动动作。随后 `N/gen28_before_deployment_manual/result.json` 已确认GUI自身手动ACK及任务/路由/动作空闲。第一次当前三帧采集 `N/gen28_current_initial_readonly_inputs/result.json` 保留失败：参数整体比对命中了相机runtime统计变化，且三份已存来源均为detected=false、mask为空；对应detector状态明确为旧generation 8的 `target_lost_requires_new_generation`。因此这些文件不能作为有效目标三帧，也不能据此宣布当前盒形定位失败。后续新实例重采与离线资格/候选结果另行追加。

用户最新要求优先从当前初始视角完成定位并直接抓取，只有该视角解决不了定位或直接规划时才补视。本轮先按此要求取得同站位原始三帧，离线运行实际fuser、原资格及候选生成器，明确误差和开口限制，再决定现有流程下一步；不为此立即增加新的生产状态机。三帧一致仅证明短时重复性，不能证明误差独立随机，更不能据此缩小共同标定偏差、持续遮挡或缺深度边界的预算。近场角点原型扩展已暂停，不属于GEN28。到本段部署检查点尚无新抓取、闭爪或实物成功记录，定位精度与完整直接路径能否通过仍待实际证据。

| GEN28关键证据 | SHA256 |
|---|---|
| `N/verification/gen28_initial_corrected_bbox_replay/correction_receipt.json` | `6e2b0c4c8c3d06a68333018de25b66cfcac62a812707433caa3a16ed920d6be0` |
| 同目录 `initial_three_frame_replay.json` | `7986395a96f441908337afaae04779cdff8651f224d1e0d6bc7953c743f3d4f6` |
| `N/verification/gen28_retained_box_source_green_r2/receipt.json` | `de43d735b33d844a5ac9d8a73ba641945fcf72d5a95adf0db8a113b1abe90714` |
| `N/verification/gen28_raw_box_source_independent_green_r2/receipt.json` | `52a53774ce5520a49b40ea821e1bef68ce56f650b4ca4e021df47c9005c6712b` |
| `N/verification/gen28_combined_r1/result.json` | `0eb0b42451a412fb36513a77804176078a12bd4b8518e79305e66427a26adbc0` |
| `N/verification/gen28_actual_raw_box_entry_r3/result.json` | `74e002d57bbfc792994acc6281aabbf98073331a9cf274bc1d4b83674fabf7b2` |
| `N/verification/gen28_final_independent_review/review.json` | `c651e3da8f303d204b06f27f83c60aa17fcdbe0f9408445cb57dd1a896f81695` |
| `N/verification/gen28_release_manifest.json` | `2ae560cd671ae402366eaa41a2235905f51bd3b5cc6f43afd379df1f842bddfd` |
| `N/gen28_deployment/result.json` | `3fb502df4cc02ca09b35328dd60bb235c3f0e72dc3a44259725a24469cfb4efd` |
| `N/gen28_readonly_current_scene/preflight.json` | `7095bfdb6b78dac46a41db03f0e7f62dfb09fdd965076ab94a49c87a94b85e91` |
| `N/gen28_before_deployment_manual/result.json` | `69a5b97ecdc46e381912d00ae7db18ce7ff33065cfc852b5ae6420e581120519` |
| `N/gen28_current_initial_readonly_inputs/result.json`（失败原件） | `5475d2aecd652c54a99512bc2add1a5858337e94aaeaa59227c0c1c4b65798df` |

## GEN28 当前原位三帧：内部顶面证据不足与重摆目标后的重新评估

后续 `N/gen28_current_initial_readonly_inputs_r2/result.json` 已取得有效目标三帧，来源时间戳为 `1792054617925998687`、`1792054618126289844`、`1792054618260626316`。采集前后GUI手动、task IDLE/inactive、同epoch，两端各20帧反馈的最大关节变化均为0。该采集没有发布者、服务调用或动作授权。离线使用实际fuser、GEN28原始同帧入口和候选生成器，绑定当时实际参数；使用的epoch 0诊断身份不是线上目标身份，也不复现异步ticket准入。生产源码没有修改。

当前三帧按各自原始RGB/depth/mask、同一份融合实测几何逐帧检查，前两帧均为 `BOX_TOP_NOT_RECTANGULAR`，未找到合格RGB矩形；第三帧确实通过四条RGB边和四角检查，角覆盖率 `[1,0.64,1,1]`、边覆盖率均为1、视觉填充率0.999778。第三帧顶面占据率为77.7457%，已满足RGB分支65%的下限，但最大内部非顶面连通域占6.35838%，超过原2%条件，因此原资格结果仍为 `BOX_TOP_CONTOUR_INCOMPLETE`，model=null。真实候选生成器走实测分支后物化候选为0，记录 `BILATERAL_SURFACE_EVIDENCE_MISSING`；没有执行完整路径规划或接触动作。

局部统计进一步区分了该“孔洞”含义：最大内部连通域286像素，bbox为 `[318,233,17,23]`，均在原始object mask内；其中230像素为零深度，另56像素有有效正深度，但相对拟合顶面低1.5009–2.5475毫米。没有NaN、超顶面的点或低于顶面种子半高的点。该指标把缺测与非顶面残差聚在一起，不能把286像素都称为真实物理孔洞，也不能把它们全当作纯缺测并抹去。现有证据未确定56个低点来自实际表面形状还是测量误差；三帧稳定性不能代替此区分，也不构成缩小系统误差的依据。

为检查孔洞条件是否为唯一阻塞，另存 `N/verification/gen28_current_internal_hole_counterfactual_r2/result.json`。这是**被拒输入的反事实数值诊断，不是合格模型**：仅在内存克隆函数中移除 `hole_fraction > .02` 一项，保留占据率及其他检查；流程继续到真正模型校验时仍因 `maximum_internal_hole_fraction outside inference limits` 拒绝。开口数值使用私有 `DiagnosticOnlyBox` 类型适配，原生产类型守卫另行确认拒绝；未发出生产模型、可执行方案或动作权限，也未评估IK、碰撞路径或闭爪契约。不能将此诊断称为“全部资格已过”。

该反事实保持原误差计算，得到窄边40.357605毫米、宽度不确定量16.505721毫米，加双侧适配间隙总1毫米，所需开口为 **57.863326毫米，仍大于50毫米**。对应edge经验误差6.972706毫米、yaw误差2.640473度、未观测边界扩展5.248388毫米。它只说明在当前公式下单独跳过孔洞条件仍不足以得到直接抓取候选，不证明孔洞可忽略、误差已解决或抓取不可全局实现。此数值与上一节历史帧的58.854761毫米属于不同输入，不能混用。

随后准备启动的 `N/gen28_attempt_1_scene/current_scene.png` 左上可见前臂，因此没有依据该图启动运动。用户之后明确已移动目标位置并重新对准，并在 `call_8691c0cf7b8b4a1c96950f77777aef9b:0` 再次回复「双手及前臂已移开，已停止调整」；该真实回复文本与ID存于 `N/verification/gen28_current_initial_document_update/site_replies.json`，不补造平台时间。`N/gen28_repositioned_target_scene/preflight.json` 记录了不同六轴姿态，采集时manual=false、task IDLE/inactive；随后 `N/gen28_after_reposition_manual/result.json` 已确认GUI自身手动ACK和inactive。本检查点尚未启动新的抓取。主线程将清除旧目标关联，按重新摆放后的目标重新做初始视角评估；本节孔洞、尺寸及开口结论只适用于重摆前的三帧，不能沿用于新位置，不把重摆或手动恢复记成自动运动成功。

| 当前三帧与重摆前后证据 | SHA256 |
|---|---|
| `N/gen28_current_initial_readonly_inputs_r2/result.json` | `05ca2b9bca204d74b0a70a2911fa2720093977437bbf160cf08dcf3e11c78e65` |
| `N/verification/gen28_current_initial_readonly_replay_r1/result.json` | `4a70af98a6a8fa90a79cdbc44f771e3a5d51545656b6598c1ffdc4dd9f2b16d7` |
| 同目录 `raw_frame_qualification.json` | `ace365b84f65ae6c584b259c0558fa079d1e8d6f73ab2aa6f03efaba2c14738a` |
| `N/verification/gen28_current_internal_hole_counterfactual_r2/result.json` | `1e9f00bd8c49d6589bc63ef38cc35cb5a6ef46bb992499e121ae833bbc789d2b` |
| `N/gen28_attempt_1_scene/current_scene.png`（可见前臂，未启动） | `8d7a572210d39b949466af7ecdfdb155f113fdf5ca00efec1f358d3734df90c9` |
| 同目录 `preflight.json` | `e65ad055285bc52c1a1a36ea2d4f3e22ce0af515cfb306229e7bffcd4bd52dc4` |
| `N/gen28_repositioned_target_scene/preflight.json` | `925dd3e51541725db663977c2004a84ef8e2f368f46611ffb3d9f762bb380f7b` |
| `N/gen28_after_reposition_manual/result.json` | `4376ff9b30c34be9c7bdb62ff676ef33ab581b8ada7215b90c3ef14ecc723ef0` |

## GEN28 重摆后初始评估与第1轮实测失败（2026-10-15 09:19–09:29 UTC）

按用户“优先初始视角直接抓取，定位解决不了时允许补视”的要求，重摆后的目标重新建立了关联。`N/gen28_repositioned_target_selection/result.json` 记录一次 `/grasp_mode/select`，generation从8更新为9（selection stamp `1792055941265166759`）；前后GUI手动、任务及动作空闲，进程身份保持。该操作仅重置感知目标，没有请求推断启动、任务启动、enable或运动。旧generation已退役的目标和旧位置几何未作为新位置依据，随后另行采集新的原始三帧。

`N/gen28_repositioned_initial_readonly_inputs/result.json` 的有效三帧为 `1792055992307435750`、`1792055992441091537`、`1792055992637341499`，采集前后均为manual=true、task inactive，关节最大变化0。实际fuser、GEN28盒形入口及候选生成器的离线回放显示三帧原始资格均为 `BOX_TOP_NOT_RECTANGULAR`。末帧顶面RMS为0.656毫米，角覆盖率 `[0.820,0.266,0.282,0.428]`、边覆盖率 `[1,1,1,0.5]`，没有合格盒形模型；同请求融合实测几何尺寸为56.387×36.005×21.081毫米。候选生成器物化数为0，记录 `GRIPPER_SWEEP_COLLISION`；这是离线候选的几何拒绝，不是实际碰撞，也未完成完整模式节点的MoveIt/IK路径证明。回放使用base remote fixture和诊断身份，相关handeye、visibility、评分及lifecycle限制继续保留。

对重摆后最后一帧另做未部署的RGB方向原型：只使用12条实际Hough线提取有限方向，连同原深度方向共评估8个方向，完整资格通过数仍为0，约0.171秒。原阈值与误差公式未改，没有任意角度扫描或动作授权。因此新位置的初始证据仍不足以给出直接抓取候选；旧位置的内部缺口6.358%、反事实57.863毫米，以及更早历史输入58.855毫米均不得当作新位置的数值。该原型未加入GEN28生产版本，也不代表所有可能直接路径已穷尽或目标全局不可抓。

随后取得新的 `N/gen28_attempt_1_scene_r2`。实际图像审核记录重新摆放的同一盒状目标可见，左侧桌沿外有人员上臂，但未见双手或前臂进入桌面目标作业区域；该图与最新 `call_8691c0cf7b8b4a1c96950f77777aef9b:0` 清场回复绑定，没有沿用前次有冲突前臂的启动图。readiness为通过：generation 9、manual=true、task IDLE/inactive、20帧关节变化0、965份来源与GEN28封存一致、部署进程未变。用户初始直接优先的原文与持续授权分别保存在本次 `resume_request.json` 和 `operator_observation.json`，没有把“允许补视”改写为无条件先补视。

09:26:59 UTC启动GEN28第1轮实际录制，目录为 `grasp_records/20261015T092659.242714Z_911b220cf1d34113b7c80a7f97dad165`（下称R28）。正常runner重新选择generation 10（stamp `1792056421473146915`），从新请求生成preview `7dbc947926dcc8cca93de7c6`、提交绑定审计、提升执行权限并调用StartGrasp；没有复用旧plan。实测watch记录 `MOVE_PREGRASP: moving 6D pregrasp`，之后为 `correcting observation endpoint`，在 `1792056472.660183` 到达 `qualified observation station; checking range before contact planning`。本轮发生了实际观察运动并进入合格补视站位，不能把它写成只有离线计算；这仍不是接触或抓取成功。

在 `1792056562.724853` 任务进入FAILED，原因是 `TARGET_LOST: station_compute_wait_expired_or_future`，随后inactive并释放执行槽；runner的StartGrasp响应同样为失败。当前已见状态和终态没有显示闭爪或抬升，physical_success仍为unverified。后续封口bag与原源码回归已经确认补视preview被clock pending覆盖的交接缺口，详见下方GEN29段；该错误码本身不能证明物体实际移动/脱离视野，也不能把本轮归为始终没有IK/规划通过的补视方案。

09:29:27 UTC R28已封口为 **complete/failure、recorder_closed=true**，recorder观察到active→inactive后正常关闭、`interrupted=false`，runner退出6、recorder退出0。bag为527,467,372字节，manifest登记SHA256 `bb5932dd295865da4c625202138fcf512ba7872748bb7d4affaa202b58bf0236`；本次文档更新只核封口小文件与bag大小，没有重新读取整包计算哈希。随后 `N/gen28_attempt_1_after_manual/result.json` 已确认generation 10、GUI自身手动ACK、任务/路由/动作inactive；恢复操作未发运动指令、未查询温度。GEN28源码和发布证据保持冻结，实物抓取仍未成功，后续处理继续遵守初始直接优先、证据不足才补视的要求。

| 重摆后评估与GEN28第1轮证据 | SHA256 |
|---|---|
| `N/gen28_repositioned_target_selection/result.json` | `ea1dc7a1049154a16b1380da76c1a5d374fca53160f8fccce1ab0899ee214d8b` |
| `N/gen28_repositioned_initial_readonly_inputs/result.json` | `a44c1e1a038740f579c2b68c6de98151b1c22713d8851c23b01b77bea1f335d7` |
| `N/verification/gen28_repositioned_initial_replay_r1/result.json` | `455a7b5f39032ad1f1ffbdae41bb471313ae2fdfd4b42ea6d4939d8c8f97abda` |
| 同目录 `raw_frame_qualification.json` | `03c722e507d60d98785f504a59a6215a4cd5f23d69092b49990bbdf8a7d3d7c7` |
| `N/verification/gen28_repositioned_rgb_direction_prototype_r1/result.json` | `e75b509d87f7a941a840f7ca2538c7298763fe9fe75978be520ceeefd88f1d97` |
| `N/gen28_attempt_1_scene_r2/operator_observation.json` | `6e2e41935b9af1335048ae5b35a1ff270bb107114954a667cb33f8736913639f` |
| 同目录 `resume_request.json` | `58a96aa1a81d23d3540578fc9e5d0bc51469a1b8e354bef42c56711a8a64bae3` |
| 同目录 `readiness/result.json` | `20fde2d7b28bbc2be4d87cda0000be89014b8133f8dacd951816ec960b3ce7f6` |
| R28 `manifest.json` | `5d1541752d7db0c4d1aacf8267d73bf4f8acc76e4705d45eaa6d8a9a386e5071` |
| R28 `command_result.json` | `4cefed723b8a750167104489ca2abbf037ca23427fc33fe2c26944ec6f2bccab` |
| R28 `recorder_outcome.json` | `4b9f7b5de45164041b7a3e0ae1e3642afeb7c751a8bebf83607f52499f4c6cec` |
| R28 `runner.log` | `ddd8aaca977e89ffa9589495a46c9905ccb230db83b3010881e8d3a854569940` |
| R28 `user_request_runner_exit.json` | `022eb79826c71ff0e69425dc637faee520fb37daa802d89084479cc658e15224` |
| `N/gen28_attempt_1_after_manual/result.json` | `20bf49bda49afc1cc610a32ab2d01e0c98d579e24253d860d2176677789accff` |

## GEN29 补视候选被等待通知覆盖：诊断、修复与定向验证

用户指出此类问题已经出现多次。本次先核对既往GEN16–GEN20和GEN27记录，历史比较见 [多次补视等待失败的历史对照](2026-10-15-repeated-observation-failures-review.md)。本轮request22的直接接触搜索因实测双侧证据及阶段倾角限制未留下局部合格候选；随后request23虽经历观察规划超时，最终确实选出严格路径通过的补视方案。它与GEN27第3轮后续观察搜索没有选出路径不同，也未走到GEN18/19曾经失败的frozen交接步骤。

根线程的封口录制诊断 `N/verification/gen29_recorded_handoff_diagnosis/result.json` 及同目录 `timeline.json` 明确绑定以下事实：

| 事件 | bag接收时间或原合同时间（ns） |
|---|---:|
| 上一完整READY，来源为1792056515347574234 | 1792056515841104030 |
| 补视preview `7e5fb7d5f533709420ca757a`，来源为1792056479674192428 | 1792056515920104980 |
| 上一READY原geometry截止 | 1792056516147574234 |
| 对应request23审计引用 | 1792056516155773401 |
| 下一READY，来源为1792056515813502311 | 1792056516258564233 |
| 无效preview `TARGET_PROCESSING_PENDING` | 1792056516269810438 |

因此公开候选窗口仅349.705458ms，约350ms；审计到达时上一READY刚过期8.199167ms。新READY随后已出现，但迟到的pending再次覆盖公开preview，之后没有新的valid补视preview，最终原90秒station期限结束。原审计的 `STRICT_SERVICE_SUCCESS`、`PREVIEW_READY` 与 `promotion=EXECUTION_FROZEN` 同时成立：活动任务不能靠预览自升执行权限，该拒绝码本身正确。bag不能暴露任务内部每次回调/线程调度；独立保留的原源码确定性回归在audit先到、pending后到以及READY恢复后audit才到三种顺序均复现候选丢失，不将bag时序夸写成内部调度逐步观测。

GEN29最小修复修改 `mode_aware_remote_grasp6d.py`、`station_compute_wait_remote.py`、`station_compute_wait_task.py` 和 `observe_return_task.py`，新增独立 `station_supplemental_admission.py`。观察用实际五帧snapshot的精确原始来源证明保存在审计内，消费者逐帧对照自己已登记的object/mask/status；沿用接触三帧规则的模块没有放宽为任意帧数。补视preview仅可作为私有计算结果保留；公开几何仍过期，typed clock通知不能授予运动。新完整READY到齐后通过独立候选选择入口重新进入原有audit、原来源时效、站位/epoch/所有权、路径和原子交接检查，未改0.8秒、90秒、120秒或任何物理余量。真实loss、来源矛盾、手动接管、换epoch/owner、deadline和实际执行发布仍撤销保留。

两处异步边界一并处理：producer在context RPC后跨原geometry截止，只保留无权限计算，释放输入锁等新READY；consumer收到audit.current但本地该帧仍未完成合法join时，仅在它原期限内等待，不能借缺证据准入。执行发布检查第一次拒绝后，本次调用立即返回原拒绝结果，避免第二次检查变为允许而绕过首条发布前的权限标志。原完整执行检查保留，没有为得到候选而放宽观察或接触条件。

验证分开保留：`gen29_supplemental_retention_red` 为3项全失败；`gen29_authority_first_check_red` 为13项中的2项真实权限竞态失败；最终 `gen29_supplemental_retention_final` 为 **343项全部通过、0失败/错误/跳过、source_unchanged=true**。末轮包含实际五帧fuser与完整ModeAware发布继承链、原子审计writer和任务audit回调、候选等待/恢复、实际执行入口首发送边界及原接触/站点回归。先前 `gen29_supplemental_remote_green_r1` 因新测试导入类名写错而collection error，原回执保留；修正的是夹具类名，不计生产缺陷。其他有交叠的定向计数不相加。此检查点完整组合、最终独立封存、部署及新实机验证仍待各自实际回执，未记为已经完成。

几何诊断与此交接修复分别保留。原近场帧1792056475938090324的真实盒形资格仍为 `BOX_TOP_NOT_RECTANGULAR`。未部署的有限RGB搜索原型即使找到四边，仍被 `BOX_MEASURED_GEOMETRY_CONFLICT` 拒绝；模型对照中993个current raw点指**当前三帧融合snapshot的实测点**，不是最后一张原始深度图的单帧点。该993点无超限，另1754个额外注册融合点中6点超限，最大2.591418mm。该tail还可能含当前view3的融合代表点，不能由prefix/tail位置断言这6点全来自更旧视角，具体视角归属仍待核。这只是原型诊断，未删除点、未放宽模型包容规则、未向执行提供合格模型，不能作为直接抓取可执行的证据。

| GEN29诊断/定向验证证据（均在N/verification内） | SHA256 |
|---|---|
| `gen29_recorded_handoff_diagnosis/result.json` | `5abf72749df3e7812a7cc1a6c5d26dd8f5e3e473fe83886001b4a1e2d3877999` |
| 同目录 `timeline.json` | `0d411b978381601dd47a1b2e7ec479bf74c098d67faa5f8121ea5aeec5421e47` |
| `gen29_supplemental_retention_red/receipt.json` | `1b3f381a10e1b87f16bac33564d1d9f99b950bb2b89196e6c9c33d42181de4be` |
| `gen29_authority_first_check_red/receipt.json` | `db8290a2a9587cbfb022f95d8bb2fd63588fa41b69ef0b1c4cc649df41a006b2` |
| `gen29_supplemental_retention_final/receipt.json` | `f0295fde73cf2a4b1aa5660d8bf3137bb153ad4913f3a4195aec68044fa15a99` |
| `gen28_attempt1_box_edges_r1/result.json` | `819e18274b65916c79599ec1c91ec884c662fc93eebae06d3c775637e036dbba` |
| `gen28_attempt1_local_rgb_edges_prototype_r1/measured_conflict_analysis.json` | `3ca4ca57f54d2c5958a5f722029bd206ccb331e8c9937caa83bd9c46f77d0788` |

独立最终回归 `gen29_supplemental_independent_final/receipt.json` 另有42项全部通过、0失败/错误/跳过、源码未变，包括audit.current早于本地完整join的两种顺序。原GEN28消费者红测中的6个失败为真实回调复现，另1个为foreign-track夹具漏同步geometry.track_id；context等待红测明确是内存反事实，不冒充旧完整生产快照。独立绿初轮的1项失败为float转ROS时间少1ns、尚未真正跨过期限；修正夹具为期限后1ms并断言实际跨界，保留原失败与更正证据，不算生产缺陷。

本轮修复与验证没有ROS/机械臂调用、温度查询、enable或失能动作。R28仍为已封口失败；没有接触、闭爪或实物夹持成功证明。项目文档与SanStand镜像由根线程按内容SHA256核验，镜像更新回执另存于N/verification。

## GEN29 完整组合通过后按用户要求暂停

完整组合 `N/verification/gen29_combined_r1/result.json` 已结束：234个测试文件、970份来源，7125项通过，0失败/错误/跳过，源码前后不变，GEN28原逐文件用例数完整保留。结果SHA256为 `63ba5bc62af0e838505b1e42f0ef09263c1cae43844e97327b5a4c4539c73c2b`。用户随后明确「完整回归结束后先暂停」，因此没有运行最终独立review封存、GEN29发布封存、部署或新抓取。review writer草稿保留，不能把它当成已出具最终review回执。

持续授权控制已设为paused revision27。收尾只读检查显示generation10任务、路由和动作均inactive，但GUI参数为manual=false；未推断其变化原因。随后使用既有恢复入口取得GUI自身latched true ACK，恢复手动，未发运动、力矩失能或温度查询。暂停检查点为 `N/gen29_paused_after_full_regression/pause.json`，手动恢复为 `N/gen29_pause_manual/result.json`。GEN28第1轮仍为历史失败，实物抓取未验证成功；等待用户恢复后，从最终审查/封存开始继续。

## GEN29 恢复、正式部署与第1轮实测：第二补视后轮廓身份丢失

用户随后明确「继续处理」。`N/gen29_resume_20261015T104448Z/resume.json` 记录恢复至持续授权 **active revision28**；上一节paused revision27是历史检查点，不再代表当前状态。随后最终独立review完成，`reviewed=true`、阻塞项为空；GEN29发布封存绑定 **970份来源、234个测试文件、7125项通过，0失败/错误/跳过**，原GEN28逐文件测试覆盖保留，源码前后不变。完整组合运行616.24秒。review和release中的 `runtime_deployment_pending=true` 是各自出具时的真实状态，后续部署成功由独立部署回执证明，未改写旧原件。

`N/gen29_deployment/result.json` 已确认 `deployment_passed=true`、`whole_patch_deployed=true`：三个计算节点重载，gateway由既有核心入口重启，7个硬件相关保留进程前后身份不变。部署没有发送运动或力矩命令，也没有查询温度。用户的初始直接优先要求保持：新选择先执行初始盒形/接触候选流程，初始证据不能满足才补视；没有因部署成功就把离线几何原型纳入生产或宣称完整抓取已成功。

本次有两份必须区分的录制。11:05:25 UTC的 `grasp_records/20261015T110525.253447Z_e1d666e812c3463a87244da38f9833ee`（R29-start）来自根线程现场准备超过90秒失败后误启动wrapper。runner因缺少该次 `operator_observation.json` 在ROS/动作启动前退出；根线程仅向只读录制wrapper发送SIGINT使其封口。原manifest为 **complete/interrupted**，runner退出1、recorder退出0、`active_seen=false`、`interrupted=true`；`startup_guard_stop_note.json` 留有操作说明。不能将这份中断记录改成机器人抓取失败、成功或正常任务终止，也不能把它当作下述实际运动轮。

真正GEN29第1轮于11:12:26 UTC启动，目录为 `grasp_records/20261015T111226.886093Z_0a4de2be16394344b0fa84a6ad2f8318`（R29）。runner新选择generation11，selection stamp为 `1792062749180071115`。本轮不仅完成第一观察站位，还实际执行了第二观察运动及端点修正；第二站位接纳的三帧为 `1792062834632175922`、`1792062834898471355`、`1792062835164456367`，两端qualified ACK齐全，任务在 `1792062835454827785` 进入合格站位，并记录补视采样完成。实机确实跨过了此前第二补视交接停滞的位置，但现有公开记录不能判明本次经正常public候选还是GEN29私有保留分支完成准入；不能据此宣称特定私有竞态已在实机逐步复现并验收。

第二站位确认约15.98秒后，感知producer首次锁定当前generation11丢失，理由为 `perception_failure: station_compute_recovery_rgb_identity_required`。首失source为 `1792062851088963747`、记录观察时间为 `1792062851418890476`；任务随后在 `1792062851457234144` 失败，消息为 `TARGET_LOST: producer reported target lost during handoff`，在 `1792062851463238000` 释放执行槽。bag开头generation10已锁存的旧支撑面失败被明确排除，不能混入本轮首因。

首失审计明确为 `SAME_STATION_COMPLETE_OUTLINE_MISMATCH`：当前候选mask对固定qualified-station末帧模板的IoU为 **0.810808807，小于0.85**。颜色距离0.217897585和轮廓距离0.158882933均低于各自0.4限制；失败不是这两项阈值。当前源在失败时年龄0.329840421秒，仍在0.8秒内，但处理已跨过上一READY的geometry截止 `1792062851369185018` 约49.619毫秒，此时恢复身份成为必需，之前可选检查中的轮廓不一致才变为锁存丢失。相同轮廓不一致在此前若干READY帧已出现。恢复检查先验证唯一候选与当前支撑几何，再到达这项RGB审计；本次直接拒绝并非支撑面阈值、registration RMSE或90秒station终止，不能把所有 `TARGET_LOST` 归为同一原因。

用户观察到侧面可见并询问灯光阴影。已按精确source导出RGB、原始深度、mask及同站位对照图；该站位180份接受关节样本的七维峰峰值均为0。画面存在可见目标不等于内部固定模板身份条件已满足，且失败发布的空mask是停止发布有效目标的结果，不是拒绝候选的原像素。现有证据只证明分割轮廓的重合不足，**没有建立灯光或阴影造成该变化的因果关系**；颜色阈值本身通过，也没有受控光照对照。导出结果没有重放资格，不能用显示图代替来源与身份验证。

R29已正常封口为 **complete/failure、recorder_closed=true**，runner退出6、recorder退出0、`interrupted=false`；bag为481,869,147字节，完整SHA已在离线诊断中重算并与manifest一致。未记录闭爪或抬升，没有实物夹持成功证明。随后 `N/gen29_attempt_1_after_manual/result.json` 确认generation11、GUI自身手动ACK、任务/路由/动作inactive；持续授权仍为active revision28。后续剩余问题是同站位分割/轮廓身份变化的具体来源及合理处理，而不是把旧交接缺口或光照直接当作已证明原因。本段只更新文档和外盘镜像/索引，不改源码、测试、控制文件或旧回执，不调用ROS或机械臂。

| GEN29正式验证、部署及本轮证据 | SHA256 |
|---|---|
| `N/verification/gen29_combined_r1/result.json` | `63ba5bc62af0e838505b1e42f0ef09263c1cae43844e97327b5a4c4539c73c2b` |
| `N/verification/gen29_final_independent_review/review.json` | `c2fc548d94005d8c1234f8b8175b772f93ffe5e9e422cdb35f49cf3359802657` |
| `N/verification/gen29_release_manifest.json` | `1551ea57d087b221fed4f6d59c8ce176c7a2e2d5398aa306829af3aa51506666` |
| `N/gen29_deployment/result.json` | `10beaf62b2f430b990c749802879fe18df32a5df67a03faf5ddee7599cbc56b5` |
| `N/gen29_resume_20261015T104448Z/resume.json` | `b7d7b84d8d9d2f2d1c499d28941f3e7c783e542d92a976d76bac5f10a627efd8` |
| R29-start `manifest.json` | `03b4837aed455b51d97681b846ffbb776cbf6222ff9cc2bf15b422cb3a3fed7e` |
| R29-start `startup_guard_stop_note.json` | `3a9a5e45ce31517e3c9bcf1e112907d8e6046cc36feae202bef28d3794987b39` |
| R29 `manifest.json` | `3a408e2957aeb399b9c8b3761a60dff9cd89bc616c26505e20a742140bdbfa9c` |
| R29 `command_result.json` | `ab580883dca7f324564a5950785853629b4cb9042eda736f5ad9e5baf3391a63` |
| R29 `recorder_outcome.json` | `18555adc2fc6aca05ae005c993ee91967d9e4ab80a9ce3c8f2a4c757489f6951` |
| R29 `keyframes.bag`（完整读回校验） | `e4393226560779b20f2140af13dd7ff89e4c8e1a7fe08e36cc20bb4658dab321` |
| `N/verification/gen29_attempt1_loss_diagnosis/findings.json` | `b09eedc841b0ceabd192a99aab7b578032d941ba47fd20903d69f2937489f9ae` |
| `N/verification/gen29_attempt1_shadow_frames/result.json` | `0fad4e0e41bfe967131b6c09bf9bc33df575d3d045c111a6055fa70d05093c23` |
| `N/gen29_attempt_1_after_manual/result.json` | `92046839e657e26d37f5510a355915ae4e1ac50971fd21389a28955e0165231b` |

文档更新前副本、修改前后SHA、外盘镜像逐字节核对及证据绑定回执保存在 `N/verification/gen29_deployment_attempt1_document_update`；以前暂停与未部署段落保留为历史检查点，由本段更新当前状态。


## GEN30 诊断检查点与按用户要求全栈重启（2026-10-15 本地 / 10-16 UTC）

用户在GEN30离线诊断期间要求「先暂停」，控制记录为paused revision29。`N/gen30_user_paused_20261015/idle_check/result.json` 当时确认GUI自身手动ACK、任务/路由/动作inactive，但没有取得新的接受关节样本，`passed=false`、`accepted_joint_count=0`；因此该暂停检查不能证明实物关节稳态。其原结果保留。用户随后明确「你重新启动各节点和上位机，然后继续处理任务」，并在设备连接问题 `call_086da6aaebce404a88b1a18faeca26f5:0` 回复「三项设备已连接到 Ubuntu」，持续授权已恢复为 **active revision30**。设备连接回复不是新的清场声明。

新运行会话C为 `/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen29_restart_20261016T000728Z`。SanStand、D405及机械臂串口恢复连接后，按现有正式GEN29发布重启节点和上位机；本检查点970份正式来源与GEN29封存一致，**GEN30尚无生产修改或部署**。`C/cold_restart_acceptance/result.json` 确认11个核心节点均为新进程、整套发布验收通过，20份真实SDK关节反馈有效，新epoch为 `1792109290479761277`；相机、GUI、控制器及仿真服务通过对应检查。7125项测试是此前已封存的GEN29组合结果，本次没有把它改称重新运行的GEN30组合。新generation1和旧generation11属于不同运行会话，不能混用旧姿态、目标关联或现场图。

重启后的首次附加策略读取发现manual=false，未推断原因；随后仅在任务空闲时使用既有恢复入口取得GUI自身latched true ACK。`C/restart_policy_r2.json` 与最终 `C/idle_gui_camera_r3/result.json` 复核manual=true、task IDLE/inactive、同戳RGB/depth各5帧更新；最终帧年龄约42ms。配置继续为无需实测双侧接触、最多2次观察、闭爪后不抬升，软件温度监测关闭、轮询率0；没有查询温度或发送关节失能命令。节点启动沿用已有正向enable配置，不能把“未启动抓取”写成“启动过程没有任何硬件配置动作”。最终图左上仍有手臂，未取得该次清场证据；没有启动新抓取或发送新的运动目标。运行指针由根线程finalizer更新，`C/finalization.json` 明确 `gen29_deployed=true`、`gen30_deployed=false`、`physical_grasp_verified=false`。

GEN30先将初始直接抓取与第二补视身份失败分别诊断。原R29初始request1的三张精确RGB/depth/mask与TF，经原fuser和盒形资格入口有限重建，三帧仍为 `BOX_TOP_NOT_RECTANGULAR`；重建几何与原trigger数值一致，但原request1完整资格审计、准确buffer成员和当时全量有效参数未保留，不能把重建冒称原运行逐步复现。末帧顶面RMS约0.617mm，仍因四边/角支持不全而拒绝；原视觉矩形搜索的一个外边组合角覆盖0.48低于0.5，内侧印刷边组合另受外侧顶面延续检查拒绝。此处尚未走到最终定位误差、开口预算或完整路径验证。追踪没有改变原资格结果；其中多行条件的line事件可重复，不能将事件次数当唯一候选数。后续应解决有证据的边界定位与误差预算，不以单独降低阈值宣称可直接抓取。

第二补视的三帧当前支撑面诊断调用原 `refine_candidate`，并核对同源 `_FrameRefinement` 缓存路径：原合法匹配fragment2的mask均逐字节复现，固定模板IoU依次0.808091、0.808370、0.810809，仍低于0.85；它们的可信侧面种子数为0。同帧另一个原几何不匹配的fragment0独立细化保留972–1044个可信侧面种子，产生轮廓IoU 0.958188、0.951651、0.993486，并包含原匹配片段的全部可信核心。两种调用方式输出相同。这支持继续研究当前帧片段到完整实例的归属；**该fragment0仍只是诊断假设，原几何匹配资格未被绕过，也未按旧模板IoU挑选它准入**。不合并旧mask、不改当前支撑面、不放宽0.85，单次耗时也不是受控性能基准。三帧带已说明的prior-position代理，未重演完整异步producer或deadline权限。

截至本检查点，当前帧完整实例归属设计和独立审查仍在进行，GEN30修复、回归、发布、部署及新实机验证均未完成。当前继续使用GEN29保持手动空闲。灯光/阴影可能影响视觉的假设仍未建立本轮因果；原R29保持已封口failure，没有闭爪、抬升或实物夹持成功。继续遵守初始视角直接抓取优先、定位证据不足才补视；新运动需重新核现场与新运行来源。

| 本检查点主要证据 | SHA256 |
|---|---|
| `C/cold_restart_acceptance/result.json` | `cb40c9e96179d86aafdec1061fd991808dddbe96e68583be75c7c48f441b2c91` |
| `C/manual_restore_after_restart/result.json` | `92d6efd3ca7ed549cdc75d1205acedad03ae6567dd18edc28d466a493662146f` |
| `C/idle_gui_camera_r3/result.json` | `6e2bcc9b078d33be1b385ebff464e4bde3216c292b6d165ac3b3ca8f09fa299b` |
| `C/finalization.json` | `6e07f6d268920f5a3d1d36dbba5e9178db5ab2258769eecdb00dccaf7d499195` |
| `N/verification/gen30_initial_view_box_replay_r1/result.json` | `8334b360c1af652b735c4bf51b89e55a5a5b26ddd0ebc5ba9ce1d783c81f56ce` |
| `N/verification/gen30_initial_view_visual_trace_r1/result.json` | `7e7012a0303016b0ed5b9e49cf9587bb11bf5be57e677ac5e5301d6a464cb53c` |
| `N/verification/gen30_current_fragment_ownership_r1/result.json` | `1ca314dc192d71755f6152f278078cf4112974126597523b15389de593e12f40` |

文档前文的revision28及旧会话状态是历史检查点，由本节更新。文档前后原文、证据SHA及镜像读回核验保存在 `C/document_update/receipt.json`；本次文档更新不改源码、测试、运行指针、控制文件或旧证据。


## GEN30 源码边界绑定与完整回归启动检查点（2026-10-16 00:57:49 UTC）

GEN30当前帧完整实例归属、逐原始融合样本的显式边界声明、规划/接触边界消费已写入本地源码；上节“尚无生产修改”仅描述重启时点。相关工作流定向验证353项全部通过，含66项独立快照绑定用例；这不证明原失败帧已经恢复，也不证明实物抓取成功。GEN29节点及上位机的全栈重启已验收；本检查点仍运行GEN29，GEN30没有部署，本轮离线修复没有启动新运动。

完整回归已在 `N/verification/gen30_combined_r1` 实际启动，范围为979份源码、246个测试文件：原234个文件全部保留，另加7个此前只被源码哈希覆盖的已修改历史测试文件及5个新测试文件。首次入口在scope预检断言处退出，未跑测试、未创建组合结果目录；修复后runner和sealer共同使用上述并集，并继续要求每个原文件的用例数不减。独立静态审核无未解决阻塞项；它仅审核范围修复，不是GEN30正式发布批准。**截至本段时间，完整回归仍在进行，尚未生成 `result.json`，未宣称组合通过或发布完成。**

实际历史第二补视三帧经当前完整归属helper仍全部返回 `current_fragment_outline_interior_conflict`；远侧片段的边缘矛盾尚未解决，不能丢弃矛盾点或以先前片段IoU较高视为恢复成功。当前已核对的原初始三帧仍为 `BOX_TOP_NOT_RECTANGULAR`，尚无合格盒形与完整定位/开口/路径验证结果。初始直接优先、精度不足再补视的要求保持；阴影因果仍未证明。

证据：`C/verification/gen30_boundary_workflow_r5/result.json`（353通过）；`C/verification/gen30_runner_scope_independent_review_r1/review.json`（runner/sealer范围静态审核）；`N/verification/gen30_scope_r1/preflight_failure.json`、`scope.json`；`N/verification/gen30_current_fragment_owner_r3/result.json`（三帧拒绝）；`N/verification/gen30_initial_view_box_replay_r1/result.json`（初始三帧资格）。本次前后原文、证据哈希及外盘镜像读回核验见 `C/document_update_gen30_regression_running/receipt.json`。


## GEN30 完整回归失败定位、定向修复与剩余几何事项（2026-10-16 01:21:16 UTC）

上一节“组合进行中”保留为历史检查点。`N/verification/gen30_combined_r1/result.json` 现已结束：979份来源、246个测试文件、**7476项，7303通过、173失败，0错误/跳过**，不能封存为通过。逐失败入口核实后，共享station造帧的主要拒绝为 `CURRENT_INSTANCE_BOUNDARY_ORIGINAL_OBJECT_MISMATCH`：旧fixture只更新ObjectPose外层stamp，pose_base与pose_camera仍保留旧stamp；同源READY缓存实际已齐全，不能把这类失败误报为缺status。另一direct-buffer fixture缺真实pose headers且未递交同源READY，拒绝为 `CURRENT_INSTANCE_BOUNDARY_SOURCE_BINDING_INVALID`。补齐后，完整worker测试另暴露CPU mock跳过真实planning检查，异常为 `prepared request lost its boundary geometry checks`。

仅修改4个旧测试文件：`test_observation_measurement_remote.py`、`test_station_compute_wait_remote.py`同步合成同源pose stamp；`test_remote_modes.py`改用真实ObjectPose并逐源通过完整status callback递交显式null声明；`test_station_compute_work_handoff.py`的CPU mock调用真实 `_prepare_regular_box_snapshot`。没有修改生产binder、放过缺声明或手造audit。原失败10个全文件及两个共享helper所在全文件，共 **500/500通过**，逐文件用例数与完整r1相同；四个变更文件均已属于原234文件范围。证据为 `C/verification/gen30_station_fixture_regression_r2/result.json` 与 `change_receipt.json`。**截至本检查点，修复后的完整组合尚未重跑；500项定向通过不能替代全scope验证或GEN30发布批准。**

历史第二补视三帧的片段拓扑已获得新同源证据。`N/verification/gen30_rectification_topology_r1/result.json`按各帧历史rectification合同重建映射且合同逐项相等：每帧42条目标跨片段对角边的两端均来自矫正前、SDK已对齐到彩色图的source栅格4邻接；矫正后两处正交中间格均为结构性未映射画布，不是可据以证明断开的实测背景。当前measured mask未改、无深度填充。它只解释采样拓扑，尚不能证明单一实例归属、采样之间都有深度、盒形资格或运动许可；不能把source栅格称作未对齐原生深度传感器栅格。基于该来源绑定的实现正在进行，尚未发布；此前三帧 `current_fragment_outline_interior_conflict` 的原结果保留，未改记成实物成功。

初始直接抓取的视觉区间诊断也保留未完成结论。独立审核指出r1的短Hough端点误差区间不足以声明外推整边的完整包络，单个名义边的外侧深度反证也不能排除整个不确定参数族；现有通过候选角覆盖均不低于0.8，取消0.5角门并非这些候选成立的必要条件。r2 `C/verification/gen30_current_interval_family_r2/result.json`撤回这种完整性推断，保留当前第三帧全部24个有限候选族，扩大到整边及全部原mask实测/未知像素包络；该保守诊断要求开口约99.239mm，超过50mm，仍为 `PRECISION_REQUIRED_NOT_PRODUCTION_QUALIFIED`。它不证明24族都物理可行，也未解决真实RGB重采样的总误差保证或有限线组以外的完备性；没有创建生产合格模型，也没有提升第一/二帧失败结果。后续仍需定位误差和全族反证，原先单候选数值不得作为直接抓取准入依据。

运行状态仍是GEN29。`C/idle_during_gen30_repair_r1/result.json`的新只读检查通过：GUI自身latched manual=true、任务IDLE/inactive、generation1，RGB/depth同戳且各5帧更新；模式仍为waiting_for_target，状态说明为target_lost_requires_new_generation。该检查没有新运动、没有启动抓取，也未声明现场清场。图像复核见左侧桌沿外一段手臂，不把未见手部在桌面当作新运动清场凭据。GEN30未部署，无新增实物抓取成功证明。本次只追加文档与外盘镜像，未改control/progress或运行指针；前后原文、上述证据SHA及镜像读回核验见 `C/document_update_gen30_fixture_repair_checkpoint/receipt.json`。


## GEN30 最小发布范围、预热兼容修复与完整 r3 运行中（2026-10-16 01:59:13 UTC）

本条为最新检查点。前述 partial-outline/boundary、旧 fixture 修补和完整 r1 结果保留为历史研究与失败记录，不能当作本次发布范围或通过结论。下文 N22 指 `ros_log/full_latest_gen22_restart_20261015T003700Z`，C 指 `ros_log/full_latest_gen29_restart_20261016T000728Z`。

最终范围收缩为：修改 `unknown_tabletop_perception.py` 与 `perception_modes.launch` 两个既有生产文件，新增 `source_sampling_topology.py`、`source_connected_owner.py` 两个纯计算模块和四个 GEN30 测试文件。未发布的独立 partial-outline 归属容差、边界协议及其 snapshot/remote 消费链已按精确副本和 SHA 归档退役；相关既有文件恢复 GEN29，原 234 个测试文件没有修改。盒形、定位误差、接触、碰撞、执行与闭爪检查仍沿用 GEN29，不将视觉边界区间 v2 研究混入本次发布。退役回执在 C `verification/gen30_boundary_scope_retirement_r1/`、`gen30_boundary_scope_retirement_r2/`，独立复核为 `gen30_boundary_retirement_independent_r1/`。

已证实的根因是：历史第二补视三个源各有 42 条跨片段画布对角边；精确原 SDK 2.56.5/profile/forward-map 证明端点在校正前实测栅格四邻接，两个正交中间格属于无源映射的画布，不是原深度缺测。新分支只在原同站、唯一原始匹配且 primary 固定模板不匹配时恢复该当前实测组件；保留原 8 mm 深度边、可信核心、RGB/质量/anchor/last/extent 与固定 0.85 模板门，不做裸画布八邻接合并、不填深度、不借历史像素生成几何。相机 SDK 依赖由既有隔离 wrapper 提供，映射后台预热；未就绪或契约不匹配不能准入恢复。

| 历史源 stamp_ns | 原 primary IoU | 恢复后 IoU | 原入站年龄＋离线计算（秒） | 普通几何 | 原盒形资格 |
| --- | --- | --- | --- | --- | --- |
| 1792062835497817993 | 0.808091 | 0.954378 | 0.380835 | 通过 | `BOX_TOP_NOT_RECTANGULAR` |
| 1792062850569185018 | 0.808370 | 0.951508 | 0.644289 | 通过 | `BOX_TOP_CONTOUR_INCOMPLETE` |
| 1792062851088963747 | 0.810809 | 0.990412 | 0.458921 | 通过 | `BOX_TOP_CONTOUR_INCOMPLETE` |

以上是最终 helper 的有限真实输入回放，N22 `verification/gen30_source_connected_owner_r4/result.json`，SHA `3960a07e86f06ba3216ed9d9f69011218d2d40420ad564dcfb7737dc3c5cec1f`。整段分割、primary、恢复、最终模板 wall 为 0.346162/0.353667/0.348750 秒；年龄模型不含完整 ROS 调度、解码、TF 与发布，不能代替实机时效验收。三个原盒形门仍失败，且回放仅保存最终 code，不能从后两个同码推断其确切内部子原因。恢复身份/OBB不是盒形合格、一次抓取完成或实物成功。

邻物反例是历史实录上明确叠加 144 个独立同色邻物实测像素的合成输入，不是新的物理抓取实验。原分割与原 RGB refinement 实际运行，primary IoU 0.799413 确实进入新分支；完整 RGB 结果原可凭 IoU 0.944501 通过模板，但新检查从完整当前 raw 前景中发现独立 source4 组件，拒绝 `current_fragment_foreign_raw_source_component_overlap`。证据为 N22 `verification/gen30_reachable_source_neighbor_r3/result.json`，SHA `6cb0b551ccade8610f69f20e0bc311d5ea28d5990dfd1a806f50b3b3067cb473`。额外 raw-only 区域按原 source4/8 mm/min60 分开计算原可信核心，仅用于桥反证，不加入目标 proposal；低于原最小证据量及原普通路径已吞并窄连接邻物的歧义仍未证明解决。

完整 r2 真正结果为 **7149 项：7144 通过、5 失败、0 错误/跳过**，N22 `verification/gen30_combined_r2/result.json`（SHA `863e80c39fe08518bc6125383c996e97150c1340e0b911e3b4bd1aadb9a54289`）原样保留。五项均为原 `test_measured_face_freeze_lock_independent.py` 首次锁定未建立：新预热无条件调用 `cache.offer`，将恢复专用的 `color_intrinsics/sdk_version` 要求泄漏到普通识别。合法旧 identity rectification 契约被提前报 `current_source_topology_profile_mismatch`。同原 fixture、同其余模块只换精确 GEN29 node 后五项全部通过，因果对照为 N22 `verification/gen30_optional_prewarm_cause_r2/`；不是线程偶发，也没有用补齐 fixture 掩盖生产变化。

最小修复仅在 node 对可选 `cache.offer` 捕获 `ValueError`。原 `require_rectified_source_after_activation` 留在此 try 之外，恢复 `cache.require` 继续完整重验；即使缓存中已有旧 ready map，缺少当前证明仍不能恢复。原五项测试字节与 GEN29 完全一致，新增三项覆盖无缓存/旧 ready 缓存下普通路径可继续而恢复仍拒绝、原 activation 失效仍阻断。原五项＋新增三项＋既有 GEN30 二十四项 **32/32 通过**，N22 `verification/gen30_optional_prewarm_fix_r1/result.json` SHA `ebf2b9c4f9b1b7372db939d88cfebf3bf54dbbdead2f3cc1b7fc0709eb9c2ac0`。冻结 node SHA `74165c24f1e99c32b1f351628f37b482251acded448f3be105b12c6c7e6e17ff`；两个恢复 helper 与 r4/邻物回放保持同字节。修复差异及前后 SHA 见 `gen30_optional_prewarm_freeze_r1/receipt.json`。

完整 r3 已实际启动，范围 **976 份来源、238 个测试文件**，scope SHA `ea46204fd42dddb6ed240d184b67665a6ba591deae70568dc15ce2314301cbfb`；本检查点尚无 `gen30_combined_r3/result.json`，只能记为运行中，不能记为完整通过、发布批准或部署。后续结果应另追加，不覆盖 r2 失败。

重启后的当前初始视角仍是未选定身份的离线分析：原 RGB/融合/普通 OBB 通过，三个源盒形均 `BOX_TOP_NOT_RECTANGULAR`；上方两个可见角的 mask 内原始深度持续缺失，视觉矩形候选也未通过原角覆盖/外带同顶延续门。相关 `gen30_current_passive_initial_r1`、`gen30_current_mask_edges_r1`、`gen30_current_visual_trace_r1` 位于 C/verification；近场可合法使用 3 帧，不能笼统称现场必须 5 帧，具体更正见前者 `effective_snapshot_count_clarification.json`。下一轮实际结果须保存完整 box reason/audit，并在模型合格后检查夹取方向不确定性和开口余量。阴影因果未验证，不将其记为本轮已证实主因。

当前运行仍为 GEN29；最近保存的只读检查 C `idle_during_gen30_repair_r1/result.json` 为 manual=true、IDLE/inactive、generation1、waiting_for_target，相机继续更新。本轮修复与文档工作未发起新运动、未取得新清场凭据，GEN30 尚未部署。汇总诊断为 C `verification/gen30_final_diagnosis_r2/diagnosis.json`，SHA `d70f277d4dc11039750c1ae409e1e95af5e612562af0fc247a5cc77f24d19654`；它是诊断，不是执行授权。文档及镜像前后 SHA/读回回执为 C `document_update_gen30_scope_r3_running/receipt.json`。


## GEN30 完整通过、部署及实际第 1 轮失败；GEN31 恢复检查点（2026-10-16 02:53:39 UTC）

上一节“r3运行中”现由已封口结果更新：完整r3为 **7152项全部通过，0失败/错误/跳过**，976份来源、238个测试文件，原234文件的7125项逐文件原数保留。独立审查与正式发布已完成；完整r2的7144通过/5失败及兼容性修复证据仍按历史原意保留。正式发布为N22 `verification/gen30_release_manifest.json`，SHA `c6554435fafe0589e73910e928439204654cd6e7edba993ad7ecd7df509f837f`。随后C `gen30_runtime_deployment.json`（SHA `cf826a271a2252f4d340d0d2404e5af0631604d8166241031ad99f34529e7f3e`）确认三个计算节点换为新进程、八个硬件/GUI进程保持，完整部署通过；`gen30_deployment_finalization.json`已完成元数据交接。发布文件内“runtime pending”描述封版时点，后续部署结果单独保存，没有回写旧证据。部署本身未发运动命令，不代表随后实际实验没有运动。

实际GEN30第1轮R30为 `grasp_records/20261016T021226.502516Z_0494c8bc84a34a74933e8eae122dd1f4`，新selection generation2。第一次补视已实际运动并取得合格观察站测量；之后规划第二次补视，未生成成功的第二观察动作，最终返回 `TARGET_LOST: station_compute_wait_expired_or_future`。R30已complete、failure、recorder/bag关闭，runner6/recorder0；未闭爪、未抬升，无实物夹持成功。启动沿用持续授权和当次精确新图复核，历史清场回复明确标注为历史，不冒称新的用户确认。

本轮初始源 `1792116749240888833` 没有留存原box完整audit；已保存的request1普通实测接触检查共同高度为null并转补视，不能把后来第一次补视的盒形原因回填成初始原因。第一次补视源 `1792116802511774778` 则有明确在线 `BOX_TOP_NOT_RECTANGULAR`：顶面矩形填充0.986556、平面RMS0.5495mm；一个角到最近合格顶面点1.62063mm，超过1.53928mm容差0.08135mm。`corner_coverage=0.473574` 实为 `1-distance/(2*tolerance)` 距离评分，不是只有47%角点像素可见。四边支持为2/10、10/10、10/10、6/10，原要求每边至少7/10。上边中间8个空段含762个mask内像素，raw深度全零；右边4个空段含200个有效深度点，但低于拟合顶面约1.524–2.198mm，被原±1.5mm顶面带排除。清楚的RGB轮廓与这些深度支持不足可以同时存在，不能仅据RGB认定角/边已获得足够度量定位。

纯离线资格重放复现上述在线数值至1e-12；标定参数在录制元数据中稳定，但元数据并非逐图同步。现有视觉矩形回退也没有通过：外沿线组的左下角5×5邻域只有2个mask像素，对应评分0.32；另两组内侧线被外侧同顶延续1.0反证拒绝。这不是单改角阈值即可证明可抓。证据为C `verification/gen30_attempt1_closed_evidence_r1/first_observation_box_trace.json`（SHA `37e6fc40f9d8660cb4581539b007772a45803edb18a73526f9aca0517e746a8d`）及 `first_observation_box_exact_overlay.png`。本轮591条感知status未记录source-connected owner恢复成功审计，只能记“未记录成功”，不能据此断言恢复从未尝试。

等待失败的有限时序与历史GEN29轮廓身份失败不同：约90秒的规划阶段内有241条ready/tracked，源年龄最大0.4691秒、固定模板IoU最小0.9285，最后有效身份仍新鲜。两个规划搜索区间内共39次 `execute=False` 严格位姿检查全部超时并记录goal sampling失败，累计63.509秒；每次规划器报告约1秒，检查总墙钟约1.62–1.64秒。首次lost出现在phase deadline之后约0.0596秒，不能把终态TARGET_LOST直接解释成图像丢失。检查与request的归属依据不重叠日志区间；超时不能证明候选几何上不可达，首个检查前约10秒的未分解时间也不归因于特定CPU步骤。依据为C `verification/gen30_attempt1_timing_diagnosis/observed_timing.json`（SHA `efbca64b643b6f42fed7f648eaa1cf80c23a10b777bd01845bab82833df059f1`）及其提取回执，仍需修复并验证可达候选选择及期限结果分类。

实验后C `gen30_attempt_1_manual_restored/result.json`取得GUI自身manual=true ACK；`gen30_attempt_1_idle_final/result.json`通过20份真实关节样本、部署进程与976源核验，任务为FAILED终态且inactive，不能写成任务状态IDLE。随后用户“先暂停”保存为revision31，`pause_after_gen30_attempt1_20261016/checkpoint.json`保留停止时点；用户最新“继续处理”已由 `gen31_resume_20261016/resume_receipt.json`记录为active revision32，`idle.json`记录手动、FAILED/inactive、关节反馈年龄约4.5ms。这些是各自采样时点的状态；恢复未声明新清场、未发运动。GEN31正继续定位误差预算与补视可达性设计，尚未验证修复或实物成功。本次仅文档/镜像更新，未改控制、运行指针或生产源码；修改前原文、证据SHA和镜像读回见C `document_update_gen30_deployment_attempt1_checkpoint/receipt.json`。
