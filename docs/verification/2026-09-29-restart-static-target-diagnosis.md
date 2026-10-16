# 再次节点恢复与静态识别对照（进行中）

用户要求恢复误关闭的所有节点，并完成未解决的抓取问题；随后确认已重新对准目标。本轮 UTC 2026-09-29（本地 09-28）记录位于 `ros_log/restart_grasp_20260929T013841Z`。全部新实验写 SanStand，local_only。

## 恢复结果

既有完整 start_ros.sh、只读监督和 handeye observer 三服务恢复，驱动、控制器、GUI、相机、MoveIt、感知与抓取节点在线。新 driver epoch 1790645893464309304；状态实读 CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE、motion_enabled=true、feedback_ready=true、protection_latched=false。没有发送失能命令。初始实测接近零位，用户经GUI调姿后电机已居中；旧运行实例的关节或回程计划未复用。

首丢锁诊断补丁随启动加载；它保留第一次原因、图源、anchor和候选关联证据，不改变原丢锁行为。lift整轨迹修复源码/新服务亦已加载，但 unknown_lift_cartesian_compensation_enabled=false，所以仍未启用；MuJoCo提升非硬门、真实提升启用保持原配置。源码摘要见 loaded_sources.json。

## 本轮已确认的首次丢锁

启动generation1在用户手动调姿中首锁（源1790645963322026252），之后源1790645966366947412首次出现 support_plane_local_distance_inconsistent：6.867mm > 4mm。停稳后仍约7.1mm。此为正面支撑不一致，不是已证明的TF暂缺或输入超时；不据此放宽阈值，也不称已确认某个机械部件松动。

用户已确认调姿完成后，建立仅用于静态识别的新generation2（1790646086790433645），完整保留gen1和此前空抓/11mm支撑故障引用，见manual_alignment_static_transition.json。此边界不是旧故障已解决，也不授予旧计划运动。

## 静态对照

unknown_first_loss_readonly_20260929T014040Z_13011保留旧gen1首次故障，200/200检测源RGB-D。第二袋014127Z_14301的221次状态全部tracked，约44秒无新丢锁；219/221检测源图保存，最初两帧因订阅握手缺失已明确列入report，不伪称完整采集。首锁三帧早于第二袋，不以此袋验证首次采集。静止后支撑局部差约0.1mm量级，完整统计另附。

## 新预览与未完成工作

仅开启现有WSL GraspNet推理并读取新预览，没有发送抓取启动/关节/夹爪命令。自动审批最初因远端图像传输拒绝；核对用户原WSL授权、原地址172.23.132.97:8000和既有启动记录后，同一预览请求获准，没有换入口绕过。预览current_grasp_preview_20260929T014236Z，plan4ff477e7b1fea512209c448e，CONTACT_EXECUTION_PLAN，3源帧跨度398ms。整体约29.34s、WSL约2.21s，仍需核对较大姿态变化及漂移。

原约10–11mm偏高尚未修复，本轮尚无抓取成功证据。下一步审核新路径与真实失败域的冻结参数候选回放，不能把静态识别稳定或规划通过当作物理高度正确。

## 已授权启用提升修复

用户明确回复“允许启用并保存修复”后，运行时 `/grasp/unknown_lift_cartesian_compensation_enabled=true`，现有主会话 resume_modes.launch 也新增 true override；保存修改前副本，参数回读和授权记录在lift_configuration_enabled.json。任务inactive，无运动请求。前述“已加载未启用”是启动时状态，已由本节更新；尚无实际提升验证。原仿真提升非硬门和真实提升启用不变。

静态221帧全段统计进一步确认：44.8368秒内六轴actual范围全0，独立支撑审计全部有效，参考局部差最大0.267mm/中位0.103mm，法向差最大0.286°，平面RMS0.717–0.820mm。这不证明跨姿态链准确。

## 最终确认图像时序修复已加载

`_confirm_final_center_alignment`现在等待到达后新鲜同目标对象，显式传入near-field reference；等待、phase切换、请求和多帧确认共享原截止时刻，不重复续时。292项相关回归通过，根独立复审。仅重载空闲任务，PID8794→29186，驱动8750及epoch1790645893464309304未变，使能仍true；selection保持generation2，外部/grasp/start仍归router，内部/grasp/internal_start归task。该修复不是已证明的旧11mm根因。

## 扩域运动学/手眼候选与新独立验证

原22训练的J2/J3姿态范围仅4.043°/3.516°；真实空抓变化70.137°/78.926°。原冻结22训练10参数候选在空抓留出中把−10mm变成+7.5mm，不能采用。预先固定扩域拆分后，仅用22原training加23原task多帧（这些task明确改为training），同10参数、同边界/权重/零初值拟合一次。8原独立组、旧空抓和J1/J2/J3居中均不参与拟合。扩域候选的旧空抓6阶段18区域Z漂移最大0.911mm；对原训练固定boardY的8独立组RMS/max为0.592/0.823mm和0.140/0.257°，过原3/5mm、1/2°验收。独立复审补全共视patch连续域上界2.541mm/法向1.046°，仍低于4mm/4°。

旧空抓已参与模型开发判断，因此不称全新盲测。候选尚未部署，也不把拟合偏置称真实电机零位。候选URDF+X成对文件位于旧resume120606Z/expanded_metric_training_once/candidate_model，1114合法q验证新URDF(q)=旧URDF(q+offset)通过；raw命令/轴/限位/网格未变。新manifest/回滚副本已备。

用户随后确认新标定板已固定并可见，root记录新板图/quality：88角点、约0.116px RMS。新布局与旧board绝对位置不混用，候选保持冻结。两次J4只读准备均0命令：020253Z因未取得直控所有权；正常空闲交接后020336Z因原全响应区间的整板边缘下界−5.009px<20px退出（CAD净空59.294mm）。正在检查精确逐轴包围与其他方向，原物理边界不扩大。


## 新板有限取景：真实边距不足，仍零动作

新增 `.runtime/bounded_board_centering.py`，从当前板几何中心一次求解 J1/J2/J3，依序 J2→J3→J1，每轴最多一次、命令不超过3°；复用原 CenteringContract 的 actual 初始±4.8°、独立 held0.4°、reverse0.5°和单轴 SDK 回执。原40mm CAD/板面、20px整板边缘和120mm深度要求不变。记录使用已验证的有界异步writer，首命令前3秒及每步停稳后3秒保留精确同源数据。新client独立复审通过，20项新增测试通过，相关复用回归合计163项通过。

只读实机准备 `bounded_board_centering_20260929T022440Z_98969` 没有创建命令发布器，commands_published=0。第一步连续界为66.528mm CAD净距、16.916px整板边距，正确拒绝。随后对实际SDK投影核得域内反例：first J2 Δq=[−0.4,+0.5,+0.4,−0.4,+0.4,−0.4]°，整板边距17.1323px；first J3为16.3057px。这不是Taylor细分保守，继续细分不能使同一响应域通过。整板在该反例仍在图内，但未达到既有20px预留；不把它夸称已发生出画或机械臂故障。

已向用户请求只将板调整向画面中央并与电机错开，电机保持原位；当前等待回复。若移动板必须重新建立固定板基准，不能将旧/新绝对board位置混比。原始标定候选仍冻结未部署，当前运行URDF/X/相机参数不变。本轮未发任何自主关节、闭爪、提升或失能命令。

## 新数据评估与成对部署准备

`.runtime/evaluate_prospective_metric_candidate.py` 冻结SHA `42295547dd7aadfc11df2ff75c3a99212f8fa50a367bd667f67fa083f6c21bcb`，29项测试通过。只读closed bag，不拟合；同袋首命令前数据建立各模型自己的固定Y，同袋后续和后续袋均用此Y。独立800ms actual停稳、同源RGB/quality/board TF、原K/URDF/X/epoch绑定；每条真实command都列覆盖，缺帧或异步归档失败明确标partial/incomplete。旧只读基准仅4个合格帧，被标baseline_only_not_validation，不能称多姿态通过。

`.runtime/bounded_wrist_async_validation.py` 薄包装复用原 wrist.path/run 和原对称响应盒，仅使用新异步记录器、3秒基准，保留原85秒动作期限；当前没有执行。

项目 `.runtime/candidate_models/expanded_metric_20260929/` 暂存候选URDF、X、配套handeye config及参数launch片段，均未接入运行或持久启动。审查 `.runtime/expanded_metric_model_deployment_dependency_review.md` 确认driver/HW/ros_control不使用origin修正SDK角度，可保留运行；新模型需重建RSP、MoveGroup、唯一handeye TF和感知/规划/任务缓存。当前MoveIt world/attached清单均为空，见predeployment_scene_inventory.json（只读时点观测）。

另发现WSL MuJoCo读独立MJCF，不能只换ROS模型；正在核查实际服务坐标映射并派生配套候选。此为部署一致性问题，尚不能断言是旧抓取失败根因。


配套MJCF待审查产物已生成：`.runtime/expanded_metric_mjcf_review/four_body_offset_candidate_NOT_FOR_DEPLOYMENT.xml`，生成器`.runtime/derive_expanded_metric_mjcf_review.py`；报告在旧resume目录 `expanded_metric_mjcf_review_20260929/report.json`。仅4个body局部旋转变化，不改变raw q、原物理、网格、摩擦或执行器参数。1114组q验证新MJCF(q)=旧MJCF(q+offset)到浮点精度，原server夹爪合同通过。旧server用两指body原点均值作TCP，与URDF tool0有约0.505mm既存差；固定末端映射后残差约0.009mm。这不是新增10mm误差，也未据此发明严格数学identity运行硬门。旧MJCF J4较URDF窄的限位仍需在实际规划范围核对，不能扩大硬件限位。该产物未部署；新独立多姿态数据仍待现场板居中。


## 02:43–02:54 UTC 自主多姿态采样实际执行结果

用户指令“继续自动调姿采样”后，新图像的板中心由约(351,315)变为(349,251)。建立新的固定布局 `fixed_board_20260929T024344Z`，不与前一板位置混比。只读基准 `bounded_board_centering_20260929T024344Z_146780` 零命令，取24合格帧；基准重复位置RMS/max 0.02188/0.03698mm。该次居中因保留SDK偏置使微小目标位移方向反转而退出，未发布命令；无需为了像素正中心再移动。

实际调姿共6条有限单轴指令：

- `bounded_wrist_async_validation_20260929T024454Z_149611`：J4出发和原SDK目标返回，共2条，实际+1.6699°/−1.5820°，两段均独立800ms停稳与3秒采样完成。
- `bounded_wrist_async_validation_20260929T024540Z_152583`：J6出发和原SDK目标返回，共2条，实际+1.7578°/−1.5820°，两段停稳与采样完成。精确返回SDK不代表实际编码器读数也精确返回。
- `bounded_shoulder_elbow_validation_20260929T025136Z_167696`：J2 −22 SDK字后实际−1.8457°，停稳采样完成；随后J3 +22 SDK字。J3发布后313.811ms，未编辑J4由+0.087890625°到−0.3515625°，相对初始变化−0.439453125°，超过原±0.4°边界0.039453125°。同一帧J3实际变化+1.58203125°，SDK J4仍为2048字/0°。程序终止后续动作发送，未返回、重试或失能。原failed记录和J3缺失的停稳窗口完整保留。

独立零命令袋 `fixed_board_passive_20260929T025403Z` 后续记录91条accepted，覆盖9.902秒，六轴在编码器量化分辨率内均无变化，J4为−0.17578125°；SDK持续不变。与越界帧之间约125秒未连续记录，不能推断回稳耗时或完整振荡波形，不能用这袋补算原J3控制成功。

冻结候选未再拟合。所有新袋使用024344固定基准，汇总216个验证帧（同姿态多帧并非216个独立姿态）：原模型位置RMS/max 0.57841/1.59962mm、角度0.09086/0.19779°；候选0.45617/1.43877mm、0.08825/0.18911°。数值满足既有标准，但完整报告状态为 `partial_command_phase_coverage`、`FAILED_PREVIOUS_COMMAND_CONTRACT_NOT_CLEARED`，J3原command phase为0合格帧，不能称整轮调姿通过。

汇总在本次restart目录 `prospective_all_current_partial/report.json`，逐帧候选评估 `prospective_j4_j6` / `prospective_shoulder_elbow_partial` / `prospective_passive_after_j3_fault`；越界审计 `prospective_shoulder_elbow_fault_audit/report.json` 及 `passive_after_fault.json`。所有实验原袋均在外盘ros_log顶层，归档完整。本轮未下达夹爪闭合、抓取提升或失能命令；没有重启driver，epoch仍1790645893464309304，现有URDF/X未替换。后续先分析J3动作引发的J4响应，禁止把缺窗或控制越界隐去后部署候选。


## 失败后被动尾段记录修复（未再次运动）

本轮J3诊断发生越界时立即关闭bag，使回稳过程没有连续记录。新增 `.runtime/diagnostic_failure_tail.py`，只接入有限肩肘客户端的异常退出路径：已有实际命令才断开本客户端publisher，原subscriber继续记录5秒，故障/contract不清除，不调用运动、停稳完成或几何snapshot路径；结束后仍抛出原异常并以failed归档。归档器故障或ROS关闭会明确标尾段不完整，不能将失败改为完成。该修复没有发送任何命令，不改变±0.4°等原界限，也不补写历史缺帧。

新增7项加原12项共19项离线测试通过；源码、测试与hash保存 `failure_tail_fix_review/`。独立复审核对原订阅 `/alicia_d/sdk_diagnostics` 在锁存故障后仍入袋，原始06/00反馈与SDK证据可在下次实验保留。首次合并运行因未source项目环境而收集失败，加载 `devel/setup.bash` 后通过。现有ROS四服务仍active，当前未再发动作。


## J4瞬态的串口原始证据与解释范围

独立审计 `j4_held_axis_feedback_audit/` 对齐198/198条accepted与同一CRC-valid 06/00六轴payload，位置完全一致，raw到accepted最大117.665µs。J4反馈原始字确实2049→2048→2044，195条SDK/reference的J4目标均为2048。因此本袋的偏移不是accepted与指令镜像混用或主机另行改变J4目标；仍无法仅凭这些数据区分伺服动态、机构响应和设备反馈内部问题。

只固定越界帧其余actual，并把J4从+0.087890625°改成−0.3515625°，旧FK估算tool0平移差为base坐标[+0.276208,−0.075812,+0.001865]mm，模0.286430mm，姿态差0.439453125°。这是本次J4单分量模型计算，不是真实末端独立测量，不能解释此前10–11mm的整体偏高。

袋内E1状态共8帧，其中5帧在第一条J2命令之前；J3前约986ms的最近温度10通道36–40°C且过滤quality为accepted。E1不是J3瞬态独有，本袋没有高温证据，不能据一帧E1断言过热根因，也不据此忽略状态码。当前保留原保护逻辑和限值，未修改电机PID、零位或驱动速度。


## 03:22 UTC 独立较小 J3 输入完成，不追认此前失败

原始报文审计后新增 `.runtime/bounded_j3_small_response.py`（SHA `13df36252dc3587fcd226cb80b2c69ab9a1e75b3070d6c10031a53920bb0b22e`）。唯一 J3 +8 SDK字，固定原epoch/六轴SDK起点及夹爪，默认只读，无返程/重试；保持原所有响应及几何边界，另永久绑定旧J4 anchor +0.087890625°±0.4°，不能通过新session居中放宽。新输入前SDK1991而actual1981，目标1999与actual相差18字/1.58203125°，明确不把+8名义输入说成实际最多0.703125°。22项small+tail离线测试及独立复审通过。

只读准备 `bounded_j3_small_response_20260929T031702Z_230542` 为0命令；执行时重新取新鲜证据，未复用准备的几何结果。真实执行 `bounded_j3_small_response_20260929T032235Z_244644` 仅1条指令（source1790652165738847732），当前总自主指令数为7。现场完整响应域512个MoveIt采样及连续几何：板面CAD净空下界65.964mm、整板边距37.877px、深度237.104mm，原40mm/20px/120mm要求不变。实际J3由1981→1986，变化+0.439453125°；J4由2048→2046，变化−0.17578125°。800ms独立窗口8帧J4峰峰1字，3秒posthold完成；3449/3449归档完整，terminal=completed。没有第二条指令、闭爪或提升。

独立记录器 `.runtime/record_joint_response_passive.py` SHA `6cf1b44471e6563c00fbf879dfae5f23f85af2778f909fce7f56a394a92739c0` 仅订阅/写袋，无Publisher/服务调用/参数写入，关闭时同锁保护并保留写入/close错误。第一份031950Z纯静止90秒袋没有动作；第二份 `j3_small_passive_monitor_20260929T032219Z` 完整覆盖命令前25.770秒至后64.127秒，814 accepted、807 SDK、986 diagnostics、唯一1条joint_commands，0归档错误。

独立raw审计：J3在发布后126.556ms到1984、226.749ms到1986后不再变化。J4在226.749ms到2047、726.567ms到2046，之后63.400秒无再变码或反号。相对旧anchor最大偏移−0.263671875°，旧anchor/held界限0违例，其他四轴不变。这证明本次较小输入后的稳定两字J4偏移，不能推广为+22、任意路径或真实抓取已可控，也不能认定已确诊机械回差。

同一024344固定Y、冻结候选且无refit，新袋动作前后各选24帧。动作后原模型位置RMS/max为0.6864/0.9022mm，候选0.3533/0.5632mm，角度候选0.0857/0.1452°；48帧整体候选0.5442/0.7151mm。报告 `prospective_small_j3/report.json`。合并264帧候选位置RMS/max 0.4734/1.4388mm，仍在原数值阈值，但 `prospective_all_with_small_j3_partial/report.json` 保持历史partial及 `FAILED_PREVIOUS_COMMAND_CONTRACT_NOT_CLEARED`，原+22 J3 phase依然缺窗。新小输入不能补原缺窗、不能清历史failed。

用户再次“继续处理”后，确认四个ROS服务仍active，无残留动作或记录脚本；不重复发送已完成+8指令。当前SDK为[853,2270,1999,2048,1912,2045]，实际末值[852,2265,1986,2046,1904,2049]（均六轴编码字）；后续任何动作仍必须新鲜读取，不复用该末值当实时反馈。驱动未重启，未发送失能，URDF/X/内参未替换。

## 成对模型加载草案完成，仿真配套准备继续

`.runtime/model_pair_reload/` 准备稿已完成42项离线测试和独立复审，仅默认只读路径及纯测试运行；没有真实activate/rollback。它保留硬件driver/控制器/epoch，重建使用模型的ROS消费者并校验新FK/TF/selection；当前readiness=false，不能拿较早腕部报告隐去后来的失败。回滚仅覆盖原硬件身份及已记录空场景，完整同步ROS RPC总时限尚无实机证明。证据 `model_pair_reload_draft_review/manifest.json`。

候选几何验收、原单轴诊断的控制失败和未来抓取成功是不同结论。正在独立复核各自适用范围；历史失败必须保留，但不要求历史缺帧被追认成通过。配套MuJoCo仍需完成与候选URDF同偏置的模型服务准备，在此之前不改变当前生产仿真端点，不进行新模型下抓取。


## 用户随后关闭机械臂电源：实机阶段暂停

用户明确说明“我刚刚关闭了机械臂电源”。该报告作为当前动作边界：不进行新调姿、闭爪、提升、在线模型加载/验证或使能变更；继续离线分析与配套仿真准备。没有发送停止/失能命令，也没有重启driver。只读进程查询确认没有残留动作脚本、独立记录器或reload进程。

后续监督日志中的input/SDK/accepted已与03:22实验末尾不同（例如SDK J2约0.4755rad、J5约−0.3497rad）；这些不是本助手的新实验指令，不能把之后的关节变化混入已闭合的固定输入对照。软件反馈仍更新不能独立证明电机供电状态。原epoch、SDK1999和旧板姿态均只作历史证据，上电后必须重新建立实时上下文；不得修改旧固定脚本pins使其自动追随新姿态。

离线几何范围复核已完成，`candidate_geometry_scope_review/idle_pair_reload_geometry_eligibility.json` 只认可冻结pair进入空闲成对加载和只读计算一致性验证的几何证据，不授权动作、不清旧控制失败、不声称真实抓取验证。此次断电使在线readiness明确为false，原报告不改写，另附 `power_off_context.json`。


## 再上电后 J4 完成，随后按用户要求暂停

用户报告已重启并对准板；新epoch1790652791028371475，原URDF/X/相机指纹不变。新只读基准 `fixed_board_passive_20260929T034007Z` 单独建立layout，不能混入024344绝对Y。最新目录指针 `.runtime/latest_power_resume_board.json`。J4 `bounded_wrist_async_validation_20260929T034354Z_294253` 已完成2条指令及两段停稳/3秒采样，精确返回本轮新SDK[836,2358,1986,2046,1820,2044]；actual最终较本轮初值+0.17578125°，不称物理精确返回。尚未评估新袋几何数值、未执行J6。

用户随后“先暂停”。处理暂停时动作客户端已自然结束；仅SIGINT退出独立纯记录器 `j3_small_passive_monitor_20260929T034334Z`，实际记录87.041秒、788 accepted、2条joint_commands、0归档错误。没有发送机械臂停止/使能/失能命令，其他准备稿未上线，所有子代理已完成且无动作任务运行。`latest_work_pause.json` 与新目录 `pause_checkpoint.json` 保存待办。暂停期间不继续分析、执行或部署，等待用户继续。
