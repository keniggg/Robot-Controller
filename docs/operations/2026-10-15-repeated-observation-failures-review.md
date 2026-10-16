# 多次补视等待失败的历史对照（2026-10-15 UTC）

用户指出问题已经出现多次，要求先查以前记录。本次对照已读取既有操作文档、仍在原路径的发布/部署回执，以及SanStand压缩回溯库中的旧审计和验证原文。结论是：观察后未完成抓取的表象反复出现，但既往已证实的原因不止一种。相同终态不能直接证明同一个缺陷复发；软件修复通过、某段实机运动通过和最终夹持成功必须分开记录。

记录根为 `/media/zhuyupei/SanStand/实验记录/Robot-Controller`。N16=`ros_log/full_latest_contact_admission_20261013T011103Z`，N18=`ros_log/full_latest_gen18_restart_20261014T012216Z`，N22=`ros_log/full_latest_gen22_restart_20261015T003700Z`。

| 阶段与原实测结果 | 已证实的问题及修复 | 验证/部署与尚未解决的边界 |
|---|---|---|
| GEN16前的第4轮：第一观察动作后，补视session等待约5.04秒失败；远端drain约5.97秒。 | 回调更新被旧参数刷新覆盖；修复并发交接，在原合同内等待真实ACK，5秒作为进度提示；恢复当前站位合法高度复验与有限候选搜索。 | GEN16组合5640项通过并部署。碰撞、真实ACK及所有权要求保留；不是整轮夹持成功。依据：`2026-10-13-grasp-gate-rationality.md`、N16 `verification/gen16_gate_rationality_audit/result.json`。 |
| GEN16第5轮 `20261013T112006…`：已做两次观察及交接，选出严格合格接触候选，却在任务接纳时报 `station_compute_wait_registration_too_late`，无夹爪动作。 | 已在有效期内登记的测量在重复检查时跨过0.8秒，被误分为来源违规。GEN17将历史来源证明与当前几何时效分开，在原身份/阶段内等待新READY后完整复验；另修登记计时、copy并发及冻结后退役交接。 | GEN17组合5704项通过并部署；原第5轮仍是失败。独立反例中的并发问题不全部归为该轮直接首因。依据：同文GEN17段、N16 `verification/attempt5_registration_timeline_r1/result.json`。 |
| GEN18第1轮 `20261014T015629…`：第一观察完成，request18接触被本地目标OBB/手指检查拒绝，request19第二观察路径通过严格服务，却在frozen交接报 `station_handoff_feedback_stale`。 | GEN19修正先取旧now、再等待参数/反馈快照的顺序；RPC后重新核原截止和归属，不截断负年龄、不放宽800ms。真实旧方法的独立测试复现。 | 5871项通过并部署。原bag没有函数内部旧now，不能声称测得了该次负年龄；下一轮在更早搜索阶段失败，没有证明这一交接分支已实机通过。依据：`2026-10-13-gen19-handoff-clock-repair.md`。 |
| GEN19第2轮 `20261014T025930…`：37个通过端点筛选的观察位只检查4个；四次真实MoveIt返回TIMED_OUT，任务启动前结束，无轨迹目标。 | GEN20保留真实超时分类；找到首条合格路径前继续原有限集合；未完成搜索发布无权限pending；补齐缓存发布的原来源预留检查，观察规划单次额度改为1秒。 | **GEN20已完成6024项组合并部署**。后续实测 `20261014T083439…` 仅执行3秒确认前缀，暴露独立bootstrap感知交接问题；GEN21对此完成6149项并部署。不能再把GEN20写成仍未部署，也不能把新失败说成所有观察位不可达。依据：GEN20文末最终部署段及 `2026-10-14-gen21-bootstrap-measurement-handoff.md`。 |
| GEN27第3轮 `20261015T075503…`：实际观察运动、到位资格通过；接触候选分别遇固定goal约束、right_finger预测OBB拒绝及计算截止，随后请求75观察搜索返回20次规划超时；终态为 `station_compute_wait_expired_or_future`。 | 原记录已分清局部候选拒绝、后续补视搜索、原station截止三个层次；request74曾成功让出搜索，不是始终未让出。GEN28随后修复盒形资格原始RGB/depth/mask与融合几何的来源混用，未改变这些路径期限或碰撞条件。 | GEN28来源修复7067项通过并部署。该历史轮未接近接触、闭爪或抬升；来源修复不是该终态全部根因的解决证明。依据：`2026-10-14-gen22-arrival-input-recovery.md`、N22 `verification/gen27_attempt3_compute_wait_trace/findings.json`。 |
| GEN28第1轮 `20261015T092659…`：重摆后新目标generation10，实际观察到站；request23找到严格合格补视路径并发布preview，约350ms后被clock pending覆盖；最终原90秒station截止。 | 已核封口bag：preview为1792056515.920104980，audit为6516.155773401，比上一READY几何截止晚8.199167ms；新READY为6516.258564233，pending为6516.269810438。GEN17保留的是接触计算，此补视候选未进入同类私有保留。GEN19的frozen交接阶段本轮尚未进入，不能称该时钟修复复发；也不能归为GEN27式始终没有可达补视。 | GEN29已实现精确五源补视计算证明、typed clock通知和消费者私有保留，新READY后仍走原完整接纳。定向343项通过；完整回归、独立最终审查、部署及实测尚未在本检查点确认。R28仍为failure，无实物夹持成功。依据：N22 `verification/gen29_recorded_handoff_diagnosis/{result,timeline}.json`、`verification/gen29_supplemental_retention_final/receipt.json`。 |

GEN20的部署状态已用原回执再次核对：N18 `verification/gen20_combined_final_r2/result.json` 为6024项通过，SHA256 `7c148cce12f2bbc92abfae142661441798d32013b5636ece36809ab44c9e62b7`；`gen20_full_storage_recovery/acceptance_r2/result.json` 为 `deployment_passed=true`、`whole_patch_deployed=true`，SHA256 `7b5964584a14ec94f651dec903abb30791288e91b56a22ecb70bea3362e16848`。旧索引的“未部署”属于历史检查点，不能继续描述当前状态。

已压缩的历史证据仍可回溯。本次只读打开 `maintenance/minimal_trace_cleanup_1791956657642651189/trace.sqlite3`，逐压缩块及完整原文校验后读取N16以下内容，没有恢复或复制大录制，也没有改写库：

| 原相对路径（N16内） | 读回SHA256 |
|---|---|
| `verification/gen16_gate_rationality_audit/result.json` | `f9a16c756e4142b424a491ce04715955aa791934cf1889f075a637ab44ab7887` |
| `verification/attempt5_registration_timeline_r1/result.json` | `4ea89bdcac0c691e314ddcec5b959e27a7ca8ff3bcc0f62d2e75d6580a0b5def` |
| `verification/gen16_success_first_actual_integrated_r3/result.json` | `92c66651fb064351130badeeb823ae11123d74dda7f1f3fc37c2313ae520a4ed` |
| `gen16_deployment.json` | `3ad18767f245e362a2c9be375712c2e4484659e474177f6fc9532522b17d0a99` |
| `verification/gen17_station_admission_actual_integrated/result.json` | `e0c33b9fcd8223e8dc6624f63970673e65b57df4e41b60bc58c5d5779667178f` |
| `gen17_deployment.json` | `b621a3c9912a9917d38f20ee6158731ad605399ce5bc875434ce5a8bd5a0227e` |

例如旧GEN16审计原路径已不再单独存在，但原文及其“5秒ACK等待/远端drain”的证据仍完整保存在库中，不能记为证据缺失。可用项目 `tools/minimal_experiment_trace.py read` 加上述数据库和完整原相对路径读取；命令会先验证内容。完整历史bag是否可恢复以各自删除回执为准，文本可读不意味着原录像仍在。

本次更新只写操作文档、SanStand镜像和回执。用户仍要求优先解决初始视角定位并尝试直接抓取，只有初始证据不能满足时才补视；该偏好不能通过省略真实资格、碰撞、所有权或时效检查来实现。下一修复应以具体旧/新回调反例及原录制证明因果，再做独立回归，不能因错误名称相同而重复修改已正确工作的门。

## GEN29 当前补视交接缺口：录制与离线因果证据

`N22/verification/gen29_recorded_handoff_diagnosis/result.json` 绑定封口manifest、bag、原request23审计及独立时间线；其SHA256为 `5abf72749df3e7812a7cc1a6c5d26dd8f5e3e473fe83886001b4a1e2d3877999`，`timeline.json` 为 `0d411b978381601dd47a1b2e7ec479bf74c098d67faa5f8121ea5aeec5421e47`。公开preview窗口精确为349.705458ms；`EXECUTION_FROZEN`表示活动任务期间预览不自升执行权限，本身是正确边界。bag记录的是消息接收顺序和原来源时间，不能还原任务内部每次线程调度；原源码三种audit顺序的确定性红测均丢失补视候选，补足可复现的因果证据。

修复只保留无执行权限的原候选和已登记来源证明，独立校验实际观察snapshot的全部五帧；不把它归为三帧接触证据。当前测量过期时公开几何仍无效，必须等待同owner的新三topic READY，再走原audit、来源时效、站位/epoch/手动撤销、路径和原子交接。context检查跨过0.8秒时等待新READY，迟到audit或尚未完成的fresh current witness不能丢掉合法计算；真实loss、冲突来源、deadline及实际执行发布仍关闭例外。

旧实现核心红测为3项全失败；另发现并保留了首轮权限检查拒绝后第二轮变为允许、可能越过publication barrier的2项真实红例，修复为本次检查拒绝即返回原拒绝结果。最终定向 `gen29_supplemental_retention_final/receipt.json` 为343项通过、零失败/错误/跳过、源码前后不变，SHA256 `f0295fde73cf2a4b1aa5660d8bf3137bb153ad4913f3a4195aec68044fa15a99`。它涵盖真实五帧fuser、模式节点MRO、原子审计写入和任务audit回调、私有候选交接，以及执行第一条发布前的权限关闭。测试全部隔离ROS与硬件，不能替代实物结果；本段没有预写完整组合、发布或部署成功。

## GEN29 后续实证：已部署，第二补视完成后出现不同的身份拒绝

以上“完整回归/部署待确认”是较早检查点。后续完整组合已通过 **7125项、970份来源、234个测试文件，0失败/错误/跳过**，独立最终review和发布封存完成，三个计算节点及gateway部署成功，7个硬件相关进程身份保持。用户先要求回归后暂停，随后恢复；当前持续授权为active revision28。本段绑定真实后续回执，不改写此前失败、中断或暂停记录。

| 后续记录 | 已证实的经过 | 与旧问题的区别及剩余限制 |
|---|---|---|
| 11:05:25 UTC `20261015T110525.253447Z_e1d666e812c3463a87244da38f9833ee` | 现场准备超过90秒失败后，根线程误调用wrapper；runner因缺少operator_observation在ROS/动作前退出，仅只读recorder被SIGINT封口，active_seen=false。 | 原分类为complete/interrupted，runner1、recorder0；不是机械臂实测轮，不能改成failure或success。启动守卫说明与原回执保留。 |
| GEN29第1轮11:12:26 UTC `20261015T111226.886093Z_0a4de2be16394344b0fa84a6ad2f8318` | 新generation11。实际第一、第二观察运动及第二站位三帧资格通过；约16秒后producer报 `station_compute_recovery_rgb_identity_required`，任务随后报handoff TARGET_LOST。 | 固定站位模板mask IoU 0.810808807低于0.85，颜色/轮廓距离阈值均过。当前帧仍fresh，处理跨上一READY截止49.619ms后恢复证据成为必需。与GEN28候选被pending覆盖、GEN27搜索耗尽、GEN19frozen取时问题不同；不能据同名TARGET_LOST归为旧缺陷复发。 |

本轮确实跨过旧第二补视交接停滞的位置，但公开bag无法区分正常public准入和GEN29私有保留恢复分支，不能把第二运动通过称为私有竞态分支的完整实机验收。当前首失来自generation11，bag开头旧generation10支撑面丢失已排除。记录显示恢复调用已走过支撑/唯一候选检查，直接拒绝是完整轮廓身份；不是已证明的支撑面失败、注册RMSE拒绝或90秒station截止。

用户看到目标侧面并询问灯光阴影。精确源图与mask/深度对照已导出；站位180份接受反馈范围为0，但未进行受控光照对照。只能确认分割轮廓与固定模板不够一致，不能确认阴影或光照是原因。失败后发布空mask表示不再提供有效目标，并非物体物理消失的证据。R29正常封口failure、runner6/recorder0，未记录闭爪/抬升，无实物夹持成功；当前generation11已恢复GUI手动，任务/路由/动作inactive，问题继续待定位。

| 后续验证/实测依据（N22内） | SHA256 |
|---|---|
| `verification/gen29_combined_r1/result.json` | `63ba5bc62af0e838505b1e42f0ef09263c1cae43844e97327b5a4c4539c73c2b` |
| `verification/gen29_final_independent_review/review.json` | `c2fc548d94005d8c1234f8b8175b772f93ffe5e9e422cdb35f49cf3359802657` |
| `verification/gen29_release_manifest.json` | `1551ea57d087b221fed4f6d59c8ce176c7a2e2d5398aa306829af3aa51506666` |
| `gen29_deployment/result.json` | `10beaf62b2f430b990c749802879fe18df32a5df67a03faf5ddee7599cbc56b5` |
| `verification/gen29_attempt1_loss_diagnosis/findings.json` | `b09eedc841b0ceabd192a99aab7b578032d941ba47fd20903d69f2937489f9ae` |
| `verification/gen29_attempt1_shadow_frames/result.json` | `0fad4e0e41bfe967131b6c09bf9bc33df575d3d045c111a6055fa70d05093c23` |
| `gen29_attempt_1_after_manual/result.json` | `92046839e657e26d37f5510a355915ae4e1ac50971fd21389a28955e0165231b` |

完整两份record的封口、原分类、恢复授权和SHA见主操作文档新增“GEN29恢复、正式部署与第1轮实测”段；本次只更新文档、外盘镜像/索引和before/after回执，未改生产、测试、当前控制或原实验依据。


## 全栈重启与GEN30当前帧归属诊断（2026-10-15 本地 / 10-16 UTC）

用户在诊断期间暂停为revision29，后明确要求「你重新启动各节点和上位机，然后继续处理任务」，并确认SanStand、相机和串口连接到Ubuntu。新会话C=`ros_log/full_latest_gen29_restart_20261016T000728Z`，持续授权active revision30；11个核心节点及上位机已按冻结GEN29全栈重启并验收，真实反馈、新epoch和图像更新通过。一次附加策略读取出现manual=false后已恢复，最终r3证据为GUI自身manual=true、IDLE/inactive；不能声称全程始终手动。最新画面仍有手臂，未以设备连接回复替代清场，本次没有启动抓取。较早暂停检查虽然软件inactive，但没有取得fresh接受关节样本，不记为实物稳态证明。

新离线证据进一步区分两个未完成事项。初始request1三帧用原资格入口有限重建，仍在完整矩形支持处返回 `BOX_TOP_NOT_RECTANGULAR`，没有进入最终定位误差/开口/路径验证；原完整在线资格审计和全量参数缺失，重建限制保留。原视觉搜索的外边组合角覆盖0.48低于0.5，内侧印刷边另有顶面外延矛盾，不能只调低阈值。第二补视三帧中，原几何合法匹配片段独立细化都只保留局部轮廓（固定模板IoU约0.808–0.811，可信侧面种子0）；同源另一几何不匹配片段独立细化可保留侧面、覆盖原片段全部可信核心，IoU约0.952–0.993。后者目前只用于归属设计的诊断，没有绕过几何资格直接准入，没有按模板得分选mask或修改0.85门限。

同源缓存与独立调用的输出逐字节相同，但三帧不是完整异步producer/deadline回放，也不能证明光照是原因。GEN30生产修复、完整回归、发布部署及实机验证尚未完成；当前运行970份来源仍与正式GEN29相符，7125项是既有GEN29封存测试，不是GEN30通过数。原R29仍是第二补视后身份失败，无实物夹持成功。当前完整实例归属设计及独立审查继续，初始直接抓取优先的用户要求保持。

详细数值、限制和哈希见 [GEN30诊断与全栈重启检查点](2026-10-14-gen22-arrival-input-recovery.md)。主要原件为N22 `verification/gen30_initial_view_box_replay_r1/result.json`、`gen30_initial_view_visual_trace_r1/result.json`、`gen30_current_fragment_ownership_r1/result.json` 及C `cold_restart_acceptance/result.json`、`manual_restore_after_restart/result.json`、`idle_gui_camera_r3/result.json`、`finalization.json`；文档镜像读回回执为C `document_update/receipt.json`。


## GEN30 完整回归进行中（2026-10-16 00:57:49 UTC）

本地源码已加入当前完整实例归属和全部原始融合样本的边界绑定；工作流定向353项通过。完整组合已实际启动，979份来源、246个测试文件（原234+额外修改旧测试7+新增5），要求原逐文件用例数不减。首次scope预检失败未执行测试；runner/sealer范围并集已修正并经独立静态审核。截至本检查点尚无组合 `result.json`，不能记为完整通过或发布批准。

历史第二补视三帧仍全部因 `current_fragment_outline_interior_conflict` 被当前helper拒绝，已核对的原初始三帧仍为 `BOX_TOP_NOT_RECTANGULAR`；这两个未完成事项没有被定向测试通过代替。C会话GEN29全栈重启已成功，GEN30尚未部署，本轮离线工作没有启动新运动。详细证据与限制见主操作文档同名检查点；镜像及SHA回执为C `document_update_gen30_regression_running/receipt.json`。


## GEN30 组合失败已分类，定向修复通过但未完成发布（2026-10-16 01:21:16 UTC）

完整r1结果为7476项、173失败、7303通过，0错误/跳过。共享station fixture只更新ObjectPose外层时间戳，真实binder据旧pose_base/pose_camera stamp拒绝；READY缓存实际齐全。direct-buffer fixture另缺真实pose headers与逐源READY；补齐后，CPU mock跳过planning证据检查的独立失败也已通过调用真实检查修复。仅4个旧测试文件改变，不放宽生产门、不伪造audit。原失败10文件加2个helper全文件共500/500通过，逐文件用例数保留；修复后的完整scope截至本检查点尚未重跑，不能记为发布通过。

第二补视三帧每帧42条跨片段对角边，经精确历史rectification映射核对，均来自矫正前SDK aligned-to-color source栅格的4邻接，两个正交中间格均为结构性未映射画布。此证据解释拓扑断裂而不填深度；source绑定实现正在进行，尚未发布，也不能只凭邻接认定同一物体或合格抓取。初始视觉区间r2撤回r1关于整边包络及排除整参数族的完整性推断，保留第三帧24候选族后保守开口约99.239mm大于50mm，仍需提高定位精度；真实RGB重采样总误差与全族反证仍待解决。

当前GEN29只读检查为manual=true、IDLE/inactive、generation1、同戳相机流更新，未启动新动作；模式仍waiting_for_target，未取得新清场凭据。GEN30未部署。证据为N22 `verification/gen30_combined_r1/result.json`、`gen30_rectification_topology_r1/result.json`，以及C `verification/gen30_station_fixture_regression_r2/`、`verification/gen30_current_interval_family_r2/result.json`、`idle_during_gen30_repair_r1/result.json`；具体根因与限制见主操作文档同名检查点，文档回执为C `document_update_gen30_fixture_repair_checkpoint/receipt.json`。


## GEN30 最小发布范围、预热兼容修复与完整 r3 运行中（2026-10-16 01:59:13 UTC）

本轮已把“第二补视可见侧面仍身份失败”定位到精确源采样拓扑：三个历史源各 42 条跨片段边在校正前四邻接，校正画布正交缺口均没有源映射。最终只恢复当前 source4 实测组件，不填深度、不借旧 mask；原质量、位置、extent、固定模板 0.85、0.8 秒时效与下游各门保持。未发布 partial-outline/boundary 消费链及相关 fixture 研究已归档退役，既有文件恢复 GEN29；当前只改两个既有生产文件、新增两个纯 helper 和四个测试文件，原 234 个测试文件未改。

历史三帧最终 owner IoU 为 0.954378/0.951508/0.990412，普通 OBB 通过；原盒形仍为 `BOX_TOP_NOT_RECTANGULAR`、`BOX_TOP_CONTOUR_INCOMPLETE`、`BOX_TOP_CONTOUR_INCOMPLETE`，没有一次抓取或实物成功结论。实录叠加 144 像素独立同色邻物的有限合成反例真实触发新入口，被完整 raw source 组件检查拒绝；不是新物理实验。对应 N22 `verification/gen30_source_connected_owner_r4/result.json` 与 `gen30_reachable_source_neighbor_r3/result.json`。当前初始视角的上方角缺深度、原视觉矩形资格及最终定位问题继续未解决，v2 区间研究不属于本次发布。

完整 r2 为 **7149 项中 5 失败、7144 通过**。五项同属原 face-freeze lock 测试，根因是可选拓扑预热把恢复专用 profile 字段变成全局普通识别 gate；相同 fixture 仅换 GEN29 node 则 5/5 通过。现仅隔离 `cache.offer` 的 `ValueError`，原 source activation 校验与恢复 `cache.require` 严格校验不变，原五项测试字节未改。5 原＋3 新边界＋24 既有 GEN30 共 **32/32 通过**；r2 失败原件保留。因果、修复与冻结证据为 N22 `verification/gen30_optional_prewarm_cause_r2/`、`gen30_optional_prewarm_fix_r1/`、`gen30_optional_prewarm_freeze_r1/`。

完整 r3 已启动，976 份来源/238 个测试文件，scope SHA `ea46204fd42dddb6ed240d184b67665a6ba591deae70568dc15ce2314301cbfb`；本检查点尚无最终 result，只记运行中。GEN29 最近已保存状态为手动、IDLE/inactive，本轮无新运动，GEN30 未部署。具体数值、剩余观测缺口和误差限制见主操作文档同名最新段；汇总 C `verification/gen30_final_diagnosis_r2/diagnosis.json` SHA `d70f277d4dc11039750c1ae409e1e95af5e612562af0fc247a5cc77f24d19654`。本次镜像回执为 C `document_update_gen30_scope_r3_running/receipt.json`，未修改控制/进度/运行指针或冻结源码。


## GEN30 已部署后的第1轮：可见顶面、盒形拒绝与第二补视规划耗尽（2026-10-16 02:53:39 UTC）

完整r3已封口7152/7152通过，976来源、238文件，原234文件7125项原数保留；独立审查、发布与三个计算节点部署均完成，八个硬件/GUI进程保留。r2的7144通过/5失败保持历史原意。正式发布N22 `verification/gen30_release_manifest.json` SHA `c6554435fafe0589e73910e928439204654cd6e7edba993ad7ecd7df509f837f`，部署C `gen30_runtime_deployment.json` SHA `cf826a271a2252f4d340d0d2404e5af0631604d8166241031ad99f34529e7f3e`。

新实际轮 `grasp_records/20261016T021226.502516Z_0494c8bc84a34a74933e8eae122dd1f4`（generation2）第一次补视已运动、测量合格，第二补视仍在规划时以 `TARGET_LOST: station_compute_wait_expired_or_future` 失败，已complete/closed/failure，runner6/recorder0；未闭爪/抬升。初始源缺原box完整audit，不能以第一次补视的 `BOX_TOP_NOT_RECTANGULAR` 回填它。后者在线填充98.6556%、顶面RMS0.5495mm；一角距离超容差0.08135mm，另两边仅2/10和6/10支持。0.473574角分数是距离函数，不是可见像素百分比。上边762个mask像素raw深度为零；右边200个有效点低于拟合顶面1.524–2.198mm，被原±1.5mm顶面带过滤。RGB轮廓清楚并不说明定位误差已充分约束。

90秒规划阶段保留241条ready/tracked，源年龄最大0.4691秒，模板IoU最小0.9285；39个strict execute=False候选检查均超时/goal sampling失败，共63.509秒，首次lost在phase deadline后0.0596秒。故本轮应诊断规划搜索与期限分类，不能归为历史同一种轮廓丢失，也不能据规划超时认定姿态数学上不可达。确切边角重放和时序限制分别见C `verification/gen30_attempt1_closed_evidence_r1/first_observation_box_trace.json` 与 `verification/gen30_attempt1_timing_diagnosis/observed_timing.json`；主操作文档同名检查点保留数值、SHA与未完成事项。

实验后GUI确认manual=true，20样本核验任务FAILED/inactive；用户暂停revision31及最新恢复active revision32分别保存于C `pause_after_gen30_attempt1_20261016`、`gen31_resume_20261016`。恢复检查仍为手动/FAILED/inactive，未声明新清场。GEN31设计/修复尚待验证，没有实物成功；本次文档与外盘镜像回执为C `document_update_gen30_deployment_attempt1_checkpoint/receipt.json`，不改控制、进度、运行指针或生产源码。
