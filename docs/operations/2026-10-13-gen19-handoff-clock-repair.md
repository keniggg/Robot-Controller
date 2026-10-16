# GEN19 补视交接时钟修复

本地日期 2026-10-13，外盘目录使用 UTC 2026-10-14。用户在暂停后明确要求“继续处理”，恢复未完成修复和此前持续抓取任务。本文区分离线通过、运行时加载和实物抓取结果，不以测试数量代替抓取成功。

## 已录制失败与根因边界

记录根目录为 `/media/zhuyupei/SanStand/实验记录/Robot-Controller`，本次节点会话为 `ros_log/full_latest_gen18_restart_20261014T012216Z`（下称 N18）。GEN18 本会话第 1 轮记录为 `grasp_records/20261014T015629.706423Z_f0e3415e597e418fa8d9a43fce55ab9f`。该轮已封口且失败，未夹持成功。

第一观察动作完成；request 18 的接触候选被最终注册后的本地必要条件检查判为目标 OBB 与手指重叠，不能把汇总中的 14 条条目说成实际调用 MoveIt 的 14 次规划失败。request 19 已产生第二观察位预览，保留的第一视点增益证据为新增注册体素 48，第二观察路径结果为 `STRICT_SERVICE_SUCCESS`。随后在实际 frozen 检查点报错：`STATION_HANDOFF_INVALID: station_handoff_feedback_stale before supplemental observation frozen`。

失败前最新实测反馈源时间为 1791943086838684823 ns，bag 接收时间为 1791943086839525222 ns；失败日志时间为 1791943086.846677 秒。这支持“当时并非持续 0.8 秒无反馈”，但 bag 未记录任务函数内部旧 `now`，不能声称直接测得了该次内部负年龄。

旧函数先取时间，再执行可能阻塞的相机参数读取和反馈锁等待，最后取得新反馈快照；这能使新反馈晚于旧检查时间而被判为未来/过期。独立测试把封存 GEN18 的真实方法装入实际 frozen 调用路径，确定性复现同一错误。独立审查还发现 non-rebound 分支在相机 RPC 后漏查原阶段截止，两个反例均证实遗漏。

## 修复与保留的判断

- 反馈与 ACK 均先取得快照，再读取真实当前时间；不把负年龄截成零，不伪造时间戳。
- 返回前复核原 ROS 和 monotonic 截止、任务归属、控制 epoch、会话、绑定方案和撤销状态，覆盖交接前后两个未退休分支。
- ACK 阻塞检查后再次核对请求及快照。合法 ACK 升级只在原预算内重取，不重发请求、不延长期限。
- frozen 交接只有在两个执行 ACK 和当前绑定检查均通过后才能退休旧站位。真实过期、未来反馈、姿态变化、相机变化和原有碰撞/执行确认规则保留。

这里修复的是时间采样顺序造成的误拒绝。反馈新鲜性涉及执行时姿态是否仍成立，不能直接降成警告；接触碰撞失败也不能当成效率告警跳过。原 800 ms 反馈界限及原阶段/会话截止未改变。

最终修改两个生产文件 `station_compute_wait_task.py`、`observe_return_task.py`，新增 44 项定向测试与 12 项独立边界测试。首次完整组合 `gen19_combined_r1` 为 5869 通过、2 失败：具体的 ROS 截止分类被通用等待超时覆盖，因此没有部署。修复保持旧测试不变，在两个检查点均先使用快照后的真实时间执行原分类验证，再于等待或返回前复核原截止。

最新相关回归 `N18/verification/gen19_handoff_clock_candidate/related_r4/result.json` 为 597 通过，0 失败/错误/跳过，包含旧分类 32 例，源码 SHA 前后一致。早期测试夹具断言错误、测试中途改断言的结果和首次完整组合失败的日志、XML、回执均保留，不作为定版通过证据。

最终完整组合 `N18/verification/gen19_combined_r2/result.json` 为 **5871 通过，0 失败、0 错误、0 跳过**，用时 418.562 秒，927 个已绑定源码/依赖文件在测试前后 SHA 一致；187 个测试文件的范围由发布清单冻结。597 项相关回归是组合验证的一部分，不与 5871 重复相加。完整组合回执 SHA256 为 `ac2eaecb92b982346852d8bc163cf153359b9b2979ac2c29f7b019862c55e6b5`；597 项相关回执 SHA256 为 `380f293c36e2f1f3a80f0baf905c41436c825024535aa15ecb0fc26c5773f61d`。

## 发布与运行时加载

独立复核通过后，已封存 `N18/verification/gen19_release_manifest.json`，SHA256 为 `804f2d468ba180185aab93af1c4135393384511b25a15a574430aabda09e4570`。该不可变清单封存于重载之前，因此其中 `runtime_deployment_pending=true` 和 `installed_semantics` 仍描述当时状态；不能据此忽略随后独立生成的部署回执，也不回写封存清单。

`N18/gen19_deployment/result.json` 已确认 `deployment_passed=true`、`whole_patch_deployed=true`、`compute_reloaded=true`、`hardware_processes_preserved=true`，并绑定上述发布 SHA。部署回执 SHA256 为 `7d29a295c74c51d5a48915237346afcd884027c8e05ddc2865ae92aae7ae9090`。

在手动空闲状态核验进程身份后，只重载 `/grasp_task_node`、`/remote_grasp6d_node`、`/unknown_tabletop_perception`。驱动、硬件接口、MoveIt、运动网关、相机、夹爪控制器、模式路由和上位机这 8 个进程的 PID 与 start ticks 前后完全一致。remote/task 两个 reset ACK 均为 `ready=true`。部署回执记录未发送运动或力矩命令、未查询温度；自动抓取前仍需提交新的选择并完成每轮录制与现场检查。

上述记录证明源码验证、封版与运行时加载完成，不证明 GEN19 已实物抓取成功。新一轮执行及夹持结果须依其独立回执追加。

## request 18/19 的几何门限制核查

以下两份封口审计位于本轮记录的 `keyframes_audits/`，不是后续会被覆盖的 `planning_audits/*latest.json`：

- request 18：`grasp_records/20261014T015629.706423Z_f0e3415e597e418fa8d9a43fce55ab9f/keyframes_audits/2362ffc03a168b9bce1280b881ea1a51bfb7c66e555c817534f048b34ba13e58.json`。
- request 19：`grasp_records/20261014T015629.706423Z_f0e3415e597e418fa8d9a43fce55ab9f/keyframes_audits/14dce9134556d54b0f2c15e93576c72360e422b1b093b2bf412a47d91cc7c11b.json`。

request 18 的 `pipeline_funnel.stage_counts.moveit_checked` 为 14 项检查完成，`moveit_reachable` 为 0 项通过；`moveit_selection.terminated_early=false`。14 个有检查结论的 `lineage` 条目均为 `contact_goal_necessary_check.proof_stage=final_registered_after_original_resolution`，左右指体的 `inevitable_overlap=true`、掌部为 false。`contact_sequence_precheck` 为 null，`contact_search_yield_attempt.collision_receipts=[]`，不能把本地拒绝当作已经调用接触预检服务后的失败回执。`final_contact_rejection.code=CONTACT_SEQUENCE_PRECHECK_FAILED` 保留了精确原因；外层 `moveit.failure_code=MOVEIT_CHECK_ERROR` 是统计包装，不足以证明服务故障、IK 失败或计算超时。

这些局部有效候选的 `rows[16..29].sampled_angle_deg` 均为 25°，其 `required_open_width_m` 约 43.005–43.037 mm；例如 `rows[16].required_open_width_m=0.043029712617249`。这是局部接触带的开口需求，不能与整盒轴投影互换。最终 `lineage[*].final_contact_height_margin.nominal` 中，桌面最小间隙为 12.839–13.128 mm、接触重叠为 2.000–2.358 mm，分别通过既有 3 mm 与 2 mm 要求；本次拒绝不是桌面间隙门过严。

为解释 OBB 拒绝，按 request 18 中同一 `base_link` 参考系作辅助几何计算：令 `s=replay_geometry.obb_size_xyz_m`，其值约为 `[58.619, 41.626, 21.545]` mm；令 `R=replay_geometry.R_base_obb`，该字段的列是 OBB 各轴在 base_link 中的方向；夹爪方向 `y=rows[16].T_base_tool0[:3,1]`，即 tool0 的正 Y 轴在 base_link 中的方向。整盒沿夹爪方向的投影为：

```text
W_OBB = sum(s[i] * abs(dot(y, R[:, i])) for i in range(3))
      ≈ 0.0657655 m
```

65.77 mm 是**冻结包围盒的轴投影计算值，不是传感器直接测得的物体宽度**，也不是上面的 43 mm 局部开口。它超过固定 50 mm 开口，与保守盒模型中的左右指体重叠一致。最终碰撞结论仍以绑定最终方案的必要条件回执为准；投影量本身不证明真实物体必然碰撞，也不授权忽略路径检查。OBB 与指体盒都是保守模型，现有记录不足以把它们判成代码误拒绝，更不能直接把碰撞改成警告。

request 19 提供独立的补视理由：`observe_return.trigger_evidence` 绑定 request 18、`NEAR_FIELD_NO_REACHABLE_CANDIDATE` 和 `current_request_independent_bilateral_view_gap`；`phase=supplemental_observation`、`view_index=2`、`trigger_code=BILATERAL_SURFACE_EVIDENCE_MISSING`。`observe_return.measured_missing_sides` 的四个方向如下，各项 `missing_side_signs=[-1]`，共同高度最低要求 `required_contact_patch_overlap_m=0.002`：

| 数组下标 | angle_deg | required_open_width_m（mm） | whole_obb_width_m（mm） | measured_common_height_m（mm） |
| --- | ---: | ---: | ---: | ---: |
| 0 | 55.7898 | 42.6263 | 41.6263 | 0.2643 |
| 1 | 57.8949 | 44.7514 | 43.7514 | 0.2242 |
| 2 | 60.0000 | 46.8175 | 45.8175 | 0.2525 |
| 3 | 50.0000 | 48.3274 | 47.3274 | 0.8798 |

这几个方向的模型开口需求能落在 50 mm 以内，但当前双侧共同测量高度均不足 2 mm。`observe_return.previous_gain` 记录 `OBSERVATION_NEW_MEASURED_COVERAGE`、`new_occupied_voxel_count=48`、`minimum_novel_voxels=3`，说明第一视点确有新测量，仍需继续补足负侧共同高度。

第二观察预览的 `lineage[0].moveit.reachable=true`、`evidence_code=STRICT_SERVICE_SUCCESS`；`observation_envelope.minimum_support_clearance_m=0.08265310560738018`，`observation_view.arrival_projection_endpoint.minimum_image_edge_distance_px=46.55120707428762`，超过对应 24 px 要求。`promotion.code=EXECUTION_FROZEN` 表示执行活动期间预览不自行升级；实际交接消费者已进入 supplemental admission，随后才被反馈时间误判中断。该轮没有执行第二观察位。

下一轮应对照补视前后的 `measured_missing_sides`、共同高度、`previous_gain` 和新候选方向；若仍只有同一 25° 候选，不能把重复检查当作新增几何证据。分别保留本地必要条件拒绝、服务预检回执、真正服务异常及交接反馈时间戳，避免通用错误标签再次混淆原因。

## 录制空间

恢复时 SanStand 可用约 7.4 GiB。授权的失败记录维护已执行并返回 `eligible_exhausted`，删除 0 字节；原核准大录制已清理完。当前诊断记录、本会话和未知/成功记录继续受保护，不为凑到 70 GiB 扩大删除范围。

两次完整回归与四次相关回归的临时夹具随后占用了约 0.754 GiB。exFAT 的 128 KiB 分配簇使大量小 JSON 和目录的实际占用明显大于内容字节数。已按精确清单移除本次结束测试的以下 8 个临时根，均位于 `N18/verification/`：

- `gen19_combined_r1/tmp`
- `gen19_combined_r1/pytest_tmp`
- `gen19_combined_r2/tmp`
- `gen19_combined_r2/pytest_tmp`
- `gen19_handoff_clock_candidate/related_r1/tmp`
- `gen19_handoff_clock_candidate/related_r2/tmp`
- `gen19_handoff_clock_candidate/related_r3/tmp`
- `gen19_handoff_clock_candidate/related_r4/tmp`

精确计划与执行回执为 `maintenance/completed_pytest_fixture_cleanup_1791946345720478434/{plan,result}.json`。移除 3316 个测试文件，清理前目录分配合计 809,238,528 B；回执测得可用空间由 7,094,272,000 B 增至 7,902,199,808 B（约 7.36 GiB），净增加 807,927,808 B，净变化包含同期其他写入，不能与目录统计混为一项。

这些内容是可重建的合成夹具及历史 fixture 的测试回放副本，未删除实验原始记录。`contact_audit.json` 等由项目中封存的 GEN11 fixture 翻译进合成场景；临时 `.bag/.active` 是 6–33 字节的固定测试字符串。无指向实验原件的符号链接或硬链接；真实回归日志、XML、result、patch manifest、发布及部署回执全部保留在临时根之外。`result.json` 明确记录 `experiment_originals_deleted=false`、`logs_xml_results_preserved=true`、`archive_policy_changed=false`。这次测试临时文件回收与已返回 `eligible_exhausted` 的失败实验清理是两项独立操作，没有扩大失败实验删除范围或变更保护名单。

下一轮录制方案只启用主录制，继续保留 LZ4 关键帧和文本/审计，不启用附加全帧录制。按 R1 约 100 秒主 bag 的体积线性估算，600 秒约 2.605 GiB；以清理回执中的 7.36 GiB 计算，预计剩约 4.75 GiB，略高于 archive 配置的 4.5 GiB 最低值。这是历史速率估算，不是最坏情况保证，仍须在每轮开始前按真实剩余空间复核。本节不表示该轮已经执行或成功。

备用的 `.runtime/record_budgeted_full_frames_20261014.py` 已通过 24 项离线测试与独立复核，默认 1 GiB bag 预算、6 GiB 空闲下限，预留 288 MiB，在 736 MiB 或空闲达到 6.28125 GiB 时开始关闭，只操作自身 writer。预算不是磁盘硬配额；收据须写明实际大小、消息覆盖时间和提前结束原因，不能声称覆盖完整 600 秒。其测试回执为外盘 `maintenance/budgeted_full_frames_validation_20261014T023114Z_24441/result.json`，尚未实录使用。旧全帧记录器和既有实验不改写。持续策略保持 `local_only`、上传关闭、通用归档删除关闭；缺盘不回退系统盘。

## 抓取前后记录要求

组合回归、封版和三个计算节点重载均已完成。抓取前重新核对画面、实测关节、模式状态与真实空间，持续授权恢复与每轮录制分开留证；实物夹持结果只根据新一轮独立录制、执行回执和实际观测追加。不查询温度，不发送关节失能命令。
