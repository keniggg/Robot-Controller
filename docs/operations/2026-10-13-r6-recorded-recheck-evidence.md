# 2026-10-13 第 6 轮原记录复核：重验拒绝与补视证据边界

SanStand 接回后，本次实际只读重读第 6 轮原审计、运行日志和结果文件，并核验 manifest 所列的全部 11 个非 bag 文件。本文补充此前缺盘时的源码分析；此前记录中“本轮未从外盘独立复验”的限制现在仅由下列明确核验项补足，不能据此扩大为完整原包重放。

本次复核没有启动 ROS 节点、连接串口、发送运动或失能命令、查询温度，没有写入或修改外盘原记录。仅新增这份项目操作文档，外盘镜像由后续统一流程处理。未修改 GEN18 已封存的离线修复文档或源码交付清单。

## 来源与完整性

以下路径缩写仅为表格可读性：

- `E` = `/media/zhuyupei/SanStand/实验记录/Robot-Controller`
- `R6` = `E/grasp_records/20261013T123054.826269Z_1578aafa633140ab9ac8b88d9513de5e`
- `N` = `E/ros_log/full_latest_contact_admission_20261013T011103Z`

| 实际读取的文件 | 字节数 | 实际计算的 SHA256 |
| --- | ---: | --- |
| `E/ros_log/planning_audits/grasp6d_gate_audit_latest.json` | 419225 | `8ae63c162409c378cafa9d99d57eee0696ba883c13a54644c56ef01b22f31772` |
| `R6/keyframes_audits/f76068419f11f98998df4dfad51a971bbaddf13da30e299907312a6dda7098d5.json` | 414301 | `f76068419f11f98998df4dfad51a971bbaddf13da30e299907312a6dda7098d5` |
| `N/attempt_6_watch.log` | 94924 | `d30a4727d03915cd607a1f4f540724b3f9173e9db952d01d3043136a7b392065` |
| `R6/manifest.json` | 5521 | `12ce76bfaf388198c7e72afe1bf2119109ac11ae12823fd6e044036b64a04371` |
| `R6/result_summary.json` | 1778 | `6087949b5cab037cde09066758cf54e5e9d473463ad04df468ba3ad18b1370e8` |
| `R6/command_result.json` | 1575 | `d6817a237f01b9adee6516f580f7ea7486d7541d5628e13c10d70fbad9997fd5` |
| `R6/recorder_outcome.json` | 1472 | `57c40232ecc4a8684e512229f10517a5da03a4c70fd41062cb120e6cd2c73b06` |
| `N/gen17_reload/consumers_ros/1252cb0c-c6a5-11f1-95fb-ab255a29cb49/remote_grasp6d_node-2.log` | 52359 | `f68b5f43c38f7f06a758092f3399953bd46b770906ea0409dd664cea1b316afe` |

第一个 audit 的 request/generation 为 `13/7`；其实际哈希与 `attempt_6_watch.log:96` 的 `audit_reference.report_sha256` 一致，该引用同时记录 `atomic_committed=true`。keyframe audit 是同一请求的另一份保存报告，两者内容和哈希不能混用。

manifest 列出的 11 个非 bag 文件逐个重新计算大小和 SHA256，全部相符：`authorization.json`、`command_result.json`、5 份 `keyframes_audits/*.json`、`recorder.log`、`recorder_outcome.json`、`recorder_ready.json`、`runner.log`。此检查不是整盘文件系统检验，也不是 bag 内容校验。

没有重新计算 `keyframes.bag` 或约 5 GB 的 `full_frames.bag` 哈希，也没有修改 manifest。原 `manifest.json` 为 `state=complete`、`result=failure`、`recorder_closed=true`；`recorder_outcome.json` 为 `bag_closed=true`，停止原因为 `observed_task_became_inactive`。这些证明原记录的结果和封口声明，不证明抓取成功。

## 已核实的失败链

request 13、generation 7、target epoch 4、snapshot `1791894719557530164` 的原 audit 与 watch 指标一致：

| 阶段 | entered | passed | rejected |
| --- | ---: | ---: | ---: |
| locally_valid | 11 | 6 | 5 |
| stable | 6 | 6 | 0 |
| moveit_shortlist | 0 | 0 | 0 |
| moveit_checked | 0 | 0 | 0 |
| moveit_reachable | 0 | 0 | 0 |
| hard_recheck | 0 | 0 | 0 |

6 个初筛通过项均来自 `tabletop_geometry`。原 `moveit_selection` 的 `shortlist_count=checked_count=reachable_count=0`、`terminated_early=false`，`selected=null`。此处 funnel 的 `hard_recheck` 名称不等同于随后从日志确认的 stable variant 重验，不能把其零计数解释为后者没有发生。

`attempt_6_watch.log:83–95` 中共有 12 条 `remote 6D stable variant recheck failed`，覆盖 track 1 至 6 各两个不同变体：

| tracks | variants | 日志中的当前对称余量 | 条目数 |
| --- | --- | ---: | ---: |
| 1、3、4、6 | 各 0、1 | 0.500000 mm | 8 |
| 2、5 | 各 0、1 | 0.000057 mm | 4 |

全部 12 条都记录相同的经验平移证据要求 `±2.676768 mm`，并明确给出原因：`No placement in the nominal continuous contact/support interval covers the unchanged translation evidence limit.`

紧接着 `attempt_6_watch.log:96–98` 和 `result_summary.json` 却报告 `NEAR_FIELD_NO_REACHABLE_CANDIDATE`，文案为所有已检查候选均未通过严格 MoveIt 可达性。由于本请求实际 MoveIt 检查为零，该分类与文案不符合发生阶段。GEN18 将完整 stable 重验拒绝与未完成重验分别记录，针对的是此处真实缺口；本次复核不代替 GEN18 的文件交接或组合回归结果。

## 高度余量能核实到哪里

原日志保存了每次拒绝、要求值和当前对称余量，但 request 13 的原审计 `stable_evaluations=[]`，没有保存每个拒绝变体的完整 `contact_height_placement`、`contact_normal_translation_intervals_m` 或 `allowed_normal_translation_intervals_m`。

因此，可以确认该轮在路径检查之前因高度放置退出；不能用 0.5 mm 或 0.000057 mm 的当前对称余量直接推算整个连续区间的宽度，也不能从这些日志断言哪一个完整区间端点阻止了居中。`place_contact_height` 已有连续区间居中与复测逻辑，原日志说明该逻辑返回失败，但本次未对原输入重放得到完整区间。之前使用已有几何 fixture 的居中回归仍属于合成回归，不能改写为第 6 轮完整区间重放。

## 独立薄面方向与补视边界

本次以实际 audit 的 `replay_geometry`（OBB 轴、尺寸、支撑法向）和 `remote_diagnostics`，在纯内存中调用现行 `bounded_bilateral_view_gaps`。开口参数使用原记录一致的 `max_inner_gap_m=0.05`、`opening_fit_clearance_each_side_m=0.0005`；没有调用 ROS、路径服务或发布运动。

`include_thin=false` 返回 0 个；`include_thin=true` 返回以下 5 个。它们来自原 `contact_boundary_profiles` 的 `fused_measured_bilateral_surface` 测量，要求接触高度为 2 mm。

| 方向角（度） | 原共同高度区间（mm） | 共同高度（mm，约） | 整体 OBB 所需开口（mm，含间隙） |
| --- | --- | ---: | ---: |
| 57.58484053184368 | [9.251962548770367, 9.444195878166847] | 0.192233 | 42.110113 |
| 58.79242026592185 | [9.250735452478452, 9.45307925176014] | 0.202344 | 43.471160 |
| 56.29242026592186 | [9.166773690067406, 9.40149507569909] | 0.234721 | 43.566080 |
| 59.99999999999999 | [9.27082806987903, 9.472101372762682] | 0.201273 | 44.813342 |
| 55.00000000000001 | [9.131448025898154, 9.402032765963764] | 0.270585 | 45.000390 |

这证实存在“共同可见高度不足、整体开口仍容得下”的独立方向候选，支持继续检查补视机会。它不证明已经满足完整补视许可，更不证明可以接触或抓取：

- 原 audit 的 `observe_return.measured_missing_sides` 使用初始来源 `1791894660686042547`，不能冒充 request 13 的最新同源侧面复测结果。
- 未在此次实际读取的 audit 和文本日志中找到完整的 `_observe_return_step_gain` / `previous_gain` 报告。`previous_gain_sha256` 为全 0，属于初次观察会话字段，不能据此推断已计算的增益成功或失败。
- `replay_geometry` 保存 2699 个 raw/registered 包络并集点，点哈希 `2c4fc262c385ba8455c218a2ac1aced0084604347f641d3f15557fd70bafa379`；`multiview_surface` 的融合面为 1767 点，报告保存其摘要、来源时间和各视图点数，但未在这些 JSON 中保存完整的前后融合点集及逐点来源。包络并集不能直接当作严格 `registered_information_gain` 输入。
- 本次没有从 bag 重建完整前后融合面或重放增益。因此不能声称 GEN18 在原第 6 轮数据上已经产生有效补视请求。后续仍须真实的同源侧面复测、有效增益、身份和注册绑定、原剩余时间及次数、独立观察运动检查。

后续验收应保留真实的逐项 placement 账本和计算所得的增益报告；不得人工补写原轮次没有保存的通过证明。该轮保持失败分类，本次记录不构成部署、完整补视许可或实物抓取成功验收。
