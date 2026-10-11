# 2026-10-08：在关节空间生成可达补视角

本轮已找到通过原有严格规划、完整轨迹跟随支撑检查和采样视野检查的补视角，并已完成仅规划节点的部署。随后 gen29、gen30 两次实机流程尝试均在上游感知/请求准备阶段终止，尚未进入新 FK 候选或严格路径检查，也未发生本轮动作。**截至本次更新，尚无本轮实机补视角或抓取成功结果。** 冻结数据、候选及计算检查均不构成执行授权；到位后仍需新的实测支撑、配准与接触证据。

GitHub 提交 `b2fdac56` 是本轮修复前的固定源码备份，已核对 2203 项文件 hash。本修复与该备份分开，不应把新增代码或本轮规划结果写成该备份的历史实物验证。历史一次性抓取的证据范围见 [初始视角抓取回归说明](2026-10-08-initial-view-grasp-regression.md)。

## 原问题与定位

冻结 gen28 的 req41 已有稳定目标，但原补视角候选把目标强制放在光轴中心，采用固定约 45° 倾斜的方向族，组合成 42 个端点。对这些端点单纯增加搜索时间，没有先证明该族在当前关节限制下可达。

- 使用实存 URDF、源帧手眼变换与实际关节状态重建，保存端点与生产端点最大矩阵差为 `8.33e-16`；夹爪状态来自实测 `right_finger=0.04975`。
- 当前关节 FK 与冻结变换、独立 FK 与 MoveIt FK 均在约 `1e-15` 内一致；当前状态无碰撞，FK→IK 自洽成功，空间雅可比秩为 6。没有证据支持“只有 5 自由度”或种子奇异的解释。
- 42 个端点在每次 100 ms 的有碰撞/无碰撞 IK 中均失败。保留原关节限位、每端点 6 个种子的离线数值拟合，其最优解全部碰到 Joint5 下限 `-1.57 rad`，仍有位置/姿态残差。这支持“居中定向候选族与 Joint5 限制冲突”的具体诊断；有限搜索不是所有观察姿态不可达的数学证明。

## 方法与集成边界

新增 `src/alicia_grasp_modes/reachable_observation_views.py`，从当前实测关节状态在原 URDF 限位内用 SLSQP 生成候选。目标允许偏离光轴中心，仍要求完整 OBB 满足原视野、边距与景深门，且补足原缺失侧的信息增益。

- 输入绑定当前准备请求、源帧、目标身份、模型、手眼变换、实测支撑点、相机参数、夹爪几何与已有跟随合同；不硬编码本次目标或求得的关节角。
- 独立复验原信息增益、15 mm 工具基线、完整 OBB 工作体积、张开夹爪几何、带关节误差盒的端点支撑下界，以及原实测支撑投影。候选自身不证明整臂碰撞或完整路径。
- 提议优化额外预留 4 px，实际接受门仍为原 `36 px`；景深仍为 `70–500 mm`，支撑净空仍为 `3 mm`。跟随误差初筛使用已有可选合同中的最紧合同，最终完整路径检查重新选择并绑定实际合同；没有降低原跟随阈值。
- 最多 2 个方向，每方向以当前实测关节状态为一个确定性种子，最多 96 次迭代，共享 2 秒协作式预算；逾期候选丢弃。该预算不表示可以强制中断一次底层数值库调用。
- `mode_aware_remote_grasp6d.py` 仅在缺双侧表面证据的补视角分支加入候选，保留原候选与原完整检查。相同 prepared/source/model/q/合同等输入才复用结果；输入变化不能重置同一 prepared 的生成预算，生成后再次检查来源、反馈和配置。初始 direct/initial_contact 路径不调用该生成器。
- 声明 `python3-scipy` 运行依赖。生产启动已有 `PYTHONNOUSERSITE=1`、`PYTHONDONTWRITEBYTECODE=1`、`OPENBLAS_NUM_THREADS=1`；验证须使用同样环境，避免 shell 中用户 NumPy 与系统 SciPy 混用。后端不可用时返回无候选及审计原因。

## 验证与失败保留

| 阶段 | 结果及意义 |
| --- | --- |
| 首个关节空间候选 | 严格规划与视野轨迹通过；加入真实跟随/停止误差后，目标处反例净空约 `-4.566 mm`，未通过原 `3 mm` 门。没有执行。 |
| 加入合同误差盒的候选 | 严格规划和完整跟随检查通过；实际 IK 目标的边距 `35.89755 px`，小于原 `36 px`，仍拒绝。没有放宽 IK 或视野门。 |
| 额外预留 4 px 的 robust_margin0 | `complete_candidate_checks_passed=true`；状态无碰撞、strict、following_path、view_path、actual_ik_goal_gain 全通过。 |
| 最终完整路径证据 | 原合同选取路径容差 `0.09 rad`，含停止及编码器余量的误差盒为 `0.12101699039494282 rad`；连续支撑下界 `3.023991 mm`。889 个视野样本最小边距 `39.843627 px`，端点实测支撑投影 4454 像素、3 个有效象限，实际 IK 目标增益 `3.193683 mm`。 |
| 生产 helper 的冻结源回放 | 使用全部 68591 个实测支撑点，2 个候选均过原端点门；总计 `0.339042 s`，两方向分别 `0.073295 s` / `0.222743 s`。首候选 q 与已检查 robust_margin0 约 `3e-8` 差异。此时延是本次回放结果，不是每次请求保证。 |
| helper 单测 | `17 passed`，覆盖真实冻结输入、输入不可变性、坏输入、后端失败、预算过期与求解器虚报成功。测试使用现场外盘 fixture，缺少该 fixture 的环境会跳过，不能把跳过记为通过。 |
| 集成及相关回归 | 在生产解释器环境下 `735 passed, 1 deselected`，用时 `19.63 s`，包含 19 项新集成测试；排除的一项既有 worker 并发测试在此前 `third.log` 中通过。该测试结果不代表动作执行。 |

完整跟随证明有明确的合同和几何适用范围，不等于已经证明硬件实际跟随、停止或标定误差；视野检查为有界采样，不声称连续可见性或未观测遮挡。部署后仍须以新的完整请求和实测反馈重新走原流程，不能执行旧计划。

## 部署及实际流程尝试

`SESSION/deployment/result.json` 记录 `deployment_passed=true`，部署只重新加载 `/remote_grasp6d_node`；其他记录中的机械臂、网关、相机、感知和 MoveIt 节点进程身份未变，部署前后实测关节位置一致。已核验四个部署源文件 hash，部署没有发送动作或关节失能命令，也没有查询温度。GitHub 固定备份 `b2fdac56` 保持不变。

| 代次 | 观测到的结果 | 未覆盖的路径 |
| --- | --- | --- |
| gen29 | 51 帧 ready，12 帧因当前支撑共识不足进入 pending，10 次以新的唯一实测帧恢复。5 个 initial_contact 准备请求被 `TARGET_MEASUREMENT_PENDING` 取消，接受计划为 0。最终失败 `source_exceeds_existing_frame_limit`。 | 未进入 observation_prefix、新 FK 候选或 strict；录制 `active_seen=false`，没有本轮动作或实物抓取成功。 |
| gen30 | 用户确认场景静止后继续尝试。请求 6 在准备末尾以 `GENERATION_STALE` 取消；首失效为 `source_frame_stale_or_future`，发生在 post_segment 的原 800 ms 新鲜度检查。 | 同样未进入 observation_prefix、新 FK 候选或 strict；录制 `active_seen=false`，没有本轮动作或实物抓取成功。 |

gen29 的末两帧 pending 在原 170/140/110 环内都未达到 70% 共识；同样的 1800 个原始样本增加到 8192 次、3 个种子的 RANSAC 后，最高仍仅约 66.7%–68.7%，完整未抽样点集也不达门。最终源帧 `1791457418721812009` 的当前支撑已经恢复，170 环共识为 `0.711667`；但恢复复验时旧已接受参考年龄为 `818.877 ms`，超过原 `800 ms`，而当前帧年龄只有 `214.203 ms`。因此最终失效应记为保留参考过期，不能误记为该帧仍无平面或相机断流。

gen29 同像素发布深度存在毫米级时间变化，录制相机位姿基本一致；这些证据不能归因于桌面物理不平或手动移动。区域滤波参数快照显示滤波比例在 0%、13%、61%、74% 之间变化，但快照明确标记 `image_time_association=none`，没有逐帧滤波状态，不能证明具体失败帧由滤波重置造成。这里的原始数组是发布的 `depth/image_raw`，可能已经包含 SDK/区域滤波/投影处理，并非滤波前 SDK 深度。

gen30 的失败帧到达输入槽时仅 `7.644 ms`，排队 `187.542 ms` 后开始处理。decode 阶段耗时 `557.185 ms`，其中当前线程 CPU 仅 `3.564 ms`；随后 segmentation 耗时 `150.682 ms`，到 post_segment 时源年龄达到 `903.716 ms`。这把最大墙钟时间区间定位到 decode，但尚不能区分调度等待、GIL/原生锁等待或其他线程工作。新的时序细分埋点仍在另行处理，本文不提前给出根因结论，也不以延长时限或降低支撑门限替代修复。旧的 `HOLDING` 状态不表示这两次发生了新动作。

## 解码计时与 gen31 的实际结果

随后仅增加四段数值计时：相机参数读取、校正来源校验、CvBridge 转换和 float32 深度缩放；原处理次序、异常、源时效和身份检查保持。411 项回归通过，源码 SHA256 为 `30ecb30fb8441de1559ba3515621cb8d36e2ebace292979f7bd15961874f7873`。只重载空闲感知节点，其他九个节点进程、真实关节及控制代次均保持不变。

20 秒无动作观察得到 102 帧，其中 96 帧正常跟踪、6 帧支撑待复核；各解码子步骤最大墙钟耗时分别为 0.076、4.199、0.283、3.076 ms，没有复现 gen30 的 557 ms 等待。这不是负载下稳定性的证明。本机 ROS `get_param` 在订阅缓存建立后也读同一缓存，不能把改成 `get_param_cached` 当作已证实的延迟修复。

gen31 再次实际尝试时，两个请求进入初始接触候选计算，但分别被测量待确认和代次失效取消；首失效为 `exact_rgbd_stream_stale`。首次失效时，距最近 color 到达约 943 ms，距最近完整 RGB-D 输入约 909 ms。最后一帧正常发布前的 RGB 精修墙钟为约 619 ms、线程 CPU 约 72 ms；此时 decode 约 1.24 ms。首次失效后另一帧校正来源校验记录约 894 ms 墙钟、4.34 ms 线程 CPU；不能把失效后的等待倒推为首次失效原因。具体线程等待、传输停顿与运行负载的因果关系仍未证实。

gen29、gen30、gen31 均未执行机械臂动作，均未进入新 FK 补视角严格校验；三轮原始失败记录已封存，最后实测关节及控制代次仍与开始前完全相同。当前待解决问题是感知与规划同时运行时的时效稳定性，不能宣布补视角实物抓取已经通过。停止重复试抓，后续应先定位有时间关联的等待来源，不修改源时效、支撑门限或既有成功备份。

对应证据：`SESSION/verification/decode_timing/`、`decode_timing_deployment/result.json`、`decode_component_observation.json`、`gen31_failure_analysis.json`、`final_runtime_state.json`。gen31 录制为 `grasp_records/20261008T113249.626396Z_f59ae58e57c94c1e94358fb3e6ad2c2a`。全部新修复和诊断源码另存 `SESSION/deployed_source/`，尚未上传，固定 GitHub 标签不变。

## 本地证据索引

所有实验保留在 SanStand；以下 `BASE` 与 `SESSION` 是阅读缩写，不是运行时环境变量：

- `BASE=/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/user_grasp_20261008T090751Z/source_snapshot_review`
- `SESSION=/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/supplement_grasp_20261008T103117Z`
- 冻结原始审计：`BASE/gen28_observation_frozen_audit.json`，SHA256 `554fffc15e60d2cbda26b11c4d1cdc8319e3e91441f4180cebb196e5542acb5f`；源 stamp `1791451738727244377`。
- 实存模型：`BASE/gen28_fk_ik_100ms.model.urdf`，SHA256 `7812d6b69e854ed9f37b4e3cbd813ecd53dab3504065b2d04d287abc6fd852e0`。
- FK/IK 服务检查：`BASE/gen28_fk_ik_100ms.jsonl`；42 端点数值限位检查：`BASE/gen28_offline_ik_all42_6seeds.jsonl`。
- 冻结实测支撑与来源：`SESSION/frozen_view_source/source.npz`、`source.json`。离线 TF 重组有约 `5.55e-16` 数值差，此冻结几何不继承旧请求执行权限。
- 失败保留：`SESSION/strict_candidate/candidate0_following_path.json`、`contract_configuration.json`；中间候选：`BASE/gen28_reachable_view_robust_candidate_0.json`。
- 最终候选：`BASE/gen28_reachable_view_robust_margin_candidate_0.json`；完整检查：`SESSION/robust_margin0_full_check/result.json`，其中 `execution_authorized=false`。
- helper 回放：`BASE/gen28_reachable_module_replay.json`；单测：`SESSION/reachable_helper_pytest.log`、`reachable_helper_pytest.xml`。
- 集成回归：`SESSION/verification/fk_variant_integration/result.json`、`final.log`、`final.xml`，既有并发测试记录为同目录 `third.log`。
- 部署：`SESSION/deployment/result.json`；gen29：`SESSION/gen29_failure_analysis.json`、`gen29_support_diagnosis/diagnosis.json`、`gen29_temporal_metadata_review.json`；gen30：`SESSION/gen30_failure_analysis.json`。
- gen29 封存录制：`/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261008T110323.888418Z_6dd99ae4ff4340a6a35d509899eb6b85`；gen30 封存录制：同一 `grasp_records` 根目录下的 `20261008T111612.195125Z_9c2c07136e4e41fc93957bfa0c43a389`。两次均保留失败结果，未改写为成功。
- 本轮 helper SHA256：`6992661a3344cb7337482a24479c82a63c7ebe5c9a61aadf4d77930507d65218`；对应测试 SHA256：`049f8da22dd2089078ec8192e66f399df51a25ec2412b31713125976a0867372`。

后续维护应保留上述失败与成功计算证据，分别记录新代次部署、原门复验、动作执行和实物结果；不能用本页的规划通过替代实机记录。
