# 2026-10-08 锁定支撑面身份误判修复与抓取复核

用户先要求“进行抓取任务”，在第一轮规划失败后要求“继续处理”。本轮只使用这两条实际指令作为执行依据，不将其写成用户已确认对准，不复用旧代次的计划。当前会话继续未知小物体 `observe_return`、闭爪保持且不抬升。

## 问题与原始证据

第一次录制 `20261008T091003.189947Z_6a1fff03ce7c4b9892c2944d43f3070b` 的 generation 27 在规划阶段失败，原因为 `perception_failure: support_gap_reference_plane_changed`。未调用 `/grasp/start`，录制器 `active_seen=false`；313 组关键帧已封口，原失败结果保留。

首个失败源戳为 `1791450609844086647`。报错时当前帧龄约 151.376 ms，最近一次合格测量龄约 432.353 ms，均未到原 800 ms 上限。冻结平面为：

```text
[-0.030537463025834762, 0.034107498699356954,
  0.9989515212884051, 0.0063876970390733655]
```

其法向长度为 `0.9999999999999999`。`tabletop.segment()` 的 locked 分支每帧再次归一化，导致系数最大变化 `1.1102230246251565e-16`；`_hold_missing_current_support()` 和恢复路径以 `np.array_equal()` 校验不可变平面，从而将数值表示变化误判为参考被替换。

原始 bag 的失败帧、原相机参数及精确源戳 TF 回放复现了这个差异。回放必须使用感知节点原有 `transform_matrix()`，不同的四元数转矩阵计算顺序不能作为逐位复现依据。

## 修复范围

`src/alicia_grasp_modes/src/alicia_grasp_modes/tabletop.py` 的 locked 路径为 `Segmentation.plane_base` 保留输入参考的独立精确副本。距离计算、候选 `metrics.support_plane_base` 和足迹继续使用原归一化值。当前帧拟合路径不变。

不修改严格身份比较、支撑拟合门槛、800 ms 期限、目标关联、RGB 身份检查、轨迹与碰撞验证。支撑缺测时仍发布不可用几何，必须取得新的合格支撑和唯一实测目标才能恢复。一个中间实现使候选几何也使用未归一化输入，独立复审发现这会破坏缩放平面的米制高度语义；最终版本已移除该行为，并增加 `segment → refine_candidate` 的 3.7 倍缩放回归。

## 验证

- 修复前，3 个新增用例复现同一错误；其中节点用例从合成倾斜场景实际拟合平面，不注入假平面。
- 最终 9 个相关测试文件共 425 通过、1 跳过。跳过项为仓库未分发的旧实测 RGB-D fixture；本轮另有真实 bag 回放。
- 真实回放选取 12 个状态源戳，其中 2 个缺少同步保存的原 depth，明确标记为缺失。其他可回放帧的候选 mask、相机/基坐标位置、尺寸、足迹、质量和全部候选指标在修复前后完全相同。
- 原失败帧的平面身份比较由 false 变为 true；其 `support_current_audit_available=false` 和 `support_plane_inlier_ratio_low` 原样保留，未被记为合格支撑。
- 实际加载只替换 unknown perception 与持有同一分割函数的 remote planner；驱动、相机、motion gateway、任务、硬件接口、MoveIt、router、夹爪控制进程的 PID/start ticks 均保持，七项关节实测位置及 control epoch 不变。

## 新一轮实机观察

修复后录制 `20261008T092818.564023Z_decf934e904a4ca1916b775e621829ce` 绑定新的 generation 28。

在源戳 `1791451764729464530`，节点发布 `support_measurement_pending_revalidation`，最近实测龄 350.388 ms，未续期或发布可抓取几何；下一帧 `1791451764930738687` 通过原支撑及 RGB 检查后恢复 `recovered_unique_measured_frame`，目标未丢失。恢复支撑内点比例 0.713333、4 象限、残差 1.052 mm；RGB mask IoU 0.918132。此结果证实实机缺测/恢复路径可用，不等于抓取成功。

最终结果为失败，未提交运动：generation 28 稳定跟踪约 86.636 秒，期间 3 次支撑缺测均通过原规则恢复，但始终没有可执行计划，最后因 `exact_rgbd_stream_stale` 终止。911 组关键帧已封口，`active_seen=false`；没有 `/grasp/start`、新的接触或闭爪。这证明平面身份修复有效，不能记为补视角抓取通过。

规划才是本轮主要未解决问题。初始接触请求 1 的 18 项候选均未通过接触终点必要条件检查；进入 `observation_prefix` 后，请求 14 的 8 项接触候选硬复核仍全部失败，观察路径规划未开始。请求 41 有 3 项通过，但共享 3 秒预算仅允许检查同一约 383 mm 距离的 4 个 roll，均 `OBSERVATION_POSE_PLAN_FAILED`，其余视角未完成检查。尚不能认定其余距离不可达。下一步使用冻结目标、模型及真实关节种子验证补观察完整路径，不再以反复整轮重试代替可达性定位。

## 存储与操作约束

完整证据位于 SanStand：

```text
/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/user_grasp_20261008T090751Z/
  support_plane_repair/
    before_tabletop.py
    before_unknown_tabletop_perception.py
    runtime_parameters.json
    real_frame_replay.json
    support_plane_regression_before.log
    support_plane_candidate_metric_review_before.log
    support_plane_final_pytest.log
    support_plane_final_pytest.xml
    deployment/result.json
  retry_after_plane_fix/progress_observation.json
```

感知配置快照明确为事后读取；相机投影参数另与 bag 内 65 份可用快照核对。源码及操作脚本保留在项目，记录、验证日志和文档镜像直接保存外盘。保持 `local_only`、禁止上传/自动删除；未查询温度，未发送关节失能命令。
