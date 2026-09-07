# 无分类 6D 抓取离线任务交接记录（2026-09-05）

状态：**Task 1–5 已完成离线实现和独立复核；Task 6 离线验证已通过，真机
验收仍未执行。禁止据此宣称真实抓取、速度或夹爪抖动问题已解决。**

本文件是后续续接任务的首要入口。详细设计、任务步骤和逐轮审查证据分别位于：

- 计划：`docs/superpowers/plans/2026-09-03-class-agnostic-precontact-geometry.md`
- 规格：`docs/superpowers/specs/2026-09-03-known-to-unknown-class-agnostic-grasp-design.md`
- 对外验证记录：`docs/superpowers/verification/2026-09-03-known-mask-class-agnostic-precontact.md`
- SDD 账本：`.superpowers/sdd/2026-09-03-class-agnostic-precontact-geometry/progress.md`

本文保留 Task 3 中断时的原始交接内容作为时间线证据；恢复任务时应以文末
“2026-09-06 最新续接状态”为准，该节已取代中间各处“Task 3 尚未完成”或
“Task 4–6 未开始”的旧状态描述。

## 1. 不可改变的目标与边界

1. 当前可以先用已训练的 `carton` 分割模型走通流程，但它只允许提供
   mask 和诊断标签。
2. 目标身份、几何、候选生成与排序、碰撞检查、严格规划、最终修正、
   夹爪闭合、提升和结果验证必须与类别、标签、模型文件名无关。
3. 最终目标是未知、未预先分类、未针对性训练/分割的目标物；所有下游
   优化必须是通用抓取框架，不能为 `carton`/`canton` 写执行特例。
4. 不允许用 OBB 填补不可见体积。接触授权必须来自真实融合点云中位于
   两侧物理夹指区域的实测表面及逐点视角 provenance。
5. 本轮只进行了离线代码和测试工作。WSL、相机、串口、ROS master/节点、
   控制器、机械臂和夹爪均未启动；没有发布 enable、disable、stop、
   torque-off 或运动命令。
6. 后续若恢复硬件，仍不得发布停止/失能命令。真机前必须先完成 Task 3–6
   的离线门；历史“已对准”消息不能代替一次新的现场对准与授权。

## 2. 仓库与保留修改快照

- 唯一工作目录：
  `/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade`
- 分支：`codex/protocol-v3-upgrade`
- HEAD：`0eea76f`
- 这是 linked worktree；`git-dir` 为
  `/home/zhuyupei/alicia_wa_full/.git/worktrees/protocol-v3-upgrade`，
  common git dir 为 `/home/zhuyupei/alicia_wa_full/.git`。
- 写入本交接日志后的 2026-09-05 最新快照：58 个 tracked modified、17 个
  untracked（新增的第 17 个即本交接日志）；tracked diff 共 7,792
  insertions / 657 deletions。此统计包含此前操作者和多轮
  Codex 的全部保留修改，不能全部归因于本计划或 Task 3。
- 暂存区为空；`git diff --check` 退出码 0。
- 本轮没有执行 `git pull`、`git reset --hard`、`git clean`、checkout、
  rollback、branch switch、stage 或 commit，也没有把修改复制回主 checkout。
- 后续必须先运行 `git status --short` 和 `git diff --stat`，在当前 dirty
  worktree 上续接。不得为了得到干净结果而回滚任何现有文件。

## 3. 已完成并经独立复审的工作

### Task 1：opaque target identity 与计划绑定

状态：**离线完成，独立复审通过；未单独提交。**

完成内容包括 `TargetTrackIdentity` / `TargetObservation`、相位/代次/轨迹
生命周期、rich-plan track/refinement 字段及 hash/ROS float32 绑定、同轨迹
结构矛盾 fail-closed、模型名只作为诊断信息、最终 refinement 新代次但不
错误换目标身份。最终历史验证证据：

- Task 1 集成：977 passed；
- `catkin_make -j2`：exit 0；
- supervisor 全套：1,866 passed / 3 skipped / 7 个既有 ROS 警告；
- 独立复审：Spec PASS / Quality PASS，无未处理 Critical/Important。

这些是 Task 1 完成时记录的结果，不是本次暂停点重新运行的全仓验证。

### Task 2：有 provenance 的多视角实测表面融合

状态：**离线完成，fix round 3 独立复审通过；未单独提交。**

核心文件：

- `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/multiview_surface.py`
- `src/alicia_flexible_grasp_supervisor/tests/test_multiview_surface.py`

实现保持以下边界：严格去重的一参考/一移动 correspondence 内点独立决定
配准接受、transform、inlier count、overlap 和 RMSE；配准成功后，融合只
接纳从这些 seed 经每条欧氏距离
`<= correspondence_max_m` 的真实 moving sample 连通分量。融合不插值、
不填 OBB，每点保留不可变 view index/stamp。

Task 3 首次尝试曾揭示 correspondence-only 融合会丢掉新显露的相反侧面。
修复后，共享顶面 + 两个相反实测侧面的真实 append 路径保留了 21 mm 的
新侧面高度跨度。空间哈希的 signed-zero 精确边界漏点也已修复：当前采用
分块完整 `remaining × frontier` 图遍历，原样欧氏比较是唯一边授权。

最终证据：

- 系统 Python / NumPy 1.17.4：68 passed；
- Task 2 三文件集成：343 passed；
- 独立图 oracle：232 组 chunk、深链、断开簇、重复、点序对照全一致；
- 独立复审：原 Important 和 dense-bucket Minor 均 ADDRESSED，Spec PASS，
  Quality PASS，无新 Critical/Important。

保留的非阻塞 Minor：`RegistrationResult` 构造器尚未强制
`ok/code/count/overlap/RMSE` 跨字段一致性；留给最终全分支审查决定。

审计文件：

- `.superpowers/sdd/2026-09-03-class-agnostic-precontact-geometry/task-2-report.md`
- `.superpowers/sdd/2026-09-03-class-agnostic-precontact-geometry/task-2-review.md`
- `.superpowers/sdd/2026-09-03-class-agnostic-precontact-geometry/task-2-r1-review.md`
- `.superpowers/sdd/2026-09-03-class-agnostic-precontact-geometry/task-2-r2-report.md`
- `.superpowers/sdd/2026-09-03-class-agnostic-precontact-geometry/task-2-r2-review.md`
- `.superpowers/sdd/2026-09-03-class-agnostic-precontact-geometry/task-2-r3-report.md`
- `.superpowers/sdd/2026-09-03-class-agnostic-precontact-geometry/task-2-r3-review.md`

## 4. 当前暂停点：Task 3 尚未完成

Task 3 目标是“只用融合的实测双侧夹爪表面授权接触，并彻底删除 carton
实体 OBB 补面分支”。原实现子代理在代码与测试修改进行中达到使用额度，
没有生成 `task-3-r1-report.md`，也没有接受独立 spec/quality review。
因此当前 Task 3 只能标记为 **unreviewed work in progress**。

### 4.1 当前未审查 diff 中已经存在的内容

以下只表示代码已出现在 dirty diff 中，不表示设计或实现已获批准：

- `gripper_geometry.py` 已加入 frozen `BilateralSurfaceEvidence` 八字段接口、
  `evaluate_bilateral_surface_evidence()` 和
  `bilateral_surface_contact_bounds_m()`。
- evaluator 使用真实 `FusedTargetSurface.points_base/view_indices`，执行
  minimum 12 points/side、robust jaw width、双侧共同高度区间和 unique view
  counts；无标签参数。
- `tabletop_geometry_candidates.py` 已加入 fused-surface/finger-geometry
  proposal 输入；top-only 完整 OBB 回归要求返回
  `BILATERAL_SURFACE_EVIDENCE_MISSING`，真实注册双侧视图回归要求通过并
  审计 point/view counts、measured width 和 contact bounds。
- `remote_grasp6d_node.py` 已删除
  `TABLETOP_SOLID_OBB_CONTACT_LABELS` 和
  `support_anchored_solid_obb_carton`，`target_label` 在候选生成入口被丢弃，
  当前 forbidden scan 无命中。
- contact phase 的 tabletop 入口已要求 `_active_multiview_surface()`；缺失时
  返回 `BILATERAL_SURFACE_EVIDENCE_MISSING`。far-field observation phase
  仍明确记录 `contact_execution_gate_deferred=True`。
- GraspNet 初次 gate、tabletop normalize、stable recheck 和 resolved-sequence
  路径已有使用 current fused points/evidence/bounds 的修改痕迹。

Task 3 六个 scoped 文件当前合计 diff 为 2,983 insertions / 246 deletions，
但其中包含 Task 1/2 和更早保留修改；续接者必须按函数与测试审查，不能把
该统计当作 Task 3 的独占改动量。

### 4.2 已知 TDD 过程记录

以下为中止前子代理回报，因缺少最终 Task 3 报告，应作为过程线索而非完成
证据：

- evaluator 初始 RED：缺失接口导致 1 个 collection failure；最小实现后
  focused 7 passed；随后完整 `test_gripper_geometry.py` 为 112 passed。
- tabletop 真实 producer RED：3 failed；实现后 focused 3 passed。
- remote 新门控 RED：6 failed / 1 passed，复现 top-only `carton` 被旧 OBB
  放行、其他标签结果不同、缺 active surface 仍放行、OBB 高度改变接触结果。
- 中止前 remote focused 曾到 6 passed / 1 failed；该唯一失败被判断为测试
  按列表位置比较不同 adaptive-tilt 候选，随后改为按相同物理轴比较，但
  没有留下最终完整报告或复审结果。

### 4.3 控制器在暂停前的最新独立验证

运行命令：

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH=/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages \
python3 -m pytest -q -p no:cacheprovider \
  src/alicia_flexible_grasp_supervisor/tests/test_gripper_geometry.py \
  src/alicia_flexible_grasp_supervisor/tests/test_tabletop_geometry_candidates.py \
  src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py \
  src/alicia_flexible_grasp_supervisor/tests/test_multiview_surface.py
```

结果：**480 passed / 1 failed / 14,688 warnings，exit 1。**

唯一失败：
`test_graspnet_contact_gate_uses_current_fused_surface_and_fails_closed`
（`test_remote_grasp6d_streaming.py` 约第 3010 行）。失败发生在测试构造
4×4 pose 时：`np.block` 收到 tuple-of-tuples，在当前 NumPy 中抛出
`TypeError: arrays is a tuple`；测试尚未进入 GraspNet 生产门控断言。
这个事实既不能证明生产门控失败，也不能作为生产门控已通过的证据。

14,688 条 warning 主要是新增 `_linear_quantiles()` 在较新 NumPy 上使用
为兼容 NumPy 1.17.4 所需的 `interpolation='linear'` 旧参数而重复触发的
DeprecationWarning。后续应在不破坏系统 NumPy 1.17.4 的前提下做兼容包装
或控制调用路径，并重新记录 warning 数量；不得简单改用只在新 NumPy
存在的 `method=`。

最新静态门：

```bash
rg -n "label.*carton|carton.*label|solid_obb_carton" \
  src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py \
  src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp
```

结果：无命中（`rg` exit 1 表示未找到）。`git diff --check` 为 exit 0，
index 为空。

### 4.4 Task 3 仍需完成的审查项

1. 修正上述测试 fixture 后，先单独运行该测试，确认它实际进入并覆盖
   GraspNet contact gate 的“当前融合表面通过 / top-only 或缺失表面失败”。
2. 重新运行四文件 481 项集合，必须得到 0 failure；随后运行 Task 3 brief
   指定的三文件集合并记录准确总数。
3. 全面审计初次 GraspNet、tabletop normalize、stable recheck、
   resolved-sequence 四条接触路径，确认 contact phase 都使用同一个当前
   phase/generation/track 绑定的 fused surface，而 far-field 只有 deferred
   observation authority。
4. 独立检查 evaluator 对 jaw、insertion 和 support-normal/物理夹指窗口的
   轴映射。当前接口没有单独 support-normal 参数，现有实现把固定 Alicia
   tool X/Z box 映射到 `cross(jaw,insertion)` / insertion；必须由 reviewer
   判断这是否完整满足 plan 中“project jaw/insertion/support-normal”的要求，
   不能凭当前 passing tests 自行宣布符合。
5. 检查 robust quantile、小样本、极端/重复点、view provenance、输入顺序、
   tilted insertion 和负轴工具配置；任何失败必须 fail closed，不能回退 OBB。
6. 评估 `np.quantile` 双版本兼容并消除海量重复 warning，保持 NumPy 1.17.4
   可运行。
7. 写完整
   `.superpowers/sdd/2026-09-03-class-agnostic-precontact-geometry/task-3-r1-report.md`，
   包含真实 RED/GREEN 命令、输出、修改范围和自审。
8. 生成 Task 3 review package，派独立 reviewer 做 Spec/Quality 审查；所有
   Critical/Important 必须按同一实现者 fix round + scoped re-review 闭合后，
   才能在账本写 `Task 3: complete`。

## 5. 后续任务安排（严格顺序）

### 第一优先：收口 Task 3

新会话必须先读本文件、plan/spec、progress ledger、Task 2 r3 report/review
和当前六文件 diff。原 Task 3 实现者因额度中止，可派新的唯一实现者接管，
但必须把本文件作为恢复上下文；不可并行派多个实现者修改同一文件。

建议的第一组只读/离线命令：

```bash
cd /home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade
git status --short
git diff --stat
git diff --check
git diff --cached --name-only
sed -n '1,260p' \
  src/alicia_flexible_grasp_supervisor/logs/2026-09-05-class-agnostic-precontact-handoff.md
tail -n 260 \
  .superpowers/sdd/2026-09-03-class-agnostic-precontact-geometry/progress.md
```

然后按 4.4 完成实现、完整测试、报告和独立审查。不要先启动 ROS/WSL/硬件。

### 第二优先：Task 4——用结构化 3D evidence 替代 centroid refinement

只有 Task 3 review clean 后开始。必须覆盖：

- clipped + 足够 3D registration -> `VALID_3D`；
- clipped + 证据不足 -> `CLEAR_VIEW_REQUIRED` 且不产生 pose correction；
- track/stamp/support/overlap/inlier/RMSE 任一不合格 ->
  `FINAL_REFINE_3D_INVALID`；
- 删除 `_copy_final_center_sample()`/2D mask centroid 的位姿执行权；
- 同 track/plan/candidate lineage 绑定，修正后重新计算 plan ID，并重新严格
  检查完整 pregrasp/approach/grasp/lift 序列；
- `final_visual_refine_center_fallback_enabled: false`。

Task 4 指定测试：remote streaming、grasp task sequence、rich plan candidate
source。仍遵循 RED/GREEN、实现者报告、独立 review、fix/re-review。

### 第三优先：Task 5——最多一次、按硬件时长排序的 clear-view reacquisition

只有 Task 4 review clean 后开始。生成两个对称、在接触包络外的无接触观察
位姿；每个只做一次 strict MoveIt preflight；按
`joint_duration_lower_bound_sec`、path cost、确定性 side preference 排序。
一个任务最多执行一次，成功后必须重新采集、重建、重绑并完整复核计划；
不可达时在接触前返回 `CLEAR_VIEW_REACQUISITION_FAILED`。不得加入类别路点。

### 第四优先：Task 6——全离线验收、文档和真机门

Task 3–5 和各自独立审查全部完成后，运行：

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_ENABLE_TESTING=ON -j2
python3 -m unittest discover \
  -s src/alicia_flexible_grasp_supervisor/tests -q
catkin_make run_tests_alicia_d_driver_gtest_actuation_confirmation_test
git diff --check
```

同时做 forbidden-command/semantic-branch 扫描、最终全分支独立 review，并
更新技术路线和验证文档。当前 dirty ownership 重叠，除非用户另行授权并
完成文件归属审计，否则保持 index 空，不强行拆 commit。

### 最后：有电真机验收

当前没有任何真机成功证据，机械臂速度慢、第二阶段中止、夹爪抖动三项都
不能标记解决。只有离线 Task 6 通过、操作者恢复接口并给出新的现场对准/
尝试授权后，才能按日志启动最新节点。验收必须监控 target track、fused
views、registration metrics、strict MoveIt stages、endpoint trim、SDK
composed target、encoder response、gripper command/feedback 和 task state。
任一证据不合格都应在接触前按准确 code 失败；不得自动发布 stop/disable。

## 6. 恢复时的状态判定规则

- 账本中 `Task 1: complete`、`Task 2: complete` 可直接信任，不得重新派发；
  除非出现新的可复现实据，只从 Task 3 当前 diff 继续。
- Task 3 当前不是 clean checkpoint：没有最终报告、没有独立 review、完整
  scoped suite 有 1 个 test-fixture failure。禁止写成“已完成”。
- Task 4、Task 5、Task 6 未开始；不得从后续代码中已有的早期字段或辅助
  函数推断任务已完成。
- 所有“pass/fixed/complete”陈述都必须附带当前会话新运行命令、完整输出
  和 exit code；子代理自报不能替代控制器复核。
- 保留现有修改；禁止 `git pull`、`git reset --hard`、`git clean` 和任何
  未经授权的回滚。

## 7. 2026-09-06 最新续接状态

### 已完成的离线工作

- Task 3 已补齐真实双侧表面 producer、物理 jaw/insertion/support-normal
  映射、所有 contact-authorizing 路径的当前 phase/generation/track 门，并删除
  `carton` 实体 OBB 补面执行分支。独立复核 Spec/Quality PASS，零发现。
- Task 4 已用结构化 `VALID_3D` 配准证据替换 2D centroid execution authority；
  修正后的 candidate/source/lineage/track/support/metrics/plan ID 完整绑定，并在
  atomic rebind 前重新严格检查 pregrasp/approach/grasp/lift。独立复核通过。
- Task 5 已实现最多一次 clear-view：两个对称姿态均 strict preflight，按
  hardware joint-duration lower bound、path cost、确定性 side 排序；camera/tool
  25 mm capsule 与 Alicia palm/双指 CAD 包络分别检查。移动后会真实推进
  `single_snapshot_direct` phase，取得五个新鲜 3D frame，检查 residual 和
  三维中心 jitter；任何失败在 approach/contact 前发布精确 code。
- Task 5 fix round 2 独立复核判定五项全部 ADDRESSED，Spec PASS / Quality
  PASS，无新 Critical/Important。

最新离线证据：Task 4/5 集成 573 passed；完整 supervisor pytest
**2042 passed / 3 skipped / 8 warnings**；catkin build exit 0，生成 8 messages /
11 services；supervisor unittest **769 tests OK**；driver gtest **49/49**；
`git diff --check` 通过；暂存区为空。协议测试只使用本机 `127.0.0.1` 临时
mock HTTP 服务，未访问外网。

### 仓库续接规则

- 只在
  `/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade` 继续；分支
  `codex/protocol-v3-upgrade`，HEAD
  `0eea76faa0a8887bebe9934178706dcd399cf39f`。
- 工作树仍包含大量操作者和此前 Codex 的保留修改，不能把整个 diff 归因于
  本计划。不要回滚、pull、reset、clean、stage 或 commit；先读
  `git status --short --branch`、`git diff --stat`、本文件、计划、规格和 ledger。
- Task 2 的 `RegistrationResult` 构造器跨字段一致性仍是已记录的 deferred
  Minor，交给最终全分支独立审查判断；它不应被悄悄遗忘或凭空升级。

### 离线 Task 6 已关闭；唯一剩余的是未来有电验收

最终全分支审查曾发现一个 Important：非零配准修正四个发布姿态后，没有对
修正后的精确姿态族重新做完整 strict MoveIt 检查。现已调整为先应用实测配准、
确定 diagnostic 和最终 `plan_id`，再从最终 rich plan 重建
pregrasp/approach/grasp/lift 并严格检查；成功前不提交 bundle/audit，失败会清除
旧 final 成功证据并返回原 MoveIt code。非零真实配准回归先 RED 后 GREEN，
四文件集成 **574 passed**。

独立复审确认原 Important **ADDRESSED**、无新 Critical/Important，最终
**Spec PASS / Quality PASS**。最终控制器证据：完整 supervisor pytest
**2043 passed / 3 skipped / 8 warnings**；catkin 生成 8 messages / 11 services；
linked-devel import 通过；unittest **769/769**；driver gtest **49/49**；生产静态
门和 `git diff --check` 均通过；index 为空。`RegistrationResult` 直接构造器的
防御式一致性仍保留为无生产触发路径的 deferred Minor。

至此 Tasks 1–6 的离线实施、验证、文档和独立复核均关闭。powered steps 继续
标为 deferred：WSL 和硬件接口均由操作者关闭，历史“已对准”不能作为新动作
授权。只有操作者未来恢复接口、重新对准并明确授权一次新的受控尝试后，才能
按启动日志恢复节点并采集真机证据。

真机仍需验证三项原问题：观察轨迹实际时长、第二阶段完整执行、夹爪是否再
抖动。届时必须监控 opaque target track、fused views/provenance、registration、
bilateral contact、四阶段 strict MoveIt、endpoint trim/encoder response、
gripper command/feedback 和最终 lift 状态。缺少任一授权证据均应在接触前按
精确 failure code 失败；不得自动发送 stop/disable/torque-off。
