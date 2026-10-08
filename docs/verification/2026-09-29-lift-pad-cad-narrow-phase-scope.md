# 成功夹持后的提升接触窄相位范围审查

本轮仅离线审查，未修改生产代码、参数、目标几何或 pad 阈值，未调用 ROS。用户确认该次电机已夹在两指之间；程序没有独立确认附着，提升未执行。

## 结论

可用真实 50 mm CAD 做更紧的窄相位，但这不是足以使本次提升通过的最小热修复。原矩形包络的空角明显放大了拒绝：左右指的 pad 余量为 −5.249 / −4.538 mm。进一步以冻结 STL 表面裁剪同一目标 OBB，**两侧真实 CAD 均有约 0.529 mm 的 pad 投影越界点**。这些点分别位于目标 OBB 内部至少 0.321 / 1.597 mm，原 jaw penetration 约 0.469 mm，未越原深度范围。因此不能把所有拒绝归因于矩形空角，也不能承诺凸分解即可通过。

该新结果是数值表面诊断，不是完整实体碰撞证书。它保留原 OBB，所以不证明真实电机材料穿入该 CAD 区域；真实电机可能比 OBB 更窄或在圆角处退缩。它也不把约 0.5 mm 的指尖倒角自动算作平面 pad。若 pad 定义、目标 OBB 与接触语义都保持不变，单纯缩紧夹指碰撞外壳仍有剩余矛盾。

## 可复用资源与绑定

使用 `src/arm-mujoco/synriard/meshes/Alicia_D_v5_6/gripper_50mm/Link7.STL` 和 `Link8.STL`，SHA256 分别为 `d546b7ab908c74281a41261412089a2f83f7519c2444739afe8ffff46450d19d`、`5e5b548734269b731c1d60502c742e3d70d5a97a1e8316db523e2232b4c5043b`；这与生产 `gripper_geometry.py` pad 常量注释完全一致。

各网格 2650 面、1313 顶点，全部边恰好二重、方向配对一致，没有数值退化面。这只验证基本闭合拓扑，未证明不存在自交。各凸包体积比有向网格体积大 56.36%，所以单凸包不是精确 CAD。仓库只有通用 `src/arm-mujoco/utils/convex_decompositon.py` CoACD 工具，没有这两指的已验证分解；近似分解结果不能未经覆盖和误差证据直接作为放行模型。

变换应来自冻结 `Alicia_D_v5_6_gripper_50mm.urdf`，通过其 `Grasp2tool` 转到当前同语义的 tool0。实际开度 `g=0.01275 m` 对应原 50 mm 关节 `left=−(0.05−g)/2`、`right=(0.05−g)/2`，与数字孪生 `_finger_qpos_for_inner_gap` 一致。解析几何以 +Y 为 `left_finger`，它实际对应 50 mm **Link8**；−Y 为 `right_finger`，对应 **Link7**。必须显式按空间侧绑定，不能只按 URDF 名称对号入座。

当前候选运动学 URDF 的 `package://alicia_d_descriptions/meshes/Link7.STL`、`Link8.STL` 哈希不同，且夹指开度模型不同；不能把这些资源代替上述冻结 50 mm 实体。新窄相位需同时绑定 arm URDF SHA、50 mm CAD/源 URDF SHA、tool0 映射、实际开度与分解产物 SHA。

## 最小正确插入方式

1. 插入点为 `lift_path_continuity.py:222` 的 `_intersection_in_pad` 附近，而不只修 `validate_lift_trajectory` 初始分支。统一结果至少区分 `separated`、`existing_pad_contact`、`forbidden_or_unresolved`。真实无交集不得被当成已有接触加入 `allowed`；真实新接触也不能因恰好位于 pad 而绕过初始授权集合。
2. 保留原矩形 broad phase、掌/相机/安装架与支撑面判定。仅在 finger broad-phase 相交时，用冻结实体的经过验证的凸覆盖或完整实体算法检查交集。若用凸分块，所有块必须被覆盖，不能只取几个最近点或相交三角面；实体完全包含而表面不相交也要判出。CoACD 不是自动覆盖证书。
3. 每个交集都须保持原有限非凸 pad 多边形、`1e−8 m` 数值内侧条件和原 penetration 界限。凸交集投影需检查所有边/弦与 pad 凹口，不能只查顶点。几何近似、接触裕量和原 box padding 的区别必须显式，不能把碰撞壳缩小当作隐性公差放宽。当前 raw STL 诊断未加膨胀就已有越界，新增保守膨胀无法消除该见证。
4. `initial`（约第 325 行）、每个 `pose_check` 的 world/carried（约第 367 行）及整段膨胀检查（约第 375 行）必须一致采用同一分类器。world 使用现有 `size+2*relative_shift` 的完整逆运动包络；carried 相对夹爪固定，但仍核相同实体关系。保留无附着假设、桌上目标、两种占据、SDK bridge、完整 nominal/shifted 曲线和 3 mm 支撑面要求。
5. 最小回归包括：矩形空角但真实分离；真实 pad 内初始接触；当前真实 CAD 指尖/pad 外见证仍拒绝；凹口跨越；物体完全包含；后续新接触；区间内碰撞/SDK bridge；mesh/hash/开度错误、退化和超时拒绝。通过初始实体判定不等于整条提升或真实抓牢通过。

推荐后续先处理已被数据暴露的剩余语义：在保留完整 world OBB 作为碰撞障碍的前提下，研究能否用同一原始多视角证据形成明确的**接触局部实体上界**，而非直接将 OBB 换成少量稀疏点；未被观测的区域须保守处理。若证据不足则报告未证明，不扩大 pad、不删除目标、不改高度强行通过。

## 复现与归档

原始 `initial_pad_audit.json`、新增 `mesh_topology_readonly.json`、`mesh_boundary_diagnostic.json` 均位于：
`/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/candidate_real_grasp_20260929T052500Z/offline_initial_pad_audit/`

新增纯离线复现入口 `.runtime/audit_lift_mesh_boundary_diagnostic.py` 使用 NumPy 和固定 STL，0.9 秒完成。它只读取已闭袋报告与 CAD，输出诊断；没有 ROS 导入、发布器或服务。不是可执行提升许可。
