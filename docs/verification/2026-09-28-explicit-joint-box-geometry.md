# 2026-09-28 显式六轴盒几何证明

为消除局部回返证书把真实各轴半宽合并为较大 L1 球所造成的过保守，本轮只新增显式盒证明，并将独立 fresh-return 的物理响应 cells 与 nominal SDK bridge cells 传入各自真实六轴半宽。原 40 mm 平面净距、20 px 完整板边距、120 mm 最低板深度及反馈范围不变，没有增加细分深度/次数。

旧 `bootstrap_mixed_geometry.py` 完整内容仍是新文件的逐字节前缀。`mixed_sync_geometry`、`certify_local_cell_geometry`、bootstrap CLI 与响应门保持不变。新增纯 API：

```python
board_camera_box_taylor_bound(fk, q6, X, Y, half_width_rad6)
certify_joint_box_geometry(fk, gripper, center7, geometry, lower6, upper6,
                          half_width_rad6)
```

各半宽必须有限且非负，总和不超过 2°（弧度输入）；允许零宽轴与零体积点。限制比较真实 `center±half_width`。返回格式兼容旧证书，另含 `joint_half_width_rad`、`joint_box_lower_rad`、`joint_box_upper_rad`；它只证明条件化几何，不授权动作，不证明全臂连续碰撞或实际控制响应。

## 数学依据

对盒 `|dq_i|≤h_i`，camera 坐标的线性项精确支撑是 `sum_i abs(J_ki)*h_i`，原 L1 球采用的 `rho*max_i(abs(J_ki))` 只利用了 `rho=sum h_i`，丢掉各关节宽度分配。新方法保留精确线性盒支撑；旧串联旋转积分 Taylor 余项仍是 `0.5*S*(sum h_i)^2`，全局 Hessian 向量范数界 S 没有改变。

每个 CAD 角点沿关节空间线段的位移由 `sum_i R_i*h_i` 保守包围，R_i 是 URDF 全姿态有效的该角点逐轴速度半径。完整板角点的 camera 坐标盒经原有每角、每cell正深度归一化、板内凸包与 SDK 外向区间投影覆盖整个板面。边界和随机内点测试只是实现回归，不替代这些连续证明。

## return 接入

`partition_cell` 把 `.5*(upper-lower)` 原样交给证书。每个叶在所有夹爪开度通过后才交给原 path；原 path 继续执行该真实叶中心的 MoveIt 验证，SDK 独立桥接也走同样路径。几何证书缓存不因“center / L1 半径相同”复用不同形状的盒：每次当前 cell 必须重新证明，随后原 path 立即消费。缓存无证书则拒绝，不能回退到旧大球证书或伪造通过。

source / epoch / SDK / actual、父原始绝对范围、每条发布起点的反向界、最多 2 条 J2→J3 one-hot、350 ms 板源龄、12 s leg / 110 s 总期限、失败不再发布均保持原合同。fresh actual 未被旧编码值覆盖。

## 最新失败场景离线重放

来源 `bounded_joint23_fresh_return_20260928T133453Z_239528/fresh_return_baseline_geometry.json`，source stamp `1790602495285155296`，SHA `420c1063289007c96f83664b88ee1c50f04d24885724051ea413b93eab7b4464`。全部旧 physical / nominal domain 不缩不扩，按原网格：

| 域 | cells | 不细分通过 | 最低平面净距 mm | 最低深度 mm | 最差单cell板边距 px |
|---|---:|---:|---:|---:|---:|
| 实际响应盒 | 1296 | 1296 | 59.757 | 216.327 | 23.166 |
| SDK 独立名义桥接盒 | 160 | 160 | 77.765 | 228.242 | 63.186 |

全部已证 cell 一起重做同场景 batch 投影，完整板最差边距下界为 **22.543549 px**。原先失败 cell 在这个场景中旧 L1 证书为 **18.350 px**，新显式盒为 **31.373 px**。本次几何重放耗时 4.77 s，没有 ROS node、MoveIt 调用或关节命令。它说明数值保守性改善，不代替下一动作的新鲜场景与 MoveIt 证明。

## 回归与冻结

主回归 159 项通过（1.93 s），覆盖新显式盒边界/内部、逐轴线性支撑、同rho不同盒、零宽与无效宽度、未放宽门限，及原 L1/bootstrap/J3/return 回归。随后追加 2 项实际半宽传递与 cache 形状碰撞反例，2 项通过（0.11 s）；运行代码保持冻结，合计 161 项。

- helper SHA：`f7a225c459c5f46ec09ad0cf72fdd8fca725947b55dfb14db432db325a0eeda9`
- return SHA：`4a857ea5eb12ac558194baf460ff5f0de953b95e2e78da58b56b7dd99498c2f0`
- counter SHA 保持：`55c8dec6ce0615e0c11c34fcb774c085eacce7d7cb65f65d17df1b876a1096ff`

完整离线 report、逐cell角点包围与分析源在 `grasp_repair_resume_20260928T120606Z/explicit_joint_box_return_replay/`。全部新实验继续由根代理独占；本代理没有发出任何硬件命令或修改 URDF/X/K。
