# 有限标定板居中观察客户端

本轮固定板的完整独立响应范围没有通过原 20 px 全板边距，默认只读入口正确拒绝，发布 0 条命令。不是仅由 Taylor 包围过宽导致：最新 `bounded_board_centering_20260929T022440Z_98969` 的真实 SDK 投影存在域内反例。

| 首轴 | 域内实际关节增量 J1…J6，度 | 真实最小全板边距 |
| --- | --- | --- |
| J2 负向 | −0.4, +0.5, +0.4, −0.4, +0.4, −0.4 | 17.1323 px |
| J3 负向 | −0.4, +0.4, +0.5, −0.4, +0.4, −0.4 | 16.3057 px |
| J1 负向 | −4.8, +0.4, +0.4, −0.4, +0.4, −0.4 | 3.3025 px |

每行都是原完整响应域内的一个具体点，足以拒绝；这些顶点采样不构成通过证明。进一步细分不能消除真实反例。板位置发生改变后，必须从新原始 RGB/Z16、板检测、TF、accepted/SDK 与 epoch 重新计算，不复用旧绝对板坐标。

新代码为 `.runtime/bounded_board_centering.py`，默认只读。它只复用已有冻结契约及写盘器，没有修改 motor/wrist/base/helper 或运行标定。以当前新板中心 `[0.09,0.0675,0]` 为观察目标，一次求解 J1/J2/J3，每轴最多 3°，量化保留原 SDK 偏差。命令顺序 J2、J3、J1，每轴一次，最多三条 one-hot 命令。没有自动返回、重试、同步或使能命令。

原 40 mm CAD 到板平面距离、20 px 全板边距、120 mm 深度、初始实际 ±4.8°、其他轴每段 ±0.4°、反向最大 0.5°全部保留。实际域与 SDK 名义桥接独立证明。每一段重新捕获源、计算完整轴盒、验证 MoveIt 与临发布场景，不重用上一段的许可。

客户端首命令前保存 3 秒 paired RGB、quality、TF、accepted 与控制数据，沿用 `/bounded_wrist/paired_rgb` 和带 `command_stamp_ns` 的命令事件，供 prospective evaluator 区分基准和验证窗口。停稳判据仍需独立 0.8 秒，单段等待 12 秒、停稳后记录 3 秒，全程上限 180 秒。实际证明和服务耗时依现场而变；不存在保证完成的时间。

原 URDF SHA `890592a24b8ad818d995e69beb670ea4254bf84f589615bfac3c8562f62449b7` 在初始化和每次 snapshot 与 FK、归档和实时参数三方核对。首次新观测的 X 全矩阵、相机指纹跨阶段冻结。候选 URDF/X 未部署。

新版客户端 SHA `6b257600a08d39638a575dde066285d86ae13391919674a284be0f586657dbc3`。20 项新测试及复用契约、SDK 连续几何和有界异步写盘回归，共 163 项通过。首轮未设置本地 SDK 包路径导致 48 个导入失败，正确 SDK 环境复测全通过；没有改测试期望或几何门限。

外盘证据：
`/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/restart_grasp_20260929T013841Z/board_centering_offline_proof/`
其中 `final_review.json` 固定最新快照、模型 SHA、具体反例、零命令终态及源码；`regression.log` 为完整通过结果。新只读入口的 1,379 条归档全部写出，无溢出或写盘错误。

仅供新场景默认只读准备的命令：

```bash
source devel/setup.bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 tools/with_local_realsense_sdk.sh /usr/bin/python3 .runtime/bounded_board_centering.py
```

此命令创建新基准和原始记录。本报告不授权执行当前被拒绝的运动，也不表示抓取高度问题或候选标定已经通过新实机验证。
