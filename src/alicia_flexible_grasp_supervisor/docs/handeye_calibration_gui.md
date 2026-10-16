# 主 GUI 手眼标定页

## 页面目标

主 GUI 提供独立的“手眼标定”页，用于 CC200 ChArUco 眼在手标定。该页面整合：

- 与原“关节控制”页相同的滑条直控和失败后正向使能恢复逻辑；
- Joint1--Joint6 的滑条目标值，以及每个滑条后的 `/joint_states` 实时实测角度；
- 按需开启的 RGB/ChArUco 标注图；
- ChArUco 质量显示、稳定性确认、样本采集/删除、算法选择、标定计算和保存。

当前 CC200 参数固定按技术路线显示为：

- 棋盘：`12 x 9`
- 字典：`DICT_5X5_100`
- marker：`11.25 mm`

页面不需要也不订阅深度图。

## 同页布局

关节姿态、RGB 图像和采样/计算功能同时保留在一个手眼标定页面：

- 左上：Joint1--Joint6 滑条、目标值和实时角度；
- 右上：RGB/ChArUco 图像；
- 下方：CC200 质量、算法、采样表和标定计算。

页面不使用横向 splitter、功能区切换页或内部 `QScrollArea`，因此右侧不会再被
关节滑条挤成窄栏；开启图像后，移动关节滑条时图像仍保持可见。

## RGB 按需订阅

页面打开时 `OnDemandRgbView` 不创建任何 ROS 图像订阅，图像区域保持关闭：

- 在“图像输出”下拉框选择“开启图像”：订阅 `/gui/handeye/image_topic`，
  默认 `/charuco/result`；
- 选择“关闭图像”：立即注销该订阅、清空图像并隐藏图像内容；
- 该下拉框只控制主 GUI 的图像订阅和显示，不负责启动或停止相机、
  `charuco_tracker` 或其他 ROS 节点。

因此，平时停留在其他页面时，该手眼页不会额外接收或转换 RGB 帧。

## 关节控制与实时记录

手眼页复用 `JointControlWidget` 的命令与恢复状态机，但只显示 Joint1--Joint6，
不显示夹爪。每行包含：

1. 关节名；
2. 原 GUI 水平滑条；
3. 滑条目标，单位 rad；
4. 编码器实时角度，单位 deg。

“同步当前关节”只把最新 `/joint_states` 同步到滑条目标。只有操作员显式启用
“关节直控模式”并改变滑条，页面才进入原 GUI 的正向使能、实测零运动同步、
有界验证步和目标发布流程。创建手眼页本身不会切换控制器，也不会发布关节目标。

成功采样时，页面把当时可见的六关节实测角度记录到本次 GUI 会话的样本表中。
easy_handeye 原始样本消息不包含关节角，因此刷新已有样本时，非本次 GUI 会话采集
的关节角显示为 `--`；标定计算仍使用 easy_handeye 服务保存的 TF 样本对。

## 采样门控

页面持续订阅低带宽 `/charuco/quality`，默认门限为：

- 数据 age 不大于 `500 ms`；
- `pose_ok=true`；
- ChArUco corners 不少于 `35`；
- 标定板到图像边缘不小于 `12 px`；
- 重投影 RMS 不大于 `1.5 px`。

点击“确认画面稳定”后，质量必须在 `2.0 s` 窗口内连续满足至少 `4` 次，
“采集样本”才会放行。发生以下任一情况都会取消本次采样许可：

- 任一关节滑条目标变化；
- 图像质量失效；
- 成功保存一个样本。

删除样本和保存最终标定都需要操作员再次确认。采集、刷新或删除样本以及改变算法
后，已有计算结果不能直接保存，必须重新计算。

## ROS 参数

参数位于 `config/gui_config.yaml` 的 `gui.handeye`：

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `calibration_namespace` | `/d405_cc200_recalib_20260809_eye_on_hand` | easy_handeye 服务命名空间 |
| `image_topic` | `/charuco/result` | 按需显示的 RGB/标注图 |
| `quality_topic` | `/charuco/quality` | ChArUco 质量 JSON |
| `max_quality_age_ms` | `500` | 最大质量数据年龄 |
| `min_charuco_corners` | `35` | 最少 ChArUco 角点 |
| `min_edge_clearance_px` | `12.0` | 最小图像边缘距离 |
| `max_reprojection_rms_px` | `1.5` | 最大重投影 RMS |
| `stable_window_sec` | `2.0` | 稳定检查窗口 |
| `stable_required_hits` | `4` | 连续合格次数 |

如果新一次标定使用不同 easy_handeye namespace，应先修改
`/gui/handeye/calibration_namespace`，再启动主 GUI。

## 文档分工

- 本文维护页面行为、操作合同和可配置参数；
- `grasp_task_technical_route.md` 维护该页面与整体标定/抓取技术路线的关系；
- `logs/2026-07-23-ros-latest-node-launch.md` 按时间记录代码落地、测试和每次实机运行；
- 标定产生的审计 JSON 继续作为数据证据单独保存，不把数据内容复制进设计文档。
