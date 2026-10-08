# 2026-09-28 重启后的标定图像延迟定位

实验目录：`/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/grasp_repair_resume_20260928T113329Z`。
仅订阅现有图像和质量消息，并对一张保存的源 RGB 离线计时。没有发出关节、使能、去使能命令，也没有重启相机或修改线上参数。

## 实测结果

8 秒订阅窗口得到 57 张 RGB 和 20 条质量消息：

| 指标 | 中位数 | P95 | 最大值 |
|---|---:|---:|---:|
| RGB 接收时距消息源戳 | 48.9 ms | 68.3 ms | 116.4 ms |
| quality 接收时距相同源戳 | 541.6 ms | 746.9 ms | 798.3 ms |

20 条质量消息中 3 条 pose_ok。18 对具有完全相同源戳的消息中，quality 比 RGB 晚到的中位数为 481.3 ms，范围 241.3–735.1 ms。因此已观察到的标定反馈延迟主要累积在图像接收后的检测及 CPU 调度，而不是这段 RGB ROS 传输。

跟踪器直接订阅 `/supervisor/camera/color/image_raw`，该路径没有额外 relay。已有单处理线程、单个可覆盖待处理帧设计，不能仅凭检测慢就认定是旧帧无限排队。

**时间戳限制：** `camera_node.py` 在 `cam.read()` 完成后、发布前生成 ROS 时间戳。上述 RGB 接收龄不包含相机采集/SDK read 前的延迟，亦未测量 GUI 绘制链路。不能据此声称端到端 RGB-D 延迟只有 49 ms。

## OpenCV 线程与检测开销

OpenCV 4.13.0 默认线程数是 2，不能把进程总计 18 个线程等同于 18 个 OpenCV 检测线程。固定同一张图、1.5 倍检测与相同 ROI，2 线程各批次的 warm 中位数为 324–439 ms，1 线程为 399–441 ms；降为 1 线程并未证实更快。

单独四次检测分段：marker 搜索为 306–403 ms，ChArUco 插值为 10–99 ms，resize 为 0.26–1.07 ms，检测器对象构造通常低于 0.06 ms（一次调度抖动为 6.9 ms）。因此缓存构造器或取消 resize 不能解释主要开销。没有据此修改生产跟踪器算法。

## 可采用的有限优化

离线使用同一 RGB、同一 12×9 DICT_5X5_100 硬板配置、0.015/0.01125 m 方格/marker 和当前 SDK 几何，对比 1.0 与 1.5 检测尺度。模型指纹为 `e5231b8a08edf9c055b7dc3eb06296eab90660bde2861781aa19ec1d0be06765`。

| 检测尺度 | 三次角点数 | 重投影 RMS | 三次检测耗时 |
|---|---|---|---|
| 1.0 | 67 / 67 / 67 | 0.2016 px | 223 / 228 / 127 ms |
| 1.5 | 69 / 76 / 76 | 0.098–0.105 px | 249 / 152 / 469 ms |

两者边缘距离均约 113.8 px；满足当前 35 角点、20 px 边缘及 1.5 px RMS 门限。稳定 ROI 后的 1.0 减 1.5 姿态差为 `[-0.0235, +0.0013, +0.1836] mm`，平移模长 0.1852 mm，转角差 0.2648°。这是单张图的算法尺度敏感性，不是独立几何真值或完整跨姿态校验。

可以仅将只读观察器的 detection_scale 固定为 1.0，再以该配置重新建立本轮所有姿态基准；不要与 1.5 倍的旧基准混合判断。CPU 抖动较大，当前三次计时不足以承诺固定加速倍数。350 ms 鲜度门限、当前 SDK 相机模型和原有手眼参数都应保留。加载由父任务协调，本文记录时尚未加载。

## 原始证据

- `tracker_restart_source_latency.json`：原始消息接收龄与源戳。
- `tracker_restart_source_rgb.png`：离线比较使用的源 RGB。
- `tracker_restart_profile.json`：默认/1/2 线程重复检测记录。
- `tracker_restart_stages.json`：四次 ROI 检测分段计时。
- `tracker_restart_scale_comparison.json`：两种检测尺度的角点、RMS、位姿与耗时。

复现脚本位于项目 `.runtime/profile_tracker_restart.py`、`profile_tracker_stages.py`、`profile_tracker_scales.py`。离线额外负载只在这些短时比较中存在；首个 RGB/quality 订阅窗口结束后才开始离线检测。


## Ubuntu 整体卡顿的现场测量

本轮 Ubuntu 运行于 VMware，仅暴露 **2 个逻辑 CPU**。11:39–11:41 UTC 采样显示：CPU 空闲 0%、user/system 71%/29%，1 分钟负载 18.63，CPU PSI some avg10 98.27%。同时 I/O PSI avg10 为 0、iowait 为 0；连续 3.497 秒 `/proc/diskstats` 中移动盘 sdb 的累计读写计数没有变化。该静默写盘窗口仍有卡顿，证据首先指向 CPU 争用，不能把当前卡顿直接归因外盘。

内存总量 7.7 GiB、available 约 2.3 GiB；已有约 394 MiB swap 占用，但本次 vmstat 窗口没有 swap in/out。移动盘为 USB 480 Mbps，D405 为 5000 Mbps。USB 2.0 连接仍可能限制大规模 RGB-D 录包，当前静默窗口不能替代录包负载测试。

已备份并更新项目根目录及工作树 `.vscode/settings.json` 的 watcher/search 排除项，只排除 build、devel、install、.ros_log、pytest 和 Python 缓存；没有排除源码，没有强制重启编辑器。此措施减少无用索引，不能代替足够的虚拟 CPU。增加 CPU 分配需要在宿主机 VMware 配置侧进行。

本轮恢复保留完整 ROS 节点、连续监督及原手眼参数；启动发送正向使能请求，driver epoch 为 1790595159705830183。尚未发送本轮任何关节运动命令。旧 J3 返回计划不跨 epoch 复用。原始系统证据摘要在当前恢复目录 `system_contention_observation.json`。
