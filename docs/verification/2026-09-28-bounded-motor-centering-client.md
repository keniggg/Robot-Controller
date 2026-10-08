# 三轴有界电机居中观察客户端

`.runtime/bounded_motor_centering.py` 是独立诊断客户端，默认仅准备，无命令发布者。它不清除既有抓取故障、不设置 production track、不修改标定或识别阈值。只有后续独立启动显式执行时，才可能对 J1/J2/J3 各发布一次 one-hot 命令；最多三条，没有自动返回、追偿或循环重试。此文记录实现合同，不代表实机已经通过或完成居中。

历史 motor/table 报告只提供当前固定标定板上的电机邻域和表面尺寸，用于关联同一个诊断候选。所有关节起点、SDK 参考、RGB/Z16/板姿和相机模型均重新采集。只使用 exact RGB、Z16、ChArUco quality、camera/board TF 四元源；保留原接收时间，写完候选证据后再次验证源时效。独立邻桌平面拟合排除整块板及其外侧 6 mm，筛出高于该平面的局部表面。原始 2 像素网格上只允许一个至少 15 点的连通域，主域占比至少 85%，板坐标中心距既有候选不超过 8 mm，X/Y 表面尺寸比处于 0.5–2.0。该关联仍是诊断证据，不是生产识别身份。

一次最小范数解只使用 J1/J2/J3，各轴不超过 3°，使用固定 1e-4 rad 的投影差分，避免 SDK float32 导致数值雅可比退化。向零量化至编码器字，目标采用原 SDK 加量化增量，保持旧有命令/反馈偏差。解算后冻结命令序列；每轴执行之前重新取实际关节、SDK 与板/邻桌证据，为当前单轴的独立响应盒和另行计算的 SDK 桥接盒重新证明。后续轴不沿用首轴几何。

实际总边界固定在初始姿态：J1/J2/J3 ±4.8°，J4/J5/J6 ±0.4°；每步非活动轴还有相对该步实际起点的独立 ±0.4°限制，活动轴反向不超过 0.5°。每条命令必须由更新后的真实写入 SDK 字确认，再以至少 0.8 s 的独立实际反馈窗口证明停稳，才允许下一条。冻结 driver epoch 与夹爪范围；不提供 enable、disable、stop、sync 或 bootstrap 路径。

连续每轴盒使用 `certify_joint_box_geometry` 的真实各轴半宽，不以统一球替代。证明包含板几何和 CAD、独立邻桌平面与可见电机最高点以上的 40 mm 净距；临发布用新邻桌法向、截距和电机高度直接重算同一整盒的全部 CAD 角点及其运动半径，不把相关的法向/截距变化分别在基坐标原点取绝对值扣减。MoveIt 检查每盒中心和三条名义桥接路径的 17 个点，不宣称全机械臂连续碰撞证明。全板 20 px 边界只是本有界诊断使用的可见性证明，未写入生产抓取的永久条件。

默认准备命令（项目 worktree 内运行）：

```bash
source devel/setup.bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 nice -n 10 \
  tools/with_local_realsense_sdk.sh /usr/bin/python3 .runtime/bounded_motor_centering.py \
  --motor-evidence '/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/grasp_repair_resume_20260928T120606Z/motor_adjacent_table_normal_oriented/bounded_joint23_counterrotation_20260928T131913Z_197737/report_with_provenance.json'
```

系统 SciPy 与用户目录 NumPy 混用会触发 `numpy.typeDict` 错误，因此必须使用 `PYTHONNOUSERSITE=1`；脚本也在初始化 ROS 前检查 SciPy/SDK 导入。输出直接写 SanStand。相机、机械臂或挂载失败时不转存系统盘。

19 项纯合同测试通过，覆盖 SDK bias 保留、真实 one-hot 前驱、源晚于发布、实际响应/停稳、固定边界、起点漂移不能放宽 held 轴、epoch 故障锁定、多候选/移位拒绝等。已有三袋 40 帧诊断 npz 均通过连通域唯一性，历史实际图像半径为 96.7–107.5 px；这验证关联方法适用于已有帧，不是当前实机许可。客户端被动归档 production mask/status/object 和可取得的同源 RGB-D。终态明确给出实际候选像素半径，有限动作完成不等于目标已被生产 tracker 锁定或抓取已成功。

## 本客户端专用归档修复

14:07:54 会话发送且完成 J1 后，状态过期门拒绝了下一步；仅一条命令。袋中 accepted/SDK 的源时钟仍约 100 ms 连续，但接收时间分别出现 590/589 ms 空洞，随后多帧集中补入；actuation status 始终 CONFIRMED，接收空洞约 603 ms。该证据支持进程回调积压，不能据此称驱动掉电或停止更新。父代理独立在线采样随后恢复正常。

centering 客户端现在单独复用 `.runtime/bounded_async_bag_writer.py`：接收回调只序列化并提交不可变 bytes；单写入线程处理 bag，保留原 source/receipt stamps。总容量严格限制为 64 MiB 和 512 项，包含排队、正在写入及保留的失败原始消息。超过容量或写入异常立即锁存客户端故障，不延长任何实时门限。关闭先停订阅并等待已进入的回调退出，再排空全部已接受的项；异常消息的原始序列化 bytes 另存外盘，summary 明示缺失/失败，绝不声称主袋完整。此修改仅属于本诊断，不改变节点、驱动或其他记录器。完整语义消息源/receipt 保留已通过真实 rosbag synthetic roundtrip 验证。
