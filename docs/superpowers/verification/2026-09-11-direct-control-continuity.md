# 2026-09-11 直控多关节目标与控制权切换连续性

## 用户要求与当前部署边界

- 一根滑条只修改一个明确通道，但先后操作的多个关节必须可以同时完成各自
  目标；不能把 one-hot 消息意图误解为“全机械臂只能一个关节运动”。
- 未操作过的关节不因 GUI 陈旧值或编码器稳态偏差被重新下达目标。
- 模式切换不触发旧轨迹、不重置为另一组伺服目标、不发送停止/失能。
- 修改实际运行工作树 `.worktrees/protocol-v3-upgrade`；IDE 根目录旧文件不
  是本轮运行源码。离线验证后，用户明确结束手动操作并授权交回控制；
  **19:54/19:55 已分别加载 driver/GUI，动态直控与抓取验收尚未完成**。
- 前一轮仅被动读取记录与离线修改。本轮授权重载沿用既有启动正向使能，
  没有替用户切模式、下发关节目标、自主抓取或发送停止/失能。遥测继续记录。

## 已复核证据

独立 bag 快照 `.ros_log/direct_control_review_20260911_VwcwmM/` 来自持续
遥测分卷 runtime_1.bag.active 的复制件；只重建复制件索引，未修改实时 bag。
结合已关闭 runtime_0.bag 共识别 36 次 gui_direct 滑条手势；消息保留 effort
one-hot 和完整 position，SDK 日志限流 1 Hz，不能冒充每一帧完整串口记录。

1. 19:10:20.980 取消直控前 SDK 日志角度约
   `[-111.5,33.0,-15.2,-.3,-9.8,-.3]`；实测
   `[-111.3574,32.6074,-15.7324,0,-9.9316,0]`。
   切换后没有新的 joint_commands，SDK 目标却变为实测值；随后 Joint2/3
   实测分别到 32.3438/-16.2598°，即额外 -.2637/-.5273°。这是目标重置引起
   摆动的直接证据，不是仅由视频推测。
2. 驱动旧逻辑在换滑条/手势间隔超过 .25 s 时，用最后发送位置重建未编辑
   通道，丢弃此前手动目标尚未发送/完成的部分。GUI 的单一 pending flag
   又只记住最后滑条，50 ms 内跨关节操作可能漏掉前一个通道。
3. GUI 每次新手势把其他滑条显示回填为实测角度，也会擦掉用户先前目标显示。
4. 记录中某些未编辑关节有小幅实测变化、邻近限流 SDK 样本未变；不能据此
   断定索引接错，也不能排除载荷/未完成跟随等实机因素。18:56:56 Joint3
   操作期间 Joint2 实测 +12.832°，需结合此前命令仍在执行分析，不能把所有
   多轴运动都认定为错误。

## 已完成的离线修复范围

- 手动控制周期内按通道累积已接纳目标；换滑条/释放/超时不删除其他用户目标。
- GUI 按关节合并待发输入，而非全局“最后一根滑条”；恢复握手失败/退出模式
  要清理未发送输入，不能恢复陈旧命令。
- 模式切换冻结最后成功发送 SDK 帧的精确位置，取消未发送的未来插补；编码器
  仍作为实测与自动交接校验，不能冒充伺服命令。无历史帧则切模式不开始发送。
- 编解码改为可精确往返的计数量化；新增每次成功发送的六关节量化目标诊断
  `/alicia_d/sdk_command`，用于区分输入意图、驱动输出和实测响应。
- 自动控制仍是完整多关节轨迹，原 .003 rad 新鲜实测交接校验、速度/加速度、
  真实运动确认、碰撞与接触精度要求不放宽。
- 所有权切换只清除旧运动的响应观察，不清除既有同步/真实确认；仅保持原
  SDK 帧期间不启动新的响应探测。首个真正接纳的新命令才重新进入响应监测，
  避免旧目标保持被误当成新探测，亦不以切模式伪造新的运动确认。
- 无历史 SDK 帧时，插补器在首个授权发送时才用当时反馈初始化，避免把
  很久前勾选模式时的姿态作为新的运动起点。
- 使能/重连/无效反馈导致的命令清理与模式切换、命令接纳和 SDK 发送共用
  所有权锁，防止清理中途恢复旧帧；直控不运行自动端点微调。

## 离线验证与发现的附带一致性问题

- 首轮 GUI/模式/串口相关测试 76 passed；增加模式交接与观测生命周期约束后，
  全量 Python 2265 passed / 3 skipped / 7 原有弃用警告（81.42 s）。末轮
  C++ 交接补充后相关 Python 69 passed；最终完整源码再次全量复核，
  **2265 passed / 3 skipped / 7 原有警告，89.16 s**，保存为 `pytest_full.xml`。
- C++ 首轮 75/76 通过，唯一失败是端点微调协调器仍复制旧的截断量化公式，
  与新 SDK 编码不一致。修复共享 codec 后，新增一项观测交接测试，最终
  **77/77 passed**，XML 为该诊断目录 `gtest_final.xml`，源生成报告为
  `build/test_results/alicia_d_driver/direct_control_continuity.xml`。
- 枚举全部 4096 个关节计数和 1001 个夹爪计数，确认 decode→encode 原样返回；
  旧截断公式存在丢一计数的输入。一个关节计数为 .087890625°，这是次级
  数值偏差，不能解释或替代前轮 Joint2 约 1.725° 的稳态误差与零响应。
- 覆盖跨滑条未完成目标保留、六关节并行目标、模式周期失效、两根滑条同一
  flush 均发送 one-hot、同轴合并而跨轴不丢失、握手期间另一轴有界等待、
  超时清空所有待发输入、滑条显示不被其他手势擦掉。未调用实机节点做测试。
- `cmake --build build --target alicia_d_driver_node actuation_confirmation_test -- -j2`
  成功；Python 编译和 `git diff --check` 通过。没有改任何抓取门限或关节限速。

### 19:47 编译产物与实际运行状态必须区分

| 项 | SHA256 / 状态 |
| --- | --- |
| 新驱动源码 | `b908bbb6a2d68b4a1f2e898e6f4beda031a29ec3c7733525c8bb311abe7360d2` |
| 新驱动二进制 | `dbd9c2472b7450e91fb98096b544db7722500aada67f5fc033cc03d53121c62a` |
| 新 GUI widget | `96ef42ac107befbc9658d18179f257ee96520269d98d4c4e0cd4ab28dd81f65d` |
| 实际仍运行 driver PID 6603 | 旧二进制 `bf1235620fc9232102f5e64be7fb6083122946fd46cdf0d897e1d99769f1e7b2` |
| 实际仍运行 GUI PID 6658 | 18:28 原进程，未加载修改 |

`/proc/6603/exe` 指向重链接前的旧 inode，路径显示 `(deleted)` 是构建替换了
磁盘二进制，**不表示运行中的驱动已结束**。ROS 主服务 PID 6526、原被动
遥测 PID 7666 持续 active。新增纯订阅记录服务
`alicia-direct-trace-20260911.service`，PID 42042，19:47:51 active；分卷
`direct_trace_*.bag` 位于本诊断目录，每卷 64 MB、最多 4 卷，记录输入/实测/
控制器/模式/状态/rosout 与新 `/alicia_d/sdk_command`。新诊断话题需新驱动
加载后才有数据，该 recorder 不产生运动或使能命令。

## 下一步现场验收边界

以下验收计划在 19:47 留存。用户随后已结束手动操作并交回控制，19:54/19:55
完成加载；尚未为了验证而切模式或滑关节，未发布停止/失能。动态验收仍分为：

1. 静止保持下两向切模式，比较 SDK 目标计数，要求切换本身不改目标计数；
   实测响应另记，不能由软件目标不变推断机械完全无摆动。
2. 单轴输入确认其余从未编辑通道目标不变；快速多轴输入确认已编辑各轴目标
   均保留并继续插补，不把合理的同时运动误判成串动。
3. 再独立验证 Joint2/3 的小命令方向响应/回差，随后调查固定目标的跨姿态
   坐标一致性。只有原始精度与几何门限得到真实通过，才继续完整抓取终态。

## 抓取未完成项继续保留

| 项目 | 状态与下一步 |
| --- | --- |
| 近场首个可达导致 Joint6 约 145° / 126.965 s 绕行 | 有界时长比较修复已加载；新一轮 8 查/4 可达；尚未证明实际执行短路径 |
| Joint2 端点微调无响应，预抓取实测误差 18.3 mm | 未解决；直控连续性修复后仍须独立验证小命令响应/回差，不能靠放宽 6 mm 门限 |
| 远近支撑面约 15 mm 跨姿态失配 | 未解决；到达后局部近场正常，需区分标定/运动学/深度链，不替换远场参考 |
| request 57 近场审计未落盘 | 证据缺口仍记录，需补齐终结与审计提交时序 |
| 完整接近、闭爪、抬升、视觉持物 | 尚无本轮成功终态；上项修复均不等于抓取成功 |

前轮完整数值、计划绑定和回放见
[晚间注册与路径验证](2026-09-11-evening-registration-repair.md)。测试和部署
结果在本文件追加，未完成项目不从主运行日志或技术路线删除。

## 19:54–19:57 用户交回控制后的实际加载记录

用户原话：“结束手动操作，可以交回控制，让你加载新驱动和 GUI”。只替换
这两个节点，独立只读审查确认析构没有 torque_off/controller-stop，两个
原 launch 节点均无 required/respawn。旧 GUI 用精确 PID 6658 的 SIGTERM
结束（其 Qt 主循环没有 rospy shutdown 联动），旧 driver 通过
`rosnode kill /alicia_d_driver_node` 退出，确认两个旧进程均消失后再开新驱动。
这些是用户授权的进程替换，不是机械臂停止/失能指令。

| 节点 | 新运行实例 | 运行证据 |
| --- | --- | --- |
| driver | `alicia-driver-continuity-20260911.service`，PID 45000，19:54:30 启动 | `/proc/45000/exe` SHA256 = `dbd9c2472b7450e91fb98096b544db7722500aada67f5fc033cc03d53121c62a`，与已测试二进制一致 |
| GUI | `alicia-gui-continuity-20260911.service`，PID 45301，19:55:03 启动 | rospy init 19:55:06；窗口 `Alicia-D 柔顺抓取上位机 v2` 可见，1320×860 |

两个服务均 WorkingDirectory 指向本工作树，source `/opt/ros/noetic/setup.bash`
及工作树 `devel/setup.bash`；ROS_MASTER_URI=`http://localhost:11311`，
ROS_LOG_DIR=`/home/zhuyupei/alicia_wa_full/.ros_log`，
ALICIA_CODE_REV=`8b25b91-direct-control-continuity-worktree`。GUI 继承
DISPLAY=`:0`、XAUTHORITY=`/run/user/1000/gdm/Xauthority`。启动程序分别是：

```text
rosrun alicia_d_driver alicia_d_driver_node __name:=alicia_d_driver_node
rosrun alicia_flexible_grasp_supervisor main_gui.py __name:=alicia_supervisor_gui
```

使用原参数服务器配置，没有重新运行 full_system.launch 或修改门限。驱动
参数前后通过 `rosparam dump` 落盘到诊断目录
`driver_params_before_reload.yaml` / `driver_params_after_reload.yaml`，diff 空。
原 ROS 主服务 PID 6526、remote PID 26533、原遥测 PID 7666、新直控遥测
PID 42042 未重启。只读 list_controllers 返回 alicia_controller 与
hand_controller 均 running；未调用 switch_controller。

### 正向使能与被动保持证据

- 19:54:31.058 驱动按原 `auto_torque_on_startup=true` 发送一次正向
  `SDK torque_on`。日志 `PENDING:POSITIVE_ENABLE_REQUESTED` 表示真实运动
  尚未重新确认，`motion_enabled=false` 是该确认状态话题，不是 torque_off。
  没有为了变成 CONFIRMED 而发探测步，没有额外 `/demonstration` 发布。
- `/alicia_d/feedback_ready=true`；重载前后读取的六关节位置均为
  `[-1.911340061705509, .5645049299419159, -.2699806186678729,
  -.007669903939428206, -.22856313739496056, -.0015339807878856412]` rad，
  夹爪关节 .04975 m。随后 5 s 被动窗口有 295 条 `/joint_states` 消息（包含
  心跳，不能冒充 295 次独立串口采样），六关节角度跨度均 0。
- 同一窗口 `/joint_commands` 与 `/alicia_d/sdk_command` 计数均 0；新诊断
  publisher 已注册，且直控 recorder 已订阅。没有新命令时不发 SDK 目标帧
  是预期行为，不能为“有数据”而制造一次物理运动。
- `/gui/joint_direct_mode` 参数与新 GUI latched 话题均 false，未制造控制权
  边沿。首次订阅在 GUI 构造期间短暂超时，随后已收到同值 false；不掩盖
  启动尚未就绪与已就绪的区别。
- `/grasp/state` 的实际类型是 `GraspState`，返回上次失败的 latched
  `FAILED / active=false / success=false`，不是本次重载造成的新抓取失败。
  未绑定或执行旧计划。

driver 的持久日志在 `journalctl --user -u alicia-driver-continuity-20260911.service`
及持续 rosbag/rosout；虽然传入了 `__log:=.../driver_continuity.log`，本次直接
启动的 roscpp 没有生成该独立文件，不把它写成已存在证据。GUI 独立日志确实
存在于本 ROS UUID 目录 `gui_continuity.log`。重载完成不等于模式切换动态验收、
Joint2 响应修复或完整抓取成功，上述未完成清单继续有效。
