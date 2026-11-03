# 本地候选 MuJoCo 服务准备稿

状态：已准备，未监听端口、未修改 ROS 或 WSL。用户已有自主修复授权；候选成对加载的证据与当前控制故障由根任务分别审查，本目录不声称 prospective 已通过、不清除旧失败。

仅 `127.0.0.1:8002`，仅 `/health` 与 `/simulate_grasp`；不提供 `/predict`、`/sync_joint_state` 或硬件入口，不导入 ROS。原 WSL GraspNet 8000 保持原状。服务器复用 `tools/mujoco_digital_twin_server.py` SHA `919ddaf6…`，所有原物理参数和默认值保持，仅指定模型、原部署的 `max_snapshot_age_sec=120` 与 `ros_sync_joint_states=false`。

`candidate_absolute_meshdir.xml` 与既有四 body 候选 XML 仅两处文本差异：compiler.meshdir 指向项目原始网格绝对路径，以及移除已知的根外 REVIEW 注释。原 injector 会在首个 `>` 后插入 option，该注释使其插到 XML 根外，因此必须做这项非物理格式适配；原文件/注释和 hash 完整保留，XML 结构语义仍只差 meshdir。`pins.json` 固定候选 URDF/X/pair、实际 XML、server/dependency、MuJoCo 本地二进制/模块和所有网格。启动会验证 pins，通过真实 `_model_for_payload`（原 options/actuator/object injectors）编译，并运行原碰撞网格夹爪合同。编译用的 20g box 只是动态模型编译夹具，不是新估重或仿真抓取结论。

默认仅审阅，不加载 MuJoCo、不创建输出或网络：

```bash
bash .runtime/candidate_simulation/start_candidate.sh
```

离线编译检查，不监听端口（输出必须是 SanStand 上新的子目录）：

```bash
bash .runtime/candidate_simulation/start_candidate.sh --verify-only --output /media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/candidate_simulation_verify_NEW
```

未来根任务明确启动时的命令，**本轮尚未执行**：

```bash
bash .runtime/candidate_simulation/start_candidate.sh --serve --output /media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/candidate_simulation_live_NEW
```

所有请求原始 JSON、响应、耗时、health 和服务日志写外盘。实际编译 XML 副本也位于该次外盘目录，原 server 的临时工作目录与 MuJoCo 原生日志不会回退系统盘。挂载不可用拒绝开始/处理新请求。

`simulation_manifest.json` 可直接供成对 reload 的 simulation manifest 检查；其 health bindings 要求实际 XML/source/URDF/X/pair SHA、真实 MuJoCo 3.2.3、已编译夹爪合同及原 120s。`/health.digital_twin.ok` 为实际后端状态；顶层 ok 仍遵循旧专用仿真服务（无本地 GraspNet 因而 false），不能把顶层 false 解释成 MuJoCo 失败。X 是候选来源绑定，**不重复加到 qpos，也不声称模拟了相机/支架**。

既存模型边界仍保留：约 0.505mm 原 TCP 定义差、约 9µm 固定坐标对齐残差、MJCF J4 比 URDF 窄；本目录不新增跨模型数值硬门、不改限位、不改摩擦/力矩/夹爪/提升判据。ROS 侧 20g 上限和提升仿真仅诊断政策不在此修改。

尚未做 HTTP 监听/客户端连通、成对加载或真实抓取/提升验证。启动此独立服务本身不选择 ROS 仿真端点，也不代表候选模型可以清除现有控制故障。停止本服务只需结束其进程，不涉及任何机械臂节点。

离线验证：24 项新增合同测试和 11 项原服务相关纯测试合计 35 项通过；独立审查再次通过 24 项新增测试。真实 MuJoCo 3.2.3 离线编译 nq=15/nv=14/nu=8，原夹爪碰撞网格合同通过。没有调用 mj_step 或执行一次模拟抓取，不能据此报告抓取成功。health 的 `compiled_model_xml_sha256` 固定的是输入原 injectors 的底板 XML，不是某次 object payload 装配后模型的二进制 hash。
