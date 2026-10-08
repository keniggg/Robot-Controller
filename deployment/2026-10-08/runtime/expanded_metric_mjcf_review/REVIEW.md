# 离线 MJCF 派生审查补充

状态：仅待审查 artifact，未部署。`NOT_FOR_DEPLOYMENT` 文件名表示本轮尚未完成成对部署编排与证据复核，不表示约9微米既存残差永久不允许部署。当前主要待完成新姿态 prospective 验证和根代理部署编排。没有新增1e-12或任何跨URDF/MJCF运行硬门。

原WSL有效端点 `172.23.132.97:8001`；health部署id `6047f9a74046…`。归档服务、当前项目服务以及health source SHA均为 `919ddaf6c9499c86eb913ad5d44bd40dbb6aabde4e5268c0267c9fa7601b7dd6`。部署包/项目MJCF相同SHA `b349fae85b95e507a5441588f44bbeeb0af37e360a351ed66d3c28b40c0e1026`。health没有暴露当前远端XML字节hash，因此来源绑定依赖部署归档，不声称新读取了远端文件。

服务器按名称将原始Joint1–6弧度直接写qpos，无换符号/零偏。TCP位置取两指body原点均值，姿态取Link6，IK位置Jacobian也取两指均值。局部轴反号由body固定坐标变换解释，不是命令应再反号。相机FK映射为 `T_world_Link6 * H_zero_pose * X_candidate`；只是虚拟相机变换，未增加物理camera/mount模型。

四body旋转派生证明 `FK_MJCF_new(q)=FK_MJCF_old(q+delta)`：1114点最大平移差6.14e-16m。关节原始q、CAD/惯性/接触配置/夹爪/摩擦/力限保持；真实server夹爪网格合同通过。以下另列它与URDF的既存差异，不能混称同一模型。

| 抽样域 / 比较项 | 旧最大平移误差 mm | 新最大平移误差 mm | 最大值增量 mm |
|---|---:|---:|---:|
| URDF_legal / raw_tcp | 0.504634607 | 0.504616184 | -0.000018424 |
| URDF_legal / fixed_frame_tool | 0.008947753 | 0.008911997 | -0.000035755 |
| URDF_legal / fixed_frame_camera | 0.006883073 | 0.006917882 | +0.000034809 |
| URDF_MJCF_intersection / raw_tcp | 0.504723090 | 0.504682560 | -0.000040529 |
| URDF_MJCF_intersection / fixed_frame_tool | 0.008992353 | 0.008945487 | -0.000046867 |
| URDF_MJCF_intersection / fixed_frame_camera | 0.006869161 | 0.006931328 | +0.000062167 |

每个域均独立使用1114点：1024固定种子随机内点、64全部限位角点、24轴边界/nextafter、零位/中点。这是有限抽样，不冒充全域精确证明。

六轴合法交集rad：`[[-2.16,2.16],[-1.57,1.57],[-.5,2.35619],[-2.7925,2.7925],[-1.57,1.50],[-3.14,3.14]]`。旧MJCF的J4比URDF窄（±2.7925而非±3.14）；J5上限反而比URDF宽（1.57而非1.50）。165个URDF合法样本超旧MJCF域全部来自J4；候选没有改变这项。后续真实路径按原已有各自限位核对，不改变限位绕过。

现有未知物体初始IK为0.2mm/0.005姿态误差，见 `tools/mujoco_digital_twin_server.py:75-76,1693-1703`；轨迹检查同源常量见2449-2500。它们约束MuJoCo自身TCP相对请求pose，不是URDF↔MuJoCo模型误差门。因此0.505mm跨模型差不能直接用0.2mm判运行失败；该差原模型就有。未找到现有跨模型FK/TCP专门门限，不新造。提升仿真继续仅诊断。

若以后要求严格同一：从冻结URDF各轴origin/axis重新形成完整运动链，保留旧CAD/惯性在可证明固定link-frame中的坐标；独立命名ROS tool0，再一起修改 `_gripper_center` 与 `_solve_ik` Jacobian的TCP来源，并保持Link6实体的palm碰撞分类不随虚拟tool frame改变。不能只改返回TCP不改Jacobian，不能把手眼X写进qpos，不能把旧夹爪q零点改成URDF单指opening而不适配现有50mm合同。本轮按根要求不扩展该重构。

证据：`/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/grasp_repair_resume_20260928T120606Z/expanded_metric_mjcf_review_20260929/report.json`、`old_new_delta_and_domain.json`、`server_source_contract.json`。没有真实抓取/提升或prospective结论。
