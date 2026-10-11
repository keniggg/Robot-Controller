# 2026-10-11 源码快照恢复说明

本快照的主源码来自 `.worktrees/protocol-v3-upgrade`，包含保存时的修改和未跟踪源码、配置、模型网格、工具、测试以及全部已有操作/问题文档。最终实验与修复结论见 [问题与修复入口](../../docs/operations/2026-10-11-code-snapshot-and-grasp-issues.md)；源码归档不表示补视抓取已成功。

`source_dependencies.json` 为原路径到本目录复制件的精确映射；每项包含大小、文件模式和 SHA256。请先在新目录检出固定提交/标签并核验 `source_tree_manifest.json`。主树可以重新构建，`build/devel` 不是源文件，未收录。

恢复顺序（本脚本没有执行这些步骤）：

1. 安装已有版本的 ROS Noetic、MoveIt、Python/相机依赖及固定 MuJoCo 3.2.3 环境；核对 dependency 清单。未进行空白机器安装测试。
2. 按精确映射将 `runtime/` 恢复至工作区 `.runtime/`，将必需的外置手眼与固定仿真契约恢复至原路径。另一机器改变路径时，launch、模型 mesh、相机标定、仿真 pins 与相关代码里的绝对路径应一致更新并重新审核。不要通过跳过 pins 检查来运行。
3. 外部模型权重没有加入本次新快照树。清单给出 SHA256、原路径和已有历史提交；可使用现有本地文件，或从已有仓库历史恢复相同路径后核验 SHA256。未在本次上传或下载 CUDA/WSL、外部 GraspNet 权重及第三方 native SDK 二进制。
4. 重新构建 catkin 工作区。当前完整启动入口为 `runtime/start_full_latest_no_skin_20261010.sh` 与同名 launch，电子皮肤为关闭。`runtime/observation_measurement_consumers_20261008.launch` 是专属三计算节点的配置，恢复时应核对当前配置入口及 ROS 节点唯一性。旧 `historical_root_entries/` 仅作历史保存，不能作为当前默认启动入口。
5. ROOT 的独立磁盘编辑仅在 `root_workspace_overlay/` 保存，其基线提交与删改清单见 `root_workspace_overlay_manifest.json`。需要恢复旧 ROOT 时请另建基线 checkout 后应用；绝不能覆盖当前主源码。未保存的 VS Code 编辑缓冲区不属于磁盘快照，不能声称已经保存。

该准备目录的 `tree/` 在 SanStand 文件系统上用字节保存软链接目标，Unix 文件模式由私有 Git index 精确保留。请从生成的 Git 树/提交检出；不要把准备目录当作已经恢复好的工作区直接构建。

原始实验、rosbag、RGB-D 流及运行日志继续保存在 SanStand。这里只包含必要的固定配置、紧凑测试 fixture 和修复/验证元数据，不改变 `local_only`、`upload_enabled=false`、`delete_enabled=false`，不启用归档自动上传/删除。本次来源清单生成与快照准备没有调用 ROS、温度或机械臂控制，也没有创建 commit、branch、tag 或 push。
