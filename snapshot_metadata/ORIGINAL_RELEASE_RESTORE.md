# 2026.10.16日未完成补充视角观察代码

这是用户要求先停止修复时保存的完整项目代码快照。包含磁盘上未提交、未跟踪和被 Git 忽略的项目源码、测试、配置、模型权重、网格、标定和启动工具；不是 Git HEAD 的普通源码压缩包。

- `current_runtime_workspace/`：实际运行项目的工作区 `.worktrees/protocol-v3-upgrade`。磁盘代码含尚未完成验证的 GEN31 修改，运行进程仍是 GEN30。补充视角实物抓取尚未成功。
- `ide_root_workspace/`：IDE 根工作区独立保存的完整项目源码。版本较旧，请勿用它覆盖上面的当前工作区。
- `external_dependencies/`：项目之外的手眼标定、仿真服务配置和固定源契约。原路径、权限、大小、SHA256 见 `source-manifest.json`。
- `historical_root_launchers/` 和 `.runtime/` 中的旧脚本/历史源码副本仅供恢复、追溯，解压后不会自动执行。不要把旧实验状态或授权当作当前机器人运动许可。

`source-manifest.json` 逐文件列出原路径、包内路径、大小、权限与 SHA256，包含精确排除清单以及两份工作区的 HEAD、分支、索引哈希和磁盘修改状态。构建产物、Git 数据库、日志、原始实验录制、缓存、代理会话和凭据目录不属于代码备份，已排除。未保存的编辑器缓冲区不在磁盘快照中。

恢复时先核对同一 Release 附件 `SHA256SUMS.txt`，将这个中文命名的 `.tar.gz` 解压到新目录，按 manifest 校验内容。使用 `current_runtime_workspace/` 作为恢复项目；外部配置按 manifest 的 source 路径恢复到隔离环境或按新环境调整。源码中的绝对路径、模型路径和固定源契约需要一起迁移并重新核验。

项目依赖 ROS Noetic、Python 3.8、MoveIt、RealSense SDK 2.56.5、MuJoCo 3.2.3 及已有 GraspNet 服务环境。包内保留项目本地已有的模型与运行库；系统 ROS、CUDA/WSL 和远程服务环境不作为项目代码打包。`src/CMakeLists.txt` 的 ROS 系统软链接需要安装 Noetic 后使用。安装依赖后重新构建 catkin；没有声称在空白机器上完成重建或实物验收。

Release 使用新标签附加到既有 Git 提交，不更新任何已有分支、标签或仓库文件。**此次完整工作区备份是本 Release 的中文命名 `.tar.gz` 附件；GitHub 自动生成的 “Source code (zip/tar.gz)” 仅对应既有基础提交，不能替代本附件。**

这是单次明确授权的代码上传。实验记录仍保存在 SanStand，通用上传和删除配置未启用。上传后继续软件修复和回归，完成后暂停，不启动新的机械臂抓取。
