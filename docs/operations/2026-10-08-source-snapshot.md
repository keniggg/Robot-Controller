# 10.8日，一次性抓取通过的完整代码

这是按用户要求在 **2026-10-08 保存的当前完整源码快照**。10.8 是保存日期，不表示这一天又做了一次实物成功实验。最近可核实的一次性抓取成功来自 2026-10-06 的 generation 8，用户明确确认“已夹住新目标”，成功后闭爪保持、不提升。补充视角抓取尚未通过。

## 版本与验收边界

保存实际运行的 `.worktrees/protocol-v3-upgrade` 工作区，包含此前未提交的新文件和修改。根工作区 `master` 是旧代码，不作为本次运行版本。快照创建在独立分支并用独立标签固定，不切换或重置当前运行分支。

- 分支：`backup/20261008-initial-view-grasp-code`
- 标签：`snapshot-20261008-initial-view-grasp-code`
- 基础提交：`14ab345`，当前源码包括此后工作区的全部源码、配置、模型、测试及文档改动。
- 历史成功记录：SanStand `grasp_records/20261006T061242.642572Z_f8b97892d0a84d5599287c74677f32d2`。
- 实物确认：`ros_log/grasp_close_hold_20261006T061242Z/operator_confirmed_result.json`。
- 成功计划：`ee718df75606887be44ea8ff`，`observe_return / initial_contact / current_view_contact_snapshot`，初始视角直接完成接触抓取。
- 历史加载文件指纹：`ros_log/contact_tracking_final_validation_20261006T055016Z/validation.json` 和 `ros_log/contact_tracking_gateway_reconcile_20261006T055605Z/result.json`。

未找到成功时刻的完整源码树或独立 commit，不能声称当前所有文件与当时逐字节相同。独立复核历史 65 项文件指纹，64 项相同，唯一改变的是启动验证测试文件；其中 10 个关键生产源文件（任务、网关、接触执行、补偿、MoveIt、仿真等）均与历史成功前加载版本完全一致。当前直接抓取、`initial_contact` 及公共执行链的针对性回归共 652 项通过，无跳过；历史成功计划也通过当前只读结构准入检查，未复用或执行旧计划。

本轮支撑面修复另有 425 项测试通过、1 项因旧实测 fixture 缺失而跳过；原始 RGB-D 回放的 10 对可比较帧中，候选 mask、位置、尺寸、质量及全部候选指标完全相同。目前未发现破坏已通过的一次性抓取的证据，但这些检查不是新的实物抓取成功证明，也不能覆盖历史未记录指纹的全部文件。详见 [独立回归复核](../verification/2026-10-08-initial-view-grasp-regression.md)。

## 快照内容与恢复

仓库根目录保留源码、ROS 消息/服务、launch/config、模型/mesh、测试、工具及操作文档。此前被 `.gitignore` 排除而运行实际需要的源码配置另存于 `deployment/2026-10-08/`：

- `runtime/`：工作区 `.runtime/` 中的自有脚本、启动文件、候选机器人模型及配套手眼标定、仿真源码和配置；包含历史诊断工具，只作为备份，不自动执行。
- `root_workspace/.ros_log/`：根工作区中的实际启动脚本和外盘路径初始化脚本。
- `external_config/`：外置手眼标定、仿真服务 unit、仿真启动所需的固定源契约。
- `source_dependencies.json`：每份复制件的原路径、大小、权限和 SHA256。

下载固定标签：

```bash
git clone --branch snapshot-20261008-initial-view-grasp-code --single-branch \
  https://github.com/keniggg/Robot-Controller.git Robot-Controller-20261008
```

恢复到原机器时，先按 `source_dependencies.json` 核对复制件的 SHA256，再将配置和脚本恢复到其中的原路径，并重新构建 catkin 工作区。不要覆盖仍在运行的另一版本。若更换用户、目录或机器，需要一起调整 launch、脚本、模型 mesh 路径和配置中的绝对路径；仿真固定文件清单/哈希需要重新验证，不能只修改路径后跳过验证。

外部依赖包括 ROS Noetic、Python 3.8、MoveIt、RealSense SDK 2.56.5 及其 Python 绑定、MuJoCo 3.2.3，以及已有 GraspNet 服务/模型环境。RealSense 安装来源及文件指纹保存在 `runtime/realsense-2.56.5/install_manifest.json`；MuJoCo 文件指纹在 `runtime/candidate_simulation/pins.json`。本仓库包含项目模型和服务源码，不包含外部 WSL/CUDA 环境及未在项目中的模型权重。Git 下载得到的是源码及恢复配置，仍需安装依赖并构建，未进行空白机器全新安装验证。

不把 `build/`、`devel/`、缓存、原始实验录制、运行日志或凭据写入本次源码提交。运行所必需的固定仿真源契约作为配置单独保留；完整实验记录继续只保存在 SanStand。保持 `local_only`、`upload_enabled=false`、`delete_enabled=false`，本次仅上传用户明确要求的源码备份。温度查询保持关闭，不发布关节失能命令。

## 补视角当前卡点

generation 28 的链路为：初始识别和稳定跟踪 → 初始接触未通过 → 转补观察 → **补观察移动路径未规划成功**。尚未进入到位后的重新识别、接触或闭爪。

请求 14 的 8 个接触候选硬复核全部失败，补观察未进入路径检查；请求 41 有 3 个通过，但共享 3 秒预算只检查同一约 383 mm 距离的 4 个 roll，全部 `OBSERVATION_POSE_PLAN_FAILED`。其余视角未完成检查，不能据此推断全部不可达。接下来以冻结目标和真实关节种子的可执行观察路径为验收，检查接触候选前置依赖及视角搜索顺序。
