# SanStand 实验记录存储

实验记录和运行日志主存储为 `/media/zhuyupei/SanStand/实验记录/Robot-Controller`。当前按最新用户指令采用 `local_only` 的最小问题回溯保存；问题、方法、验证、原结果及未完成事项保留，可无损去重压缩必要证据，再清理不重要的旧原件。已迁移、标定、当前控制文件、成功参考及当前未解决问题必要的代表数据保留，不按最新5份删除。

## 最新指令：最小问题回溯

用户要求保存能够以后回溯的有用信息，主要是出现过的问题、已经解决的方法和还没有解决的问题，“如何可以最小化保存就怎么来，其他的不是很重要的就继续清理”。该授权覆盖下文旧的全部文本原件与全部历史录制完整保留要求；“已经迁移的就不管了”继续适用。

可阅读的问题索引见 [最小实验历史与问题回溯](2026-10-13-minimal-experiment-history.md)，外盘镜像为 `文档/问题回溯索引.md`。关键文本/审计/验证依据按原路径和SHA256无损去重压缩保存。历史大录制保留完整非图像ROS状态消息和首尾图像样本后才执行精确清单删除，原结果分类保持真实；精确回执明确整份原录制不可恢复。当前控制与持续授权、标定、成功参考、正在使用的文件及当前复现所需代表数据保留。

本次计划、压缩回溯库、验证与回执位于 `maintenance/minimal_trace_cleanup_1791956657642651189/`。未来查旧路径时优先查问题索引与压缩库，不能因为未压缩原件已清理而误报证据未保存。源码、模型和构建仍留项目，通用上传/删除开关关闭，缺盘不回退系统盘；原低于10 GiB触发和80 GiB原始录制停止口径继续适用。

## 此前指令：撤回GitHub上传，SanStand保留资料

用户原话：“先不上传到github，就保存在sanstand吧，能清理多少使用空间就清理多少使用空间”。随后明确：“已经迁移的就不管了”。这些指令覆盖下段此前的上传与本地元数据删除授权。

后续不上传GitHub，问题、修复方法、验证摘要、原结果、授权、文本日志、审计和精确回执全部留SanStand。仍沿用低于10 GiB触发和80 GiB可用空间目标，原始失败录制能核准多少清多少；合格不足时记 `eligible_exhausted`。不能因此删除成功、未知、标定、打开或当前定位所需原件。

前一版任务已有资料包上传，后续上传已停止；其曾删除的本地元数据已从仍在外盘的已校验临时包逐文件恢复。最新指令之后，既有迁移、已完成的恢复及其压缩包均保持当前状态，不再自动补传、下载、恢复或删除。已删失败原始录制不属于元数据包，不声称可恢复。当前策略schema=3、`metadata_retention=local_only`，关闭元数据上传和本地元数据删除；通用归档上传/删除仍关闭。完整恢复与原始录制清理回执继续保留。

2026-10-14 UTC 收尾已完成：本地策略与100项通过的隔离测试源码SHA256绑定，关闭后续上传及元数据删除；`alicia-failed-record-space.timer`恢复每5分钟维护。三个通用上传/finisher服务保持停止，未重启机械臂或上位机。

此前授权下新增清理的第6轮失败录制为 `grasp_records/20261012T101953.207970Z_4e3aa6eb22544ecc91e46d6de1d2cf0c/keyframes.bag`，实际删除 **3,824,608,857字节（3.561944 GiB）**，SHA256为 `e25d6357cf18f64469195de87d0b41c3e47abd9e05ad9a40b0c47121461dfc8f`。原始大录制未上传，不能从元数据包恢复；该实验原结果仍为失败。此前本地恢复回执为 `maintenance/local_metadata_restore_1791955414686771130/receipt.json`，1469份恢复资料在此次收尾前后SHA256一致。已上传资料及其暂存包保持现状，未补传尚未完成核验的云端最终回执。

另外仅删除五次已结束隔离验证目录中的2699个合成临时文件，其文件分配空间共 **360,972,288字节（0.336182 GiB）**，测试日志、XML和结果全部保留。逐文件路径、大小、SHA256、意图与实际删除回执保存在 `maintenance/user_failed_cleanup_local_only_final_1791956035958323500/`；原有迁移材料、已恢复元数据、分类表、116项保护表及通用存储配置均核验未变化。

当前86份已核分类记录中，剩余可清理原始大录制为0。最新仅原始录制维护回执 `maintenance/user_failed_cleanup_space_20261014T053533Z_60174/receipt.json` 为 `eligible_exhausted`，新增删除0字节，核验时可用 **9,080,537,088字节（约8.46 GiB）**。未达到80 GiB，不扩大删除范围；可用空间会随持续日志写入变化。

## 2026-10-13 此前授权（已撤回）：目标80 GiB，元数据迁移至GitHub

用户原话：“现在就改成清理到80G的可使用空间，删除前保留问题、修复方法、验证摘要、原结果、授权、文本日志和精确回执这些直接上传到github仓库”。随后明确：“每份待清理实验会先把你列出的摘要、结果、授权、文本日志和删除清单上传到现有 GitHub 仓库并核验，sanstand就不用保存这些上传到github的文件资料了，以此来获得更多的可使用空间”。

该版授权的空间规则是：**可用空间低于10 GiB时触发，清理至可用空间约80 GiB停止**。空间口径继续适用，上传与本地元数据删除授权已撤回。仍从最早已确认失败、封口且无实物成功冲突的记录开始；受保护、正在写入及当前定位仍需的原始材料排除。合格记录不足时记 `eligible_exhausted`，不能为凑80 GiB删除成功或未知材料。

每批先生成逐实验问题/修复/验证摘要，打包原结果、授权、文本日志、审计及准确删除清单，直接上传现有 `keniggg/Robot-Controller` 仓库的独立 `failed-cleanup-metadata-*` Release。核验包内每个文件的原路径、大小与SHA256，再从GitHub逐字节读回核验上传附件。任何上传或核验失败都保留本地原件。核验通过后删除范围内的原始失败录制与已上传的本地元数据；删除后补传实际回执，并移除外盘临时上传包。外盘只留小型GitHub恢复索引及清理回执。

原始大录制不属于本次元数据上传范围；未声明的二进制资料默认保留。通用归档仍为 `upload_enabled=false`、`delete_enabled=false`、`archive_policy=local_only`，旧上传worker/finisher不启动。本次由独立失败元数据迁移与空间维护执行，不改变既有云端历史。策略使用 `failed_record_space_guard_policy.json` schema=2，目标80 GiB，并要求 `metadata_github.delete_local_metadata_after_verified=true`。

以下70 GiB段落是历史执行记录；当前80 GiB目标和本地保留规则以本文开头的最新指令为准。

## 2026-10-12 新授权：低于10 GiB时清理至约70 GiB

用户原话：“如果sanstand空间不够，就按之前我的要求记录在文档中的，清理70G出来，以后少于10G时就清理70G出来”。沿用此前已经执行并记载的口径：**可用空间低于10 GiB时触发，清理至可用空间约70 GiB后停止**，不是每次固定删除70 GiB。本次新授权允许持续执行这一条件规则。

仍按实验开始时间从早到晚，仅处理有明确失败、封口和非实物成功证据的原始大录制。每目录先保留问题、方法与验证摘要；结果、授权、文本日志、审计、恢复元数据、原始manifest和精确删除回执保留。成功（包括夹住后提升报错）、未知、标定、打开/正在写入，以及当前问题定位仍需要的数据排除。清理目录和保护列表需绑定审查证据；新增未分类记录默认保护。合格记录不足时记录 `eligible_exhausted`，不为达到70 GiB而删除受保护或未知数据。

采用独立的失败记录空间维护流程。通用归档仍为 `upload_enabled=false`、`delete_enabled=false`、`archive_policy=local_only`，存储控制保持 `resumed`；不启用上传或通用归档删除，不改变云端历史，不重启ROS/机械臂，也不查询温度或发送失能。缺盘、挂载身份变化或证据变化时停止，不能回退系统盘。持续规则与已核候选/保护列表保存在外盘 `maintenance/failed_record_space_guard_*.json`，逐次精确执行计划及回执保存在外盘maintenance。首次执行结果及定时维护部署状态见下段。

首次执行已结束：按原开始时间清理85份失败且封口的原始bag，共 **50,485,603,220字节（47.0184 GiB）**，分配空间50,491,293,696字节。结束时可用 **55,852,400,640字节（52.0169 GiB）**。这批合格候选已用尽，未达到70 GiB，状态明确记为 `eligible_exhausted`；成功、未知、标定与当前定位证据保留，没有为凑容量扩大删除范围。每目录保留新摘要及精确 `空间清理回执_*.json`，原manifest及结果未改写。

本次计划、逐文件实际SHA256、执行版本绑定和最终核验位于外盘 `maintenance/user_failed_cleanup_space_20261012T085620Z_111214/`。核验确认 **1274份保留元数据SHA256、231份受保护原始文件身份、三份存储配置SHA256和844份机械臂部署源码** 均保持一致。普通用户执行器采用已实测支持的Linux写租约：已有打开者或新的打开请求会拒绝/取消删除，原始文件校验后先保存持久intent，再核对同盘临时位置的inode后删除；未完成intent须恢复核验，不能自动跳过。机制依据见 [Linux文件租约说明](https://man7.org/linux/man-pages/man2/f_setlease.2const.html)。本轮执行源码快照保存在项目 `.runtime/failed_space_cleanup_execution_source_20261012.py`，与回执哈希一致。

持续入口为项目 `tools/maintain_failed_record_space.py`，18项隔离测试通过。用户级 `alicia-failed-record-space.timer` 已启用，每5分钟检查一次；独立service按普通用户权限运行，当前空间高于触发线，启动验证成功且未触发新删除。规则、精确已核清单与保护表分别为外盘 `maintenance/failed_record_space_guard_policy.json`、`catalog.json`、`protected.json`（后两者完整文件名前缀同为 `failed_record_space_guard_`）。仅可自动补入至少7天前、complete/failure、双方封口、`active_seen=false`、`NO_FRESH_MODE_PREVIEW`、操作员观察为空、无成功/未知录制重叠且不在当前进度/保护表中的启动前失败；其他新增未分类记录继续保护，需要明确分类复核。合格数据不足时保留 `eligible_exhausted` 回执，不能承诺总能达到70 GiB。

用户最后要求清理完成后暂停后续抓取。本轮未启动第6次抓取、未创建新录制或新selection；generation仍为5。任务与动作已核为空闲，恢复时等待GUI自身持久发布者确认手动模式，再独立复核。空间维护按持续授权保持运行，抓取等待用户唤醒。普通后台维护暂停仍由 `maintenance/storage_control.json` 的 `state=paused` 阻断。

## 2026-10-04 用户授权的一次性空间清理

用户明确要求从最早失败实验开始删除约 50G 原始记录，保留发现的问题、解决思路和方法、验证记录，达到约 50G 空间后停止。本次已按时间顺序处理 2026-09-07 至 2026-09-28 的 112 份明确失败实验，仅删除清单中的旧 bag、图像序列和数据输入，共释放 49.149 GiB；停止时剩余约 50.314 GiB。实际夹持成功（包括自动程序在提升阶段报失败的重叠录制）、结果未知、标定和当前运行会话均保留。

每个已清理目录新增 `实验简要记录.md` 和 `user_cleanup_receipt.json`，保留原始结果、授权、文本日志、审计及恢复元数据。原始 manifest 作为历史证据未改写；其登记的部分录制文件现已按用户指令删除，查询本地可用性时必须同时看清理回执，不能把历史清单理解为当前仍有完整 bag。摘要分别标明单次失败与关联问题族的后续修复/回归，不能据此将原失败实验记为成功。

清理索引、精确计划、删除回执、分类证据及核验结果保存在：
`/media/zhuyupei/SanStand/实验记录/Robot-Controller/maintenance/user_failed_cleanup_20261004T065532Z/`。
112 份摘要、1437 个保留源文件的 SHA256 和受保护录制均已核验；清理脚本的 12 项隔离测试通过。

该授权仅适用于本次限定清单，不启用后续自动删除或上传：`upload_enabled=false`、`delete_enabled=false`、`archive_policy=local_only` 及维护控制文件保持原样。本次清理不发送机械臂命令，也不停止 ROS 节点。

查询先在外盘 `ros_log/`、`grasp_records/` 和 `文档/` 查找。之前已经上传的 GitHub 历史内容及恢复索引保持原样；只有明确需要历史复核时才使用查询/恢复入口，不自动补传、下载或删除云端内容。

## 2026-10-07 用户授权的约 60G 单次清理

用户原话：“你先进行60个G的删除任务，这个任务就是和之前要求你删除50个G空间占用的要求一样”；随后要求“这个删除要求要记录到文档中”，并要求“清理完成后，继续处理没有完成的问题”。

沿用前次实际执行口径：以 SanStand **可用空间达到约 60 GiB** 为停止条件，按失败实验发生时间从早到晚清理。当前可用约 2.2 GiB，预计释放约 58 GiB；实际释放量、最后剩余量和停止条件均写入回执。单位口径沿用前次 GiB，不把目标理解成长期自动维持的最低空间。

清理仅限有结果/日志证据明确失败且已经关闭的实验中的大体积原始 bag、图像序列、视频和数据输入。每个目录先保存问题发现、解决思路和方法、验证结果的简要记录，再删除精确清单内的大文件；原结果、授权、文本日志、审计、恢复元数据及历史清单保留。历史 manifest 记载的已删原始文件须结合本次清理回执解释。

实物成功（包括闭爪成功后提升报错的实验）、成功、未知结果、标定、打开/仍写入的文件，以及本轮尚需用于定位断帧/支撑缺测的证据排除。不会仅因目录名含“失败”或自动程序非零退出就删除。达到目标后立即停止，并核对保留摘要/验证文件和自动存储策略。

本次是用户明确授权的一次性例外；`local_only`、`upload_enabled=false`、`delete_enabled=false` 保持。执行计划、分类证据、删除回执、摘要索引及核验落在外盘 `maintenance/user_failed_cleanup_*`。清理结束后恢复未完成的卡顿对照与抓取问题处理，不自动复用失效目标或旧轨迹。

本次已完成：按时间顺序清理 2026-09-28 03:20 至 2026-10-06 08:18 UTC 的 50 份明确失败且封口的录制，仅删除 50 个 bag，分配空间合计 **58.778 GiB**；开始可用 2.187 GiB，达到 **60.953 GiB** 后停止。并发日志继续写入，故可用空间净增量与删除文件分配量略有差异。本次无需删除任何图像或分析输入。

精确计划、独立分类/成功冲突复核、删除回执、摘要索引和核验保存在：
`/media/zhuyupei/SanStand/实验记录/Robot-Controller/maintenance/user_failed_cleanup_20261007T134426Z/`。
50 份摘要和 896 个保留文件的 SHA256、16 个受保护原始文件的身份及 3 份存储策略 SHA256 已核验一致，清理执行器 19 项 `/tmp` 隔离测试通过。用户确认实际夹住但提升失败的录制、结果未知、标定和本轮定位证据未删除。此任务已结束，不自动再次补足 60 GiB。

## 2026-10-12 UTC 两份失败录制的新单次授权清理

本次抓取录制在创建新实验、选择新目标和发出运动前，因外盘可用空间低于既有录制余量而被拒绝。用户另行明确回复“允许，仅清理这两份失败录制后继续抓取”，授权回执为 `call_aaa53eb3ec4c42c0b6425dd35736be4b:0`。这是新的精确清单授权，不复用 2026-10-07 已结束的单次授权，也不设持续补足空间的任务。

2026-10-12 03:18:47 UTC 执行完成，仅删除以下两份已关闭、明确在执行前的规划阶段失败的实验中的 `keyframes.bag`：

| `grasp_records/` 下的实验目录 | 删除字节数 | 删除前实际 SHA256 |
|---|---:|---|
| `20261006T085709.051090Z_bc8e72c457f542a18a60e9e94fb6b3e1` | 1,705,533,291 | `c61bdde09cd0f31f82712599d14954b58fb5217489c1a61128afb8f12119debd` |
| `20261006T093000.069892Z_2b67317c7fb54de5a1fd94d6b4f7bb49` | 1,497,013,570 | `20aa9bbf4d09fe4fd602b2da3a88c5bf3ec268e17603bf18781465e1c9964a5b` |

合计 **3,202,546,861 字节（2.982604 GiB）**。执行回执记录可用空间从 4,475,584,512 增至 7,677,018,112 字节（约 4.168 → 7.150 GiB）；并发日志及新摘要的写入使净空间变化与原始文件大小不完全相等。本次限定清单已经耗尽，不继续清理其他记录。

两份实验分别因 `source_frame_stale_or_future` 和 `exact_rgbd_stream_stale` 结束，原记录为 `complete/failure`、`recorder_closed=true`，录制器 `bag_closed=true`、`active_seen=false`；原文档明确未运动、闭爪或提升。本次未处理更早涉及闭爪后提升失败、成功、未知或标定的录制，也未处理当前接触高度、闭爪契约定位需要的记录。两文件先全部读取并验证 SHA256 与原 manifest 一致，再分别保存 `问题方法与验证摘要_20261012.md` 后执行精确删除。摘要保留问题、方法、验证结果和未根治项，不将失败写成成功。

原 manifest、result_summary、recorder_outcome、command_result、授权、全部文本日志和关键帧审计均保留；执行回执 `all_retained_evidence_hashes_unchanged=true`。原 manifest 未改写，仍登记历史 bag，查当前文件可用性必须结合删除回执。执行前可访问的宿主文件描述符未发现打开者；其他用户受保护进程的描述符不可读取，这一检查范围与录制器已经关闭的独立证据均在回执中记录。

精确候选清单、新授权、每实验摘要及执行回执位于：
`/media/zhuyupei/SanStand/实验记录/Robot-Controller/maintenance/user_failed_cleanup_proposed_20261012T0310Z/`。
其中 `execution_receipt.json` 的状态为 `complete`，SHA256 为 **`efb895fdd1ed73518dcce0605b65e4d17dbd0c932d42b0526f4c0b404f0cc2aa`**。前期 `proposal.json` 仅保留提案时状态，最终授权与执行结果以执行回执为准。

`local_only`、`upload_enabled=false`、`delete_enabled=false` 及存储策略文件 SHA256 保持不变。未启动归档 worker 或上传 finisher，未上传或改变云端历史，未发送机械臂命令。后续抓取使用新录制及当前实测状态，不复用被拒绝尝试的执行资格。

| 内容 | 外盘位置 |
|---|---|
| 正式 canonical 抓取记录 | `grasp_records/` |
| 当前未知物体会话及正式记录 | `ros_log/unknown_grasp_20260923/` |
| 未指定 ROS_LOG_DIR 的默认日志 | `ros_default_log/`（`~/.ros/log` 已指向此处） |
| ROS 会话日志 | `ros_log/ros_live_20260922_session/` |
| 连续终端监督 | `ros_log/unknown_grasp_20260923/supervision_continuous.log` |
| 归档清单与下载位置 | 对应记录目录中的 `manifest.json`、`result_summary.json`、`archive_manifest.json` |
| 全量盘点索引 | `ros_log/unknown_grasp_20260923/record_inventory_20260927.json`（盘点时快照，当前状态以各记录 manifest 为准） |
| 文档快照 | 外盘 `文档/`；源码文档原件仍在项目中 |

项目源码、构建产物及 operational `.sh`、`.py`、`.launch` 原件继续保留在 `/home/zhuyupei/alicia_wa_full`，尤其两个会话目录内的启动入口。`ROS_LOG_DIR` 只表示日志落点，不再用它查找 launch 或 Python 源码。不要把整个项目根替换成外盘路径，也不要将运行中的源码目录整体移动。

## 后续启动和正式录制

项目内 `.ros_log/unknown_grasp_20260923/sanstand_storage.sh` 是当前可重复使用启动入口的共同存储配置。它先确认 SanStand 为已挂载目录，再建立并检查外盘输出目录。未挂载或不可写时输出 `EXPERIMENT_STORAGE_UNAVAILABLE` 并退出，不改写到系统盘。这只是存储路径可用性检查，不发送任何机械臂命令。

`start_ros.sh` 的 ROS 日志和 stdout 写入外盘 ROS 会话目录，launch 原件仍取项目内 `resume_modes.launch`。`start_watch_current.sh` 的监督日志直接写外盘。`start_close_hold_attempt.sh` 将原有 420 秒默认录制及同一抓取命令输出到外盘 `ros_log/unknown_grasp_20260923`；策略、质量门限、运动参数和命令保持原值。`start_recorded_aligned.sh` 使用外盘 canonical `grasp_records`，原 600 秒设置不变。授权证据依旧通过录制器传入的 `GRASP_RECORD_DIR/authorization.json` 写入同一外盘实验目录。查授权时应遍历这两个外盘记录根，不能继续只查旧系统盘目录。

审计入口 `start_remote_with_audit.sh` 继续执行项目内 Python 原件，仿真 request/response 和候选路径审计使用外盘 unknown 会话目录。本轮已经运行的节点不会因编辑脚本而重载；修改在下次正常启动相应入口时生效。历史单次实验脚本与旧离线回放脚本作为实验资料保留，不应当作后续启动入口。

用户于 2026-10-04 明确要求“以后不要管机械臂温度的情况了，不要查询也不要对其反应”。后续启动持续采用 `/alicia_d_driver_node/software_temperature_monitoring_enabled=false`、`temperature_poll_rate_hz=0.0`；驱动默认及 `alicia_d_driver.launch`、`alicia_d_bringup.launch` 均已固定关闭，现有 `resume_modes.launch` 经完整启动链继承。维护、诊断及实验流程不主动查询温度，也不以温度告警、暂停、降级或使能/失能。温度专用解析、发布与软件门控关闭；SDK 原始通信、硬件固有保护、通用故障及真实编码器运动确认保持原规则。

该策略由驱动在启动时读取，给旧实例修改参数不能当作已生效。2026-10-04 的策略维护仅替换驱动进程，退出路径不发送失能，正向使能请求不等于编码器已确认运动。实际进程、二进制、前后 control epoch 和反馈证据保存在外盘 `ros_log/full_ros_resume_20261004T002930Z/driver_temperature_off_20261004T084125Z/`；后续冷启动保留关闭策略，不因本条说明再次重启节点。

项目工作树的 `.runtime/experiment_storage.json` 保存本机映射（运行配置由部署步骤生成，不提交凭据）：

```json
{
  "mount_point": "/media/zhuyupei/SanStand",
  "archive_root": "/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records",
  "ros_log_root": "/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log"
}
```

`tools/grasp_archive.py` 和 `tools/run_recorded_grasp.py` 默认使用配置的 canonical 根；`GRASP_RECORD_ROOT` 优先，显式的其他根（例如独立 `/tmp` 测试目录）继续有效。旧 canonical 根和旧 `unknown_grasp_20260923` 根这两个明确别名自动映射到上述外盘落点，其他任意路径不做替换。涉及外盘的根均要求配置挂载点在线；不存在该本机配置时，原默认行为保持兼容。两个外盘 `archive_config.json` 的 `retention_roots` 必须同时列出两个外盘实际根。

## 本轮正在运行的日志

本轮活跃 ROS stdout 的文件描述符仍指向系统盘原文件，暂留内部并持续镜像到外盘，直到写入者自然关闭。不能删除、移动或截断仍打开的源文件，也不为了更换日志落点重启机械臂节点。镜像是当前日志快照，最终关闭后核对字节数和 SHA256，再按部署流程完成最终迁移。新启动的监督进程和未来会话直接使用外盘。

## 查找和恢复更早实验

先查看外盘完整记录和索引。仅对以前已经上传、且需要独立查询或恢复的历史记录，使用 manifest 中的仓库、Release、附件顺序和 SHA256；历史 GitHub 目标为 `keniggg/Robot-Controller` 的 Releases。需要恢复时应取该实验完整的 `archive_manifest.json` 和登记附件，不能只取运行日志而遗漏 bag、关键帧审计或操作员观察。以下为按需恢复说明，不会自动执行下载。

在项目工作树执行恢复工具，目标选择外盘中一个尚不存在的目录：

```bash
python3 tools/grasp_archive.py restore "/外盘/对应实验/archive_manifest.json" "/media/zhuyupei/SanStand/实验记录/Robot-Controller/restored/RECORD_ID"
```

工具按附件顺序恢复并核对逐块、完整文件 SHA256。恢复副本放入独立目录，避免覆盖原始记录。除2026-10-12授权的失败记录空间清理外，不自动删除SanStand中的新旧实验；是否已经上传不改变本地保留要求。

历史 `legacy-grasp-*` Release 使用独立的历史恢复清单格式，应使用对应恢复入口：

```bash
python3 tools/archive_legacy_grasp_records.py \
  --restore-manifest "/外盘/历史实验/archive_manifest.json" \
  --restore-directory "/media/zhuyupei/SanStand/实验记录/Robot-Controller/restored/LEGACY_RECORD_ID"
```

该入口恢复归档的精确字节；已无损压缩的历史文件恢复为 `.gz`，其原文件 SHA256 和压缩审计仍保存在恢复清单中。它不删除 GitHub 或源文件，也不启动机械臂。

补充材料的 `supplemental-grasp-*` Release 内是 `historical_supplement.tar.gz` 与 `members.json`，需要两步恢复。先在对应 `recovery_index.json` 所在目录找到 `RECORD_ID.remote_manifest.json`，使用历史恢复入口从 GitHub 下载并校验这两个附件：

```bash
python3 tools/archive_legacy_grasp_records.py \
  --restore-manifest "/外盘/补充归档/RECORD_ID.remote_manifest.json" \
  --restore-directory "/media/zhuyupei/SanStand/实验记录/Robot-Controller/restored/SUPPLEMENTAL_RECORD_ID-assets"
```

再从这两个已验证附件离线恢复原始文件；工具逐文件验证大小和 SHA256，并按 `members.json` 中的相对路径创建文件：

```bash
python3 tools/prepare_supplemental_log_archive.py --restore \
  --bundle "/media/zhuyupei/SanStand/实验记录/Robot-Controller/restored/SUPPLEMENTAL_RECORD_ID-assets/historical_supplement.tar.gz" \
  --members "/media/zhuyupei/SanStand/实验记录/Robot-Controller/restored/SUPPLEMENTAL_RECORD_ID-assets/members.json" \
  --restore-directory "/media/zhuyupei/SanStand/实验记录/Robot-Controller/restored/SUPPLEMENTAL_RECORD_ID-files"
```

替换清单路径和示例记录 ID，两个目标目录均须尚不存在，SanStand 须已挂载。第二步不访问网络，也不上传或清理任何记录；恢复参数不能与准备、上传或清理参数混用。恢复出的 `.gz` 保留归档时的字节，不自动解压原日志。

## 持续维护服务

当前 `alicia-grasp-archive.service`、`alicia-grasp-archive-unknown.service` 和 `alicia-archive-finish-20260927.service` 已停止并禁用自动启动。两个外盘根及两个旧源根的 `archive_config.json` 均设置 `upload_enabled=false`、`delete_enabled=false`、`archive_policy=local_only`。`repo` 仅保留为已上传历史的查询信息，不代表上传授权。上传工具默认关闭上传；恢复/查询现有历史仍可用。

`alicia-active-log-mirror.service` 继续每 5 秒增量镜像本轮仍使用旧文件描述符的 ROS 日志，状态保存在外盘 `ros_log/.active_ros_log_mirror.json`。监督日志已经直接写外盘，不再从旧内部文件覆盖。`alicia-log-spool-cleanup.timer` 仅处理关闭后的系统盘重复副本：两端 SHA256 一致后移除内部副本，外盘原件保持完整，代码、配置及打开的源文件保留。

`system_disk_migration_cleanup.json` 保留逐文件原始身份、目的身份、大小和 SHA256，解释 exFAT 时间戳精度变化。已生成的历史 `legacy/`、`incomplete/` 索引和未完成补充打包文件继续保留作追溯材料；取消上传不删除这些外盘数据，也不把原上传队列显示为已经上传完成。当前状态见 `storage_status_20260927.json` 和 `maintenance/archive_finish_20260927/status.json`。

## 外盘晚挂载与后台暂停

`alicia-sanstand-recovery.timer` 每分钟检查外盘。`local_only` 策略下只恢复日志镜像，禁止启动两个归档 worker 和历史 finisher；已经运行的镜像进程保持原样。它不重启 ROS，不修改机械臂使能。挂载不可用时不写入空挂载目录，也不回退系统盘。

持久控制文件为外盘 `maintenance/storage_control.json`，当前内容含 `state=resumed`、`archive_policy=local_only`。恢复入口及存储服务的 `ExecCondition` 读取同一标记，区分本地镜像/系统盘重复副本迁移与 GitHub 上传。`paused`、标记缺失或损坏时不启动任务；普通暂停/恢复保留现有 `local_only` 策略。旧 `/tmp` 临时标记不作为持续上传授权。

收到明确的后台暂停指令时，在项目工作树执行：

```bash
python3 tools/recover_sanstand_archive_services.py --set-state paused --reason '用户明确暂停本地存储维护' &&
systemctl --user stop alicia-sanstand-recovery.timer alicia-log-spool-cleanup.timer alicia-active-log-mirror.service alicia-log-spool-cleanup.service
```

收到明确的继续本地维护指令后：

```bash
python3 tools/recover_sanstand_archive_services.py --set-state resumed --reason '用户明确继续本地存储维护' &&
systemctl --user start alicia-sanstand-recovery.timer alicia-log-spool-cleanup.timer &&
python3 tools/recover_sanstand_archive_services.py
```

这些命令不会启用已禁用的 GitHub 服务。设置标记要求 SanStand 已挂载；缺盘时应停止维护定时器和服务，挂载后再保存暂停状态，不把写入失败当作状态已持久保存。后续新增材料直接写入外盘；成功、未知、标定及尚未完成分类/封口复核的材料完整保留，已核失败材料仅依2026-10-12独立空间规则处理。不追加GitHub队列，不修改以前的不可变恢复索引。

## 本次最小化清理结果（2026-10-14 UTC）

执行完成并核验：无损压缩后移除 **33,845** 份分散文本原件；移除 **102** 份已封口大录制，完整非图像ROS消息及首尾视觉样本留在回溯库中。原件分配空间合计 **75.481 GiB**，其中录制 **61.949 GiB**、文本 **13.532 GiB**；压缩库占用约 **1.689 GiB**。核验时外盘可用 **82.202 GiB**，停止原因为 `target_reached`。

**4813** 份受保护文件的设备、inode、大小与修改时间全部一致；所有实际删除均有原路径、原SHA256及留存位置的持久回执，无未决删除意图。18项专项验证及独立复核通过，原结果保持真实，软件修复不能据此记为实物抓取成功。

精确依据：`maintenance/minimal_trace_cleanup_1791956657642651189/receipt.json`、`final_verification.json`、`final_result.json`；逐文件删除记录在同目录 `trace.sqlite3` 的 `deletions` 表。已迁移材料保持现状，没有新增GitHub联系。低空间定时器恢复，自动扩充名单关闭，低于10 GiB触发、80 GiB目标不变；通用上传及删除开关仍关闭。

另清理本轮三次已结束验证的115个合成临时文件，分配空间约14.375 MiB；测试源码、日志、XML和结果均保留，核验未变。逐文件回执为同目录 `synthetic_fixture_cleanup.json`，不计入上面的实验原件删除统计。
