# 2026.10.16 未完成补充视角观察代码：可浏览核心快照

此分支逐字节导出自此前已上传并读回核验的 Release 附件，**不是之后继续修复的 GEN31 工作区版本**。当时状态为未完成 GEN31、尚未部署、尚未通过完整回归；未声称补充视角抓取或实物夹持成功。

- 分支根 `src/`、`docs/`、`tools/`、`srv/` 等来自归档 `current_runtime_workspace/`，用于直接浏览当时实际运行工作区的核心代码和操作文档。
- `ide_root_workspace/` 保存同一归档里的旧 IDE 根工作区核心文本，仅供对照；它与主工作区不同，不能混用。
- `external_dependencies/` 保留归档内的三份手眼配置、服务和来源合同文本；未从当前机器补入任何新内容。
- `snapshot-export-manifest.json` 记录每个原归档路径到分支路径的映射、字节数和 SHA256；符号链接以原链接目标校验。原文件内容未改写。
- `src/**/tests/fixtures/` 中保留用于源码回归的文本测试夹具，其中包含从历史录制提取的最小样本；它们不是全部合成数据。原实验目录、原始录制和验证输出不在分支中。

本分支供 GitHub 直接读取核心源码。权重、大型网格、图片、NPZ/实验录制、构建/缓存目录、`.runtime` 历史备份和旧 deployment 重复副本不在此导出范围，因此**不是包含所有依赖和资产的可直接运行安装包**。原脚本内的绝对路径、外部 SDK/模型/标定引用保持历史原文。`src/CMakeLists.txt` 的原 catkin 符号链接仍指向 `/opt/ros/noetic/share/catkin/cmake/toplevel.cmake`。

完整原始快照及 `source-manifest.json`、SHA256SUMS、历史恢复说明仍在 [原 Release](https://github.com/keniggg/Robot-Controller/releases/tag/snapshot-20261016-unfinished-supplemental-view-code)。`snapshot_metadata/ORIGINAL_RELEASE_RESTORE.md` 原文描述的是该完整附件，不是此精简分支。它不代表当前软件已修复、已经部署或可自动启动机械臂。

## 原附件身份

- Archive SHA256: `d7f60fda328a34b60355a65b53e0dea1377c657d7fa278647f5dab13c7ac24ad`
- Source manifest SHA256: `55ec5e794bfc2776297cc394d412e9e9db2ff94583f568e21a672ac6d072b97c`
- Archive: `2026.10.16-unfinished-supplemental-view-code.tar.gz`
- Release title: `2026.10.16日未完成补充视角观察代码`

本分支创建不会修改现有分支或原 Release tag。所有软件/硬件验证结论以相应时期记录为准。
