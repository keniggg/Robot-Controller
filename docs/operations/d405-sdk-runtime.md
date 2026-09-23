# D405 相机使用隔离 SDK 2.56.5

当前设备在 640×480 下必须使用包含官方 VGA 内参计算修复的 SDK。工作树原 Python 3.8 环境中 pyrealsense2 2.55.1 的彩色焦距约 438.6；同设备 SDK 2.56.5 返回约 395.8。原始内参查询和回放证据见 `docs/verification/2026-09-23-d405-sdk-projection-repair.md`。

## 运行入口

`alicia_flexible_grasp_supervisor/launch/sensors.launch` 的相机节点使用 `tools/with_local_realsense_sdk.sh` 作为 launch-prefix。它只给该进程设置 PYTHONPATH/LD_LIBRARY_PATH，不替换系统 Python、全局 pip 包或其他 ROS 节点。

本地布局：

```
.runtime/realsense-2.56.5/
  pyrealsense2.cpython-38-x86_64-linux-gnu.so
  lib/librealsense2.so.2.56.5
  lib/librealsense2.so.2.56 -> librealsense2.so.2.56.5
  install_manifest.json
```

`.runtime/` 已忽略，不提交二进制到 Git。若目录缺失，相机启动会明确报告缺少依赖，不静默退回有已知问题的 SDK。其它独立相机启动方式也应使用此 wrapper，例如：

```
tools/with_local_realsense_sdk.sh rosrun alicia_flexible_grasp_supervisor camera_node.py __name:=supervisor_camera
```

完整 ROS 主服务已经运行时，不能重复启动第二个相机进程占用设备。现场切换只替换相机进程，不停止机械臂服务或关节使能。

## 构建来源

- 官方 core 包：`https://librealsense.intel.com/Debian/apt-repo/pool/focal/main/librealsense2_2.56.5-0~realsense.17053_amd64.deb`
- SHA-256：`3d7102572f70ab0221f1c6340ec3f1390e51683672105e092cc787fe87e668ee`
- 只用 `dpkg-deb -x` 解压，不执行 apt/dpkg 安装。
- Python 接口使用官方 v2.56.5 源码，为本机 Python 3.8 构建；独立 CMake 描述及依赖来源在 `tools/realsense_sdk_python/`。核心运算库使用官方二进制，避免在运行 ROS 的双核主机重新编译整套 SDK。
- 最终模块使用相对 RPATH `$ORIGIN` / `$ORIGIN/lib`，不得依赖构建时 `/tmp` 路径。安装清单记录实际产物哈希和版本。

## 内参来源与标定数据

`camera_node` 启动或重连时，在第一帧发布前，从实际活动流一次更新 `/camera` 的 K、depth_scale、intrinsics_source 和 runtime_profile。元数据包含 serial、firmware、SDK、color/native depth K、畸变、depth→color 外参、对齐与滤波状态。

旧 YAML 文件不再覆盖 SDK 的实际对齐投影。旧 RGB 重采样候选保持关闭，不能在升级 SDK 后叠加旧投影修正。深度单位保持设备读取值，不为改善轮廓而缩放深度。

原始标定文件和手眼外参保持不动。后续离线标定必须使用每次采集对应的真实投影；历史旧 SDK 对齐深度需要明确其原投影，不能直接按新 K 解释。当前 CameraInfo 配置项此前未被发布，原静态板检测入口仍可能使用旧文件，因此不能把那个旧入口未经调整地作为新版内参验证依据。

抓取录制的时长、频率、分辨率与话题均不因这次 SDK 修复而改变。更新相机软件不等于独立手眼验证或抓取已通过。
