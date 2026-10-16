# External storage migration: runtime dependency review

Scope: read-only process/socket and path inspection, plus three script path corrections. No ROS, WSL, hardware, service, or archive worker command was issued.

## Runtime startup

The current session entry points under `.ros_log/ros_live_20260922_session` and `.ros_log/unknown_grasp_20260923` source `sanstand_storage.sh`. They retain their workspace control scripts and launch YAML, and direct `ROS_LOG_DIR`, `GRASP_RECORD_ROOT`, and current grasp outputs to SanStand. The D405 baseline is `.worktrees/protocol-v3-upgrade/.runtime/d405_carton_depth_baseline.json`, outside the migrated log roots. The active session launch imports packages from the worktree, rather than from historical experiment JSON.

The inspected `start_current_target.py` and `aligned_handoff_recorded.py` do not read historical experimental JSON as a startup configuration. Historical runtime snapshots such as `gen7_runtime_config_20260926.json` are inputs to offline diagnostic scripts, not to the current ROS startup chain.

## WSL deployment dependencies

`wsl_transfer/serve_restricted.py` used to resolve all transfer assets next to the Python source. That fails after its `.transfer_token`, peer state, `.tar.gz`, and `.sha256` files move away while `.py` remains. Its data root now uses `ALICIA_EXPERIMENT_STORAGE_BASE` with default `/media/zhuyupei/SanStand/实验记录/Robot-Controller`, followed by `ros_log/unknown_grasp_20260923/wsl_transfer`. Authentication, source pinning, permitted routes, payload bytes, and deployment checksums were not changed.

`wsl_transfer/switch_verified_endpoint.py` also now reads its deployment manifest and writes its backup/result evidence in that external transfer root. Its preserved workspace launch-script path is unchanged. This script was not executed.

At inspection time, neither `/proc/net/tcp` nor `/proc/net/tcp6` had a listening socket on port 8766. The transfer server source has a 1,800-second lifetime. No transfer service was restarted.

The WSL unified launcher loads its model, checkpoint, source repository, and verified MuJoCo bundle from WSL-local paths under the WSL user's home directory. It does not read Ubuntu `.ros_log` after installation. The Ubuntu migration therefore changes the reinstall/transfer source location, rather than those WSL-local runtime dependencies. This is static dependency evidence; WSL services were not contacted.

The extracted deployment's `verify.py`, `run_simulation.py`, XML, meshes, manifest and replay fixtures form one versioned bundle. Do not edit its contents to fix workspace-relative replay paths: that would break the pinned deployment hashes. If the bundle is used locally, execute its unchanged external copy as a unit. The transferred tarball and the installed WSL bundle remain unchanged.

## Supervision and historical queries

`read_current_supervision.py` previously read the old sibling `supervision_continuous.log` and cursor. It now reads `ALICIA_UNKNOWN_LOG_DIR`, defaulting to the external unknown-grasp data root. Its parser and feedback freshness checks were not changed.

Historical diagnostic programs including `compare_gen17_gen11_position.py`, `compare_gen18_gen11_position.py`, `offline_pose_geometry_20260926.py`, and scripts beneath `gen11_height_diagnosis/scripts` still contain old experiment input paths. They are outside the startup chain. Replaying these historical runs requires external data paths (or restored GitHub artifacts after retention pruning); blindly rerunning the old hard-coded paths will fail. A future replay invocation should use explicit external input/output arguments or a storage-root setting rather than recreating data under the system disk.

Validation: all three changed Python files parsed successfully with `ast.parse`; their root expressions were evaluated in isolation and matched the expected external directories. No script body or service command was executed. No credentials are included in this report.
