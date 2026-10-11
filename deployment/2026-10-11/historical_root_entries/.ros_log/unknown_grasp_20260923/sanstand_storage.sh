#!/usr/bin/env bash
# Source from session entry points; this only selects writable storage paths.
alicia_require_experiment_storage() {
  local experiment_mount=/media/zhuyupei/SanStand
  if ! mountpoint -q -- "$experiment_mount"; then
    printf '%s\n' "EXPERIMENT_STORAGE_UNAVAILABLE: SanStand is not mounted: $experiment_mount" >&2
    return 1
  fi
  export ALICIA_EXPERIMENT_STORAGE_BASE="$experiment_mount/实验记录/Robot-Controller"
  export ALICIA_SESSION_LOG_DIR="$ALICIA_EXPERIMENT_STORAGE_BASE/ros_log/ros_live_20260922_session"
  export ALICIA_UNKNOWN_LOG_DIR="$ALICIA_EXPERIMENT_STORAGE_BASE/ros_log/unknown_grasp_20260923"
  export GRASP_RECORD_ROOT="$ALICIA_EXPERIMENT_STORAGE_BASE/grasp_records"
  if ! mkdir -p -- "$ALICIA_SESSION_LOG_DIR" "$ALICIA_UNKNOWN_LOG_DIR" "$GRASP_RECORD_ROOT"; then
    printf '%s\n' "EXPERIMENT_STORAGE_UNAVAILABLE: cannot create experiment storage directories" >&2
    return 1
  fi
  if [[ ! -w "$ALICIA_SESSION_LOG_DIR" || ! -w "$ALICIA_UNKNOWN_LOG_DIR" || ! -w "$GRASP_RECORD_ROOT" ]]; then
    printf '%s\n' "EXPERIMENT_STORAGE_UNAVAILABLE: experiment storage is not writable" >&2
    return 1
  fi
}
alicia_require_experiment_storage
