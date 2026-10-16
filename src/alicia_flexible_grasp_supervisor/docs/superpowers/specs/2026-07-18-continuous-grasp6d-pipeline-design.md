# Continuous Grasp6D Pipeline Design

**Date:** 2026-07-18

**Scope:** WSL-side real-time GraspNet inference optimization and ROS-side
stable 6D candidate gating. This design deliberately avoids repository-wide
refactoring.

## Goal

Turn “生成 6D 候选” into a bounded-latency continuous inference mode, stabilize
GraspNet candidates across frames, apply expensive MoveIt planning only to a
small stable shortlist, and separate continuously changing preview output from
an immutable execution plan.

The implementation continues to use the official GraspNet baseline and the
existing checkpoint. It does not change the network architecture, training
flow, checkpoint contents, or introduce generated fallback grasps.

## Non-goals

This phase does not implement pre-grasp secondary estimation, final visual
servoing, electronic-skin slip compensation, dynamic-object MuJoCo modeling, a
ROS action-server migration, or changes to GraspNet model/training code. The
new state and data interfaces leave explicit extension points for those phases.

## Existing Flow

1. camera_node.py publishes aligned color and depth with one source timestamp.
2. perception_node.py consumes only the latest camera frame, runs detection or
   instance segmentation, and publishes /perception/object and
   /perception/object_mask with the source header.
3. remote_grasp6d_node.py collects exact-timestamp RGB, depth, object, mask, and
   joint-state samples. It fuses three stable samples and freezes target
   geometry, support plane, OBB, and snapshot TF.
4. remote_grasp6d_client.py compresses RGB-D into NPZ and calls HTTP /predict.
5. The deployed combined WSL service delegates /predict to
   GraspNetBaselineBackend. The model loads lazily once, but every prediction
   currently calls torch.cuda.empty_cache().
6. ROS converts OpenCV optical candidates to canonical camera/base frames,
   applies the GraspNet-to-tool transform, and restores the exact identity /
   tool-Rz(180 degree) parallel-jaw symmetry.
7. The selector runs analytical gripper checks, target/approach/view filters,
   and strict MoveIt reachability before choosing one candidate.
8. The result is immediately published as /grasp_6d/plan_enriched, which is
   also execution authority.
9. The GUI captures plan_id and calls /grasp/start.
10. grasp_task_node.py copies the requested plan, but later checkpoints still
    require the current cached authority to retain the same plan_id.

## Problems to Correct

- The GUI service performs one blocking request instead of entering continuous
  inference.
- _request_lock rejects concurrency but has no replaceable latest pending slot.
- Raising request_hz cannot guarantee bounded latency or prevent stale work.
- The HTTP envelope lacks mandatory request sequencing and timing correlation.
- Normal inference unconditionally clears the CUDA allocator cache.
- Selection operates on one GraspNet frame and may oscillate.
- Center distance, approach, visibility, and joint motion can independently
  discard otherwise viable candidates.
- Strict MoveIt may be attempted for every locally surviving candidate and
  both symmetry variants.
- Continuous publication on the existing rich-plan topic would replace or
  tombstone execution authority.
- Diagnostics do not expose one uniform stage funnel, rejection proportions,
  primary failure gate, and rolling latency percentile.

## Architecture

Use a two-phase candidate pipeline:

1. Local phase: validate requests, enforce physical/safety contracts, calculate
   soft features, track candidates across frames, and fuse stable tracks.
2. MoveIt phase: pre-rank stable tracks, expand both parallel-jaw wrist
   variants, run strict MoveIt for only the configured top N poses, add motion
   cost, and select a preview candidate.

Focused pure-Python components are called by the existing ROS nodes. This does
not introduce an action server or redesign unrelated packages.

## Latest-only Scheduling

Add a pure LatestOnlyInferenceCoordinator with these invariants:

- exactly one inference may be active;
- at most one snapshot may be pending;
- submitting while busy atomically replaces pending;
- no FIFO request queue exists;
- every accepted snapshot gets a monotonically increasing request_id;
- starting or stopping streaming increments a generation;
- completion is accepted only when request ID, generation, snapshot stamp, and
  target-instance epoch still match;
- a completion older than result_max_age_sec is dropped;
- after active work finishes, the worker takes the current pending snapshot.

The snapshot producer forms rolling stable windows and submits only when the
newest exact source timestamp is newer than the last submitted timestamp.
Windows may overlap; existing three-frame stability checks still apply.

/grasp_6d/request_plan keeps TriggerZero:

- trigger=true starts continuous inference and returns immediately;
- trigger=false stops it, clears pending, increments generation, and makes the
  active result ineligible when it returns.

The GUI button toggles between “生成 6D 候选” and “停止生成候选”. The existing
“停止抓取” remains exclusively responsible for robot execution stop.
auto_request=true starts streaming with the node; false waits for the GUI or
service command.

## WSL Protocol and GPU Lifetime

The /predict protocol changes from version 2 to 3 because correlation fields
become mandatory. Standalone and combined GraspNet+MuJoCo servers share it.

Requests add a positive integer request_id and finite positive
snapshot_stamp_sec. Responses echo both and add:

- server_receive_sec and server_send_sec;
- preprocess_ms, inference_ms, postprocess_ms, and server_total_ms;
- gpu_allocated_mb, gpu_reserved_mb, and gpu_peak_allocated_mb.

The client validates strict JSON and exact request correlation before decoding
candidates. A mismatch cannot enter tracking.

GraspNetBaselineBackend holds the official model, checkpoint, CUDA context,
baseline helpers, and collision-detector class for the service lifetime.
Normal requests use torch.inference_mode() and do not call
torch.cuda.empty_cache(). Cache clearing is allowed only in a logged CUDA OOM
recovery path after rejecting that request. The existing non-blocking backend
lock remains an additional server guard.

--warmup continues to load the model at startup. Shape-dependent allocator and
kernel caches populated by the first real payload remain available later.

## Target Instance Epoch

ObjectPose has no detector-provided instance ID, so the remote node owns a
monotonic target-instance epoch:

- it remains stable while label/model choice match and target motion remains
  within target_instance_association_threshold_m;
- target loss, model reload, label change, invalid jump, or stream restart
  increments it and clears tracking;
- candidates from different epochs never match.

This gives instance isolation without changing ObjectPose.msg.

## Candidate Tracking and Fusion

Each frame candidate carries target epoch, label, model choice, request ID,
source stamp, raw/symmetry lineage, grasp center, tool0 pose, approach vector,
candidate and required widths, model score, analytical geometry result, safety
margins, and soft-feature values.

Tracks use deterministic gated nearest-neighbor matching. A match requires:

- same target epoch;
- center distance within tracking_position_threshold_m;
- approach angle within tracking_approach_threshold_deg;
- width difference within tracking_width_threshold_m;
- symmetry-aware orientation distance within
  tracking_orientation_threshold_deg.

Symmetry-aware orientation distance is the smaller of the normal quaternion
angle and the angle after applying tool-Rz(180 degree) to the candidate. Only
one observation per request ID contributes to a track.

A track is stable when it appears in at least three distinct requests among the
latest five valid inference results.

Fusion is robust:

- position: confidence/freshness-weighted coordinate median;
- width: weighted median;
- orientation: symmetry-aware quaternion medoid followed by aligned normalized
  robust weighted averaging;
- score: median;
- safety margin: conservative lower quantile;
- source header and geometry: newest contributing snapshot.

The fused pose is rechecked against the newest snapshot's hard gates before
MoveIt. Fusion cannot bypass safety.

## Hard Gates

These remain hard failures:

- missing, non-finite, invalid, or out-of-domain GraspNet depth;
- invalid point, quaternion, rotation matrix, frame convention, or rigid TF;
- missing/lost target or mismatched target epoch;
- required physical opening outside (0, 0.050] m;
- gripper contract mismatch;
- actual finger, palm, non-contact OBB, support-plane, or swept-envelope
  collision;
- invalid target/support geometry that prevents a physical check;
- strict MoveIt collision, joint-limit, IK, or planning failure.

An absolute target-instance sanity distance remains a hard guard against a
candidate on another object. It is much larger than the normal soft target
distance.

## Soft Scoring

These contribute weighted costs instead of independent rejection:

- model confidence;
- target-cloud and target-center distance;
- approach preference;
- predicted camera centering and visibility margin;
- support clearance above the physical collision boundary;
- jaw-tilt preference;
- total joint path cost and maximum single-joint motion;
- non-colliding geometric margins;
- track hit ratio and temporal dispersion.

The old 40 mm target-center threshold, downward-approach threshold, predicted
view threshold, and maximum-joint-motion preference become soft knees.
Slightly crossing one knee cannot discard a candidate.

Local features produce pre_moveit_score. Strict MoveIt metrics are then added
to produce final_score. Every component and weight is audited.

## MoveIt Bound, Cooldown, and Hysteresis

Only stable tracks enter MoveIt:

1. revalidate hard gates on each fused track;
2. expand identity and tool-Rz(180 degree) variants;
3. sort by pre_moveit_score;
4. call /supervisor/check_pose_strict for at most moveit_top_n variants;
5. add joint-path metrics and choose the best reachable variant.

Reachability is reusable while track, pose, orientation, and target remain
within configured deltas. Replanning is allowed only for:

- position change beyond replan_position_delta_m;
- orientation change beyond replan_orientation_delta_deg;
- target drift beyond replan_target_drift_m;
- current-candidate absence for candidate_consecutive_invalidations results;
- explicit /grasp_6d/replan_execution.

replan_cooldown_sec prevents repeated planning. A challenger must improve by
selection_hysteresis_ratio before replacing a valid choice.

/grasp_6d/replan_execution uses TriggerZero. Only trigger=true is valid and
promotion is rejected while robot execution is active.

## Preview and Execution Isolation

No ROS message definition changes are needed. Authority is separated by topic:

- preview visualization: /grasp_6d/preview_plan;
- preview rich plan: /grasp_6d/preview_plan_enriched;
- execution visualization/compatibility: existing /grasp_6d/plan;
- execution authority: existing /grasp_6d/plan_enriched.

Every accepted stable result may update Preview. Preview is never physical
execution authority.

The first stable, hard-safe, MoveIt-reachable preview may be promoted while the
robot is idle and no usable execution plan exists. Promotion revalidates the
fused pose against latest geometry, builds one rich plan, computes one
content-bound plan_id, and publishes an immutable deep copy on execution topics.

Later previews do not mutate, tombstone, republish, or replace that execution
plan. A new execution plan is allowed only while idle and after a configured
invalidation or replan condition.

When execution begins, grasp_task_node.py freezes its copied execution plan.
Preview callbacks only update diagnostics. Execution checkpoints validate the
frozen plan's integrity, explicit stop state, and live safety; they do not
compare it to Preview.

Hard safety events such as target loss, invalid transform authority, or
detected collision may revoke execution and stop progress. They may not
silently promote another preview during execution.

## Diagnostics and Performance Logs

Publish bounded strict JSON on /grasp_6d/pipeline_metrics and through ROS logs.
Every completed or dropped request contains:

- request ID, generation, target epoch, snapshot stamp;
- pending replacement count and drop reason;
- ROS snapshot/build/encode/transport/decode timing;
- WSL preprocess/inference/postprocess/total timing;
- end-to-end and result-age milliseconds;
- rolling p50/p95 over performance_window_size;
- GPU allocated/reserved/peak MB;
- raw, NMS, collision, returned, locally valid, stable, MoveIt checked,
  reachable, preview, and promoted counts;
- rejection count and ratio by stable reason code;
- per-stage remaining counts;
- primary failure gate when nothing survives.

The existing atomic planning audit remains authoritative for candidate lineage.
The bounded telemetry topic cannot authorize execution.

## Initial Configuration

    grasp_6d:
      plan_validity_sec: 5.0
      remote:
        request_hz: 1.5
        result_max_age_sec: 1.2
        stability_window_size: 5
        stability_min_hits: 3
        tracking_position_threshold_m: 0.025
        tracking_orientation_threshold_deg: 25.0
        tracking_approach_threshold_deg: 20.0
        tracking_width_threshold_m: 0.008
        target_instance_association_threshold_m: 0.08
        target_absolute_sanity_distance_m: 0.15
        moveit_top_n: 5
        replan_position_delta_m: 0.012
        replan_orientation_delta_deg: 12.0
        replan_target_drift_m: 0.025
        replan_cooldown_sec: 1.0
        selection_hysteresis_ratio: 0.12
        candidate_consecutive_invalidations: 2
        performance_window_size: 100

After real camera and WSL measurements show acceptable P95 latency, GPU
reserved memory, expiration rate, replacement rate, and candidate stability,
request_hz may rise in steps to 2, 3, 4, and 5 Hz. Rising replacement or
expiration rates require reducing frequency, not adding timer triggers.

## Files

New focused modules:

- src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/latest_only_inference.py
- src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_stability.py
- src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_pipeline.py

Primary modifications:

- src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py
- src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/remote_grasp6d_client.py
- tools/graspnet_baseline_server.py
- tools/mujoco_digital_twin_server.py
- src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py
- src/alicia_flexible_grasp_supervisor/gui/widgets/grasp6d_control_widget.py
- src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml
- src/alicia_flexible_grasp_supervisor/docs/remote_grasp6d_wsl2.md

No build artifact, __pycache__, generated ROS file, checkpoint, or GraspNet
training file is modified.

## Test Strategy

Pure unit tests cover:

- latest-only pending replacement and no busy backlog;
- stop/generation/request/time-expired response drops;
- cross-frame matching, three-of-five stability, and 180-degree symmetry;
- robust position/orientation/width fusion;
- soft scoring and mandatory hard gates;
- MoveIt top-N call bounds, cooldown, and hysteresis.

Node and integration tests cover:

- streaming start/stop semantics;
- slow fake WSL inference with repeated snapshot replacement;
- strict protocol-3 correlation and timing fields;
- model load once and no normal-path CUDA cache clear;
- preview/execution topic isolation;
- preview updates during execution without plan_id replacement;
- explicit replan rejection during execution;
- promotion only while idle;
- complete stage counts, rejection ratios, and primary failure output.

Existing remote-node, client, GraspNet protocol, combined WSL protocol, GUI,
grasp-task, RGB-D snapshot, geometry, and MoveIt tests remain in regression.

## Acceptance and Hardware Validation

Offline completion requires passing unit/integration tests and a successful
catkin build. It does not establish real-world grasp success.

Real hardware acceptance still requires ROS camera/segmentation timing, the
actual WSL environment and synchronized clocks, RTX 4070 Laptop GPU P95 and
memory measurement, repeated stable-candidate observations, MoveIt and
physical-arm dry runs, confirmation that execution stays frozen while previews
continue, and staged 1.5-to-5 Hz trials with no hidden backlog.

No success-rate claim may be made until those measurements are recorded.
