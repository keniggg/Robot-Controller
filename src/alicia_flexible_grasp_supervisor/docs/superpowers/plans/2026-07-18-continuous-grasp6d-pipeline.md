# Continuous Grasp6D Pipeline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a bounded-latency continuous GraspNet path with latest-only scheduling, cross-frame stable 6D candidates, soft preference scoring, bounded MoveIt checks, and immutable execution-plan authority.

**Architecture:** Keep the existing RGB-D snapshot, official GraspNet, analytical gripper, MoveIt, MuJoCo, and plan-integrity contracts. Add three focused pure-Python modules, split local candidate evaluation from bounded MoveIt evaluation, and isolate continuous Preview topics from existing Execution topics.

**Tech Stack:** ROS Noetic, Python 3.8, NumPy, PyTorch/CUDA in WSL2, official GraspNet baseline, MoveIt, PyQt5, unittest/pytest, catkin.

## Global Constraints

- Continue to use the official GraspNet baseline and the existing checkpoint.
- Do not change the network architecture, checkpoint, training flow, or generate fallback grasps.
- Do not implement pre-grasp secondary estimation, final visual servoing, electronic-skin slip compensation, or dynamic-object MuJoCo modeling.
- Keep one active WSL inference and at most one replaceable latest pending snapshot.
- Preview output is never physical execution authority.
- Execution begins from one immutable content-bound plan_id and remains frozen.
- Invalid depth, invalid transforms, target loss, physical width, collision, joint limits, and strict MoveIt failure remain hard gates.
- Normal WSL inference must not call torch.cuda.empty_cache().
- No build artifact, generated ROS file, __pycache__, model checkpoint, or unrelated package is modified.
- Hardware performance and grasp-success claims require later real ROS, WSL, RTX 4070 Laptop GPU, and robot testing.

---

### Task 1: Latest-only Coordinator and Rolling Snapshot Admission

**Files:**
- Create: src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/latest_only_inference.py
- Modify: src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/rgbd_snapshot.py
- Create: src/alicia_flexible_grasp_supervisor/tests/test_latest_only_inference.py
- Modify: src/alicia_flexible_grasp_supervisor/tests/test_rgbd_snapshot.py

**Interfaces:**
- Produces: InferenceTicket, SubmitDecision, CompletionDecision, LatestOnlyInferenceCoordinator.
- Produces: SynchronizedRgbdBuffer.wait_for_samples(..., newest_after_ns=0).
- Consumes later: remote_grasp6d_node.py uses tickets to run exactly one WSL request and replace pending input.

- [ ] **Step 1: Write failing coordinator tests**

Add tests with this public behavior:

    from alicia_flexible_grasp.vision.latest_only_inference import (
        LatestOnlyInferenceCoordinator,
    )

    def test_busy_submit_keeps_only_latest_pending_snapshot():
        clock = FakeClock(10.0)
        queue = LatestOnlyInferenceCoordinator(clock=clock)
        queue.start()
        first = queue.submit('frame-1', 9.8, target_epoch=7)
        second = queue.submit('frame-2', 9.9, target_epoch=7)
        third = queue.submit('frame-3', 10.0, target_epoch=7)

        assert first.ticket_to_start.payload == 'frame-1'
        assert second.ticket_to_start is None
        assert third.ticket_to_start is None
        assert third.replaced_request_id == second.pending_request_id
        assert queue.pending_count == 1

    def test_completion_starts_latest_pending_without_backlog():
        clock = FakeClock(10.0)
        queue = LatestOnlyInferenceCoordinator(clock=clock)
        queue.start()
        first = queue.submit('frame-1', 9.8, target_epoch=7)
        queue.submit('frame-2', 9.9, target_epoch=7)
        latest = queue.submit('frame-3', 10.0, target_epoch=7)

        done = queue.complete(first.ticket_to_start, now_sec=10.2)

        assert done.accepted is True
        assert done.next_ticket.request_id == latest.pending_request_id
        assert done.next_ticket.payload == 'frame-3'
        assert queue.pending_count == 0

    def test_stop_and_age_drop_active_completion():
        clock = FakeClock(10.0)
        queue = LatestOnlyInferenceCoordinator(
            result_max_age_sec=0.5,
            clock=clock,
        )
        queue.start()
        stopped = queue.submit('frame-1', 9.8, target_epoch=7).ticket_to_start
        queue.stop()
        assert queue.complete(stopped, now_sec=10.1).code == 'GENERATION_STALE'

        queue.start()
        old = queue.submit('frame-2', 9.0, target_epoch=8).ticket_to_start
        assert queue.complete(old, now_sec=10.0).code == 'RESULT_EXPIRED'

Run:

    PYTHONDONTWRITEBYTECODE=1 ./devel/env.sh python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_latest_only_inference.py -q

Expected: FAIL because latest_only_inference.py does not exist.

- [ ] **Step 2: Implement the minimal thread-safe coordinator**

Create frozen dataclasses with these fields:

    @dataclass(frozen=True)
    class InferenceTicket:
        request_id: int
        generation: int
        snapshot_stamp_sec: float
        target_epoch: int
        payload: object
        submitted_monotonic_sec: float

    @dataclass(frozen=True)
    class SubmitDecision:
        ticket_to_start: Optional[InferenceTicket]
        pending_request_id: Optional[int]
        replaced_request_id: Optional[int]

    @dataclass(frozen=True)
    class CompletionDecision:
        accepted: bool
        code: str
        next_ticket: Optional[InferenceTicket]
        result_age_sec: float

Implement LatestOnlyInferenceCoordinator with an RLock and:

    start() -> int
    stop() -> int
    submit(payload, snapshot_stamp_sec, target_epoch) -> SubmitDecision
    complete(ticket, now_sec=None, target_epoch=None) -> CompletionDecision
    reset_target_epoch(target_epoch) -> None

Reject non-finite/non-positive stamps. complete must reject unknown request IDs,
stopped generations, target-epoch mismatches, and expired source age. It must
promote only the single current pending ticket.

- [ ] **Step 3: Verify coordinator tests pass**

Run the command from Step 1.

Expected: all coordinator tests PASS.

- [ ] **Step 4: Write the failing rolling-window test**

Extend test_rgbd_snapshot.py:

    def test_wait_for_samples_requires_newest_stamp_after_previous_window():
        buffer = SynchronizedRgbdBuffer(
            source_clock_ns=lambda: 10_000_000_000,
            monotonic_clock=lambda: 10.0,
        )
        for stamp in (9.1, 9.2, 9.3):
            add_complete_sample(buffer, stamp)

        first = buffer.wait_for_samples(
            3, 0.01, True, 1.0, newest_after_ns=0,
        )
        repeated = buffer.wait_for_samples(
            3, 0.01, True, 1.0,
            newest_after_ns=first[-1].stamp_ns,
        )
        add_complete_sample(buffer, 9.4)
        advanced = buffer.wait_for_samples(
            3, 0.01, True, 1.0,
            newest_after_ns=first[-1].stamp_ns,
        )

        assert repeated == []
        assert [sample.stamp_sec for sample in advanced] == [9.2, 9.3, 9.4]

Run:

    PYTHONDONTWRITEBYTECODE=1 ./devel/env.sh python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_rgbd_snapshot.py -k newest_stamp -q

Expected: FAIL because newest_after_ns is not accepted.

- [ ] **Step 5: Implement rolling admission and run snapshot regression**

Add newest_after_ns=0 to wait_for_samples. A returned window must have
samples[-1].stamp_ns greater than that value, but earlier samples may overlap.
Do not change exact-timestamp component matching, inference-latency checks,
source-age checks, or mutation invalidation.

Run:

    PYTHONDONTWRITEBYTECODE=1 ./devel/env.sh python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_rgbd_snapshot.py src/alicia_flexible_grasp_supervisor/tests/test_latest_only_inference.py -q

Expected: PASS.

- [ ] **Step 6: Commit Task 1**

    git add src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/latest_only_inference.py src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/rgbd_snapshot.py src/alicia_flexible_grasp_supervisor/tests/test_latest_only_inference.py src/alicia_flexible_grasp_supervisor/tests/test_rgbd_snapshot.py
    git commit -m "feat: add latest-only grasp inference scheduling"

### Task 2: Protocol 3 Correlation, Timing, and Resident CUDA Cache

**Files:**
- Modify: src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/remote_grasp6d_client.py
- Modify: tools/graspnet_baseline_server.py
- Modify: tools/mujoco_digital_twin_server.py
- Modify: src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_client.py
- Modify: src/alicia_flexible_grasp_supervisor/tests/test_graspnet_baseline_server_protocol.py
- Modify: src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_server_protocol.py

**Interfaces:**
- Changes: GRASP6D_PROTOCOL_VERSION becomes 3 on client and both WSL servers.
- Produces: PredictionBatch(candidates, diagnostics, performance).
- Changes: RemoteGrasp6DClient.predict requires request_id and snapshot_stamp_sec.
- Produces: client.last_performance and exact request/stamp validation.

- [ ] **Step 1: Write failing protocol-correlation tests**

Add client tests:

    def test_predict_requires_exact_protocol3_request_correlation(self):
        client = RemoteGrasp6DClient('http://wsl')
        client._request_json = lambda _path, payload: protocol3_response(
            request_id=payload['request_id'],
            snapshot_stamp_sec=payload['snapshot_stamp_sec'],
        )
        candidates = client.predict(
            color(), depth(), intrinsics(),
            request_id=41,
            snapshot_stamp_sec=123.25,
        )
        assert candidates
        assert client.last_performance['server_total_ms'] >= 0.0

    @pytest.mark.parametrize(
        ('request_id', 'stamp'),
        [(42, 123.25), (41, 123.5)],
    )
    def test_predict_drops_mismatched_request_or_stamp(request_id, stamp):
        client = RemoteGrasp6DClient('http://wsl')
        client._request_json = lambda *_args: protocol3_response(
            request_id=request_id,
            snapshot_stamp_sec=stamp,
        )
        with pytest.raises(ValueError, match='correlation'):
            client.predict(
                color(), depth(), intrinsics(),
                request_id=41,
                snapshot_stamp_sec=123.25,
            )

Run:

    PYTHONDONTWRITEBYTECODE=1 ./devel/env.sh python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_client.py -q

Expected: FAIL because protocol version 3 and correlation fields are unsupported.

- [ ] **Step 2: Implement strict client protocol 3**

Update encode_rgbd_payload to require:

    request_id: int
    snapshot_stamp_sec: float

Reject bool request IDs, non-positive IDs, and non-finite/non-positive stamps.
validate_predict_protocol_envelope accepts expected_request_id and
expected_snapshot_stamp_sec, requires every performance field to be finite and
non-negative, and requires exact request ID plus a stamp difference no larger
than 1e-9 seconds. RemoteGrasp6DClient.predict clears both last_diagnostics and
last_performance before transport.

- [ ] **Step 3: Write failing resident-backend tests**

Add backend tests with fake torch/model objects:

    def test_backend_loads_once_and_normal_predict_never_empties_cuda_cache():
        backend, torch = loaded_fake_backend()
        backend.predict_batch(valid_payload(request_id=1))
        backend.predict_batch(valid_payload(request_id=2))
        assert backend.load_count == 1
        assert torch.cuda.empty_cache.call_count == 0

    def test_prediction_batch_has_correlated_timing_and_gpu_memory():
        backend, _torch = loaded_fake_backend()
        batch = backend.predict_batch(valid_payload(request_id=7))
        assert batch.request_id == 7
        assert batch.snapshot_stamp_sec > 0.0
        assert batch.performance['inference_ms'] >= 0.0
        assert batch.performance['gpu_reserved_mb'] >= 0.0

Run:

    PYTHONDONTWRITEBYTECODE=1 ./devel/env.sh python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_graspnet_baseline_server_protocol.py -q

Expected: FAIL because predict_batch does not exist and predict clears cache.

- [ ] **Step 4: Implement PredictionBatch and timed inference**

In tools/graspnet_baseline_server.py add:

    @dataclass(frozen=True)
    class PredictionBatch:
        request_id: int
        snapshot_stamp_sec: float
        candidates: tuple
        diagnostics: dict
        performance: dict

Implement predict_batch under the existing non-blocking lock. Measure server
receive/send wall time plus decode/model-input, CUDA inference, and
postprocessing with time.perf_counter().
Synchronize CUDA immediately before and after the inference timing boundary
when the device is CUDA. Use:

    with self.torch.inference_mode():
        end_points = self.net(batch_data)
        grasp_preds = self.pred_decode(end_points)

Snapshot memory through memory_allocated, memory_reserved, and
max_memory_allocated when available. Do not call _empty_cuda_cache in finally.
Keep predict as a compatibility wrapper returning list(predict_batch(...).candidates).

Update standalone HTTP handling to call predict_batch and build the complete
protocol-3 envelope from the returned immutable batch.

- [ ] **Step 5: Update combined WSL server and tests**

Add one helper in tools/mujoco_digital_twin_server.py:

    def _predict_success_response(batch, backend_name):
        return {
            'ok': True,
            'backend': backend_name,
            'protocol_version': 3,
            'candidate_fields': list(CANDIDATE_FIELDS),
            'request_id': batch.request_id,
            'snapshot_stamp_sec': batch.snapshot_stamp_sec,
            'candidates': list(batch.candidates),
            'diagnostics': copy.deepcopy(batch.diagnostics),
            **batch.performance,
        }

The failure response echoes validated request correlation when recoverable,
contains an empty candidate list, and contains finite zero timings. Update mock
backends to produce PredictionBatch. Extend unified HTTP tests to assert exact
request/stamp echo and timing fields.

Run:

    PYTHONDONTWRITEBYTECODE=1 ./devel/env.sh python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_client.py src/alicia_flexible_grasp_supervisor/tests/test_graspnet_baseline_server_protocol.py src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_server_protocol.py -q

Expected: PASS.

- [ ] **Step 6: Commit Task 2**

    git add src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/remote_grasp6d_client.py tools/graspnet_baseline_server.py tools/mujoco_digital_twin_server.py src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_client.py src/alicia_flexible_grasp_supervisor/tests/test_graspnet_baseline_server_protocol.py src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_server_protocol.py
    git commit -m "feat: correlate timed resident graspnet inference"

### Task 3: Cross-frame Candidate Tracking and Robust Fusion

**Files:**
- Create: src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_stability.py
- Create: src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_stability.py

**Interfaces:**
- Produces: TrackingConfig, CandidateObservation, StableCandidate, CandidateTracker.
- Consumes later: remote node feeds locally safe base-frame candidates; pipeline consumes stable fused output.

- [ ] **Step 1: Write failing matching and stability tests**

    def test_candidate_track_requires_three_distinct_hits_in_last_five_frames():
        tracker = CandidateTracker(TrackingConfig(window_size=5, min_hits=3))
        assert tracker.update(1, [observation(x=0.100)]) == []
        assert tracker.update(2, [observation(x=0.104)]) == []
        stable = tracker.update(3, [observation(x=0.099)])
        assert len(stable) == 1
        assert stable[0].hit_count == 3

    def test_same_request_cannot_count_two_candidates_as_two_hits():
        tracker = CandidateTracker(TrackingConfig(window_size=5, min_hits=3))
        tracker.update(1, [observation(x=0.1), observation(x=0.101)])
        tracker.update(2, [observation(x=0.1)])
        assert tracker.stable_candidates() == []

    def test_parallel_jaw_rz180_is_same_physical_track():
        tracker = CandidateTracker(
            TrackingConfig(
                window_size=5,
                min_hits=2,
                orientation_threshold_deg=5.0,
            )
        )
        tracker.update(1, [observation(quaternion=identity_quaternion())])
        stable = tracker.update(2, [observation(quaternion=tool_rz180())])
        assert len(stable) == 1

    def test_target_epoch_position_approach_and_width_gate_matching():
        tracker = CandidateTracker(permissive_two_hit_config())
        tracker.update(1, [observation(target_epoch=3)])
        assert tracker.update(2, [observation(target_epoch=4)]) == []
        assert tracker.update(3, [observation(x=0.2)]) == []
        assert tracker.update(4, [observation(approach=(1, 0, 0))]) == []
        assert tracker.update(5, [observation(width_m=0.03)]) == []

Run:

    PYTHONDONTWRITEBYTECODE=1 ./devel/env.sh python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_stability.py -q

Expected: FAIL because grasp6d_stability.py does not exist.

- [ ] **Step 2: Implement immutable tracking types and symmetry distance**

CandidateObservation fields are:

    request_id, snapshot_stamp_sec, target_epoch, target_label, model_choice,
    center_base_xyz, tool0_position_xyz, quaternion_xyzw,
    approach_base_xyz, required_open_width_m, model_width_m, model_score,
    geometry_margin_m, pre_moveit_score, payload

Defensively copy NumPy arrays and mark them read-only. Validate finite values,
unit-normalize quaternion and approach, and canonicalize quaternion sign.

Implement:

    quaternion_angle_rad(first, second)
    parallel_jaw_orientation_distance_rad(first, second)
    weighted_median(values, weights)

The orientation distance evaluates the direct candidate and candidate composed
with local tool-Rz(pi), returning the smaller angle and the aligned quaternion.

- [ ] **Step 3: Implement deterministic track assignment**

CandidateTracker keeps a deque of request IDs with length window_size and a
track dictionary. For each batch:

1. reject duplicate/non-increasing request IDs;
2. compute gated pair costs;
3. sort pairs by cost, track ID, candidate index;
4. greedily assign each track and observation at most once;
5. create tracks for unmatched observations;
6. evict observations outside the last window;
7. remove empty tracks.

Pair cost is the sum of normalized center, orientation, approach, and width
distances after every gate passes.

- [ ] **Step 4: Write failing robust-fusion tests**

    def test_fusion_uses_robust_position_width_and_orientation():
        tracker = CandidateTracker(TrackingConfig(window_size=5, min_hits=3))
        tracker.update(1, [observation(x=0.100, width_m=0.040)])
        tracker.update(2, [observation(x=0.101, width_m=0.041)])
        stable = tracker.update(3, [observation(x=0.140, width_m=0.049)])
        fused = stable[0]
        assert fused.tool0_position_xyz[0] < 0.110
        assert fused.required_open_width_m < 0.045
        assert quaternion_angle_rad(
            fused.quaternion_xyzw,
            identity_quaternion(),
        ) < math.radians(2.0)

Run the Task 3 test file.

Expected: FAIL until stable fusion is implemented.

- [ ] **Step 5: Implement fusion and verify**

Use freshness/model-score-derived positive weights capped so one observation
cannot exceed half the total. Use coordinate-wise weighted medians, weighted
width median, score median, and lower-quartile geometry margin. Select the
quaternion medoid minimizing total symmetry-aware angular distance, align all
quaternions to it, perform normalized weighted averaging, and retain the newest
observation as representative payload/source header.

Run:

    PYTHONDONTWRITEBYTECODE=1 ./devel/env.sh python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_stability.py -q

Expected: PASS.

- [ ] **Step 6: Commit Task 3**

    git add src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_stability.py src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_stability.py
    git commit -m "feat: stabilize grasp candidates across frames"

### Task 4: Hard Safety Policy, Soft Scoring, Stage Funnel, and MoveIt Top N

**Files:**
- Create: src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_pipeline.py
- Create: src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_pipeline.py
- Modify: src/alicia_flexible_grasp_supervisor/tests/test_gripper_geometry.py

**Interfaces:**
- Produces: SafetyGateInput, GateDecision, SoftCandidateFeatures, SoftScoreWeights, ScoredStableCandidate, CandidateStageFunnel.
- Produces: mandatory_safety_gate, soft_candidate_cost, bounded_moveit_select.
- Consumes later: remote node maps existing analytical geometry and ROS data into these pure types.

- [ ] **Step 1: Write failing hard-gate tests**

    @pytest.mark.parametrize(
        ('field', 'value', 'code'),
        [
            ('depth_valid', False, 'DEPTH_INVALID'),
            ('transform_valid', False, 'TRANSFORM_INVALID'),
            ('target_present', False, 'TARGET_LOST'),
            ('same_target_instance', False, 'TARGET_INSTANCE_MISMATCH'),
            ('required_open_width_m', 0.051, 'GRIPPER_TOO_NARROW'),
            ('collision_free', False, 'COLLISION'),
        ],
    )
    def test_mandatory_safety_conditions_remain_hard(field, value, code):
        gate = replace(valid_safety_input(), **{field: value})
        result = mandatory_safety_gate(gate)
        assert result.ok is False
        assert result.code == code

    def test_preference_thresholds_do_not_become_hard_gates():
        gate = valid_safety_input()
        result = mandatory_safety_gate(gate)
        assert result.ok is True

Run:

    PYTHONDONTWRITEBYTECODE=1 ./devel/env.sh python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_pipeline.py -q

Expected: FAIL because grasp6d_pipeline.py does not exist.

- [ ] **Step 2: Implement mandatory safety gate and stage funnel**

SafetyGateInput contains only physical/contract facts:

    depth_valid, transform_valid, target_present, same_target_instance,
    target_absolute_distance_m, target_absolute_limit_m,
    required_open_width_m, physical_open_width_m,
    geometry_valid, collision_free

GateDecision is immutable with ok, code, reason. CandidateStageFunnel records:

    input_count
    remaining_by_stage
    rejection_counts

record_rejection increments a stable code. record_stage stores remaining count.
to_dict emits counts, ratios against input_count, and primary_failure as the
highest rejection count with lexical code tie-break.

- [ ] **Step 3: Write failing soft-score tests**

    def test_approach_center_visibility_and_joint_motion_are_soft_costs():
        preferred = soft_features()
        degraded = replace(
            preferred,
            center_distance_m=0.060,
            downward_approach_cos=0.30,
            visibility_center_cost=0.80,
            joint_path_cost=2.0,
            joint_max_delta_rad=2.1,
        )
        assert soft_candidate_cost(degraded, weights()).total > (
            soft_candidate_cost(preferred, weights()).total
        )
        assert math.isfinite(soft_candidate_cost(degraded, weights()).total)

    def test_model_score_and_geometry_margin_improve_rank_without_bypass():
        low = soft_features(model_score=0.2, geometry_margin_m=0.003)
        high = soft_features(model_score=0.9, geometry_margin_m=0.010)
        assert soft_candidate_cost(high, weights()).total < (
            soft_candidate_cost(low, weights()).total
        )

Run the Task 4 test file.

Expected: FAIL until soft scoring is implemented.

- [ ] **Step 4: Implement normalized soft scoring**

SoftCandidateFeatures contains finite:

    model_score, cloud_distance_m, center_distance_m,
    downward_approach_cos, visibility_center_cost,
    support_margin_m, jaw_tilt_cos, geometry_margin_m,
    joint_path_cost, joint_max_delta_rad,
    stability_hit_ratio, position_dispersion_m, orientation_dispersion_rad

SoftScoreWeights holds explicit non-negative weights and knee values. Implement
piecewise-linear normalized penalties with no infinity return. Return a
SoftScore containing total and a component dictionary. Model score, geometry
margin, and stability are rewards represented as bounded negative costs.

- [ ] **Step 5: Write failing MoveIt bound test**

    def test_moveit_checks_only_top_n_stable_candidates():
        calls = []

        def check(candidate):
            calls.append(candidate.track_id)
            return MoveItResult(
                reachable=candidate.track_id == 3,
                joint_path_cost=float(candidate.track_id),
                joint_max_delta_rad=0.1,
                reason='',
            )

        candidates = [scored_candidate(track_id=i, score=float(i)) for i in range(20)]
        result = bounded_moveit_select(candidates, check, top_n=5)

        assert calls == [0, 1, 2, 3, 4]
        assert result.selected.track_id == 3
        assert result.funnel.remaining_by_stage['moveit_checked'] == 5

Run the Task 4 test file.

Expected: FAIL until bounded_moveit_select exists.

- [ ] **Step 6: Implement bounded MoveIt selection and verify geometry regression**

Sort by pre_moveit_score then track_id. Clamp top_n to 3 through 10. Call the
provided checker only for the slice. Strict failure is recorded but does not
stop checking the remaining shortlist. Add motion soft costs to reachable
candidates and select minimum final cost with deterministic track/variant
tie-break.

Run:

    PYTHONDONTWRITEBYTECODE=1 ./devel/env.sh python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_pipeline.py src/alicia_flexible_grasp_supervisor/tests/test_gripper_geometry.py -q

Expected: PASS.

- [ ] **Step 7: Commit Task 4**

    git add src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_pipeline.py src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_pipeline.py src/alicia_flexible_grasp_supervisor/tests/test_gripper_geometry.py
    git commit -m "feat: add two-stage grasp candidate scoring"

### Task 5: Continuous Remote Node Orchestration and Operational Metrics

**Files:**
- Modify: src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py
- Modify: src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py
- Create: src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py

**Interfaces:**
- Consumes: LatestOnlyInferenceCoordinator, CandidateTracker, mandatory_safety_gate, soft_candidate_cost, bounded_moveit_select.
- Produces: /grasp_6d/preview_plan, /grasp_6d/preview_plan_enriched, /grasp_6d/pipeline_metrics.
- Changes: /grasp_6d/request_plan true starts and false stops streaming immediately.

- [ ] **Step 1: Write failing service and slow-client integration tests**

Construct an offline node fixture with fake frames, publishers, clock, and a
blocking fake client:

    def test_request_service_starts_and_stops_without_waiting_for_inference():
        node = streaming_node()
        started = node.request_plan_cb(trigger_request(True))
        assert started.success is True
        assert node.streaming_enabled is True
        stopped = node.request_plan_cb(trigger_request(False))
        assert stopped.success is True
        assert node.streaming_enabled is False

    def test_busy_inference_replaces_pending_and_drops_stopped_result():
        client = BlockingClient()
        node = streaming_node(client=client)
        node.start_streaming()
        node.submit_snapshot(snapshot(1.0))
        client.wait_until_entered()
        node.submit_snapshot(snapshot(2.0))
        node.submit_snapshot(snapshot(3.0))
        assert node.inference_coordinator.pending_count == 1
        node.stop_streaming()
        client.release()
        node.join_worker()
        assert node.tracker.update_count == 0
        assert node.metrics[-1]['drop_reason'] == 'GENERATION_STALE'

Run:

    PYTHONDONTWRITEBYTECODE=1 ./devel/env.sh python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py -q

Expected: FAIL because continuous orchestration does not exist.

- [ ] **Step 2: Add node streaming state without changing candidate semantics**

In __init__ add:

- preview PoseArray and Grasp6DPlan publishers;
- strict-JSON metrics String publisher;
- coordinator, tracker, condition, worker thread, stop event;
- streaming_enabled, last_submitted_stamp_ns, target_instance_epoch;
- rolling latency deque sized by performance_window_size;
- /grasp/state subscriber for execution-active status.

request_plan_cb only starts/stops and returns. spin calls a non-blocking
_poll_stream_snapshot at request_hz. The worker waits on a condition and runs
only coordinator tickets. rospy shutdown stops and joins the worker with a
bounded timeout.

- [ ] **Step 3: Extract request preparation and prediction from publication**

Introduce:

    @dataclass(frozen=True)
    class PreparedPrediction:
        ticket: InferenceTicket
        snapshot: SnapshotResult
        stamp: object
        geometry: object
        pose_estimator: FrozenSnapshotCandidatePoseEstimator
        graspnet_input: object
        candidates: tuple
        remote_diagnostics: dict
        remote_performance: dict

Refactor _process_frame into:

    _prepare_and_predict(ticket) -> PreparedPrediction
    _accept_prediction(prepared) -> bool

Keep _process_frame as a synchronous compatibility wrapper for existing unit
tests. It must no longer publish PLAN_PENDING to execution topics.

Pass ticket.request_id and snapshot.stamp_sec to client.predict. After return,
call coordinator.complete before any tracker, audit, MoveIt, or publisher
mutation. Dropped completion writes telemetry and exits.

- [ ] **Step 4: Write failing local-soft-gate and tracker integration tests**

Add tests showing:

- far-but-sane target center is retained with a higher soft cost;
- low downward cosine is retained with a higher soft cost;
- predicted off-center view is retained with a higher soft cost;
- non-finite depth, target loss, >50 mm physical opening, analytical collision,
  and invalid frozen TF remain rejected;
- one and two observations publish no preview;
- the third matching observation publishes one preview;
- MoveIt fake service receives no more than moveit_top_n calls.

Run only those named tests. Expected: FAIL on the old single-frame hard gates.

- [ ] **Step 5: Integrate local evaluation, tracking, and bounded MoveIt**

Split existing _candidate_matches_target into:

    _candidate_safety_gate(...)
    _candidate_soft_features(...)

Keep current depth/contract checks and CandidateGateResult failures hard.
Change normal target/cloud distance, downward approach, predicted visibility,
jaw tilt, and joint motion thresholds to SoftCandidateFeatures. Apply only the
absolute target sanity distance as a target hard gate.

Build CandidateObservation after camera/base conversion and analytical geometry
success. Feed the complete locally safe batch to CandidateTracker. Revalidate
each fused track against the newest geometry, expand both symmetry variants,
and call bounded_moveit_select.

Preserve raw candidate_index and variant_index in the existing full audit.
Add track_id, track hit/window counts, score components, and stage funnel.

- [ ] **Step 6: Implement metrics and primary failure reporting**

Build one strict-JSON dictionary per request:

    event, request_id, generation, target_epoch, snapshot_stamp_sec,
    status, drop_reason, pending_replacements,
    ros_prepare_ms, transport_ms, decode_ms,
    wsl_preprocess_ms, wsl_inference_ms, wsl_postprocess_ms,
    wsl_total_ms, end_to_end_ms, result_age_ms,
    latency_p50_ms, latency_p95_ms,
    gpu_allocated_mb, gpu_reserved_mb, gpu_peak_allocated_mb,
    stage_counts, rejection_counts, rejection_ratios, primary_failure

Use json.dumps(..., allow_nan=False, sort_keys=True, separators=(',', ':')).
Bound published text to the existing operational topic limit while retaining
full data in the atomic audit. If every candidate fails, status includes the
primary failure code and count, never only “无可用候选”.

- [ ] **Step 7: Run remote-node regression**

Run:

    PYTHONDONTWRITEBYTECODE=1 ./devel/env.sh python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py src/alicia_flexible_grasp_supervisor/tests/test_latest_only_inference.py src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_stability.py src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_pipeline.py -q

Expected: PASS.

- [ ] **Step 8: Commit Task 5**

    git add src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py
    git commit -m "feat: stream stable grasp previews without backlog"

### Task 6: Preview/Execution Promotion, Cooldown, and Hysteresis

**Files:**
- Modify: src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_pipeline.py
- Modify: src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py
- Modify: src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_pipeline.py
- Modify: src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py

**Interfaces:**
- Produces: ExecutionPlanController and PromotionDecision.
- Produces: /grasp_6d/replan_execution TriggerZero service.
- Maintains: existing /grasp_6d/plan and /grasp_6d/plan_enriched as sole execution authority.

- [ ] **Step 1: Write failing pure lifecycle tests**

    def test_preview_updates_do_not_replace_execution_plan():
        state = ExecutionPlanController(replan_cooldown_sec=1.0)
        first = state.observe_preview(preview('A'), now_sec=1.0, robot_active=False)
        assert first.promote is True
        state.commit_execution('plan-A', preview('A'), now_sec=1.0)
        second = state.observe_preview(preview('B'), now_sec=2.0, robot_active=False)
        assert second.promote is False
        assert state.execution_plan_id == 'plan-A'

    def test_execution_active_freezes_plan_against_new_preview():
        state = controller_with_execution('plan-A')
        decision = state.observe_preview(
            preview('B'), now_sec=2.0, robot_active=True,
        )
        assert decision.promote is False
        assert decision.code == 'EXECUTION_FROZEN'
        assert state.execution_plan_id == 'plan-A'

    def test_consecutive_invalid_cooldown_and_hysteresis_control_replan():
        state = controller_with_execution('plan-A', score=1.0)
        assert not state.observe_invalid(now_sec=1.1).invalidate
        assert state.observe_invalid(now_sec=1.2).invalidate
        assert not state.observe_preview(
            preview('B', score=0.95), now_sec=1.5, robot_active=False,
        ).promote
        assert state.observe_preview(
            preview('B', score=0.70), now_sec=2.3, robot_active=False,
        ).promote

Run Task 6 tests. Expected: FAIL because lifecycle controller does not exist.

- [ ] **Step 2: Implement the lifecycle controller**

ExecutionPlanController stores:

    execution_plan_id, execution_signature, execution_score,
    last_promotion_sec, invalid_streak, explicit_replan_requested

It never stores or mutates ROS messages. observe_preview returns a frozen
PromotionDecision with promote, code, reason. Promotion requires robot idle,
no valid current execution or a confirmed invalidation/explicit replan,
cooldown elapsed, and hysteresis satisfied. observe_invalid increments the
streak. request_replan is rejected while active.

- [ ] **Step 3: Write failing node topic-isolation tests**

    def test_preview_publication_does_not_touch_execution_publishers():
        node = promotion_node()
        node.publish_preview(plan('preview-A'))
        node.publish_preview(plan('preview-B'))
        assert ids(node.preview_rich_pub) == ['preview-A', 'preview-B']
        assert ids(node.rich_plan_pub) == []

    def test_execution_plan_is_published_once_and_frozen_while_active():
        node = promotion_node()
        node.promote_preview(plan('exec-A'))
        node.robot_execution_active = True
        node.handle_stable_preview(plan('preview-B'))
        assert ids(node.rich_plan_pub) == ['exec-A']
        assert node.latest_rich_plan.plan_id == 'exec-A'

Run named tests. Expected: FAIL on current shared publication path.

- [ ] **Step 4: Integrate isolated publication and explicit replan**

Add:

    _publish_preview(plan)
    _maybe_promote_preview(plan, signature, score)
    replan_execution_cb(req)

_publish_preview deep-copies to preview topics only. Promotion uses the existing
atomic execution legacy/rich publication helper and commits controller state
only after publication succeeds. A preview failure does not call
_publish_invalid_plan_pair. Hard safety invalidation may still tombstone
execution authority.

Subscribe to /grasp/state and set robot_execution_active from GraspState.active.

- [ ] **Step 5: Verify lifecycle and remote streaming tests**

Run:

    PYTHONDONTWRITEBYTECODE=1 ./devel/env.sh python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_pipeline.py src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py -q

Expected: PASS.

- [ ] **Step 6: Commit Task 6**

    git add src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_pipeline.py src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_pipeline.py src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py
    git commit -m "feat: isolate preview and execution grasp plans"

### Task 7: Freeze the Bound Execution Plan in GraspTaskNode

**Files:**
- Modify: src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py
- Modify: src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py

**Interfaces:**
- Keeps: /grasp/start requires exact execution plan_id.
- Changes: active execution validates a frozen bound copy rather than the latest preview/current replacement.
- Keeps: hard safety revocation and explicit stop cancel execution.

- [ ] **Step 1: Write failing frozen-execution tests**

    def test_active_execution_ignores_new_valid_plan_replacement():
        node = task_node_with_plan(plan('plan-A'))
        bound = node._copy_requested_grasp6d_plan('plan-A')[1]
        node.active = True
        node._freeze_execution_plan(bound)
        node.grasp6d_plan_cb(plan('plan-B'))

        result = node._validate_bound_plan(bound, grasp_config())

        assert result.ok is True
        assert node._bound_execution_plan.plan_id == 'plan-A'

    def test_preview_callback_cannot_change_execution_authority():
        node = task_node_with_plan(plan('plan-A'))
        node.grasp6d_preview_plan_cb(plan('preview-B'))
        assert node.latest_grasp6d_plan.plan_id == 'plan-A'

    def test_hard_tombstone_and_stop_still_revoke_frozen_execution():
        node = task_node_with_plan(plan('plan-A'))
        node.active = True
        node._freeze_execution_plan(node.latest_grasp6d_plan)
        node.grasp6d_plan_cb(invalid_plan('TARGET_LOST'))
        assert node._validate_bound_plan(
            node._bound_execution_plan, grasp_config(),
        ).code == 'EXECUTION_AUTHORITY_REVOKED'

Run:

    PYTHONDONTWRITEBYTECODE=1 ./devel/env.sh python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py -k "frozen or preview_callback or tombstone" -q

Expected: FAIL because active validation depends on latest_grasp6d_plan.

- [ ] **Step 2: Implement frozen execution authority**

Add:

    self.latest_grasp6d_preview_plan = None
    self._bound_execution_plan = None
    self._execution_authority_revoked = False

Subscribe preview rich topic to grasp6d_preview_plan_cb for diagnostics only.
At successful start binding, _freeze_execution_plan deep-copies the validated
plan and clears revocation. In start_cb finally, clear the bound plan.

While active:

- a new valid execution plan is rejected/logged as EXECUTION_FROZEN and does
  not replace latest or bound authority;
- an invalid execution tombstone sets _execution_authority_revoked;
- object loss/unsafe target state also sets revocation;
- preview callbacks never mutate either execution object.

_validate_bound_plan_locked validates the supplied plan against the immutable
bound copy and integrity digest. It does not compare against Preview or a later
valid plan. It still requires active=true, no explicit stop, no revocation, and
passing live target drift/safety checks.

- [ ] **Step 3: Run complete grasp-task regression**

Run:

    PYTHONDONTWRITEBYTECODE=1 ./devel/env.sh python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py -q

Expected: PASS, including stop linearization, MuJoCo gate, plan integrity, and
strict cached execution tests.

- [ ] **Step 4: Commit Task 7**

    git add src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py
    git commit -m "fix: freeze bound grasp execution authority"

### Task 8: GUI Toggle, Configuration, Launch Compatibility, and Runbook

**Files:**
- Modify: src/alicia_flexible_grasp_supervisor/gui/widgets/grasp6d_control_widget.py
- Modify: src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_control_widget.py
- Modify: src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml
- Modify: src/alicia_flexible_grasp_supervisor/launch/grasp_system.launch
- Modify: src/alicia_flexible_grasp_supervisor/docs/remote_grasp6d_wsl2.md

**Interfaces:**
- GUI starts/stops continuous inference through existing /grasp_6d/request_plan.
- GUI displays Preview separately and enables execution only from Execution.
- Launch keeps the existing shared WSL URL for predict and MuJoCo.

- [ ] **Step 1: Write failing GUI state tests**

    def test_generate_button_toggles_streaming_without_using_stop_grasp():
        widget = offline_widget()
        widget._run_set_streaming(True)
        widget._run_set_streaming(False)
        assert widget.request_calls == [True, False]
        assert widget.stop_grasp_calls == []

    def test_preview_updates_display_but_never_enable_execution():
        widget = offline_widget()
        widget._update_preview_plan(valid_plan('preview-A'))
        assert widget.preview_plan_id == 'preview-A'
        assert widget.execute_btn.isEnabled() is False
        widget._update_plan(valid_plan('exec-A'))
        assert widget.execute_btn.isEnabled() is True

    def test_button_labels_expose_start_and_stop_candidate_generation():
        labels = grasp6d_button_labels()
        assert labels.request_plan == '生成 6D 候选'
        assert labels.stop_inference == '停止生成候选'
        assert labels.stop == '停止抓取'

Run:

    PYTHONDONTWRITEBYTECODE=1 ./devel/env.sh python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_control_widget.py -q

Expected: FAIL because streaming and preview GUI state do not exist.

- [ ] **Step 2: Implement GUI streaming and dual-plan display**

Add stop_inference to Grasp6DButtonLabels. Keep one request button and change
its text according to _streaming_enabled. The service worker sends true to
start and false to stop. Starting streaming must not clear the execution
readiness tracker or disable a valid frozen execution plan.

Subscribe /grasp_6d/preview_plan_enriched to a preview-only tracker. Summary and
chips show Preview stability/age and Execution readiness separately. execute
button depends only on the existing execution readiness tracker.

- [ ] **Step 3: Add validated initial parameters**

Add the exact design defaults to grasp_params.yaml:

    plan_validity_sec: 5.0
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

At node load and runtime refresh, reject non-finite values, enforce window 5 /
hits 3 for the production default contract, and clamp moveit_top_n to 3..10.
Do not weaken existing physical gripper settings.

grasp_system.launch does not add a second WSL endpoint. It may add explicit
topic parameters only where the GUI/node currently reads configurable topics.

- [ ] **Step 4: Rewrite the affected runbook sections**

Update remote_grasp6d_wsl2.md to include:

- protocol version 3 and synchronized clocks;
- start/stop continuous service calls;
- Preview and Execution topic meanings;
- initial 1.5 Hz operation and staged 2/3/4/5 Hz tuning;
- pipeline_metrics strict-JSON example and P95 interpretation;
- WSL resident model/cache behavior;
- rejection funnel and primary failure interpretation;
- execution freeze and explicit replan;
- commands to verify no request backlog;
- explicit statement that offline and hardware checks do not guarantee success.

- [ ] **Step 5: Verify GUI/config/docs**

Run:

    PYTHONDONTWRITEBYTECODE=1 ./devel/env.sh python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_control_widget.py src/alicia_flexible_grasp_supervisor/tests/test_gui_ros_lifecycle.py -q
    python3 -m py_compile src/alicia_flexible_grasp_supervisor/gui/widgets/grasp6d_control_widget.py
    git diff --check

Expected: PASS and no whitespace errors.

- [ ] **Step 6: Commit Task 8**

    git add src/alicia_flexible_grasp_supervisor/gui/widgets/grasp6d_control_widget.py src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_control_widget.py src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml src/alicia_flexible_grasp_supervisor/launch/grasp_system.launch src/alicia_flexible_grasp_supervisor/docs/remote_grasp6d_wsl2.md
    git commit -m "feat: expose continuous grasp preview controls"

### Task 9: Full Integration, Build, and Performance-log Contract

**Files:**
- Modify only if a test exposes a defect: files already listed in Tasks 1-8.
- Test: all alicia_flexible_grasp_supervisor tests.

**Interfaces:**
- Verifies the complete ROS-to-WSL protocol and Preview-to-Execution lifecycle.
- Produces no hardware success claim.

- [ ] **Step 1: Run the focused end-to-end regression**

Run:

    PYTHONDONTWRITEBYTECODE=1 ./devel/env.sh python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_latest_only_inference.py src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_stability.py src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_pipeline.py src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_client.py src/alicia_flexible_grasp_supervisor/tests/test_graspnet_baseline_server_protocol.py src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_server_protocol.py src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_control_widget.py -q

Expected: PASS with zero failures.

- [ ] **Step 2: Validate performance-log schema with a deterministic fake run**

Run the streaming integration test that emits at least 100 fake requests with
known timing. Assert:

    metrics['latency_p95_ms'] == expected_p95
    metrics['stage_counts']['moveit_checked'] <= configured_top_n
    sum(metrics['rejection_counts'].values()) >= 0
    all(math.isfinite(value) for each numeric metric)
    json.dumps(metrics, allow_nan=False)

Expected: PASS. This verifies log shape, not RTX 4070 performance.

- [ ] **Step 3: Run the complete package test suite**

Run:

    PYTHONDONTWRITEBYTECODE=1 ./devel/env.sh python3 -m pytest src/alicia_flexible_grasp_supervisor/tests -q

Expected: zero failures. If an environment-only hardware test is skipped, record
its exact test name and skip reason in the final handoff.

- [ ] **Step 4: Build ROS interfaces and scripts**

Run:

    catkin_make --force-cmake --pkg alicia_flexible_grasp_supervisor

Expected: exit code 0. No ROS message definition changed, but this verifies
imports, installed scripts, and existing generated interfaces.

- [ ] **Step 5: Run static and repository checks**

Run:

    python3 -m py_compile src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/latest_only_inference.py src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/remote_grasp6d_client.py src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_stability.py src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_pipeline.py tools/graspnet_baseline_server.py tools/mujoco_digital_twin_server.py
    git diff --check
    git status --short

Expected: compile exit 0, diff check exit 0, and status contains only intended
source/docs changes plus the pre-existing user-owned __pycache entries.

- [ ] **Step 6: Review requirements against evidence**

Check the design requirement by requirement and record:

- exact tests proving latest-only replacement and stale drop;
- exact tests proving 3-of-5 stability and Rz(180) symmetry;
- exact tests proving soft preferences and hard safety gates;
- exact MoveIt top-N call count;
- exact Preview/Execution isolation and frozen-execution tests;
- protocol/GPU-cache tests;
- focused/full test counts and catkin build result;
- untested real ROS/WSL/GPU/arm risks.

- [ ] **Step 7: Commit final integration corrections, if any**

If Step 1-6 required a correction, stage only those reviewed files and run the
relevant focused tests again before:

    git commit -m "test: verify continuous grasp6d pipeline"

If no correction was needed, do not create an empty commit.
