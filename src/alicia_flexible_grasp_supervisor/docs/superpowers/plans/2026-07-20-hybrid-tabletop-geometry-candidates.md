# Hybrid Tabletop Geometry Candidates Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add category-independent, support-plane-aware top-down geometry candidates to the existing GraspNet pipeline so a real 50 mm Alicia gripper can select an object's narrow grasp direction without bypassing analytical, MoveIt, stability, MuJoCo, or rich-plan gates.

**Architecture:** A new pure NumPy generator converts one immutable target cloud and support plane into bounded tabletop proposals with explicit contact, approach, jaw, and aperture evidence. Source-specific adapters convert GraspNet and tabletop proposals into a shared candidate contract, then the existing continuous pipeline gates, tracks, simulates, audits, previews, and promotes them through one source-neutral path.

**Tech Stack:** ROS Noetic, Python 3.8, NumPy, pytest 4.6, geometry_msgs, existing GraspNet protocol v3, MoveIt, MuJoCo, PyQt5.

## Global Constraints

- The physical gripper contract remains `Alicia_D_v5_6_gripper_50mm` with maximum inner gap exactly `0.050 m`.
- Required opening is the cleaned target-cloud projection plus `0.002 m` clearance on each jaw side.
- Tabletop jaw angles cover `[0, 180 degrees)` in `15-degree` increments plus OBB principal directions, deduplicated within `2 degrees`.
- At most eight tabletop geometry proposals may leave the generator per snapshot.
- The semantic insertion direction is `-support_normal`; the jaw direction lies in the support plane.
- Geometry candidates never fabricate GraspNet `depth_m`, `width_m`, or model confidence.
- Candidate sources are `graspnet` and `tabletop_geometry`; cross-source ranking has no fixed source preference.
- Every source uses the same 50 mm aperture, CAD envelope, swept collision, MoveIt, three-hit stability, MuJoCo, and rich-plan integrity requirements.
- Preview generation never authorizes physical motion; Execution publication remains explicit and integrity-bound.
- No new failure path publishes robot motion, protective stop, torque-off, or torque-on commands.
- Preserve all pre-existing dirty-worktree changes. Stage and commit only the files listed by each task.

---

## File Structure

### New focused modules

- `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/tabletop_geometry_candidates.py`
  owns support-frame construction, angle sampling, cloud projection, bilateral contact evidence, explicit tabletop proposal records, and bounded source-local ranking.
- `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/hybrid_grasp_candidates.py`
  owns normalized source-aware planning records, source-neutral deduplication, source-lineage merging, and conversions into stability observations.

### Existing modules with targeted changes

- `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/gripper_geometry.py`
  gains a public explicit-tool0 evaluation path while retaining the strict GraspNet depth path unchanged.
- `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_stability.py`
  carries candidate source lineage and permits `model_width_m=None` for geometry candidates.
- `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/rich_plan_integrity.py`
  binds candidate source and source lineage into the rich-plan digest.
- `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/mujoco_digital_twin_client.py` and `tools/mujoco_digital_twin_server.py`
  carry optional model width, source lineage, and category-independent OBB simulation through MuJoCo schema v3.
- `src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py`
  adapts both sources into the common pipeline, rechecks stable candidates by source, emits source-aware audit data, and keeps Preview/Execution authority unchanged.
- `src/alicia_flexible_grasp_supervisor/msg/Grasp6DPlan.msg`
  adds canonical candidate-source provenance fields.
- `src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml`
  adds validated production defaults from the approved specification.
- `src/alicia_flexible_grasp_supervisor/gui/widgets/grasp6d_control_widget.py`
  displays the selected source without changing controls.
- `src/alicia_flexible_grasp_supervisor/tools/benchmark_tabletop_geometry.py`
  captures one synchronized RealSense regression fixture and benchmarks the pure generator without exposing motion controls.

---

### Task 1: Pure Tabletop Proposal Generator

**Files:**
- Create: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/tabletop_geometry_candidates.py`
- Create: `src/alicia_flexible_grasp_supervisor/tests/test_tabletop_geometry_candidates.py`

**Interfaces:**
- Consumes: cleaned `object_points_base: np.ndarray`, OBB center/axes/size, support point/normal, and `TabletopGeometryConfig`.
- Produces: `generate_tabletop_proposals -> TabletopGenerationResult`, containing at most eight immutable `TabletopProposal` records.

- [ ] **Step 1: Write failing tests for the real carton dimensions and category independence**

Add helpers and tests that build a dense rectangular surface cloud, rotate it around the support normal, and never pass a class label:

```python
def box_cloud(size_xyz, yaw_rad=0.0):
    xs = np.linspace(-size_xyz[0] / 2.0, size_xyz[0] / 2.0, 21)
    ys = np.linspace(-size_xyz[1] / 2.0, size_xyz[1] / 2.0, 17)
    zs = np.linspace(0.0, size_xyz[2], 7)
    points = []
    for x in xs:
        for z in zs:
            points.extend(((x, -size_xyz[1] / 2.0, z), (x, size_xyz[1] / 2.0, z)))
    for y in ys:
        for z in zs:
            points.extend(((-size_xyz[0] / 2.0, y, z), (size_xyz[0] / 2.0, y, z)))
    rotation = np.array([
        [np.cos(yaw_rad), -np.sin(yaw_rad), 0.0],
        [np.sin(yaw_rad), np.cos(yaw_rad), 0.0],
        [0.0, 0.0, 1.0],
    ])
    return np.asarray(points, dtype=float).dot(rotation.T)


def test_real_carton_prefers_35mm_side_and_requires_39mm():
    result = generate_tabletop_proposals(
        object_points_base=box_cloud((0.051, 0.035, 0.011)),
        obb_center_base=np.array([0.0, 0.0, 0.0055]),
        R_base_obb=np.eye(3),
        obb_size_xyz_m=np.array([0.051, 0.035, 0.011]),
        support_point_base=np.zeros(3),
        support_normal_base=np.array([0.0, 0.0, 1.0]),
        config=TabletopGeometryConfig(),
    )
    assert result.ok
    assert len(result.proposals) <= 8
    best = result.proposals[0]
    assert best.required_open_width_m == pytest.approx(0.039, abs=5e-4)
    assert abs(np.dot(best.jaw_axis_base, [0.0, 1.0, 0.0])) > 0.999
    assert np.dot(best.insertion_axis_base, [0.0, 0.0, 1.0]) < -0.999


@pytest.mark.parametrize('yaw_deg', (0.0, 17.0, 63.0, 121.0))
def test_rotated_unknown_instance_preserves_short_side_solution(yaw_deg):
    yaw = np.deg2rad(yaw_deg)
    points = box_cloud((0.051, 0.035, 0.011), yaw)
    result = generate_tabletop_proposals(
        points,
        np.array([0.0, 0.0, 0.0055]),
        rotation_about_z(yaw),
        np.array([0.051, 0.035, 0.011]),
        np.zeros(3),
        np.array([0.0, 0.0, 1.0]),
        TabletopGeometryConfig(),
    )
    assert result.proposals[0].required_open_width_m == pytest.approx(0.039, abs=8e-4)
```

- [ ] **Step 2: Run the focused tests and verify the import fails**

Run:

```bash
source devel/setup.bash
python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_tabletop_geometry_candidates.py -q
```

Expected: collection fails because `tabletop_geometry_candidates` does not exist.

- [ ] **Step 3: Implement immutable configuration, proposal, result, and projection helpers**

Create the module with these public contracts and validation rules:

```python
@dataclass(frozen=True)
class TabletopGeometryConfig:
    max_inner_gap_m: float = 0.050
    angle_step_deg: float = 15.0
    angle_dedup_deg: float = 2.0
    jaw_clearance_each_side_m: float = 0.002
    min_contact_band_points: int = 6
    contact_band_fraction: float = 0.12
    max_candidates: int = 8


@dataclass(frozen=True)
class TabletopProposal:
    source_index: int
    contact_center_base: np.ndarray
    insertion_axis_base: np.ndarray
    jaw_axis_base: np.ndarray
    required_open_width_m: float
    aperture_margin_m: float
    negative_contact_count: int
    positive_contact_count: int
    contact_symmetry: float
    source_score: float
    angle_deg: float
    audit: Mapping


@dataclass(frozen=True)
class TabletopGenerationResult:
    proposals: tuple
    failure_code: str
    failure_reason: str
    sampled_angles_deg: tuple

    @property
    def ok(self):
        return bool(self.proposals)


def generate_tabletop_proposals(
    object_points_base,
    obb_center_base,
    R_base_obb,
    obb_size_xyz_m,
    support_point_base,
    support_normal_base,
    config,
):
    points = _finite_points(object_points_base)
    center = _finite_vector(obb_center_base, 'obb_center_base')
    rotation = _validated_rotation(R_base_obb)
    size = _positive_vector(obb_size_xyz_m, 'obb_size_xyz_m')
    support_point = _finite_vector(support_point_base, 'support_point_base')
    normal = _unit_vector(support_normal_base, 'support_normal_base')
    if float(np.dot(rotation[:, 2], normal)) < 1.0 - 1e-5:
        return TabletopGenerationResult((), 'SUPPORT_PLANE_INVALID',
                                         'OBB vertical axis does not match support normal', ())
    angles = _sample_angles(rotation, normal, config)
    proposals = []
    for angle_deg, jaw_axis in angles:
        projection = points.dot(jaw_axis)
        lower = float(np.min(projection))
        upper = float(np.max(projection))
        required = upper - lower + 2.0 * config.jaw_clearance_each_side_m
        if not 0.0 < required <= config.max_inner_gap_m:
            continue
        negative, positive = _contact_counts(
            projection, lower, upper, config.contact_band_fraction
        )
        if min(negative, positive) < config.min_contact_band_points:
            continue
        symmetry = min(negative, positive) / float(max(negative, positive))
        proposal = TabletopProposal(
            source_index=len(proposals),
            contact_center_base=_robust_contact_center(points, support_point, normal),
            insertion_axis_base=-normal,
            jaw_axis_base=jaw_axis,
            required_open_width_m=required,
            aperture_margin_m=config.max_inner_gap_m - required,
            negative_contact_count=negative,
            positive_contact_count=positive,
            contact_symmetry=symmetry,
            source_score=-(config.max_inner_gap_m - required) - 0.01 * symmetry,
            angle_deg=angle_deg,
            audit=MappingProxyType({'projection_min_m': lower, 'projection_max_m': upper}),
        )
        proposals.append(proposal)
    proposals.sort(key=lambda item: (item.source_score, item.angle_deg))
    bounded = tuple(replace(item, source_index=index)
                    for index, item in enumerate(proposals[:config.max_candidates]))
    if not bounded:
        return TabletopGenerationResult((), 'NO_FIT_DIRECTION',
                                         'no sampled jaw direction fits the 50 mm gripper',
                                         tuple(item[0] for item in angles))
    return TabletopGenerationResult(bounded, '', '', tuple(item[0] for item in angles))
```

Implement `_sample_angles` with OBB X/Y directions plus `np.arange(0, 180, 15)`, treating jaw axes as undirected and removing angles closer than 2 degrees. Freeze every returned NumPy array and audit mapping defensively.

Do not let validation exceptions escape the public generator. Map malformed configuration, OBB size/rotation, and non-finite scalar inputs to `TABLETOP_GEOMETRY_INPUT_INVALID`; empty, sparse, or non-finite object points to `TARGET_CLOUD_INVALID`; zero, non-unit, or OBB-inconsistent support normals to `SUPPORT_PLANE_INVALID`; no aperture-valid angle to `NO_FIT_DIRECTION`; and aperture-valid angles whose two contact bands fail to `CONTACT_SUPPORT_INVALID`. Track `width_valid_count` and `contact_valid_count` explicitly so the last two cases cannot collapse into the same diagnostic.

- [ ] **Step 4: Add invalid-input, no-fit, bilateral-contact, and bounded-output tests**

Add tests asserting stable failures for non-finite points, zero support normal, misaligned OBB, a `0.060 x 0.055 m` object, one-sided sparse points, and `max_candidates=3`.

- [ ] **Step 5: Run the generator tests**

Run the focused command from Step 2.

Expected: all tests pass; the real carton case reports approximately `0.039 m`.

- [ ] **Step 6: Commit the pure generator**

```bash
git add src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/tabletop_geometry_candidates.py \
        src/alicia_flexible_grasp_supervisor/tests/test_tabletop_geometry_candidates.py
git commit -m "feat: generate category-independent tabletop grasps"
```

---

### Task 2: Explicit Tool0 Pose and Analytical Gripper Gate

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/tabletop_geometry_candidates.py`
- Modify: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/gripper_geometry.py:580`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_tabletop_geometry_candidates.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_gripper_geometry.py`

**Interfaces:**
- Consumes: `TabletopProposal`, existing CAD centers and boxes from `gripper_geometry.py`, `tool_jaw_axis='y'`, `tool_finger_length_axis='z'`.
- Produces: `materialize_tabletop_candidates -> tuple[TabletopCandidate]` with explicit `T_base_tool0`; `evaluate_explicit_candidate -> CandidateGateResult` without a GraspNet depth field.

- [ ] **Step 1: Write failing tests for physical top-down placement**

Add tests requiring the generated tool rotation to map the semantic insertion and jaw axes correctly, preserve 3 mm support clearance, and create two exact 180-degree wrist variants:

```python
def test_materialized_carton_candidate_places_fingers_above_table():
    candidates = materialize_tabletop_candidates(
        proposal=real_carton_result().proposals[0],
        support_point_base=np.zeros(3),
        support_normal_base=np.array([0.0, 0.0, 1.0]),
        gripper=GRIPPER,
        tool_jaw_axis='y',
        tool_finger_length_axis='z',
    )
    assert len(candidates) == 2
    candidate = candidates[0]
    np.testing.assert_allclose(candidate.T_base_tool0[:3, 1], candidate.jaw_axis_base)
    np.testing.assert_allclose(candidate.T_base_tool0[:3, 2], candidate.insertion_axis_base)
    assert candidate.minimum_finger_support_clearance_m >= 0.003 - 1e-9
    assert candidate.required_open_width_m == pytest.approx(0.039, abs=5e-4)
    relative = candidates[1].T_base_tool0[:3, :3].T @ candidate.T_base_tool0[:3, :3]
    np.testing.assert_allclose(relative, np.diag([-1.0, -1.0, 1.0]), atol=1e-8)


def test_explicit_candidate_never_requires_or_fabricates_graspnet_depth():
    result = evaluate_explicit_candidate(**explicit_carton_fixture())
    assert result.ok
    assert result.required_open_width_m == pytest.approx(0.039, abs=5e-4)
```

Retain a regression test proving `evaluate_candidate` still rejects missing, non-finite, or non-bin GraspNet depth.

- [ ] **Step 2: Run the two focused test files and verify failure**

```bash
source devel/setup.bash
python3 -m pytest \
  src/alicia_flexible_grasp_supervisor/tests/test_tabletop_geometry_candidates.py \
  src/alicia_flexible_grasp_supervisor/tests/test_gripper_geometry.py -q
```

Expected: new tests fail because materialization and explicit evaluation do not exist.

- [ ] **Step 3: Add the explicit candidate record and CAD-based pose solver**

Extend the new module with:

```python
@dataclass(frozen=True)
class TabletopCandidate:
    source_index: int
    variant_index: int
    contact_center_base: np.ndarray
    T_base_tool0: np.ndarray
    insertion_axis_base: np.ndarray
    jaw_axis_base: np.ndarray
    required_open_width_m: float
    minimum_finger_support_clearance_m: float
    source_score: float
    audit: Mapping


def materialize_tabletop_candidates(
    proposal,
    support_point_base,
    support_normal_base,
    gripper,
    tool_jaw_axis='y',
    tool_finger_length_axis='z',
):
    variants = []
    for variant_index, jaw_axis in enumerate(
        (proposal.jaw_axis_base, -proposal.jaw_axis_base)
    ):
        rotation = semantic_axes_to_tool_rotation(
            insertion_axis_base=proposal.insertion_axis_base,
            jaw_axis_base=jaw_axis,
            tool_jaw_axis=tool_jaw_axis,
            tool_finger_length_axis=tool_finger_length_axis,
        )
        transform = np.eye(4, dtype=float)
        transform[:3, :3] = rotation
        transform[:3, 3] = solve_tool0_translation_for_support_clearance(
            rotation=rotation,
            lateral_target=proposal.contact_center_base,
            support_point=support_point_base,
            support_normal=support_normal_base,
            clearance_m=gripper.support_clearance_m,
            gripper=gripper,
        )
        variants.append(
            _candidate_with_frozen_arrays(
                proposal,
                transform,
                variant_index=variant_index,
                jaw_axis_base=jaw_axis,
            )
        )
    return tuple(variants)
```

`solve_tool0_translation_for_support_clearance` must evaluate all eight corners of both analytical finger boxes at full physical opening. First align the jaw-center plane laterally with the proposal contact center, then translate only along the support normal until the minimum finger-corner clearance is exactly the configured clearance. Reject a pose whose usable finger side band no longer overlaps the proposal contact height or whose palm lies on the object side of the fingers.

Flatten direction variants in source-local score order and keep only the first `config.max_candidates` records, so the two wrist variants do not increase the pipeline input beyond eight candidates.

Raise `TabletopCandidateContractError(code, reason)` with code `TABLETOP_APPROACH_INVALID` when insertion is not antiparallel to the unit support normal or the jaw is not support-parallel. Use `TOOL0_GEOMETRY_INVALID` for non-orthonormal axis mapping, failed contact-band overlap, palm-side inversion, or inability to solve the 3 mm CAD clearance. `_prepare_prediction` catches only this typed exception, records its stable code, and continues with independently valid GraspNet candidates.

- [ ] **Step 4: Split source validation from the common analytical gates**

Refactor `gripper_geometry.py` so the current public `evaluate_candidate` retains its discrete GraspNet depth checks, while the new public wrapper supplies an already validated explicit tool0. Add `projected_cloud_width_m(points, jaw_axis, clearance_each_side_m)` to `gripper_geometry.py` and use that one helper from both the generator and explicit gate so projection semantics cannot drift:

```python
def evaluate_explicit_candidate(
    *,
    gripper,
    candidate_center_base,
    candidate_tool0_base,
    R_base_tool,
    required_open_width_m,
    target_points_base,
    obb_center_base,
    R_base_obb,
    obb_size_xyz_m,
    support_normal_base,
    support_offset_m,
    pregrasp_T_base_tool,
    approach_T_base_tool,
    grasp_T_base_tool,
    lift_T_base_tool,
    tool_jaw_axis='y',
    tool_finger_length_axis='z',
    motion_cost=0.0,
):
    recomputed = projected_cloud_width_m(
        target_points_base,
        R_base_tool @ parse_tool_axis(tool_jaw_axis)[0],
        gripper.jaw_clearance_each_side_m,
    )
    if abs(float(required_open_width_m) - recomputed) > 5e-4:
        return _failed_result(
            'jaw_width', 'GRIPPER_WIDTH_INVALID',
            'explicit width does not match target-cloud projection',
            2, recomputed, 0.0, -1.0e6, 0.0, motion_cost, 0.0,
        )
    return _evaluate_physical_candidate(
        gripper=gripper,
        candidate_center_base=candidate_center_base,
        candidate_tool0_base=candidate_tool0_base,
        R_base_tool=R_base_tool,
        required_width_m=recomputed,
        obb_center_base=obb_center_base,
        R_base_obb=R_base_obb,
        obb_size_xyz_m=obb_size_xyz_m,
        support_normal_base=support_normal_base,
        support_offset_m=support_offset_m,
        pregrasp_T_base_tool=pregrasp_T_base_tool,
        approach_T_base_tool=approach_T_base_tool,
        grasp_T_base_tool=grasp_T_base_tool,
        lift_T_base_tool=lift_T_base_tool,
        tool_jaw_axis=tool_jaw_axis,
        tool_finger_length_axis=tool_finger_length_axis,
        motion_cost=motion_cost,
    )
```

Move only the shared center, jaw-line, aperture, finger-reach, static-envelope, and swept-envelope code into `_evaluate_physical_candidate`. Do not weaken `validate_execution_tool0_contract` or the existing GraspNet wrapper.

Wrap `_evaluate_explicit_candidate_impl` with the same fail-closed exception boundary used by `evaluate_candidate`: typed analytical errors retain their code, unexpected validation errors become `GRIPPER_SWEEP_COLLISION`, and every public call returns a `CandidateGateResult` rather than throwing.

- [ ] **Step 5: Run analytical and generator regressions**

Run the Step 2 command.

Expected: new explicit tests and all existing GraspNet analytical tests pass.

- [ ] **Step 6: Commit the explicit physical path**

```bash
git add src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/tabletop_geometry_candidates.py \
        src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/gripper_geometry.py \
        src/alicia_flexible_grasp_supervisor/tests/test_tabletop_geometry_candidates.py \
        src/alicia_flexible_grasp_supervisor/tests/test_gripper_geometry.py
git commit -m "feat: validate explicit tabletop tool poses"
```

---

### Task 3: Source-aware Hybrid Candidate Contract and Tracking

**Files:**
- Create: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/hybrid_grasp_candidates.py`
- Create: `src/alicia_flexible_grasp_supervisor/tests/test_hybrid_grasp_candidates.py`
- Modify: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_stability.py:194`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_stability.py`
- Modify: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_pipeline.py:404`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_pipeline.py`

**Interfaces:**
- Consumes: already source-validated GraspNet or `TabletopCandidate` poses plus `CandidateGateResult`.
- Produces: immutable `NormalizedPlanningCandidate`, `merge_hybrid_candidates`, and source-aware `CandidateObservation`/`StableCandidate` records.

- [ ] **Step 1: Write failing hybrid deduplication and optional-model-field tests**

```python
def test_cross_source_duplicate_keeps_physical_winner_and_both_lineages():
    graspnet = normalized_candidate('graspnet', center=(0.0, 0.0, 0.01), common_cost=0.20)
    geometry = normalized_candidate('tabletop_geometry', center=(0.002, 0.0, 0.01), common_cost=0.10)
    merged = merge_hybrid_candidates(
        (graspnet, geometry),
        MergeConfig(0.005, 10.0, 10.0),
    )
    assert len(merged) == 1
    assert merged[0].candidate_source == 'tabletop_geometry'
    assert merged[0].source_lineage == ('graspnet', 'tabletop_geometry')


def test_geometry_observation_has_no_model_width_or_confidence():
    observation = observation_from_candidate(normalized_geometry_candidate())
    assert observation.model_width_m is None
    assert observation.model_score is None
    assert observation.candidate_source == 'tabletop_geometry'
```

Add stability tests proving the same physical pose may keep one track while its merged source lineage expands, and still requires at least three hits.

- [ ] **Step 2: Run focused tests and verify failure**

```bash
source devel/setup.bash
python3 -m pytest \
  src/alicia_flexible_grasp_supervisor/tests/test_hybrid_grasp_candidates.py \
  src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_stability.py \
  src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_pipeline.py -q
```

Expected: hybrid module import and optional-model assertions fail.

- [ ] **Step 3: Implement the normalized record and undirected-axis deduplication**

Create:

```python
VALID_CANDIDATE_SOURCES = frozenset(('graspnet', 'tabletop_geometry'))


@dataclass(frozen=True)
class MergeConfig:
    center_distance_m: float = 0.005
    insertion_angle_deg: float = 10.0
    jaw_angle_deg: float = 10.0


@dataclass(frozen=True)
class NormalizedPlanningCandidate:
    candidate_source: str
    source_index: int
    variant_index: int
    source_lineage: tuple
    contact_center_base: np.ndarray
    T_base_tool0: np.ndarray
    insertion_axis_base: np.ndarray
    jaw_axis_base: np.ndarray
    required_open_width_m: float
    model_width_m: Optional[float]
    model_score: Optional[float]
    source_local_score: float
    common_physical_cost: float
    geometry_gate: object
    grasp_sequence: object
    payload: object
    audit: Mapping


def merge_hybrid_candidates(candidates, config):
    ordered = sorted(candidates, key=lambda item: (
        item.common_physical_cost,
        item.required_open_width_m,
        -item.geometry_gate.support_clearance_m,
        item.source_index,
        item.variant_index,
    ))
    merged = []
    for candidate in ordered:
        match = next(
            (
                item
                for item in merged
                if item.candidate_source != candidate.candidate_source
                and item.variant_index == candidate.variant_index
                and _duplicates(item, candidate, config)
            ),
            None,
        )
        if match is None:
            merged.append(candidate)
            continue
        lineage = tuple(sorted(set(match.source_lineage + candidate.source_lineage)))
        winner = candidate if candidate.common_physical_cost < match.common_physical_cost else match
        merged[merged.index(match)] = replace(winner, source_lineage=lineage)
    return tuple(merged)
```

Use sign-invariant angles for jaw axes, ordinary vector angles for insertion axes, and defensive read-only copies. Deduplicate only across different producers and only for matching wrist-variant indices; never collapse the two 180-degree variants within one source before MoveIt. The ranking key contains no candidate-source name or source confidence. If every physical term ties, stable input order may choose the canonical record, but both source names remain in sorted lineage and the selected pose/cost are identical. Add reversed-input equality and same-source-two-variant regressions.

- [ ] **Step 4: Carry source lineage and optional model values through stability**

Modify `CandidateObservation` and `StableCandidate`:

```python
candidate_source: str
source_lineage: tuple
model_width_m: Optional[float]
model_score: Optional[float]
```

Validation must require a known canonical source, a non-empty sorted unique lineage containing that source, and either `None` or a finite non-negative model value. Track matching remains physical-pose based; fused output unions the lineages of its hits and uses the latest candidate source. Replace confidence-weighted fusion with normalized freshness weights for the shared tracker, because model confidence is unavailable for geometry and is not cross-source comparable. Fuse optional model width and score only over observations whose value is not `None`; slice the matching freshness weights, renormalize them, and return `None` when a track contains no value for that field.

- [ ] **Step 5: Remove model confidence from cross-source common cost**

Keep `soft_candidate_cost` backward compatible, but add a source-neutral wrapper used by hybrid candidates:

```python
def source_neutral_candidate_cost(features, weights):
    neutral = replace(features, model_score=0.0)
    return soft_candidate_cost(neutral, weights)
```

Extend `SoftCandidateFeatures` and `SoftScoreWeights` with `contact_balance` and `contact_balance_weight`. Validate the feature in `[0, 1]` and add `-contact_balance_weight * contact_balance` to the common cost. Compute it for both sources from the same cleaned target points and jaw-axis projection bands; do not reuse the geometry producer's source-local score. Test that changing only GraspNet confidence cannot make an otherwise identical GraspNet pose beat a lower physical-cost geometry pose, while better bilateral support can. Model confidence and the generator's contact-symmetry score remain available only for sorting candidates within their own sources before normalization.

- [ ] **Step 6: Run focused hybrid tests**

Run the Step 2 command.

Expected: all tests pass.

- [ ] **Step 7: Commit the shared contract**

```bash
git add src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/hybrid_grasp_candidates.py \
        src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_stability.py \
        src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_pipeline.py \
        src/alicia_flexible_grasp_supervisor/tests/test_hybrid_grasp_candidates.py \
        src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_stability.py \
        src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_pipeline.py
git commit -m "feat: add source-aware hybrid grasp contract"
```

---

### Task 4: Bind Candidate Source into Rich Plans

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/msg/Grasp6DPlan.msg`
- Modify: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/rich_plan_integrity.py:112`
- Modify: `src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/mujoco_digital_twin_client.py:150`
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py:2291`
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py:400`
- Modify: `src/alicia_flexible_grasp_supervisor/gui/widgets/grasp6d_control_widget.py:40`
- Modify: `tools/mujoco_digital_twin_server.py:150`
- Create: `src/alicia_flexible_grasp_supervisor/tests/test_rich_plan_candidate_source.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_control_widget.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_client.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_server_protocol.py`

**Interfaces:**
- Consumes: selected normalized candidate source and lineage.
- Produces: `Grasp6DPlan.candidate_source`, `candidate_source_lineage`, an explicit model-width presence bit, MuJoCo schema-v3 source provenance, and a plan ID that changes when any provenance field changes.

- [ ] **Step 1: Write failing message/integrity tests**

Add the fields expected by test doubles, then assert source validation and digest binding:

```python
def test_plan_id_binds_candidate_source_and_lineage():
    plan = valid_plan(
        candidate_source='graspnet',
        candidate_source_lineage=['graspnet'],
        has_candidate_model_width=True,
        candidate_width_m=0.039,
    )
    original = compute_plan_id(plan)
    plan.candidate_source_lineage = ['graspnet', 'tabletop_geometry']
    assert compute_plan_id(plan) != original


def test_geometry_plan_encodes_absent_model_width_without_fabricating_one():
    plan = valid_plan(
        candidate_source='tabletop_geometry',
        candidate_source_lineage=['tabletop_geometry'],
        has_candidate_model_width=False,
        candidate_width_m=0.0,
    )
    validate_candidate_model_width(plan)
    payload = build_request_payload(plan, valid_joint_state(), GRIPPER_CONFIG)
    assert payload['schema_version'] == 3
    assert payload['candidate_width_m'] is None


@pytest.mark.parametrize('source,lineage', (
    ('', []),
    ('unknown', ['unknown']),
    ('graspnet', []),
    ('tabletop_geometry', ['graspnet']),
))
def test_invalid_candidate_source_fails_closed(source, lineage):
    with pytest.raises(ValueError):
        validate_candidate_source(source, lineage)
```

- [ ] **Step 2: Run rich-plan tests and verify failure**

```bash
source devel/setup.bash
python3 -m pytest \
  src/alicia_flexible_grasp_supervisor/tests/test_rich_plan_candidate_source.py \
  src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py \
  src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_control_widget.py \
  src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_client.py \
  src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_server_protocol.py -q
```

Expected: missing message fields and validator failures.

- [ ] **Step 3: Extend the ROS message and canonical digest**

Append to `Grasp6DPlan.msg`:

```text
string candidate_source
string[] candidate_source_lineage
bool has_candidate_model_width
```

Add:

```python
SUPPORTED_CANDIDATE_SOURCES = frozenset(('graspnet', 'tabletop_geometry'))


def validate_candidate_source(source, lineage):
    canonical = str(source or '')
    items = tuple(str(item or '') for item in lineage or ())
    if canonical not in SUPPORTED_CANDIDATE_SOURCES:
        raise ValueError('candidate source is unsupported')
    if not items or tuple(sorted(set(items))) != items:
        raise ValueError('candidate source lineage must be sorted and unique')
    if canonical not in items or any(item not in SUPPORTED_CANDIDATE_SOURCES for item in items):
        raise ValueError('candidate source lineage is inconsistent')
    return canonical, items


def validate_candidate_model_width(plan):
    source, _lineage = validate_candidate_source(
        plan.candidate_source,
        plan.candidate_source_lineage,
    )
    present = bool(plan.has_candidate_model_width)
    width = float(plan.candidate_width_m)
    if not math.isfinite(width) or width < 0.0:
        raise ValueError('candidate model width is invalid')
    if source == 'graspnet' and (not present or width <= 0.0):
        raise ValueError('GraspNet plan requires its model width')
    if source == 'tabletop_geometry' and (present or width != 0.0):
        raise ValueError('geometry plan must encode absent model width')
    return None if not present else width
```

Pack UTF-8 source, each lineage item with explicit lengths, the model-width presence bit, and the paired float value in `canonical_plan_bytes` before pose bytes. `build_rich_plan` sets `has_candidate_model_width=True` only when `NormalizedPlanningCandidate.model_width_m` is present; otherwise it writes the ROS float field as exactly `0.0` and leaves the presence bit false. Update task-node and GUI validation to use `validate_candidate_model_width` rather than treating zero as a real prediction.

Upgrade the MuJoCo request payload to schema version 3. Send `candidate_source`, sorted `candidate_source_lineage`, and JSON `candidate_width_m: null` when the presence bit is false. Replace the semantic object-type literal `carton_box` with `obb_box`; the simulator still uses the measured pose and OBB dimensions, but no candidate path depends on a class label. The checked-in server accepts `null` only for canonical `tabletop_geometry`, requires a positive finite number for `graspnet`, validates the lineage and `obb_box`, and echoes the same provenance in its result. Add client/server protocol tests for both sources, mismatched source/width pairs, and rejection of category-specific object types. This wire representation prevents a geometry candidate from masquerading as a GraspNet-width prediction and keeps future unknown masks category-independent.

- [ ] **Step 4: Rebuild generated messages**

```bash
catkin_make --pkg alicia_flexible_grasp_supervisor
source devel/setup.bash
```

Expected: message generation completes and Python exposes both new fields.

- [ ] **Step 5: Run focused rich-plan regressions**

Run the Step 2 command plus:

```bash
python3 -m pytest src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py -q
```

Expected: all source, integrity, producer, consumer, and GUI tests pass.

- [ ] **Step 6: Commit source-bound plans**

```bash
git add src/alicia_flexible_grasp_supervisor/msg/Grasp6DPlan.msg \
        src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/rich_plan_integrity.py \
        src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/mujoco_digital_twin_client.py \
        src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py \
        src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py \
        src/alicia_flexible_grasp_supervisor/gui/widgets/grasp6d_control_widget.py \
        tools/mujoco_digital_twin_server.py \
        src/alicia_flexible_grasp_supervisor/tests/test_rich_plan_candidate_source.py \
        src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py \
        src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py \
        src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_control_widget.py \
        src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_client.py \
        src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_server_protocol.py
git commit -m "feat: bind grasp source into rich plans"
```

---

### Task 5: Integrate Both Sources into the Continuous ROS Pipeline

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py:407`
- Modify: `src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml:168`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_graspnet_input_default_config.py`

**Interfaces:**
- Consumes: `PreparedPrediction.geometry.object_points_base`, Tabletop generator, GraspNet candidates, source-specific gate adapters, hybrid merge contract.
- Produces: one merged tuple of `CandidateObservation` records and unchanged Preview/Execution publication flow.

- [ ] **Step 1: Write failing integration tests for a graspable narrow side**

Create a prepared snapshot whose GraspNet candidates all collide with the table but whose `51 x 35 x 11 mm` cloud is valid:

```python
def test_local_pipeline_keeps_tabletop_candidate_when_graspnet_hits_table(node):
    prepared = prepared_prediction(
        geometry=carton_geometry(points=box_cloud((0.051, 0.035, 0.011))),
        candidates=(upward_or_side_graspnet_candidate(),),
    )
    funnel, observations = node._evaluate_local_candidates(prepared)
    assert funnel['source_counts']['graspnet']['locally_valid'] == 0
    assert funnel['source_counts']['tabletop_geometry']['locally_valid'] >= 1
    assert any(item.candidate_source == 'tabletop_geometry' for item in observations)


def test_geometry_preview_survives_graspnet_transport_failure_but_cannot_promote_without_mujoco(node):
    prepared = prepared_prediction_with_remote_failure('WSL_UNAVAILABLE')
    result = node._process_prepared_prediction(prepared)
    assert result.preview_count >= 1
    assert result.promotion_count == 0
    assert result.primary_failure == 'WSL_UNAVAILABLE'
```

Also test that invalid support geometry emits no geometry candidate, that a target wider than 50 mm returns `NO_FIT_DIRECTION`, and that target epoch invalidation clears both sources.

- [ ] **Step 2: Run streaming tests and verify failure**

```bash
source devel/setup.bash
python3 -m pytest \
  src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py \
  src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py \
  src/alicia_flexible_grasp_supervisor/tests/test_graspnet_input_default_config.py -q
```

Expected: geometry-source assertions fail.

- [ ] **Step 3: Load and validate production geometry configuration**

Add the approved YAML block under `/grasp_6d/remote`:

```yaml
tabletop_geometry_candidates:
  enabled: true
  angle_step_deg: 15.0
  angle_dedup_deg: 2.0
  jaw_clearance_each_side_m: 0.002
  min_contact_band_points: 6
  contact_band_fraction: 0.12
  min_finger_support_clearance_m: 0.003
  max_candidates: 8
  merge_center_distance_m: 0.005
  merge_insertion_angle_deg: 10.0
  merge_jaw_angle_deg: 10.0
```

Construct immutable `TabletopGeometryConfig` and `MergeConfig` at startup and on the existing atomic runtime refresh. Set `TabletopGeometryConfig.max_inner_gap_m` from `self.gripper_geometry.max_inner_gap_m`, not from a second YAML width. Reject jaw/support clearances that conflict with the analytical gripper contract using `CONTINUOUS_CONFIG_INVALID`.

- [ ] **Step 4: Generate tabletop proposals before source-specific adaptation**

Extend `PreparedPrediction` with:

```python
tabletop_candidates: tuple = ()
remote_failure_code: str = ''
remote_failure_reason: str = ''
```

After snapshot geometry succeeds, call `generate_tabletop_proposals`, materialize both wrist variants, sort by `(source_score, source_index, variant_index)`, and retain at most eight in `tabletop_candidates`. A failed WSL prediction becomes source diagnostics rather than aborting the request when at least one tabletop candidate exists. If the WSL endpoint is unavailable, permit local Preview evidence but set promotion count to zero because the required MuJoCo service shares that endpoint.

- [ ] **Step 5: Split local evaluation into source adapters and merge the results**

Add two focused methods. In `_normalize_graspnet_candidate`, move the current conversion, tool0-contract validation, sequence construction, analytical gate, mandatory safety gate, feature calculation, and payload construction statements out of `_evaluate_local_candidates` without changing their order. Replace only the final observation construction with this exact normalized return:

```python
def _normalize_graspnet_candidate(self, prepared, raw_candidate, source_index):
    camera_candidate, grasp_pose, center_base, sequence, gate = (
        self._evaluate_graspnet_source_contract(
            prepared,
            raw_candidate,
            source_index,
        )
    )
    transform = pose_matrix(grasp_pose)
    approach = self._pose_approach_base_xyz(grasp_pose)
    jaw_axis = self._pose_jaw_base_xyz(grasp_pose)
    features = self._common_soft_features(
        prepared=prepared,
        contact_center_base=center_base,
        grasp_pose=grasp_pose,
        approach_axis_base=approach,
        jaw_axis_base=jaw_axis,
        gate=gate,
    )
    common_score = source_neutral_candidate_cost(
        features,
        self.soft_score_weights,
    )
    payload = LocalCandidatePayload(
        raw_candidate_index=int(source_index),
        variant_index=0,
        raw_candidate=raw_candidate,
        camera_candidate=camera_candidate,
        grasp_pose=grasp_pose,
        geometry_gate=gate,
        soft_features=features,
        score_components=dict(common_score.components),
    )
    return NormalizedPlanningCandidate(
        candidate_source='graspnet',
        source_index=int(source_index),
        variant_index=0,
        source_lineage=('graspnet',),
        contact_center_base=np.asarray(center_base, dtype=float),
        T_base_tool0=transform,
        insertion_axis_base=approach,
        jaw_axis_base=jaw_axis,
        required_open_width_m=float(gate.required_open_width_m),
        model_width_m=float(camera_candidate.width_m),
        model_score=float(camera_candidate.score),
        source_local_score=-float(camera_candidate.score),
        common_physical_cost=float(common_score.total),
        geometry_gate=gate,
        grasp_sequence=sequence,
        payload=payload,
        audit=MappingProxyType({
            'depth_m': float(camera_candidate.depth_m),
            'model_width_m': float(camera_candidate.width_m),
            'model_score': float(camera_candidate.score),
        }),
    )


def _normalize_tabletop_candidate(self, prepared, tabletop_candidate):
    transform = np.asarray(tabletop_candidate.T_base_tool0, dtype=float)
    quaternion = quaternion_from_matrix(transform)
    grasp_pose = make_pose_stamped(
        'base_link',
        transform[:3, 3],
        quaternion,
        stamp=prepared.stamp,
    )
    sequence = make_grasp_sequence_from_grasp_pose(
        grasp_pose,
        pregrasp_distance_m=float(self.grasp_config['pregrasp_distance_m']),
        approach_offset_m=float(self.grasp_config['final_approach_offset_m']),
        lift_height_m=float(self.grasp_config['lift_height_m']),
        approach_direction_base=tabletop_candidate.insertion_axis_base,
    )
    geometry = prepared.geometry
    gate = evaluate_explicit_candidate(
        gripper=self.gripper_geometry,
        candidate_center_base=tabletop_candidate.contact_center_base,
        candidate_tool0_base=transform[:3, 3],
        R_base_tool=transform[:3, :3],
        required_open_width_m=tabletop_candidate.required_open_width_m,
        target_points_base=geometry.object_points_base,
        obb_center_base=geometry.center_base,
        R_base_obb=geometry.axes_base,
        obb_size_xyz_m=geometry.size_xyz_m,
        support_normal_base=geometry.support_normal_base,
        support_offset_m=geometry.support_offset_m,
        pregrasp_T_base_tool=pose_matrix(sequence.pregrasp),
        approach_T_base_tool=pose_matrix(sequence.approach),
        grasp_T_base_tool=pose_matrix(sequence.grasp),
        lift_T_base_tool=pose_matrix(sequence.lift),
        tool_jaw_axis=self.gripper_tool_jaw_axis,
        tool_finger_length_axis=self.gripper_tool_finger_length_axis,
        motion_cost=0.0,
    )
    features = self._common_soft_features(
        prepared=prepared,
        contact_center_base=tabletop_candidate.contact_center_base,
        grasp_pose=grasp_pose,
        approach_axis_base=tabletop_candidate.insertion_axis_base,
        jaw_axis_base=tabletop_candidate.jaw_axis_base,
        gate=gate,
    )
    common_score = source_neutral_candidate_cost(
        features,
        self.soft_score_weights,
    )
    return NormalizedPlanningCandidate(
        candidate_source='tabletop_geometry',
        source_index=int(tabletop_candidate.source_index),
        variant_index=int(tabletop_candidate.variant_index),
        source_lineage=('tabletop_geometry',),
        contact_center_base=tabletop_candidate.contact_center_base,
        T_base_tool0=transform,
        insertion_axis_base=tabletop_candidate.insertion_axis_base,
        jaw_axis_base=tabletop_candidate.jaw_axis_base,
        required_open_width_m=float(tabletop_candidate.required_open_width_m),
        model_width_m=None,
        model_score=None,
        source_local_score=float(tabletop_candidate.source_score),
        common_physical_cost=float(common_score.total),
        geometry_gate=gate,
        grasp_sequence=sequence,
        payload=tabletop_candidate,
        audit=MappingProxyType(dict(tabletop_candidate.audit)),
    )
```

Add `approach_direction_base=None` to `make_grasp_sequence_from_grasp_pose`. Implement one `_validated_approach_direction` helper that reshapes to three components, rejects non-finite or near-zero input, and returns a unit vector. When the explicit direction is supplied, `_offset_along_vector(grasp_pose, -abs(distance), direction)` creates pregrasp and approach; otherwise the existing `_offset_along_approach_axis` branch remains byte-for-byte unchanged for GraspNet.

Evaluate hard gates for both lists, sort each source by `(source_local_score, source_index, variant_index)`, merge with `merge_hybrid_candidates`, then create `CandidateObservation` with `candidate_source`, `source_lineage`, the normalized physical fields, optional model fields, and `payload=normalized_candidate`. `_common_soft_features` must use `model_score=0.0`, the shared bilateral `contact_balance`, Euclidean target-center distance, actual insertion/support and jaw/support cosines, analytical margins, current visibility cost, and zero MoveIt/stability terms; source-local scores never enter it.

- [ ] **Step 6: Dispatch stable rechecks by source**

In `_recheck_and_score_stable`, require `stable.payload` to be a `NormalizedPlanningCandidate` and branch on `stable.payload.candidate_source`. GraspNet rechecks use `stable.payload.payload` as the existing `LocalCandidatePayload` and preserve the current depth/tool0 reconstruction. Tabletop rechecks use the stable fused pose plus `prepared.geometry.object_points_base`, re-run `evaluate_explicit_candidate`, and never request a depth value. Both source branches still evaluate the two 0/180 wrist variants, mandatory safety gate, strict MoveIt result, and source-neutral score before selection.

- [ ] **Step 7: Run continuous pipeline tests**

Run the Step 2 command.

Expected: hybrid local evaluation, remote-failure Preview, stability, invalidation, and default-config tests pass.

- [ ] **Step 8: Commit continuous integration**

```bash
git add src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py \
        src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_sequence.py \
        src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml \
        src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py \
        src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py \
        src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_sequence.py \
        src/alicia_flexible_grasp_supervisor/tests/test_graspnet_input_default_config.py
git commit -m "feat: merge tabletop and GraspNet candidates"
```

---

### Task 6: Source-aware Audit, Metrics, and GUI Status

**Files:**
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py:6052`
- Modify: `src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py:9913`
- Modify: `src/alicia_flexible_grasp_supervisor/gui/widgets/grasp6d_control_widget.py:447`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py`
- Modify: `src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_control_widget.py`

**Interfaces:**
- Consumes: normalized candidate source, lineage, projection audit, gate decisions, stability and MuJoCo outcomes.
- Produces: bounded audit JSON and metrics with `source_counts`, plus selected-source GUI text.

- [ ] **Step 1: Write failing audit and GUI tests**

```python
def test_gate_audit_records_both_sources_and_projection_evidence(node):
    node._active_gate_audit_report = hybrid_base_audit_report()
    report = node._finalize_gate_audit_report(
        evaluation_records=hybrid_evaluation_records(),
        selected_candidate=None,
        selected_pose=None,
        plan_id='',
        outcome_code='NO_STABLE_PLAN',
        outcome_reason='test fixture has not selected a plan',
        valid_plan=False,
    )
    geometry_rows = [row for row in report['rows']
                     if row['candidate_source'] == 'tabletop_geometry']
    assert geometry_rows
    row = geometry_rows[0]
    assert row['source_lineage'] == ['tabletop_geometry']
    assert row['required_open_width_m'] < 0.050
    assert row['projection_min_m'] < row['projection_max_m']
    assert abs(np.dot(row['jaw_axis_base'], row['support_normal_base'])) < 1e-3


def test_audit_lineage_does_not_collide_when_sources_share_numeric_indices(node):
    report = finalize_audit_with_same_numeric_indices(node)
    keys = {
        (row['candidate_source'], row['source_index'], row['variant_index'])
        for row in report['rows']
    }
    assert ('graspnet', 0, 0) in keys
    assert ('tabletop_geometry', 0, 0) in keys


def test_gui_summary_displays_geometry_source_without_enabling_execution():
    plan = valid_preview(candidate_source='tabletop_geometry')
    widget._update_preview_plan(plan)
    assert '桌面几何' in widget._preview_readiness.state().text
    assert not widget._execution_readiness.state().fresh
```

- [ ] **Step 2: Run audit/GUI tests and verify failure**

```bash
source devel/setup.bash
python3 -m pytest \
  src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py \
  src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py \
  src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_control_widget.py -q
```

Expected: source-aware fields and text are missing.

- [ ] **Step 3: Add bounded per-source funnel and row evidence**

Add this metrics shape without removing existing aggregate fields:

```python
'source_counts': {
    'graspnet': {
        'generated': int(graspnet_generated),
        'locally_valid': int(graspnet_valid),
        'stable': int(graspnet_stable),
        'preview': int(graspnet_preview),
        'promoted': int(graspnet_promoted),
    },
    'tabletop_geometry': {
        'generated': int(geometry_generated),
        'locally_valid': int(geometry_valid),
        'stable': int(geometry_stable),
        'preview': int(geometry_preview),
        'promoted': int(geometry_promoted),
    },
}
```

Every geometry row records its source indices, merged lineage, sampled angle, insertion/jaw axes, contact counts, projection extrema, required width, explicit tool0 transform, gate stages, tracking result, and MuJoCo result. Keep raw target point samples out of the production audit; record only hashes and bounded projection evidence.

Replace every audit identity key `(candidate_index, variant_index)` with `(candidate_source, source_index, variant_index)` in base rows, selector evaluation records, selected-candidate lookup, consistency checks, and the final lineage list. Keep the legacy numeric `candidate_index` field as a GraspNet-only display alias, but never use it as identity. Validate canonical source and sorted lineage before an audit row can be finalized; this prevents GraspNet index zero and geometry index zero from overwriting one another.

- [ ] **Step 4: Show selected source in status and GUI**

Status text for a ready plan begins with:

```python
'remote 6D plan ready source=%s score=%.3f required_open=%.3f' % (
    selected.candidate_source,
    selected.score,
    selected.required_open_width_m,
)
```

Map `graspnet` to `GraspNet` and `tabletop_geometry` to `桌面几何` in GUI plan-state text. Do not alter Generate, Stop Generation, Execute, or Stop buttons.

- [ ] **Step 5: Run focused audit and GUI tests**

Run the Step 2 command.

Expected: per-source metrics, bounded audit provenance, and Preview/Execution separation tests pass.

- [ ] **Step 6: Commit diagnostics**

```bash
git add src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py \
        src/alicia_flexible_grasp_supervisor/gui/widgets/grasp6d_control_widget.py \
        src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py \
        src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_node.py \
        src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_control_widget.py
git commit -m "feat: audit hybrid grasp candidate sources"
```

---

### Task 7: RealSense Fixture, Performance, Regression, and Preview Acceptance

**Files:**
- Create: `src/alicia_flexible_grasp_supervisor/tests/fixtures/carton_tabletop_cloud.json`
- Create: `src/alicia_flexible_grasp_supervisor/tests/test_tabletop_geometry_realsense_fixture.py`
- Create: `src/alicia_flexible_grasp_supervisor/tools/benchmark_tabletop_geometry.py`
- Modify: `src/alicia_flexible_grasp_supervisor/docs/remote_grasp6d_wsl2.md`

**Interfaces:**
- Consumes: one bounded cleaned target cloud captured from the current RealSense planning snapshot, production configuration, combined WSL endpoint.
- Produces: reproducible fixture regression, measured CPU timing, runbook instructions, and live Preview evidence without physical execution.

- [ ] **Step 1: Capture one bounded real target cloud fixture through a test-only command**

Add a `--dump-tabletop-fixture PATH` option to `tools/benchmark_tabletop_geometry.py`. The capture mode uses `message_filters.ApproximateTimeSynchronizer(queue_size=10, slop=0.08)` on these stamped topics:

- `/supervisor/camera/depth/image_raw` (`sensor_msgs/Image`);
- `/perception/object_mask` (`sensor_msgs/Image`);
- `/grasp_6d/object_geometry` (`alicia_flexible_grasp_supervisor/ObjectGeometry`).

Read `fx`, `fy`, `cx`, `cy`, and `depth_scale` from `/camera`; reject absent, non-finite, or non-positive focal/depth-scale values. Convert `16UC1` depth using `depth_scale` and accept `32FC1` as metres. Back-project only finite positive pixels whose synchronized mask value is non-zero. Look up the frozen `base_link <- depth.header.frame_id` transform at the depth stamp, transform the masked optical points to base coordinates, and require the ObjectGeometry header to match that stamp within the synchronizer tolerance.

Use the geometry message's pose quaternion as `R_base_obb`, its pose position as `obb_center_base`, and its support normal/offset as the plane equation. Remove points whose signed support height is below `/grasp_6d/remote/target_cloud_support_plane_min_height_m`; then crop to the OBB expanded by 5 mm on each axis. Deterministically lexicographically sort 2 mm voxel indices, retain the first point per voxel, and evenly subsample to 512 when needed. Reject captures with fewer than 120 retained points.

The fixture schema contains exactly these keys and types:

- `schema_version`: integer `1`;
- `source`: string `realsense`;
- `depth_stamp_ns`: positive integer;
- `depth_frame`: non-empty string;
- `intrinsics`: object containing numeric `fx`, `fy`, `cx`, `cy`, and `depth_scale`;
- `T_base_depth`: finite 4-by-4 numeric array;
- `object_points_base`: between 120 and 512 finite XYZ rows;
- `obb_center_base`, `obb_size_xyz_m`, `support_point_base`, `support_normal_base`: finite XYZ rows;
- `R_base_obb`: finite orthonormal 3-by-3 array;
- `audit_sha256`: 64-character lowercase SHA-256 of the exact planning-audit bytes.

Derive `support_point_base` as `-support_offset_m * support_normal_base`. Read the audit path from `/grasp_6d/remote/gate_audit_output_path`, require its snapshot stamp to equal `depth_stamp_ns`, and require a successful `tabletop_geometry` row before writing. The tool calls only `/grasp_6d/request_plan` with `trigger:true` to obtain that Preview evidence and always calls the same service with `trigger:false` in a `finally` block. It must not import or call `/grasp/start`, `/grasp/stop`, torque, joint, Cartesian, or execution services.

Run while the carton is segmented and stationary:

```bash
source devel/setup.bash
python3 src/alicia_flexible_grasp_supervisor/tools/benchmark_tabletop_geometry.py \
  --dump-tabletop-fixture \
  src/alicia_flexible_grasp_supervisor/tests/fixtures/carton_tabletop_cloud.json
```

Expected: the command prints `captured tabletop fixture points=<N>`, where `120 <= N <= 512`, and does not call any grasp-execution or motion service.

- [ ] **Step 2: Add fixture and performance tests**

```python
def test_realsense_carton_fixture_has_a_width_valid_topdown_candidate():
    fixture = json.loads(FIXTURE.read_text())
    result = generate_tabletop_proposals_from_fixture(fixture)
    assert result.ok
    best = result.proposals[0]
    assert best.required_open_width_m < 0.050
    assert np.dot(best.insertion_axis_base,
                  fixture['support_normal_base']) < -0.99
    assert abs(np.dot(best.jaw_axis_base,
                      fixture['support_normal_base'])) < 0.01


def test_production_generator_is_bounded():
    fixture = load_fixture()
    result = generate_tabletop_proposals_from_fixture(fixture)
    assert len(result.proposals) <= 8
```

The benchmark tool runs 100 iterations after 10 warmups with `time.perf_counter`, prints median and p95 milliseconds, and exits non-zero if median exceeds 30 ms on the ROS host.

- [ ] **Step 3: Run focused fixture and benchmark checks**

```bash
source devel/setup.bash
python3 -m pytest \
  src/alicia_flexible_grasp_supervisor/tests/test_tabletop_geometry_realsense_fixture.py -q
python3 src/alicia_flexible_grasp_supervisor/tools/benchmark_tabletop_geometry.py \
  --fixture src/alicia_flexible_grasp_supervisor/tests/fixtures/carton_tabletop_cloud.json \
  --iterations 100 --max-median-ms 30
```

Expected: fixture test passes, candidate count is at most eight, and median is at most 30 ms.

- [ ] **Step 4: Run the complete automated regression set**

```bash
source devel/setup.bash
python3 -m pytest src/alicia_flexible_grasp_supervisor/tests -q
```

Expected: all package tests pass. Record the exact passed count in the implementation handoff.

- [ ] **Step 5: Rebuild and verify generated/install-space code**

```bash
catkin_make --pkg alicia_flexible_grasp_supervisor
source devel/setup.bash
python3 -m pytest \
  src/alicia_flexible_grasp_supervisor/tests/test_rich_plan_candidate_source.py \
  src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py -q
```

Expected: build and post-build tests pass.

- [ ] **Step 6: Run live Preview acceptance without physical execution**

With ROS nodes and the combined WSL v3 service running, start only continuous 6D Preview:

```bash
source devel/setup.bash
preview_cleanup() {
  rosservice call /grasp_6d/request_plan "trigger: false" >/dev/null
}
trap preview_cleanup EXIT INT TERM
rosservice call /grasp_6d/request_plan "trigger: true"
preview_status=0
timeout 30s rostopic echo /grasp_6d/pipeline_metrics || preview_status=$?
if [ "$preview_status" -ne 0 ] && [ "$preview_status" -ne 124 ]; then
  exit "$preview_status"
fi
rosservice call /grasp_6d/request_plan "trigger: false"
trap - EXIT INT TERM
```

Acceptance requires three stable observations and an audit row with:

```text
candidate_source=tabletop_geometry
required_open_width_m < 0.050
insertion_dot_support_normal < -0.99
abs(jaw_dot_support_normal) < 0.01
analytical_result.ok = true
strict_reachability.ok = true
MuJoCo result = accepted
```

Do not call `/grasp/start`, `/grasp/stop`, torque, joint, Cartesian, or execution services during this step.

- [ ] **Step 7: Update the runbook and commit final verification assets**

Document hybrid source counts, failure codes, fixture/benchmark commands, Preview evidence, and the rule that physical execution remains explicit.

```bash
git add src/alicia_flexible_grasp_supervisor/tests/fixtures/carton_tabletop_cloud.json \
        src/alicia_flexible_grasp_supervisor/tests/test_tabletop_geometry_realsense_fixture.py \
        src/alicia_flexible_grasp_supervisor/tools/benchmark_tabletop_geometry.py \
        src/alicia_flexible_grasp_supervisor/docs/remote_grasp6d_wsl2.md
git commit -m "test: verify hybrid tabletop grasp preview"
```

---

## Specification Traceability

- Category independence and unknown-mask reuse: Tasks 1, 4, 5, and 7; tests never pass `carton` into generation and MuJoCo schema v3 uses `obb_box`.
- Support frame, 15-degree sampling, 2-degree deduplication, bilateral contact, and narrow-side width: Task 1 focused numeric tests.
- Explicit top-down tool0, 3 mm support clearance, palm/finger CAD, and two wrist variants: Task 2 analytical tests.
- Eight-candidate bound, source-local ordering, source-neutral ranking, cross-source deduplication, and lineage: Tasks 2 and 3 hybrid tests.
- Same mandatory safety, swept collision, MoveIt, three-hit stability, MuJoCo, and invalidation rules: Tasks 3 through 5 integration tests.
- Optional source-specific model fields and integrity-bound source provenance: Task 4 message, digest, task-node, and MuJoCo protocol tests.
- Bounded source audit, non-colliding lineage keys, per-source funnel, and unchanged GUI authority: Task 6 audit/GUI tests.
- Real RealSense evidence, CPU timing, full regression, and Preview-only acceptance: Task 7 fixture, benchmark, build, and live audit steps.

---

## Completion Evidence

The implementation is complete only when all seven task commits exist and the final handoff includes:

- focused unit and integration test outputs;
- full package test passed count;
- successful catkin message/build output;
- RealSense fixture audit SHA-256;
- generator median and p95 timing;
- live Preview audit SHA-256;
- selected source and required open width;
- successful MuJoCo schema-v3 source/optional-width round trip;
- confirmation that no physical execution, stop, or torque command was issued during Preview acceptance.
