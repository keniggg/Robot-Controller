# Hybrid GraspNet and Tabletop Geometry Candidate Design

**Date:** 2026-07-20

**Status:** Design approved; written specification awaiting review

## 1. Problem and Evidence

The current remote 6D pipeline can segment the target correctly and obtain
hundreds of GraspNet proposals, but a tabletop object can still have no valid
plan. The observed carton is physically graspable: its measured opening
requirement is below the real 50 mm gripper limit when the jaws close across
the short side.

The latest runtime evidence separates this from a width-contract problem:

- the real gripper contract is `Alicia_D_v5_6_gripper_50mm`;
- the target OBB was approximately `0.051 x 0.035 x 0.011 m`;
- GraspNet produced candidates whose analytical required opening was
  `0.044-0.049 m`;
- those width-valid candidates were rejected because a finger entered the
  support plane;
- every audited GraspNet approach direction was sideward or upward relative
  to the detected support plane rather than a usable top-down insertion.

GraspNet uses model +X as its approach axis and +Y as its jaw-closing axis.
The existing model-to-tool rotation remains valid. The missing capability is
therefore not an axis-sign workaround: the pipeline needs category-independent
tabletop candidates whose approach and jaw axes are deliberately constructed
from the observed support plane and target geometry.

## 2. Goals

1. Keep the official GraspNet proposals as one candidate source.
2. Add deterministic tabletop candidates derived only from an instance mask,
   RGB-D target cloud, support plane, snapshot transform, and physical gripper
   geometry.
3. Generate top-down candidates whose jaws preferentially close across the
   smallest valid target projection.
4. Support future unknown object instances without checking a semantic class
   name such as `carton`.
5. Apply the same physical aperture, collision, MoveIt, stability, MuJoCo, and
   rich-plan integrity requirements to every candidate source.
6. Preserve the separation between Preview generation and explicit physical
   Execution authorization.

## 3. Non-goals

- Changing the measured 50 mm physical gripper contract.
- Relaxing support-plane, finger, palm, swept-volume, MoveIt, or MuJoCo gates.
- Retraining or replacing GraspNet.
- Treating an OBB pose as an automatic authorization to move the robot.
- Synthesizing a fake GraspNet insertion depth for a geometry candidate.
- Adding category-specific carton dimensions or grasp poses.

## 4. Architecture

One immutable RGB-D planning snapshot feeds two independent producers:

```text
instance mask + depth + support plane + frozen TF
                       |
          +------------+-------------+
          |                          |
    GraspNet producer       tabletop geometry producer
          |                          |
          +------------+-------------+
                       |
          normalized planning candidates
                       |
               deduplicate and rank
                       |
       physical geometry and swept-volume gates
                       |
             strict MoveIt evaluation
                       |
             cross-frame stabilization
                       |
               MuJoCo simulation
                       |
          Preview / Execution rich plans
```

The new pure-Python `TabletopGeometryCandidateGenerator` has no ROS publisher,
service, model dependency, or motion side effect. It accepts immutable numeric
inputs and returns bounded candidate records plus generation diagnostics.

The two producers normalize into a shared planning-candidate representation.
The shared representation carries:

- `source`: `graspnet` or `tabletop_geometry`;
- source-local candidate and orientation-variant indices;
- contact-line center in base coordinates;
- explicit `T_base_tool0`;
- semantic insertion direction in base coordinates;
- jaw-closing direction in base coordinates;
- model width when the source has one;
- analytically required open width;
- score components and immutable snapshot lineage.

Source contracts remain distinct. A GraspNet candidate must retain its model
rotation, contact center, discrete depth, and model-to-tool derivation. A
tabletop geometry candidate must retain the point-cloud projection evidence,
support-plane evidence, CAD-derived tool pose, and generator configuration.
Neither source can populate missing fields by pretending to be the other.

## 5. Tabletop Geometry Candidate Generation

### 5.1 Inputs and prerequisites

Generation requires all of the following from the same frozen snapshot:

- cleaned target-only point cloud in the base frame;
- valid support-plane point and unit normal;
- valid target OBB whose positive vertical axis agrees with the support normal;
- measured gripper geometry and 50 mm maximum inner gap;
- configured 2 mm clearance on each jaw side;
- calibrated tool0, finger CAD envelopes, and finger contact band.

If any prerequisite is absent, non-finite, stale, or belongs to another
snapshot, the geometry producer returns no candidates and a stable failure
code. This failure does not invalidate a GraspNet candidate that independently
satisfies its selected input-mode contract; for example, `context_roi` may
still impose its own support-plane prerequisite.

### 5.2 Support-aligned target representation

The generator constructs a right-handed support frame:

- frame +Z is the support normal pointing away from the table;
- frame X/Y span the support plane;
- the origin is the robust target-cloud center projected onto the support
  plane.

The cleaned target points are transformed into this frame. Support-plane
points, invalid depth, statistical outliers, and mask-edge depth discontinuity
points have already been removed by the existing geometry estimator. The
generator does not use the 2D detection-box center.

### 5.3 Jaw-angle sampling

Jaw directions are undirected because swapping identical fingers does not
change the grasp. The generator evaluates directions over `[0, 180 degrees)`:

1. the minimum-projection in-plane OBB axis;
2. its orthogonal axis;
3. uniform supplemental angles at 15-degree intervals.

Duplicate angles within 2 degrees are removed before evaluation. For every
direction, the required width is:

```text
cleaned target-cloud projection span along the jaw axis
+ 2 * 0.002 m jaw clearance
```

The span uses the full cleaned target cloud, not a class dimension or the
GraspNet model width. A direction is rejected immediately when the required
opening is non-positive, non-finite, or greater than 0.050 m.

### 5.4 Contact quality

For a width-valid direction, candidate contact support is measured in two
bands near the negative and positive projection extrema. Both bands must
contain target points, and their median heights must overlap the physical
finger contact band. The score favors:

- larger remaining aperture margin;
- balanced point support on the two jaw sides;
- smaller distance from the jaw line to the robust cloud center;
- larger predicted finger-to-support clearance.

This allows unknown irregular objects while rejecting a direction supported
on only one side or by a single noisy extremum.

### 5.5 Pose construction from physical geometry

The semantic insertion direction is always `-support_normal`: from above the
object toward the support plane. The jaw axis lies in the support plane. The
remaining orthogonal axis completes a right-handed frame. A dedicated axis
mapping function converts these semantic axes into the configured Alicia tool
axes; generation does not hard-code a raw tool `+Z` or `-Z` assumption.

The contact-line center and tool0 are deliberately separate quantities. The
contact line is placed at the robust target mid-height. The tool0 translation
is then solved from the checked-in URDF/STL finger envelope so that:

- the lowest finger CAD point retains the configured 3 mm support clearance;
- the usable finger contact band overlaps the target mid-height;
- the jaw centerline crosses both sides of the target cloud;
- the palm remains on the approach side of the target.

This physical construction replaces any temptation to attach a fake GraspNet
depth to the geometry candidate.

Every accepted jaw direction creates two physically equivalent wrist variants
separated by 180 degrees about the insertion axis. Both are retained until
MoveIt evaluates their joint-space cost. After scoring, at most eight tabletop
geometry candidates are returned per snapshot.

## 6. Merge, Deduplication, and Ranking

GraspNet and tabletop candidates are merged only after their source contracts
pass. Candidates are duplicates when their contact-line centers are within
5 mm, their insertion axes are within 10 degrees, and their undirected jaw axes
are within 10 degrees. The candidate with the better common physical score is
kept while both source lineages remain in the audit record.

There is no fixed source preference. Hard gates run before soft ranking. The
GraspNet score and geometry contact-symmetry score order candidates only within
their own source before normalization; neither becomes a fabricated
cross-source confidence. The common cross-source ranking terms are:

- remaining aperture margin;
- target contact support and centering;
- finger and palm support-plane clearance;
- strict MoveIt path cost and maximum joint delta;
- cross-frame stability;
- MuJoCo result.

The bounded candidate count prevents the new source from increasing MoveIt or
MuJoCo work without limit.

## 7. Gates and Execution Boundary

Every normalized candidate passes the following ordered gates:

1. immutable snapshot and target-identity consistency;
2. source-specific candidate contract;
3. finite contact center, tool transform, and axis orthonormality;
4. required physical opening in `(0, 0.050] m`;
5. insertion from the support side and jaw axis approximately parallel to the
   support plane;
6. contact line crossing both sides of the target;
7. complete finger, palm, static, and swept-volume collision checks;
8. strict MoveIt IK, joint-limit, path, and joint-motion checks;
9. at least three consistent observations in the existing continuous
   candidate tracker;
10. MuJoCo validation using the same object geometry and plan lineage.

A generated candidate may be published on Preview topics after the applicable
preview requirements pass. It is not an Execution authorization. Only a
stable, fully gated, simulated, integrity-bound rich plan can be published on
the Execution topic, and physical execution remains an explicit user action.

No failure path publishes robot motion, stop, torque-off, or torque-on
commands. No old plan is reused after target loss, snapshot invalidation, or a
new target epoch.

## 8. Failure Codes and Audit

The geometry producer uses stable failures including:

- `TABLETOP_GEOMETRY_INPUT_INVALID`;
- `SUPPORT_PLANE_INVALID`;
- `TARGET_CLOUD_INVALID`;
- `NO_FIT_DIRECTION`;
- `CONTACT_SUPPORT_INVALID`;
- `TABLETOP_APPROACH_INVALID`;
- `TOOL0_GEOMETRY_INVALID`.

Downstream gates keep existing codes including `GRIPPER_TOO_NARROW`,
`GRIPPER_SWEEP_COLLISION`, `IK_UNREACHABLE`, `CANDIDATE_UNSTABLE`, and
`MUJOCO_REJECTED`.

The bounded audit report records, for every candidate:

- source and source lineage;
- snapshot and target epoch hashes;
- sampled jaw angle and semantic axes;
- projection extrema, required opening, and aperture margin;
- contact-band counts and symmetry score;
- explicit contact center and tool0 transform;
- each gate result and rejection reason;
- deduplication lineage;
- stability track and MuJoCo result.

The GUI summary distinguishes `GraspNet`, `tabletop geometry`, and merged
counts without changing the explicit Generate and Execute controls.

## 9. Configuration

Production defaults are category-independent:

```yaml
tabletop_geometry_candidates:
  enabled: true
  angle_step_deg: 15.0
  angle_dedup_deg: 2.0
  jaw_clearance_each_side_m: 0.002
  min_finger_support_clearance_m: 0.003
  max_candidates: 8
  merge_center_distance_m: 0.005
  merge_insertion_angle_deg: 10.0
  merge_jaw_angle_deg: 10.0
```

Startup validates these values against the fixed analytical 50 mm gripper
contract. Runtime changes are frozen into each snapshot audit and cannot alter
an in-flight plan.

## 10. Tests

### 10.1 Unit tests

- A `0.051 x 0.035 x 0.011 m` box cloud produces a short-side candidate with
  approximately `0.039 m` required opening including both clearances.
- Rotating that cloud arbitrarily in the support plane preserves the physical
  result and rotates the chosen jaw direction with it.
- A target wider than the available opening in every direction returns
  `NO_FIT_DIRECTION`.
- Sloped support planes, irregular point clouds, depth noise, sparse contact
  bands, and bounded outliers have deterministic results.
- Every generated pose satisfies an insertion dot support normal near `-1`
  and a jaw dot support normal near `0`.
- The two 180-degree wrist variants are physically equivalent but retain
  independent MoveIt lineage.
- GraspNet depth contracts and tabletop CAD contracts reject missing or
  cross-populated fields.
- Merge and deduplication retain both source lineages.
- No failed gate can create an Execution rich plan.

### 10.2 Integration tests

- Mocked MoveIt and MuJoCo accept only a fully gated candidate.
- A saved RealSense RGB-D fixture of the current carton produces at least one
  width-valid top-down candidate.
- Existing GraspNet protocol, analytical gripper, streaming pipeline,
  rich-plan integrity, GUI, and MuJoCo protocol tests continue to pass.
- The generator returns no more than eight candidates and completes within
  30 ms on the ROS host test fixture.

## 11. Runtime Acceptance

Acceptance uses Preview first. For at least three consecutive snapshots, the
audit must show a stable `source=tabletop_geometry` candidate with:

- required opening below 0.050 m;
- insertion from above the support plane;
- jaw axis parallel to the support plane;
- both fingers and the palm clear of the table;
- strict MoveIt reachability;
- successful MuJoCo validation.

Only after this evidence exists may the normal explicit Execute control use
the rich plan. Candidate generation itself never moves the physical robot.

## 12. Performance and Compatibility

The new generator operates on the already cleaned target cloud and uses only
bounded NumPy operations. It has a 30 ms CPU target and returns at most eight
candidates. GraspNet inference, WSL protocol v3, the fixed 50 mm gripper model,
and existing Preview/Execution topic authority remain compatible.
