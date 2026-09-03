# Known-Segmentation to Unknown-Object 6D Grasp Design

Status: approved by the operator on 2026-09-03; ready for implementation
planning and test-driven execution.

## Problem statement

The immediate objective is to complete a real 6D grasp of the known `carton`
target. The operator-trained segmentation model is allowed to provide the
target mask during this first acceptance phase. It must be the only
class-specific component. Every downstream geometry, planning, control, and
verification decision must already implement the final class-agnostic grasp
framework so the later unknown-object transition replaces only the perception
adapter.

The latest real run proved that the first observation stage and the
candidate-specific near-field pregrasp can both execute. The near-field plan
passed analytical gripper checks and the complete strict MoveIt sequence. Its
measured pregrasp endpoint residual was `0.6 mm / 0.22 deg`. Execution then
stopped before approach, close, and lift because the final visual-refinement
input was clipped at the bottom image edge. The live bbox was approximately
`(232, 364, 115, 116)` in a `640 x 480` image. The existing safety gate
correctly prohibited that clipped 2D mask centroid from translating the plan,
but the mandatory refinement timed out instead of obtaining valid geometric
evidence from another view.

The same run also exposed a control transition that can explain the observed
end-effector shake. Gripper command and feedback remained fixed at
`gripper_raw=995`; the fingers were not commanded to open and close. At the
trajectory endpoint, the joint endpoint trimmer applied two rapid corrections,
including Joint2 accumulated trim changing from `1.3 deg` to `2.3 deg`.
About `0.37 s` later the task released the precision lease and the driver
rebased to a measured hold. This can reverse an outstanding servo correction
even though the final Cartesian endpoint is already acceptable.

Finally, the temporary contact-height repair is selected by the semantic
label `carton`. That was useful to prove that incomplete close-view surface
coverage caused the earlier contact-patch failures, but it violates the final
architecture and cannot remain an execution-authority path.

## Approved operating model

The approved interpretation is:

- target-specific training, class-specific planning branches, label-based
  execution authority, and pre-authored per-object geometry are forbidden;
- the operator-trained `carton` segmentation model is allowed only for the
  current known-target bring-through phase;
- a generic pretrained GraspNet model is allowed;
- online class-agnostic RGB-D foreground extraction or generic segmentation
  is allowed for the later unknown-object phase; and
- the target selected after operator alignment is the stable geometric
  component nearest the camera optical axis.

## Goals

1. Complete the known-carton real grasp through approach, close, lift, and
   result verification without weakening existing physical safety gates.
2. Make the semantic mask source replaceable without changing downstream
   grasp behavior.
3. Remove all target-label decisions from geometry recovery, candidate
   generation, ranking, strict planning, execution, and result verification.
4. Use measured multi-view surface evidence instead of assuming that a
   recognized class fills a solid OBB.
5. Never use a clipped 2D mask centroid to retarget physical motion.
6. Eliminate endpoint-trim command reversal and large correction steps while
   retaining the measured Cartesian endpoint contract.
7. Preserve the operator's prohibition on automatic stop, torque-off,
   disable, controller-stop, emergency, and `/demonstration=true` commands.

## Non-goals

- The first phase does not replace the working `carton` segmentation model.
- The first phase does not claim unknown-object perception acceptance.
- No class-specific size, solidity, material, contact pose, or motion waypoint
  may be introduced to make the carton pass.
- The design does not raise Joint3/4/6 velocity limits or relax path,
  endpoint, collision, support, aperture, or MoveIt constraints.
- A failed or insufficient visual observation never becomes evidence that a
  grasp is safe.

## Architecture and authority boundary

The pipeline is divided by one explicit observation interface:

```text
phase 1: trained carton mask       phase 2: online unknown-object extraction
                  \                 /
                   TargetObservation adapter
                    - opaque track ID
                    - RGB-D target points
                    - support plane
                    - source timestamps / TF
                    - image-boundary visibility
                               |
                  class-agnostic multi-view fusion
                               |
          GraspNet + deterministic geometry candidates
                               |
       CAD/contact/collision gates + strict MoveIt sequence
                               |
        occlusion-aware final 3D registration / clear view
                               |
       measured endpoint control + close + lift verification
```

The adapter may retain the source label for display and diagnostics, but it
must generate an opaque geometric track identity. Code after this boundary
must not branch on the label, model filename, class index, or a list of known
objects. Tests will supply different and empty labels while holding geometry
constant and require identical downstream decisions.

## Phase 1: known-mask full-path acceptance

The existing trained carton model continues to generate the runtime mask.
The perception adapter validates timestamps, depth alignment, mask/image
dimensions, and TF, then emits only the common observation contract. Target
selection remains the operator-aligned component nearest the optical axis.

The current run is not repeated until the three downstream repairs below pass
offline tests and the relevant ROS nodes have been hot-loaded. The driver and
arm enable state are not restarted or disabled as part of that work.

Phase-one acceptance requires one fresh task to complete all of these stages:

1. duration-aware far-field observation selection and execution;
2. reached-view target reference capture;
3. class-agnostic near-field candidate generation and complete strict MoveIt
   sequence validation;
4. near-field pregrasp with measured endpoint confirmation;
5. valid precontact 3D refinement or a bounded clear-view reacquisition;
6. linear approach and grasp-pose execution;
7. plan-bound gripper close;
8. linear lift; and
9. post-lift visual contradiction check.

Every stage must retain exact plan-ID and target-track binding.

## Class-agnostic multi-view geometry

The reached far-field observation becomes the initial target surface model.
Subsequent near-field observations are registered to it in base coordinates
using robust geometric overlap, support-plane consistency, and bounded target
motion. Each point retains snapshot and view provenance. Fusion accepts only
finite, depth-consistent points and never fills an unobserved volume merely
because an OBB exists.

Contact candidates use the fused measured surfaces. A jaw direction may claim
bilateral contact only when the fused views contain sufficient opposing-side
support inside the physical finger bands. When coverage is insufficient, the
planner may request one additional collision-checked, no-contact observation
view chosen by the same hardware-duration metric used for the far-field roll
search. It may not convert missing evidence into a solid-object assumption.

The existing `carton` label condition around solid-OBB contact-height recovery
will be removed. Its audit evidence remains useful as a regression fixture,
but the expected result is governed by view coverage and geometric support,
not the fixture label.

## Precontact visual refinement

Final refinement is a 3D registration gate, not a 2D centroid controller.
Fresh partial target points are aligned to the fused target surface while the
support plane and already selected grasp family remain fixed. The result may
update only a bounded rigid target correction permitted by the existing
translation and orientation limits. The corrected pregrasp, approach, grasp,
and lift sequence must pass the complete strict MoveIt check again before any
new motion.

Image-edge clearance remains an observation-quality signal. A clipped mask is
allowed to contribute valid interior depth points to partial-cloud
registration, but its centroid cannot translate the plan. Registration must
meet explicit overlap, inlier-count, residual, support-normal, freshness, and
track-identity bounds.

If the clipped view cannot satisfy those bounds, the task does not time out in
place and does not continue contact blindly. It executes at most one
preplanned clear-view reacquisition pose outside the contact envelope, obtains
a fresh observation, rebuilds and strictly rechecks the contact plan, then
returns to the new pregrasp. Failure to obtain valid evidence after that
bounded attempt ends the task before approach.

## Endpoint-trim continuity

The driver endpoint trimmer keeps one command owner and one composed SDK target
across task-precision lease transitions.

- Each trim increment is capped by a hardware-derived step bound and may not
  be followed by another increment until fresh encoder feedback demonstrates
  the prior response and becomes stable.
- Releasing or expiring the lease while a response is outstanding enters a
  pending-release state. The last composed SDK target is preserved until the
  response settles or its bounded response deadline expires.
- The final rebase changes the internal reference and trim atomically while
  preserving the composed SDK joint target to within one SDK position quantum.
- Once the measured Cartesian endpoint contract passes, no corrective target
  reversal is allowed merely because the task changes state.
- A fresh explicit GUI command remains authoritative and clears task trim
  history through the existing command-source handoff.

These rules are independent of perception, object identity, and grasp class.

## Phase 2: unknown-object perception replacement

After phase one completes, the carton-specific adapter is replaced by online
class-agnostic target extraction. The primary path removes the observed
support plane, applies the configured workspace/range bounds, forms connected
RGB-D components, and tracks the stable component nearest the optical axis.
A generic pretrained segmentation proposal may assist the mask boundary, but
it cannot supply semantic authority or class geometry.

Phase two must emit the same `TargetObservation` contract. No downstream
planner, control, execution, or validation code is changed for the transition.
Unknown-object acceptance will cover multiple rigid tabletop shapes and will
require deterministic failure on transparent, reflective, too-wide, unstable,
or insufficiently observed targets rather than a class fallback.

## Failure behavior and audit

New failures distinguish evidence problems without causing a hardware stop:

- `TARGET_GEOMETRY_TRACK_INVALID` for inconsistent observation identity;
- `MULTIVIEW_REGISTRATION_FAILED` for insufficient or inconsistent overlap;
- `BILATERAL_SURFACE_EVIDENCE_MISSING` when contact support is unobserved;
- `CLEAR_VIEW_REACQUISITION_FAILED` after the single bounded observation move;
- `FINAL_REFINE_3D_INVALID` when the precontact correction cannot be proven;
- `ENDPOINT_TRIM_RESPONSE_TIMEOUT` when a trim response does not settle; and
- `ENDPOINT_TRIM_CONTINUITY_VIOLATION` when a lease handoff would jump the
  composed target.

Audits record source snapshots, view provenance, registration metrics,
coverage, exact candidate lineage, strict MoveIt stages, composed SDK target
before/after trim transitions, measured joint response, and final endpoint
residual. Labels are diagnostic fields only.

## Test strategy

Implementation follows test-driven development.

Geometry tests first reproduce the failed close-view fixture with randomized,
empty, and `carton` labels. Equivalent geometry must produce equivalent
candidates. Tests then cover multi-view opposing-surface recovery, insufficient
coverage failure, support-plane disagreement, stale/mixed snapshots, and no
unobserved-volume contact claims.

Refinement tests reproduce the exact bottom-clipped bbox. They assert that no
centroid motion is generated, partial 3D registration succeeds only with
sufficient evidence, and insufficient evidence requests exactly one checked
clear-view reacquisition before failing closed. Every corrected plan must
re-run the full pregrasp/approach/grasp/lift strict sequence.

Driver tests reproduce the two rapid trim updates and lease release from the
real log. They assert the per-update step cap, feedback-response serialization,
pending release, composed-command continuity within one SDK quantum, no
post-release reversal, and explicit GUI ownership transfer. Tests also assert
that none of these paths writes torque-off or disable frames.

Integration tests bind a fresh known-mask observation and require the full
stage order through lift. A second fixture swaps the perception adapter to an
unknown/empty label without changing downstream results. Complete Python,
C++ driver, configuration, build, and whitespace verification run before
powered validation.

## Deployment sequence

1. Implement and verify endpoint-trim continuity offline.
2. Replace label-gated OBB recovery with multi-view measured-surface evidence.
3. Replace centroid final refinement with 3D registration and bounded
   clear-view reacquisition.
4. Update task/remote interfaces and audits without changing the phase-one
   carton mask source.
5. Hot-load only the affected inactive nodes; do not stop or disable the arm.
6. Run one fresh known-carton grasp from operator alignment through lift while
   monitoring commands, feedback, plan state, and visual evidence.
7. After phase-one acceptance, implement the unknown-object perception adapter
   and repeat the same downstream acceptance suite unchanged.
