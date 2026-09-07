# Known-mask class-agnostic precontact verification

Status: **offline implementation/review complete through Task 5; powered
deployment and real-grasp acceptance not performed**.

Plan: `docs/superpowers/plans/2026-09-03-class-agnostic-precontact-geometry.md`.
Spec: `docs/superpowers/specs/2026-09-03-known-to-unknown-class-agnostic-grasp-design.md`.
The operator closed WSL and all hardware interfaces. All evidence below is
offline; no ROS master, runtime node, camera, serial connection, or hardware
command was started. Historical alignment messages do not authorize resuming
motion while these interfaces remain closed.

## Scope and accepted architecture

The trained carton segmentation model supplies only the current mask and
diagnostic label. The required downstream target identity is geometric and
opaque; it is independent of label and model filename. Completion also requires
measured multiview surfaces, measured bilateral jaw support, bounded 3D
registration, and at most one strictly planned clear-view reacquisition.
Those later tasks are not satisfied by the identity changes alone.

## 2026-09-04 Task 1 partial verification — chronological record

- Added immutable `TargetTrackIdentity` / `TargetObservation` contracts and
  target-track/refinement fields to rich-plan messages. Canonical plan bytes
  change from V1 to V2 to bind the new fields, with ROS float32 quantization.
- Task-node integrated regression command:
  `source /opt/ros/noetic/setup.bash && source devel/setup.bash && python3 -m unittest src.alicia_flexible_grasp_supervisor.tests.test_grasp_task_sequence -q`
  Result: **164/164 passed**. Log: `/tmp/precontact-task1-root-i4iR88/sequence.log`.
  Tests exercise diagnostic label/model invariance in admission, drift and
  refinement; changed-track revocation; frozen-plan preservation; out-of-order
  geometry handling; and near-field reference-track propagation.
- Remote-node integrated regression command:
  `source /opt/ros/noetic/setup.bash && source devel/setup.bash && python3 -m unittest src.alicia_flexible_grasp_supervisor.tests.test_remote_grasp6d_node -q`
  Result: **156 passed, 1 failed**. Log: `/tmp/precontact-task1-root-i4iR88/remote-node.log`.
  Remaining test at that checkpoint: `test_near_field_geometry_uses_far_view_planar_center_anchor`;
  its reached-view fixture lacks the new reference track. This is an open
  failure, not a waived gate.
- Phase transition must invalidate outstanding inference requests without
  advancing the geometric target epoch. Stream restart and association loss
  must invalidate the target identity. Combined propagation tests and the
  independent task review are still pending.
- Existing working-tree changes are preserved. These counts describe the
  integrated dirty worktree, not a clean Task 1 commit. Do not combine them
  with endpoint-plan counts or present them as full-suite acceptance.

### Latest repository-preserving continuation

Read-only git status/diff checks covered both the main `master` checkout and
the existing `codex/protocol-v3-upgrade` worktree before edits. Main's existing
59 tracked changes (430 insertions / 304 deletions), including generated files,
were unchanged at the follow-up check. Both indexes remained empty. No pull,
reset, clean, rollback, branch switch, or copy-back was performed.

The remote fixture migration exposed an actual `NameError` in successful
near-field anchor auditing: removed `anchor_label` was still referenced. The
audit now records `target_track_id: anchor_track`; the test asserts that field.
Both remote test files pass **395/395** after this fix.

The combined nine-file Task 1 pytest run passes **916/916**, with seven existing
ROS Noetic `warn` deprecation warnings. Command: the nine files listed under
Task 1 (including `test_target_observation`, snapshot, stability, pipeline,
hybrid candidates, integrity, both remote files, and task sequence), using
`python3 -m pytest -q` after sourcing ROS Noetic and this worktree's devel setup.
Log: `/tmp/precontact-task1-root-i4iR88/focused-combined.log`.
Offline catkin build/message generation also exited 0; log:
`/tmp/precontact-task1-root-i4iR88/catkin-build.log`.

These later results supersede the earlier fixture-failure counts, not the
open integration/review gates. The previous three task-side findings have
candidate fixes and regressions; scoped independent re-review remains open.
A further lifecycle issue is recorded: direct near-field success stops the
inference stream, then final refinement starts it again, advancing target
identity. The same-task lifecycle needs correction without allowing a new
target to inherit the old plan. **Task 1 is not yet accepted or committed.**

The review regression for same-track hard-invalid geometry was subsequently
observed RED and repaired. `TF_UNAVAILABLE` now revokes immediately under
expected occlusion; the narrow TARGET_LOST exception remains. Fresh evidence:
the focused test passes 1/1, the task sequence passes 168/168, and independent
boundary re-review passes 2/2 with Spec/Quality PASS for this finding. This
does not close the stop/start lifecycle finding.

One complete supervisor pytest checkpoint produced **1808 passed, 3 skipped,
18 failed, 7 warnings**. Seventeen failures originate in the GUI rich-plan
fixture, which still emits the pre-Task-1 message shape without target-track
and refinement fields. One separate failure is the retained pre-existing
tabletop configuration/test mismatch (`angle_step_deg` is 5.0 in the dirty
configuration while the test expects 15.0). These failures are recorded, not
hidden by reverting operator work.

## Remaining acceptance

Independent task-side review found two Important issues and one additional
spec gap despite the passing sequence tests: invalid TARGET_LOST geometry
overrides expected-occlusion preservation; a different-track preview can pass
optional-anchor-independent rebind; and raw/geometry timestamp, frame and
position association is incomplete. These findings are assigned for failing
regressions and fixes, then scoped re-review. They have not been waived.

Additional offline compatibility check:
`python3 -m pytest -q src/alicia_flexible_grasp_supervisor/tests/test_mujoco_digital_twin_client.py`
passed **80/80**. This file's pytest functions are not all collected by
unittest, so its earlier unittest-only two-test count was not full coverage.
Log: `/tmp/precontact-task1-root-i4iR88/mujoco-client-pytest.log`.

1. Finish Task 1 propagation tests, independent spec/quality review, and scoped
   commit verification.
2. Implement and verify multiview registration and measured bilateral support;
   remove the temporary class-specific solid-OBB authority.
3. Replace centroid refinement with plan-bound 3D evidence and one bounded
   no-contact clear-view attempt.
4. Run complete offline Python/build/driver tests and record fresh totals.
5. Powered acceptance remains unavailable until the operator restores the
   interfaces and authorizes a fresh aligned trial. There is no new evidence
   yet that real-arm speed, shaking, or full grasp completion is resolved.

## 2026-09-05 Task 1 integrated checkpoint

The identity lifecycle defect is repaired and independently re-reviewed.
Direct-near success keeps the stream and target ID; final refinement opens a
new phase/generation so old inference, previews and samples cannot be reused.
True stop/start and actual target changes still create a new ID. Cached
geometry can span elapsed motion time only when it covers the bound plan and
passes valid/track/frame/finite/position checks; the raw observation must
still be fresh.

Evidence: lifecycle TDD 4 RED then 4 GREEN; task plus remote streaming 408/408;
lifecycle re-review ADDRESSED with Spec/Quality PASS; GUI V2 tests 29/29; ten
Task 1/GUI files 948/948; complete supervisor suite **1829 passed, 3 skipped,
7 existing ROS warnings**.

The final stale benchmark assertion was updated to the existing
5-degree/1-degree/32-candidate production contract; runtime settings were not
changed. Task 1 awaits full-scope review and remains uncommitted because these
integrated files also carry preserved earlier edits. Tasks 2–5 remain open.

## 2026-09-05 Task 1 final offline acceptance

Full review initially found two real authority leaks: expected occlusion could
hide structurally contradictory same-track geometry, and model/profile names
could reject otherwise identical evidence or publish hard-invalid geometry.
Both were reproduced, fixed with TDD, and independently re-reviewed as
ADDRESSED with no new Critical/Important issue. A remaining Minor was closed
by persisted generated-message tests for all track/refinement hash fields,
invalid states and ROS float32 wire round trips.

Fresh evidence after all fixes:

- Task 1 ten-file integration: **977 passed**, seven existing warnings;
- ROS message/package build: `catkin_make -j2`, exit 0;
- complete supervisor tests: **1866 passed, 3 skipped, 7 existing warnings**;
- whole-worktree `git diff --check`: exit 0.

This is an offline contract result only. No ROS master/node, WSL inference
service, camera, serial interface, controller, arm, gripper, motion, enable,
disable, stop or torque command was used. Multi-view measured surface fusion,
bilateral contact evidence, carton-branch deletion, structured final 3D
refinement and the one clear-view attempt are still pending in Tasks 2–5.

## 2026-09-05 Task 2 completion and Task 3 pause checkpoint

The current resumable handoff is recorded in
`src/alicia_flexible_grasp_supervisor/logs/2026-09-05-class-agnostic-precontact-handoff.md`.
That file supersedes the final sentence of the preceding chronological Task 1
entry: Task 2 has since completed offline implementation and independent review;
Task 3 is partially implemented but not complete.

Task 2 now separates strict registration correspondences from fusion
membership. Registration acceptance and metrics remain correspondence-only;
after acceptance, only real moving-view samples connected to those seeds by
measured Euclidean links at or below the exact configured radius enter the
fused surface. The real shared-top/opposing-side producer regression retains
the newly visible face with immutable provenance. A signed-zero spatial-hash
boundary defect was found in independent review and fixed by a complete,
chunked radius-graph traversal. Final Task 2 evidence: system NumPy 1.17.4
**68 passed**, three-file integration **343 passed**, and 232 independent graph
oracle comparisons; scoped re-review returned Spec/Quality PASS with no new
Critical or Important finding.

Task 3 currently contains an unreviewed implementation of measured bilateral
surface evidence and integration changes across gripper geometry, tabletop
generation, remote contact gates, and their tests. The original implementer
stopped at a tool usage limit before writing `task-3-r1-report.md`; no independent
Task 3 review exists. A fresh controller run of the Task 3 three-file suite plus
the Task 2 multiview regression produced **480 passed, 1 failed, 14,688
warnings**. The sole failure is a new test-fixture construction error:
`np.block` receives tuple blocks in
`test_graspnet_contact_gate_uses_current_fused_surface_and_fails_closed`, so the
test does not reach its production gate assertion. This is not a production
pass or failure verdict.

The current semantic forbidden scan has no hit for
`label.*carton|carton.*label|solid_obb_carton`, `git diff --check` passes, and
the index is empty. Nevertheless Task 3 remains in progress until that fixture,
all current fused-surface contact paths, NumPy compatibility warnings, a full
RED/GREEN report, and independent spec/quality review are closed. Tasks 4–6 and
powered acceptance remain pending. No ROS, WSL, hardware, control, enable,
disable, stop, torque or motion action was performed for this checkpoint.

## 2026-09-06 Tasks 2–5 final offline acceptance

This section supersedes the earlier chronological “pending” statements above.
It does not supersede the powered-deployment prohibition.

- Task 2 fuses only measured registered RGB-D samples with immutable per-point
  view provenance. Registration acceptance remains correspondence-only; the
  fused addition is a measured connected inlier component, with exact-bound
  handling compatible with system NumPy 1.17.4. Independent review is clean.
- Task 3 requires real fused samples on both physical jaw sides within the
  support-normal/insertion contact window. Top-only surfaces, missing/current-
  phase-mismatched surfaces, or inadequate view provenance fail closed. The
  temporary semantic `carton` solid-OBB completion branch is removed;
  independent review found no Critical, Important, or Minor issue.
- Task 4 replaces 2D centroid execution authority with plan-bound `VALID_3D`
  registration evidence. Candidate source/lineage, opaque track, support,
  metrics and corrected plan hash are bound; all four motion stages are
  strictly rechecked before atomic rebind. Independent Spec/Quality review
  passed.
- Task 5 permits one class-independent clear-view detour. Two symmetric
  candidates use measured tool/camera transforms, camera optical-axis aiming,
  a positive camera/tool capsule, and Alicia palm/finger CAD envelopes. Both
  are strict-preflighted and ranked by joint-duration lower bound, path cost,
  then deterministic side; at most one executes. Corrected-pregrasp
  confirmation obtains five genuinely fresh same-bound 3D frames by advancing
  the production one-submission-per-phase gate. Independent review passed.

Fresh integrated evidence after the last Task 5 correction:

```text
Task 4/5 four-file pytest:        573 passed, 7 warnings
complete supervisor pytest:      2042 passed, 3 skipped, 8 warnings
catkin build/message generation: exit 0 (8 messages, 11 services)
supervisor unittest discovery:   769 tests, OK
driver actuation gtest:          49/49 passed, 6 suites
git diff --check:                exit 0
index:                           empty
```

The complete pytest and unittest protocol fixtures used only a temporary
localhost (`127.0.0.1`) mock HTTP server. No external network, WSL inference,
ROS graph, camera, serial device, controller, arm or gripper was started. No
enable, disable, stop, torque or motion command was sent.

Static production scans found no `carton`/`canton` label-conditioned execution
branch, no `solid_obb_carton`, no retired center-sample symbol, and no newly
added robot-actuation stop/disable/torque-off invocation in the precontact
paths. The sole generic added `.stop()` match is inference-generation
cancellation, not a controller, arm or gripper command. The known carton model
remains only the phase-one mask adapter; its diagnostic label does not grant
downstream authority.

## 2026-09-06 final cross-interface review closure

The 2042-test table above predates the final whole-branch review fix. Review
found one Important ordering problem: bounded nonzero registration could
rewrite the four published poses after the strict-check sequence was frozen.
The final path now applies measured registration first, finalizes diagnostic
and `plan_id`, rebuilds the exact corrected four poses and completes the full
strict MoveIt gate before bundle/audit commit. Failure clears stale final-check
evidence and preserves the exact MoveIt failure code.

The real nonzero-registration regression failed before implementation and
passed afterward; four-file integration is **574 passed, 7 existing warnings**.
Independent re-review found the Important **ADDRESSED**, no new
Critical/Important, and **Spec PASS / Quality PASS** after 19 focused, 284
remote-streaming and 71 task-side tests. The `RegistrationResult` direct-
constructor concern remains a deferred Minor with no production producer path.

Fresh final controller evidence is **2043 passed, 3 skipped, 8 warnings** for
the complete supervisor pytest suite; catkin generation exited 0 for 8 messages
and 11 services; linked-devel imports passed; unittest is **769/769**; driver
gtest is **49/49**; production static gates and `git diff --check` pass; the
index is empty. This closes only the offline Tasks 1–6 scope.

## Powered acceptance still required

The offline result does not prove that the real arm is faster, that the gripper
will not shake, or that a full grasp completes. WSL and all hardware interfaces
remain operator-closed. A later powered run requires restored interfaces plus
a new onsite alignment/authorization and a fresh plan; historical “已对准”
messages are not reusable. Runtime evidence must include target track, fused
views, registration metrics, both bilateral surfaces, strict MoveIt stages,
endpoint-trim/encoder response, gripper command/feedback, lift result and exact
failure state. Any missing evidence must fail before contact.
