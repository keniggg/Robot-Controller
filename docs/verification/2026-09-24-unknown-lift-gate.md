# Unknown-object lift gate switch

User request: temporarily treat simulated lift as diagnostic, with an explicit way to restore the hard gate.

`/mujoco_digital_twin/unknown_lift_gate_enabled` defaults to boolean `true` in the repository. The current session launch overrides it to `false`. It only applies to rich plans with `model_choice=unknown_tabletop`; other modes stay strict. Candidate selection and final execution both read the policy and record it. Change it between attempts; final execution always reevaluates with its current setting.

When false, WSL still runs the entire simulation. Plan identity, provenance, explicit booleans, finite JSON, full sequence IK/orientation, collisions and loaded opposed closure remain required. The deployed v1 server emits `settled_gripper_state` only after opposed preload succeeds. The client validates that structured state and completed diagnostics; it does not infer closure from diagnostic text or the aggregate `contact_success`, which also includes lift retention. The real 0.08 rad/s lift reference speed contract remains checked.

The full score includes lift/retention, so `min_score` applies only in strict mode. Diagnostic mode records the unmodified score, simulation verdict, lift evidence and `MUJOCO_LIFT_DIAGNOSTIC_ONLY`. A policy pass is not physical grasp success or a full simulation pass. No WSL physics/model/launcher changes are required.

Enable the lift hard gate again for subsequent attempts:

```bash
rosparam set /mujoco_digital_twin/unknown_lift_gate_enabled true
```

For persistence, also change the current session launch override. The WSL unified startup command remains unchanged.

Validation: 429 client/task/mode tests passed. The broader remote-node tests passed in the preceding run (583 passed, one log-text compatibility failure subsequently fixed). A captured WSL response with raw score 55 and failed lift passes only diagnostic policy; mutations covering closure, IK, collision, identity, provenance, malformed evidence, path speed and orientation remain rejected. Task integration verifies execution eligibility and truthful audit output; both direct and two-stage candidate selection are covered. Actual grasp success remains to be established from live execution.


## Expected occlusion and physical close-and-hold

The supplied video (`5b0867aa90e5b6bc60956d9d35831530.mp4`, 93.433 s) shows the approach ending before jaw closure. Recorded attempt `20260924T050313.794328Z_251a799497f646fe805846b892f6847d` reached its pregrasp, then failed with `PREGRASP_COMPENSATION_FAILED: EXECUTION_AUTHORITY_REVOKED`. The detector reported `TARGET_LOST: insufficient_support_spatial_coverage` during pregrasp. No closure or lift occurred. Direct mode had not enabled the frozen target's occlusion allowance until the approach stage; the two-stage rebind already enabled it earlier.

After a fresh execution gate and checkpoint, direct unknown contact plans now allow expected visual loss from pregrasp onward. The frozen plan and target remain unchanged. Target identity changes, confirmed drift, collisions, cancellation, and non-visual tombstones still revoke authority. A regression delivers missing detection, invalid geometry, and a TARGET_LOST tombstone at the first action and verifies preservation; a subsequent collision tombstone still revokes it.

`/grasp/unknown_lift_after_close_enabled` is a separate physical-action policy, default `true`, snapshotted at execution entry. The current session overrides it to boolean `false`. After a successful closure command it leaves the arm and jaws in place; it does not execute a lift, reopen, retreat, or require post-lift visual detection. Other model choices keep the existing lift sequence. This does not change WSL physics, full-path validation, or the simulated lift policy above.

Physical success requires fresh, valid bilateral tactile contact on at least three distinct samples spanning 0.2 seconds after closure, positive finite forces, and no slip, with tactile simulation explicitly disabled. A bounded two-second observation sends no motion. Without that evidence, terminal `HOLDING` / `GRASP_HOLD_UNVERIFIED` and service `success=false` explicitly mean the closure command was accepted but the physical grip is unconfirmed. Recording classifies this as `unknown`, not success or failure, and leaves the closure intact. Cancellation during verification still prevents success acknowledgement.

Validation: 272 task, mode-task and recording-lifecycle tests passed (seven existing rospy deprecation warnings). Tests cover full close-and-hold execution without lift calls, strict boolean configuration, other-mode compatibility, stale/one-sided/slipping/invalid tactile rejection, sustained real tactile success, simulated/missing evidence, cancellation, and preservation of the inactive HOLDING result. This is software validation, not proof of a real grasp.

The user powered off and then reported power-on and alignment. No torque-disable command was issued. The repaired task node was reloaded after the reported power-on; the new recorded direct attempt is `20260924T054836.781510Z_d43b6f78dee04bf390524ce2edb18be7`. Its physical outcome must be established from the recording and live evidence.


Latest outcome: that attempt preserved authority through visual occlusion and completed pregrasp, but failed before closure after 8 compensation steps at 6.667775 mm. The user then powered off all hardware. The subsequent offline Joint2 recovery, scope, conditional evidence and final 422-test validation are recorded in [close-and-hold recovery](2026-09-24-close-hold-recovery.md). It has not been physically executed.
