The near-field attempt rejected the controller handoff because the launch overlay
set `strict_execution_controller_sync_timeout_sec` to 4.0 seconds while the
physical-reference transaction requires `0.05 <= duration < timeout <= 3.0`.
The overlay now retains the original 2.0-second timeout and 0.30-second duration.
The separate observation search budget remains 5 seconds. No object dimension,
collision margin, feedback freshness limit, or physical transaction guard changed.

New `tools/full_stack_start.py plan` contracts use `manual_full_stack_restart`.
They preserve source, configuration, build, storage, and boot bindings, then start
the existing simulation and full-stack entries without a cold scan or a pre-start
remote health request. Post-start acceptance still checks the actual processes,
GUI window, measured feedback, synchronized RGB-D, controllers, and the remote
service protocol. It also reads back the loaded controller timeout and duration.

The `inherited_full_plus_targeted` release format keeps the recorded full run and
the current targeted run separate. It verifies their exact source delta and case
inventories, including an independent evidence review. Typed startup identity
schema 6 retains the pinned prior hardware/interface assets and admits only the
reviewed native/typed verifier changes. Its original state checks, QUERY exchange,
freshness window, and runtime acceptance remain in use. The ordinary recorded
grasp entry accepts the new native contract without a historical launcher overlay.

Validation:

- Current isolated run `physical_reference_timing_restart_target_20261104_r4`:
  10 files, 838 passed, no skips or failures, 8 workers. Result SHA256:
  `332a11faf1dff754a50ed74103b52dd81423abcf2b9d664ebfd17b214d2e8663`.
- Historical full run `integrated_full_regression_r27_support_proof_20261103_r1`:
  713 files, 20,303 passed and 33 recorded skips. This is inherited evidence;
  a new complete regression is not claimed.
- The preceding targeted run remains recorded as 1,262 passed and 80 failed.
  Independent review classified 78 as strict source-identity refusals in frozen
  historical fixtures. Two new timing assertions omitted an existing 2 ms
  provider wait and were corrected to observe the real acknowledgement time.
  Historical tests and production identity checks were not weakened, and those
  failures were not relabelled as passes.
- Real release preparation after the 831-case r3 run exposed a declaration
  mismatch: targeted metadata omits unselected historical explicit files.
  The verifier now uses the validated baseline declarations separately, while
  checking the actual targeted files and cases against their own recorded scope.
  The original r3 result and preparation failure remain unchanged.

The user requested repair and code synchronization first, then a full manual
startup, followed by their own arm alignment and a new grasp instruction. Startup
does not grant automatic grasp authority. A new driver may correctly remain
unconfirmed until actual manual feedback establishes readiness; do not manufacture
confirmation or move the arm to satisfy the verifier. Physical grasp and final
descent success remain unverified at this source publication. Runtime evidence
and experiment records stay on the selected SSD with the local-only policy.
