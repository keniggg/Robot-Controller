# Unknown-object contact diagnosis, 2026-09-23

Status: ongoing. No successful physical grasp has been established by these experiments.

## Reproducible source

Recorded attempt `20260923T053213.276238Z_6b02449eb5a646489ca8f4dca1af0729`, selection generation 7, original snapshot `1790141544628220319` ns. Source files and all experiment scripts/results remain in `.ros_log/ros_live_20260922_session/contact_repair/` in the parent workspace. The compact, original-timestamp payload is in `tests/fixtures/unknown_contact_20260923.json` under the supervisor package. This fixture is historical evidence only; do not refresh its timestamp or send it to an execution authority.

The user reported approximately 10 g. All principal contact replays retain the existing 20 g floor (2x operator estimate), original friction request, 5 N actuator cap, closure command, 0.8 s settle, 15 mm minimum actual object lift and opposed-contact checks. Replay's 40 mm required opening is validator-only; the same original 50 mm initial simulation opening and full-close command apply.

The isolated local MuJoCo 3.2.3 runtime exactly reproduced all 12 recorded candidates' failure reasons and all four initial IK results. This establishes a useful baseline without robot motion or network inference. These are genuine physics simulations, not mocked successful responses.

## Confirmed precision defect and correction

Initial simulation IK accepted 5 mm translation error and 0.16 rad cross-product orientation error. The target is only 14.4 mm tall. Candidate 1 stopped 4.25 mm from its requested grasp point, while lift waypoints required 0.2 mm. Unknown-tabletop initial IK now uses the existing lift tolerances (0.2 mm / 0.005 rad) and a bounded 400 iterations. Carton retains its existing initial-IK contract.

The historical fixture converges below 0.2 mm at all four poses, with actual geodesic orientation also below 0.005 rad. This allows several previously rejected candidates to retain opposed contact after preload; they still fail actual simulated lift. Precision alone is not a complete grasp fix.

Validation: supervisor `test_mujoco_digital_twin_server_protocol.py`: 123 passed, 1 skipped, using the isolated real MuJoCo runtime and sourced ROS environment. The added fixture test ran. The existing skipped test is environment-dependent.

## Remaining failure evidence

- Default replay: 10 contact failures and 2 IK failures; no physical `/grasp/start` in that source attempt.
- Tight initial IK: reachable candidates can preload but lose contact during lift; object rise remains about 0.5 mm, below the unchanged 15 mm requirement.
- Exact short-face candidates existed (track 4), but were unexamined before the live budget ended. Separate offline evaluation of tracks 2 and 4 also failed lift. Therefore ordering alone has not been demonstrated to solve the failure.
- A stationary precise short-face grasp drifts roughly 3.3 mm/s in the contact plane during a 2.4 s hold. Reducing the time step from 2 ms to 0.5 ms does not eliminate this drift.
- Independent tests of multi-point CCD, force-driven closure, 6D contacts, firmer normal contacts, and increased elliptic friction impedance have not established a complete passing grasp. None of these exploratory numerical changes is deployed or included in the production patch.
- Force telemetry shows large opposing tangential forces and support loads during the transition into lift. Further diagnosis of mesh contacts and the prescribed-motion dynamics is ongoing.
- MuJoCo's actual mixed finger/object sliding friction is 1.0 (max of finger 1.0 and requested object 0.1), and contact dimension is 3. Thus requested torsional/rolling coefficients are not active in the original model. Do not describe this as measured real-object friction or silently tune it to obtain a pass.

Official references consulted: [MuJoCo 3.2.3 slip diagnosis](https://mujoco.readthedocs.io/en/3.2.3/modeling.html#preventing-slip) and [solver impedance reference](https://mujoco.readthedocs.io/en/3.2.3/XMLreference.html#option-impratio). Impedance and multi-point CCD experiments were diagnostic, not changes to the physical pass criteria.

## Operator power cycle

During diagnosis the operator restarted arm power. Fresh measured joints changed and the old target lock became `target_lost_requires_new_generation`. Task remained IDLE. Only `/demonstration=False` (positive SDK torque_on) was sent; the resulting status was `CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE`, `motion_enabled=True`. No disable command was sent. A new alignment/target generation and fresh joint state are required before any new execution; old snapshot trajectories remain offline only.

## Deployment and new aligned attempt

The operator power cycle was followed by a new explicit alignment confirmation. A fresh unknown/direct generation 8 was created with a newly scoped 10 g estimate (20 g simulation). Attempt directory: `.grasp_records/20260923T063815.187830Z_d905bf53d780409ea747a29111456e5a`. It ended planning with `NEAR_FIELD_DIRECT_TIMEOUT`, before `/grasp/start`; the original 600 s recorder continues until normal close. New measured OBB: 26.48 x 18.95 x 13.70 mm, 334 points. Twelve candidates reached real MuJoCo: 9 contact failures, 3 IK failures. No physical grasp success.

The WSL inference endpoint remains `http://172.23.132.97:8000`. WSL SSH port 22 refused connection. Patched real MuJoCo 3.2.3 is therefore deployed separately at `http://127.0.0.1:8001` via user service `alicia-mujoco-contact-repair-20260923.service` (PartOf the active main ROS service). Only `/mujoco_digital_twin/server_url` changed; previous complete config is saved as `contact_repair/prior_mujoco_config.json` in the session. The local endpoint intentionally has no inference backend; its `/health` top-level inference readiness is false, while `digital_twin.ok` is true. No mock inference or simulation results are used. The real WSL GraspNet health remains loaded/online.

Performance sampling identified general NumPy cross-product axis dispatch in the repeated 3D IK error as a major cost. Equivalent fixed-size arithmetic preserves the original operation order. Pure IK and FK reference construction/validation use MuJoCo kinematics/COM transforms without contact-force solving; all trajectory collision checks and all plant contact/lift steps still use full dynamics. Historical track 4's entire response matches exactly, including every contact/lift evidence field. Same replay elapsed time decreased from 31.15 s to 9.57 s; wall times depend on host load. Final protocol suite: 123 passed, 1 opt-in smoke skipped; the opt-in real carton compile smoke is run separately.

The independent conservative finger-mesh slab experiment includes every clipped original CAD triangle in per-slab convex hulls. It also failed dynamic lift; it is not deployed. Other numerical/contact parameter experiments likewise remain offline only.

Next planned attempt: existing unknown/two_stage workflow, after generation 8's recorder has closed normally. The session handoff accepts `GRASP_EXECUTION_STRATEGY=two_stage` only while the active mode remains unknown. It creates a new mode generation, target identity and operator mass binding. Do not overlap recordings or shorten their duration to retry.
