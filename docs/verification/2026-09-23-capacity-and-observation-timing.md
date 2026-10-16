# Capacity recovery and unknown observation timing — 2026-09-23

## Public code backup

The operator explicitly confirmed public publication to `keniggg/Robot-Controller` after approval review requested that clarification. The push completed and `git ls-remote` matched all three refs:

- `backup/full-code-20260923-0835`: `fd3f9b158eda2718abe81a2ce08cb296afd80544` — complete active source, tests, documentation and compact fixtures; includes the root workspace's untracked mode-design document.
- `backup/root-workspace-20260923`: `5900419e7ad1061a854d46d7e7fc95a826926041`.
- `backup/carton-workspace-20260923`: `72c1d5f70c9b0af396a34a8c14eb48fcd24414cd`.

Root-workspace outstanding modifications were generated build/devel/cache files, not omitted source changes. Recording bags and isolated runtimes remain excluded from Git. Subsequent fixes are on `fix/unknown-motion-timing-20260923`.

## Ubuntu capacity

The whole root filesystem was inventoried. Major retained usage included ROS experiment recordings, Git history, other worktrees, installed system/ROS packages, calibration and model data. No Git history or worktree was removed.

- Fourteen closed older ROS text logs were losslessly compressed, streaming SHA-256/readback verified before removing the original text files: 728,958,776 bytes reclaimed.
- Verified duplicate drag-and-drop files and an installed-version package installer: 419,963,244 bytes reclaimed. Unique cached videos, models and CAD were preserved.
- Nineteen installed-version extension installer archives plus the already installed VS Code `.deb`: 869,257,327 bytes reclaimed. Installed extensions, settings, user history and active C++ index were preserved.
- Disposable comparison runtimes created during this diagnosis were removed after saving results/scripts: 344,273,716 bytes. Production MuJoCo 3.2.3 and D405 SDK 2.56.5 remain unchanged.
- The archive worker independently completed and reverified records `052129...b0c711...` and `053213...6b0244...`, then removed only their registered recording data under the existing approved retention rule. All summaries and download locations remain.

Free space recovered from approximately 1.6 GiB to 6.32 GiB before the next recording. Recording subsequently consumes space again; use live `df`/archive disk status, not this historical peak, for admission. Latest five complete records remain local regardless of result; active/unverified records are additionally retained. Disabled older Snap revisions were inventoried but remain installed because administrative authentication was unavailable.

Exact per-file audit plans, hashes and results are in the parent workspace `.ros_log/ros_live_20260922_session/capacity_recovery/`. No recording duration/rate/resolution/topic was changed.

## Observation motion change

Increasing MoveIt's scaling alone is insufficient: controller quintic interpolation can have substantially higher continuous speeds than its waypoint velocities. A historical trial with scalings 0.30/0.20 had a continuous velocity bound of 0.114 rad/s before the existing 0.08 rad/s guard stretched it.

The unknown-object observation profile now proposes C2 quartic acceleration/deceleration ramps only for strictly collinear, monotone joint paths with at least three points. Every original position and its order remain exactly unchanged. Intermediate cruise waypoints have constant velocity; both end velocities and accelerations are zero. Non-collinear/reversing paths use the existing retiming route. Minimum duration, all joint limits, continuous controller-reference checks, measured tracking, CAD, collision and execution gates remain authoritative.

The profile affects only the separate observation planner. Contact/approach/lift limits are unchanged. Carton restores the observation planner's exact configured baseline. A changed mode generation or timing preference invalidates the cached trajectory; execution rechecks the current timing key before submission.

Exact historical generation-11 commanded path comparison:

| Metric | Recorded command | New proposal |
| --- | ---: | ---: |
| Duration | 6.142016425 s | 5.268348187 s |
| Continuous peak velocity upper bound | 0.0740274 rad/s | 0.0760000 rad/s |
| Continuous peak acceleration upper bound | 0.1888075 rad/s² | 0.0865546 rad/s² |

This is a 14.2% duration reduction and about 54% lower acceleration bound **for this recorded path**, not a measured physical grasp improvement. The complete interpolants were checked for forward progress, not just sampled waypoint velocities. Evidence: `contact_repair/smooth_observation_timing_comparison.json` and the small committed trajectory fixture.

Validation: 276 relevant timing/gateway/controller-reference/observation/MoveIt regression tests passed. The original `/motion_gateway` was reloaded through its existing respawning launch, without restarting the arm driver or publishing torque-off. An initially extra gateway unit was removed before any task execution; only the original gateway provides the services.

## New attempt and remaining blockers

Record `20260923T091030.224365Z_d5d78f77388d44d894f2dc9b383d9b4d`, unknown/two-stage generation 13, did not reach `/grasp/start`. Its handoff could not establish a fresh target for the operator's mass annotation within the existing window. Do not report a physical contact failure or success from that attempt. Its original 600-second recording is left to close normally.

Fresh same-frame RGB-D reproduces `support_plane_inlier_ratio_low`. The original 70–170 px support ring has about 60.4% measured inliers within the unchanged 2 mm band, below the 70% requirement. All tested smaller/alternate rings stayed around 59–61%; therefore sampling radius alone is not demonstrated to fix it. These diagnostics retained original data and did not lower production thresholds. The operator agreed to adjust the view more squarely toward the table within 7–50 cm and provide a new alignment confirmation. No further physical attempt should start before that confirmation and the current recorder's normal closure.

Other unresolved evidence remains: generation 11's cross-view local support separation was 5.607 mm, and direct candidates lost simulated contact during lift. Isolated MuJoCo 3.3.7/3.6.0 and native multi-contact comparisons still failed the unchanged lift requirement; production physics remains 3.2.3. No numerical/friction/force tuning from these comparisons was deployed. No successful physical grasp has been established.
