# RGB-D silhouette fixture

`unknown_rgbd_silhouette.npz` contains eight consecutive measured frames from
`unknown_direct_attempt2/keyframes.bag` in the 2026-09-21 live session. Source
timestamps, actual recorded joints, source-time TF, support planes, intrinsics
and depth scale are preserved. RGB and depth outside x=240:400, y=200:340 are
set to zero (unavailable) to retain only the target and its local context.
The masks are the original published depth masks, not corrected labels.

The fixture reproduces changing depth-validity masks for the same stationary
object. Tests exercise current-frame RGB silhouette refinement and the original
snapshot fusion thresholds; missing depth is never supplied by the new code.
The full 40-frame replay and its manifest are stored in the session directory
`mask_stability_fix/rgb_snapshot_exact_replay.json`.

`shadow_jump_20260923.npz` keeps three same-scene frames from the corrected
D405 SDK 2.56.5 capture, including the independently measured 3.669 mm local
jump in frame 1. The JSON companion lists original file SHA-256 values and the
actual runtime camera profile. The saved support planes and original depth
proposals were calculated before cropping; RGB/depth outside x=220:430,
y=155:350 are unavailable. Fixed evaluation patches label known support/shadow
and dark object-side pixels only in tests; production has no scene coordinates.
