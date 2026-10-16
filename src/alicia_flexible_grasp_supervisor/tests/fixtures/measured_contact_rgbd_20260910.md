# Measured contact-resolution regression

The NPZ contains two stationary, three-frame RGB-D windows acquired during
the 2026-09-10 live investigation, before contact motion. It retains depth,
segmented target depth, mask, snapshot-time base/optical transform, bounding
box, intrinsics and integer source stamp. No RGB photographs are included.

- Source stamps: 1789028944102307081 and 1789028947010818243 ns.
- Source artifacts: workspace `.ros_log/surface_resolution_20260910/`.
- NPZ SHA256: `719e94665d21d480b996b279e291ce8cc882a287137cad7c18e0b74098d22416`.
- The recorded carton footprint was approximately 52 × 38 × 21 mm.

Using the same pixels and unchanged physical gates, 2.5 mm voxelization
produces 556 fused points and no accepted contact candidates; 1.5 mm produces
1466 fused points and six analytically accepted candidates across three jaw
directions and their symmetric variants. The regression re-extracts geometry
from these pixels using the repository's configured resolution, registers
the views, generates contact candidates and runs the analytical gripper gates.
It asserts that at least one physical candidate survives, not an exact count
or a particular voxel constant.

This fixture proves an offline perception/contact regression only. It does
not supply MoveIt reachability, live final refinement or held-object evidence.
