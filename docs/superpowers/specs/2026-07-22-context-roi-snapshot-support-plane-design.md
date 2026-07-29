# Context ROI Same-Snapshot Support Plane Design

## Problem

The continuous v3 worker estimates valid object geometry from one frozen RGB-D
snapshot, then builds the GraspNet `context_roi` input before that geometry is
activated into the node's mutable `latest_*` cache.  The builder currently
reads `latest_support_plane_camera_point` and
`latest_support_plane_camera_normal`, so the first request after a clean node
start fails with `SUPPORT_PLANE_INVALID` even though the same snapshot already
produced a valid support plane.

The synchronous predecessor did not expose the bug because it activated the
geometry before building the GraspNet input.  Reordering the continuous worker
to publish mutable geometry before the request is accepted would weaken its
stale-request and cancellation guarantees, so that behavior must not be
restored.

## Approved Design

Derive the support-plane point and normal in ROS `camera_link` coordinates from
the current request's immutable `GeometryEstimate` and frozen
`T_base_optical`.  Pass those values explicitly to
`_build_frozen_graspnet_input`.  When explicit values are supplied, the builder
must use them and must not consult `latest_support_plane_camera_*`.

Keep cache population in `_activate_geometry` for telemetry, visualization,
and legacy callers.  Extract the existing base-to-camera plane conversion into
one helper so the request-local and activation paths use identical math.

## Failure Behavior

- A malformed or non-finite request-local plane still fails closed with
  `SUPPORT_PLANE_INVALID`.
- A non-`ros_camera_link` projection convention still fails with
  `SUPPORT_PLANE_FRAME_INVALID`.
- `masked_target` and `full_scene` behavior is unchanged.
- No fallback to stale cached geometry, another snapshot, or a relaxed plane
  threshold is permitted in the continuous request path.

## Verification

Add a regression test for a clean continuous node whose global support-plane
cache is empty.  The test supplies a valid current-snapshot estimate and
transform, proves that `context_roi` reaches the input builder/remote boundary,
and proves that stale cached plane values cannot override the request-local
plane.  Run the focused context-input and streaming tests, followed by a live
ROS Preview attempt against the already-running WSL v3 service.

Live verification must not publish `/grasp/stop`, torque-off, disable, or any
motion command.  Physical execution remains a separate explicit operator
action after Preview becomes stable and executable.
