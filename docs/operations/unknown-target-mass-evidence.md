# Task-scoped mass estimate for an unknown target

RGB-D does not measure mass or friction. Default unknown-object simulation uses
the existing live-OBB density envelope. When an operator provides a mass estimate
for the current object, a temporary `/grasp_mode/operator_mass_estimate` record
can bind that statement to one unknown-mode selection and one immutable
perception anchor. It is not a category-wide default.

The record contains the exact selection (mode, strategy, generation, source
cutoff), anchor source timestamp, estimated kilograms, original operator
statement, source `operator_estimate`, report time and expiry. Nanoseconds use
canonical decimal strings for ROS XML-RPC. Maximum lifetime is twenty minutes.
The current integration is the direct unknown-object candidate check and the
independent task-side simulation gate. No GUI mass control is added here.

Before each payload is built, the boundary checks the current selection, a ready
perception observation at or after the plan snapshot, unchanged anchor, target
lock and expiry. The existing plan/source-age gates continue to govern physical
execution; mass does not introduce a second, shorter camera-frame-age limit.
The copied evidence is then bound to the exact plan ID, target track ID and
snapshot timestamp, with a SHA-256 of the original evidence. A different
selection ignores the old annotation and uses default dynamics; a matching
selection with invalid target evidence fails the simulation request.

Simulation uses `max(existing_mass_floor, 2 * operator_estimate)` and rejects an
estimate whose doubled mass exceeds the existing operating ceiling. For the
operator's statement “重量为10g左右”, the simulated mass is 20 g. This is explicitly
an estimate with a chosen uncertainty allowance, **not a measured upper bound**.
The original density-envelope mass is retained in the request audit. Friction,
gripper force/geometry, contact/lift acceptance, timestamps and motion gates are
unchanged. Candidate and execution gates each validate the evidence independently.

The session-specific recorded handoff accepts `GRASP_OPERATOR_MASS_KG=0.01`,
waits for a fresh target after its normal mode reset, and saves the evidence in
`authorization.json` before starting the runner. That environment variable is
set only on the explicitly authorized transient attempt. New ordinary attempts
do not receive it. Never set a mass estimate merely from appearance or a target
class. A 20 g simulation result does not demonstrate successful lifting of an
object whose true mass lies outside the declared estimate assumption.
