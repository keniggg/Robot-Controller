# Unknown near-field contact probe reuse

Near-field candidate generation previously rebuilt the entire CAD materialization for every scalar angle tested by the four-branch, fourteen-iteration contact boundary search. This work can consume the source frame's inference validity window before strict pose checks begin.

The unknown near-field path now constructs request-local finger corners and validates the unchanged materializer inputs once. Each scalar probe uses the original semantic-axis rotation, support-clearance geometry and contact overlap calculation. Every final candidate still goes through the full CAD materializer. Carton and far-field generation use their existing path, as do alternative tool-axis conventions. Search iterations, contact thresholds, freshness limits, joint-turn limits and execution timing are unchanged.

Validation: 559 relevant tests passed, including arbitrary rigid-frame pose/overlap equivalence, both jaw branches and tilt polarities, bisection boundaries, invalid geometry, and mode isolation. An offline 32-candidate fixture produced identical final transforms and ordering; its measured median generation time fell about 40% in a live ROS environment. Timing varied with system load and is not a real-grasp success claim. The next physical task still needs to pass current perception, strict motion planning and contact/lift simulation.
