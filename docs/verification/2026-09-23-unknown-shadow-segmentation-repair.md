# Unknown-object shadow segmentation repair

Scope: current-frame instance ownership for unknown objects. Carton perception,
raw RGB/depth, camera capture/filter/publication settings, arm controls, grasp
contact/collision thresholds and recording settings are unchanged.

## Cause

After the independent D405 SDK intrinsics repair, a recorded static scene still
has a 3.669 mm local depth change (support-plane contribution -0.141 mm). The
original RGB refinement made the entire eroded depth proposal definite GrabCut
foreground. A raised shadow patch therefore could not be removed by colour.
Tracked refinement failures also silently fell back to publishing the original
depth proposal as a detected object.

## Implementation

`measured_mask_seeds.py` makes only the upper measured interior and independently
measured planar side surfaces definite foreground. Side normals use current
5x5 covariance, at least 21 valid samples, <=0.5 mm planar residual, <=0.04
normal variance ratio, <=8 mm local depth range, nondegenerate tangential spread,
and >=45 degrees to the support normal. No image coordinates, semantic class,
past mask, hole filling or completed depth are used as evidence.

Measured support pixels within the existing 2 mm band become definite background
only when the upper object surface clears the existing 4 mm foreground threshold
by the full support uncertainty band (another 4 mm). Low-profile objects otherwise
suffer false background labels on real dark sides; those pixels remain probable
labels for RGB to resolve. Raw depth is never replaced.

The full noisy proposal remains probable foreground. The 85% retention condition
now applies to the trusted core rather than forcing 85% of the contaminated full
proposal into the result. Full proposal retention remains an explicit diagnostic.
The unique connected instance/core, search-boundary and expansion checks remain.
Downstream three-frame mask IoU >=0.85, measured points, missing-depth, target
identity, contact evidence and collision gates are unchanged.

A tracked refinement failure now publishes detected=false with an empty mask for
that exact source frame, while retaining target identity for a fresh-frame retry.
It never authorizes the contaminated depth-only proposal as a fallback.

## Evidence

- Three original corrected-SDK frames: the independently inspected shadow patch
  contains 0/410/45 foreground pixels before and 0/8/1 after; nearby table patch
  contains 88/114/86 before and 0/5/0 after. The dark object face retains
  477/477/476 of 477 reference pixels.
- A separate current small-object sequence: all 24 frames refine successfully;
  consecutive mask IoU minimum 0.880, median 0.940. The old algorithm rejects
  three of these frames at the search boundary. This is segmentation stability,
  not a claim of independently verified hand-eye accuracy or grasp success.
- Original eight-frame low-profile object replay passes its original snapshot
  fusion requirement (at least four accepted three-frame windows), without
  reducing the IoU threshold or supplying missing depth.
- 52 tests passed across RGB refinement, stream rejection/recovery, tabletop
  geometry and support-plane contracts. Six warnings are existing cv_bridge
  `tostring()` deprecations.
- A 378,993-byte cropped regression fixture preserves measured source data and
  source file SHA-256 values. Evaluation patches are test-only references and
  do not define algorithm seeds.

Session evidence:
`/home/zhuyupei/alicia_wa_full/.ros_log/ros_live_20260922_session/shadow_repair/`.
`evaluation.json` contains replay metrics and timing. Note that the fixed paper-box
pixel patches in its held-out rows refer to a different current object and must
not be interpreted as its object annotation; only its actual masks/IoU apply.
`deployed_status.json` records live post-reload observations.

Only `/unknown_tabletop_perception` is reloaded. Its temporary user unit is
`alicia-unknown-shadow-repair-20260923.service`, PartOf the existing live ROS unit.
Future full launches use the changed source normally. No torque-disable command
is issued by this repair.

## Authorized live attempt

The deployed node produced 48 consecutive ready observations after the initial
three-frame acquisition (generation 4). The recorded handoff then selected a
fresh generation 5, retaining unknown/direct mode and the existing controller.

Record ID: `20260923T050636.660058Z_c75328beb6b2418495725e66c98e994e`.
The exact source snapshot used three measured frames spanning 1.400 s and passed
the unchanged fusion requirements. All 32 tabletop proposals passed local
geometry gates, giving 46 unique orientation/sequence candidates. The planner
checked seven: two failed strict MoveIt and five failed MuJoCo contact/lift.
Two simulated closures never established simultaneous bilateral contact; three
established contact but lost it during lift, with zero object lift. The phase
ended with `NEAR_FIELD_DIRECT_TIMEOUT`. The remaining untested candidates are
unproven, not classified as impossible. No `/grasp/start` call or physical grasp
was made; the actual task result is failure, not success.

The configured category-independent dynamics envelope derives 0.100537 kg from
the observed 21.064 x 17.005 x 14.034 mm OBB, using density 20,000 kg/m^3 and
friction lower bound 0.10. These are assumptions, not a measured object mass or
material. Simulated settled normal force was approximately 1.5 N per finger.
This is a separate contact/dynamics blocker; neither changing calibration nor
claiming segmentation success establishes a successful lift. The user was asked
for object identity and measured weight/confirmed upper bound. No mass, friction,
force, collision, contact, path or time-limit parameter was relaxed.

`planning_failure_diagnosis.json` saves the actual seven checked results, source
stamps, geometry and dynamics assumptions. The original 600 s recorder remains
responsible for closing the bag and deriving the manifest from runner evidence;
the independent archive queue subsequently uploads and verifies it. Completion
and archive state are recorded below after file closure.


## Operator weight update and retry

The user subsequently reported “重量为10g左右”. A target-scoped operator evidence
path was implemented (see `docs/operations/unknown-target-mass-evidence.md`),
with 208 relevant tests passing. The second independent recorded attempt is
`20260923T052129.597176Z_b0c7112511b4405389551821ce275c72`, selection generation 6.
Its authorization includes the original estimate, anchor timestamp and expiry.
The simulated mass is 20 g with the original friction and all execution gates.
Only the idle planner/task nodes were reloaded; driver, motion gateway, camera
and joint enable were preserved. The first recording closed naturally at
2026-09-23T05:16:49.447157Z, result failure, and entered the archive queue.


The mass-informed retry also ended in planning timeout before `/grasp/start`.
Five candidate simulations actually used 20 g: two failed MuJoCo IK, two lost
opposed contact during preload, and one never established bilateral contact.
Five additional candidates were rejected before network simulation by a newly
introduced one-second mass/status age check. A separate 31-message probe found
one valid ready observation with 1.115 s source age and 0.681 s processing time,
while the same target remained locked and not lost. That additional mass-only
age gate was incorrect: mass depends on target identity, and the existing plan
lifetime already governs motion. It now requires an unchanged ready anchor
observed at or after the frozen plan snapshot. Original camera, plan and task
freshness gates are unchanged. The targeted 79-test suite passes, including
pre-snapshot observation rejection and planning-latency regression. Planner
and idle task were reloaded with this timing fix before the next retry.


## Final automatic attempt with corrected mass binding

Record `20260923T053213.276238Z_6b02449eb5a646489ca8f4dca1af0729`, unknown/direct,
generation 7, was started only after the previous recording closed. All twelve
allowed MuJoCo candidate attempts used 20 g and none failed the operator identity
binding. Ten failed contact/closure/preload validation and two failed MuJoCo IK;
no valid execution plan was selected before `NEAR_FIELD_DIRECT_TIMEOUT`, and no `/grasp/start` call was
made. Thus the requested automatic physical grasp is **not completed**. This is
not a success inferred from a normal process or recording exit. Detailed counts
and actual reasons are in `shadow_repair/mass_timing_retry_result.json` and the
independent record's original gate audits. No further identical retries are
scheduled; contact-pose/digital-twin diagnosis remains necessary before physical
execution. The 600 s recording and asynchronous archive continue under their
existing persistent service lifecycle; joint drivers and read-only monitoring
remain running.
