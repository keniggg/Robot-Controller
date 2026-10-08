# Review-only four-body offset MJCF

Not deployed, not an exact replacement for the candidate URDF. See /media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/grasp_repair_resume_20260928T120606Z/expanded_metric_mjcf_review_20260929/report.json for 1114-q comparison and the pre-existing TCP/model/limit mismatch. Only Joint2–5 body quaternions change; raw q, physics, meshes, gripper ranges, friction and force limits remain original.

XML meshdir intentionally retains original deployment layout. The generator resolves the existing source asset directory only in memory for local compilation.

Status clarification: NOT_FOR_DEPLOYMENT means this review-only artifact has not been prospectively accepted or deployed. It is not a newly imposed 1e-12 runtime gate or a permanent numerical rejection. The approximately 0.505 mm TCP and micrometre aligned differences, and joint-limit differences, already exist in the original model; see REVIEW.md and old_new_delta_and_domain.json for deltas.
