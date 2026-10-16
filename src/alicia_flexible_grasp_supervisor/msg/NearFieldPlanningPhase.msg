std_msgs/Header header
bool active
uint64 phase_id
time deadline
bool reference_center_valid
string reference_label
string reference_target_track_id
geometry_msgs/Point reference_center_base
time reference_source_stamp
# Contact replanning uses the current observation. Final refinement also
# carries the exact already-bound contact plan; it grants no execution rights.
string planning_stage
alicia_flexible_grasp_supervisor/Grasp6DPlan reference_contact_plan
