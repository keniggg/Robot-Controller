"""Unknown-object observation preferences shared by planner and executor.

Working distance is a view preference, not contact clearance. All original
measured visibility, CAD, tracking, IK and final execution gates still apply.
Carton and current-view direct planning retain their original configuration.
"""
import math


UNKNOWN_OBSERVATION_MAX_JOINT_DELTA_RAD = math.pi / 2.0


def observation_config(config, selection):
    if (selection or {}).get('mode') != 'unknown' or (selection or {}).get('strategy') != 'two_stage':
        return config
    # A farther view leaves room for the open fingers without rotating the
    # wrist half a turn. These three distances remain in the existing camera
    # working interval; actual image and physical feasibility are evaluated.
    return dict(config,
                observation_camera_target_nominal_distance_m=0.26,
                observation_camera_target_min_distance_m=0.22,
                observation_camera_target_max_distance_m=0.30)
