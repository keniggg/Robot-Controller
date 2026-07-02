from copy import deepcopy
import math


def make_pregrasp_pose(object_pose, pregrasp_distance=0.08, camera_pose=None, mode='base_z'):
    pose = deepcopy(object_pose)
    distance = float(pregrasp_distance)
    if str(mode or '').lower() in ('camera_ray', 'line_of_sight') and camera_pose is not None:
        obj = pose.pose.position
        cam = camera_pose.pose.position
        vx = float(obj.x) - float(cam.x)
        vy = float(obj.y) - float(cam.y)
        vz = float(obj.z) - float(cam.z)
        length = math.sqrt(vx * vx + vy * vy + vz * vz)
        if length > 1e-6:
            scale = distance / length
            obj.x -= vx * scale
            obj.y -= vy * scale
            obj.z -= vz * scale
            return pose

    # Fallback: approach from above in base frame when camera pose is unavailable.
    pose.pose.position.z += distance
    return pose

def make_lift_pose(current_pose, lift_height=0.05):
    pose = deepcopy(current_pose)
    pose.pose.position.z += float(lift_height)
    return pose
