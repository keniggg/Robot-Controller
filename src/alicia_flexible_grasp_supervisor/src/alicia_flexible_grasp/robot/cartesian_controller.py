from copy import deepcopy
from tf.transformations import quaternion_multiply, quaternion_from_euler

class CartesianJogger:
    def __init__(self, planner):
        self.planner = planner
    def jog(self, dx=0, dy=0, dz=0, droll=0, dpitch=0, dyaw=0, execute=True):
        current = self.planner.get_current_pose()
        if current is None:
            return False, 'current pose unavailable'
        target = deepcopy(current)
        target.pose.position.x += dx
        target.pose.position.y += dy
        target.pose.position.z += dz
        q0 = [target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w]
        dq = quaternion_from_euler(droll, dpitch, dyaw)
        q = quaternion_multiply(q0, dq)
        target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w = q
        return self.planner.move_to_pose(target, execute=execute)
