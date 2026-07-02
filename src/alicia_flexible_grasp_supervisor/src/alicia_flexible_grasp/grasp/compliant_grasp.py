import time

class CompliantGraspLogic:
    def __init__(self, contact_threshold=200.0, target_force=1500.0, max_force=4000.0,
                 close_step_fast=0.002, close_step_slow=0.0007, open_step_safe=0.003,
                 gripper_min=0.0, gripper_max=0.05):
        self.contact_threshold = float(contact_threshold)
        self.target_force = float(target_force)
        self.max_force = float(max_force)
        self.close_step_fast = float(close_step_fast)
        self.close_step_slow = float(close_step_slow)
        self.open_step_safe = float(open_step_safe)
        self.gripper_min = float(gripper_min)
        self.gripper_max = float(gripper_max)

    def next_position(self, current_pos, force):
        f = float(force)
        pos = float(current_pos)
        if f > self.max_force:
            return max(self.gripper_min, pos - self.open_step_safe), 'over_force'
        if f < self.contact_threshold:
            return min(self.gripper_max, pos + self.close_step_fast), 'closing_fast'
        if f < self.target_force:
            return min(self.gripper_max, pos + self.close_step_slow), 'closing_slow'
        return pos, 'hold'
