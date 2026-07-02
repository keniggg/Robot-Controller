class GripperCommander:
    def __init__(self, joint_commander, gripper_index=6, min_m=0.0, max_m=0.05):
        self.joint_commander = joint_commander
        self.gripper_index = gripper_index
        self.min_m = float(min_m)
        self.max_m = float(max_m)
        self.current = self.min_m
    def set_position(self, pos_m, arm_positions=None):
        pos = max(self.min_m, min(self.max_m, float(pos_m)))
        positions = self.joint_commander.last_positions[:]
        if len(positions) <= self.gripper_index:
            positions += [0.0]*(self.gripper_index + 1 - len(positions))
        if arm_positions is not None:
            for index, value in enumerate(list(arm_positions)[:self.gripper_index]):
                if index < len(positions):
                    positions[index] = value
        positions[self.gripper_index] = pos
        self.current = pos
        return self.joint_commander.publish(positions)
    def open(self):
        return self.set_position(self.min_m)
    def close_limit(self):
        return self.set_position(self.max_m)
