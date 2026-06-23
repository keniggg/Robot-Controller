#!/usr/bin/env python

import rospy
import sys
from controller_manager_msgs.srv import (
    LoadController, UnloadController, SwitchController
)
    

def set_pid_for_all_joints(p, i, d):
    # List of all joints
    joints = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
    
    # Set parameters for all joints
    try:
        for joint in joints:
            rospy.set_param(f"/arm_pos_controller/gains/{joint}/p", p)
            rospy.set_param(f"/arm_pos_controller/gains/{joint}/i", i)
            rospy.set_param(f"/arm_pos_controller/gains/{joint}/d", d)
        rospy.loginfo(f"Parameters set for all joints: P={p}, I={i}, D={d}")
    except Exception as e:
        rospy.logerr(f"Failed to set parameters: {e}")
        return False
    
    # Restart the controller to apply changes
    try:
        # 1. First stop the controller
        rospy.wait_for_service('/controller_manager/switch_controller', timeout=2.0)
        switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        
        resp = switch_controller(
            stop_controllers=['arm_pos_controller'],
            start_controllers=[],
            strictness=1,
            start_asap=False,
            timeout=2.0
        )
        
        if not resp.ok:
            rospy.logerr("Failed to stop controller")
            return False
        
        rospy.loginfo("Controller stopped successfully")
        
        # 2. Unload the controller
        rospy.wait_for_service('/controller_manager/unload_controller', timeout=2.0)
        unload_controller = rospy.ServiceProxy('/controller_manager/unload_controller', UnloadController)
        
        resp = unload_controller('arm_pos_controller')
        if not resp.ok:
            rospy.logerr("Failed to unload controller")
            return False
            
        rospy.loginfo("Controller unloaded successfully")
        
        # 3. Load the controller with new parameters
        rospy.wait_for_service('/controller_manager/load_controller', timeout=2.0)
        load_controller = rospy.ServiceProxy('/controller_manager/load_controller', LoadController)
        
        resp = load_controller('arm_pos_controller')
        if not resp.ok:
            rospy.logerr("Failed to load controller")
            return False
            
        rospy.loginfo("Controller loaded successfully")
        
        # 4. Start the controller
        resp = switch_controller(
            stop_controllers=[],
            start_controllers=['arm_pos_controller'],
            strictness=1,
            start_asap=False,
            timeout=2.0
        )
        
        if not resp.ok:
            rospy.logerr("Failed to start controller")
            return False
        
        rospy.loginfo(f"Successfully updated PID values for all joints: P={p}, I={i}, D={d}")
        return True
            
    except rospy.ROSException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False



if __name__ == "__main__":
    rospy.init_node('pid_tuner')
    
    if len(sys.argv) < 4:
        print("Usage: tune_pid_all.py <p> <i> <d>")
        sys.exit(1)
    
    p = float(sys.argv[1])
    i = float(sys.argv[2])
    d = float(sys.argv[3])
    
    success = set_pid_for_all_joints(p, i, d)
    if not success:
        sys.exit(1)