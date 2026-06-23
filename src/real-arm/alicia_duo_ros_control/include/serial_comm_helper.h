#pragma once

#include <ros/ros.h>
#include <alicia_duo_driver/ArmJointState.h>  // Correct message type for ArmJointState
#include <std_msgs/Float32MultiArray.h>
#include <vector>
#include <mutex>

class SerialCommHelper
{
public:
    explicit SerialCommHelper(ros::NodeHandle& nh);

    /**
     * @brief Write servo command (6 joints + gripper)
     * @param joint_rad Joint angles in radians (size=6)
     * @param gripper_rad Gripper position in radians (0.0=closed, >0.002=open)
     */
    void writeServoCommand(const std::vector<double>& joint_rad, double gripper_rad);

    /**
     * @brief Read current joint and gripper state
     * @return Pair of:
     *         - vector<double> (6 joint angles in radians)
     *         - double (gripper state in radians)
     */
    std::pair<std::vector<double>, double> readJointAndGripper();

private:
    void feedbackCallback(const alicia_duo_driver::ArmJointState::ConstPtr& msg);  // Changed to ArmJointState message type

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;

    std::vector<double> filtered_angles_;  // 6 joint angles in radians
    double gripper_angle_;                 // Gripper state in radians (0.0 or ~0.14)

    std::mutex data_mutex_;
};
