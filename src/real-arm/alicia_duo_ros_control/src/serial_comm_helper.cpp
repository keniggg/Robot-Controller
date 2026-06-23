#include "serial_comm_helper.h"
#include <cmath>
#include <iostream>
SerialCommHelper::SerialCommHelper(ros::NodeHandle& nh)
    : nh_(nh), gripper_angle_(0.0)  // Removed servo_control_num_
{
    pub_ = nh_.advertise<alicia_duo_driver::ArmJointState>("arm_joint_command", 10);
    sub_ = nh_.subscribe("arm_joint_state", 10, &SerialCommHelper::feedbackCallback, this);

    filtered_angles_.resize(6, 0.0);  // 6 joints + gripper
}

void SerialCommHelper::writeServoCommand(const std::vector<double>& joint_rad, double gripper_rad)
{
    alicia_duo_driver::ArmJointState msg;

    // Convert joints from radians to degrees and populate the message fields
    msg.joint1 = joint_rad[0];
    msg.joint2 = joint_rad[1];
    msg.joint3 = joint_rad[2];
    msg.joint4 = joint_rad[3];
    msg.joint5 = joint_rad[4];
    msg.joint6 = joint_rad[5];

    // Convert gripper from radians to a binary state (0.0 or 0.14)
    msg.gripper = (gripper_rad > 0.002) ? 0.14f : 0.0f;

    // Optionally, you can add a time field (default 0)
    msg.time = 0.0;

    // Publish the message
    pub_.publish(msg);
}

std::pair<std::vector<double>, double> SerialCommHelper::readJointAndGripper()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    return {filtered_angles_, gripper_angle_};
}

void SerialCommHelper::feedbackCallback(const alicia_duo_driver::ArmJointState::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    // Directly assign the received values to the filtered angles
    filtered_angles_[0] = msg->joint1;
    filtered_angles_[1] = msg->joint2;
    filtered_angles_[2] = msg->joint3;
    filtered_angles_[3] = msg->joint4;
    filtered_angles_[4] = msg->joint5;
    filtered_angles_[5] = msg->joint6;

    // Gripper is already in radians (0.0 or 0.14)
    gripper_angle_ = msg->gripper;
}
