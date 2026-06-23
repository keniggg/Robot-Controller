#pragma once

#include <ros/ros.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include "serial_comm_helper.h"
#include <control_msgs/JointTrajectoryControllerState.h> // For controller state
#include <mutex>          
class MyRobotHW : public hardware_interface::RobotHW
{
public:
    explicit MyRobotHW(ros::NodeHandle& nh);
    // ~MyRobotHW() = default;
    // virtual ~MyRobotHW() = default;
    virtual ~MyRobotHW();  


    bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
    void read(const ros::Time& time, const ros::Duration& period) override;
    void write(const ros::Time& time, const ros::Duration& period) override;

    // Getters for current state
    const std::vector<double>& getJointPositions() const { return joint_position_; }
    double getGripperPosition() const { return gripper_position_; }

    // Setters for sending commands
    void setJointCommands(const std::vector<double>& positions);
    void setGripperCommand(double gripper_deg);
    void readpid(ros::NodeHandle& root_nh, const ros::Time& time, const ros::Duration& period);

private:
    ros::NodeHandle nh_;
    int num_joints_;  // Declare num_joints_ here
    SerialCommHelper serial_helper_;

    // Hardware interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;

    // Joint configuration
    static constexpr int NUM_JOINTS = 6;  // Declare num_joints_
    std::vector<std::string> joint_names_ = {
        "Joint1", "Joint2", "Joint3", 
        "Joint4", "Joint5", "Joint6"
    };

    // State vectors
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;

    // Command vectors
    std::vector<double> joint_position_command_;
    double gripper_position_ = 0.0;
    double gripper_position_command_ = 0.0;


    // Zero velocity and effort for initialization
    double zero_velocity_ = 0.0;
    double zero_effort_ = 0.0;

    
    // PID parameters
    std::vector<double> p_gains_;
    std::vector<double> i_gains_;
    std::vector<double> d_gains_;
    
    // Error tracking
    std::vector<double> position_error_;
    std::vector<double> position_error_integral_;
    std::vector<double> position_error_derivative_;
    std::vector<double> prev_position_error_;

    ros::Subscriber controller_state_subscriber_;
    std::mutex error_mutex_; // To protect access to position_error_

    void controllerStateCallback(const control_msgs::JointTrajectoryControllerStateConstPtr& msg);
};
