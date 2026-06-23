#include "alicia_hw.h"



MyRobotHW::MyRobotHW(ros::NodeHandle& nh)
    : nh_(nh), serial_helper_(nh) {}


// Corrected read method signature to match the base class
void MyRobotHW::read(const ros::Time& time, const ros::Duration& period)
{
    auto [joints, gripper_deg] = serial_helper_.readJointAndGripper();
    if (joints.size() == 6)
    {
        joint_position_ = joints;
        gripper_position_ = gripper_deg;
    }
}

// Corrected write method signature to match the base class
void MyRobotHW::write(const ros::Time& time, const ros::Duration& period)
{
    const int num_interpolation_steps = 15; // 您可以调整这个值
    
    // Create a vector for interpolated positions for this write cycle
    std::vector<double> interpolated_joint_positions(num_joints_);

    // Perform interpolation and send commands
    for (int step = 0; step < num_interpolation_steps; ++step)
    {
        for (int i = 0; i < num_joints_; ++i)
        {
            // Calculate the interpolation factor (alpha) from 0.0 to 1.0
            // For the last step, ensure alpha is exactly 1.0 to reach the target
            double alpha = (num_interpolation_steps > 1) ? 
                           static_cast<double>(step) / (num_interpolation_steps - 1) : 
                           1.0;
            if (step == num_interpolation_steps - 1) { // Ensure last step reaches target
                alpha = 1.0;
            }

            // Linear interpolation: current_actual + alpha * (target_command - current_actual)
            interpolated_joint_positions[i] = joint_position_[i] + alpha * (joint_position_command_[i] - joint_position_[i]);
        }
        
        // Send the interpolated joint positions

        serial_helper_.writeServoCommand(interpolated_joint_positions, gripper_position_command_);
    }

    
    // PID control

    // std::vector<double> current_cycle_position_error(num_joints_);
    // {
    //     std::lock_guard<std::mutex> lock(error_mutex_);
    //     // Copy the subscribed error to use for this PID cycle
    //     // This ensures consistency if the callback updates it during calculations
    //     // and minimizes lock duration.
    //     current_cycle_position_error = position_error_;
    // }

    // std::vector<double> control_signals(num_joints_);
    // std::vector<double> control_positions(num_joints_);

    // if (period.toSec() <= 0.0) { // Avoid division by zero if period is invalid
    //     ROS_WARN_THROTTLE(1.0, "PID control: period is zero or negative, skipping derivative calculation.");
    //     for (int i = 0; i < num_joints_; ++i) {
    //         control_signals[i] = p_gains_[i] * current_cycle_position_error[i]; // Basic P control if period is bad
    //     }
    // } else {
    //     for (int i = 0; i < num_joints_; ++i)
    //     {
    //         // position_error_[i] = joint_position_command_[i] - joint_position_[i]; // REMOVED - Now using error from topic

    //         double error_for_this_joint_pid = current_cycle_position_error[i] * 1000000;
    //         // double error_for_this_joint_pid = current_cycle_position_error[i]; 

    //         // Integral term - accumulates the error from the topic
    //         position_error_integral_[i] += error_for_this_joint_pid * period.toSec();
    //         // TODO: Add anti-windup for position_error_integral_[i] if not already implemented

    //         // Derivative term - rate of change of the error from the topic
    //         position_error_derivative_[i] = (error_for_this_joint_pid - prev_position_error_[i]) / period.toSec();
            
    //         // Update previous error with the error used in this cycle's D calculation
    //         prev_position_error_[i] = error_for_this_joint_pid;

    //         // PID control logic
    //         // std::cout << "Joint " << i << " PID: P=" << p_gains_[i] << ", I=" << i_gains_[i] << ", D=" << d_gains_[i] << std::endl;
    //         control_signals[i] = (p_gains_[i] * error_for_this_joint_pid +
    //                              i_gains_[i] * position_error_integral_[i] +
    //                              d_gains_[i] * position_error_derivative_[i])/ 1000000;
    //         // std::cout << "Joint " << i << " Control Signal: " << control_signals[i] << std::endl;
    //         control_positions[i] = joint_position_[i] + control_signals[i] ; // Calculate the new position
    //         // std::cout << "Joint " << i << " Control Position: " << control_positions[i] << std::endl;
    //     }
    // }
    // serial_helper_.writeServoCommand(control_positions, gripper_position_command_);


}

// Setters for sending joint commands and gripper command
void MyRobotHW::setJointCommands(const std::vector<double>& positions)
{
    if (positions.size() == joint_position_command_.size())
    {
        joint_position_command_ = positions;
    }
}

void MyRobotHW::setGripperCommand(double gripper_deg)
{
    gripper_position_command_ = gripper_deg;
}

MyRobotHW::~MyRobotHW() {}


bool MyRobotHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
    ROS_INFO("MyRobotHW::init() called, interfaces registered.");

    num_joints_ = 6;
    joint_names_ = {"Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"};

    joint_position_.resize(num_joints_, 0.0);
    joint_velocity_.resize(num_joints_, 0.0);
    joint_effort_.resize(num_joints_, 0.0);
    joint_position_command_.resize(num_joints_, 0.0);

    // Initialize PID variables
    p_gains_.resize(num_joints_, 0.0);
    i_gains_.resize(num_joints_, 0.0);
    d_gains_.resize(num_joints_, 0.0);
    position_error_.resize(num_joints_, 0.0);
    position_error_integral_.resize(num_joints_, 0.0);
    position_error_derivative_.resize(num_joints_, 0.0);
    prev_position_error_.resize(num_joints_, 0.0);


    for (int i = 0; i < num_joints_; ++i)
    {
        joint_state_interface_.registerHandle(
            hardware_interface::JointStateHandle(joint_names_[i],
                                                 &joint_position_[i],
                                                 &joint_velocity_[i],
                                                 &joint_effort_[i]));

        position_joint_interface_.registerHandle(
            hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[i]),
                                            &joint_position_command_[i]));
    }




    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);


    readpid(root_nh, ros::Time::now(), ros::Duration(0.0)); // Call readpid to initialize PID parameters
    ROS_INFO("Subscribed to /arm_pos_controller/state");
    return true;
}


void MyRobotHW::controllerStateCallback(const control_msgs::JointTrajectoryControllerStateConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(error_mutex_);
    if (msg->error.positions.size() != num_joints_) {
        ROS_WARN_THROTTLE(1.0, 
            "Controller state: error.positions size (%zu) does not match expected num_joints_ (%d). Skipping error update.",
            msg->error.positions.size(), num_joints_);
        return;
    }
    for (int i = 0; i < num_joints_; ++i) {
        position_error_[i] = msg->error.positions[i];
    }
}

void MyRobotHW::readpid(ros::NodeHandle& root_nh, const ros::Time& time, const ros::Duration& period)
{
    // This function is not used in the current implementation
    // but can be used to read PID parameters if needed.
    // Load PID gains from parameter server
    for (int i = 0; i < num_joints_; i++) {
        std::string param_name = "arm_pos_controller/gains/" + joint_names_[i] + "/p";
        if (!root_nh.getParam(param_name, p_gains_[i])) {
            ROS_WARN("Could not find P gain for joint %s, using default", joint_names_[i].c_str());
            p_gains_[i] = 10; // Default value
        }
        
        param_name = "arm_pos_controller/gains/" + joint_names_[i] + "/i";
        if (!root_nh.getParam(param_name, i_gains_[i])) {
            ROS_WARN("Could not find I gain for joint %s, using default", joint_names_[i].c_str());
            i_gains_[i] = 0.0; // Default value
        }
        
        param_name = "arm_pos_controller/gains/" + joint_names_[i] + "/d";
        if (!root_nh.getParam(param_name, d_gains_[i])) {
            ROS_WARN("Could not find D gain for joint %s, using default", joint_names_[i].c_str());
            d_gains_[i] = 0.0; // Default value
        }
        
        // ROS_INFO("PID gains for joint %s: P=%f, I=%f, D=%f", 
                //   joint_names_[i].c_str(), p_gains_[i], i_gains_[i], d_gains_[i]);
    }
    controller_state_subscriber_ = root_nh.subscribe<control_msgs::JointTrajectoryControllerState>(
        "/arm_pos_controller/state", 1, &MyRobotHW::controllerStateCallback, this);
}