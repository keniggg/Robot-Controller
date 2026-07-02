#include "alicia_d_driver/alicia_d_hw_interface.h"
#include <cmath>
#include <vector>

namespace alicia_d_driver
{
AliciaDHardwareInterface::AliciaDHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {}

bool AliciaDHardwareInterface::init()
{
    // Get joint names from the parameter server
    if (!nh_.getParam("joints", joint_names_))
    {
        ROS_ERROR("Could not find 'joints' parameter on the parameter server.");
        return false;
    }
    num_joints_ = joint_names_.size();
    ROS_INFO("Initializing hardware interface for %d joints.", (int)num_joints_);

    // Resize storage vectors
    joint_velocities_.resize(num_joints_, 0.0); // Initialize dummy velocities to zero
    joint_efforts_.resize(num_joints_, 0.0); // Not used, but required by the interface
    joint_positions_.resize(num_joints_, 0.0);
    joint_position_commands_.resize(num_joints_, 0.0);
    raw_joint_positions_.resize(num_joints_, 0.0);
    last_published_positions_.resize(num_joints_, 0.0);

    ros::NodeHandle pnh("~");
    pnh.param<bool>("publish_only_on_change", publish_only_on_change_, true);
    pnh.param<bool>("publish_initial_command", publish_initial_command_, false);
    pnh.param<double>("command_publish_epsilon", command_publish_epsilon_, 1e-5);

    // Create a map for efficient name-to-index lookup
    for (size_t i = 0; i < num_joints_; ++i)
    {
        joint_name_to_index_map_[joint_names_[i]] = i;
    }

    // Register handles with the ros_control interfaces
    for (size_t i = 0; i < num_joints_; ++i)
    {
        // Joint State Interface
        jnt_state_interface_.registerHandle(hardware_interface::JointStateHandle(
                joint_names_[i], &joint_positions_[i], &joint_velocities_[i], &joint_efforts_[i]));
        // Position Joint Interface
        pos_jnt_interface_.registerHandle(hardware_interface::JointHandle(
            jnt_state_interface_.getHandle(joint_names_[i]), &joint_position_commands_[i]));
    }

    // Register the interfaces with this class
    registerInterface(&jnt_state_interface_);
    registerInterface(&pos_jnt_interface_);

    // Initialize ROS publisher and subscriber
    joint_command_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_commands", 1);
    joint_state_sub_ = nh_.subscribe("/joint_states", 10, &AliciaDHardwareInterface::jointStateCallback, this);

    ROS_INFO("Alicia-D hardware interface initialized successfully.");
    return true;
}



void AliciaDHardwareInterface::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(command_mutex_); // Lock to ensure thread safety

    // Update joint positions from the received message
    bool matched_joint = false;
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
    {
        auto it = joint_name_to_index_map_.find(msg->name[i]);
        // ROS_INFO("Received joint state for %s: %f", msg->name[i].c_str(), msg->position[i]);
        if (it != joint_name_to_index_map_.end())
        {
            size_t index = it->second;
            if (index < joint_positions_.size())
            {
                raw_joint_positions_[index] = msg->position[i];
                matched_joint = true;
            }
        }
    }
    if (matched_joint)
    {
        has_received_joint_state_ = true;
    }
}


void AliciaDHardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
    std::lock_guard<std::mutex> lock(command_mutex_);
    // Copy raw joint positions to the main positions vector
    joint_positions_ = raw_joint_positions_;
    if (!is_initialized_ && has_received_joint_state_)
    {
        joint_position_commands_ = joint_positions_;
        last_published_positions_ = joint_position_commands_;
        is_initialized_ = true;
    }

}



void AliciaDHardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
    if (!is_initialized_ && publish_only_on_change_ && !publish_initial_command_)
    {
        return;
    }

    bool changed = !have_published_command_;
    for (size_t i = 0; i < joint_position_commands_.size() && i < last_published_positions_.size(); ++i)
    {
        if (std::fabs(joint_position_commands_[i] - last_published_positions_[i]) > command_publish_epsilon_)
        {
            changed = true;
            break;
        }
    }

    if (publish_only_on_change_ && !changed)
    {
        return;
    }

    if (!publish_initial_command_ && !have_published_command_)
    {
        last_published_positions_ = joint_position_commands_;
        have_published_command_ = true;
        ROS_INFO("Skipping initial ros_control /joint_commands publish to avoid overriding GUI/direct control.");
        return;
    }

    sensor_msgs::JointState command_msg;
    command_msg.header.stamp = ros::Time::now();
    command_msg.name = joint_names_;
    command_msg.position = joint_position_commands_;

    // Publish the joint command message
    joint_command_pub_.publish(command_msg);
    last_published_positions_ = joint_position_commands_;
    have_published_command_ = true;
}

}
