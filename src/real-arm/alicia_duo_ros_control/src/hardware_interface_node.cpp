#include "serial_comm_helper.h"
#include "alicia_hw.h"
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "alicia_hardware_interface_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    MyRobotHW robot_hw(nh);

    // IMPORTANT: Call init() manually, since you're not using a plugin-based loader
    if (!robot_hw.init(nh, private_nh))
    {
        ROS_ERROR("Failed to initialize MyRobotHW!");
        return 1;
    }


    controller_manager::ControllerManager cm(&robot_hw, nh);

    ros::AsyncSpinner spinner(2);  // Launch AsyncSpinner with 2 threads
    spinner.start();

    ros::Rate loop_rate(50);  // Control loop frequency
    ros::Time last_time = ros::Time::now();

    while (ros::ok())
    {
        ros::Time now = ros::Time::now();
        ros::Duration elapsed_time = now - last_time;
        last_time = now;

        // Get the joint positions and gripper position after reading
        const std::vector<double>& joint_positions = robot_hw.getJointPositions(); 

        // Print the joint positions and gripper position
        // std::cout << "[Joint Positions] ";
        // for (size_t i = 0; i < joint_positions.size(); ++i)
        // {
        //     std::cout << "J" << (i + 1) << ": " << joint_positions[i] << " rad  ";
        // }
        // std::cout << std::endl;

        // std::cout << "Gripper: " << gripper_position << " rad" << std::endl;
    
        robot_hw.read(now, elapsed_time);

        cm.update(now, elapsed_time);
        // read pid parameters
        robot_hw.readpid(nh, now, elapsed_time);
        robot_hw.write(now, elapsed_time);
        loop_rate.sleep();
    }

    return 0;
}
