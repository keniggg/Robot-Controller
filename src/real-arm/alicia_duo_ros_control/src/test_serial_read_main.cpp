#include <ros/ros.h>
#include "serial_comm_helper.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_read_test_node");
    ros::NodeHandle nh;

    SerialCommHelper serial_helper(nh);
    ros::Rate rate(10);  // 10 Hz

    while (ros::ok())
    {
        auto [joint_angles, gripper_angle] = serial_helper.readJointAndGripper();

        ROS_INFO_STREAM("[Filtered Joint Angles + Gripper]");
        for (size_t i = 0; i < joint_angles.size(); ++i)
        {
            std::cout << "Joint" << i + 1 << ": " << joint_angles[i] << " rad  ";
        }
        std::cout << "Gripper: " << gripper_angle << " deg" << std::endl;

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
