#include "serial_server_node/serial_server_node.h"

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "serial_server_node");
    
    // 创建并运行节点
    SerialServerNode node;
    node.run();
    
    return 0;
}