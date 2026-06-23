#ifndef SERIAL_SERVER_NODE_H
#define SERIAL_SERVER_NODE_H

#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <serial/serial.h>
#include <mutex>
#include <thread>
#include <atomic>
#include <vector>
#include <string>

class SerialServerNode {
public:
    SerialServerNode();
    ~SerialServerNode();
    
    void run();

private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::Publisher pub_serial_data_;
    ros::Subscriber sub_send_data_;
    
    // 添加调试模式标志
    bool debug_mode_;
    
    // 串口相关
    serial::Serial serial_port_;
    std::mutex serial_mutex_;
    std::thread read_thread_;
    std::atomic<bool> is_running_;
    std::string port_name_;
    
    // 配置参数
    int baudrate_;
    int timeout_ms_;
    
    // 添加缺少的函数声明
    void shutdownCallback();
    
    bool connectSerial();
    void sendSerialDataCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg);
    void readFrameThread();
    std::string findSerialPort();
    bool serialDataCheck(const std::vector<uint8_t>& data);
    uint8_t sumElements(const std::vector<uint8_t>& data);
    void printHexFrame(const std::vector<uint8_t>& data, int type);
};

#endif // SERIAL_SERVER_NODE_H