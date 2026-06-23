#include "serial_server_node/serial_server_node.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <chrono>
#include <boost/locale.hpp>

SerialServerNode::SerialServerNode() : 
    baudrate_(921600),
    timeout_ms_(1000),
    is_running_(false),
    debug_mode_(false)  // 默认关闭调试模式
{
    // 初始化ROS发布者和订阅者
    pub_serial_data_ = nh_.advertise<std_msgs::UInt8MultiArray>("/read_serial_data", 10);
    sub_send_data_ = nh_.subscribe("/send_serial_data", 10, &SerialServerNode::sendSerialDataCallback, this);
    
    // 从参数服务器读取调试模式参数
    nh_.param<bool>("debug_mode", debug_mode_, false);
    ROS_INFO_STREAM("Debug mode: " << (debug_mode_ ? "enabled" : "disabled"));

    // 从参数服务器读取串口参数
    nh_.param<std::string>("port", port_name_, "");
    nh_.param<int>("baudrate", baudrate_, 921600);
    
    ROS_INFO_STREAM("Port name from param: " << (port_name_.empty() ? "not specified" : port_name_));
    ROS_INFO_STREAM("Baudrate: " << baudrate_);

    // 使用正确的语法设置关闭回调
    ros::NodeHandle n;
    n.setParam("/serial_server_node/shutdown_requested", false);
    
    // 连接串口
    connectSerial();
}

// 析构函数
SerialServerNode::~SerialServerNode() {
    // 确保线程和串口在析构时被正确关闭
    shutdownCallback();
}

// 运行节点
void SerialServerNode::run() {
    ROS_INFO("serial_server_node is running");
    ros::spin();
}

// 关闭回调
void SerialServerNode::shutdownCallback() {
    is_running_ = false;
    
    // 等待线程结束
    if (read_thread_.joinable()) {
        read_thread_.join();
    }
    
    // 关闭串口
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_port_.isOpen()) {
        serial_port_.close();
        ROS_INFO("Serial port closed");
    }
}

// 查找可用的串口
std::string SerialServerNode::findSerialPort() {
    // 记录当前时间，避免过频繁打印日志
    static ros::Time last_log_time = ros::Time(0);
    bool should_log = (ros::Time::now() - last_log_time).toSec() >= 5.0; // 每5秒允许打印一次日志
    
    // 获取串口列表
    std::vector<serial::PortInfo> ports;
    try {
        ports = serial::list_ports();
    } catch (const std::exception& e) {
        if (should_log) {
            ROS_ERROR_STREAM("列出端口时异常: " << e.what());
            last_log_time = ros::Time::now();
        }
        return "";
    }
    
    // 如果有端口
    if (!ports.empty() && should_log) {
        std::stringstream ss;
        ss << "找到 " << ports.size() << " 个串口设备:";
        for (const auto& port : ports) {
            ss << " " << port.port;
        }
        ROS_INFO_STREAM(ss.str());
        last_log_time = ros::Time::now();
    }
    
    // 如果没有端口
    if (ports.empty()) {
        return "";
    }
    
    // 首先尝试使用指定的端口
    if (!port_name_.empty()) {
        std::string full_port_path = "/dev/" + port_name_;
        // 检查指定的端口是否在列表中
        for (const auto& port : ports) {
            if (port.port == full_port_path) {
                if (access(port.port.c_str(), R_OK | W_OK) == 0) {
                    if (should_log) {
                        ROS_INFO_STREAM("使用launch指定的端口: " << port.port);
                    }
                    return port.port;
                }
            }
        }
        
        if (should_log) {
            ROS_WARN_STREAM("指定的端口 " << full_port_path << " 不可用，将搜索其他ttyUSB设备");
        }
    }
    
    // 尝试所有ttyUSB设备
/*
    for (const auto& port : ports) {
        if (port.port.find("ttyUSB") != std::string::npos) {
            if (access(port.port.c_str(), R_OK | W_OK) == 0) {
                if (should_log) {
                    ROS_INFO_STREAM("Found available ttyUSB device: " << port.port);
                }
                return port.port;
            }
        }
    }
    
    if (should_log) {
        ROS_WARN("未找到可用的ttyUSB设备");
    }
        */
     //尝试所有ttyACM设备
        for (const auto& port : ports) {
        if (port.port.find("ttyACM") != std::string::npos) {
            if (access(port.port.c_str(), R_OK | W_OK) == 0) {
                if (should_log) {
                    ROS_INFO_STREAM("Found available ttyACM device: " << port.port);
                }
                return port.port;
            }
        }
    }
    
    if (should_log) {
        ROS_WARN("未找到可用的ttyACM设备");
    }
    return "";
}

// 连接串口
bool SerialServerNode::connectSerial() {
    static int failure_counter = 0;
    static ros::Time last_reconnect_time = ros::Time(0);
    
    // 避免过于频繁的尝试
    double time_since_last = (ros::Time::now() - last_reconnect_time).toSec();
    if (time_since_last < 1.0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        return false;
    }
    
    last_reconnect_time = ros::Time::now();
    
    try {
        // 查找可用串口
        std::string port = findSerialPort();
        
        // 没有找到可用串口
        if (port.empty()) {
            failure_counter++;
            // 使用指数退避策略，避免过于频繁的重试
            int wait_seconds = std::min(10, (failure_counter / 10) + 1);
            
            // 安排下一次尝试，但不以错误状态退出
            ros::Timer timer = nh_.createTimer(
                ros::Duration(wait_seconds),
                [this](const ros::TimerEvent&) { connectSerial(); },
                true
            );
            return false;
        }
        
        // 有可用端口，尝试连接
        ROS_INFO_STREAM("正在连接端口: " << port);
        
        // 关闭已有连接
        std::lock_guard<std::mutex> lock(serial_mutex_);
        if (serial_port_.isOpen()) {
            serial_port_.close();
        }
        // port="/dev/ttyUSB0";
        
        // 设置串口参数
        serial_port_.setPort(port);
        serial_port_.setBaudrate(baudrate_);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(2000);
        serial_port_.setTimeout(timeout);
        
        // 尝试连接，最多3次
        for (int attempt = 1; attempt <= 3; attempt++) {
            try {
                serial_port_.open();
                if (serial_port_.isOpen()) {
                    ROS_INFO("串口连接成功");
                    failure_counter = 0; // 重置失败计数
                    
                    // 启动读取线程
                    is_running_ = true;
                    if (read_thread_.joinable()) {
                        read_thread_.join(); // 确保旧线程已结束
                    }
                    read_thread_ = std::thread(&SerialServerNode::readFrameThread, this);
                    return true;
                }
            } catch (const std::exception& e) {
                ROS_WARN_STREAM("连接尝试 #" << attempt << " 失败: " << e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(800));
            }
        }
        
        // 如果所有尝试都失败
        failure_counter++;
        
        // 安排下一次尝试
        ros::Timer timer = nh_.createTimer(
            ros::Duration(2.0),
            [this](const ros::TimerEvent&) { connectSerial(); },
            true
        );
        return false;
        
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("连接串口异常: " << e.what());
        failure_counter++;
        
        // 安排下一次尝试
        ros::Timer timer = nh_.createTimer(
            ros::Duration(2.0),
            [this](const ros::TimerEvent&) { connectSerial(); },
            true
        );
        return false;
    }
}

// 发送数据回调
void SerialServerNode::sendSerialDataCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg) {
    // 将ROS消息转换为字节数组
    std::vector<uint8_t> data(msg->data.begin(), msg->data.end());
    
    try {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        if (!serial_port_.isOpen()) {
            ROS_WARN("Serial port is not open, attempting to reconnect");
            if (!connectSerial()) {
                ROS_ERROR("Failed to reconnect to serial port");
                return;
            }
        }
        
        // 写入数据

        size_t bytes_written = serial_port_.write(data);
        if (bytes_written != data.size()) {
            ROS_WARN_STREAM("Only wrote " << bytes_written << " of " << data.size() << " bytes");
        }
        
        printHexFrame(data, 0);
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception while sending data: " << e.what());
        // 尝试重新连接而不是退出程序
        connectSerial();
    }
}

// 读取数据线程
void SerialServerNode::readFrameThread() {
    std::vector<uint8_t> frame_buffer;
    bool wait_for_start = true;
    
    while (is_running_ && ros::ok()) {
        try {
            // 检查串口是否打开
            {
                std::lock_guard<std::mutex> lock(serial_mutex_);
                if (!serial_port_.isOpen()) {
                    // 串口已关闭，尝试重新连接
                    static ros::Time last_warning = ros::Time(0);
                    if ((ros::Time::now() - last_warning).toSec() >= 5.0) {
                        ROS_WARN("串口已关闭，正在尝试重新连接");
                        last_warning = ros::Time::now();
                    }
                    
                    // 解锁后再尝试重连，避免死锁
                }
                
                // 如果串口未打开，释放锁并等待
                if (!serial_port_.isOpen()) {
                    // 不要在锁内部休眠
                }
            }
            
            // 串口未打开，尝试重连并等待
            if (!serial_port_.isOpen()) {
                // 解锁后再重连，避免死锁
                connectSerial();
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }
            
            // 读取数据
            uint8_t byte = 0;
            size_t bytes_read = 0;
            
            {
                std::lock_guard<std::mutex> lock(serial_mutex_);
                // 只有串口打开且有数据可读时才读取
                if (serial_port_.isOpen() && serial_port_.available() > 0) {
                    try {
                        bytes_read = serial_port_.read(&byte, 1);
                    } catch (const std::exception& e) {
                        // 读取出错，可能是串口断开
                        ROS_ERROR_STREAM("读取串口数据异常: " << e.what());
                        serial_port_.close(); // 关闭串口以便重新连接
                        wait_for_start = true;
                        frame_buffer.clear();
                        continue;
                    }
                } else {
                    // 无数据可读，短暂休眠
                }
            }
            
            // 没有数据可读或串口未打开，短暂休眠
            if (bytes_read == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            
            static int reconnect_attempts = 0;
            reconnect_attempts = 0;  // 成功读取，重置尝试计数
            
            // 帧解析逻辑
            if (wait_for_start) {
                // 查找帧开始标记
                if (byte == 0xAA) {
                    frame_buffer.clear();
                    frame_buffer.push_back(byte);
                    wait_for_start = false;
                }
            } else {
                // 构建帧
                frame_buffer.push_back(byte);
                
                // 检查是否找到帧结束标记
                if (byte == 0xFF && frame_buffer.size() >= 3) {
                    uint8_t expected_length = frame_buffer[2] + 5;  // 数据长度+5等于帧长度
                    
                    if (frame_buffer.size() == expected_length) {
                        // 验证校验和
                        if (serialDataCheck(frame_buffer)) {
                            // 构建ROS消息并发布
                            std_msgs::UInt8MultiArray msg;
                            msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
                            msg.layout.dim[0].size = frame_buffer.size();
                            msg.layout.dim[0].stride = 1;
                            msg.data = frame_buffer;
                            // std::cout << "接收到数据: ";
                            // for (const auto& byte : frame_buffer) {
                            //     std::cout << std::hex << std::uppercase << std::setfill('0') 
                            //               << std::setw(2) << static_cast<int>(byte) << " ";
                            // }
                            printHexFrame(frame_buffer, 1);
                            pub_serial_data_.publish(msg);
                        } else {
                            ROS_WARN("Frame checksum validation failed");
                        }
                        wait_for_start = true;
                    } else if (expected_length > 64 || frame_buffer.size() > 64) {
                        // 帧太长，认为帧错误
                        ROS_WARN("Frame too long, discarding");
                        wait_for_start = true;
                    }
                }
            }
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Exception in read thread: " << e.what());
            wait_for_start = true;
            
            // 尝试重新连接
            std::this_thread::sleep_for(std::chrono::seconds(1));
            connectSerial();
        }
    }
}


// 检查数据的校验和
bool SerialServerNode::serialDataCheck(const std::vector<uint8_t>& data) {
    if (data.size() < 4) {
        return false;
    }
    
    uint8_t calculated_check = sumElements(data) % 2;
    uint8_t received_check = data[data.size() - 2];
    
    return (calculated_check == received_check);
}

// 计算数据的校验和
uint8_t SerialServerNode::sumElements(const std::vector<uint8_t>& data) {
    if (data.size() < 4) {
        ROS_ERROR("Data array too small for checksum calculation");
        return 0;
    }
    
    // 计算从第3个字节到倒数第2个字节之前的所有元素的和
    uint32_t sum = 0;
    for (size_t i = 3; i < data.size() - 2; ++i) {
        sum += data[i];
    }
    
    return sum % 2;
}
// 打印十六进制数据
void SerialServerNode::printHexFrame(const std::vector<uint8_t>& data, int type) {
    // 如果不是调试模式，直接返回
    if (!debug_mode_) {
        return;
    }
    
    std::stringstream ss;
    
    // 根据类型选择前缀
    if (type == 0) {
        ss << "发送数据: ";
    } else if (type == 1) {
        ss << "数据接收: ";
    } else {
        ss << "接收数据的一部分: ";
    }
    
    // 添加每个字节的十六进制表示
    for (const auto& byte : data) {
        ss << std::uppercase << std::setfill('0') << std::setw(2) 
           << std::hex << static_cast<int>(byte) << " ";
    }
    
    ROS_INFO_STREAM(ss.str());
}