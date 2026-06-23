#!/usr/bin/env python3
# coding=utf-8

import rospy
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import Int32
import time

def print_hex_frame(frame_msg):
    """
        @brief 将整数数组转换为十六进制字符串并打印

        @param frame_msg 整数数组
    """
    hex_output = ""
    
    # 遍历 frame_msg 数组，并将每个 byte_int 转换为十六进制字符串格式
    for byte_int in frame_msg:
        hex_str = hex(byte_int)[2:].upper().zfill(2)

        if hex_output:  # 如果 hex_output 不是空字符串
            hex_output += " " + hex_str
        else:           # 无字符时，不添加开头空格
            hex_output += hex_str
        
    # 使用rospy日志替代print
    rospy.loginfo(hex_output)

class SerialReaderNode:
    def __init__(self):
        rospy.init_node('read_serial_type_node') # 初始化节点
        
        # 从参数服务器读取调试模式参数
        self.debug_mode = rospy.get_param('~debug_mode', False)
        rospy.loginfo("Debug mode: %s", "enabled" if self.debug_mode else "disabled")
        
        # 统计数据 - 仅用于诊断，不影响核心逻辑
        self.frame_count = 0
        self.start_time = time.time()
        
        # 保持原始的变量名称
        self.pub_2 = rospy.Publisher('/gripper_angle', Int32, queue_size=10)
        self.pub_4 = rospy.Publisher('/servo_states', UInt8MultiArray, queue_size=10)
        self.pub_6 = rospy.Publisher('/servo_states_6', UInt8MultiArray, queue_size=10)
        self.pub_EE = rospy.Publisher('/error_frame_deal', UInt8MultiArray, queue_size=10)
        
        # 订阅串口数据话题
        self.sub = rospy.Subscriber('/read_serial_data', UInt8MultiArray, self.serial_data_callback, queue_size=50)
        
        # 诊断计时器 - 不影响核心逻辑
        if self.debug_mode:
            rospy.Timer(rospy.Duration(30), self.report_stats)
    
    def report_stats(self, _):
        """报告处理统计信息 - 仅用于诊断"""
        duration = time.time() - self.start_time
        if duration > 0:
            rate = self.frame_count / duration
            rospy.loginfo("处理率: %.2f 帧/秒 (总计: %d 帧)", rate, self.frame_count)

    def serial_data_callback(self, serial_msg):
        """
            @brief 串口数据回调函数

            @param serial_msg: 串口数据
        """
        self.frame_count += 1  # 统计计数 - 不影响核心逻辑
        
        # 数据验证 - 防止崩溃，不改变处理逻辑
        if len(serial_msg.data) < 2:
            rospy.logwarn("数据帧过短，无法处理")
            return
            
        command = serial_msg.data[1] # 指令id
        
        # === 核心逻辑部分 - 保持不变 ===
        if command == 0x02:
            # 检查数据长度
            if len(serial_msg.data) < 6:
                rospy.logwarn("夹爪角度数据帧长度不足")
                return
                
            gripper_angle = serial_msg.data[4] | (serial_msg.data[5] << 8)
            self.pub_2.publish(gripper_angle)
            
            if self.debug_mode:
                rospy.loginfo("夹爪角度: %d", gripper_angle)
                
        elif command == 0x04:
            self.pub_4.publish(serial_msg)
            if self.debug_mode:
                rospy.loginfo("舵机状态数据")
                print_hex_frame(serial_msg.data)
                
        elif command == 0x06:
            self.pub_6.publish(serial_msg)
            if self.debug_mode:
                rospy.loginfo("扩展舵机状态数据")
                print_hex_frame(serial_msg.data)
                
        elif command == 0xEE:
            self.pub_EE.publish(serial_msg)
            
            # 修复格式化字符串错误
            if len(serial_msg.data) >= 5:
                error_type = serial_msg.data[3]
                error_param = serial_msg.data[4]
                rospy.logwarn("错误帧: 类型=0x%02X, 参数=0x%02X", error_type, error_param)
            else:
                rospy.logwarn("0x%02X 有话题，但暂时无接收处理", command)
        else:
            # 修复格式化字符串错误
            rospy.logwarn("暂无该指令id 0x%02X 的功能", command)

def main():
    try:
        node = SerialReaderNode()
        rospy.loginfo("read_serial_type_node 已启动")
        rospy.spin() # 保持节点运行
    except rospy.ROSInterruptException: # 捕获crtl+c异常
        pass
    except Exception as e:
        rospy.logerr("节点异常: %s", str(e))

if __name__ == '__main__':
    main()