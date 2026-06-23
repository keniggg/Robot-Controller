#!/usr/bin/env python3
# coding=utf-8

"""
标准机械臂控制节点
接收7自由度(6关节+夹爪)的弧度控制命令，转换为硬件协议格式并发送。
"""

import rospy
import math
import numpy as np
from std_msgs.msg import UInt8MultiArray, MultiArrayDimension,Float32, Bool
from alicia_duo_driver.msg import ArmJointState

# 常量定义
RAD_TO_DEG = 180.0 / math.pi  # 弧度转角度系数
DEG_TO_RAD = math.pi / 180.0  # 角度转弧度系数

# 协议常量
FRAME_HEADER = 0xAA
FRAME_FOOTER = 0xFF
CMD_SERVO_CONTROL = 0x04
CMD_EXTENDED_CONTROL = 0x06
CMD_GRIPPER_CONTROL = 0x02
CMD_ZERO_CAL = 0x03
CMD_DEMO_CONTROL = 0x13

class ServoControlNode:
    def __init__(self):
        """初始化节点"""
        rospy.init_node('servo_control_node')
        
        # 配置参数
        self.servo_count = rospy.get_param('~servo_count', 9)  # 总舵机数量
        self.joint_count = 6  # 有效关节数
        self.debug_mode = rospy.get_param('~debug_mode', False)
        
        # 设置数据结构
        self._setup_data()
        
        # 设置ROS通信
        self._setup_communication()
        
        rospy.loginfo("机械臂控制节点已初始化，使用标准弧度接口")
        
    def _setup_data(self):
        """初始化数据结构"""
        # 帧大小计算：帧头(1)+命令(1)+长度(1)+数据(舵机数*2)+校验(1)+帧尾(1)
        self.frame_size = self.servo_count * 2 + 5
        
        # 标准舵机控制帧
        self.servo_angle_frame = [0] * self.frame_size
        self.servo_angle_frame[0] = FRAME_HEADER
        self.servo_angle_frame[1] = CMD_SERVO_CONTROL
        self.servo_angle_frame[2] = self.servo_count * 2
        self.servo_angle_frame[-1] = FRAME_FOOTER
        
        # 夹爪控制帧 (固定长度)
        self.gripper_frame = [0] * 8  
        self.gripper_frame[0] = FRAME_HEADER
        self.gripper_frame[1] = CMD_GRIPPER_CONTROL
        # self.gripper_frame[2] = 5
        self.gripper_frame[2] = 3
        self.gripper_frame[3] = 1
        self.gripper_frame[-1] = FRAME_FOOTER
        
        # 舵机映射表：关节索引->舵机索引
        # 机械臂的6个关节需要映射到9个舵机上
        # [关节1, 关节1(重复), 关节2, 关节2(反向), 关节3, 关节3(反向), 关节4, 关节5, 关节6]
        self.joint_to_servo_map = [
            (0, 1.0),    # 关节1 -> 舵机1 (正向)
            (0, 1.0),    # 关节1 -> 舵机2 (正向重复)
            (1, 1.0),    # 关节2 -> 舵机3 (正向)
            (1, -1.0),   # 关节2 -> 舵机4 (反向)
            (2, 1.0),    # 关节3 -> 舵机5 (正向)
            (2, -1.0),   # 关节3 -> 舵机6 (反向)
            (3, 1.0),    # 关节4 -> 舵机7 (正向)
            (4, 1.0),    # 关节5 -> 舵机8 (正向)
            (5, 1.0),    # 关节6 -> 舵机9 (正向)
        ]
    
    def _setup_communication(self):
        """设置订阅者和发布者"""
        # 发布者 - 发送硬件协议数据到串口节点
        self.serial_pub = rospy.Publisher('/send_serial_data', UInt8MultiArray, queue_size=10)
        
        # 订阅者 - 接收标准关节控制命令
        self.joint_sub = rospy.Subscriber('/arm_joint_command', ArmJointState, self.joint_command_callback, queue_size=10)
    
        self.gripper_sub = rospy.Subscriber('/gripper_control', Float32, self.gripper_control_callback, queue_size=10)
        
        self.zero_calibration = rospy.Subscriber('/zero_calibrate', Bool, self.zero_calib_callback, queue_size=10)
        self.demo_mode = rospy.Subscriber('/demonstration', Bool, self.move_free_callback, queue_size=10)
        
        
    def calculate_checksum(self, frame):
        """计算校验和"""
        # 计算索引3到倒数第2个元素的总和，对2取模
        checksum = sum(frame[3:-2]) % 2
        return checksum
    
    def rad_to_hardware_value_grip(self, angle_rad):
        """将弧度转换为夹爪舵机值(2048-2900)"""
        # 先转换为角度
        angle_deg = angle_rad * RAD_TO_DEG
        
        # 范围检查
        if angle_deg < 0:
            rospy.logwarn("角度值超出范围: %.2f度，会被截断", angle_deg)
            angle_deg = 0
        elif angle_deg > 300.0:
            rospy.logwarn("角度值超出范围: %.2f度，会被截断", angle_deg)
            angle_deg = 300.0
        
        # 转换公式：0度对应2048，100度对应2900
        value = int(2048 + (angle_deg * 8.52))
        
        # 范围限制
        return max(2048, min(2900, value))
    
    def rad_to_hardware_value(self, angle_rad):
        """将弧度转换为硬件值(0-4095)"""
        # 先转换为角度
        angle_deg = angle_rad * RAD_TO_DEG
        
        # 范围检查
        if angle_deg < -180.0 or angle_deg > 180.0:
            rospy.logwarn("角度值超出范围: %.2f度，会被截断", angle_deg)
            angle_deg = max(-180.0, min(180.0, angle_deg))
        
        # 转换公式: -180° → 0, 0° → 1024, +180° → 2048
        # 实际映射到0-4095
        value = int((angle_deg + 180.0) / 360.0 * 4096)
        
        # 范围限制
        return max(0, min(4095, value))
    
    # def rad_to_hardware_value_grip(self, angle_rad):
    #     """将弧度转换为硬件值(0-4095)"""
    #     # 先转换为角度
    #     angle_deg = angle_rad * RAD_TO_DEG
        
    #     # 范围检查
    #     if angle_deg < 0:
    #         rospy.logwarn("角度值超出范围: %.2f度，会被截断", angle_deg)
    #         angle_deg = 0
    #     elif angle_deg > 100.0:
    #         rospy.logwarn("角度值超出范围: %.2f度，会被截断", angle_deg)
    #         angle_deg = 100.0
        

    #     value = int((angle_deg + 180.0) / 360.0 * 4096)
        
        # 范围限制
        # return max(0, min(4095, value))
    
    def joint_command_callback(self, msg):
        """处理标准关节命令"""
        try:
            if self.debug_mode:
                rospy.loginfo("接收到关节命令: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f], 夹爪: %.2f", 
                             msg.joint1, msg.joint2, msg.joint3, 
                             msg.joint4, msg.joint5, msg.joint6, 
                             msg.gripper)
            
            # 将关节角度放入数组便于处理
            joint_angles = [msg.joint1, msg.joint2, msg.joint3, 
                          msg.joint4, msg.joint5, msg.joint6]
            
            # 映射关节角度到各个舵机
            for servo_idx, (joint_idx, direction) in enumerate(self.joint_to_servo_map):
                # 应用方向系数(有些舵机需要反向)
                servo_angle_rad = joint_angles[joint_idx] * direction
                
                # 转换为硬件值
                hardware_value = self.rad_to_hardware_value(servo_angle_rad)
                
                # 写入到帧数据
                self.servo_angle_frame[3 + servo_idx*2] = hardware_value & 0xFF  # 低字节
                self.servo_angle_frame[3 + servo_idx*2 + 1] = (hardware_value >> 8) & 0xFF  # 高字节
            
            # 计算并设置校验和
            self.servo_angle_frame[-2] = self.calculate_checksum(self.servo_angle_frame)
            
            # # 处理夹爪命令
            # gripper_value = self.rad_to_hardware_value_grip(msg.gripper)
            # self.gripper_frame[4] = gripper_value & 0xFF  # 低字节
            # self.gripper_frame[5] = (gripper_value >> 8) & 0xFF  # 高字节
            # self.gripper_frame[6] = self.calculate_checksum(self.gripper_frame)
            
            # 创建并发送舵机控制消息
            servo_msg = UInt8MultiArray()
            servo_msg.data = self.servo_angle_frame
            self.serial_pub.publish(servo_msg)
            
            # # 创建并发送夹爪控制消息
            # gripper_msg = UInt8MultiArray()
            # gripper_msg.data = self.gripper_frame
            # self.serial_pub.publish(gripper_msg)
            
            if self.debug_mode:
                self._print_debug_info()
                
        except Exception as e:
            rospy.logerr("处理关节命令出错: %s", str(e))
    
    def _print_debug_info(self):
        """打印调试信息"""
        # 打印舵机帧
        servo_hex = " ".join([f"{b:02X}" for b in self.servo_angle_frame])
        rospy.logdebug("舵机帧: %s", servo_hex)
        
        # 打印夹爪帧
        gripper_hex = " ".join([f"{b:02X}" for b in self.gripper_frame])
        rospy.logdebug("夹爪帧: %s", gripper_hex)

    def zero_calib_callback(self, msg):
        """处理零点校准命令"""
        try:
            if self.debug_mode:
                rospy.loginfo("接收到零点校准命令: %d", msg.data)

            if msg.data:
                # 发送零点校准命令
                zero_calib_msg = self.frame_ge(CMD_ZERO_CAL)
                rospy.loginfo("开始零点校准")
            # else:
                
            # zero_calib_msg =self.frame_ge(CMD_ZERO_CAL)
            self.serial_pub.publish(zero_calib_msg)

            
            rospy.loginfo("零点校准完成")
                
        except Exception as e:
            rospy.logerr("处理零点校准命令出错: %s", str(e))
            
    def frame_ge(self, control_cmd, control_data=0x00, check=True):
        frame_d = [0] * 6
        frame_d[0] = FRAME_HEADER
        frame_d[1] = control_cmd
        frame_d[2] = 0x01  # 数据长度
        frame_d[3] = control_data  # 数据内容
        if check:
            frame_d[-2] = self.calculate_checksum(frame_d)
        else:
            frame_d[-2] = 0x00
        frame_d[-1] = FRAME_FOOTER
            
        binary_data = bytearray()
        for byte in frame_d:
            binary_data.append(byte)
            # 创建并发送零点校准消息
        frame_d_msg = UInt8MultiArray()
        frame_d_msg.data = binary_data
        return frame_d_msg
      
    def move_free_callback(self, msg):
        try:
            if self.debug_mode:
                rospy.loginfo("接收到拖动示教模式命令: %d", msg.data)
            if msg.data:
                # 发送拖动示教模式命令
                move_msg = self.frame_ge(CMD_DEMO_CONTROL)
                rospy.loginfo("0力矩设置")
            elif msg.data == False:
                # 发送拖动示教模式命令
                move_msg = self.frame_ge(CMD_DEMO_CONTROL, 0x01, check=True)
                rospy.loginfo("满力矩设置")
            self.serial_pub.publish(move_msg)

            
                
        except Exception as e:
            rospy.logerr("切换拖动示教模式出错: %s", str(e))
            
            
            
    def gripper_control_callback(self, msg):
        """
                    # # 处理夹爪命令
                    # gripper_value = self.rad_to_hardware_value_grip(msg.gripper)
                    # self.gripper_frame[4] = gripper_value & 0xFF  # 低字节
                    # self.gripper_frame[5] = (gripper_value >> 8) & 0xFF  # 高字节
                    # self.gripper_frame[6] = self.calculate_checksum(self.gripper_frame)
                    
                    # # 创建并发送夹爪控制消息
                    # gripper_msg = UInt8MultiArray()
                    # gripper_msg.data = self.gripper_frame
                    # self.serial_pub.publish(gripper_msg)
        """

            
        """单独处理夹爪控制命令"""
        try:
            gripper_rad = msg.data  # 直接获取Float32数据
            
            if self.debug_mode:
                rospy.loginfo("接收到夹爪控制命令: %.2f弧度 (%.2f度)", 
                            gripper_rad, gripper_rad * RAD_TO_DEG)
            
            # 处理夹爪命令
            gripper_value = self.rad_to_hardware_value_grip(gripper_rad)
            self.gripper_frame[4] = gripper_value & 0xFF  # 低字节
            self.gripper_frame[5] = (gripper_value >> 8) & 0xFF  # 高字节
            self.gripper_frame[6] = self.calculate_checksum(self.gripper_frame)
            
            # 创建并发送夹爪控制消息
            gripper_msg = UInt8MultiArray()
            gripper_msg.data = self.gripper_frame
            self.serial_pub.publish(gripper_msg)
            
            if self.debug_mode:
                # 打印夹爪帧
                gripper_hex = " ".join([f"{b:02X}" for b in self.gripper_frame])
                rospy.logdebug("夹爪帧: %s", gripper_hex)
                
        except Exception as e:
            rospy.logerr("处理夹爪命令出错: %s", str(e))

def main():
    """主函数"""
    try:
        node = ServoControlNode()
        rospy.loginfo("机械臂控制节点已启动")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("机械臂控制节点异常: %s", str(e))

if __name__ == '__main__':
    main()