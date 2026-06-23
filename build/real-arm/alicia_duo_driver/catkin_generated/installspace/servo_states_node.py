#!/usr/bin/env python3
# coding=utf-8

"""
舵机状态处理节点
接收舵机和夹爪原始数据，转换为标准弧度单位并发布标准关节状态消息。
"""

import rospy
import math
import time
import numpy as np
from std_msgs.msg import UInt8MultiArray, Float32MultiArray, Int32
from alicia_duo_driver.msg import ArmJointState

# 常量定义
DEG_TO_RAD = math.pi / 180.0  # 角度转弧度系数
RAD_TO_DEG = 180.0 / math.pi  # 弧度转角度系数

def u8_array_to_rad(u8_array):
    """将原始字节数据转换为弧度值"""
    try:
        if len(u8_array) != 2:
            rospy.logwarn("数据长度错误：需要2个字节，实际%d个字节", len(u8_array))
            return 0.0
        
        # 构造16位整数
        hex_value = (u8_array[0] & 0xFF) | ((u8_array[1] & 0xFF) << 8)
        
        # 值范围检查
        if hex_value < 0 or hex_value > 4095:
            rospy.logwarn("舵机值超出范围: %d (有效范围0-4095)", hex_value)
            hex_value = max(0, min(hex_value, 4095))
        
        # 转换为角度: -180到+180度
        angle_deg = -180.0 + (hex_value / 2048.0) * 180.0
        
        # 转换为弧度并返回
        return angle_deg * DEG_TO_RAD
        
    except Exception as e:
        rospy.logerr("字节转换异常: %s", str(e))
        return 0.0

class ServoStatesNode:
    def __init__(self):
        """初始化舵机状态节点"""
        rospy.init_node('servo_states_node')
        
        # 配置参数
        self.debug_mode = rospy.get_param('~debug_mode', False)
        self.servo_count = rospy.get_param('~servo_count', 9)
        self.rate_limit = rospy.get_param('~rate_limit', 0.001)  # 节流限制(秒)
        
        # 派生配置
        self.servo_id_min = 0
        self.servo_id_max = self.servo_count
        self.joint_count = 6  # 有效关节数
        
        # 数据存储
        self.servo_angles = np.zeros(self.servo_count, dtype=np.float32)  # 所有舵机角度(弧度)
        self.gripper_angle_rad = 0.0  # 夹爪角度(弧度)
        
        # 关节映射 - 舵机到标准关节的映射
        # 特殊处理:
        # - 舵机1和2同为关节1
        # - 舵机3和4为关节2(舵机4是反向)
        # - 舵机5和6为关节3(舵机6是反向)
        self.servo_to_joint_map = {
            0: (0, 1.0),    # 舵机1 -> 关节1 (正向)
            1: None,        # 舵机2 -> 忽略(重复)
            2: (1, 1.0),    # 舵机3 -> 关节2 (正向)
            3: None,        # 舵机4 -> 忽略(重复反向)
            4: (2, 1.0),    # 舵机5 -> 关节3 (正向)
            5: None,        # 舵机6 -> 忽略(重复反向)
            6: (3, 1.0),    # 舵机7 -> 关节4 (正向)
            7: (4, 1.0),    # 舵机8 -> 关节5 (正向)
            8: (5, 1.0),    # 舵机9 -> 关节6 (正向)
        }
        
        # 时间戳 - 用于数据节流
        self._last_process_time = 0
        
        # 创建发布者和订阅者
        self._setup_ros_interface()
        
        rospy.loginfo("舵机状态节点已初始化 (单位: 弧度)")
        
    def _setup_ros_interface(self):
        """设置ROS接口"""
        # 标准关节状态发布者
        self.joint_state_pub = rospy.Publisher('/arm_joint_state', ArmJointState, queue_size=10)
        
        # 向后兼容的发布者
        self.array_pub = rospy.Publisher('/servo_states_main', Float32MultiArray, queue_size=10)
        
        # 订阅者
        self.servo_sub = rospy.Subscriber('/servo_states', UInt8MultiArray, 
                                      self.servo_states_callback, queue_size=10)
        self.gripper_sub = rospy.Subscriber('/gripper_angle', Int32, 
                                       self.gripper_angle_callback, queue_size=10)
    
    def _should_process(self):
        """检查是否应该处理当前数据(节流控制)"""
        current_time = time.time()
        if current_time - self._last_process_time >= self.rate_limit:
            self._last_process_time = current_time
            return True
        return False
    
    def gripper_angle_callback(self, msg):
        """处理夹爪角度数据"""
        try:
            # 获取原始舵机值
            servo_value = int(msg.data)
            
            # 范围检查
            if servo_value < 2048 or servo_value > 2900:
                # rospy.logwarn("夹爪舵机值超出范围: %d (有效范围2048-2900)", servo_value)
                servo_value = max(2048, min(servo_value, 2900))
            
            # 转换为角度 (0-100度)
            angle_deg = (servo_value - 2048) / 8.52
            
            # 转换为弧度
            self.gripper_angle_rad = angle_deg * DEG_TO_RAD
            
            if self.debug_mode:
                rospy.logdebug("夹爪舵机值: %d, 转换为角度: %.2f度 (%.4f弧度)", 
                            servo_value, angle_deg, self.gripper_angle_rad)
        except Exception as e:
            rospy.logerr("处理夹爪角度数据异常: %s", str(e))
    # def gripper_angle_callback(self, msg):
    #     """处理夹爪角度数据"""
    #     # 角度到弧度的转换
    #     self.gripper_angle_rad = float(msg.data) #* DEG_TO_RAD
        
    #     if self.debug_mode:
    #         rospy.logdebug("夹爪角度: %.2f度 (%.4f弧度)", msg.data, self.gripper_angle_rad)
    
    def servo_states_callback(self, msg):
        """处理舵机状态数据"""
        # 节流控制
        if not self._should_process():
            return
            
        try:
            # 基本数据验证
            if len(msg.data) < 3:
                rospy.logwarn("舵机数据帧过短")
                return
                
            # 检查舵机数量
            expected_count = self.servo_count
            actual_count = msg.data[2] / 2
            
            if actual_count != expected_count:
                rospy.logwarn("舵机数量不匹配: 期望%d个, 实际%.1f个", 
                             expected_count, actual_count)
                return
            
            # 处理每个舵机数据
            for i in range(self.servo_id_min, self.servo_id_max):
                # 数据索引计算
                byte_idx = 3 + i * 2
                if byte_idx + 1 >= len(msg.data):
                    rospy.logwarn("舵机数据越界: 索引%d超出范围", byte_idx)
                    continue
                
                # 转换为弧度值
                self.servo_angles[i] = u8_array_to_rad(
                    msg.data[byte_idx:byte_idx+2])
            
            # 创建标准关节状态消息
            joint_state = ArmJointState()
            joint_state.header.stamp = rospy.Time.now()
            
            # 映射舵机数据到关节
            joint_values = [0.0] * self.joint_count
            
            for servo_idx, mapping in self.servo_to_joint_map.items():
                if mapping is not None:
                    joint_idx, direction = mapping
                    joint_values[joint_idx] = self.servo_angles[servo_idx] * direction
            
            # 填充消息字段
            joint_state.joint1 = joint_values[0]
            joint_state.joint2 = joint_values[1]
            joint_state.joint3 = joint_values[2]
            joint_state.joint4 = joint_values[3]
            joint_state.joint5 = joint_values[4]
            joint_state.joint6 = joint_values[5]
            joint_state.gripper = self.gripper_angle_rad
            
            # 发布关节状态
            self.joint_state_pub.publish(joint_state)
            
            # 向后兼容 - 发布为数组
            compat_msg = Float32MultiArray()
            compat_data = joint_values + [self.gripper_angle_rad]
            compat_msg.data = compat_data
            self.array_pub.publish(compat_msg)
            
            if self.debug_mode:
                degrees = [rad * RAD_TO_DEG for rad in joint_values]
                rospy.logdebug("关节角度: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]度, 夹爪: %.2f度",
                              degrees[0], degrees[1], degrees[2], 
                              degrees[3], degrees[4], degrees[5],
                              self.gripper_angle_rad * RAD_TO_DEG)
                
        except Exception as e:
            rospy.logerr("处理舵机状态异常: %s", str(e))

def main():
    """主函数"""
    try:
        node = ServoStatesNode()
        rospy.loginfo("舵机状态节点已启动")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("舵机状态节点异常: %s", str(e))

if __name__ == '__main__':
    main()