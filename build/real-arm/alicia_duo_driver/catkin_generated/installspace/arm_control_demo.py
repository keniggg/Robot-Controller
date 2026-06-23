#!/usr/bin/env python3
# coding=utf-8

"""
机械臂控制接口测试示例
演示如何使用标准关节接口读取和控制机械臂
"""

import rospy
import math
import time
from alicia_duo_driver.msg import ArmJointState

class ArmControlDemo:
    def __init__(self):
        """初始化示例节点"""
        rospy.init_node('arm_control_demo')
        
        # 存储当前关节状态
        self.current_joints = ArmJointState()
        self.joints_received = False
        
        # 创建订阅者 - 接收机械臂状态
        self.state_sub = rospy.Subscriber(
            '/arm_joint_state', ArmJointState, self.joint_state_callback, queue_size=10)
        
        # 创建发布者 - 发送控制命令
        self.cmd_pub = rospy.Publisher(
            '/arm_joint_command', ArmJointState, queue_size=10)
        
        # 等待连接建立
        rospy.sleep(1.0)
        rospy.loginfo("机械臂控制示例已初始化")
    
    def joint_state_callback(self, msg):
        """处理关节状态消息"""
        self.current_joints = msg
        self.joints_received = True
        
    def wait_for_state(self, timeout=5.0):
        """等待接收关节状态"""
        start_time = time.time()
        rate = rospy.Rate(10)  # 10Hz
        
        self.joints_received = False
        while not self.joints_received and time.time() - start_time < timeout:
            rate.sleep()
            
        return self.joints_received
    
    def print_current_state(self):
        """打印当前关节状态"""
        if not self.joints_received:
            rospy.logwarn("未接收到关节状态")
            return
            
        # 将弧度转换为角度以便阅读
        j1 = math.degrees(self.current_joints.joint1)
        j2 = math.degrees(self.current_joints.joint2)
        j3 = math.degrees(self.current_joints.joint3)
        j4 = math.degrees(self.current_joints.joint4)
        j5 = math.degrees(self.current_joints.joint5)
        j6 = math.degrees(self.current_joints.joint6)
        grip = math.degrees(self.current_joints.gripper)
        
        rospy.loginfo("当前关节状态(角度):")
        rospy.loginfo("关节1: %.2f°", j1)
        rospy.loginfo("关节2: %.2f°", j2)
        rospy.loginfo("关节3: %.2f°", j3)
        rospy.loginfo("关节4: %.2f°", j4)
        rospy.loginfo("关节5: %.2f°", j5)
        rospy.loginfo("关节6: %.2f°", j6)
        rospy.loginfo("夹爪: %.2f°", grip)
    
    def send_command(self, j1=0.0, j2=0.0, j3=0.0, j4=0.0, j5=0.0, j6=0.0, grip=0.0, wait_time=2.0):
        """发送控制命令"""
        cmd = ArmJointState()
        cmd.header.stamp = rospy.Time.now()
        
        # 将角度转换为弧度
        cmd.joint1 = math.radians(j1)
        cmd.joint2 = math.radians(j2)
        cmd.joint3 = math.radians(j3)
        cmd.joint4 = math.radians(j4)
        cmd.joint5 = math.radians(j5)
        cmd.joint6 = math.radians(j6)
        cmd.gripper = math.radians(grip)
        
        rospy.loginfo("发送命令(角度): [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f], 夹爪: %.1f", 
                    j1, j2, j3, j4, j5, j6, grip)
        
        # 发送命令
        self.cmd_pub.publish(cmd)
        
        # 等待指定时间
        if wait_time > 0:
            rospy.sleep(wait_time)
    
    def run_demo(self):
        """运行演示序列"""
        # 检查当前状态
        if self.wait_for_state():
            rospy.loginfo("成功接收到机械臂状态")
            self.print_current_state()
        else:
            rospy.logwarn("未能接收到机械臂状态，继续演示...")
        
        # 演示序列
        try:
            # 1. 回到原点
            rospy.loginfo("=== 测试1: 回到原点 ===")
            self.send_command(0, 0, 0, 0, 0, 0, 0)
            
            # 2. 测试关节1
            rospy.loginfo("=== 测试2: 关节1移动 ===")
            self.send_command(30, 0, 0, 0, 0, 0, 0)
            self.send_command(30, 0, 0, 0, 0, 0, 0)
            # # 3. 测试关节2
            # rospy.loginfo("=== 测试3: 关节2移动 ===")
            
            # self.send_command(30, 10, 0, 0, 0, 0, 0)
            # self.send_command(0, 5, 0, 0, 0, 0, 0)
            # self.send_command(0, 0, 0, 0, 0, 0, 0)

            rospy.loginfo("演示完成!")
            
        except KeyboardInterrupt:
            rospy.loginfo("演示被用户中断")
            
        except Exception as e:
            rospy.logerr("演示出错: %s", str(e))
            
        finally:
            # 最后回到安全位置
            self.send_command(0, 0, 0, 0, 0, 0, 0)

def main():
    """主函数"""
    try:
        demo = ArmControlDemo()
        demo.run_demo()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()