#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from robot import *

# 定义阈值A和B
threshold_A = 6
threshold_B = 2

# 标记是否已经执行过打开或关闭夹爪的动作
has_opened_gripper = False
has_closed_gripper = False

def open_gripper_1():
    global has_opened_gripper,has_closed_gripper
    if not has_opened_gripper:
        rospy.loginfo("Opening gripper.")

        # 在这里添加打开夹爪的具体逻辑
        open_gripper()
        has_opened_gripper = True
        has_closed_gripper = False

def close_gripper_1():
    global has_closed_gripper,has_opened_gripper
    if not has_closed_gripper:
        rospy.loginfo("Closing gripper.")
        # 在这里添加关闭夹爪的具体逻辑
        close_gripper()
        has_closed_gripper = True
        has_opened_gripper = False

def callback(data):
    global has_opened_gripper, has_closed_gripper

    # 检查position[6]的值
    position_6 = data.position[6]
  #  print("claw is")
  #  print(position_6)

    # 当首次大于阈值A时，打开夹爪
    if position_6 > threshold_A and not has_opened_gripper:
        open_gripper_1()

    # 当首次小于阈值B时，关闭夹爪
    if position_6 < threshold_B and not has_closed_gripper:
        close_gripper_1()

def listener():
    # 初始化ROS节点
    rospy.init_node('gripper_controller', anonymous=True)

    # 创建订阅者
    rospy.Subscriber('/master/joint_right', JointState, callback)

    # 防止节点退出
    rospy.spin()

if __name__ == '__main__':
    listener()