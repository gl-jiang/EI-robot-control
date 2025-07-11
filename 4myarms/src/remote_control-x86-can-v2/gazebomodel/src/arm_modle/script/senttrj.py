#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def send_trajectory():
    rospy.init_node('trajectory_sender', anonymous=True)
    pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    # 定义关节名称
    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    # 创建一个关节轨迹消息
    traj = JointTrajectory()
    traj.joint_names = joint_names

    # 添加一个轨迹点
    point = JointTrajectoryPoint()
    point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 初始位置
    point.time_from_start = rospy.Duration(1.0)
    traj.points.append(point)

    # 添加另一个轨迹点
    point = JointTrajectoryPoint()
    point.positions = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]  # 目标位置
    point.time_from_start = rospy.Duration(5.0)
    traj.points.append(point)

    # 发布轨迹
    pub.publish(traj)
    rospy.loginfo("Trajectory sent")

if __name__ == '__main__':
    try:
        send_trajectory()
    except rospy.ROSInterruptException:
        pass