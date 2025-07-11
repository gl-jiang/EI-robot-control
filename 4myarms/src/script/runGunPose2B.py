#!/usr/bin/env python

import json
import numpy as np
import rospy
from sensor_msgs.msg import JointState
import time
import argparse

class JointInterpolator:
    def __init__(self, rc1_filename, subscribe_topic_name='/puppet/joint_left', publish_topic_name='/master/joint_left'):
        self.rc1_filename = rc1_filename
        self.subscribe_topic_name = subscribe_topic_name
        self.publish_topic_name = publish_topic_name
        self.position_received = False
        self.position1 = None
        self.position2, self.effort = self.load_positions_and_effort_from_json()
        self.pub = rospy.Publisher(self.publish_topic_name, JointState, queue_size=10)
        self.sub = rospy.Subscriber(self.subscribe_topic_name, JointState, self.callback)

    def load_positions_and_effort_from_json(self):
        """
        从JSON文件中加载并返回position和effort数据。
        
        返回:
        tuple: 包含position和effort数据的元组。
        """
        with open(self.rc1_filename, 'r') as file:
            data = json.load(file)
            positions = data.get('position', [])
            effort = data.get('effort', [0.0] * len(positions))  # 如果没有effort字段，默认为零
            return positions, effort

    def linear_interpolation(self, start_pos, end_pos, steps):
        """
        在start_pos和end_pos之间进行线性插值。
        
        参数:
        start_pos (list): 初始位置列表。
        end_pos (list): 目标位置列表。
        steps (int): 插值步数。
        
        返回:
        list of lists: 包含插值后的所有位置的列表。
        """
        if len(start_pos) != len(end_pos):
            raise ValueError("Start and end positions must have the same length.")
        
        interpolated_positions = []
        for i in range(len(start_pos)):
            interpolated_positions.append(
                np.linspace(start_pos[i], end_pos[i], steps).tolist()
            )
        
        return [list(position) for position in zip(*interpolated_positions)]

    def interpolate_and_publish(self, position1, position2, effort, steps=20, rate_hz=20):
        """
        对两个位置进行线性插值，并发布到指定的ROS话题。
        
        参数:
        position1 (list): 初始位置列表。
        position2 (list): 目标位置列表。
        effort (list): 努力值列表。
        steps (int, optional): 插值步数，默认为20。
        rate_hz (int, optional): 发布频率，默认为20 Hz。
        """
        rate = rospy.Rate(rate_hz)
        interpolated_positions = self.linear_interpolation(position1, position2, steps)

        joint_names = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']  # 假设7个关节
        velocity = [0.0] * len(joint_names)  # 速度可以设置为零或根据需要调整

        for pos in interpolated_positions:
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = joint_names
            msg.position = pos
            msg.velocity = velocity
            msg.effort = effort  # 使用从文件中读取的effort值
            
            self.pub.publish(msg)
            rate.sleep()

    def callback(self, data):
        """
        订阅回调函数，接收实时位置并触发插值和发布。
        """
        if not self.position_received:
            self.position_received = True
            self.position1 = list(data.position)
            rospy.loginfo("Initial position received from topic %s", self.subscribe_topic_name)
            
            # 确保关节数量一致
            if len(self.position1) != len(self.position2) or len(self.position1) != len(self.effort):
                rospy.logerr("The number of joints in the received message and the file do not match.")
                return
            
            print("Interpolating from real-time position to file position...")
            self.interpolate_and_publish(self.position1, self.position2, self.effort)
            rospy.signal_shutdown("Interpolation and publishing completed.")

def main():
    parser = argparse.ArgumentParser(description="Joint interpolator ROS node")
    parser.add_argument('rc1_filename', type=str, help='Path to the JSON file containing target positions and efforts')
    args = parser.parse_args()

    rospy.init_node('joint_position_subscriber', anonymous=True)
    
    interpolator = JointInterpolator(args.rc1_filename)

    try:
        rospy.spin()  # Keep the node running until it's shut down
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()