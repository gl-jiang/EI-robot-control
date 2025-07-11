#!/usr/bin/env python

import json
import numpy as np
import rospy
from sensor_msgs.msg import JointState
import time

def load_positions_from_json(filename):
    """
    从JSON文件中加载并返回position数据。
    
    参数:
    filename (str): JSON文件的路径。
    
    返回:
    list: 包含position数据的列表。
    """
    with open(filename, 'r') as file:
        data = json.load(file)
        positions = data.get('position', [])
        return positions

def linear_interpolation(start_pos, end_pos, steps):
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
    
    # 使用numpy.linspace在每个维度上进行插值
    interpolated_positions = []
    for i in range(len(start_pos)):
        interpolated_positions.append(
            np.linspace(start_pos[i], end_pos[i], steps).tolist()
        )
    
    # 转置结果，使得每一行代表一个时间点的所有关节位置
    return [list(position) for position in zip(*interpolated_positions)]

def interpolate_and_publish(rc1_filename, rc2_filename, topic_name='/master/joint_left', steps=20, rate_hz=20):
    """
    从两个JSON文件中读取位置，进行线性插值，并发布到指定的ROS话题。
    
    参数:
    rc1_filename (str): 第一个JSON文件的路径。
    rc2_filename (str): 第二个JSON文件的路径。
    topic_name (str, optional): ROS话题名称，默认为'/master/joint_left'。
    steps (int, optional): 插值步数，默认为20。
    rate_hz (int, optional): 发布频率，默认为20 Hz。
    """
    rospy.init_node('joint_position_publisher', anonymous=True)
    pub = rospy.Publisher(topic_name, JointState, queue_size=10)
    rate = rospy.Rate(rate_hz)

    start_positions = load_positions_from_json(rc1_filename)
    end_positions = load_positions_from_json(rc2_filename)
    
    interpolated_positions = linear_interpolation(start_positions, end_positions, steps)

    joint_names = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']  # 假设7个关节
    effort = [0.0] * len(joint_names)  # 努力值可以设置为零或根据需要调整
    velocity = [0.0] * len(joint_names)  # 速度可以设置为零或根据需要调整

    for pos in interpolated_positions:
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = joint_names
        msg.position = pos
        msg.velocity = velocity
        msg.effort = effort
        
        pub.publish(msg)
        print("adj gun pose")
        rate.sleep()

if __name__ == '__main__':
    rc1_filename = 'rc1.json'
    rc2_filename = 'rc2.json'

    try:
        interpolate_and_publish(rc1_filename, rc2_filename)
        print("Interpolation and publishing completed.")
        
    except Exception as e:
        print(f"An error occurred during interpolation or publishing: {e}")