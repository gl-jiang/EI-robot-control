#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState  # 使用正确的消息类型
import json  # 用于保存为JSON格式到文件
import sys
import os

class MessageRecorder:
    def __init__(self, topic_name, filename):
        self.topic_name = topic_name
        self.filename = filename
        self.message_received = False
        rospy.init_node('joint_left_listener', anonymous=True)
        self.sub = rospy.Subscriber(topic_name, JointState, self.callback)

    def callback(self, data):
        if not self.message_received:
            message_data = {
                'header': {
                    'seq': data.header.seq,
                    'stamp': {
                        'secs': data.header.stamp.secs,
                        'nsecs': data.header.stamp.nsecs
                    },
                    'frame_id': data.header.frame_id
                },
                'name': list(data.name),
                'position': list(data.position),
                'velocity': list(data.velocity),
                'effort': list(data.effort)
            }

            with open(self.filename, 'w') as file:
                json.dump(message_data, file, indent=4)

            print(f"Message saved to {self.filename}")
            self.message_received = True
            rospy.signal_shutdown('Message saved, shutting down.')

    def spin_once(self):
        rate = rospy.Rate(10)  # 设置一个适当的频率来检查是否已经接收到消息
        while not rospy.is_shutdown() and not self.message_received:
            rate.sleep()

        # 强制退出Python脚本以确保子进程立即终止
        os._exit(0)

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: ros_recorder.py <topic_name> <output_filename>")
        sys.exit(1)

    topic_name = sys.argv[1]
    output_filename = sys.argv[2]

    recorder = MessageRecorder(topic_name, output_filename)
    recorder.spin_once()