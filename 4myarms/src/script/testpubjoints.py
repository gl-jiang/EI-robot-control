#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import sys

def publish_joint_states():
    rospy.init_node('joint_state_publisher_example', anonymous=True)
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pub.publish(joint_state)
        rate.sleep()
        print("pub!")

if __name__ == '__main__':
    try:
        publish_joint_states()
    except rospy.ROSInterruptException:
        pass