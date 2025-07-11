#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped

class EndEffectorPosition:
    def __init__(self):
        rospy.init_node('end_effector_position', anonymous=True)
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        self.tf_listener = tf.TransformListener()
        self.end_effector_link = 'link6'  # 替换为你的末端执行器链接名
        self.base_link = 'base_link'  # 替换为你的基座链接名

    def joint_state_callback(self, msg):
        try:
            # 等待tf缓冲区中有足够的数据
            self.tf_listener.waitForTransform(self.base_link, self.end_effector_link, rospy.Time(), rospy.Duration(4.0))
            # 获取末端执行器在基座坐标系下的位置
            (trans, rot) = self.tf_listener.lookupTransform(self.base_link, self.end_effector_link, rospy.Time(0))
            rospy.loginfo("End effector position: x=%f, y=%f, z=%f", trans[0], trans[1], trans[2])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("TF Exception: %s", str(e))

if __name__ == '__main__':
    try:
        node = EndEffectorPosition()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass