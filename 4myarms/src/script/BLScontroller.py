import os
import scipy.io
import numpy as np
import rospy
import rosbag
from sensor_msgs.msg import JointState


class BLSController:
    def __init__(self,t_step=0.1):
        # 读取训练好的网络参数
        self.param_x = scipy.io.loadmat('ELM_trained_S_X_nospeed_BLS.mat')
        self.X_A1 = self.param_x['AA1']
        self.X_A2 = self.param_x['AA2']
        self.X_beta1 = self.param_x['beta1']
        self.X_beta2 = self.param_x['beta2']
        self.X_b1 = self.param_x['b_1']
        self.X_b2 = self.param_x['b_2']
        self.X_L = int(self.param_x['L'][0])

        self.t_step = t_step

    def cal_control_law(self, error):

        u = 0
        z = np.zeros([1, self.X_L])
        input = error
        for i in range(self.X_L):
            u += self.X_beta1[:, i] @ (2 / (1 + np.exp(-2 * (self.X_A1[:, i].T * input + self.X_b1[:, i]))) - 1)
            z[0, i] = 2 / (1 + np.exp(-2 * (self.X_A1[:, i].T * input + self.X_b1[:, i]))) - 1

        for i in range(self.X_L):
            u += self.X_beta2[:, i] * (2 / (1 + np.exp(-2 * (self.X_A2[:, i].T @ z[:, i] + self.X_b2[:, i]))) - 1)

        return u
    

class Track_control_right:
    def __init__(self):
        rospy.init_node('Track_control_right', anonymous=True)
        self.joint_pose_pub = rospy.Publisher('/master/joint_right', JointState, queue_size=10)
        self.joint_pose_sub = rospy.Subscriber('/puppet/joint_right', JointState, self.robot_pose_callback) 
        self.frequency = 20
        self.t_step = 1/self.frequency
        self.BLS_controller = BLSController()

    def load_bag(bag_file,filepath = '../bag'):
        fullbag_file = os.path.join(filepath, bag_file)
        joint_angle_desired = []
        joint_velocity_desired = []
        with rosbag.Bag(fullbag_file, 'r') as bag:
            for topic, msg, t in bag.read_messages():
                # print(f"主题: {topic}")
                # print(f"时间戳: {t.to_sec()}")
                # print(f"消息类型: {type(msg).__name__}")
                # print(f"消息内容: {msg}\n")
                # print(type(msg))
                joint_angle_desired.append(msg.position[0:6])
                joint_velocity_desired.append(msg.velocity[0:6])
        
        return joint_angle_desired,joint_velocity_desired
    
    def track_control(self,bag_file):
         joint_angle_desired = self.load_bag(bag_file)
         joint_velocity_desired = self.load_bag(bag_file)
         rate = rospy.Rate(self.frequency)
         while not rospy.is_shutdown():
            i = 0
            j0 = self.joint_pose_sub.position[0]
            j1 = self.joint_pose_sub.position[1]
            j2 = self.joint_pose_sub.position[2]
            j3 = self.joint_pose_sub.position[3]
            j4 = self.joint_pose_sub.position[4]
            j5 = self.joint_pose_sub.position[5]


            e0 = j0 - joint_angle_desired[i,0]
            e1 = j1 - joint_angle_desired[i,1]
            e2 = j2 - joint_angle_desired[i,2]
            e3 = j3 - joint_angle_desired[i,3]
            e4 = j4 - joint_angle_desired[i,4]
            e5 = j5 - joint_angle_desired[i,5]

            u0 = BLSController.cal_control_law(self,e0)
            u1 = BLSController.cal_control_law(self,e1)
            u2 = BLSController.cal_control_law(self,e2)
            u3 = BLSController.cal_control_law(self,e3)
            u4 = BLSController.cal_control_law(self,e4)
            u5 = BLSController.cal_control_law(self,e5)
            
            j0_v = joint_velocity_desired[i,0] - u0
            j1_v = joint_velocity_desired[i,1] - u1
            j2_v = joint_velocity_desired[i,2] - u2
            j3_v = joint_velocity_desired[i,3] - u3
            j4_v = joint_velocity_desired[i,4] - u4
            j5_v = joint_velocity_desired[i,5] - u5

            joint_angles = np.array([j0+j0_v*self.t_step,j1+j1_v*self.t_step,j2+j2_v*self.t_step,j3+j3_v*self.t_step,j4+j4_v*self.t_step,j5+j5_v*self.t_step])
            msg = JointState()
            msg.header.stamp = rospy.Time.now() 
            num_joint = len(joint_angles) 
            msg.name = [f'joint{i}' for i in range(num_joint)]

            
            msg.position = joint_angles  

            self.joint_pose_pub.publish(msg)

            i += 1 
            if i == len(joint_angle_desired):   
                break
            rate.sleep()
                
class Track_control_left:
    def __init__(self):
        rospy.init_node('Track_control_left', anonymous=True)
        self.joint_pose_pub = rospy.Publisher('/master/joint_left', JointState, queue_size=10)
        self.joint_pose_sub = rospy.Subscriber('/puppet/joint_left', JointState, self.robot_pose_callback) 
        self.frequency = 20
        self.t_step = 1/self.frequency
        self.BLS_controller = BLSController()

    def load_bag(bag_file,filepath = '../bag'):
        fullbag_file = os.path.join(filepath, bag_file)
        joint_angle_desired = []
        joint_velocity_desired = []
        with rosbag.Bag(fullbag_file, 'r') as bag:
            for topic, msg, t in bag.read_messages():
                # print(f"主题: {topic}")
                # print(f"时间戳: {t.to_sec()}")
                # print(f"消息类型: {type(msg).__name__}")
                # print(f"消息内容: {msg}\n")
                # print(type(msg))
                joint_angle_desired.append(msg.position[0:6])
                joint_velocity_desired.append(msg.velocity[0:6])
        
        return joint_angle_desired,joint_velocity_desired
    
    def track_control(self,bag_file):
         joint_angle_desired = self.load_bag(bag_file)
         joint_velocity_desired = self.load_bag(bag_file)
         rate = rospy.Rate(self.frequency)
         while not rospy.is_shutdown():
            i = 0
            j0 = self.joint_pose_sub.position[0]
            j1 = self.joint_pose_sub.position[1]
            j2 = self.joint_pose_sub.position[2]
            j3 = self.joint_pose_sub.position[3]
            j4 = self.joint_pose_sub.position[4]
            j5 = self.joint_pose_sub.position[5]


            e0 = j0 - joint_angle_desired[i,0]
            e1 = j1 - joint_angle_desired[i,1]
            e2 = j2 - joint_angle_desired[i,2]
            e3 = j3 - joint_angle_desired[i,3]
            e4 = j4 - joint_angle_desired[i,4]
            e5 = j5 - joint_angle_desired[i,5]

            u0 = BLSController.cal_control_law(self,e0)
            u1 = BLSController.cal_control_law(self,e1)
            u2 = BLSController.cal_control_law(self,e2)
            u3 = BLSController.cal_control_law(self,e3)
            u4 = BLSController.cal_control_law(self,e4)
            u5 = BLSController.cal_control_law(self,e5)
            
            j0_v = joint_velocity_desired[i,0] - u0
            j1_v = joint_velocity_desired[i,1] - u1
            j2_v = joint_velocity_desired[i,2] - u2
            j3_v = joint_velocity_desired[i,3] - u3
            j4_v = joint_velocity_desired[i,4] - u4
            j5_v = joint_velocity_desired[i,5] - u5

            joint_angles = np.array([j0+j0_v*self.t_step,j1+j1_v*self.t_step,j2+j2_v*self.t_step,j3+j3_v*self.t_step,j4+j4_v*self.t_step,j5+j5_v*self.t_step])
            msg = JointState()
            msg.header.stamp = rospy.Time.now() 
            num_joint = len(joint_angles) 
            msg.name = [f'joint{i}' for i in range(num_joint)]

            
            msg.position = joint_angles  

            self.joint_pose_pub.publish(msg)

            i += 1 
            if i == len(joint_angle_desired):   
                break
            rate.sleep()
