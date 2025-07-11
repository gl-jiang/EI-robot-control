import tkinter as tk
from tkinter import filedialog, ttk
import rosbag
import rospy
from sensor_msgs.msg import JointState
import threading
import tf
from geometry_msgs.msg import PoseStamped


import matplotlib
matplotlib.use('TkAgg')  # 设置 matplotlib 后端为 TkAgg

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # 导入 3D 绘图模块

class RosbagJointPublisher:
    def __init__(self, master):
        self.master = master
        master.title("Rosbag Joint Publisher")

        self.label = tk.Label(master, text="Select a rosbag file to publish joint states")
        self.label.pack()

        self.select_button = tk.Button(master, text="Select rosbag", command=self.select_rosbag)
        self.select_button.pack()

        self.topic_label = tk.Label(master, text="Topics:")
        self.topic_label.pack()

        self.topic_combobox = ttk.Combobox(master, state="readonly")
        self.topic_combobox.pack()

        self.publish_button = tk.Button(master, text="Publish Joints", command=self.publish_joints)
        self.publish_button.pack()

        self.status_label = tk.Label(master, text="")
        self.status_label.pack()

        self.bag = None
        self.topics = []
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        rospy.init_node('rosbag_joint_publisher', anonymous=True)

        self.positions = []
        self.velocities = []
        self.efforts = []
        self.timestamps = []
        self.joint_name=[]

        self.base_frame = "base_link"
        self.end_effector_frame = "link6"  # 替换为你的末端执行器链接名称
        self.listener = tf.TransformListener()
        self.end_effector_positions = []


    def select_rosbag(self):
        file_path = filedialog.askopenfilename(filetypes=[("Bag files", "*.bag")])
        if file_path:
            self.bag = rosbag.Bag(file_path, 'r')
            self.topics = list(self.bag.get_type_and_topic_info()[1].keys())
            self.topic_combobox['values'] = self.topics
            self.topic_combobox.current(0)
            self.status_label.config(text=f"Selected: {file_path}")

    def load_data(self, topic):
        self.positions = []
        self.velocities = []
        self.efforts = []
        self.timestamps = []
        self.joint_name=[]

        with rosbag.Bag(self.bag.filename, 'r') as bag:
            for _, msg, t in bag.read_messages(topics=[topic]):
                #if isinstance(msg, JointState):
                self.positions.append(msg.position)
                self.velocities.append(msg.velocity)
                self.efforts.append(msg.effort)
                self.timestamps.append(t.to_sec())
                self.joint_name.append(msg.name)

    def get_end_effector_pose(self):
        try:
            (trans, _) = self.listener.lookupTransform(self.base_frame, self.end_effector_frame, rospy.Time(0))
            self.end_effector_positions.append(trans)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to get transform: {e}")
    
    def plot_trajectory(self):
        if not self.end_effector_positions:
            self.status_label.config(text="No end effector positions recorded")
            return

        positions = self.end_effector_positions
        x = [p[0] for p in positions]
        y = [p[1] for p in positions]
        z = [p[2] for p in positions]

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # 绘制轨迹
        ax.plot(x, y, z, label='End Effector Trajectory')

        # 标记起点和终点
        ax.scatter(x[0], y[0], z[0], color='r', marker='o', s=100, label='Start Point')  # 起点用圆圈标记
        ax.scatter(x[-1], y[-1], z[-1], color='b', marker='^', s=100, label='End Point')  # 终点用三角形标记

        # 设置坐标轴标签
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        # 设置坐标系原点
        ax.scatter(0, 0, 0, color='g', marker='x')  # 原点用叉号标记

        # 添加图例
        ax.legend()

        # 设置标题
        plt.title('End Effector Trajectory')

        # 显示图像
        plt.show()


    def publish_joints(self):
        if not self.bag:
            self.status_label.config(text="No rosbag selected")
            return

        selected_topic = self.topic_combobox.get()
        if not selected_topic:
            self.status_label.config(text="Please select a topic")
            return

        self.status_label.config(text="Loading data...")

        def publish_thread():
            self.load_data(selected_topic)
            if not self.positions:
                self.status_label.config(text="No joint states found in the selected topic")
                return

            self.status_label.config(text="Publishing joints...")

            for i in range(len(self.positions)):
                msg = JointState()
                msg.header.stamp = rospy.Time.now()
                msg.position = self.positions[i]
                msg.velocity = self.velocities[i]
                msg.effort = self.efforts[i]
                #msg.name = self.joint_name[i]
                msg.name = [f"joint{j+1}" for j in range(len(msg.position))]
                self.joint_pub.publish(msg)
                rospy.sleep(0.005)  # 适当调整发布频率
                # 获取末端执行器位置
                self.get_end_effector_pose()

            self.status_label.config(text="Joints published successfully")
              # 绘制轨迹图
            self.plot_trajectory()

        threading.Thread(target=publish_thread).start()

if __name__ == "__main__":
    root = tk.Tk()
    app = RosbagJointPublisher(root)
    root.mainloop()