import tkinter as tk
from tkinter import filedialog, messagebox
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import rospy
from sensor_msgs.msg import JointState

class RosbagEditor:
    def __init__(self, root):
        self.root = root
        self.root.title("Rosbag Editor")
        
        # Initialize ROS node and publisher
        rospy.init_node('rosbag_editor_node', anonymous=True)
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
        self.bag_path = None
        self.topic_data = None
        
        self.load_button = tk.Button(root, text="Load Rosbag", command=self.load_rosbag)
        self.load_button.pack(pady=10)
        
        self.slider = tk.Scale(root, from_=0, to=100, orient=tk.HORIZONTAL, length=400, resolution=1, command=self.update_time)
        self.slider.set(0)
        self.slider.pack(pady=20)
        
        self.time_label = tk.Label(root, text="Time: 0.0s")
        self.time_label.pack()
        
        self.fig, self.ax = plt.subplots(figsize=(6, 3))
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
    def load_rosbag(self):
        self.bag_path = filedialog.askopenfilename(filetypes=[("ROSBAG files", "*.bag")])
        if not self.bag_path:
            return
        
        bag = rosbag.Bag(self.bag_path)
        topics = bag.get_type_and_topic_info()[1].keys()
        
        if len(topics) != 1:
            messagebox.showerror("Error", "The rosbag must contain exactly one topic.")
            return
        
        self.topic_name = list(topics)[0]
        self.topic_data = []
        
        for _, msg, t in bag.read_messages(topics=[self.topic_name]):
            timestamp = t.to_sec()
            joint_angles = [msg.position[i] for i in range(6)]
            self.topic_data.append((timestamp, joint_angles))
        
        bag.close()
        
        self.topic_data.sort(key=lambda x: x[0])
        timestamps = [data[0] for data in self.topic_data]
        min_time = min(timestamps)
        max_time = max(timestamps)
        
        self.slider.config(from_=min_time, to=max_time, resolution=(max_time - min_time) / 1000)
        self.plot_joint_angles()
    
    def update_time(self, value):
        time_value = float(value)
        self.time_label.config(text=f"Time: {time_value:.2f}s")
        selected_data = next(data for data in self.topic_data if data[0] >= time_value)
        positions = selected_data[1]
        velocities = [0.0] * len(positions)  # Assuming zero velocity for simplicity
        efforts = [0.0] * len(positions)     # Assuming zero effort for simplicity
        
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.position = positions
        msg.velocity = velocities
        msg.effort = efforts
        msg.name = [f"joint{j+1}" for j in range(len(msg.position))]
        
        self.joint_pub.publish(msg)
        print(f"Joint Angles at {selected_data[0]:.2f}s: {selected_data[1]}")
    
    def plot_joint_angles(self):
        timestamps = [data[0] for data in self.topic_data]
        joint_angles = np.array([data[1] for data in self.topic_data]).T
        
        self.ax.clear()
        labels = ['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6']
        for i, angles in enumerate(joint_angles):
            self.ax.plot(timestamps, angles, label=labels[i])
        
        self.ax.legend()
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Joint Angle')
        self.ax.set_title('Joint Angles Over Time')
        self.canvas.draw()

if __name__ == "__main__":
    root = tk.Tk()
    app = RosbagEditor(root)
    root.mainloop()






