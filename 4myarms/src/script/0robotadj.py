import tkinter as tk
from tkinter import ttk
import rospy
from sensor_msgs.msg import JointState
import threading
from threading import Lock

class JointStateDisplay:
    def __init__(self, master):
        self.master = master
        self.master.title("Joint State Display")
        self.master.geometry('800x600')  # Set window size
        self.master.resizable(False, False)  # Disable resizing

        # Initialize ROS node and subscribers/publishers
        rospy.init_node('joint_state_display', anonymous=True)
        self.sub_left = rospy.Subscriber('/puppet/joint_left', JointState, self.left_callback)
        self.sub_right = rospy.Subscriber('/puppet/joint_right', JointState, self.right_callback)
        self.pub_left = rospy.Publisher('/master/joint_left', JointState, queue_size=10)
        self.pub_right = rospy.Publisher('/master/joint_right', JointState, queue_size=10)

        # Store joint states and efforts with a lock for thread-safe access
        self.lock = Lock()
        self.joint_states = {
            'left': {'position': [0.0] * 7, 'effort': [0.0] * 7},
            'right': {'position': [0.0] * 7, 'effort': [0.0] * 7},
        }

        # Create UI elements for both arms with fixed sizes and spacing
        self.create_ui()

        # Start a separate thread to update the GUI
        self.update_gui_thread = threading.Thread(target=self.update_gui_periodically)
        self.update_gui_thread.daemon = True
        self.update_gui_thread.start()

    def create_ui(self):
        padding = {'padx': 10, 'pady': 5}  # Padding for widgets

        # Create labels for joint names in the first column
        for i in range(7):
            joint_label = ttk.Label(self.master, text=f"Joint {i+1}", width=10)
            joint_label.grid(column=0, row=i, sticky=tk.W, **padding)

        # Initialize joint_vars dictionary here to ensure it's accessible in update_gui_from_joint_states
        self.joint_vars = {}

        # Create UI for left arm in columns 1-3
        self.create_arm_ui('left', column=1, padding=padding)
        # Create UI for right arm in columns 4-6
        self.create_arm_ui('right', column=4, padding=padding)

    def create_arm_ui(self, side, column, padding):
        for i in range(7):
            # Position label with fixed width
            position_var = tk.StringVar()
            position_label = ttk.Label(self.master, textvariable=position_var, width=15, anchor='e')
            position_label.grid(column=column, row=i, **padding)
            self.joint_vars[(side, i)] = position_var  # Ensure this is added to the main dict

            # Increase button with fixed width
            inc_button = ttk.Button(self.master, text="+", width=5, command=lambda s=side, idx=i: self.publish_joint_change(s, idx, +0.1))
            inc_button.grid(column=column+1, row=i, **padding)

            # Decrease button with fixed width
            dec_button = ttk.Button(self.master, text="-", width=5, command=lambda s=side, idx=i: self.publish_joint_change(s, idx, -0.1))
            dec_button.grid(column=column+2, row=i, **padding)

    def left_callback(self, msg):
        with self.lock:
            self.joint_states['left']['position'] = list(msg.position)
            self.joint_states['left']['effort'] = list(msg.effort)

    def right_callback(self, msg):
        with self.lock:
            self.joint_states['right']['position'] = list(msg.position)
            self.joint_states['right']['effort'] = list(msg.effort)

    def update_gui_periodically(self):
        rate = rospy.Rate(10)  # Update GUI at 10 Hz
        while not rospy.is_shutdown():
            self.update_gui_from_joint_states()
            rate.sleep()

    def update_gui_from_joint_states(self):
        with self.lock:
            for side in ['left', 'right']:
                for i in range(7):
                    value = self.joint_states[side]['position'][i]
                    widget_info = self.joint_vars.get((side, i))
                    if widget_info:
                        widget_info.set(f"{value:.3f}")
                    else:
                        print(f"Warning: No widget found for side {side}, index {i}")

    def publish_joint_change(self, side, index, delta):
        with self.lock:
            current_positions = self.joint_states[side]['position'].copy()
            current_efforts = self.joint_states[side]['effort']

        # Update the specific joint's position
        new_position = current_positions[index] + delta

        # Create a copy of the positions to send only the changed one
        updated_positions = current_positions.copy()
        updated_positions[index] = new_position

        rate = rospy.Rate(20)  # 20 Hz
        for _ in range(20):  # Publish 20 times
            joint_msg = JointState()
            joint_msg.header.stamp = rospy.Time.now()
            joint_msg.name = [f"joint_{i+1}" for i in range(7)]  # Assuming joint names are joint_1 to joint_7
            joint_msg.position = updated_positions
            joint_msg.effort = current_efforts  # Use stored effort values

            pub = self.pub_left if side == 'left' else self.pub_right
            pub.publish(joint_msg)
            rate.sleep()

if __name__ == '__main__':
    root = tk.Tk()
    app = JointStateDisplay(root)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        rospy.signal_shutdown("Program interrupted by user.")