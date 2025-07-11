import rosbag
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import make_interp_spline
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import rospy

topicname2 = '/master/joint_right'
topicname = '/master/joint_left'

input_bag_path = None
velocity_threshold = 0.4
pause_threshold = 200
joint_trajectory = None
joint_velocity = None
joint_time = None

def load_rosbag():
    global input_bag_path, joint_trajectory, joint_velocity, joint_time
    input_bag_path = filedialog.askopenfilename(filetypes=[("ROS Bag Files", "*.bag")])
    if not input_bag_path:
        return
    try:
        messagebox.showinfo("File Loaded", f"Loaded ROS bag file: {input_bag_path}")
        positions = []
        time = []
        velocity = []
        # effort = []
        with rosbag.Bag(input_bag_path, 'r') as bag:
            bag_info = bag.get_type_and_topic_info()
            topics = str(bag_info.topics.keys())

            if topicname not in topics:
                messagebox.showerror("Error", "The selected bag file should contain topic /master/joint_right.")
                return

            for topic, msg, t in bag.read_messages():
                if topic == topicname:
                    # print(f"主题: {topic}")
                    # print(f"时间戳: {t.to_sec()}")
                    # print(f"消息类型: {type(msg).__name__}")
                    # print(f"消息内容: {msg}\n")
                    # print(type(msg))
                    positions.append(msg.position[0:6])
                    time.append(t.to_sec())
                    velocity.append(msg.velocity)
                    # effort.append(msg.effort)


        positions = np.array(positions)
        time = np.array(time)
        velocity = np.array(velocity)
        # effort = np.array(effort)
        t_sum = time[-1] - time[0]
        t_start = time[0]
        time = time - t_start
        joint_trajectory = positions
        joint_velocity = velocity
        joint_time = time
    
    except Exception as e:
        messagebox.showerror("Error", f"Failed to load the rosbag: {e}")

def remove_long_pause():
    global velocity_threshold, pause_threshold, joint_trajectory, joint_velocity, joint_time
    """
    - velocity_threshold: 速度阈值，小于该阈值的点被认为是暂停点
    - pause_threshold: 暂停阈值，连续暂停超过该阈值的点被认为是长时间停滞点
    """

    velocity_norm = np.linalg.norm(joint_velocity, axis=1)
    pause_points = velocity_norm < velocity_threshold
    # print(pause_points)
    keep_indices = np.ones(len(joint_trajectory),dtype = bool)
    # print(len(velocity_norm))
    # print(velocity_norm[0:100])

    # 检查连续停滞的点
    consecutive_pause_count = 0
    pause_start_index = None
    for i in range(len(pause_points)):
        if pause_points[i]:
            if consecutive_pause_count == 0:
                pause_start_index = i
            consecutive_pause_count += 1


            # 如果连续停滞超过阈值，则标记这些点为删除
            if consecutive_pause_count >= pause_threshold:
                keep_indices[pause_start_index:i] = False  # 删除起始到当前的停滞点
                keep_indices[i] = True  # 保留最后一个点
        else:
            # 重置计数器和开始位置
            consecutive_pause_count = 0 
            pause_start_index = None

    # false_positions = np.where(keep_indices == False)[0]
    # print("Positions with False values:", false_positions)
    # print(len(pause_points))


    # 保留有效点
    cleaned_trajectory = joint_trajectory[keep_indices]
    cleaned_time = joint_time[0:len(cleaned_trajectory)]
    cleaned_velocity = joint_velocity[keep_indices]
    # print(len(cleaned_time))
    # print(len(cleaned_trajectory))
    return cleaned_trajectory, cleaned_velocity, cleaned_time

def smooth(positions,velocity,time):
    # 降采样
    interval = 100  # 采样间隔
    time_sample  = time[::interval]
    positions_sample  = positions[::interval]
    if time[-1] not in time_sample:
        time_sample = np.append(time_sample, time[-1])
        positions_sample = np.append(positions_sample, [positions[-1]], axis=0)

    # 平滑插值点也就是原来的时间戳

    # 使用BSpline插值
    spline = make_interp_spline(time_sample, positions_sample, k=3, bc_type='clamped')
    positions_smooth = spline(time)
    velocity_smooth = spline.derivative(1)(time)

    return positions_smooth, velocity_smooth, time

def write_rosbag():
    global input_bag_path
    cleaned_trajectory, cleaned_velocity, cleaned_time = remove_long_pause()
    trajectory, velocity, time_set = smooth(cleaned_trajectory, cleaned_velocity, cleaned_time)

    # 自定义保存文件名
    output_bag_path = filedialog.asksaveasfilename(title="Save File As",
                                        defaultextension=".bag")
                                        
    output_bag = rosbag.Bag(output_bag_path, 'w')
    # rospy.init_node('joint_trajectory_generator', anonymous=True)

    with rosbag.Bag(input_bag_path, 'r') as inbag:
        # start_time = rospy.Time.now()
        # print(start_time)
        # 获取消息格式
        for topic, msg, t in inbag.read_messages():
            if topic == topicname: 
                topic_std = topic
                msg_std = msg
                t_std = t
                break
        if topic_std != topicname:
            raise ValueError("输入的bag文件中不包含topicname主题")


    time_step = 0.005
    for i in range(len(trajectory)):
        msg = msg_std
        position_list = list(msg_std.position)
        velocity_list = list(msg_std.velocity)

        position_list[0:6] = trajectory[i]
        velocity_list[0:6] = velocity[i]
        t = t_std + rospy.Duration.from_sec(i * time_step)   
        msg.position = tuple(position_list)
        msg.velocity = tuple(velocity_list)

        # 将修改后的消息写入新 bag
        output_bag.write(topic, msg, t)

    output_bag.close()
    messagebox.showinfo("Success", f"Trajectory generated and saved to {output_bag_path} successfully.")



# 创建主窗口
root = tk.Tk()
root.title("轨迹优化")
root.geometry("200x200")

# 创建按钮以加载 rosbag
load_button = tk.Button(root, text="Load Rosbag", command=load_rosbag)
load_button.pack(pady=20)

# 创建按钮保存 rosbag
load_button = tk.Button(root, text="Save Rosbag", command=write_rosbag)
load_button.pack(pady=20)

# 启动事件循环
root.mainloop()