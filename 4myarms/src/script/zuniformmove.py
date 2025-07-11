import rospy
import rosbag
import numpy as np
import tkinter as tk
from tkinter import messagebox
from matplotlib import pyplot as plt

def generate_joint_trajectory(start_angles, end_angles, max_speed, time_step=0.005):
    """
    参数：
    - start_angles: 起点的 6 个关节角（列表或数组），单位为弧度。
    - end_angles: 终点的 6 个关节角（列表或数组），单位为弧度。
    - max_speed: 各个关节的最大速度（可以是一个数值或包含 6 个关节速度的列表），单位为弧度/秒。
    - time_step: 时间步长，单位为秒。
    
    返回：
    - trajectory: 包含每个时间步各个关节角的轨迹
    - time_set: 时间序列
    """
    
    start_angles = np.array(start_angles)
    end_angles = np.array(end_angles)
    max_speed = np.array(max_speed) if hasattr(max_speed, "__len__") else np.full(6, max_speed)
    angle_diffs = end_angles - start_angles
    
    # 计算每个关节的总移动时间，确保不超过最大速度
    time_to_reach = np.abs(angle_diffs) / max_speed  # 每个关节到达终点所需的时间
    total_time = np.max(time_to_reach)  # 总时间取决于最慢的关节
    velocity = angle_diffs / total_time  # 每个关节的速度
    
    # 计算所需的总步数
    num_steps = int(np.ceil(total_time / time_step))
    
    # 为每个时间步生成轨迹
    trajectory = []
    time_set = np.arange(0, total_time + time_step, time_step)
    for step in range(num_steps + 1):
        current_time = step * time_step
        current_angles = start_angles + np.clip(current_time / total_time, 0, 1) * angle_diffs
        trajectory.append(current_angles)
    
    return np.array(trajectory), np.array(time_set),np.array(velocity)

def plot_trajectory():
    try:
        # 获取用户输入的值
        start_angles = list(map(float, start_angles_entry.get().split(',')))
        end_angles = list(map(float, end_angles_entry.get().split(',')))
        max_speed = float(max_speed_entry.get())
        time_step = float(time_step_entry.get())

        # 检查输入
        if len(start_angles) != 6 or len(end_angles) != 6:
            messagebox.showerror("输入错误", "起点和终点的关节角度需要包含 6 个值")
            return

        # 生成轨迹
        trajectory, time_set, _ = generate_joint_trajectory(start_angles, end_angles, max_speed, time_step)

        # 绘制轨迹
        plt.figure()
        for joint_index in range(6):
            plt.plot(time_set, trajectory[:, joint_index], label=f'Joint {joint_index}')
        plt.title('Joint Angles Over Time')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Joint Angles (radians)')
        plt.legend(loc='upper right')
        plt.grid(True)
        plt.show()

    except ValueError:
        messagebox.showerror("输入错误", "请输入有效的数字")

def generate_rosbag():
    try:
        # 获取用户输入的值
        start_angles = list(map(float, start_angles_entry.get().split(',')))
        end_angles = list(map(float, end_angles_entry.get().split(',')))
        max_speed = float(max_speed_entry.get())
        time_step = float(time_step_entry.get())
        
        # 检查输入
        if len(start_angles) != 6 or len(end_angles) != 6:
            messagebox.showerror("输入错误", "起点和终点的关节角度需要包含 6 个值")
            return

        # 生成轨迹
        trajectory, time_set, velocity = generate_joint_trajectory(start_angles, end_angles, max_speed, time_step)
        
        # .bag文件路径(自行修改) input_bag_path需要设置正确，因为需要使用input_bag的消息格式
        input_bag_path = 'clawArm1__master_joint_right.bag'
        output_bag_path = 'generate_joint_trajectory.bag'
        output_bag = rosbag.Bag(output_bag_path, 'w')
        # rospy.init_node('joint_trajectory_generator', anonymous=True)

        with rosbag.Bag(input_bag_path, 'r') as inbag:
            # start_time = rospy.Time.now()
            # print(start_time)
            # 获取消息格式
            for topic, msg, t in inbag.read_messages():
                if topic == '/master/joint_right': 
                    topic_std = topic
                    msg_std = msg
                    t_std = t
                    break
            if topic_std != '/master/joint_right':
                raise ValueError("输入的bag文件中不包含'/master/joint_right'主题")



        for i in range(len(trajectory)):
            msg = msg_std
            position_list = list(msg_std.position)
            velocity_list = list(msg_std.velocity)

            position_list[0:6] = trajectory[i]
            velocity_list[0:6] = velocity
            t = t_std + rospy.Duration.from_sec(i * time_step)   
            msg.position = tuple(position_list)
            msg.velocity = tuple(velocity_list)

            # 将修改后的消息写入新 bag
            output_bag.write(topic, msg, t)

        output_bag.close()
        messagebox.showinfo("Success", f"Trajectory generated and saved to {output_bag_path} successfully.")
    except Exception as e:
        messagebox.showerror("Error", f"Failed to write the rosbag: {e}")


# 创建主窗口
root = tk.Tk()
root.title("关节轨迹生成器")

# 创建标签和输入字段
tk.Label(root, text="起点角度（弧度，逗号分隔 6 个值）").grid(row=0, column=0, padx=10, pady=5)
start_angles_entry = tk.Entry(root, width=50)
start_angles_entry.grid(row=0, column=1, padx=10, pady=5)
start_angles_entry.insert(0, "0.0284195,0.0036239,0.0932703,-0.4667348,-0.0001907,-0.0764856")

tk.Label(root, text="终点角度（弧度，逗号分隔 6 个值）").grid(row=1, column=0, padx=10, pady=5)
end_angles_entry = tk.Entry(root, width=50)
end_angles_entry.grid(row=1, column=1, padx=10, pady=5)
end_angles_entry.insert(0, "0.83867359,1.82440662,1.34832573,0.8871212,0.11272621,-0.02651215")

tk.Label(root, text="最大速度（弧度/秒）").grid(row=2, column=0, padx=10, pady=5)
max_speed_entry = tk.Entry(root, width=50)
max_speed_entry.grid(row=2, column=1, padx=10, pady=5)
max_speed_entry.insert(0, "0.1")

tk.Label(root, text="时间步长（秒）").grid(row=3, column=0, padx=10, pady=5)
time_step_entry = tk.Entry(root, width=50)
time_step_entry.grid(row=3, column=1, padx=10, pady=5)
time_step_entry.insert(0, "0.005")



# 创建生成按钮
generate_button1 = tk.Button(root, text="生成轨迹并绘制", command=plot_trajectory)
generate_button2 = tk.Button(root, text="保存轨迹bag", command=generate_rosbag)
generate_button1.grid(row=4, column=0, columnspan=2, pady=10)
generate_button2.grid(row=5, column=0, columnspan=2, pady=10)


# 运行主循环
root.mainloop()

