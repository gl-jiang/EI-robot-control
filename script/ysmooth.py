import rosbag
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import make_interp_spline

def smooth_and_save_rosbag(input_bag_path, output_bag_path):
    """
    从输入的rosbag文件中读取关节位置和速度数据，进行平滑处理，并将结果保存到新的rosbag文件中。

    :param input_bag_path: 输入的rosbag文件路径
    :param output_bag_path: 输出的rosbag文件路径
    """
    # 初始化数据列表
    positions = []
    time = []
    velocity = []

    # 读取数据
    with rosbag.Bag(input_bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            positions.append(msg.position[0:6])
            time.append(t.to_sec())
            velocity.append(msg.velocity)

    # 转换为numpy数组
    positions = np.array(positions)
    time = np.array(time)
    velocity = np.array(velocity)

    # 时间归一化
    t_sum = time[-1] - time[0]
    t_start = time[0]
    time = time - t_start

    # 降采样
    interval = 100  # 采样间隔
    time_sample = time[::interval]
    positions_sample = positions[::interval]
    if time[-1] not in time_sample:
        time_sample = np.append(time_sample, time[-1])
        positions_sample = np.append(positions_sample, [positions[-1]], axis=0)

    # 使用BSpline插值
    spline = make_interp_spline(time_sample, positions_sample, k=3, bc_type='clamped')
    positions_smooth = spline(time)
    velocity_smooth = spline.derivative(1)(time)

    # 绘制平滑前后的关节角度和速度
    plt.figure(1)
    for joint_index in range(6):
        plt.plot(time, positions[:, joint_index], label=f'Joint {joint_index}')
        plt.plot(time, positions_smooth[:, joint_index], label=f'Joint {joint_index} smooth')
    plt.title('Joint Angles Over Time')
    plt.xlabel('Time Points')
    plt.ylabel('Joint Angles')
    plt.legend(loc='upper right')
    plt.grid(True)
    plt.show()

    plt.figure(2)
    for joint_index in range(6):
        plt.plot(time, velocity[:, joint_index], label=f'Joint {joint_index}')
        plt.plot(time, velocity_smooth[:, joint_index], label=f'Joint {joint_index} smooth')
    plt.title('Joint Velocities Over Time')
    plt.xlabel('Time Points')
    plt.ylabel('Joint Velocities')
    plt.legend(loc='upper right')
    plt.grid(True)
    plt.show()

    # 写入新的rosbag文件
    output_bag = rosbag.Bag(output_bag_path, 'w')
    i = 0
    with rosbag.Bag(input_bag_path, 'r') as inbag:
        for topic, msg, t in inbag.read_messages():
            #if topic == '/master/joint_right':
            position_list = list(msg.position)
            velocity_list = list(msg.velocity)
            position_list[0:6] = positions_smooth[i]
            velocity_list[0:6] = velocity_smooth[i]
            msg.position = tuple(position_list)
            msg.velocity = tuple(velocity_list)
            i += 1
            # 保持话题名称一致
            output_bag.write(topic, msg, t)
    output_bag.close()

# 调用函数
input_bag_path = '../bag/gunarm4__master_joint_left.bag'
output_bag_path = '../bag/smooth_gunarm4__master_joint_left.bag'
smooth_and_save_rosbag(input_bag_path, output_bag_path)