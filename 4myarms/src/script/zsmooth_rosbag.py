import rosbag
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import make_interp_spline

#平滑轨迹并保存到新的bag文件中

# .bag文件路径
input_bag_path = '../bag/gunarm1__master_joint_left.bag'
output_bag_path = '../bag/smooth_gunarm1__master_joint_left.bag'

# 打开bag文件并打印信息

positions = []
time = []
velocity = []
# effort = []

# 频率约为200Hz
with rosbag.Bag(input_bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages():
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

# print(positions_smooth-positions)

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

output_bag = rosbag.Bag(output_bag_path, 'w')

i = 0
with rosbag.Bag(input_bag_path, 'r') as inbag:
    for topic, msg, t in inbag.read_messages():
        if topic == '/master/joint_right': 
            position_list = list(msg.position)
            velocity_list = list(msg.velocity)
            position_list[0:6] = positions_smooth[i]
            velocity_list[0:6] = velocity_smooth[i]
            
            msg.position = tuple(position_list)
            msg.velocity = tuple(velocity_list)

            i += 1
        # 将修改后的消息写入新 bag
        output_bag.write(topic, msg, t)

output_bag.close()
