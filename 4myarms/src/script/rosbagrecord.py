import subprocess
import os
import psutil
import time

import rosbag

def start_rosbag_recording(topics=None, output_filename='output.bag', recording_path='../bag'):
    """
    开始记录 rosbag。

    :param topics: 要记录的话题列表。
    :param output_filename: 输出文件名，默认为 'output.bag'。
    :param recording_path: 存储 rosbag 文件的目录，默认为 '/tmp'。
    :return: 返回子进程对象，可以通过此对象控制或检查记录过程。
    """
    if not os.path.exists(recording_path):
        os.makedirs(recording_path)

    full_output_path = os.path.join(recording_path, output_filename)
    
    command = ['rosbag', 'record','-a']
    if topics:
        command.extend(topics)
    #else:
    #    command.extend('-a')

    command.extend(['-O', full_output_path])
    
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    print(f"Rosbag {output_filename} recording started. Output will be saved to {full_output_path}")
    
    return process

def stop_rosbag_recording(process):
    """
    结束指定的 rosbag 记录进程。

    :param process: 由 start_rosbag_recording 函数返回的 Popen 对象。
    """
    if process is not None and process.poll() is None:  # 检查进程是否还在运行
        process.terminate()
        process.wait()  # 等待进程结束
        print("Rosbag recording stopped.")
    else:
        print("The provided process is not running or has already been terminated.")


def play_rosbag(bag_file, topics=None, rate=1.0, start_time=None, duration=None,filepath='../bag'):
    """
    播放 rosbag 文件。

    :param bag_file: 要播放的 rosbag 文件路径。
    :param topics: 要播放的话题列表，如果为 None，则播放所有话题。
    :param rate: 播放速率，默认为 1.0（实时）。
    :param start_time: 开始播放的时间偏移（秒），默认从头开始。
    :param duration: 播放持续时间（秒），默认播放到结束。
    :return: 返回子进程对象，可以通过此对象控制或检查播放过程。
    """

    fullbag_file = os.path.join(filepath, bag_file)
    # 构建 rosbag play 命令
    command = ['rosbag', 'play', fullbag_file]
    
    if topics:
        command.append('--topics')
        command.extend(topics)  # 添加话题到命令
    
    if rate != 1.0:
        command.extend(['--rate', str(rate)])
    
    if start_time is not None:
        command.extend(['--start', str(start_time)])
    
    if duration is not None:
        command.extend(['--duration', str(duration)])

    # 使用 gnome-terminal 打开新窗口并执行命令
    #terminal_command = ['gnome-terminal', '--', 'bash', '-c', ' '.join(command) + '; exec bash']
    terminal_command = ['gnome-terminal', '--', 'bash', '-c', ' '.join(command) ]
   
    
    # 启动终端窗口
    process = subprocess.Popen(terminal_command)

    # 启动 rosbag 播放
    #process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    print(f"Rosbag playing started for file: {command}")
    
    return process

def play_bag_and_wait(bag_file, topics=None, rate=1.0, start_time=None, duration=None,filepath='../bag',event=None):

    fullbag_file = os.path.join(filepath, bag_file)

    baglen = get_bag_duration(fullbag_file)
    # 构建 rosbag play 命令
    command = ['rosbag', 'play', fullbag_file]
    
    if topics:
        command.append('--topics')
        command.extend(topics)  # 添加话题到命令
    
    if rate != 1.0:
        command.extend(['--rate', str(rate)])
    
    if start_time is not None:
        command.extend(['--start', str(start_time)])
    
    if duration is not None:
        command.extend(['--duration', str(duration)])

    # 使用 gnome-terminal 打开新窗口并执行命令
    #terminal_command = ['gnome-terminal', '--', 'bash', '-c', ' '.join(command) + '; exec bash']
    terminal_command = ['gnome-terminal', '--', 'bash', '-c', ' '.join(command) ]
   
    baglen = baglen*(1/rate)
    # 启动终端窗口
    process = subprocess.Popen(terminal_command)

    # 启动 rosbag 播放
    #process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    print(f"Rosbag playing started for file: {command} in time {baglen} seconds")
    process.wait()

    while baglen>0:
        print("nxt")
        time.sleep(1)  # 每隔 1 秒检查一次
        baglen=baglen-1

    print(f"Rosbag playing finished for file: {command}")
    # 轮询检查进程状态
    # 设置事件以通知主程序
    if event:
        event.set()



def get_bag_duration(bag_file):
    """
    读取 rosbag 文件并返回其时间长度（以秒为单位）。

    :param bag_file: 要读取的 rosbag 文件路径。
    :return: rosbag 的时间长度（秒）。
    """
    with rosbag.Bag(bag_file, 'r') as bag:
        # 获取第一个消息的时间戳
        start_time = None
        for topic, msg, t in bag.read_messages():
            if start_time is None:
                start_time = t
                break

        # 获取最后一个消息的时间戳
        end_time = None
        for topic, msg, t in bag.read_messages():
            end_time = t

        if start_time is None or end_time is None:
            raise ValueError("Bag file is empty or does not contain any messages.")

        # 计算时间差
        duration = (end_time - start_time).to_sec()
        return duration


# 使用示例
if __name__ == '__main__':
    # 启动 rosbag 记录
    record_process = start_rosbag_recording(topics=['/camera/image_raw', '/imu/data'], output_filename='my_recording.bag')

    # 在这里可以执行其他操作，例如等待用户输入或在特定条件满足时停止记录
    input("Press Enter to stop the recording...")

    # 停止 rosbag 记录
    stop_rosbag_recording(record_process)