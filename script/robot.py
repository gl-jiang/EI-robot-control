import subprocess
import time
from claw import *


process_robotstart = None
process_robotarmrun = None




def open_can():
    print("Opening CAN")
    """运行给定路径的shell脚本"""
    script_path = "../remote_control-x86-can-v2/tools/can.sh"
     # 对于GNOME Terminal，你可以使用如下命令
    gnome_command = ["gnome-terminal", "--", "bash", "-c", f"{script_path}; exec bash"]
    subprocess.Popen(gnome_command)
    print("CAN is openned")

def start_robot():
    global process_robotstart
    print("Starting robot")
    script_path = "../remote_control-x86-can-v2/tools/jgl_2follower.sh"
     # 对于GNOME Terminal，你可以使用如下命令
    gnome_command =  ["gnome-terminal", "--", "bash", "-c", f"{script_path}; exec bash"]
    #process_robotstart =  subprocess.Popen(gnome_command)
    process_robotstart= subprocess.run(["bash", script_path])
    time.sleep(2)
    if process_robotstart is not None and process_robotstart.poll() is None: 
        print("robot is started")
    


def execute_path_1():#griper1 
    print("gripper switch1")
    data = ">02G9158"
    #datainitopen = b'\x02\x06\x01\x00\x00\x01\x49\xf6'  # 示例字节序列
    # 调用函数发送数据
    rotswithch(data)
   

def execute_path_2():#griper2
    print("gripper switch2")
    data = ">02D000001100bad"
    #datainitopen = b'\x02\x06\x01\x00\x00\x01\x49\xf6'  # 示例字节序列
    # 调用函数发送数据
    rotswithch(data)

def execute_path_3():#griper3
    print("gripper switch3")
    data =  ">02h000005461388B61E"
    #data = ">02h0000044C2710AEC5"#">02D000005005aed">02h0000044C2710AEC5
    #datainitopen = b'\x02\x06\x01\x00\x00\x01\x49\xf6'  # 示例字节序列
    # 调用函数发送数据
    rotswithch(data)



def open_gripper():
    print("Opening gripper")
    initdata = b'\x01\x06\x01\x00\x00\x01\x49\xF6'
    shifang = b'0x\01\x06\x03\x04\x00\x01\x09\x8F'
    shineng = b'0x\01\x06\x03\x04\x00\x02\x09\x8F'
    # 准备要发送的数据
    #data = b'\x01\x06\x01\x05\x00\x32\x19\xe2'  # 示例字节序列
    datainitopen = b'\x01\x06\x01\x00\x00\x01\x49\xf6'  # 示例字节序列
    # 调用函数发送数据
    clawop(datainitopen)


def close_gripper():
    print("closing gripper")
    # 准备要发送的数据
    data = b'\x01\x06\x01\x05\x00\x64\x99\xdc'  # 示例字节序列 100%
    clawop(data)


def start_robot_train():
    global process_robotstart
    print("Starting robot training")
    script_path = "../remote_control-x86-can-v2/tools/remote.sh"
     # 对于GNOME Terminal，你可以使用如下命令
    gnome_command = ["gnome-terminal", "--", "bash", "-c", f"{script_path}; exec bash"]
    #process_robotstart =  subprocess.Popen(gnome_command)
    process_robotstart= subprocess.run(["bash", script_path])
    time.sleep(2)
    if process_robotstart is not None and process_robotstart.poll() is None: 
        print("robot is started")

def run_script(script_path):
    """
    启动另一个 Python 脚本并等待其完成。

    :param script_path: 要启动的 Python 脚本路径。
    :param args: 传递给脚本的参数。
    :return: 脚本的返回码。
    """
    # 构建命令
    command = ['python3', script_path] 
    
    # 启动子进程
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
       
    return process


def launch_ros_file(launch_file):
    """
    启动一个 roslaunch 文件并等待其完成。

    :param launch_file: 要启动的 roslaunch 文件路径。
    :param args: 传递给 roslaunch 的参数。
    :return: roslaunch 进程的返回码。
    """
    # 构建 roslaunch 命令
    command = ['roslaunch', 'robot_r' ,launch_file] 
    
    # 启动子进程
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    # 打印启动信息
    print(f"Launching {launch_file} ")
     
    return process

def start_Claw_lsn():
    run_script('rightclawlsn.py')
    
def start_dipan():
    launch_ros_file('00base.launch')


def execute_path_4():
    print("getting bot")
    rosbag_file = "../traj/getbot.bag"
    play_rosbag(rosbag_file)


def execute_path_5():
    print("cooling bot")
    rosbag_file = "../traj/cool.bag"
    play_rosbag(rosbag_file)

def execute_path_6():
    print("return robot")
    rosbag_file = "../traj/return.bag"
    play_rosbag(rosbag_file)

def stop_robot():
    print("Stopping robot")
    global process_robotarmrun
    if process_robotarmrun is not None and process_robotstart.poll() is None:  # 检查进程是否还在运行
        process_robotarmrun.terminate()  # 发送SIGTERM信号尝试优雅地终止进程
        process_robotarmrun.wait()       # 等待进程真正结束，避免僵尸进程
        process_robotarmrun = None       # 清理进程引用

def initialize_robot():
    print("Initializing robot")

def play_rosbag(rosbag_file):
    # 这里是你的rosbag文件路径
    global process_robotarmrun
    #rosbag_file = "/path/to/your/rosbag_file.bag"
    try:
        # 使用subprocess.Popen运行命令
        # 注意这里使用的是Popen，因为它可以启动一个持续运行的进程
        process_robotarmrun = subprocess.Popen(["rosbag", "play", "-r", "0.5", rosbag_file])
    except Exception as e:
        print(f"Error executing rosbag play: {e}")


