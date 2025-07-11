import tkinter as tk
import queue
import threading
import subprocess
import rospy
from actionlib_msgs.msg import GoalStatusArray
from gun import SerialController  # 假设 gun 模块中定义了 SerialController 类

import threading
import time

import rospy

#import intera_interface


# 定义一个全局队列，用于线程间的通信
#result_queue = queue.Queue()

runStatus = 0
lock = threading.Lock()  # 创建一个线程锁

filepub = None

def status_callback(data):
    global  runStatus , lock
    for status in data.status_list:
        with lock:
            runStatus = status.status


class RobotArmSimulator:
    def __init__(self, root):
        self.root = root
        self.root.title("机械臂控制")
        self.root.geometry("800x600")


        # 轨迹存储
        self.trajectories = {f"位置{i+1}": [] for i in range(4)}
        self.gun = SerialController()  # 假设已经定义了SerialController类
        
        # 左列 - 功能按钮
        self.function_frame = tk.Frame(root)
        self.function_frame.pack(side=tk.LEFT, padx=10, pady=10)

        self.position_buttons = {
            "位置1": (100, 100),
            "位置2": (200, 100),
            "位置3": (300, 100),
            "位置4": (400, 100)
        }

        #self.command1 = ["rosrun", "intera_examples", "joint_trajectory_file_playback.py", "-f", "a1"]
        #self.command2 = ["rosrun", "intera_examples", "joint_trajectory_file_playback.py", "-f", "a2"]
        #self.command3 = ["rosrun", "intera_examples", "joint_trajectory_file_playback.py", "-f", "a3"]
        #self.command4 = ["rosrun", "intera_examples", "joint_trajectory_file_playback.py", "-f", "a4"]

        rosbagfile1 = "a1"
        self.command1 = ["rosbag", "play", "-r", "0.5", rosbagfile1]
        rosbagfile2 = "a2"
        self.command2 = ["rosbag", "play", "-r", "0.5", rosbagfile2]
        rosbagfile3 = "a3"
        self.command3 = ["rosbag", "play", "-r", "0.5", rosbagfile3]
        rosbagfile4 = "a4"
        self.command4 = ["rosbag", "play", "-r", "0.5", rosbagfile4]


        #self.commandstart = ["rosrun", "intera_interface", "joint_trajectory_action_server.py", "-m", "position"]



        # 创建标签用于显示状态
        self.label = tk.Label(root, text="Waiting for status updates...")
        self.label.pack(pady=20)

        #btnstartserver = tk.Button(root, text="start robot", command= self.startrobot() )
        #btnstartserver.pack(pady=20)

        button0 = tk.Button(root, text="gun_init", command=self.gun.send_command_one)
        button0.pack(pady=10)

        button1 = tk.Button(root, text="gun_xiye1", command=self.gun.send_command_two)
        button1.pack(pady=10)

        # 创建另一个按钮并绑定到another_action函数
        button2 = tk.Button(root, text="gun_xiye2", command=self.gun.send_command_three)
        button2.pack(pady=10)

             # 创建另一个按钮并绑定到another_action函数
        button2 = tk.Button(root, text="gun_tuye1", command=self.gun.send_command_four)
        button2.pack(pady=10)



        # 创建一个按钮并绑定到启动流程的函数
        start_button = tk.Button(root, text="Start Sequence", command=self.start_sequence)
        start_button.pack(pady=10)

                # 创建一个标签用于显示当前的状态
        self.status_label = tk.Label(root, text="Ready")
        self.status_label.pack(pady=10)

        # 创建一个Event对象用于同步
        self.next_step_event = threading.Event()

 
        for position, coord in self.position_buttons.items():
            btn = tk.Button(self.function_frame, text=position, command=lambda name=position: self.move_robot_arm(name))
            btn.pack(pady=5)

        # 右列 - 手动拖动按钮
        self.draggable_frame = tk.Frame(root)
        self.draggable_frame.pack(side=tk.RIGHT, padx=10, pady=10)

        self.dragging = False
        self.current_position = None

        for position in self.position_buttons.keys():
            btn = tk.Button(self.draggable_frame, text=f"记录{position}轨迹", command=lambda pos=position: self.start_dragging(pos))
            btn.pack(pady=5)

        self.check_status_id = None

    def function_a(self):
        """第一个函数"""
        self.update_status("Executing Function A...")
        self.run_ros_script(self.command1)
        time.sleep(3)  # 模拟耗时操作


    def function_b(self):
        """第二个函数"""
        self.update_status("Executing Function B...")
        self.gun.send_command_one()
        time.sleep(3)  # 模拟耗时操作
        self.gun.send_command_two()
        time.sleep(3)  # 模拟耗时操作
        self.update_status("Function B completed.")


    def function_c(self):
        """第三个函数"""
        self.update_status("Executing Function C...")
        self.run_ros_script(self.command2)
        time.sleep(3)  # 模拟耗时操作

    def function_d(self):
        """第二个函数"""
        self.update_status("Executing Function d...")
        #self.run_ros_script(self.command2)
        self.gun.send_command_three()
        time.sleep(5)  # 模拟耗时操作
        #self.update_status("Function B completed.")

    def function_e(self):
        self.update_status("Executing Function e...")
        self.run_ros_script(self.command3)
        time.sleep(3)

    def function_f(self):
        self.update_status("Executing Function f...")
        #self.run_ros_script(self.command2)
        self.gun.send_command_four()
        time.sleep(5)  # 模拟耗时操作
        #self.update_status("Function B completed.")
   
    global filepub
    def function_run2(self):
        print("send and pub cmd run 2")
        filepub.publish("a2")

    def function_run3(self):
        filepub.publish("a3")
    
    def function_run4(self):
        filepub.publish("a4")

    def start_sequence(self):
        """启动按顺序执行函数的流程"""
        self.update_status("Starting sequence...")

        def execute_functions():
                # 清除Event标志
            try:
                # 清除Event标志
                self.next_step_event.clear()
                print("Event cleared before executing function_a")

                # 执行第一个函数
                #self.function_a()
                #self.next_step_event.clear()  # 清除Event标志

                #self.next_step_event.wait()
                # 执行第二个函数
                self.function_b()
                self.next_step_event.clear()  # 清除Event标志

                # 执行第一个函数
                self.function_c()
                self.next_step_event.clear()  # 清除Event标志

                self.next_step_event.wait()
                self.function_d()
                self.next_step_event.clear()  # 清除Event标志

                self.function_e()
                self.next_step_event.clear()  # 清除Event标志

                self.next_step_event.wait()
                self.function_f()
                self.next_step_event.clear()  # 清除Event标志
                print("Event cleared after function_的")

                              # 执行第一个函数
                self.function_a()
                self.next_step_event.clear()  # 清除Event标志

                self.next_step_event.wait()


            except Exception as e:
                self.update_status(f"An error occurred: {e}")


        # 在新线程中启动流程
        thread = threading.Thread(target=execute_functions)
        thread.start()


    def update_status(self, status):
        """更新状态标签"""
        self.status_label.config(text=status)



    def startrobot(self):
        self.run_ros_script(self.commandstart)
       
    def update_gui(self):
        global runStatus
        try:
            #status = queue.get_nowait()
            self.label.config(text=f"Current Status: {runStatus}")
        #except queue.Empty:
            # 如果队列为空，则不更新
        #    pass
        finally:
            # 每隔一定时间更新一次GUI
            self.root.after(100, lambda: self.update_gui())

    def move_robot_arm(self, name):
        # 这里添加机械臂移动到指定位置的代码
        print(name)
        if name == "位置1":
            self.run_ros_script(self.command1)
            #self.gun.send_command_one()
        elif name == "位置2":
            self.run_ros_script(self.command2)
            #self.gun.send_command_two()
        elif name == "位置3":
            self.run_ros_script(self.command3)
            #self.gun.send_command_three()
        elif name == "位置4":
            self.run_ros_script(self.command4)
            #self.gun.send_command_four()
        print(f"移动机械臂到 {name}")

    def start_dragging(self, position):
        #self.current_position = position
        self.dragging = True
        self.function_run2()
        #self.trajectories[position] = []  # 重置当前轨迹
        #subprocess.Popen(['python3', 'joint_recorder.py', ' -f ', position])


    def run_ros_script(self, command):
        global lock, runStatus
        with lock:  # 获取锁
            if runStatus in [1, 2]:
                print("robot not ready or running!")
                return 0
            commandexe = command
            def execute_ros_command(commandexe):
                try:
                    # 使用Popen执行ROS命令
                    subprocess.Popen(commandexe)
                except Exception as e:
                    print(f"An error occurred: {e}")
                # Wait until start
                self.realstatus = 0
                counter = 0
                # 开始检查状态
                self.check_status(commandexe, counter)

            # 在新线程中运行ROS命令
            thread = threading.Thread(target=execute_ros_command, args=(commandexe,))
            thread.start()

    def check_status(self, commandexe, counter):
        """定期检查状态并更新"""
        global lock, runStatus
        with lock:
            if self.check_status_id is not None:
                self.root.after_cancel(self.check_status_id)  # 取消之前的任务
            self.check_status_id = self.root.after(500, lambda: self.check_status(commandexe, counter + 1))
            #print(f"counter={counter}, runStatus={runStatus}, realstatus={self.realstatus}")
            if self.realstatus == 0 and runStatus == 1:
                self.realstatus = 1
                print(f"counter={counter}, runStatus={runStatus}, realstatus={self.realstatus}")
            if self.realstatus == 1 and runStatus == 3:
                self.realstatus = 3
                time.sleep(2)          
                self.check_status_id = None  # 任务完成，取消任务ID
                print(f"counter={counter}, runStatus={runStatus}, realstatus={self.realstatus}")
                print("ROS script execution completed.")
                self.next_step_event.set()
                self.stop_check_status()   
                return
            if counter > 500:
                print("robot run time out")
                self.stop_check_status()
                self.check_status_id = None  # 任务超时，取消任务ID
                return

    def stop_check_status(self):
        """停止检查状态"""
        if self.check_status_id is not None:
            self.root.after_cancel(self.check_status_id)
            self.check_status_id = None

from std_msgs.msg import String

if __name__ == "__main__":

    root = tk.Tk()
    app = RobotArmSimulator(root)

    # 在主线程中初始化ROS节点和启动订阅者
    rospy.init_node('goal_status_listener', anonymous=True)
    rospy.Subscriber('/robot/limb/right/follow_joint_trajectory/status', GoalStatusArray, status_callback)

    # 启动ROS订阅者线程
    ros_listener_thread = threading.Thread(target=rospy.spin, daemon=True)
    ros_listener_thread.start()

      # 创建一个发布者，发布到/file_name_topic话题
    filepub = rospy.Publisher('/trajfile_topic', String, queue_size=2)



    # 初始更新GUI
    app.update_gui()

    root.mainloop()

