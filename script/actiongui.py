from gui1 import CANStatusIndicator
import os
import tkinter as tk
from tkinter import Canvas
from robot import *

# import seria

from tkinter import filedialog
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from gun import SerialController  # 假设 gun 模块中定义了 SerialController 类

from rosbagrecord import *
import threading
from tkinter import StringVar

from BLScontroller import Track_control_left, Track_control_right

class newgui(CANStatusIndicator):
    def __init__(self, root):
        super().__init__(root)
        self.root = root

        self.ismearingTemp = False
        self.lock = threading.Lock()  # 添加锁对象

    
                # 创建一个绿色背景的帧来容纳第二组按钮
        ctl_frame = tk.Frame(root, bg="#e0e7ff", width=300, height=350)
        ctl_frame.place(x=1500,y=150)
        ctl_frame.pack_propagate(False)  # 禁止frame自动调整大小
        ctl_frame.grid_propagate(False)

        # 配置网格行列权重，使按钮均匀分布
        for i in range(6):
            ctl_frame.grid_rowconfigure(i, weight=1)
        ctl_frame.grid_columnconfigure(0, weight=1)

        self.Rcstop = tk.Button(ctl_frame, text="RunAll", font=self.font, command=self.startRunAllThread)
        self.Rcstop.grid(row=0,column=0, padx=10, pady=5, sticky="ew")

        self.Rcstop_gun = tk.Button(ctl_frame, text="RunALLGunOnly", font=self.font, command=self.startRunAllThread_onlyGun)
        self.Rcstop_gun.grid(row=2,column=0, padx=10, pady=5, sticky="ew")

        self.Rcstop_gunAdj = tk.Button(ctl_frame, text="RunALLGunAdj", font=self.font, command=self.startRunAllThread_gunAdj)
        self.Rcstop_gunAdj.grid(row=1,column=0, padx=10, pady=5, sticky="ew")
         # 创建一个输入框
        #self.entry2 = tk.Entry(ctl_frame)
        self.entry2 = tk.Entry(ctl_frame, width=5, font=self.font)  # 设置宽度为5个字符
        self.entry2.insert(0, '1')  # 设置初始值
        self.entry2.grid(row=3,column=0, padx=10, pady=5, sticky="ew")

        self.spdops = [24000,1000,500]
        self.selectedops = StringVar(ctl_frame)
        self.selectedops.set(str(self.spdops[0]))
        self.opmenu = tk.OptionMenu(ctl_frame,self.selectedops,*[str(opt) for opt in self.spdops])
        self.opmenu.config(font=self.font)
        self.opmenu.grid(row=4,column=0, padx=10, pady=5, sticky="ew")
        self.trajspdops = [1,0.5,0.2,0.1]
        self.selectedops_trj = StringVar(ctl_frame)
        self.selectedops_trj.set(str(self.trajspdops[0]))
        self.opmenu_trj = tk.OptionMenu(ctl_frame,self.selectedops_trj,*[str(opt) for opt in self.trajspdops])
        self.opmenu_trj.config(font=self.font)
        self.opmenu_trj.grid(row=5,column=0, padx=10, pady=5, sticky="ew")



        self.setgunspdBtn = tk.Button( self.gun_frame, text="setgunspd", font=self.font, command=self.setgunspd)
        self.setgunspdBtn.grid(row=4, column=0, padx=20, pady=20, sticky="ew")

        self.ActionRuning = False
        self.UseGunPoseAdj = False

        pnt_frame = tk.Frame(root, bg="#b2d9e3", width=300, height=350)
        pnt_frame.place(x=1200,y=150)
        pnt_frame.pack_propagate(False)  # 禁止frame自动调整大小
        pnt_frame.grid_propagate(False)
        # 配置网格行列权重，使按钮均匀分布
        for i in range(6):
            pnt_frame.grid_rowconfigure(i, weight=1)
        pnt_frame.grid_columnconfigure(0, weight=1)

        self.RcG1 = tk.Button(pnt_frame, text="recordGunpos1", font=self.font, command=self.recordPnt1)
        self.RcG1.grid(row=0,column=0, padx=10, pady=5, sticky="ew")
        self.RcG2 = tk.Button(pnt_frame, text="recordGunpos2", font=self.font, command=self.recordPnt2)
        self.RcG2.grid(row=1,column=0, padx=10, pady=5, sticky="ew")
        self.RcG3 = tk.Button(pnt_frame, text="recordGunpos3", font=self.font, command=self.recordPnt3)
        self.RcG3.grid(row=2,column=0, padx=10, pady=5, sticky="ew")
        self.RcG4 = tk.Button(pnt_frame, text="recordGunpos4", font=self.font, command=self.recordPnt4)
        self.RcG4.grid(row=3,column=0, padx=10, pady=5, sticky="ew")
        self.RcG5 = tk.Button(pnt_frame, text="recordGunpos5", font=self.font, command=self.recordPnt5)
        self.RcG5.grid(row=4,column=0, padx=10, pady=5, sticky="ew")
        self.RcG6 = tk.Button(pnt_frame, text="recordGunposZero", font=self.font, command=self.recordPnt0)
        self.RcG6.grid(row=5,column=0, padx=10, pady=5, sticky="ew")

    def get_rc1_json_string(self):
        # 检查 self.entry2 是否为整数并且在 1 到 6 之间
        val = int(self.entry2.get())
        print(val)
        if isinstance(val, int) and 1 <= val <= 5:
            return f"rc{val}.json"
        else:
            return ""

    def runAll_test(self):
        cyc = 10
        while cyc>0:
            cyc = cyc-1
            time.sleep(10)
            self.RunAll_1217()

    def setgunspd(self):
        # value = int(self.selectedops.get())
        # print(f"gun speed = {value}")
        # self.gun.send_command_setSpeed(value)
        pass

    def RunOnlyGun(self):
        self.ActionRuning = True
        execute_path_1()  # jiance wendu
        open_gripper()
         # 发送第一个命令
        #self.gun.send_command_one()
        self.setgunspd()
        time.sleep(0.5)
        self.gun.send_command_four()
        time.sleep(0.5)
        # 发送第二个命令
        self.gun.send_command_two()
        time.sleep(1)

        self.next_step_event.clear()
        play_bag_and_wait("smooth_gunarm1__master_joint_left.bag", event=self.next_step_event)      
        self.next_step_event.wait()
        time.sleep(1)
         # 重置事件
      #  self.next_step_event.clear()
      #  time.sleep(1)
      #  play_rosbag('smooth_NclawArm1__master_joint_right.bag')
      #  time.sleep(1)

        #add water
        self.next_step_event.clear()
        play_bag_and_wait("smooth_gunarm2__master_joint_left.bag", event=self.next_step_event)
        self.next_step_event.wait()
        time.sleep(1)
        self.gun.send_command_three_08()
        time.sleep(6)
        self.next_step_event.clear()
        # 播放第二个包
        play_bag_and_wait("smooth_gunarm3__master_joint_left.bag", event=self.next_step_event)
        # 等待事件被设置
        self.next_step_event.wait()

        if self.UseGunPoseAdj:
            self.next_step_event.clear()
            ret = self.get_rc1_json_string()
            if ret=="":
                print("error load value, no gun adjust skip!!!!")
            else:
                print("adjusting..................")
                self.start_gun_adj(ret,done_event=self.next_step_event)
                self.next_step_event.wait()


        time.sleep(1)
        self.gun.send_command_four()
        time.sleep(2)
        if self.UseGunPoseAdj:
            self.next_step_event.clear()
            ret = "rc0.json" #self.get_rc1_json_string()
            if ret=="":
                print("error load value, no gun adjust skip!!!!")
            else:
                print("adjusting..................")
                self.start_gun_adj(ret,done_event=self.next_step_event)
                self.next_step_event.wait()
  
        play_rosbag('smooth_gunarm4__master_joint_left.bag')
        time.sleep(3)
        
        #get bot
     #   self.next_step_event_clawarm.clear()
      #  play_bag_and_wait("smooth_NclawArm2__master_joint_right.bag",rate=1, event=self.next_step_event_clawarm)        #readypose-->getpose --->closeclaw  
      #  self.next_step_event_clawarm.wait() 

      #  execute_path_3()  # jiance wendu
      #  time.sleep(3)
      #  self.startTempThread()

       # time.sleep(1)

        #goto cool
      #  self.next_step_event_clawarm.clear()
      #  play_bag_and_wait("smooth_NclawArm3__master_joint_right.bag", event=self.next_step_event_clawarm)       #getpose-->coolpose1   
      #  self.next_step_event_clawarm.wait()

        #cooling
      #  while True:
      #      with self.lock:
      #          print(f"current temp {self.ismearingTemp} __ {self.currentTemp}")
      #          if not self.ismearingTemp:
      #              break
      #      time.sleep(0.05)
      #  print('recording finished')

     #   execute_path_1()

      #  self.next_step_event_clawarm.clear()
      #  play_bag_and_wait("smooth_NclawArm4__master_joint_right.bag", event=self.next_step_event_clawarm)          #coolpose1-->coolpose2   
      #  self.next_step_event_clawarm.wait() 

      #  self.next_step_event_clawarm.clear()
      #  play_bag_and_wait("smooth_NclawArm5__master_joint_right.bag", event=self.next_step_event_clawarm)          #coolpose1-->coolpose2   
      #  self.next_step_event_clawarm.wait() 
        self.ActionRuning = False

    def RunAll_1217(self):
        self.Track_control_right = Track_control_right()
        self.Track_control_left = Track_control_left()
        #if(self.ActionRuning == True):
        #    return
        self.ActionRuning = True
        execute_path_1()  # jiance wendu
        open_gripper()
         # 发送第一个命令
        #self.gun.send_command_one()
        self.setgunspd()
        time.sleep(0.5)
        self.gun.send_command_four()
        time.sleep(0.5)
        # 发送第二个命令
        self.gun.send_command_two()
        time.sleep(1)

        self.Track_control_left.track_control("smooth_gunarm1__master_joint_left.bag")
        time.sleep(1)
         # 重置事件
        self.next_step_event.clear()
        time.sleep(1)
        self.Track_control_right.track_control('smooth_clawArm1__master_joint_right.bag')
        time.sleep(1)

        #add water

        self.Track_control_left.track_control("smooth_gunarm2__master_joint_left.bag", event=self.next_step_event)

        time.sleep(1)
        self.gun.send_command_three_08()
        time.sleep(6)
        self.next_step_event.clear()
        # 播放第二个包
        self.Track_control_left.track_control("smooth_gunarm3__master_joint_left.bag", event=self.next_step_event)
        # 等待事件被设置
        self.next_step_event.wait()
        if self.UseGunPoseAdj:
            self.next_step_event.clear()
            ret = self.get_rc1_json_string()
            if ret=="":
                print("error load value, no gun adjust skip!!!!")
            else:
                print("adjusting..................")
                self.start_gun_adj(ret,done_event=self.next_step_event)
                self.next_step_event.wait()

        time.sleep(1)
        self.gun.send_command_four()
        time.sleep(2)

        if self.UseGunPoseAdj:
            self.next_step_event.clear()
            ret = "rc0.json" #self.get_rc1_json_string()
            if ret=="":
                print("error load value, no gun adjust skip!!!!")
            else:
                print("adjusting..................")
                self.start_gun_adj(ret,done_event=self.next_step_event)
                self.next_step_event.wait()

        self.Track_control_left.track_control('smooth_gunarm4__master_joint_left.bag')
        time.sleep(3)
        
        #get bot

        self.Track_control_right.track_control("smooth_clawArm2__master_joint_right.bag",rate=1, event=self.next_step_event_clawarm)        #readypose-->getpose --->closeclaw  


        execute_path_3()  # jiance wendu
        time.sleep(3)
        self.startTempThread()

        time.sleep(1)

        #goto cool
        self.next_step_event_clawarm.clear()
        trjrate = float(self.selectedops_trj.get())
        #play_bag_and_wait("smooth_clawArm3__master_joint_right.bag", event=self.next_step_event_clawarm)       #getpose-->coolpose1   
        self.Track_control_right.track_control("smooth_clawArm3__master_joint_right.bag")

        self.next_step_event_clawarm.wait()
        

        #cooling
        while True:
            with self.lock:
                print(f"current temp {self.ismearingTemp} __ {self.currentTemp}")
                if not self.ismearingTemp:
                    break
            time.sleep(0.05)
        print('recording finished')

        execute_path_1()


        self.Track_control_right.track_control("smooth_clawArm4__master_joint_right.bag", event=self.next_step_event_clawarm)          #coolpose1-->coolpose2   


        #self.next_step_event_clawarm.clear()
        #play_bag_and_wait("smooth_clawArm5__master_joint_right.bag", event=self.next_step_event_clawarm)          #coolpose1-->coolpose2   
        #self.next_step_event_clawarm.wait() 
        self.ActionRuning = False

    def recordPnt1(self):
        self.start_gun_recorder("/master/joint_left","rc1.json")

    def recordPnt2(self):
        self.start_gun_recorder("/master/joint_left","rc2.json")

    def recordPnt3(self):
        self.start_gun_recorder("/master/joint_left","rc3.json")

    def recordPnt4(self):
        self.start_gun_recorder("/master/joint_left","rc4.json")

    def recordPnt5(self):
        self.start_gun_recorder("/master/joint_left","rc5.json")

    def recordPnt0(self):
        self.start_gun_recorder("/puppet/joint_left","rc0.json")

    def start_gun_adj(self,rcfilename, done_event,timeout=3):
        try:
            # 构造命令列表，确保正确传递参数给子进程
            command = ['python3', 'runGunPose2B.py', rcfilename]
            #command = ['python3', 'GunPntRcord.py', topic_name, output_filename]    
            print("start cmd gun adj")
            print(command)   
            # 启动子进程并等待其完成，同时捕获输出
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=timeout
            )
            done_event.set()
            return result.returncode, result.stdout, result.stderr
    
        except subprocess.TimeoutExpired:
            print("The recording process took too long and was terminated.")
            return -1, "", "Timeout expired"
        except Exception as e:
            print(f"An error occurred while starting the ROS recorder: {e}")
            return -2, "", str(e)
    

    def start_gun_recorder(self, topic_name, output_filename, timeout=10):

        try:
            # 构造命令列表，确保正确传递参数给子进程
            
            command = ['python3', 'GunPntRcord.py', topic_name, output_filename]    
            print("start cmd")
            print(command)   
            # 启动子进程并等待其完成，同时捕获输出
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=timeout
            )
            return result.returncode, result.stdout, result.stderr
    
        except subprocess.TimeoutExpired:
            print("The recording process took too long and was terminated.")
            return -1, "", "Timeout expired"
        except Exception as e:
            print(f"An error occurred while starting the ROS recorder: {e}")
            return -2, "", str(e)



    def RunAll(self):
        self.ActionRuning = True
        execute_path_1()  # jiance wendu
        open_gripper()

        self.next_step_event.clear()
        play_bag_and_wait("smooth_gunarm1__master_joint_left.bag", rate=0.8, event=self.next_step_event)      
        self.next_step_event.wait()
        time.sleep(1)
        # 重置事件
        self.next_step_event.clear()
        # 发送第一个命令
        self.gun.send_command_one()
        time.sleep(0.5)
        # 发送第二个命令
        self.gun.send_command_two()
        time.sleep(1)
        play_rosbag('smooth_clawArm1__master_joint_right.bag')
        time.sleep(1)

        self.next_step_event.clear()
        play_bag_and_wait("smooth_gunarm2__master_joint_left.bag", event=self.next_step_event)
        self.next_step_event.wait()
        time.sleep(1)
        self.gun.send_command_three_08()
        time.sleep(6)
        self.next_step_event.clear()
        # 播放第二个包
        play_bag_and_wait("smooth_gunarm3__master_joint_left.bag", event=self.next_step_event)
        # 等待事件被设置
        self.next_step_event.wait()
        time.sleep(1)
        self.gun.send_command_four()
        time.sleep(2)
        play_rosbag('smooth_gunarm4__master_joint_left.bag')
        time.sleep(3)
        self.startTempThread()

        self.next_step_event_clawarm.clear()
        play_bag_and_wait("smooth_clawArm2__master_joint_right.bag",rate=1.2, event=self.next_step_event_clawarm)        #readypose-->getpose --->closeclaw  
        self.next_step_event_clawarm.wait() 

        while True:
            with self.lock:
                print(f"current temp {self.ismearingTemp} __ {self.currentTemp}")
                if not self.ismearingTemp:
                    break
            time.sleep(0.05)
            
        print('recording finished')

        self.next_step_event_clawarm.clear()
        play_bag_and_wait("smooth_clawArm3__master_joint_right.bag", event=self.next_step_event_clawarm)       #getpose-->coolpose1   
        self.next_step_event_clawarm.wait()

        self.next_step_event_clawarm.clear()
        play_bag_and_wait("smooth_clawArm4__master_joint_right.bag", event=self.next_step_event_clawarm)          #coolpose1-->coolpose2   
        self.next_step_event_clawarm.wait() 
        self.ActionRuning = False



    def starttempRecord(self):
        print("temp thread starting record")
        with self.lock:
            self.ismearingTemp = True
        time.sleep(1)
        with self.tempLock:
            temper = self.currentTemp
        self.toggle_record()
        idx = 0
        while idx < 650 or (idx < 6000 and temper > self.tempTH):
            #self.currentTemp = self.read_temp()
            time.sleep(0.05)
            idx += 1
            with self.tempLock:
                temper = self.currentTemp
                print(f"current temp= {temper}")
                if self.recording and self.record_file is not None:
                    self.record_file.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')}, {self.currentTemp}\n")
            # time.sleep(5)  # temp mem
        self.toggle_record()
        with self.lock:
            self.ismearingTemp = False
        print("Temperature recording finished")
        print

    
    def startTempThread(self):
        print("reading temp started!")
        self.tempThread = threading.Thread(target=self.starttempRecord)
        self.tempThread.start()


    def startRunAllThread(self):
        if self.ActionRuning:
            return
        self.UseGunPoseAdj = False
        self.run_all_thread = threading.Thread(target=self.RunAll_1217)
        #self.run_all_thread = threading.Thread(target=self.RunOnlyGun)
        self.run_all_thread.start()

    def startRunAllThread_gunAdj(self):
        if self.ActionRuning:
            return
        self.UseGunPoseAdj = True
        self.run_all_thread2 = threading.Thread(target=self.RunAll_1217)
        #self.run_all_thread = threading.Thread(target=self.RunOnlyGun)
        self.run_all_thread2.start()


    def startRunAllThread_onlyGun(self):
        if self.ActionRuning:
            return
        self.UseGunPoseAdj = True
        self.run_all_thread1 = threading.Thread(target=self.RunOnlyGun)
        self.run_all_thread1.start()






if __name__ == "__main__":
    root = tk.Tk()
    app = newgui(root)
    root.mainloop()