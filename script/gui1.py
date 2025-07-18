import os
import tkinter as tk
from tkinter import Canvas
from robot import *

import serial

from tkinter import filedialog
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from gun import SerialController  # 假设 gun 模块中定义了 SerialController 类

from rosbagrecord import *
import threading

# 添加字体配置
import tkinter.font as tkFont


class CANStatusIndicator:
    def __init__(self, master):
        self.master = master
        self.master.configure(bg='#f0f0f0')
        #self.master.title("CAN Status Indicator")
        self.tempTH = 60
        self.tempLock = threading.Lock()

        self.master.title("Robot Control Panel")
        self.master.geometry("1850x1150+400+300")

        self.font = tkFont.Font(size=14, weight="bold")
        self.font2 = tkFont.Font(size=18, weight="bold")


        # 创建一个Canvas小部件用于显示状态指示器
        self.canvas = Canvas(self.master, width=350, height=30)
        self.canvas.pack()

        self.canvas.create_rectangle(0, 0, 360, 30, fill="white")
       # self.canvas_txt = Canvas(self.master, width=350, height=30)
       # self.canvas.pack()
        self.statext = tk.Label(self.canvas,text="lefthand  righthand   base",font=self.font)
        self.statext.pack(pady=20)


        # 创建一个Label组件用来显示温度
        self.temperature_label = tk.Label(self.master, text="Current Temperature: -")
        self.temperature_label.pack(pady=20)


        # 创建一个蓝色背景的帧来容纳第一组按钮
        blue_frame = tk.Frame(self.master, bg="#ADD8E6", width=300, height=350)
        blue_frame.place(x=50,y=150)
        blue_frame.pack_propagate(False)  # 防止框架因子控件而改变大小
        blue_frame.grid_propagate(False)  # 防止框架因网格子控件而改变大小

        # 配置网格权重，使按钮在框架中均匀分布
        for i in range(7):
            blue_frame.grid_rowconfigure(i, weight=1)
        blue_frame.grid_columnconfigure(0, weight=1)

        # 在蓝色帧内创建按钮
        self.open_can_button = tk.Button(blue_frame, text="Open CAN", font = self.font, command=open_can)
        self.open_can_button.grid(row=0, column=0, padx=20, pady=10, sticky="ew")

        self.start_robot_button = tk.Button(blue_frame, text="Start Robot", font = self.font, command=start_robot)
        self.start_robot_button.grid(row=1, column=0, padx=20, pady=10, sticky="ew")

        self.execute_path_1_button = tk.Button(blue_frame, text="Detect Position 0", font = self.font, command=execute_path_1)
        self.execute_path_1_button.grid(row=2, column=0, padx=20, pady=10, sticky="ew")

        self.execute_path_2_button = tk.Button(blue_frame, text="Detect Temperature", font = self.font, command=execute_path_2)
        self.execute_path_2_button.grid(row=3, column=0, padx=20, pady=10, sticky="ew")

        self.execute_path_3_button = tk.Button(blue_frame, text="Detect Spectrum", font = self.font, command=execute_path_3)
        self.execute_path_3_button.grid(row=4, column=0, padx=20, pady=10, sticky="ew")

        self.close_gripper_button = tk.Button(blue_frame, text="Open Gripper", font = self.font, command=open_gripper, bg="#f9e5b7")
        self.close_gripper_button.grid(row=5, column=0, padx=20, pady=10, sticky="ew")

        self.close_gripper_button = tk.Button(blue_frame, text="Close Gripper", font = self.font, command=close_gripper, bg="#f9e5b7")
        self.close_gripper_button.grid(row=6, column=0, padx=20, pady=10, sticky="ew")

        # 创建一个绿色背景的帧来容纳第二组按钮
        green_frame = tk.Frame(self.master, bg="#a8e6cf", width=350, height=350)
        green_frame.place(x=850,y=150)
        green_frame.pack_propagate(False)  # 防止框架因子控件而改变大小
        green_frame.grid_propagate(False)  # 防止框架因网格子控件而改变大小

        # 配置网格权重，使按钮在框架中均匀分布
        for i in range(6):
            green_frame.grid_rowconfigure(i, weight=1)
        green_frame.grid_columnconfigure(0, weight=1)

        self.start_robot_button_train = tk.Button(green_frame, text="Start Robot_claw", font = self.font, command=start_Claw_lsn)
        self.start_robot_button_train.grid(row=0, column=0, padx=20, pady=10, sticky="ew")

        self.execute_path_4_button = tk.Button(green_frame, text="Start Robot_base", font = self.font, command=start_dipan)
        self.execute_path_4_button.grid(row=1, column=0, padx=20, pady=10, sticky="ew")

        self.execute_path_5_button = tk.Button(green_frame, text="actbot", font = self.font, command=execute_path_5)
        self.execute_path_5_button.grid(row=2, column=0, padx=20, pady=10, sticky="ew")

        self.execute_path_6_button = tk.Button(green_frame, text="putandreturn", font = self.font, command=execute_path_6)
        self.execute_path_6_button.grid(row=3, column=0, padx=20, pady=10, sticky="ew")

        self.initialize_robot_button = tk.Button(green_frame, text="Robot Initialize", font = self.font, command=initialize_robot, bg="#FFD700")
        self.initialize_robot_button.grid(row=4, column=0, padx=20, pady=10, sticky="ew")

        self.stop_robot_button = tk.Button(green_frame, text="Robot Emergency Stop", font = self.font, command=stop_robot)
        self.stop_robot_button.grid(row=5, column=0, padx=20, pady=10, sticky="ew")


        # 创建一个绿色背景的帧来容纳第二组按钮
        self.gun_frame = tk.Frame(self.master, bg="#dcedc1", width=250, height=350)
        self.gun_frame.place(x=350,y=150)
        self.gun_frame.pack_propagate(False)  # 防止框架因子控件而改变大小
        self.gun_frame.grid_propagate(False)  # 防止框架因网格子控件而改变大小

        # 配置网格权重，使按钮在框架中均匀分布
        self.gun_frame.grid_rowconfigure(0, weight=1)
        self.gun_frame.grid_rowconfigure(1, weight=1)
        self.gun_frame.grid_rowconfigure(2, weight=1)
        self.gun_frame.grid_rowconfigure(3, weight=1)
        self.gun_frame.grid_rowconfigure(4, weight=1)
        self.gun_frame.grid_columnconfigure(0, weight=1)

        self.initGunBtn = tk.Button( self.gun_frame, text="initGun", font = self.font, command=self.init_GunFun)
        self.initGunBtn.grid(row=0, column=0, padx=20, pady=20, sticky="ew")

        self.addLqBtn = tk.Button( self.gun_frame, text="Addlq_air", font = self.font, command=self.add_LqCmd_air)
        self.addLqBtn.grid(row=1, column=0, padx=20, pady=20, sticky="ew")

        self.addLq3mlBtn = tk.Button( self.gun_frame, text="Lq_3ml", font = self.font, command=self.LqCmd_3ml)
        self.addLq3mlBtn.grid(row=2, column=0, padx=20, pady=20, sticky="ew")

        self.addLqTu3mlBtn = tk.Button( self.gun_frame, text="Lq_tu3ml", font = self.font, command=self.LqCmd_tu3ml)
        self.addLqTu3mlBtn.grid(row=3, column=0, padx=20, pady=20, sticky="ew")




         # 添加一个新的面板用于显示温度
        temp_panel = tk.Frame(self.master, bg="#f6f8f7", width=250, height=350)
        temp_panel.place(x=600,y=150)
        temp_panel.pack_propagate(False)  # 防止框架因子控件而改变大小
        temp_panel.grid_propagate(False)  # 防止框架因网格子控件而改变大小

        # 配置网格权重，使按钮在框架中均匀分布
        temp_panel.grid_rowconfigure(0, weight=1)
        temp_panel.grid_rowconfigure(1, weight=1)
        temp_panel.grid_columnconfigure(0, weight=1)

        # 在温度面板中添加一个按钮用于控制温度记录
        self.record_button = tk.Button(temp_panel, text="Start \nRecording", font = self.font, command=self.toggle_record)
        self.record_button.grid(row=0, column=0, padx=20, pady=10, sticky="ew")
         # 添加一个按钮来打开绘图窗口
        #plot_panel = tk.Frame(self.master, bg="white", width=350, height=50)
        #plot_panel.pack(pady=10)
        self.plot_temperature_button = tk.Button(temp_panel, text="Plot \nTemperature\nCurve", font = self.font, command=self.plot_temperature)
        self.plot_temperature_button.grid(row=1, column=0, padx=20, pady=10, sticky="ew")

        #elf.plot_temperature_button = tk.Button(temp_panel, text="绘制温度曲线", command=self.plot_temperature)
        #self.plot_temperature_button.pack(side=tk.TOP, padx=10, pady=10)

        # 创建一个绿色背景的帧来容纳第二组按钮
        recorder_frame = tk.Frame(self.master, bg="#c3d6e5", width=1750, height=350)
        recorder_frame.place(x=50,y=550)
        recorder_frame.pack_propagate(False)  # 防止框架因子控件而改变大小
        recorder_frame.grid_propagate(False)  # 防止框架因网格子控件而改变大小

        # 配置网格权重，使按钮在框架中均匀分布
        for i in range(3):
            recorder_frame.grid_rowconfigure(i, weight=1)
        for j in range(7):
            recorder_frame.grid_columnconfigure(j, weight=1)

        self.Switch = tk.Button(recorder_frame, text="playMod", font = self.font, command=self.switchfun,bg="#ffab91")
        self.Switch.grid(row=0,column=0, padx=10, pady=10, sticky="ew")
        #self.Switch.

        self.Rcstop = tk.Button(recorder_frame, text="StopRecord", font = self.font, command=self.jilu_stop)
        self.Rcstop.grid(row=0,column=1, padx=10, pady=10, sticky="ew")

        self.GunArm1 = tk.Button(recorder_frame, text="GunArm1", font = self.font, command=self.jilu_gunarm_1)
        self.GunArm1.grid(row=0,column=2, padx=10, pady=10, sticky="ew")

        self.GunArm2 = tk.Button(recorder_frame, text="GunArm2", font = self.font, command=self.jilu_gunarm_2)
        self.GunArm2.grid(row=0,column=3, padx=10, pady=10, sticky="ew")

        self.GunArm3 = tk.Button(recorder_frame, text="GunArm3", font = self.font, command=self.jilu_gunarm_3)
        self.GunArm3.grid(row=0,column=4,padx=10, pady=10, sticky="ew")

        self.GunArm4 = tk.Button(recorder_frame, text="GunArm4", font = self.font, command=self.jilu_gunarm_4)
        self.GunArm4.grid(row=0,column=5,padx=10, pady=10, sticky="ew")

        self.GunArm5 = tk.Button(recorder_frame, text="GunArm5", font = self.font, command=self.jilu_gunarm_5)
        self.GunArm5.grid(row=0,column=6, padx=10, pady=10, sticky="ew")

        self.ClawArm1 = tk.Button(recorder_frame, text="ClawArm1", font = self.font, command=self.jilu_clawArm_1)
        self.ClawArm1.grid(row=1,column=0, padx=10, pady=10, sticky="ew")

        self.ClawArm2 = tk.Button(recorder_frame, text="ClawArm2", font = self.font, command=self.jilu_clawArm_2)
        self.ClawArm2.grid(row=1,column=1, padx=10, pady=10, sticky="ew")

        self.ClawArm3 = tk.Button(recorder_frame, text="ClawArm3", font = self.font, command=self.jilu_clawArm_3)
        self.ClawArm3.grid(row=1,column=2, padx=10, pady=10, sticky="ew")

        self.ClawArm4 = tk.Button(recorder_frame, text="ClawArm4", font = self.font, command=self.jilu_clawArm_4)
        self.ClawArm4.grid(row=1,column=3, padx=10, pady=10, sticky="ew")

        self.ClawArm5 = tk.Button(recorder_frame, text="ClawArm5", font = self.font, command=self.jilu_clawArm_5)
        self.ClawArm5.grid(row=1,column=4, padx=10, pady=10, sticky="ew")

        self.ClawArm6 = tk.Button(recorder_frame, text="ClawArm6", font = self.font, command=self.jilu_clawArm_6)
        self.ClawArm6.grid(row=1,column=5, padx=10, pady=10, sticky="ew")

        self.ClawArm7 = tk.Button(recorder_frame, text="ClawArm7", font = self.font, command=self.jilu_clawArm_7)
        self.ClawArm7.grid(row=1,column=6, padx=10, pady=10, sticky="ew")

        self.ClawArm2_a = tk.Button(recorder_frame, text="ClawArm2_toair", font = self.font, command=self.jilu_clawArm_2_toair)
        self.ClawArm2_a.grid(row=2,column=1, padx=10, pady=10, sticky="ew")

        self.ClawArm3_a = tk.Button(recorder_frame, text="ClawArm3_toair", font = self.font, command=self.jilu_clawArm_3_toair)
        self.ClawArm3_a.grid(row=2,column=2, padx=10, pady=10, sticky="ew")

        self.ClawArm4_a = tk.Button(recorder_frame, text="ClawArm4_toair", font = self.font, command=self.jilu_clawArm_4_toair)
        self.ClawArm4_a.grid(row=2,column=3, padx=10, pady=10, sticky="ew")


        seqFrame = tk.Frame(self.master, bg="#b3e0d6", width=800, height=200)
        seqFrame.place(x=500,y=950)
        seqFrame.pack_propagate(False)  # 防止框架因子控件而改变大小
        seqFrame.grid_propagate(False)  # 防止框架因网格子控件而改变大小

        # 配置网格权重，使按钮在框架中均匀分布
        for i in range(2):
            seqFrame.grid_rowconfigure(i, weight=1)
        for j in range(3):
            seqFrame.grid_columnconfigure(j, weight=1)

        self.addWaterSeqBtn = tk.Button(seqFrame, text="AddWaterSeq", font = self.font, command=self.runseq_AddWater)
        self.addWaterSeqBtn.grid(row=0,column=1, padx=10, pady=10, sticky="ew")

        self.botprocessbtn = tk.Button(seqFrame, text="BotProcess", font = self.font, command=self.runseq_ProcessBot)
        self.botprocessbtn.grid(row=0,column=2, padx=10, pady=10, sticky="ew")

                # 创建一个标签
        label = tk.Label(seqFrame, font = self.font, text="Please enter TEMP:")
        label.grid(row=1,column=0, padx=10, pady=10, sticky="ew")

        # 创建一个输入框
        self.entry = tk.Entry(seqFrame, width=10, font=self.font)
        self.entry.insert(0, '60')  # 设置初始值
        self.entry.grid(row=1,column=1, padx=10, pady=10, sticky="ew")
       
        
        # 定义定时任务，每1000毫秒更新一次状态
        self.master.after(1000, self.check_device)

        #temp module
        self.tempser = serial.Serial(port='/dev/serialTempreMem', baudrate=57600, timeout=1)   
        if not self.tempser.is_open:
            self.tempser.open()
        print("temp port open!")
        self.command_readTemp = b'\x03\x03\x00\x02\x00\x02\x64\x29'
        self.currentTemp = 0.0
        # 定义定时任务，每100毫秒更新一次状态
        self.master.after(15, self.read_temp_cyc)
        #temp module

        # 初始状态，灰色表示未知
        self.status = "unknown"
        self.recording = False
        self.record_file = None
        self.record_start_time = None

        #self.lable_left =  self.canvas.create_rectangle(0, 0, 50, 30, fill="lightgreen")
        #self.lable_right =  self.canvas.create_rectangle(60, 0, 110, 30, fill="lightgreen")
        #self.lable_base =  self.canvas.create_rectangle(120, 0, 170, 30, fill="lightgreen")

        self.update_status()
        # self.gun = SerialController()  # 假设已经定义了SerialController类

        self.recordmod = False
           # 创建一个Event对象用于同步
        self.next_step_event = threading.Event()
        self.next_step_event_clawarm = threading.Event()



    # gun action
    def init_GunFun(self):
        self.gun.send_command_one() #initial
    
    def add_LqCmd_air(self):
        self.gun.send_command_two() #initial add

    def LqCmd_3ml(self):
        self.gun.send_command_three_08() #add

    def LqCmd_tu3ml(self):
        self.gun.send_command_four() #tu

    def switchfun(self):
        if self.recordmod == True:
            self.recordmod = False
            self.Switch.config(bg='red')
            self.Switch.config(text='playMod')
        else:
            self.recordmod = True
            self.Switch.config(bg='green')
            self.Switch.config(text='recordMod')

    #recorder 
    def jilu_stop(self):
        stop_rosbag_recording(self.recordprocess)
    # gun arm move record
    def jilu_gunarm_1(self):
        if(self.recordmod):
            self.recordprocess = start_rosbag_recording(output_filename = "gunarm1.bag")#init-->upbot
        else:
            self.playprocess = play_rosbag(bag_file="gunarm1.bag")

    def jilu_gunarm_2(self):
        if(self.recordmod):
            self.recordprocess = start_rosbag_recording(output_filename = "gunarm2.bag")#init-->upbot
        else:
            self.playprocess = play_rosbag(bag_file="gunarm2.bag")
       # self.recordprocess = start_rosbag_recording(output_filename = "gunarm2.bag")#upbot-->insidebot

    def jilu_gunarm_3(self):
        if(self.recordmod):
            self.recordprocess = start_rosbag_recording(output_filename = "gunarm3.bag")#init-->upbot
        else:
            self.playprocess = play_rosbag(bag_file="gunarm3.bag")
        #self.recordprocess = start_rosbag_recording(output_filename = "gunarm3.bag")#upbot--->addpose

    def jilu_gunarm_4(self):
        if(self.recordmod):
            self.recordprocess = start_rosbag_recording(output_filename = "gunarm4.bag")#init-->upbot
        else:
            self.playprocess = play_rosbag(bag_file="gunarm4.bag")
        #self.recordprocess = start_rosbag_recording(output_filename = "gunarm4.bag")#addpose--->upbot

    def jilu_gunarm_5(self):
        if(self.recordmod):
            self.recordprocess = start_rosbag_recording(output_filename = "gunarm5.bag")#init-->upbot
        else:
            self.playprocess = play_rosbag(bag_file="gunarm5.bag")
       # self.recordprocess = start_rosbag_recording(output_filename = "gunarm5.bag")#upbot--->init  #finish
    
       # self.master.after(100,self.read_temp)
    def jilu_clawArm_1(self):
        if(self.recordmod):
            self.recordprocess = start_rosbag_recording(output_filename = "clawArm1.bag")
        else:
            self.playprocess = play_rosbag(bag_file="clawArm1.bag")
        #self.recordprocess = start_rosbag_recording(output_filename = "clawArm1.bag")#init-->readypose

    def jilu_clawArm_2(self):
        if(self.recordmod):
            self.recordprocess = start_rosbag_recording(output_filename = "clawArm2.bag")
        else:
            self.playprocess = play_rosbag(bag_file="clawArm2.bag")
        #self.recordprocess = start_rosbag_recording(output_filename = "clawArm2.bag")#readypose-->getpose --->closeclaw
    def jilu_clawArm_3(self):
        if(self.recordmod):
            self.recordprocess = start_rosbag_recording(output_filename = "clawArm3.bag")#init-->upbot
        else:
            self.playprocess = play_rosbag(bag_file="clawArm3.bag")
        #self.recordprocess = start_rosbag_recording(output_filename = "clawArm3.bag")#gerpose-->readypose2

    def jilu_clawArm_4(self):
        if(self.recordmod):
            self.recordprocess = start_rosbag_recording(output_filename = "clawArm4.bag")#init-->upbot
        else:
            self.playprocess = play_rosbag(bag_file="clawA当前rm4.bag")
        #self.recordprocess = start_rosbag_recording(output_filename = "clawArm4.bag")#readypose2-->coolpose
                                                                                                                #cooling
    def jilu_clawArm_5(self):
        if(self.recordmod):
            self.recordprocess = start_rosbag_recording(output_filename = "clawArm5.bag")#init-->upbot
        else:
            self.playprocess = play_rosbag(bag_file="clawArm5.bag")
        #self.recordprocess = start_rosbag_recording(output_filename = "clawArm5.bag")#coolpose-->putpose1    # openclaw
    
    def jilu_clawArm_6(self):
        if(self.recordmod):
            self.recordprocess = start_rosbag_recording(output_filename = "clawArm6.bag")#init-->upbot
        else:
            self.playprocess = play_rosbag(bag_file="clawArm6.bag")
        #self.recordprocess = start_rosbag_recording(output_filename = "clawArm6.bag")#putpose1-->readypose       
    def jilu_clawArm_7(self):
        if(self.recordmod):
            self.recordprocess = start_rosbag_recording(output_filename = "clawArm7.bag")#init-->upbot
        else:
            self.playprocess = play_rosbag(bag_file="clawArm7.bag")
        #self.recordprocess = start_rosbag_recording(output_filename = "clawArm7.bag")#readypose-->init    # finish

    def jilu_clawArm_2_toair(self):
        if(self.recordmod):
            self.recordprocess = start_rosbag_recording(output_filename = "clawArm2_air.bag")#init-->upbot
        else:
            self.playprocess = play_rosbag(bag_file="clawArm2_air.bag")

    def jilu_clawArm_3_toair(self):
        if(self.recordmod):
            self.recordprocess = start_rosbag_recording(output_filename = "clawArm3_air.bag")#init-->upbot
        else:
            self.playprocess = play_rosbag(bag_file="clawArm3_air.bag")

    def jilu_clawArm_4_toair(self):
        if(self.recordmod):
            self.recordprocess = start_rosbag_recording(output_filename = "clawArm4_air.bag")#init-->upbot
        else:
            self.playprocess = play_rosbag(bag_file="clawArm4_air.bag")


    def runseq_AddWater(self):
        self.next_step_event.clear()
        play_bag_and_wait("gunarm1.bag", event=self.next_step_event)      
        self.next_step_event.wait()
        time.sleep(1)
        # 重置事件
        self.next_step_event.clear()
        # 发送第一个命令
        self.gun.send_command_one()
        time.sleep(2)
        # 发送第二个命令
        self.gun.send_command_two()
        time.sleep(2)

        self.next_step_event.clear()
        play_bag_and_wait("gunarm2.bag", event=self.next_step_event)
        self.next_step_event.wait()
        time.sleep(1)

        self.gun.send_command_three_08()
        time.sleep(3)

        self.next_step_event.clear()
        # 播放第二个包
        play_bag_and_wait("gunarm3.bag", event=self.next_step_event)
        # 等待事件被设置
        self.next_step_event.wait()
        time.sleep(1)

        self.gun.send_command_four()
        time.sleep(3)

        self.next_step_event.clear()
        # 播放第二个包
        play_bag_and_wait("gunarm4.bag", event=self.next_step_event)
        # 等待事件被设置
        self.next_step_event.wait()
        time.sleep(1)

    def runseq_ProcessBot(self):
        self.tempTH = float(self.entry.get())
        print(f"experiment using temp: {self.tempTH}")

        self.next_step_event_clawarm.clear()
        play_bag_and_wait("clawArm1.bag", event=self.next_step_event_clawarm)     #init-->readypose 
        #open_gripper()
        self.next_step_event_clawarm.wait() 
        time.sleep(1)

        self.next_step_event_clawarm.clear()
        play_bag_and_wait("clawArm2.bag", event=self.next_step_event_clawarm)        #readypose-->getpose --->closeclaw  
        self.next_step_event_clawarm.wait() 
        #close_gripper()
        execute_path_3() #jiance wendu

        time.sleep(3)
        self.currentTemp = self.read_temp()
        self.toggle_record()      
        idx = 0
        while idx<3000 and self.currentTemp>self.tempTH :
            self.currentTemp = self.read_temp()
            time.sleep(0.05)
            idx = idx +1
            temper = self.currentTemp
            print(f"current temp= {temper}")
            if self.recording and self.record_file is not None:
               self.record_file.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')}, {self.currentTemp}\n")

        #time.sleep(5) #temp mem
        self.toggle_record()   

        self.next_step_event_clawarm.clear()
        play_bag_and_wait("clawArm3.bag", event=self.next_step_event_clawarm)       #getpose-->coolpose1   
        self.next_step_event_clawarm.wait()

        self.next_step_event_clawarm.clear()
        play_bag_and_wait("clawArm4.bag", event=self.next_step_event_clawarm)          #coolpose1-->coolpose2   
        self.next_step_event_clawarm.wait()         # if not self.tempser.is_open:
        #     print("temp error, return 0")
        #     return 9999
        # self.tempser.write(self.command_readTemp)
        # # 接收响应
        # response = self.tempser.read(5 + 2 * 2)  # 8字节头部信息加上2个寄存器的数据
        # data = response[3:7]
        # #print(data.hex())
        # temp_32bit = int.from_bytes(data,byteorder='big', signed=True)
        # temperature = temp_32bit / 100.0
        # return temperature
        self.tempTH = float(self.entry.get())
        print(f"experiment using temp: {self.tempTH}")

        self.next_step_event_clawarm.clear()
        play_bag_and_wait("clawArm1.bag", event=self.next_step_event_clawarm)     #init-->readypose 
        #open_gripper()
        self.next_step_event_clawarm.wait() 
        time.sleep(1)

        self.next_step_event_clawarm.clear()
        play_bag_and_wait("clawArm2_toair.bag", event=self.next_step_event_clawarm)        #readypose-->getpose --->closeclaw  
        self.next_step_event_clawarm.wait() 
        #close_gripper()
        execute_path_3() #jiance wendu

        time.sleep(3)
        self.currentTemp = self.read_temp()
        self.toggle_record()      
        idx = 0
        while idx<2000 and self.currentTemp>self.tempTH :
            self.currentTemp = self.read_temp()
            time.sleep(0.05)
            idx = idx +1
            temper = self.currentTemp
            print(f"current temp= {temper}")
            if self.recording and self.record_file is not None:
               self.record_file.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')}, {self.currentTemp}\n")

        #time.sleep(5) #temp mem
        self.toggle_record()   

        self.next_step_event_clawarm.clear()
        play_bag_and_wait("clawArm3_toair.bag", event=self.next_step_event_clawarm)       #getpose-->coolpose1   
        self.next_step_event_clawarm.wait()

        self.next_step_event_clawarm.clear()
        play_bag_and_wait("clawArm4_toair.bag", event=self.next_step_event_clawarm)          #coolpose1-->coolpose2   
        self.next_step_event_clawarm.wait() 


    def read_temp(self):
        if not self.tempser.is_open:
            print("temp error, return 0")
            return 9999
        self.tempser.write(self.command_readTemp)
        # 接收响应
        response = self.tempser.read(5 + 2 * 2)  # 8字节头部信息加上2个寄存器的数据
        data = response[3:7]
        #print(data.hex())
        temp_32bit = int.from_bytes(data,byteorder='big', signed=True)
        temperature = temp_32bit / 100.0
        return temperature

    
    def read_temp_cyc(self):
        with self.tempLock:
            self.currentTemp = self.read_temp()
        if self.recording and self.record_file is not None:
            self.record_file.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')}, {self.currentTemp}\n")
        self.master.after(50, self.read_temp_cyc)

    def plot_temperature(self):
        filename = filedialog.askopenfilename(title="选择温度文件", filetypes=[("Text files", "*.txt")])
        if filename:
            self.plot_temperature_from_file(filename)

    def plot_temperature_from_file(self, filename):
        temps = []
        with open(filename, 'r') as file:
            next(file)  # 跳过标题行
            for line in file:
                _, temp_str = line.strip().split(', ')
                try:
                    temp = float(temp_str)
                    temps.append(temp)
                except ValueError:
                    print(f"无效的温度值: {temp_str}")

        fig, ax = plt.subplots()
        ax.plot(range(len(temps)), temps)
        ax.set_title('temp')
        ax.set_xlabel('id')
        ax.set_ylabel('value (°C)')
        ax.set_ylim([-40, 160])  # 设置y轴范围

        # 使用Tkinter嵌入matplotlib图形
        plot_window = tk.Toplevel(self.master)
        plot_window.title('temp cyc')
        canvas = FigureCanvasTkAgg(fig, master=plot_window)
        canvas.draw()
        canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)



    def update_status(self):
        if self.status == "exists":
            self.canvas.create_rectangle(30, 0, 90, 30, fill="lightgreen")
            # 如果设备存在，显示绿色
            #self.canvas.create_rectangle(0, 0, 350, 30, fill="lightgreen")
            #self.canvas.create_text('left hand')
           # self.open_can_button.config(state=tk.DISABLED)
           # self.open_can_button.config(text="can opened")
        elif self.status == "not_exists":
            # 如果设备不存在，显示灰色
            self.canvas.create_rectangle(30, 0, 90, 30, fill="gray")
            self.open_can_button.config(state=tk.NORMAL)
            self.open_can_button.config(text="open can")
        else:
            # 如果状态未知，显示白色
            self.canvas.create_rectangle(30, 0, 90, 30, fill="white")
        newtmp = self.read_temp()
        #print(newtmp)
        self.temperature_label.config(text=f"当前温度: {newtmp}°C", font=self.font2)

    def check_device(self):
        device_path = "/dev/canable1"
        device_path2 = "/dev/canable3"
        device_path3 = "/dev/canable4"
        if os.path.exists(device_path):
            self.status = "exists"
        else:
            self.status = "not_exists"
        if os.path.exists(device_path2):
            self.canvas.create_rectangle(160, 0, 220, 30, fill="lightgreen")
        else:
            self.canvas.create_rectangle(160, 0, 220, 30, fill="gray")
        if os.path.exists(device_path3):
            self.canvas.create_rectangle(290, 0, 340, 30, fill="lightgreen")
        else:
            self.canvas.create_rectangle(290, 0, 340, 30, fill="gray")

        self.update_status()
        # 重新调度任务，每1秒执行一次
        self.master.after(1000, self.check_device)


    def toggle_record(self):
        if not self.recording:
            # 开始记录
            self.record_start_time = time.strftime("%Y%m%d_%H%M%S")
            self.record_file = open(f"temperature_{self.record_start_time}.txt", "w")
            self.record_file.write("时间, 温度\n")
            self.record_button.config(text="停止记录")
            self.recording = True
            print(f"start recording temp {self.record_start_time }")
        else:
            # 停止记录
            self.record_file.close()
            self.record_button.config(text="开始记录")
            self.recording = False
            print(f"stop recording temp {self.record_start_time}.txt")

if __name__ == "__main__":
    root = tk.Tk()
    app = CANStatusIndicator(root)
    root.mainloop()