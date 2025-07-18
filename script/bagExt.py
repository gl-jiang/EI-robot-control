import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import rosbag

# 用于存储 bag 文件信息的全局变量
input_bag_path = None
topics_info = {}
selected_topic = None  # 初始化 selected_topic 为 StringVar

def load_rosbag():
    global input_bag_path, topics_info
    # 打开文件对话框选择 .rosbag 文件
    input_bag_path = filedialog.askopenfilename(filetypes=[("ROS Bag files", "*.bag")])
    if not input_bag_path:
        return  # 如果没有选择文件，则退出
    
    try:
        with rosbag.Bag(input_bag_path, 'r') as bag:
            # 获取话题列表及消息数量
            topics_info = {topic: bag.get_message_count(topic) for topic in bag.get_type_and_topic_info()[1].keys()}
        
        # 更新 GUI 显示话题和消息数
        update_topics_display()
    except Exception as e:
        messagebox.showerror("Error", f"Failed to load the rosbag: {e}")

def update_topics_display():
    # 清除之前的显示
    for widget in topics_frame.winfo_children():
        widget.destroy()
    
    # 显示每个话题及其消息数
    for topic, count in topics_info.items():
        tk.Label(topics_frame, text=f"{topic} ({count} messages)").pack(anchor='w')
    
    # 更新下拉菜单
    if topics_info:
        # 清空下拉菜单
        topics_menu['values'] = []
        
        # 添加新的话题到下拉菜单
        topics_menu['values'] = list(topics_info.keys())
        topics_menu.current(0)  # 默认选中第一个话题
        selected_topic.set(topics_menu['values'][0])  # 设置 selected_topic 的值
    else:
        selected_topic.set('')

def extract_and_save_topic():
    global input_bag_path, topics_info
    if not input_bag_path:
        messagebox.showwarning("Warning", "Please load a rosbag first.")
        return
    
    topic_to_extract = selected_topic.get()
    if not topic_to_extract or topic_to_extract not in topics_info:
        messagebox.showwarning("Warning", "No valid topic selected. Please select a topic from the list.")
        return
    
    try:
        # 构建输出文件名
        output_bag_name = f"{input_bag_path.rsplit('.', 1)[0]}_{topic_to_extract.replace('/', '_')}.bag"
        
        with rosbag.Bag(input_bag_path, 'r') as input_bag, rosbag.Bag(output_bag_name, 'w') as output_bag:
            # 提取指定话题的消息
            for topic, msg, t in input_bag.read_messages(topics=[topic_to_extract]):
                # 将消息写入新文件
                output_bag.write(topic, msg, t)
        
        messagebox.showinfo("Success", f"Topic '{topic_to_extract}' extracted and saved to {output_bag_name} successfully.")
    except Exception as e:
        messagebox.showerror("Error", f"Failed to extract and save the topic: {e}")

# 创建主窗口
root = tk.Tk()
root.title("Rosbag Topic Extractor")

selected_topic=tk.StringVar()

# 创建按钮以加载 rosbag
load_button = tk.Button(root, text="Load Rosbag", command=load_rosbag)
load_button.pack(pady=10)

# 创建框架以显示话题信息
topics_frame = tk.Frame(root)
topics_frame.pack(fill=tk.X, padx=10, pady=5)

# 创建下拉菜单以选择话题
topics_menu = ttk.Combobox(root, textvariable=selected_topic, state="readonly")
topics_menu.pack(pady=5)

# 创建按钮以提取并保存指定话题
extract_button = tk.Button(root, text="Extract & Save Topic", command=extract_and_save_topic)
extract_button.pack(pady=10)

# 启动事件循环
root.mainloop()