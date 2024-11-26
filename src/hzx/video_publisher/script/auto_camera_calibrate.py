#!/usr/bin/env python3
# coding=utf-8

import tkinter as tk
import os
import subprocess
import rospy
from rosgraph_msgs.msg import Log

def create_gui():
    global root, rows_entry, columns_entry, square_entry, image_entry, camera_entry

    # 创建窗口
    root = tk.Tk()
    root.title("Camera Calibration Parameters")

    # 创建输入框和标签
    rows_label = tk.Label(root, text="行数：")
    rows_label.grid(row=0, column=0)
    rows_entry = tk.Entry(root)
    rows_entry.insert(0, "11")  # 设置默认值
    rows_entry.grid(row=0, column=1)

    columns_label = tk.Label(root, text="列数：")
    columns_label.grid(row=1, column=0)
    columns_entry = tk.Entry(root)
    columns_entry.insert(0, "8")  # 设置默认值
    columns_entry.grid(row=1, column=1)

    square_label = tk.Label(root, text="单格尺寸：")
    square_label.grid(row=2, column=0)
    square_entry = tk.Entry(root)
    square_entry.insert(0, "0.03")  # 设置默认值
    square_entry.grid(row=2, column=1)

    image_label = tk.Label(root, text="图像接收:")
    image_label.grid(row=3, column=0)
    image_entry = tk.Entry(root)
    image_entry.insert(0, "/camera/image_raw")  # 设置默认值
    image_entry.grid(row=3, column=1)

    camera_label = tk.Label(root, text="摄像头:")
    camera_label.grid(row=4, column=0)
    camera_entry = tk.Entry(root)
    camera_entry.insert(0, "/camera")  # 设置默认值
    camera_entry.grid(row=4, column=1)

    # 创建按钮
    run_button = tk.Button(root, text="开始标定", command=run_calibration)
    run_button.grid(row=5, columnspan=2)

    # 运行窗口
    root.mainloop()

def run_calibration():
    global root, rows_entry, columns_entry, square_entry, image_entry, camera_entry
    rows = rows_entry.get()
    columns = columns_entry.get()
    square = square_entry.get()
    image = image_entry.get()
    camera = camera_entry.get()

    # roslaunch usb_cam usb_cam-test.launch
    # rosrun camera_calibration cameracalibrator.py --size 10x7 --square 0.022 image:=/usb_cam/image_raw camera:=/usb_cam

    # 构建完整的命令
    # my_command = "roslaunch usb_cam usb_cam-test.launch"
    my_command2 = f"rosrun camera_calibration cameracalibrator.py --size {rows}x{columns} --square {square} image:={image} camera:={camera}"

    # 在新的gnome-terminal中运行命令
    # os.system(f"gnome-terminal -- bash -c '{my_command}; exec bash'")
    os.system(f"gnome-terminal -- bash -c '{my_command2}; exec bash'")

    # 关闭窗口
    root.destroy()


    # 启动ROS节点，监听rosout话题
    def log_callback(msg):
        if "writing calibration data to" in msg.msg:
            rospy.loginfo("相机标定程序结束")
            # 执行文件移动操作
            move_command = [
                "mv",
                "/home/nvidia/.ros/camera_info/head_camera.yaml",
                "/home/nvidia/Downloads/",
            ]
            subprocess.Popen(move_command)
            rospy.loginfo("mv to /home/nvidia/Downloads/")
            rospy.signal_shutdown("Terminating node as requested.")

    rospy.init_node("subrosout")
    sub = rospy.Subscriber("rosout", Log, log_callback)
    rospy.spin()


if __name__ == "__main__":
    create_gui()