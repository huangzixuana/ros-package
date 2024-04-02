#!/usr/bin/env python3

import re

# 读取路径文件
with open("path.txt", "r") as file:
    path_data = file.read()

# 使用正则表达式匹配每个姿态的位置和姿态消息，并转换为浮点数
pattern = re.compile(r"position:\s*\n\s*x: ([-\d.]+)\n\s*y: ([-\d.]+)\n\s*z: ([-\d.]+)\n\s*orientation:\s*\n\s*x: ([-\d.]+)\n\s*y: ([-\d.]+)\n\s*z: ([-\d.]+)\n\s*w: ([-\d.]+)")
matches = pattern.findall(path_data)

# 保存到新文件
with open("poses.txt", "w") as file:
    for match in matches:
        pose_values = [float(value) for value in match]
        file.write(" ".join(str(value) for value in pose_values) + "\n")
