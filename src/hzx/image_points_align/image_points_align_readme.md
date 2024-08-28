README

1.作用: 检测光伏组件阵列断点;

2.启动:

`source devel/setup.bash`

`roslaunch image_points_align image_points_align.launch`

3.参数设置

直接在launch文件里设置

`rgb_image_topic: /camera/color/image_raw     接收图像topic
            depth_cloud_topic: /camera/depth/color/points    接收点云topic
            trig_name: solar_gap_detect  `     trig开关名字，打开和关闭此程序

4. 使用本程序前提条件

   启动run1，机械臂处于让清扫机跟踪状态;

5. 绿色点为检测到的光伏组件阵列断点位置。

   ![](./solar_gap_detect_readme.assets/rviz_screenshot_2023_06_25-09_34_56.png)

   

   

   

   

