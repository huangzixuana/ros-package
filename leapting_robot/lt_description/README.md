# 1. URDF整理，二代安装机器人

2023-5-24

## 1.1. 文件修改

### 1.1.1 /robot_description定义

- ~/zs_vehicle_ws/src/kuka_kr210_support/launch/test_kr210l150 .launch: 内有load /robot_description定义
- ~/zs_vehicle_ws/src/kuka_eki_hw_interface/test/test_hardware_interface.launch: 内有load /robot_description定义
- ~/zs_vehicle_ws/src/kuka_moveit_config/launch/demo.launch: 内有load /robot_description定义

以上对/robot_description多次定义，易引起冲突和混乱，需整合在一处，目前只保留在test_hardware_interface.launch内。



### 1.1.2 零散的 static tf publisher

- ~/bringup/launch/kuka_moveit.launch : **tool0 -->camera_link** 
- ~/bringup/launch/localization_3d.launch: **base_footprint --> base_link**,  **base_link --> base_arm**

将零散的静态TF发布整合到urdf文件。



## 1.2. Robot Model

### 1.2.1 新建description包

└── lt_description
    ├── config
    ├── launch
    │   ├── load_hangcha.launch
    │   └── view_hangcha_model.launch
    ├── meshes
    │   ├── hangcha
    │   │   ├── back_left_link.STL
    │   │   ├── ...
    │   │   └── front_right_link.STL
    │   └── kr210r3100
    │       ├── base_arm.STL
    │       ├── ...
    │       └── link_6.STL
    ├── package.xml
    ├── README.md
    ├── rviz
    │   └── hangcha.rviz
    └── urdf
        ├── _d455.urdf.xacro（相机D455模块, 发布安装螺丝位置-->camera_link的TF）
        ├── hangcha_chassis.urdf.xacro （安装机器人底盘模块）
        ├── hangcha_installer.urdf.xacro（安装机器人整体集成）
        ├── kr210r3100.xacro（kuka 3100模块）
        ├── _rplidar.xacro（雷达demo模块）



![rviz_screenshot_2023_05_24-17_31_32](./README_img/rviz_screenshot_2023_05_24-17_31_32.png)



# 2 URDF整理，二代清扫机器人

## 2.1 文件修改

### 2.1.1 robot_description定义

- bringup/comm.launch

- arm_moveit_config/demo.launch

  以上对/robot_description多次定义，易引起冲突和混乱，需整合在一处，目前只保留在bringup/comm.launch内。

### 2.1.2 零散的static tf publisher

- bringup/frontec.launch: link2-->camera_link

- 将零散的静态TF发布整合到urdf文件。

  d455支持node/urdf发布TF,法兰泰克配置成URDF发布。



## 2.2 Robot Model

将机器人模型参数统一整合在bringup/urdf内, 通过cfg进行组装生成完成的模型（保留原始的零位参数）

![rviz_screenshot_2023_06_02-09_21_28](./README_img/rviz_screenshot_2023_06_02-09_21_28.png)

零位



![rviz_screenshot_2023_05_30-14_55_56](./README_img/rviz_screenshot_2023_05_30-14_55_56.png)

新的零位
