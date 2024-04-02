# 手眼标定半-自动化

## 系统环境

### x86

可直接运行

### arm

jeston运行有如下问题：https://github.com/opencv/opencv/issues/14884

解决方法:

在该程序执行的终端设置如下环境变量

```shell
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1
```
小kuka具体配置如下：

![image-20231210115253552](./handeye_action_img/image-20231210115253552.png)



## web socket接口

1. flexbe任务名称： HandEyeCalibration， 参数： if_auto_all（布尔类型）

   ![image-20231210120137617](./handeye_action_img/image-20231210120137617.png)

2. trig下一步（移动机械臂）： 发布 "UI_handeye_arm:next?" , 接收 next

![image-20231210115630227](./handeye_action_img/image-20231210115630227.png)



![image-20231210115730825](./handeye_action_img/image-20231210115730825.png)

3. trig下一步（采集数据）： 发布 "UI_handeye_take:next?" , 接收 next

![image-20231210115812891](./handeye_action_img/image-20231210115812891.png)



![image-20231210115826217](./handeye_action_img/image-20231210115826217.png)

## 标定点位

存储到相应的srdf文件

点位具有顺序性和命名规范，请遵循 calibration_x 格式，标定点位至少2个。

![image-20231101153413292](./handeye_action_img/image-20231101153413292.png)





## Flexbe标定程序 HandeyeAction

### parameter

- **client_action** :  'take'(take_sample)/ 'compute'(compute_calibration and save calibration yaml file)
- **algorithm**: 'Tsai-Lenz'/ 'Park'/ 'Horaud'/ 'Andreff'(default)/ 'Daniilidis'
- **if_write**: if write result to cfg only when client_action=compute

![image-20231207140948999](./handeye_action_img/image-20231207140948999.png)




## Flexbe 标定流程 HandEyeCalibration

### 逻辑设计
![logic](./handeye_action_img/logic.png)





遍历所有 calibration_x点位：

-    arm规划控制到calibration_x
-    等待用户确认
-   采集该标定数据

计算标定结果，并保存到文件



### Behavior

![image-20231207161841493](./handeye_action_img/image-20231207161841493.png)



### Demo

1. 标定点位数据输出示例：
   ![image-20231101154223009](./handeye_action_img/image-20231101154223009.png)



2. 标定计算和保存输出示例：
![image-20231207145615093](./handeye_action_img/image-20231207145615093.png)



3. 标定结果终端输出示例：
   ![image-20231101154431932](./handeye_action_img/image-20231101154431932.png)



4. 标定结果文件示例：
   ![image-20231101154502607](./handeye_action_img/image-20231101154502607.png)



5. cfg更新示例，注意**只增加/更新camera下面的xyz-rpy数据**，其它信息不做填充和修改。
![image-20231207145731449](./handeye_action_img/image-20231207145731449.png)



## 附录

### kr2700 21个标定点位一览

#### 1
![c1](./handeye_action_img/c1.png)



#### 2
![c2](./handeye_action_img/c2.png)



#### 3
![c3](./handeye_action_img/c3.png)



#### 4
![c4](./handeye_action_img/c4.png)



#### 5
![c5](./handeye_action_img/c5.png)



#### 6
![c6](./handeye_action_img/c6.png)



#### 7
![c7](./handeye_action_img/c7.png)




#### 8

![c8](./handeye_action_img/c8.png)




#### 9
![c9](./handeye_action_img/c9.png)



#### 10
![c10](./handeye_action_img/c10.png)



#### 11
![c11](./handeye_action_img/c11.png)



#### 12
![c12](./handeye_action_img/c12.png)



#### 13
![c13](./handeye_action_img/c13.png)




#### 14

![c14](./handeye_action_img/c14.png)



#### 15
![c15](./handeye_action_img/c15.png)



#### 16
![c16](./handeye_action_img/c16.png)




#### 17

![c17](./handeye_action_img/c17.png)





#### 18
![c18](./handeye_action_img/c18.png)



#### 19
![c19](./handeye_action_img/c19.png)



#### 20
![c20](./handeye_action_img/c20.png)



#### 21
![c21](./handeye_action_img/c21.png)




*自测启动备忘，请忽略：*

- *roslaunch lt_gazebo kuka2700_cover_bringup.launch*
- *roslaunch kr2700_moveit_config moveit_planning_execution.launch* 
- *rosrun tf2_ros static_transform_publisher 0.4 2.0 0 0 0 0 base_arm cleaner_bundle*
- *roslaunch easy_handeye calibrate2.launch*
- *roslaunch flexbe_app flexbe_full.launch* 
