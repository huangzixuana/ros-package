# Cartesian limit of position

new user interface:
![image-20240108111435874](./arm_limit_img/image-20240108111438837.png)



input dictionary format:

- key: *x_max*, *y_max*, *z_max*, *x_min*, *y_min*, *z_min* (set key in need ), 
- value: float
- format default:  {}



## update2024-02-06:

测试起点和behavior
![image-20240206112445911](./arm_limit_img/image-20240206112445911.png)

![image-20240206112541169](./arm_limit_img/image-20240206112541169.png)

插值之前：

![image-20240206112607364](./arm_limit_img/image-20240206112607364.png)

![image-20240206112626761](./arm_limit_img/image-20240206112626761.png)

插值后：

![image-20240206112721356](./arm_limit_img/image-20240206112721356.png)

![image-20240206112746925](./arm_limit_img/image-20240206112746925.png)



对比：

可减少因为直线斜率变化引起的剐蹭。
