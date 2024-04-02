设计的behavior一览

![image-20231130131013214](./bilateral_install_img/image-20231130131013214.png)





安装前的检测：

先检测负向（base arm Z轴右手系的正负），检测成功返回方向，失败则检测正向，检测成功返回方向，失败返回失败。

![image-20231130130934232](./bilateral_install_img/image-20231130130934232.png)



其它安装、拆卸流程与单侧一致，但可接收参数: Pos/Neg, 执行不同侧向任务