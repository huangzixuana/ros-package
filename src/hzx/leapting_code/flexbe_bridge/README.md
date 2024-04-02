# flexbe_bridge

管理flexbe相关操作的节点。

## 1. ros接口

### 1.1 发布的话题

- /flexbe/behavior_list [std_msgs/String]

- /flexbe/site_collection [std_msgs/String]

### 1.2 订阅的话题  

/trig [std_msgs/Header]

### 1.3 参数

- behavior_pkg : flexbe轨迹包的名称，以冒号‘:’分隔。
- waypoints_path: flexbe_waypoints.yaml路径，支持相对路径和绝对路径.

### 1.4 launch示例

```xml
<launch>
    <node name="flexbe_bridge_node" pkg="rosbridge_system" type="flexbe_bridge.py" output="screen">
        <rosparam subst_value="true">
            waypoints_path: "~/catkin_ws/dbparam/flexbe_waypoints.yaml"
            behavior_pkg: "comm_behaviors:scheduling_flexbe_behaviors"
        </rosparam>
    </node>
</launch> 
```



## 2. behavior list相关
### 2.1 behavior list查看

轨迹按照名称字母升序排列

```shell
rostopic echo /flexbe/behavior_list
```



### 2.2 behavior 删除

删除flexbe 轨迹的指令操作：

```shell
rostopic pub /trig std_msgs/Header "seq: 0
stamp:
  secs: 0
  nsecs: 0
frame_id: 'flexbe_delete:turn_left'" 
```

删除成功后会有下述输出，且/flexbe/behavior_list会自动更新发布。

```shell
[INFO] [1678091177.499422]: flexbe behavior turn_left is removed
```

若删除不存在的轨迹，会有如下提示，不会引起异常。

```shell
flexbe behavior turn_lefftt not exists
```



## 3. flexbe site 相关
### 3.1 flexbe site查看

站点按照名称字母升序排列

```shell
rostopic echo /flexbe/site_collection
```
### 3.2 flexbe site新增

```shell
rostopic pub /trig std_msgs/Header "seq: 0
stamp:
  secs: 0
  nsecs: 0
frame_id: 'create_site:test_site23'" 
```

终端会有新增的输出提示

```shell
[INFO] [1678091735.972435]: flexbe site test_site23 is created
```

注意，为了避免不同定位方式的影响，新增采点会读取map到base_link的tf坐标变换，以该变换的位置和姿态作为当前新增点位的数据。

在一定时间内tf读取失败，则给一个默认零值。

若新增已经存在的站点，默认不会使用新数值覆盖，并有如下提示：

```shell
[WARN] [1678092681.525666]: test_site23 already exists in flexbe waypoint
```

新增站点后，/flexbe/site_collection会自动更新发布。

### 3.3 flexbe site删除

```shell
rostopic pub /trig std_msgs/Header "seq: 0
stamp:
  secs: 0
  nsecs: 0
frame_id: 'delete_site:test_site33'"
```

删除成功后，终端如下提示：

```shell
[INFO] [1678092275.319545]: test_site33 deleted
```

若删除不存在的站点，程序会忽略，不会引起异常。

删除后，/flexbe/site_collection会自动更新发布。



# reference


https://gitpython.readthedocs.io/en/stable/tutorial.html
