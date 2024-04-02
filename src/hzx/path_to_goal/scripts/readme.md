1.节点功能
将收到的path分解为一系列goal，并依次将这些goal通过话题发布。
2.节点实现方法
 接收base_laser坐标系下的path，并通过tf将其转换至map坐标系下的path
    rospy.Subscriber("path_topic", Path, 
                    lambda msg: self.local_plan_callback(msg, self.tf_buffer))
    def convert_pose(self, pose, tf_buffer)
    def local_plan_callback(self, msg, tf_buffer)
 计算path总长度，每隔1m取一个goal，并将这些goal存放至一个列表中
    def calculate_path_length(self, path):
    def extract_goals(self, transformed_plan, num_segments):
 发布第一个goal
 获取小车位置
    rospy.Subscriber('/odom', Odometry, self.callback)
    def callback(self, msg):
 对比小车位置与目前goal之间的距离，获得下一个要发送的goal
    def distance(self, point1, point2):
    def get_goal(self, current_position, goals):
 通过话题依次发送目标goal
    self.goal_publisher = rospy.Publisher(
            "move_base_simple/goal", PoseStamped, latch=True, queue_size=10)
    def publish_goal(self):