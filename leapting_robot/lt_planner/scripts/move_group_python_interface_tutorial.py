#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import geometry_msgs.msg


class PvmManager(object):

    def __init__(self, test="attach",
                 pvm_size=[2.3, 1.3, 0.035],
                 frame_id="tool0",
                 position_z=0):
        super(PvmManager, self).__init__()
        self.size = pvm_size
        self.frame_id=frame_id
        self.z = position_z
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "leapting_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        eef_link = move_group.get_end_effector_link()
        self.pvm_name = ""
        self.robot = robot
        self.scene = scene
        self.eef_link = eef_link

    def attach_pvm(self):
        pvm_name = self.pvm_name
        scene = self.scene
        size = self.size
        frame_id=self.frame_id
        position_z = self.z    
        pvm_pose = geometry_msgs.msg.PoseStamped()
        pvm_pose.header.frame_id = frame_id
        pvm_pose.pose.position.z = position_z 
        pvm_name = "pvm"
        scene.add_box(pvm_name, pvm_pose, size=(size[0],size[1], size[2]))
        self.pvm_name = pvm_name

        robot = self.robot
        eef_link = self.eef_link
        grasping_group = "leapting_arm"
        scene.attach_box(eef_link, pvm_name, touch_links=[])


    def detach_pvm(self):
        pvm_name = self.pvm_name
        scene = self.scene
        eef_link = self.eef_link
        scene.remove_attached_object(eef_link, name=pvm_name)
        
        scene.remove_world_object(pvm_name)

def main():
    try:
        tutorial = PvmManager()
        input("============ Press `Enter` to attach a pvm to the robot ...")
        tutorial.attach_pvm()
        input("============ Press `Enter` to detach the pvm from the robot ...")
        tutorial.detach_pvm()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()



