#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf

def char_to_float(c):
    return float(c)

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('control_node', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    end_effector_link = move_group.get_end_effector_link()
    current_pose = move_group.get_current_pose(end_effector_link).pose

    reference_frame = "world"
    move_group.set_pose_reference_frame(reference_frame)

    move_group.allow_replanning(True)
    move_group.set_goal_position_tolerance(0.001)
    move_group.set_goal_orientation_tolerance(0.01)
    move_group.set_max_acceleration_scaling_factor(0.2)
    move_group.set_max_velocity_scaling_factor(0.2)

    target_pose = geometry_msgs.msg.Pose()

    if len(sys.argv) > 1:
        if char_to_float(sys.argv[1]) == 1:
            move_group.set_named_target("home")
            move_group.go(wait=True)
            rospy.sleep(1)
            return

        # 获取当前位姿
        current_position = current_pose.position

        # 计算新的目标位姿
        target_pose.position.x = current_position.x + char_to_float(sys.argv[1])
        target_pose.position.y = current_position.y + char_to_float(sys.argv[2])
        target_pose.position.z = current_position.z + char_to_float(sys.argv[3])

        # 保持当前的姿态
        target_pose.orientation = current_pose.orientation
    else:
        rospy.logerr("No command line arguments provided. Please provide x, y, z coordinate increments.")
        return

    move_group.set_start_state_to_current_state()
    move_group.set_pose_target(target_pose)

    plan = move_group.plan()

    if plan[0]:
        rospy.loginfo("Plan (pose goal) succeeded")
        move_group.execute(plan[1], wait=True)
    else:
        rospy.loginfo("Plan (pose goal) failed")

    rospy.sleep(1)

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
