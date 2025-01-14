#!/usr/bin/env python

import rospy
import moveit_commander
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from moveit_commander.conversions import pose_to_list
import subprocess
import time

joint_velocities = None
failure_count = 0


def joint_states_callback(msg):
    global joint_velocities
    joint_velocities = msg.velocity

def move_arm_increment(move_group, x_increment, y_increment, z_increment):
    global failure_count
    end_effector_link = move_group.get_end_effector_link()
    current_pose = move_group.get_current_pose(end_effector_link).pose

    # 计算新的目标位姿
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = current_pose.position.x + x_increment
    target_pose.position.y = current_pose.position.y + y_increment
    target_pose.position.z = current_pose.position.z + z_increment

    # 保持当前的姿态
    target_pose.orientation = current_pose.orientation

    move_group.set_start_state_to_current_state()
    move_group.set_pose_target(target_pose)

    while True:
        plan = move_group.plan()
        if plan[0]:
            rospy.loginfo("Plan (pose goal) succeeded")
            move_group.execute(plan[1], wait=True)
            failure_count = 0  # 重置失败计数器
            break
        else:
            rospy.loginfo("Plan (pose goal) failed")
            failure_count += 1
            if failure_count >= 3:
                rospy.loginfo("Planning failed 3 times, returning to home position and restarting the program.")
                move_group.set_named_target("home")
                move_group.go(wait=True)
                subprocess.call(["rosrun", "mycobot_moveit_py_crtl", "mycobot_moveit_py_search.py"])
                rospy.signal_shutdown("Restarting the program")
                return
        
    while True:
        current_pose = move_group.get_current_pose(end_effector_link).pose
        if (abs(current_pose.position.x - target_pose.position.x) <= 0.01 and
            abs(current_pose.position.y - target_pose.position.y) <= 0.01 and
            abs(current_pose.position.z - target_pose.position.z) <= 0.01):
            break
        else:
            rospy.sleep(0.5)
            rospy.loginfo("waiting_1")

    rospy.sleep(1)
  




def main():
    moveit_commander.roscpp_initialize([])
    rospy.init_node('control_node', anonymous=True)

    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    reference_frame = "world"
    move_group.set_pose_reference_frame(reference_frame)

    move_group.allow_replanning(True)
    move_group.set_goal_position_tolerance(0.001)
    move_group.set_goal_orientation_tolerance(0.01)
    move_group.set_max_acceleration_scaling_factor(0.1)
    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_planning_time(5.0)  # 设置规划时间为5秒
    move_group.set_num_planning_attempts(10)  # 设置规划尝试次数为10次

    # 订阅 /joint_states 主题
    rospy.Subscriber('/joint_states', JointState, joint_states_callback)

    move_group.set_named_target("home")
    move_group.go(wait=True)
    
    rospy.sleep(1)

    while not rospy.is_shutdown():
        x_increment = 0  # 示例增量
        y_increment = 0  # 示例增量
        z_increment = -0.03 # 示例增量

        move_arm_increment(move_group, x_increment, y_increment, z_increment)

        x_increment = 0  # 示例增量
        y_increment = 0.05  # 示例增量
        z_increment = 0 # 示例增量

        move_arm_increment(move_group, x_increment, y_increment, z_increment)

        x_increment = 0  # 示例增量
        y_increment = 0  # 示例增量
        z_increment = -0.15 # 示例增量

        move_arm_increment(move_group, x_increment, y_increment, z_increment)

        x_increment = 0  # 示例增量
        y_increment = -0.1  # 示例增量
        z_increment = 0 # 示例增量

        move_arm_increment(move_group, x_increment, y_increment, z_increment)

        x_increment = 0  # 示例增量
        y_increment = 0  # 示例增量
        z_increment = 0.15 # 示例增量

        move_arm_increment(move_group, x_increment, y_increment, z_increment)

        x_increment = 0  # 示例增量
        y_increment = 0.05  # 示例增量
        z_increment = 0 # 示例增量

        move_arm_increment(move_group, x_increment, y_increment, z_increment)

        x_increment = 0  # 示例增量
        y_increment = 0  # 示例增量
        z_increment = 0.03 # 示例增量

        move_arm_increment(move_group, x_increment, y_increment, z_increment)

        move_group.set_named_target("home")
        move_group.go(wait=True)
    
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
