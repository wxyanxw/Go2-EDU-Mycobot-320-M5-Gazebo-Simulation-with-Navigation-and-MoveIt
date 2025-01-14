#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import copy
from moveit_commander.conversions import pose_to_list

def print_waypoints(waypoints):
    for i, waypoint in enumerate(waypoints):
        rospy.loginfo(f"Waypoint {i+1}:")
        rospy.loginfo(f"  Position: x={waypoint.position.x}, y={waypoint.position.y}, z={waypoint.position.z}")
        rospy.loginfo(f"  Orientation: x={waypoint.orientation.x}, y={waypoint.orientation.y}, z={waypoint.orientation.z}, w={waypoint.orientation.w}")

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
    move_group.set_max_acceleration_scaling_factor(0.01)
    move_group.set_max_velocity_scaling_factor(0.01)

    # 定义轨迹点
    waypoints = []

    i = 0

    while i <= 10:
        current_pose.position.x -= 0
        current_pose.position.y -= 0
        current_pose.position.z -= 0.03
        current_pose.orientation = current_pose.orientation
        waypoints.append(copy.deepcopy(current_pose))
        i = i + 1

    # i = 0

    # while i < 10:
    #     current_pose.position.x -= 0
    #     current_pose.position.y -= 0
    #     current_pose.position.z += 0.03
    #     current_pose.orientation = current_pose.orientation
    #     waypoints.append(copy.deepcopy(current_pose))
    #     i = i + 1

    # 打印 waypoints 数据
    print_waypoints(waypoints)

    # 规划轨迹
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints,   # waypoints to follow
        0.003,        # eef_step
        0.0)         # jump_threshold

    if fraction > 0.9:
        rospy.loginfo("Plan (trajectory) succeeded")
        while True:
            move_group.execute(plan, wait=True)
            rospy.sleep(1) 
        move_group.execute(plan, wait=True)

    else:
        rospy.loginfo("Plan (trajectory) failed")
        return
    

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
