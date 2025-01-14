#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped
import time
import subprocess
import signal
import os
import moveit_commander
import geometry_msgs.msg
from sensor_msgs.msg import JointState

# 全局变量
joint_velocities = None
failure_count = 0

def terminate_process():
    # 查找mycobot_moveit_py_search.py的进程ID
    try:
        output = subprocess.check_output(["pgrep", "-f", "mycobot_moveit_py_search.py"])
        pids = output.decode().strip().split('\n')
        for pid in pids:
            if pid:
                os.kill(int(pid), signal.SIGTERM)
                print(f"Terminated process with PID {pid}")
    except subprocess.CalledProcessError:
        print("No process found with the name mycobot_moveit_py_search.py")

def custom_function(msg, move_group):
    # 从消息中提取xyz增量
    x_increment = msg.point.x
    y_increment = msg.point.y
    z_increment = msg.point.z
    
    # 打印xyz增量信息
    rospy.loginfo(f"Received xyz increments: x={x_increment}, y={y_increment}, z={z_increment}")
    
    # 调用move_arm_increment函数
    move_arm_increment(move_group, x_increment, y_increment, z_increment)
    
    # 条件判断
    if abs(x_increment) <= 0.01 and abs(y_increment) <= 0.01 and abs(z_increment) <= 0.01:
        print("Finsh!, exiting...")
        rospy.signal_shutdown("Condition met")

def listener():
    global default_pose
    rospy.init_node('terminate_node', anonymous=True)
    
    # 初始化moveit_commander
    moveit_commander.roscpp_initialize([])
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    # 订阅 /joint_states 主题
    rospy.Subscriber('/joint_states', JointState, joint_states_callback)
    
    while not rospy.is_shutdown():
        try:
            msg = rospy.wait_for_message('/red_object_center', PointStamped, timeout=None)
            terminate_process()  # 终止mycobot_moveit_py_search.py进程
            # 检测关节是否静止
            while True:
                if joint_velocities is not None:
                    if all(abs(vel) < 0.003 for vel in joint_velocities):
                        rospy.loginfo("All joints are static")
                        break
            rospy.sleep(1)            
            custom_function(msg, move_group)  # 执行自定义函数
        except rospy.ROSInterruptException:
            break

def joint_states_callback(msg):
    global joint_velocities
    joint_velocities = msg.velocity

def move_arm_increment(move_group, x_increment, y_increment, z_increment):
    global failure_count
    global default_pose
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

    rospy.sleep(1)

    # 检测关节是否静止
    while True:
        if joint_velocities is not None:
            if all(abs(vel) < 0.003 for vel in joint_velocities):
                rospy.loginfo("All joints are static")
                break
        rospy.sleep(0.5)

if __name__ == '__main__':
    listener()