#include <ros/ros.h>
#include <sstream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h> //这个库原本居然没有配，害得我还得额外下载

float charToFloat(const char* c);


int main(int argc, char *argv[]){

  ros::init(argc, argv, "control_node");
  //创建一个异步spinner，用于在后台启动一个线程进行消息处理

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // 创建一个MoveGroupInterface对象，用于与机器人进行交互
  moveit::planning_interface::MoveGroupInterface arm("arm");


  // 获取终端link的名称
  std::string end_effector_link = arm.getEndEffectorLink();
  robot_state::RobotStatePtr current_state = arm.getCurrentState();
  Eigen::Affine3d transform = current_state->getGlobalLinkTransform(end_effector_link);

  // Eigen::Vector3d position = transform.translation();  
  // double px = position.x();  
  // double py = position.y();  
  // double pz = position.z();

  // std::cout << "position: "  
  //           << px << ", "  
  //           << py << ", "  
  //           << pz << std::endl;  

  Eigen::Matrix3d rotation_matrix = transform.rotation();  
  Eigen::Quaterniond orientation(rotation_matrix);  
  double ox = orientation.x();  
  double oy = orientation.y();  
  double oz = orientation.z();
  double ow = orientation.w();

  // std::cout << "Orientation: "  
  //           << ox << ", "  
  //           << oy << ", "  
  //           << oz << ", "  
  //           << ow << std::endl;  

  //设置目标位置所使用的参考坐标系
  std::string reference_frame = "world";
  arm.setPoseReferenceFrame(reference_frame);

  // 当运动规划失败后，允许重新规划
  arm.allowReplanning(true);

  // 设置位置(单位：米)和姿态（单位：弧度）的允许误差
  arm.setGoalPositionTolerance(0.001);
  arm.setGoalOrientationTolerance(0.01);

  // 设置允许的最大速度和加速度
  arm.setMaxAccelerationScalingFactor(0.2);
  arm.setMaxVelocityScalingFactor(0.2);

  // 设置机器人终端的目标位置
  geometry_msgs::Pose target_pose;

  if(charToFloat(argv[1])==1){
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);
    return 0;
  }

  // target_pose.orientation.x = ox;
  // target_pose.orientation.y = oy;
  // target_pose.orientation.z = oz;
  // target_pose.orientation.w = ow;

  target_pose.position.x = charToFloat(argv[1]); //x=0 y=0.137 z=0.523
  target_pose.position.y = charToFloat(argv[2]);
  target_pose.position.z = charToFloat(argv[3]);

  // 设置机器臂当前的状态作为运动初始状态
  arm.setStartStateToCurrentState();
  //arm.setPoseTarget(target_pose);
  arm.setApproximateJointValueTarget(target_pose);

  // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::core::MoveItErrorCode success = arm.plan(plan);

  // 打印运动规划结果信息
  ROS_INFO("Plan (pose goal) %s",success?"":"FAILED");   

  // 让机械臂按照规划的轨迹开始运动。
  if(success)
    arm.execute(plan);
  sleep(1);

  // 关闭ROS节点，释放资源
  ros::shutdown(); 

	return 0;
}



float charToFloat(const char* c) {
    std::stringstream ss;
    ss << c;
    float f;
    ss >> f;
    return f;
}
