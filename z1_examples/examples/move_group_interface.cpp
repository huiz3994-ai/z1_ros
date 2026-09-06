#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

/*补充说明：
机器人运动学求解器配置在z1_moveit_config/config/kinematics.yaml
*/

/*头文件解释：
move_group_interface.h：提供了与MoveIt!中的“MoveGroup”接口进行交互的功能，用于实现路径规划和执行。
planning_scene_interface.h：用于操作和管理规划场景。
moveit_msgs/DisplayRobotState.h、moveit_msgs/DisplayTrajectory.h等：用于显示机器人状态和轨迹的消息类型。
moveit_visual_tools.h：提供了可视化工具接口，帮助在Rviz中展示机器人操作。
*/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface");//初始化ROS节点，节点名为move_group_interface。
  ros::NodeHandle nh;//创建一个ROS节点句柄，负责与ROS系统的通信。

  ros::AsyncSpinner spinner(1);//创建一个异步的ROS执行器，1表示一个线程用于处理ROS消息。
  spinner.start();//启动异步执行器。
 
  static const std::string PLANNING_GROUP = "manipulator";    //定义了一个字符串常量，表示规划组的名称（通常在机器人配置文件中定义）。这里的“manipulator”表示机械臂的规划组

  /*连接控制规划组（控制对象）*/
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  move_group_interface.setMaxVelocityScalingFactor(0.15);     //设置最大速度缩放因子为0.15，限制机械臂的速度
  move_group_interface.setMaxAccelerationScalingFactor(0.5);  //设置最大加速度缩放因子为0.5，限制机械臂的加速度

  //获取当前机械臂的状态，并通过getJointModelGroup方法获取指定规划组的关节模型。
  const moveit::core::JointModelGroup* jmg = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  /* Move to target position 设置目标位姿 */
  geometry_msgs::Pose target_pose1;    
  target_pose1.orientation.w = 0;
  target_pose1.position.x = 0.3;
  target_pose1.position.y = 0;
  target_pose1.position.z = 0.4;
  move_group_interface.setPoseTarget(target_pose1);   //告诉MoveIt!机械臂的目标位置

  /*使用MoveIt!规划一条(关节角度)运动轨迹，plan1用于存储规划的结果。move_group_interface.plan(plan1)会根据设置的目标位置进行路径规划，并返回执行状态（成功或失败）*/
  moveit::planning_interface::MoveGroupInterface::Plan plan1;
  bool success = (move_group_interface.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

  //如果路径规划成功，执行规划的路径，即让机械臂按照规划的路径移动
  if(success) {
    move_group_interface.execute(plan1);
  }

  /*这部分代码通过指定多个中间目标位置来生成一条笛卡尔路径（直线轨迹）*/
  /*补充说明：
  这里笛卡尔路径规划的原理是设定“路经点”，通过两点确定一条直线来控制机械臂走直线;
  这已经是第二段路径规划了，上文的部分是关节空间规划；
  rviz中add“robotmodel”再在links中可以勾选显示关节路径。
  */
  // Cartesinan Paths(笛卡尔路径)
  // ^^^^^^^^^^^^^^^^
  std::vector<geometry_msgs::Pose> waypoints;//创建路径点列表waypoints
  waypoints.push_back(target_pose1); //将目标位姿target_pose1加入到waypoints中
  
  geometry_msgs::Pose target_pose2 = target_pose1;  

  target_pose2.position.x += 0.1;     //在target_pose1的基础上，修改target_pose2的位置，使其X轴方向向前移动0.1单位
  waypoints.push_back(target_pose2);  // forward  （将新的目标位置加入路径(waypoints)中）

  target_pose2.position.z -= 0.1;     //继续修改target_pose2的位置，使其向沿Z轴方向上升0.1单位
  waypoints.push_back(target_pose2);  // up   （将修改后的目标位置加入路径(waypoints)中）

  /*通过waypoints生成机械臂的笛卡尔路径*/
  moveit_msgs::RobotTrajectory trajectory;  //trajectory对象存储计算得到的轨迹
  const double jump_threhold = 0.0;         //jump_threhold表示跳跃阈值（跳过求解失败的点）。设置为0.0，表示不允许机器人在路径计算时进行跳跃（例如突然跨越较大的距离）。
  const double eef_step = 0.01;             //eef_step表示每一步的最小步长,用于控制路径计算的精细度
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threhold, trajectory);//计算笛卡尔空间路径
  /*补充说明：.computeCartesianPath(aypoints, eef_step, jump_threhold, trajectory)是关键API
  其中各参数已在上两行介绍，未介绍的有：
  fraction：表示所规划路径的覆盖率（路径完全可达即为1（即100%），不完整可达（遇到障碍物）则小于1（取值0～1））；
  此方法还有更多参数，包括碰撞检测等等；
  笛卡尔空间下的圆弧路径规划：通过算三角函数，将圆弧分解为一段段直线进行插补。包括抛物线等等各类曲线轨迹规划均以此方式实现。
  */

  /*将生成的笛卡尔路径存储在plan2中并执行该路径，使机械臂沿着计算出的路径移动*/
  moveit::planning_interface::MoveGroupInterface::Plan plan2;

  plan2.trajectory_ = trajectory;
  move_group_interface.execute(plan2);


  /* Move to saved position （将机械臂移动到名为“home”的预设位置，通过setNamedTarget("home")可以选择一个事先定义好的目标位置）*/
  move_group_interface.setNamedTarget("home");
  moveit::planning_interface::MoveGroupInterface::Plan plan_home;
  success = (move_group_interface.plan(plan_home) == moveit::core::MoveItErrorCode::SUCCESS);

  //如果路径规划成功，执行回到“home”位置的路径
  if(success) {
    move_group_interface.execute(plan_home);
  }

  ros::shutdown();    //调用ros::shutdown()关闭ROS节点，程序结束

  return 0;
}