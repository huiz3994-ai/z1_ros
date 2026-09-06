#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Quaternion.h>  // 使用 tf2 的四元数运算库（需安装 libtf2 和依赖）
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // 提供 tf2::toMsg 的转换
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <signal.h>

/*********************************************
 *   ********  *******    ****    ********
 *      **     **        **   *      **
 *      **     *******     ***       **
 *      **     **        *  ***      **
 *      **     *******    ***        **
 * *******************************************/
/*补充说明：
0.本代码基于 test3_real_time_ik_solver.cpp，实现Touch + 视觉伺服共享控制，Touch 主导控制的同时加入视觉伺服辅助控制，追踪 aruco 标记——眼在手上。

1.目前实现：
————————————————————————————————————————————————————————————————————————————
     控制源     | 控制指令占比(pose.position) | 控制指令占比(pose.orientation) 
---------------|---------------------------|-------------------------------
     Touch     |              0%           |            100%              
 aruco_ros(软件)|            100%           |             0%               
      合计      |            100%           |            100%  
————————————————————————————————————————————————————————————————————————————
数据流：aruco_ros(processed_pose2 --> processed_aruco_marker_pose) 
        >> Touch(processed_pose --> target_pose_msg) 
          >> 机械臂

最后编辑于 2025.5.15 张耀华
*/

/*头文件解释：
move_group_interface.h：提供了与MoveIt!中的“MoveGroup”接口进行交互的功能，用于实现路径规划和执行。
planning_scene_interface.h：用于操作和管理规划场景。
moveit_msgs/DisplayRobotState.h、moveit_msgs/DisplayTrajectory.h等：用于显示机器人状态和轨迹的消息类型。
moveit_visual_tools.h：提供了可视化工具接口，帮助在Rviz中展示机器人操作。
*/

// 全局变量存储目标位姿和接收标志
geometry_msgs::PoseStamped target_pose_msg;
bool new_target_pose_received = false;
// bool FORWARD = false;  // 默认使用姿态映射
bool Anchor_updated = false;
bool back_to_home = false;
volatile sig_atomic_t exit_flag = 0;  // 新增退出标志
double K_touch_pos = 0.1; // Touch 末端位置映射缩放系数 (最大化空间利用率建议取 0.6，正常灵敏度 0.2，稳定慢速灵敏度 0.05)
double K_touch_ori = 1;   // Touch 末端姿态映射缩放系数，由于是绝对映射，目前无法缩放，直接修改会导致不可解
double K_aruco_pos = 1; 
double K_aruco_ori = 1; 
float x_Anchor_arm_received = 0.25;
float y_Anchor_arm_received = 0;
float z_Anchor_arm_received = 0.43;
float delta_x_received = 0;
float delta_y_received = 0;
float delta_z_received = 0;
int last_grey_state = 0;
int last_white_state = 0;
// ==== 平滑控制变量 ====
geometry_msgs::PoseStamped current_pose;       // 当前实际发布的位姿
const double SMOOTH_FACTOR_POS = 0.02;          // 位置平滑系数 
const double SMOOTH_FACTOR_ORI = 0.01;          // 姿态平滑系数 
bool first_pose_received = false;              // 是否收到初始位姿
// ==== aruco marker 位姿处理 ====
geometry_msgs::PoseStamped processed_aruco_marker_pose; //处理后的 aruco marker 位姿数据，可以直接发送给机械臂末端控制

void sigintHandler(int sig) {
    exit_flag = 1;  // 设置退出标志
    ROS_WARN_STREAM("[SHUT DOWN] Captured exit signal, preparing...");
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {//Touch 映射回调函数
    geometry_msgs::PoseStamped processed_pose = *msg;

    // ========== 位置映射部分 ========== 
    //坐标轴变换
    /******************
    z1 机械臂     Touch
        x   ————  -z
        y   ————  -x
        z   ————   y
    *******************/
    /******[Touch]根据机械臂当前位置进行相对位移控制，比例缩放同上（以末端执行器实时位置为锚点，动态映射）******/
/***
    processed_pose.pose.position.x = x_Anchor_arm_received;//启动时 0.25;
    processed_pose.pose.position.y = y_Anchor_arm_received;//启动时 0;
    processed_pose.pose.position.z = z_Anchor_arm_received;//启动时 0.43;
    processed_pose.pose.position.x += K_touch_pos*(-delta_z_received)/100; 
    processed_pose.pose.position.y += K_touch_pos*(-delta_x_received)/100;
    processed_pose.pose.position.z += K_touch_pos*(delta_y_received)/100; 
***/
    /******[Aruco markers]根据机械臂link00绝对空间映射******/
    processed_pose.pose.position.x = K_aruco_pos*processed_aruco_marker_pose.pose.position.x; 
    processed_pose.pose.position.y = K_aruco_pos*processed_aruco_marker_pose.pose.position.y;
    processed_pose.pose.position.z = K_aruco_pos*processed_aruco_marker_pose.pose.position.z; 
    // ========== 姿态映射部分 ==========
    // 使用真实姿态映射
    /*坐标轴转换规则：
        Touch的x → 机械臂的-z
        Touch的y → 机械臂的-x
        Touch的z → 机械臂的+y
    */
    static geometry_msgs::Quaternion last_orientation; // 保存松开时的姿态
    static bool first_run = true;
    if (first_run) { // 初始化默认朝前姿态
        last_orientation.w = 1.0;
        last_orientation.x = last_orientation.y = last_orientation.z = 0.0;
        first_run = false;
    }

    if (last_white_state) { // 按键按下时更新姿态
        processed_pose.pose.orientation.x = K_touch_ori*(-msg->pose.orientation.z);
        processed_pose.pose.orientation.y = K_touch_ori*(-msg->pose.orientation.x);
        processed_pose.pose.orientation.z = K_touch_ori*(msg->pose.orientation.y);
        processed_pose.pose.orientation.w = K_touch_ori*(msg->pose.orientation.w);
        last_orientation = processed_pose.pose.orientation; // 持续记录最新姿态
    } else { // 松开时使用最后一次记录的姿态
        processed_pose.pose.orientation = last_orientation;
    }

    if(back_to_home){//命令行 rostopic 发布“true”到布尔型话题 /back_to_home 即可实现机械臂复位并锁定机械臂，Touch 按下灰色按钮解锁
        processed_pose.pose.position.x = 0.25;
        processed_pose.pose.position.y = 0;
        processed_pose.pose.position.z = 0.43;
        last_orientation.w = 1.0;
        last_orientation.x = last_orientation.y = last_orientation.z = 0.0;
        //back_to_home = false;//解锁交由 Touch 端实现
    }
    //写入预处理后的位姿数据
    target_pose_msg = processed_pose;
    new_target_pose_received = true;
    
}

void poseFROMarucoCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {//aruco_marker 映射回调函数
        geometry_msgs::PoseStamped processed_pose2 = *msg;

    // ========== 位置映射部分 ========== 
    //坐标轴变换
    /******************
    z1 机械臂    aruco    Touch
        x   ————  z  ————  -z
        y   ———— -x  ————  -x
        z   ———— -y  ————   y
    *******************/
    /******根据机械臂link00绝对空间映射******/
    processed_pose2.pose.position.x = (msg->pose.position.x-0.4)*1; 
    processed_pose2.pose.position.y = (msg->pose.position.y)*1;
    processed_pose2.pose.position.z = (msg->pose.position.z-0.055)*1; 

    // ========== 姿态映射部分 ==========
    // 使用真实姿态映射
    // Step1: 定义固定偏移四元数的逆 (x=-0.5, y=0.5, z=0.5, w=0.5)
    static const tf2::Quaternion q_offset_inv(-0.5, 0.5, 0.5, 0.5);

    // Step2: 提取标签原始四元数
    tf2::Quaternion q_label(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);

    // Step3: 应用四元数乘法抵消固定偏移
    tf2::Quaternion q_corrected = q_offset_inv * q_label;
    q_corrected.normalize();

    // Step4: 写入处理后的姿态
    processed_pose2.pose.orientation.x = q_corrected.z()*1;
    processed_pose2.pose.orientation.y = -q_corrected.x()*1;
    processed_pose2.pose.orientation.z = q_corrected.y()*1;
    processed_pose2.pose.orientation.w = q_corrected.w()*1;

    // processed_pose2.pose.orientation.x = 0;
    // processed_pose2.pose.orientation.y = 0;
    // processed_pose2.pose.orientation.z = 0;
    // processed_pose2.pose.orientation.w = 1;  

    //写入预处理后的位姿数据
    processed_aruco_marker_pose = processed_pose2;
    // target_pose_msg = processed_pose2;
    new_target_pose_received = true;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    // 检查按钮数组有效性
    if(msg->buttons.size() < 2) {
        ROS_WARN_ONCE("Invalid buttons message size!");
        return;
    }

    // 获取当前状态
    int current_grey = msg->buttons[0];
    int current_white = msg->buttons[1];

    // 灰色按钮状态变化检测
    if(current_grey != last_grey_state) {
        if(current_grey == 1) {
            ROS_INFO_STREAM("Gray button PRESSED");
            // 这里添加按下处理代码
        } else {
            ROS_INFO_STREAM("Gray button RELEASED");
        }
        last_grey_state = current_grey;
    }

    // 白色按钮状态变化检测
    if(current_white != last_white_state) {
        if(current_white == 1) {
            ROS_INFO_STREAM("White button PRESSED"); 
            // 这里添加按下处理代码
        } else {
            ROS_INFO_STREAM("White button RELEASED");
        }
        last_white_state = current_white;
    }
}
void boolCallback(const std_msgs::Bool::ConstPtr& msg) {
    Anchor_updated = msg->data;
    //ROS_INFO_STREAM("Received: %s", msg->data ? "true" : "false");
}
void BackToHomeboolCallback(const std_msgs::Bool::ConstPtr& msg) {
    back_to_home = msg->data;
    ROS_INFO_STREAM("[HOME]Received back to home cmd!");
}
void BackToZeroboolCallback(const std_msgs::Bool::ConstPtr& msg) {
    if(msg->data) exit_flag = 1;
    ROS_INFO_STREAM("[SHUT DOWN] Received back to zero cmd!");
}
void poseAnchorCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){// 锚点回调函数(接收话题/end_effector_pose，也就是机械臂自身位姿数据，而非 Touch)
    if (Anchor_updated)
    {
        x_Anchor_arm_received = msg->pose.position.x;
        y_Anchor_arm_received = msg->pose.position.y;
        z_Anchor_arm_received = msg->pose.position.z;
        //ROS_INFO_STREAM("x_Anchor_arm_received:"<<x_Anchor_arm_received<<", y_Anchor_arm_received:"<<y_Anchor_arm_received<<", z_Anchor_arm_received"<<z_Anchor_arm_received);
    }
    Anchor_updated = false;
}
void poseIncrementCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    delta_x_received = msg->data[0];
    delta_y_received = msg->data[1];
    delta_z_received = msg->data[2];
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "real_time_ik_solver");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // ==== 注册退出信号处理，同时捕获SIGINT（Ctrl+C）和SIGTERM（kill命令）====
  signal(SIGINT, sigintHandler);
  signal(SIGTERM, sigintHandler); // 对 rosnode kill 不起作用，对系统级 kill 有效？

  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  const std::string end_effector_link = move_group.getEndEffectorLink();
  const moveit::core::RobotModelConstPtr &kinematic_model = move_group.getRobotModel();
  const moveit::core::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("manipulator");

  // 逆运动学参数
  const double timeout = 0.1;
  const int attempts = 10;
  double solve_time = 0.0;
  ros::Rate rate(300);

  // 初始化ROS节点
  ros::NodeHandle nh("~");
  ros::Subscriber pose_sub = nh.subscribe("/Geomagic/pose", 10, poseCallback);
  ros::Subscriber pose_from_aruco_sub = nh.subscribe("/aruco_single/pose", 10, poseFROMarucoCallback);
  ros::Subscriber sub = nh.subscribe("/Touch_Anchor_updated", 1, boolCallback);
  ros::Subscriber back_to_home_sub = nh.subscribe("/back_to_home_arm", 1, BackToHomeboolCallback);
  ros::Subscriber back_to_zero_sub = nh.subscribe("/back_to_zero_arm", 1, BackToZeroboolCallback);
  ros::Subscriber pose_Anchor_sub = nh.subscribe("/end_effector_pose", 1, poseAnchorCallback);
  ros::Subscriber increment_sub = nh.subscribe("/Touch_Pose_increment", 10, poseIncrementCallback);
  ros::Subscriber sub_Touch_button = nh.subscribe("/Geomagic/joy", 1, joyCallback);
  ros::Publisher joint_pub = nh.advertise<std_msgs::Float64MultiArray>("/goal_joint_positions", 10);    // 新增发布器
  ros::Publisher target_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ttttttttarget_pose", 10); // 新增目标位姿发布器

  while (ros::ok() && !exit_flag)
  {
    if (new_target_pose_received)
    {
      // ==== 姿态平滑处理 ====
      if (!first_pose_received)
      {
        current_pose = target_pose_msg;
        first_pose_received = true;
      }

      target_pose_pub.publish(target_pose_msg); // 发布目标位姿

      // 线性插值处理位置
      current_pose.pose.position.x += SMOOTH_FACTOR_POS *
                                      (target_pose_msg.pose.position.x - current_pose.pose.position.x);
      current_pose.pose.position.y += SMOOTH_FACTOR_POS *
                                      (target_pose_msg.pose.position.y - current_pose.pose.position.y);
      current_pose.pose.position.z += SMOOTH_FACTOR_POS *
                                      (target_pose_msg.pose.position.z - current_pose.pose.position.z);
      // current_pose.pose.position = target_pose_msg.pose.position;  // 完全跳过滤波

      // 球面插值处理姿态（使用tf2）
      tf2::Quaternion q_current, q_target, q_interp;
      tf2::fromMsg(current_pose.pose.orientation, q_current);
      tf2::fromMsg(target_pose_msg.pose.orientation, q_target);
      q_interp = tf2::slerp(q_current, q_target, SMOOTH_FACTOR_ORI);
      q_interp.normalize();
      current_pose.pose.orientation = tf2::toMsg(q_interp);

      // ==== 逆运动学解算 ====
      Eigen::Isometry3d target_pose;
      tf2::fromMsg(current_pose.pose, target_pose); // 使用平滑后的位姿

      moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
      moveit::core::RobotState robot_state(*current_state);

      ros::Time start_time = ros::Time::now();

      bool ik_solved = robot_state.setFromIK(
          joint_model_group,
          target_pose,
          end_effector_link,
          attempts,
          timeout);

      if (ik_solved)
      {
        std::vector<double> joint_values;
        robot_state.copyJointGroupPositions(joint_model_group, joint_values);

        // ==== 关节空间二次平滑 ====
        static std::vector<double> last_joint_values(joint_values.size(), 0.0);
        for (size_t i = 0; i < joint_values.size(); ++i)
        {
          joint_values[i] = last_joint_values[i] + 0.5 *
                                                       (joint_values[i] - last_joint_values[i]);
        }
        last_joint_values = joint_values;

        // 发布关节位置
        std_msgs::Float64MultiArray joint_positions_msg;
        joint_positions_msg.data = joint_values;
        joint_pub.publish(joint_positions_msg);

        ros::Time end_time = ros::Time::now();
        double time_diff = (end_time - start_time).toSec();
        if (time_diff > 0.0003)
        {
          solve_time = time_diff;
        }
        ROS_INFO_STREAM("--------------Time for solving: " << solve_time << " seconds--------------");
        ROS_INFO_STREAM("[SUCCEED]IK Solution Found:");
        for (size_t i = 0; i < joint_values.size(); ++i)
        {
          ROS_INFO_STREAM("  Joint " << i + 1 << ": "
                                     << joint_values[i] << " radians ("
                                     << joint_values[i] * 180.0 / M_PI << " degrees)");
        }
      }
      else
      {
        ROS_WARN_STREAM("[WARNING]IK solve failed!");
      }
      new_target_pose_received = false;
    }
    ros::spinOnce();
    rate.sleep();
  }
  // ==== 退出时归零处理 ====
  if (exit_flag)
  {
    ROS_WARN_STREAM("[SHUT DOWN] Sending back to zero cmd...");

    std_msgs::Float64MultiArray joint_positions_msg;
    joint_positions_msg.data = {0.0, 0.0, -0.005, -0.074, 0.0, 0.0};

    // 发送3次确保接收
    for (int i = 0; i < 3; ++i)
    {
      joint_pub.publish(joint_positions_msg);
      ros::spinOnce();
      ros::Duration(0.05).sleep();
    }

    ROS_WARN_STREAM("[SHUT DOWN] Cmd sent successfully, shutting down the node...");
    ros::Duration(0.5).sleep(); // 等待消息发出
  }

  return 0;
}