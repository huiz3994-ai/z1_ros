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
0.本代码基于 test3_real_time_ik_solver.cpp，实现纯视觉伺服控制，追踪 aruco 标记——眼在手外，通过标定板实现视觉遥操作。

1.机器人运动学求解器配置在z1_moveit_config/config/kinematics.yaml

2.z1机械臂的 Touch 相对位置映射遥操作 IK 解算逻辑：

    S1：用户控制 Touch 发布锚点更新信号到话题/Touch_Anchor_updated 
            ——>  本程序响应该信号，并读取机械臂自身的当前末端位置作为锚点；

    S2：用户控制 Touch 发布相对锚点的位移量到话题/Touch_Pose_increment 
            ——> 本程序接收该位移量并映射为合适的值 
                ——> 本程序开始 IK 解算，将解算结果发布到话题/goal_joint_positions；

    S3：z1_controller （z1_ros/z1_hw/src/high_freq_control.cpp）自动接收解算结果并驱动 z1 机械臂，回到 S1 并高频率刷新。

Tips：机械臂并不是执行轨迹规划，而是步进跟踪给定的目标位置，所以本方案可实现低延迟、实时遥操作。

3.按下 Touch 灰色按键：控制位置；按下白色按键：控制姿态。

4.添加了插值处理来平滑输入指令，效果不错，如有必要可以尝试移除二次平滑。

5.添加复位锁定与 Touch 解锁功能：命令行 rostopic 发布“true”到布尔型话题 /back_to_home 即可实现机械臂复位并锁定机械臂，Touch 按下灰色按钮解锁。

6.添加机械臂归零功能，通过 ctrl+c 退出本程序即可自动归零（可用脚本控制），或者向话题 /back_to_zero 发布“true”或者“false”即可。

7.已经固定下稳定版本v1.65。

最后编辑于 2025.4.29 张耀华
-------------------以下为新内容---------------------
1.把 Touch 控制改为眼在手外视觉伺服控制（back to home 功能失效，已删除，其他残留代码尚未完全清理），模拟效果不错，四元数转换部分可以考虑优化。

最后编辑于 2025.4.29 张耀华
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
volatile sig_atomic_t exit_flag = 0;  // 新增退出标志
double K_pos = 1; // 末端位置映射缩放系数 
double K_ori = 2;   // Touch 末端姿态映射缩放系数
float x_default = 0.25;
float y_default = 0;
float z_default = 0.43;
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

void sigintHandler(int sig) {
    exit_flag = 1;  // 设置退出标志
    ROS_WARN_STREAM("[SHUT DOWN] Captured exit signal, preparing...");
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {//Touch 映射回调函数
    geometry_msgs::PoseStamped processed_pose = *msg;

    // ========== 位置映射部分 ========== 
    //坐标轴变换
    /******************
    z1 机械臂    aruco    Touch
        x   ————  z  ————  -z
        y   ———— -x  ————  -x
        z   ———— -y  ————   y
    *******************/
    /******根据机械臂当前位置进行相对位移控制，比例缩放同上（以末端执行器默认工作位置为锚点，动态映射）******/
    processed_pose.pose.position.x = x_default;//启动时 0.25;
    processed_pose.pose.position.y = y_default;//启动时 0;
    processed_pose.pose.position.z = z_default;//启动时 0.43;
    // processed_pose.pose.position.x += K_pos*(-delta_z_received)/100; 
    // processed_pose.pose.position.y += K_pos*(-delta_x_received)/100;
    // processed_pose.pose.position.z += K_pos*(delta_y_received)/100; 
    processed_pose.pose.position.x += K_pos*(msg->pose.position.z-0.3); 
    processed_pose.pose.position.y += K_pos*(-msg->pose.position.x);
    processed_pose.pose.position.z += K_pos*(-msg->pose.position.y); 

    // ========== 姿态映射部分 ==========
    // 使用真实姿态映射
    // Step1: 提取四元数并转换为旋转矩阵
    tf2::Quaternion q_orig(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);
    tf2::Matrix3x3 R_orig(q_orig);

    // Step2: 应用Z轴反转修正矩阵（绕X轴180度）
    tf2::Matrix3x3 R_flip(
        1,  0,  0,
        0, -1,  0,
        0,  0, -1);
    tf2::Matrix3x3 R_corrected = R_orig * R_flip;

    // Step3: 转换回四元数
    tf2::Quaternion q_corrected;
    R_corrected.getRotation(q_corrected);
    q_corrected.normalize();

    // Step4: 写入处理后的姿态
    processed_pose.pose.orientation.x = q_corrected.z()*K_ori;
    processed_pose.pose.orientation.y = -q_corrected.x()*K_ori;
    processed_pose.pose.orientation.z = -q_corrected.y()*K_ori;
    processed_pose.pose.orientation.w = q_corrected.w()*K_ori;

    // // 若K_ori影响模长，需重新归一化
    // if (std::abs(K_ori - 1.0) > 1e-6)
    // {
    //     q_corrected.normalize();
    //     processed_pose.pose.orientation.x = q_corrected.z();
    //     processed_pose.pose.orientation.y = -q_corrected.x();
    //     processed_pose.pose.orientation.z = -q_corrected.y();
    //     processed_pose.pose.orientation.w = q_corrected.w();
    // }

    //写入预处理后的位姿数据
    target_pose_msg = processed_pose;
    new_target_pose_received = true;
}

void BackToZeroboolCallback(const std_msgs::Bool::ConstPtr& msg) {
    if(msg->data) exit_flag = 1;
    ROS_INFO_STREAM("[SHUT DOWN] Received back to zero cmd!");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "visial_servo_solver");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // ==== 注册退出信号处理，同时捕获SIGINT（Ctrl+C）和SIGTERM（kill命令）====
    signal(SIGINT, sigintHandler);
    signal(SIGTERM, sigintHandler);//对 rosnode kill 不起作用，对系统级 kill 有效？

    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    const std::string end_effector_link = move_group.getEndEffectorLink();
    const moveit::core::RobotModelConstPtr& kinematic_model = move_group.getRobotModel();
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");

    // 逆运动学参数
    const double timeout = 0.1;
    const int attempts = 10;
    double solve_time = 0.0;
    ros::Rate rate(300);

    // 初始化ROS节点
    ros::NodeHandle nh("~");
    ros::Subscriber pose_sub = nh.subscribe("/aruco_single/pose", 10, poseCallback);
    ros::Subscriber back_to_zero_sub = nh.subscribe("/back_to_zero_arm", 1, BackToZeroboolCallback);
    ros::Publisher joint_pub = nh.advertise<std_msgs::Float64MultiArray>("/goal_joint_positions", 10);  // 新增发布器


    while (ros::ok() && !exit_flag) {        
        if (new_target_pose_received) {
            // ==== 姿态平滑处理 ====
            if (!first_pose_received) {
                current_pose = target_pose_msg;
                first_pose_received = true;
            }
            
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
            tf2::fromMsg(current_pose.pose, target_pose);  // 使用平滑后的位姿

            moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
            moveit::core::RobotState robot_state(*current_state);

            ros::Time start_time = ros::Time::now();

            bool ik_solved = robot_state.setFromIK(
                joint_model_group,
                target_pose,
                end_effector_link,
                attempts,
                timeout
            );

            if (ik_solved) {
                std::vector<double> joint_values;
                robot_state.copyJointGroupPositions(joint_model_group, joint_values);

                // ==== 关节空间二次平滑 ====
                static std::vector<double> last_joint_values(joint_values.size(), 0.0);
                for (size_t i = 0; i < joint_values.size(); ++i) {
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
                if (time_diff > 0.0003) {
                    solve_time = time_diff;
                }
                ROS_INFO_STREAM("--------------Time for solving: " << solve_time << " seconds--------------");
                ROS_INFO_STREAM("[SUCCEED]IK Solution Found:");
                for (size_t i = 0; i < joint_values.size(); ++i) {
                    ROS_INFO_STREAM("  Joint " << i + 1 << ": " 
                        << joint_values[i] << " radians ("
                        << joint_values[i] * 180.0 / M_PI << " degrees)");
                }
            } else {
                ROS_WARN_STREAM("[WARNING]IK solve failed!");
            }
            new_target_pose_received = false;
        }
        ros::spinOnce();
        rate.sleep();
    }
    // ==== 退出时归零处理 ====
    if (exit_flag) {
        ROS_WARN_STREAM("[SHUT DOWN] Sending back to zero cmd...");
        
        std_msgs::Float64MultiArray joint_positions_msg;
        joint_positions_msg.data = {0.0, 0.0, -0.005, -0.074, 0.0, 0.0};
        
        // 发送3次确保接收
        for (int i = 0; i < 3; ++i) {
            joint_pub.publish(joint_positions_msg);
            ros::spinOnce();
            ros::Duration(0.05).sleep();
        }
        
        ROS_WARN_STREAM("[SHUT DOWN] Cmd sent successfully, shutting down the node...");
        ros::Duration(0.5).sleep();  // 等待消息发出
    }

    return 0;
}