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

1.把 Touch 控制改为眼在手外视觉伺服控制（back to home 功能失效，已删除，其他残留代码尚未完全清理），模拟效果不错，四元数转换部分可以考虑优化。

2.本代码实现数字孪生的视觉伺服，追踪 aruco marker 。流程为：
    RGB 相机接收原始图像
        ——> aruco_ros 识别 aruco marker 相对机械臂基座 link00 的位姿（aruco_ros/aruco_ros/launch/realsense_single2.launch）
            ——> 在回调函数中将 aruco marker 位姿映射到机械臂末端，完成控制

3。本代码需要启用 rviz 来查看相关信息，例如 aruco_ros 识别结果画面、tf树。

最后编辑于 2025.5.14 张耀华
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
volatile sig_atomic_t exit_flag = 0;  // 新增退出标志
double K_pos = 1; // 末端位置映射缩放系数 
double K_ori = 1;   // aruco marker 末端姿态映射缩放系数
// ==== 平滑控制变量 ====
geometry_msgs::PoseStamped current_pose;       // 当前实际发布的位姿
const double SMOOTH_FACTOR_POS = 0.02;          // 位置平滑系数 
const double SMOOTH_FACTOR_ORI = 0.01;          // 姿态平滑系数 
bool first_pose_received = false;              // 是否收到初始位姿

void sigintHandler(int sig) {
    exit_flag = 1;  // 设置退出标志
    ROS_WARN_STREAM("[SHUT DOWN] Captured exit signal, preparing...");
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {//aruco_marker 映射回调函数
    geometry_msgs::PoseStamped processed_pose = *msg;

    // ========== 位置映射部分 ========== 
    //坐标轴变换
    /******************
    z1 机械臂    aruco    Touch
        x   ————  z  ————  -z
        y   ———— -x  ————  -x
        z   ———— -y  ————   y
    *******************/
    /******根据机械臂link00绝对空间映射******/
    processed_pose.pose.position.x = K_pos*(msg->pose.position.x-0.4); 
    processed_pose.pose.position.y = K_pos*(msg->pose.position.y);
    processed_pose.pose.position.z = K_pos*(msg->pose.position.z-0.055); 

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
    processed_pose.pose.orientation.x = q_corrected.z()*K_ori;//
    processed_pose.pose.orientation.y = -q_corrected.x()*K_ori;//-
    processed_pose.pose.orientation.z = q_corrected.y()*K_ori;//
    processed_pose.pose.orientation.w = q_corrected.w()*K_ori;//

    // processed_pose.pose.orientation.x = 0;
    // processed_pose.pose.orientation.y = 0;
    // processed_pose.pose.orientation.z = 0;
    // processed_pose.pose.orientation.w = 1;  

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