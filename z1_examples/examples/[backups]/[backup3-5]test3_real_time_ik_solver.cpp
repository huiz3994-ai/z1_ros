/*********说明*********
若需要使用固定姿态（朝前）启动程序（默认不启用），需要修改参数 FORWARD：
bash`
rosrun z1_examples test3_real_time_ik_solver _FORWARD:=true

若需要​在launch文件中配置：
xml`
<node name="real_time_ik_solver" pkg="your_package" type="your_node">
  <param name="FORWARD" value="true" />
</node>
*********************/
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
#include <std_msgs/Float64MultiArray.h>

/*********************************************
 *   ********  *******    ****    ********
 *      **     **        **   *      **
 *      **     *******     ***       **
 *      **     **        *  ***      **
 *      **     *******    ***        **
 * *******************************************/
/*补充说明：
机器人运动学求解器配置在z1_moveit_config/config/kinematics.yaml
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
// geometry_msgs::Point filtered_position;  // [效果不佳]全局变量保存滤波后的位置
// double alpha = 0.6;                      // [效果不佳]滤波系数(0<α<1）——> α→1：几乎不滤波（快速响应，但噪声敏感）；α→0：强平滑（延迟大，但抗噪性强）。
bool FORWARD = false;  // 默认使用姿态映射

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    geometry_msgs::PoseStamped processed_pose = *msg;

    // ========== 位置映射部分 ========== 
    //坐标轴变换
    /******************
    z1 机械臂     Touch
        x   ————  -z
        y   ————  -x
        z   ————   y
    *******************/
    processed_pose.pose.position.x = -msg->pose.position.z; 
    processed_pose.pose.position.y = -msg->pose.position.x;
    processed_pose.pose.position.z = msg->pose.position.y;
    //将 Touch 的控制信号缩小 100 倍
    processed_pose.pose.position.x /= 100.0;
    processed_pose.pose.position.y /= 100.0;
    processed_pose.pose.position.z /= 100.0;

    // [效果不佳]一阶低通滤波（主要用于平滑输入信号，消除传感器噪声或操作抖动带来的突变，确保机械臂运动更稳定）
    // filtered_position.x = alpha * processed_pose.pose.position.x + (1-alpha) * filtered_position.x;
    // filtered_position.y = alpha * processed_pose.pose.position.y + (1-alpha) * filtered_position.y;
    // filtered_position.z = alpha * processed_pose.pose.position.z + (1-alpha) * filtered_position.z;
    // processed_pose.pose.position = filtered_position;

    // ========== 姿态映射部分 ==========
    if (FORWARD) {
        // 使用固定姿态（示例：朝前）
        processed_pose.pose.orientation.w = 1.0;
        processed_pose.pose.orientation.x = 0.0;
        processed_pose.pose.orientation.y = 0.0;
        processed_pose.pose.orientation.z = 0.0;
    } else {
        // 使用真实姿态映射
        // 提取Touch的原始四元数
        double tx = msg->pose.orientation.x;
        double ty = msg->pose.orientation.y;
        double tz = msg->pose.orientation.z;
        double tw = msg->pose.orientation.w;

        /*坐标轴转换规则：
        Touch的x → 机械臂的-z
        Touch的y → 机械臂的-x
        Touch的z → 机械臂的+y
        对应四元数分量映射：*/
        processed_pose.pose.orientation.x = -tz;  // Touch.z → 机械臂.x
        processed_pose.pose.orientation.y = -tx;  // Touch.x → 机械臂.y
        processed_pose.pose.orientation.z = ty;   // Touch.y → 机械臂.z
        processed_pose.pose.orientation.w = tw;   // 保持四元数实部不变
    }

    //写入预处理后的位姿数据
    target_pose_msg = processed_pose;
    new_target_pose_received = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "real_time_ik_solver");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    const std::string end_effector_link = move_group.getEndEffectorLink();
    const moveit::core::RobotModelConstPtr& kinematic_model = move_group.getRobotModel();
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");

    // 逆运动学参数
    const double timeout = 0.1;
    const int attempts = 10;
    double solve_time = 0.0;
    ros::Rate rate(100);

    // 初始化ROS节点
    ros::NodeHandle nh("~");
    ros::Subscriber pose_sub = nh.subscribe("/Geomagic/pose", 10, poseCallback);
    ros::Publisher joint_pub = nh.advertise<std_msgs::Float64MultiArray>("/goal_joint_positions", 10);  // 新增发布器
    // 从参数服务器读取配置(姿态映射选择)
    nh.param<bool>("FORWARD", FORWARD, false);
    ROS_INFO_STREAM("Using " << (FORWARD ? "FIXED" : "MAPPED") 
                    << " orientation mode");

    while (ros::ok()) {
        if (new_target_pose_received) {
            Eigen::Isometry3d target_pose;
            tf2::fromMsg(target_pose_msg.pose, target_pose);

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

                // 创建并发布关节位置消息
                std_msgs::Float64MultiArray joint_positions_msg;
                joint_positions_msg.data = joint_values;
                joint_pub.publish(joint_positions_msg);  // 发布消息

                ros::Time end_time = ros::Time::now();
                double time_diff = (end_time - start_time).toSec();
                if (time_diff > 0.0003) {
                    solve_time = time_diff;
                }
                ROS_INFO_STREAM("--------------Time for solving: " << solve_time << " seconds--------------");
                
                ROS_INFO_STREAM("IK Solution Found:");
                for (size_t i = 0; i < joint_values.size(); ++i) {
                    ROS_INFO_STREAM("  Joint " << i + 1 << ": " 
                        << joint_values[i] << " radians ("
                        << joint_values[i] * 180.0 / M_PI << " degrees)");
                }
            } else {
                ROS_WARN("[error]IK solve failed!");
            }
            new_target_pose_received = false;
        }
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}