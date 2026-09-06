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
#include <tf2/LinearMath/Quaternion.h>  // 使用 tf2 的四元数运算库（需安装 libtf2 和依赖）
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // 提供 tf2::toMsg 的转换
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>

/*********************************************
 *   ********  *******    ****    ********
 *      **     **        **   *      **
 *      **     *******     ***       **
 *      **     **        *  ***      **
 *      **     *******    ***        **
 * *******************************************/
/*补充说明：
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

最后编辑于 2025.4.16 张耀华
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
bool Anchor_updated = false;
double K_touch_pos = 0.1; // Touch 末端位置映射缩放系数 (最大化空间利用率建议取 0.6，正常灵敏度 0.2，稳定慢速灵敏度 0.05)
double K_touch_ori = 1;   // Touch 末端姿态映射缩放系数，由于是绝对映射，目前无法缩放，直接修改会导致不可解
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
//geometry_msgs::PoseStamped target_pose_smooth; // 平滑处理后的目标位姿
const double SMOOTH_FACTOR_POS = 0.02;          // 位置平滑系数 
const double SMOOTH_FACTOR_ORI = 0.01;          // 姿态平滑系数 
bool first_pose_received = false;              // 是否收到初始位姿


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
    /******根据机械臂当前位置进行相对位移控制，比例缩放同上（以末端执行器实时位置为锚点，动态映射）******/
    processed_pose.pose.position.x = x_Anchor_arm_received;//启动时 0.25;
    processed_pose.pose.position.y = y_Anchor_arm_received;//启动时 0;
    processed_pose.pose.position.z = z_Anchor_arm_received;//启动时 0.43;
    processed_pose.pose.position.x += K_touch_pos*(-delta_z_received)/100; 
    processed_pose.pose.position.y += K_touch_pos*(-delta_x_received)/100;
    processed_pose.pose.position.z += K_touch_pos*(delta_y_received)/100; 

    // ========== 姿态映射部分 ==========
    // 使用真实姿态映射
    /*坐标轴转换规则：
        Touch的x → 机械臂的-z
        Touch的y → 机械臂的-x
        Touch的z → 机械臂的+y
    对应四元数分量映射：*/
    // if(last_grey_state)// Touch 按下灰色按键时解锁姿态控制
    // {
    //     processed_pose.pose.orientation.x = -msg->pose.orientation.z;  // Touch.z → 机械臂.x
    //     processed_pose.pose.orientation.y = -msg->pose.orientation.x;  // Touch.x → 机械臂.y
    //     processed_pose.pose.orientation.z = msg->pose.orientation.y;   // Touch.y → 机械臂.z
    //     processed_pose.pose.orientation.w = msg->pose.orientation.w;   // 保持四元数实部不变
    // }else{// 松开按键时使用固定姿态（示例：朝前）
    //     processed_pose.pose.orientation.w = 1.0;
    //     processed_pose.pose.orientation.x = 0.0;
    //     processed_pose.pose.orientation.y = 0.0;
    //     processed_pose.pose.orientation.z = 0.0;
    // }
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

    if (FORWARD) {// 演示参数：使用固定姿态（示例：朝前）
        processed_pose.pose.orientation.w = 1.0;
        processed_pose.pose.orientation.x = 0.0;
        processed_pose.pose.orientation.y = 0.0;
        processed_pose.pose.orientation.z = 0.0;
    }

    //写入预处理后的位姿数据
    target_pose_msg = processed_pose;
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
            ROS_INFO("White button PRESSED"); 
            // 这里添加按下处理代码
        } else {
            ROS_INFO("White button RELEASED");
        }
        last_white_state = current_white;
    }
}
void boolCallback(const std_msgs::Bool::ConstPtr& msg) {
    Anchor_updated = msg->data;
    //ROS_INFO("Received: %s", msg->data ? "true" : "false");
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
    ros::Subscriber pose_sub = nh.subscribe("/Geomagic/pose", 10, poseCallback);
    ros::Subscriber sub = nh.subscribe("/Touch_Anchor_updated", 1, boolCallback);
    ros::Subscriber pose_Anchor_sub = nh.subscribe("/end_effector_pose", 1, poseAnchorCallback);
    ros::Subscriber increment_sub = nh.subscribe("/Touch_Pose_increment", 10, poseIncrementCallback);
    ros::Subscriber sub_Touch_button = nh.subscribe("/Geomagic/joy", 1, joyCallback);
    ros::Publisher joint_pub = nh.advertise<std_msgs::Float64MultiArray>("/goal_joint_positions", 10);  // 新增发布器
    // 从参数服务器读取配置(姿态映射选择)
    nh.param<bool>("FORWARD", FORWARD, false);
    ROS_INFO_STREAM("Using " << (FORWARD ? "FIXED" : "MAPPED") 
                    << " orientation mode");

    while (ros::ok()) {
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
            //tf2::fromMsg(target_pose_msg.pose, target_pose);
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
    
    return 0;
}