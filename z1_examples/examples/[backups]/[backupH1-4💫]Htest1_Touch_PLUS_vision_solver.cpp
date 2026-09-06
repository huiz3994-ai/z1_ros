/**********************
Htest1_Touch_PLUS_vision_solver.cpp（已集成 Touch 相对姿态控制）
原理：
=）记录两份“锚点”：
touch_anchor：触发（按下）那一刻 Touch 的姿态（四元数）
robot_anchor：触发（按下）那一刻 机械臂的姿态（四元数）

=）当前 Touch 姿态 q_touch 与 touch_anchor 的相对旋转：
q_rel = q_touch * q_touch_anchor.inverse()（表示把 touch 从锚点转到当前的旋转）

=）如果想把相同的相对旋转作用到机械臂锚点上，新的机械臂姿态：
q_new = q_rel * q_robot_anchor

=）如果需要按比例缩放这个姿态变化（类似位置上乘以系数 K_touch_ori_FIXED），把 q_rel 转成轴角 (axis, angle)，将 angle 乘以系数后重建四元数再做合成：
q_rel_scaled = axis_angle(axis, angle * K)，再 q_new = q_rel_scaled * q_robot_anchor。

📅2025.9.4 张耀华
*************************/

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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <signal.h>
#include <atomic>
#include <map>
#include <cmath> // M_PI

// 启动参数
std::atomic<int> ctrl_mode(1);
#define only_touch 1
#define aruco_for_pos 2
#define aruco_for_ori 3
#define mixed_pos 4

// 全局变量
geometry_msgs::PoseStamped target_pose_msg;
bool new_target_pose_received = false;
bool Anchor_updated = false;
bool orient_anchor_set = false;
bool back_to_home = false;
volatile sig_atomic_t exit_flag = 0;

double K_touch_pos_FIXED = 0.001;
double K_touch_ori_FIXED = 1; // 若需要按角度缩放，可在接收端实现
double K_aruco_pos_FIXED = 1;
double K_aruco_ori_FIXED = 1;

float x_Anchor_arm_received = 0.25;
float y_Anchor_arm_received = 0;
float z_Anchor_arm_received = 0.43;
float delta_x_received = 0;
float delta_y_received = 0;
float delta_z_received = 0;

// ** 新增：接收姿态相关变量 **
geometry_msgs::Quaternion delta_orient_received;      // Touch 发布的 q_delta = q_anchor^{-1} * q_current
geometry_msgs::Quaternion orient_anchor_arm_received; // 机械臂端记录的姿态锚点（由 /end_effector_pose 在收到 Touch_Ori_Anchor_updated 时记录）
bool orient_anchor_update_request = false;            // 当收到 Touch_Ori_Anchor_updated 时置 true，在 poseAnchorCallback 中记录并清零

int last_grey_state = 0;
int last_white_state = 0;

// 共享与平滑控制
bool ori_init_once = false;
bool aruco_data_active_flag = false;
double K_touch_pos_CTRL = 1;
double K_touch_ori_CTRL = 0.2;
double K_aruco_ori_CTRL = 1;
int temp_counter = 0;

geometry_msgs::PoseStamped current_pose;
const double SMOOTH_FACTOR_POS = 0.02;
const double SMOOTH_FACTOR_ORI = 0.01;
bool first_pose_received = false;

geometry_msgs::PoseStamped processed_aruco_marker_pose;

// 工具：检测四元数是否合法（分量有限且模长不接近0）
inline bool isValidQuaternion(const geometry_msgs::Quaternion &q)
{
    using std::isfinite;
    if (!isfinite(q.x) || !isfinite(q.y) || !isfinite(q.z) || !isfinite(q.w))
        return false;
    double norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return norm2 > 1e-8; // 阈值可调
}

void sigintHandler(int sig)
{
    exit_flag = 1;
    ROS_WARN_STREAM("[SHUT DOWN] Captured exit signal, preparing...");
}

// 辅助：把四元数转换为轴角并返回 scaled 四元数
// q_in: 已归一化的四元数； K: 缩放系数； max_angle_rad: 缩放后角度的最大绝对值（弧度），<=0 表示不限制
static tf2::Quaternion scaleQuaternionByAngle(const tf2::Quaternion &q_in, double K, double max_angle_rad = M_PI/2.0) {
    // 保护：确保四元数归一
    tf2::Quaternion q = q_in;
    q.normalize();

    double qw = q.getW();
    // clamp qw 数值到 [-1,1] 以防数值误差
    if (qw > 1.0) qw = 1.0;
    if (qw < -1.0) qw = -1.0;

    double angle = 2.0 * acos(qw); // 返回 [0, pi]
    double s = sqrt(std::max(0.0, 1.0 - qw*qw)); // s = sin(angle/2)

    tf2::Vector3 axis;
    if (s < 1e-8) {
        // 角度极小：选任意轴（不会影响结果，因为 angle ≈ 0）
        axis = tf2::Vector3(1.0, 0.0, 0.0);
    } else {
        axis = tf2::Vector3(q.getX() / s, q.getY() / s, q.getZ() / s);
        axis.normalize();
    }

    double angle_scaled = angle * K;

    // 可选：限制最大角度（避免过大跳变），如果 max_angle_rad <= 0 则表示不限制
    if (max_angle_rad > 0.0 && angle_scaled > max_angle_rad) {
        angle_scaled = max_angle_rad;
    }

    tf2::Quaternion q_out;
    q_out.setRotation(axis, angle_scaled);
    q_out.normalize();
    return q_out;
}

/* 回调声明（包括新增的姿态订阅回调） */
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
void poseFROMarucoCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
void AnchorboolCallback(const std_msgs::Bool::ConstPtr &msg);
void arucoflagboolCallback(const std_msgs::Bool::ConstPtr &msg);
void BackToHomeboolCallback(const std_msgs::Bool::ConstPtr &msg);
void BackToZeroboolCallback(const std_msgs::Bool::ConstPtr &msg);
void poseAnchorCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
void poseIncrementCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);
void orientIncrementCallback(const std_msgs::Float64MultiArray::ConstPtr &msg); // 新增：接收 Touch_Orient_increment
void OriAnchorFlagCallback(const std_msgs::Bool::ConstPtr &msg);                // 新增：接收 Touch_Ori_Anchor_updated

int main(int argc, char **argv)
{
    ros::init(argc, argv, "H_real_time_mixed_ctrl_solver");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 参数服务器与模式映射（保留）
    ros::NodeHandle nh("~");
    std::map<std::string, std::int32_t> ctrl_mode_map = {
        {"default", only_touch},
        {"aruco_for_pos", aruco_for_pos},
        {"aruco_for_ori", aruco_for_ori},
        {"mixed_pos", mixed_pos}};
    std::string ctrl_mode_str;
    nh.param<std::string>("ctrl_mode", ctrl_mode_str, "default");
    auto it = ctrl_mode_map.find(ctrl_mode_str);
    if (it != ctrl_mode_map.end())
    {
        ctrl_mode.store(it->second);
        ROS_INFO_STREAM("Control mode set to: " << ctrl_mode_str << " (" << it->second << ")");
    }
    else
    {
        ROS_WARN_STREAM("Invalid control mode parameter: " << ctrl_mode_str << ". Using default.");
        ctrl_mode.store(ctrl_mode_map["default"]);
    }

    signal(SIGINT, sigintHandler);
    signal(SIGTERM, sigintHandler);

    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    const std::string end_effector_link = move_group.getEndEffectorLink();
    const moveit::core::RobotModelConstPtr &kinematic_model = move_group.getRobotModel();
    const moveit::core::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("manipulator");

    const double timeout = 0.1;
    const int attempts = 10;
    double solve_time = 0.0;
    ros::Rate rate(300);

    // 话题初始化（新增两个姿态相关订阅）
    ros::Subscriber pose_sub = nh.subscribe("/Geomagic/pose", 10, poseCallback);
    ros::Subscriber pose_from_aruco_sub = nh.subscribe("/aruco_single/pose", 10, poseFROMarucoCallback);
    ros::Subscriber flag_from_aruco_sub = nh.subscribe("/aruco_single/aruco_detected_flag", 1, arucoflagboolCallback);
    ros::Subscriber sub = nh.subscribe("/Touch_Anchor_updated", 1, AnchorboolCallback);
    ros::Subscriber sub_ori_anchor_flag = nh.subscribe("/Touch_Ori_Anchor_updated", 1, OriAnchorFlagCallback); // 新增
    ros::Subscriber back_to_home_sub = nh.subscribe("/back_to_home_arm", 1, BackToHomeboolCallback);
    ros::Subscriber back_to_zero_sub = nh.subscribe("/back_to_zero_arm", 1, BackToZeroboolCallback);
    ros::Subscriber pose_Anchor_sub = nh.subscribe("/end_effector_pose", 1, poseAnchorCallback);
    ros::Subscriber increment_sub = nh.subscribe("/Touch_Pose_increment", 10, poseIncrementCallback);
    ros::Subscriber orient_increment_sub = nh.subscribe("/Touch_Orient_increment", 10, orientIncrementCallback); // 新增
    ros::Subscriber sub_Touch_button = nh.subscribe("/Geomagic/joy", 1, joyCallback);
    ros::Publisher joint_pub = nh.advertise<std_msgs::Float64MultiArray>("/goal_joint_positions", 10);
    ros::Publisher target_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ttttttttarget_pose", 10);

    // 初始化 delta_orient 为单位四元数
    orient_anchor_set = false;
    delta_orient_received.x = 0.0;
    delta_orient_received.y = 0.0;
    delta_orient_received.z = 0.0;
    delta_orient_received.w = 1.0;
    orient_anchor_arm_received = delta_orient_received;

    while (ros::ok() && !exit_flag)
    {
        if (new_target_pose_received)
        {
            if (!first_pose_received)
            {
                current_pose = target_pose_msg;
                first_pose_received = true;
            }

            target_pose_pub.publish(target_pose_msg); // 发布目标位姿

            // 位置插值
            current_pose.pose.position.x += SMOOTH_FACTOR_POS *
                                            (target_pose_msg.pose.position.x - current_pose.pose.position.x);
            current_pose.pose.position.y += SMOOTH_FACTOR_POS *
                                            (target_pose_msg.pose.position.y - current_pose.pose.position.y);
            current_pose.pose.position.z += SMOOTH_FACTOR_POS *
                                            (target_pose_msg.pose.position.z - current_pose.pose.position.z);

            // 姿态 slerp
            tf2::Quaternion q_current, q_target, q_interp;
            tf2::fromMsg(current_pose.pose.orientation, q_current);
            tf2::fromMsg(target_pose_msg.pose.orientation, q_target);
            q_interp = tf2::slerp(q_current, q_target, SMOOTH_FACTOR_ORI);
            q_interp.normalize();
            current_pose.pose.orientation = tf2::toMsg(q_interp);

            // IK 解算
            Eigen::Isometry3d target_pose;
            tf2::fromMsg(current_pose.pose, target_pose);

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

                static std::vector<double> last_joint_values(joint_values.size(), 0.0);
                for (size_t i = 0; i < joint_values.size(); ++i)
                {
                    joint_values[i] = last_joint_values[i] + 0.5 *
                                                                 (joint_values[i] - last_joint_values[i]);
                }
                last_joint_values = joint_values;

                // 发布关节
                std_msgs::Float64MultiArray joint_positions_msg;
                joint_positions_msg.data = joint_values;
                joint_pub.publish(joint_positions_msg);

                ros::Time end_time = ros::Time::now();
                double time_diff = (end_time - start_time).toSec();
                if (time_diff > 0.0003)
                {
                    solve_time = time_diff;
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

    if (exit_flag)
    {
        ROS_WARN_STREAM("[SHUT DOWN] Sending back to zero cmd...");
        std_msgs::Float64MultiArray joint_positions_msg;
        joint_positions_msg.data = {0.0, 0.0, -0.005, -0.074, 0.0, 0.0};
        for (int i = 0; i < 3; ++i)
        {
            joint_pub.publish(joint_positions_msg);
            ros::spinOnce();
            ros::Duration(0.05).sleep();
        }
        ROS_WARN_STREAM("[SHUT DOWN] Cmd sent successfully, shutting down the node...");
        ros::Duration(0.5).sleep();
    }

    return 0;
}

/* ----------------- 回调实现区 ----------------- */

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    geometry_msgs::PoseStamped processed_pose = *msg;
    static geometry_msgs::Quaternion last_orientation;
    static bool first_run = true;
    if (first_run)
    {
        last_orientation.w = 1.0;
        last_orientation.x = last_orientation.y = last_orientation.z = 0.0;
        first_run = false;
    }

    int current_ctrl_mode = ctrl_mode.load();
    switch (current_ctrl_mode)
    {
    case only_touch:
        // 位置（相对控制，保持不变）
        processed_pose.pose.position.x = x_Anchor_arm_received;
        processed_pose.pose.position.y = y_Anchor_arm_received;
        processed_pose.pose.position.z = z_Anchor_arm_received;
        processed_pose.pose.position.x += K_touch_pos_FIXED * (-delta_z_received);
        processed_pose.pose.position.y += K_touch_pos_FIXED * (delta_y_received);
        processed_pose.pose.position.z += K_touch_pos_FIXED * (delta_x_received);

        // 姿态（相对控制）
        // 当白键按下时使用 Touch 发布的相对四元数 delta_orient_received，并把它作用到机械臂的锚点姿态 orient_anchor_arm_received
        // 替换或修改原先的分支
        if (last_white_state)
        {
            // 只有在机械臂已记录锚点且收到有效的 delta_orient 时才执行相对姿态合成
            if (!orient_anchor_set)
            {
                ROS_WARN_STREAM_THROTTLE(1.0, "[WARN] White key pressed but arm orientation anchor NOT set yet. Ignoring orient update.");
                processed_pose.pose.orientation = last_orientation;
            }
            else if (!isValidQuaternion(orient_anchor_arm_received))
            {
                ROS_WARN_STREAM_THROTTLE(1.0, "[WARN] arm orient anchor invalid. Ignoring orient update.");
                processed_pose.pose.orientation = last_orientation;
            }
            else if (!isValidQuaternion(delta_orient_received))
            {
                ROS_WARN_STREAM_THROTTLE(1.0, "[WARN] delta_orient_received invalid. Ignoring orient update.");
                processed_pose.pose.orientation = last_orientation;
            }
            else
            {
                tf2::Quaternion q_robot_anchor(
                    orient_anchor_arm_received.x,
                    orient_anchor_arm_received.y,
                    orient_anchor_arm_received.z,
                    orient_anchor_arm_received.w);

                tf2::Quaternion q_delta(
                    delta_orient_received.x,
                    delta_orient_received.y,
                    delta_orient_received.z,
                    delta_orient_received.w);

                // 额外保险：再 normalize
                q_robot_anchor.normalize();
                q_delta.normalize();

                tf2::Quaternion q_new = q_robot_anchor * q_delta;

                // 固定偏转（保持你原来的 -90°）//目前效果是按下就转90度
                // const double half_pi = M_PI / 2.0;
                // tf2::Quaternion q_offset;
                // q_offset.setRotation(tf2::Vector3(1.0, 0.0, 0.0), -half_pi);
                // q_new = q_new * q_offset;

                // 检查 q_new 有效性
                if (!std::isfinite(q_new.x()) || !std::isfinite(q_new.y()) ||
                    !std::isfinite(q_new.z()) || !std::isfinite(q_new.w()))
                {
                    ROS_ERROR_STREAM("[ERROR] Computed q_new contains NaN/Inf. Skipping orientation update.");
                    processed_pose.pose.orientation = last_orientation;
                    // 这里可选择把 orient_anchor_set = false; 或 delta_orient_received = identity 以避免重复错误
                }
                else
                {
                    q_new.normalize();
                    processed_pose.pose.orientation = tf2::toMsg(q_new);
                    last_orientation = processed_pose.pose.orientation;
                }
            }
        }
        else
        {
            processed_pose.pose.orientation = last_orientation;
        }

        break;

    default:
        ROS_ERROR_STREAM("[ERROR] Invalid control mode!");
        break;
    }

    if (back_to_home)
    {
        processed_pose.pose.position.x = 0.25;
        processed_pose.pose.position.y = 0;
        processed_pose.pose.position.z = 0.43;
        last_orientation.w = 1.0;
        last_orientation.x = last_orientation.y = last_orientation.z = 0.0;
    }

    target_pose_msg = processed_pose;
    new_target_pose_received = true;
}

void poseFROMarucoCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // 保持你原有的 aruco 处理逻辑（按需粘回）
    geometry_msgs::PoseStamped processed_pose2 = *msg;
    // ...（略）...
    processed_aruco_marker_pose = processed_pose2;
    new_target_pose_received = true;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
    if (msg->buttons.size() < 2)
    {
        ROS_WARN_ONCE("Invalid buttons message size!");
        return;
    }
    int current_grey = msg->buttons[0];
    int current_white = msg->buttons[1];

    if (current_grey != last_grey_state)
    {
        if (current_grey == 1)
            ROS_INFO_STREAM("Gray button PRESSED");
        else
            ROS_INFO_STREAM("Gray button RELEASED");
        last_grey_state = current_grey;
    }

    if (current_white != last_white_state)
    {
        if (current_white == 1)
            ROS_INFO_STREAM("White button PRESSED");
        else
            ROS_INFO_STREAM("White button RELEASED");
        last_white_state = current_white;
    }
}
void AnchorboolCallback(const std_msgs::Bool::ConstPtr &msg)
{
    Anchor_updated = msg->data;
}
void OriAnchorFlagCallback(const std_msgs::Bool::ConstPtr &msg)
{
    // Touch 端发送 Touch_Ori_Anchor_updated -> 机器人在下一次收到自身 /end_effector_pose 时记录锚点姿态
    if (msg->data)
    {
        orient_anchor_update_request = true;
    }
}

void arucoflagboolCallback(const std_msgs::Bool::ConstPtr &msg)
{
    aruco_data_active_flag = msg->data;
}

void BackToHomeboolCallback(const std_msgs::Bool::ConstPtr &msg)
{
    back_to_home = msg->data;
    ROS_INFO_STREAM("[HOME]Received back to home cmd!");
}
void BackToZeroboolCallback(const std_msgs::Bool::ConstPtr &msg)
{
    if (msg->data)
        exit_flag = 1;
    ROS_INFO_STREAM("[SHUT DOWN] Received back to zero cmd!");
}

// 接收机械臂自身末端位姿（用于在 Anchor_updated / orient_anchor_update_request 时记录锚点）
void poseAnchorCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if (Anchor_updated)
    {
        x_Anchor_arm_received = msg->pose.position.x;
        y_Anchor_arm_received = msg->pose.position.y;
        z_Anchor_arm_received = msg->pose.position.z;
        Anchor_updated = false;
        ROS_INFO_STREAM("[ANCHOR] Arm position anchor recorded.");
    }
    if (orient_anchor_update_request)
    {
        orient_anchor_arm_received = msg->pose.orientation;

        // 归一化一遍并校验
        tf2::Quaternion qtmp(orient_anchor_arm_received.x, orient_anchor_arm_received.y,
                             orient_anchor_arm_received.z, orient_anchor_arm_received.w);
        if (std::isfinite(qtmp.x()) && std::isfinite(qtmp.y()) &&
            std::isfinite(qtmp.z()) && std::isfinite(qtmp.w()))
        {
            qtmp.normalize();
            orient_anchor_arm_received.x = qtmp.x();
            orient_anchor_arm_received.y = qtmp.y();
            orient_anchor_arm_received.z = qtmp.z();
            orient_anchor_arm_received.w = qtmp.w();
            orient_anchor_set = true; // **关键**：告知已记录锚点
            ROS_INFO_STREAM("[ANCHOR] Arm orientation anchor recorded: ("
                            << orient_anchor_arm_received.x << ", "
                            << orient_anchor_arm_received.y << ", "
                            << orient_anchor_arm_received.z << ", "
                            << orient_anchor_arm_received.w << ")");
        }
        else
        {
            ROS_WARN_STREAM("[WARN] Received invalid /end_effector_pose orientation when recording anchor. Ignoring anchor.");
            orient_anchor_set = false;
        }
        orient_anchor_update_request = false;
    }
}

// 接收位置增量
void poseIncrementCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    if (msg->data.size() >= 3)
    {
        delta_x_received = msg->data[0];
        delta_y_received = msg->data[1];
        delta_z_received = msg->data[2];
    }
}

// 接收 Touch 发布的姿态相对四元数增量，格式 [x,y,z,w]，做缩放并写入 delta_orient_received
void orientIncrementCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->data.size() < 4) return;

    // 读取原始 delta（q_delta = q_anchor^{-1} * q_current）
    geometry_msgs::Quaternion q_msg;
    q_msg.x = msg->data[0];
    q_msg.y = msg->data[1];
    q_msg.z = msg->data[2];
    q_msg.w = msg->data[3];

    // 检查有限性
    if (!(std::isfinite(q_msg.x) && std::isfinite(q_msg.y) &&
          std::isfinite(q_msg.z) && std::isfinite(q_msg.w))) {
        ROS_WARN_STREAM_THROTTLE(1.0, "[WARN] orientIncrementCallback: received non-finite quaternion, ignoring.");
        return;
    }

    // 转为 tf2 四元数并归一化
    tf2::Quaternion qraw(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    if (!std::isfinite(qraw.x()) || !std::isfinite(qraw.y()) ||
        !std::isfinite(qraw.z()) || !std::isfinite(qraw.w())) {
        ROS_WARN_STREAM_THROTTLE(1.0, "[WARN] orientIncrementCallback: qraw has NaN/Inf, ignoring.");
        return;
    }
    qraw.normalize();

    // 缩放系数（使用已有全局变量，实时修改生效）
    double K = K_touch_ori_CTRL;
    if (!(std::isfinite(K))) K = 1.0; // 安全兜底

    // 可选：设置缩放后的最大角度，避免过大跳变（例如限制为 60 deg）
    const double max_angle_rad = M_PI * 60.0 / 180.0; // 60 deg 可按需修改

    // 进行轴角缩放
    tf2::Quaternion q_scaled = scaleQuaternionByAngle(qraw, K, max_angle_rad);

    // 可选平滑：把 new delta 与上一次 delta 做插值，避免突变（如果想用，取消注释）
    /*
    const double DELTA_SMOOTH = 0.5; // 0.0 = no smooth, 1.0 = keep old
    tf2::Quaternion q_prev(delta_orient_received.x, delta_orient_received.y, delta_orient_received.z, delta_orient_received.w);
    q_prev.normalize();
    tf2::Quaternion q_smoothed = tf2::slerp(q_prev, q_scaled, 1.0 - DELTA_SMOOTH);
    q_smoothed.normalize();
    q_scaled = q_smoothed;
    */

    // 写回全局 delta_orient_received（geometry_msgs::Quaternion）
    /*====无映射赋值====*/
    // delta_orient_received.x = q_scaled.x();
    // delta_orient_received.y = q_scaled.y();
    // delta_orient_received.z = q_scaled.z();
    // delta_orient_received.w = q_scaled.w();
    /*==做机械臂竖装映射(右)==*/
    delta_orient_received.x = -q_scaled.z();
    delta_orient_received.y = q_scaled.y();
    delta_orient_received.z = q_scaled.x();
    delta_orient_received.w = q_scaled.w();

    // DEBUG（限频打印）
    ROS_INFO_STREAM_THROTTLE(1.0, "[INFO] orientIncrementCallback: received angle*K -> "
                             << "qx=" << delta_orient_received.x
                             << " qy=" << delta_orient_received.y
                             << " qz=" << delta_orient_received.z
                             << " qw=" << delta_orient_received.w
                             << " K=" << K);
}

