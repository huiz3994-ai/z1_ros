/**********************
========|| 添加姿态灵敏度模式切换

使用说明（如何切换/调参）
默认 ori_filter_mode = 1（保留 x 轴灵敏度，其他轴灵敏度 K_swing = 0.25）。
运行时你可以在命令行切换模式，例如：
rostopic pub /Touch_Ori_Filter_Mode std_msgs/Int32 "data: 0" — passthrough
rostopic pub /Touch_Ori_Filter_Mode std_msgs/Int32 "data: 1" — 保留 x，高灵敏度
rostopic pub /Touch_Ori_Filter_Mode std_msgs/Int32 "data: 2" — 全局降敏
rostopic pub /Touch_Ori_Filter_Mode std_msgs/Int32 "data: 3" — custom

调整增益：
在代码里修改 K_twist, K_swing, K_global 的默认值，或把它们从参数服务器读出（我可以帮你把它们加入 nh.param()）。

测试建议：
在不开机器人实际运动时，用 rostopic pub /Touch_Orient_increment 手动发送一些小旋转（例如绕 z 轴 ±10°）测试 mode=0/1/2 的效果。
先把 K_swing 设小（0.1~0.3），K_twist=1.0，观察机械臂在 x 轴旋转上仍敏感、其他方向变慢。
若出现不期望方向，先把日志打开（ROS_DEBUG 或 ROS_INFO）查看 delta_orient_received 与分解后的 swing/twist。

========|| 添加双臂通用代码模式切换

使用说明
only_touch_HR 1 ==> 用于竖装右臂
only_touch_HL 2 ==> 用于竖装左臂

运行节点时命令行尾附加：
_ctrl_mode:=default # 用于竖装右臂
_ctrl_mode:=default2 # 用于竖装左臂

📅2025.9.5 张耀华
**********************/
/**********************
========|| Htest1_Touch_PLUS_vision_solver.cpp（已集成 Touch 相对姿态控制）
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
#include <std_msgs/Int32.h>
#include <sensor_msgs/Joy.h>
#include <signal.h>
#include <atomic>
#include <map>
#include <cmath> // M_PI

// 启动参数
std::atomic<int> ctrl_mode(1);
#define only_touch_HR 1
#define only_touch_HL 2
#define aruco_for_pos 3
#define aruco_for_ori 4
#define mixed_pos 5

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

// ====== 新增全局变量（自由度软锁） ======
int ori_filter_mode = 1; // 默认模式：1 = keep-x-high
double K_twist = 0.85;    // 保持 x 轴灵敏度（默认 1.0 = 不缩放）
double K_swing = 0.2;   // 非 x 轴部分缩放系数（可调，<1 降敏）
double K_global = 0.5;   // 全局缩放系数（模式 2 用）

geometry_msgs::PoseStamped current_pose;
const double SMOOTH_FACTOR_POS = 0.02;
const double SMOOTH_FACTOR_ORI = 0.01;
bool first_pose_received = false;

geometry_msgs::PoseStamped processed_aruco_marker_pose;



/* ----------------- 工具函数区 ----------------- */
void sigintHandler(int sig)
{
    exit_flag = 1;
    ROS_WARN_STREAM("[SHUT DOWN] Captured exit signal, preparing...");
}

// 工具：检测四元数是否合法（分量有限且模长不接近0）
inline bool isValidQuaternion(const geometry_msgs::Quaternion &q)
{
    using std::isfinite;
    if (!isfinite(q.x) || !isfinite(q.y) || !isfinite(q.z) || !isfinite(q.w))
        return false;
    double norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return norm2 > 1e-8; // 阈值可调
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
// swing-twist 分解（q = swing * twist），twist_axis 为单位向量（世界坐标系）
static void swingTwistDecompose(const tf2::Quaternion &q, const tf2::Vector3 &twist_axis_world,
                                tf2::Quaternion &swing, tf2::Quaternion &twist) {
    // q = [v, w]
    tf2::Vector3 v(q.x(), q.y(), q.z());
    tf2::Vector3 proj = twist_axis_world * v.dot(twist_axis_world); // 向量在 axis 上的投影
    // twist quaternion candidate
    twist = tf2::Quaternion(proj.x(), proj.y(), proj.z(), q.w());
    // 在极少数数值情况下，twist 可能是零长度，归一化前检查
    if (std::isfinite(twist.x()) && std::isfinite(twist.y()) && std::isfinite(twist.z()) && std::isfinite(twist.w())) {
        twist.normalize();
    } else {
        twist = tf2::Quaternion(0,0,0,1);
    }
    // swing = q * twist^{-1}
    tf2::Quaternion twist_inv = twist.inverse();
    swing = q * twist_inv;
    swing.normalize();
}
// 把四元数按角度缩放（轴-角方式），q_in 假定为单位四元数
static tf2::Quaternion scaleQuaternionAngle(const tf2::Quaternion &q_in, double K, double max_angle_rad = M_PI) {
    tf2::Quaternion q = q_in;
    q.normalize();
    double qw = q.getW();
    if (qw > 1.0) qw = 1.0;
    if (qw < -1.0) qw = -1.0;
    double angle = 2.0 * acos(qw); // [0, pi]
    double s = sqrt(std::max(0.0, 1.0 - qw*qw)); // sin(angle/2)
    tf2::Vector3 axis;
    if (s < 1e-8) {
        axis = tf2::Vector3(1.0, 0.0, 0.0);
    } else {
        axis = tf2::Vector3(q.getX()/s, q.getY()/s, q.getZ()/s);
        axis.normalize();
    }
    double angle_scaled = angle * K;
    if (max_angle_rad > 0.0 && std::abs(angle_scaled) > max_angle_rad) {
        angle_scaled = (angle_scaled > 0 ? max_angle_rad : -max_angle_rad);
    }
    tf2::Quaternion q_out;
    q_out.setRotation(axis, angle_scaled);
    q_out.normalize();
    return q_out;
}
// 映射：把 touch_basis 四元数 q_in 变为 robot_basis（竖装-右）
// q_in: tf2::Quaternion (touch basis)
// 返回 q_mapped: tf2::Quaternion (robot basis)
static tf2::Quaternion mapTouchToRobotQuaternion(const tf2::Quaternion &q_in){
    // 机械臂（右）竖装的规则： delta.x = -q_scaled.z; delta.y = q_scaled.y; delta.z = q_scaled.x; delta.w = q_scaled.w;
    int current_ctrl_mode_ori = ctrl_mode.load();
    // 首先，在switch外部声明所有需要在最后创建四元数时使用的变量
    double rx, ry, rz, rw; // 此处仅声明，不初始化

    switch(current_ctrl_mode_ori) {
        case only_touch_HR: {// 这里把触摸系 q_in 映射为 robot 格式 q_mapped = M(q_in)
            // 注意：现在这些变量是在这个独立的花括号作用域内声明和计算的
            double rx_local = -q_in.z();
            double ry_local =  q_in.y();
            double rz_local =  q_in.x();
            double rw_local =  q_in.w();
            
            // 将计算结果赋值给外部变量，以便switch结束后使用
            rx = rx_local;
            ry = ry_local;
            rz = rz_local;
            rw = rw_local;
        } 
        break; // break 语句应放在花括号外

        case only_touch_HL: {
            double rx_local = -q_in.z();
            double ry_local = -q_in.y();
            double rz_local = -q_in.x();
            double rw_local =  q_in.w();

            rx = rx_local;
            ry = ry_local;
            rz = rz_local;
            rw = rw_local;
        } 
        break;

        default: {
            double rx_local = -q_in.z();
            double ry_local =  q_in.y();
            double rz_local =  q_in.x();
            double rw_local =  q_in.w();

            rx = rx_local;
            ry = ry_local;
            rz = rz_local;
            rw = rw_local;
        } 
        break;
    }

    // 现在 rx, ry, rz, rw 在此作用域内是已定义的
    tf2::Quaternion q_mapped(rx, ry, rz, rw);
    return q_mapped;
}

/* 回调声明（包括新增的姿态订阅回调） */
void oriFilterModeCallback(const std_msgs::Int32::ConstPtr& msg);
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
        {"default", only_touch_HR},
        {"default2", only_touch_HL},
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
    ros::Subscriber ori_filter_mode_sub = nh.subscribe("/Touch_Ori_Filter_Mode", 1, oriFilterModeCallback);  //自由度软锁
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
//模式控制回调
void oriFilterModeCallback(const std_msgs::Int32::ConstPtr& msg) {
    int m = msg->data;
    if (m < 0) m = 0;
    if (m > 3) m = 3;
    ori_filter_mode = m;
    switch (ori_filter_mode)
    {
    case 0:
        ROS_INFO_STREAM("[ORI FILTER] mode set to " << ori_filter_mode << " ==> origin.");
        break;
    case 1:
        ROS_INFO_STREAM("[ORI FILTER] mode set to " << ori_filter_mode << " ==> higher twist(keep-x-high).");
        break;
    case 2:
        ROS_INFO_STREAM("[ORI FILTER] mode set to " << ori_filter_mode << " ==> global reduced.");
        break;
    case 3:
        ROS_INFO_STREAM("[ORI FILTER] mode set to " << ori_filter_mode << " ==> custom: example lower swing.");
        break;   
    default:
        ROS_WARN_STREAM("[ORI FILTER] mode set failed!! mode now set to " << ori_filter_mode);
        break;
    }
}

//末端位姿回调（主实现）
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
    case only_touch_HR:
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

    case only_touch_HL:
        // 位置（相对控制，保持不变）
        processed_pose.pose.position.x = x_Anchor_arm_received;
        processed_pose.pose.position.y = y_Anchor_arm_received;
        processed_pose.pose.position.z = z_Anchor_arm_received;
        processed_pose.pose.position.x += K_touch_pos_FIXED * (-delta_z_received);
        processed_pose.pose.position.y += K_touch_pos_FIXED * (-delta_y_received);
        processed_pose.pose.position.z += K_touch_pos_FIXED * (-delta_x_received);

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

//aruco 回调，暂时未启用
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
void poseAnchorCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (Anchor_updated) {
        x_Anchor_arm_received = msg->pose.position.x;
        y_Anchor_arm_received = msg->pose.position.y;
        z_Anchor_arm_received = msg->pose.position.z;
        Anchor_updated = false;
    }
    if (orient_anchor_update_request) {
        orient_anchor_arm_received = msg->pose.orientation;
        // 归一化并检查
        if (isValidQuaternion(orient_anchor_arm_received)) {
            tf2::Quaternion qtmp(orient_anchor_arm_received.x, orient_anchor_arm_received.y,
                                 orient_anchor_arm_received.z, orient_anchor_arm_received.w);
            qtmp.normalize();
            orient_anchor_arm_received.x = qtmp.x();
            orient_anchor_arm_received.y = qtmp.y();
            orient_anchor_arm_received.z = qtmp.z();
            orient_anchor_arm_received.w = qtmp.w();
            orient_anchor_set = true; // 标记已设置
            ROS_INFO_STREAM("[ANCHOR] Arm orientation anchor recorded.");
        } else {
            orient_anchor_set = false;
            ROS_WARN_STREAM("[ANCHOR] Received invalid arm anchor orientation; ignored.");
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
// ====== 4) 替换/修改 orientIncrementCallback：在接收阶段先进行模式滤波（或延后在合成前也可） ======
void orientIncrementCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->data.size() < 4) return;

    // 1) 读取原始 touch delta 四元数（touch_basis）
    tf2::Quaternion qraw(msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
    if (!std::isfinite(qraw.x()) || !std::isfinite(qraw.y()) ||
        !std::isfinite(qraw.z()) || !std::isfinite(qraw.w())) {
        ROS_WARN_STREAM_THROTTLE(1.0, "[WARN] orientIncrementCallback: received non-finite quaternion, ignoring.");
        return;
    }
    qraw.normalize();

    // 2) 映射到 robot 格式（这样我们后续以机械臂的本地 x 轴来做 swing-twist）
    tf2::Quaternion q_mapped = mapTouchToRobotQuaternion(qraw);

    // 3) 根据当前过滤模式进行分解与缩放（以 q_mapped 为工作四元数）
    tf2::Quaternion q_scaled_mapped = q_mapped; // 默认 passthrough

    switch (ori_filter_mode) {
        case 0: // passthrough
            q_scaled_mapped = q_mapped;
            break;
        case 1: // keep-x-high: swing-twist with robot anchor x-axis as twist axis
        default: {
            if (!orient_anchor_set) {
                // 未设置锚点时先不过滤（或按需下降警告）
                q_scaled_mapped = q_mapped;
                ROS_WARN_STREAM_THROTTLE(1.0, "[WARN] keep-x-high mode but arm orient anchor not set yet.");
            } else {
                // 计算机械臂锚点的世界系 x 轴:
                tf2::Quaternion q_anchor(orient_anchor_arm_received.x,
                                         orient_anchor_arm_received.y,
                                         orient_anchor_arm_received.z,
                                         orient_anchor_arm_received.w);
                q_anchor.normalize();
                tf2::Vector3 local_x(1.0, 0.0, 0.0);
                tf2::Vector3 twist_axis_world = tf2::quatRotate(q_anchor, local_x);

                // 对 q_mapped 做 swing-twist 分解（注意 q_mapped 已处于 robot 格式）
                tf2::Quaternion swing, twist;
                swingTwistDecompose(q_mapped, twist_axis_world, swing, twist);

                // 缩放 swing / twist（使用你设置的 K_swing / K_twist）
                tf2::Quaternion swing_s = scaleQuaternionAngle(swing, K_swing, M_PI);
                tf2::Quaternion twist_s = scaleQuaternionAngle(twist, K_twist, M_PI);

                q_scaled_mapped = swing_s * twist_s;
                q_scaled_mapped.normalize();
            }
        } break;
        case 2: { // global reduced
            q_scaled_mapped = scaleQuaternionAngle(q_mapped, K_global, M_PI);
        } break;
        case 3: { // custom: example lower swing
            if (!orient_anchor_set) q_scaled_mapped = q_mapped;
            else {
                tf2::Quaternion q_anchor(orient_anchor_arm_received.x,
                                         orient_anchor_arm_received.y,
                                         orient_anchor_arm_received.z,
                                         orient_anchor_arm_received.w);
                q_anchor.normalize();
                tf2::Vector3 twist_axis_world = tf2::quatRotate(q_anchor, tf2::Vector3(1,0,0));
                tf2::Quaternion swing, twist;
                swingTwistDecompose(q_mapped, twist_axis_world, swing, twist);
                tf2::Quaternion swing_s = scaleQuaternionAngle(swing, K_swing * 0.5, M_PI);
                tf2::Quaternion twist_s = scaleQuaternionAngle(twist, K_twist, M_PI);
                q_scaled_mapped = swing_s * twist_s;
                q_scaled_mapped.normalize();
            }
        } break;
    }

    // 4) 把缩放后的 robot-format 四元数直接写入 delta_orient_received（robot 格式对应你的后续使用）
    delta_orient_received.x = q_scaled_mapped.x();
    delta_orient_received.y = q_scaled_mapped.y();
    delta_orient_received.z = q_scaled_mapped.z();
    delta_orient_received.w = q_scaled_mapped.w();

    // debug
    ROS_DEBUG_STREAM_THROTTLE(1.0, "[DEBUG] orientIncrement: delta_orient_received = ["
                               << delta_orient_received.x << ", "
                               << delta_orient_received.y << ", "
                               << delta_orient_received.z << ", "
                               << delta_orient_received.w << "] mode=" << ori_filter_mode);
}

