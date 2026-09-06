#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_eigen/tf2_eigen.h>  // 新增头文件


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

int main(int argc, char** argv) {
    ros::init(argc, argv, "real_time_ik_solver");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    // 初始化MoveGroup接口
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    const std::string end_effector_link = move_group.getEndEffectorLink();
    
    // 获取机器人模型和关节组
    const moveit::core::RobotModelConstPtr& kinematic_model = move_group.getRobotModel();
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    
    // 逆运动学参数设置
    const double timeout = 0.1;     // 求解超时时间（秒）
    const int attempts = 10;        // 最大尝试次数
    double solve_time = 0.0;        // 单次求解时间
    ros::Rate rate(10);  // 控制循环频率
    
    while (ros::ok()) {
        // 获取当前末端位姿（geometry_msgs/Pose格式）
        geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose(end_effector_link);
        
        // 转换为Eigen格式
        Eigen::Isometry3d target_pose;
        tf2::fromMsg(current_pose.pose, target_pose);
        
        // 创建机器人状态副本
        moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
        moveit::core::RobotState robot_state(*current_state);
        
        ros::Time start_time = ros::Time::now();

        // 执行逆运动学计算
        bool ik_solved = robot_state.setFromIK(
            joint_model_group,       // 关节模型组
            target_pose,            // 目标位姿
            end_effector_link,       // 末端执行器Link
            attempts,               // 尝试次数
            timeout                 // 超时时间
        );
        
        // 处理计算结果
        if (ik_solved) {
            // 提取关节角度
            std::vector<double> joint_values;
            robot_state.copyJointGroupPositions(joint_model_group, joint_values);

            ros::Time end_time = ros::Time::now();
            double time_diff = (end_time - start_time).toSec();
            if (time_diff>0.0003)
            {
                solve_time = time_diff;
            }
            ROS_INFO_STREAM("-----------------Time for solving: " << solve_time << " seconds-----------------");
            
            // 打印关节角度
            ROS_INFO_STREAM("IK Solution Found:");
            for (size_t i = 0; i < joint_values.size(); ++i) {
                ROS_INFO_STREAM("  Joint " << i + 1 << ": " 
                    << joint_values[i] << " radians ("
                    << joint_values[i] * 180.0 / M_PI << " degrees)");
            }
        } else {
            ROS_WARN("[error]IK solve failed!");
        }
        
        rate.sleep();
    }
    
    return 0;
}