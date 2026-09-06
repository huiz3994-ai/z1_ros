#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_eigen/tf2_eigen.h>
/*********************************************
 *   ********  *******    ****    ********
 *      **     **        **   *      **
 *      **     *******     ***       **
 *      **     **        *  ***      **
 *      **     *******    ***        **
 * *******************************************/
int main(int argc, char** argv) {
    ros::init(argc, argv, "real_time_fk_solver");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    // 初始化MoveGroup接口
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    const std::string base_frame = move_group.getPlanningFrame(); // 获取基坐标系
    const std::string end_effector_link = move_group.getEndEffectorLink();
    
    // 获取机器人模型和关节组
    const moveit::core::RobotModelConstPtr& kinematic_model = move_group.getRobotModel();
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    
    ros::Rate rate(10);  // 控制循环频率
    
    while (ros::ok()) {
        // 获取当前关节角度
        std::vector<double> joint_values = move_group.getCurrentJointValues();
        
        // if (joint_values.empty()) {
        //     ROS_WARN("Failed to get joint values!");
        //     continue;
        // }

        // 创建机器人状态并设置关节角度
        moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
        current_state->setJointGroupPositions(joint_model_group, joint_values);
        
        // 正运动学计算
        const Eigen::Isometry3d& end_effector_state = current_state->getGlobalLinkTransform(end_effector_link);
        
        // 转换为ROS消息格式
        geometry_msgs::Pose end_effector_pose = tf2::toMsg(end_effector_state);
        
        // 打印结果
        ROS_INFO_STREAM("End effector pose relative to " << base_frame << ":");
        ROS_INFO_STREAM("Position: ["
            << end_effector_pose.position.x << ", " 
            << end_effector_pose.position.y << ", "
            << end_effector_pose.position.z << "]");
        ROS_INFO_STREAM("Orientation: ["
            << end_effector_pose.orientation.w << ", "
            << end_effector_pose.orientation.x << ", "
            << end_effector_pose.orientation.y << ", "
            << end_effector_pose.orientation.z << "]");
        
        rate.sleep();
    }
    
    return 0;
}