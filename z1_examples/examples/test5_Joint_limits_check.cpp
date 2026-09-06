#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>
/***********     程序运行结果
            === Joint Limits ===
 Joint joint1: [-2.61799 rad, 2.61799 rad]
 Joint joint2: [0 rad, 2.96706 rad]
 Joint joint3: [-2.87979 rad, 0 rad]
 Joint joint4: [-1.51844 rad, 1.51844 rad]
 Joint joint5: [-1.3439 rad, 1.3439 rad]
 Joint joint6: [-2.79253 rad, 2.79253 rad]
************/
int main(int argc, char** argv) {
    ros::init(argc, argv, "workspace_analyzer");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    const moveit::core::JointModelGroup* joint_model_group = 
        move_group.getCurrentState()->getJointModelGroup("manipulator");
    
    // 获取所有关节模型
    const std::vector<const moveit::core::JointModel*>& joint_models = 
        joint_model_group->getJointModels();
    
    ROS_INFO_STREAM("=== Joint Limits ===");
    
    for (const auto& joint_model : joint_models) {
        // 跳过固定关节
        if (joint_model->getType() == moveit::core::JointModel::FIXED)
            continue;
            
        // 获取第一个变量的边界（通常对应位置）
        const moveit::core::VariableBounds& bounds = 
            joint_model->getVariableBounds()[0];
            
        ROS_INFO_STREAM("Joint " << joint_model->getName() 
            << ": [" << bounds.min_position_ << " rad, " 
            << bounds.max_position_ << " rad]");
    }
    
    return 0;
}