#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <random>
/***程序运行结果（机械臂末端笛卡尔空间坐标取值范围）
X Range: [-0.659, 0.668]
Y Range: [-0.657, 0.678]
Z Range: [-0.167, 0.797]
****/
/***可视化结果查看方法
[启动rviz，添加点云显示层]
点击左下角 ​Add 按钮
选择 ​By topic → ​PointCloud2 → ​/workspace_points
确认 ​Global Options 中 ​Fixed Frame 设置为机械臂基坐标系（如base_link）
​
[显示优化配置]
    参数项	           推荐设置	            作用说明
Style	              Points	      点云显示为离散点
Size (Pixels)	       3-5	           增大点云可见性
Color Transformer	Intensity	      根据Z轴高度染色
Decay Time	            0	           避免历史点残留

[数据保存与后处理]
​1.实时数据记录
使用rosbag记录点云数据：
bash`
rosbag record -O workspace_data.bag /workspace_points
​2.点云导出分析
在程序中添加PCD文件保存代码：
cpp`
pcl::io::savePCDFileASCII("workspace.pcd", cloud);
可用CloudCompare或Python Open3D库进行三维分析。
*******/
int SIMPLE_MAX = 10000; //蒙特卡洛采样样本数量，默认 10000，可尝试5至10万提升边界精度

int main(int argc, char** argv) {
    ros::init(argc, argv, "workspace_calculator");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    const moveit::core::JointModelGroup* joint_model_group = 
        move_group.getCurrentState()->getJointModelGroup("manipulator");
    
    // 初始化点云
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("workspace_points", 1);
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // 正确获取关节限制
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    std::vector<std::pair<double, double>> joint_limits;
    for (const auto& name : joint_names) {
        const auto* joint = joint_model_group->getJointModel(name);
        if (joint->getType() == moveit::core::JointModel::REVOLUTE) {
            const moveit::core::VariableBounds& bounds = joint->getVariableBounds()[0];
            joint_limits.emplace_back(bounds.min_position_, bounds.max_position_);
        }
    }

    // 随机数生成器
    std::default_random_engine generator;
    std::vector<std::uniform_real_distribution<double>> distributions;
    for (const auto& limit : joint_limits) {
        distributions.emplace_back(limit.first, limit.second);
    }

    // 蒙特卡洛采样
    const int SAMPLE_NUM = SIMPLE_MAX;
    for (int i = 0; i < SAMPLE_NUM; ++i) {
        std::vector<double> joint_values;
        for (auto& dist : distributions) {
            joint_values.push_back(dist(generator));
        }

        moveit::core::RobotStatePtr state = move_group.getCurrentState();
        state->setJointGroupPositions(joint_model_group, joint_values);
        
        if (state->satisfiesBounds(joint_model_group)) {
            Eigen::Isometry3d end_effector_pose = state->getGlobalLinkTransform(
                move_group.getEndEffectorLink());
            
            pcl::PointXYZ point;
            point.x = end_effector_pose.translation().x();
            point.y = end_effector_pose.translation().y();
            point.z = end_effector_pose.translation().z();
            cloud.push_back(point);
        }
    }

    // 发布点云
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = move_group.getPlanningFrame();
    pub.publish(output);

    // 计算各轴范围（兼容C++11）
    auto x_extremes = std::minmax_element(cloud.begin(), cloud.end(),
        [](const pcl::PointXYZ& p1, const pcl::PointXYZ& p2){ return p1.x < p2.x; });
    auto y_extremes = std::minmax_element(cloud.begin(), cloud.end(),
        [](const pcl::PointXYZ& p1, const pcl::PointXYZ& p2){ return p1.y < p2.y; });
    auto z_extremes = std::minmax_element(cloud.begin(), cloud.end(),
        [](const pcl::PointXYZ& p1, const pcl::PointXYZ& p2){ return p1.z < p2.z; });

    ROS_INFO("X Range: [%.3f, %.3f]", x_extremes.first->x, x_extremes.second->x);
    ROS_INFO("Y Range: [%.3f, %.3f]", y_extremes.first->y, y_extremes.second->y);
    ROS_INFO("Z Range: [%.3f, %.3f]", z_extremes.first->z, z_extremes.second->z);

    return 0;
}