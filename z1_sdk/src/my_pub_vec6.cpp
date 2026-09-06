#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <z1_sdk/Vec6.h> 

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "vec6_publisher");
    ros::NodeHandle nh;

    // 创建一个发布者，发布自定义的 Vec6 消息
    ros::Publisher pub = nh.advertise<z1_sdk::Vec6>("vec6_topic", 10);

    // 设置发布频率
    ros::Rate rate(30);

    while (ros::ok())
    {
        // 创建一个 Eigen::Matrix<double, 6, 1> 向量
        Eigen::Matrix<double, 6, 1> vec6;
        vec6 << 0.3, 0, 0, 0, 0, 0;

        // 将 Eigen 向量转换为 ROS 消息
        z1_sdk::Vec6 msg;
        for (int i = 0; i < 6; ++i) {
            msg.data[i] = vec6(i);
        }

        // 发布消息
        pub.publish(msg);

        // 打印日志
        ROS_INFO("Publishing Vec6: [%f, %f, %f, %f, %f, %f]",
                 msg.data[0], msg.data[1], msg.data[2],
                 msg.data[3], msg.data[4], msg.data[5]);

        // 按照设定的频率休眠
        rate.sleep();
    }

    return 0;
}