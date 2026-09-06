#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "doubleArrary_publisher");
    ros::NodeHandle nh;

    // 创建一个发布者，发布自定义的 Vec6 消息
    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("doubleArray_topic", 10);

    // 设置发布频率
    ros::Rate rate(300);

    while (ros::ok())
    {
        // 创建一个消息并赋值
        std_msgs::Float64MultiArray msg;
        // 设置数据
        msg.data.clear();  // 清空数据以防旧数据残留
        msg.data.push_back(0.3);
        msg.data.push_back(0.3);
        msg.data.push_back(-0.3);
        msg.data.push_back(0.3);
        msg.data.push_back(0.3);
        msg.data.push_back(0.3);

        // 发布消息
        pub.publish(msg);

        // 输出日志
        ROS_INFO("Publishing: [%f, %f, %f, %f, %f, %f]", 
             msg.data[0], msg.data[1], msg.data[2], 
             msg.data[3], msg.data[4], msg.data[5]);

        // 处理回调函数
        ros::spinOnce();

        // 按照设定的频率休眠
        rate.sleep();
    }

    return 0;
}