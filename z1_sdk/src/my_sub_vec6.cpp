#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <z1_sdk/Vec6.h>  // 替换为你的包名

// 回调函数，处理接收到的消息
void vec6Callback(const z1_sdk::Vec6::ConstPtr& msg)
{
    // 将 ROS 消息转换为 Eigen 向量
    Eigen::Matrix<double, 6, 1> vec6;
    for (int i = 0; i < 6; ++i) {
        vec6(i) = msg->data[i];
    }

    // 打印接收到的消息
    ROS_INFO("Received Vec6: [%f, %f, %f, %f, %f, %f]",
             vec6(0), vec6(1), vec6(2),
             vec6(3), vec6(4), vec6(5));
}

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "vec6_subscriber");
    ros::NodeHandle nh;

    // 创建一个订阅者，订阅自定义的 Vec6 消息
    ros::Subscriber sub = nh.subscribe("vec6_topic", 10, vec6Callback);

    // 进入循环，等待消息
    ros::spin();

    return 0;
}