#include "unitree_arm_sdk/unitree_arm.h"
#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
/**
 * @example example_joint_ctrl.cpp
 * An example showing how to control joint velocity and position.
 * 
 * Run `roslaunch z1_bringup **_ctrl.launch` first.
 */

//调试用变量
//int joint_count = 0;

// 全局变量，用于存储接收到的 double 数据
std::vector<double> received_doubleArray(6, 0.0);
bool received_flag = false;
Vec6 dq;
// 回调函数，处理接收到的消息
void doubleArrayCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    received_doubleArray = msg->data;
    received_flag = true;  // 标记已接收到数据
    
    // 打印接收到的消息
    ROS_INFO("Received: [%f, %f, %f, %f, %f, %f]", 
           msg->data[0], msg->data[1], msg->data[2], 
           msg->data[3], msg->data[4], msg->data[5]);
}


int main(int argc, char** argv)
{
  // 初始化 ROS 节点
  ros::init(argc, argv, "test_joint_ctrl_node");
  ros::NodeHandle nh;

  // 创建订阅者，订阅 double 话题
  ros::Subscriber sub = nh.subscribe("doubleArray_topic", 1, doubleArrayCallback);

  /* Connect to z1_controller */
  std::string controller_IP = argc > 1 ? argv[1] : "127.0.0.1";
  //std::string controller_IP = argc > 1 ? argv[1] : "192.168.123.235";
  UNITREE_ARM_SDK::UnitreeArm z1(controller_IP);
  z1.init();

  //Timer timer(z1.dt);
  
  ros::Rate rate(300);  // 设置 ROS 循环频率为 30 Hz

  while (ros::ok())
    {
      // 如果接收到新的 double 数据，则更新 dq
      if (received_flag) {
        //dq << received_double,0,0,0,0,0;
        for (int i = 0; i < 6; i++)
        {
          dq[i]=received_doubleArray[i];
        }
        received_flag = false;  // 重置标志
        
        // 设置关节速度控制模式
        z1.armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::JointSpeedCtrl;  //设定控制模式为 关节速度控制模式
        z1.armCmd.setDq(dq);  //设置机械臂各关节的目标速度
        //z1.armCmd.gripperCmd.angle -= 0.3*z1.dt;

        // 发送控制命令
        z1.sendRecv();
        //ros::Duration(3).sleep();//等待指令执行
      }
        
      // 控制循环频率
      rate.sleep();
      // 处理 ROS 回调
      ros::spinOnce();

    }
  
  /* Joint velocity control （设定第一个关节的速度为 0.3 rad/s，其他关节速度设为 0）*/
/*
  double jntSpeed = 0.3;
  Vec6 dq;  //dq 是一个 6x1 的向量，代表 6 个关节的速度
  dq << jntSpeed, 0, 0, 0, 0, 0;

  //发送速度控制命令
  for (size_t i = 0; i < 400; i++)  //控制循环执行 400 次，相当于 400 * z1.dt 秒。
  {
    z1.armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::JointSpeedCtrl;  //设定控制模式为 关节速度控制模式
    z1.armCmd.setDq(dq);  //设置机械臂各关节的目标速度。
    //z1.armCmd.gripperCmd.angle -= jntSpeed*z1.dt; //让机械臂的夹爪缓慢闭合（角度减少）
    z1.sendRecv();  //发送控制指令并接收机械臂的状态反馈
    timer.sleep();  //让循环执行的频率符合 z1.dt 的周期，防止指令过快发送
  }
*/
/*******************
  double jntSpeed = 0.3;//临时调试用，仅为实现下面的代码
  /* Joint position control /
  // Get current desired q
  Vec6 q = z1.armState.getQ_d();
  for (size_t i = 0; i < 400; i++)
  {
    z1.armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::JointPositionCtrl;
    q(0) -= jntSpeed * z1.dt;
    z1.armCmd.setQ(q);
    z1.armCmd.dq_d[0] = -jntSpeed;
    z1.armCmd.gripperCmd.angle += jntSpeed*z1.dt;
    z1.sendRecv();
    timer.sleep();
  }
***************/
  z1.armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::Passive;
  z1.sendRecv();

  return 0;
}