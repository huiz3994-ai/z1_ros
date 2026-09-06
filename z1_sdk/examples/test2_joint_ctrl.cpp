#include "unitree_arm_sdk/unitree_arm.h"
#include "ros/ros.h"
/**
 * @example example_joint_ctrl.cpp
 * An example showing how to control joint velocity and position.
 * 
 * Run `roslaunch z1_bringup **_ctrl.launch` first.
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "test2_joint_ctrl");
  ros::NodeHandle nh;
  /* Connect to z1_controller */
  std::string controller_IP = argc > 1 ? argv[1] : "127.0.0.1";
  UNITREE_ARM_SDK::UnitreeArm z1(controller_IP);
  z1.init();

  Timer timer(z1.dt);

  // /* Joint velocity control */
   double jntSpeed = 0.6;
  // Vec6 dq;
  // dq << 0.00473689, 1.57517, -1.60495, -0.00358461, -0.000620296, 0.00205229;
  // for (size_t i = 0; i < 400; i++)
  // {
  //   z1.armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::JointSpeedCtrl;
  //   z1.armCmd.setDq(dq);
  //   z1.armCmd.gripperCmd.angle -= jntSpeed*z1.dt;
  //   z1.sendRecv();
  //   timer.sleep();
  // }

  /* Joint position control */
  // Get current desired q
  Vec6 q = z1.armState.getQ_d();
  // q(0) = -0.000083;
  // q(1) = -0.000014;
  // q(2) = -1.439974;
  // q(3) = -0.000491;
  // q(4) = -0.000257;
  // q(5) = -0.006395;
  
  // ROS_INFO("q: [%f, %f, %f, %f, %f, %f]", 
  //        z1.armState.q[0], 
  //        z1.armState.q[1], 
  //        z1.armState.q[2], 
  //        z1.armState.q[3], 
  //        z1.armState.q[4], 
  //        z1.armState.q[5]);
  for (size_t i = 0; i < 400; i++)
  {
    z1.armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::JointPositionCtrl;
    q(0) += jntSpeed * z1.dt;
    q(2) -= jntSpeed * z1.dt;
    z1.armCmd.setQ(q);
    //z1.armCmd.dq_d[0] = -jntSpeed;
    z1.armCmd.gripperCmd.angle += jntSpeed*z1.dt;
    z1.sendRecv();
    timer.sleep();

      ROS_INFO_STREAM("step value: " << jntSpeed * z1.dt );
      ROS_INFO("q: [%f, %f, %f, %f, %f, %f]", 
         z1.armState.q[0], 
         z1.armState.q[1], 
         z1.armState.q[2], 
         z1.armState.q[3], 
         z1.armState.q[4], 
         z1.armState.q[5]);
  }
  // for (size_t i = 0; i < 400; i++)
  // {
  //   z1.armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::JointPositionCtrl;
  //   q(2) -= jntSpeed * z1.dt;
  //   z1.armCmd.setQ(q);
  //   z1.armCmd.dq_d[0] = -jntSpeed;
  //   z1.armCmd.gripperCmd.angle += jntSpeed*z1.dt;
  //   z1.sendRecv();
  //   timer.sleep();
  // }

  // z1.armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::Passive;
  // z1.sendRecv();
  ros::spin();
  return 0;
}