#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


// 初始化控制参数
double min_angle = -1.57;  // 全开位置
double max_angle = 0.1;     // 全闭位置
double current_target = max_angle; // 从闭合开始
double step_size = 0.05;   // 默认步长
double max_effort = 10.0;  // 默认力矩


// 键盘监听函数
int getKey() {
    struct timeval tv = {0L, 0L};
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(0, &fds);
    
    if (select(1, &fds, NULL, NULL, &tv) == 0)
        return -1;
    
    int ch = getchar();
    // 处理方向键（3字节序列）
    if (ch == 27) {
        if (getchar() == 91) {
            ch = getchar();
            switch(ch) {
                case 'D': return 1;  // ←
                case 'C': return 2;  // →
            }
        }
    }
    return ch;
}

void callback(const sensor_msgs::Joy::ConstPtr &joy)
{
  

   if (joy->axes[7] > 0.9) //十字上
    {
        step_size = std::min(step_size * 1.5, 0.15);
    }

    if (joy->axes[7] < -0.9) //十字下
    {
        step_size = std::max(step_size * 0.8, 0.02);
    }

    if (joy->axes[6] > 0.9) //十字左
    {
        max_effort = std::max(max_effort - 1.0, 3.0);
    }

    if (joy->axes[6] < -0.9) //十字右
    {
        max_effort = std::min(max_effort + 1.0, 20.0);
    }

    if (joy->buttons[4] == 1) //扳机左
    {
        current_target += step_size;
    }

    if (joy->axes[2] < -0.9) //扳机右
    {
        current_target -= step_size; 
    }

    
    ROS_INFO("T: %.3f rad | S: %.3f | F: %.1f N·m", 
                 current_target, step_size, max_effort);
      
    // ROS_INFO("Waiting oooookkkkk");

    //  if (key == 'w') step_size = std::min(step_size * 1.5, 0.15);
    //   if (key == 's') step_size = std::max(step_size * 0.8, 0.02);
      
    //   // 调整力矩
    //   if (key == 'c') max_effort = std::min(max_effort + 1.0, 20.0);
    //   if (key == 'v') max_effort = std::max(max_effort - 1.0, 3.0);
      
    //   // 方向控制
    //   if (key == 'd') current_target += step_size;  // 左键：打开夹爪
    //   if (key == 'a') current_target -= step_size;  // 右键：闭合夹爪
      // current_target = std::max(min_angle, std::min(current_target, max_angle));
      
      // // 发送控制指令
      // control_msgs::GripperCommandGoal goal;
      // goal.command.position = current_target;
      // goal.command.max_effort = max_effort;
      // z1_gripper_client.sendGoal(goal);
 
}

int main(int argc, char** argv)
{
  if(argc < 2) {
    std::cerr << "Usage: \n"
              << "Position mode: rosrun ros_gripper_ctrl <angle> [max_effort]\n"
              << "Keyboard mode: rosrun ros_gripper_ctrl k"
              << std::endl;
    return -1;
  }

  ros::init(argc, argv, "gripper_ctrl");
  ros::NodeHandle nh;

  ros::Subscriber sub;

  // 创建夹爪客户端
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> z1_gripper_client("z1_gripper", true);

  sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &callback);
  ROS_INFO("Waiting oooookkkkk");

  // === 键盘控制模式 ===
  if (std::string(argv[1]) == "k") {
    ROS_INFO("Keyboard control mode activated");
    ROS_INFO("Use a d to open/close, w s for speed, c v for force");
    
    z1_gripper_client.waitForServer();
    
    // // 初始化控制参数
    // double min_angle = -1.57;  // 全开位置
    // double max_angle = 0.1;     // 全闭位置
    // double current_target = max_angle; // 从闭合开始
    // double step_size = 0.05;   // 默认步长
    // double max_effort = 10.0;  // 默认力矩
    
    // 设置非阻塞输入模式
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    
    // 控制循环
    while (ros::ok()) {
      int key = getKey();
      
      // 退出程序
      //if (key == 'q' || key == 'Q') break;
      
      // 调整步长（速度）
      if (key == 'w') step_size = std::min(step_size * 1.5, 0.15);
      if (key == 's') step_size = std::max(step_size * 0.8, 0.02);
      
      // 调整力矩
      if (key == 'c') max_effort = std::min(max_effort + 1.0, 20.0);
      if (key == 'v') max_effort = std::max(max_effort - 1.0, 3.0);
      
      // 方向控制
      if (key == 'd') current_target += step_size;  // 左键：打开夹爪
      if (key == 'a') current_target -= step_size;  // 右键：闭合夹爪
      
    
      current_target = std::max(min_angle, std::min(current_target, max_angle));
      
      // 发送控制指令
      control_msgs::GripperCommandGoal goal;
      goal.command.position = current_target;
      goal.command.max_effort = max_effort;
      z1_gripper_client.sendGoal(goal);
      
      // 显示当前状态
      if (key != -1) {
        ROS_INFO("T: %.3f rad | S: %.3f | F: %.1f N·m", 
                 current_target, step_size, max_effort);
      }
      ros::spinOnce();
      ros::Duration(0.05).sleep();
    }
    
    // 恢复终端设置
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return 0;
  }
  
  // === 原始位置控制模式 ===
  if(argc < 2 || argc > 3) {
    std::cerr << "Invalid arguments for position mode" << std::endl;
    return -1;
  }

  ROS_INFO("Waiting for action server to start...");
  z1_gripper_client.waitForServer();

  ROS_INFO("Move Gripper.");
  control_msgs::GripperCommandGoal goal;
  goal.command.position = std::stod(argv[1]); 
  goal.command.max_effort = argc == 3 ? std::stod(argv[2]) : 10.0;

  z1_gripper_client.sendGoal(goal);
  z1_gripper_client.waitForResult();
  
  if(z1_gripper_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Gripper move successful. Position: %f, Effort: %f",
             z1_gripper_client.getResult()->position, 
             z1_gripper_client.getResult()->effort);
  } else {
    ROS_WARN("Gripper movement failed");
  }

  return 0;
}