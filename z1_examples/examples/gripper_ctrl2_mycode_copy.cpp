#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <geometry_msgs/Twist.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include <algorithm>
#include <ros/timer.h>


// 初始化控制参数
double min_angle = -1.57;  // 全开位置
double max_angle = 0.1;     // 全闭位置
double current_target = max_angle; // 从闭合开始
double step_size = 1;   // 默认步长
double max_effort = 10.0;  // 默认力矩

bool FF = false;
bool SS = false;
bool TT = false;

int cmd_times = 0;

bool open_left = false;
int time_open = 0;

int flag = 0;
int time_kai = 0;
bool first_time = true; 


void callback_cmd(const geometry_msgs::Twist &cmd_v)
{
  time_open ++;
  if( time_open > 180 && time_open < 200 && FF == true){
    current_target -= step_size;
    FF = false;
  }else if (time_open > 210 && time_open < 250){
    current_target = 0.1;
  }else if( time_open > 380 && time_open < 400 && SS == true){
    current_target -= step_size;
    SS = false;
  }else if (time_open > 410 && time_open < 450){
    current_target = 0.1;
  }else if( time_open > 595 && time_open < 600 && TT == true){
    current_target -= step_size;
    TT = false;
  }else if (time_open > 610 && time_open < 700){
    current_target = 0.1;
  }

  ROS_INFO("time_open: %d ",time_open);
  ROS_INFO("current_target: %f ",current_target);

}


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
  

   if (joy->buttons[2]> 0.9) //first yes
    {
        FF = true;
        flag = 1;
    }

    if (joy->buttons[0] > 0.9) //first no
    {
        FF = false;
        flag = 2;
    }

    if (joy->buttons[3]> 0.9) //second yes
    {
        SS = true;
        flag = 3;
    }

    if (joy->buttons[1] > 0.9) //second no
    {
        SS = false;
        flag = 4;
    }

    if (joy->buttons[5] > 0.9) //third yes
    {
        TT = true;
        flag = 5;
    }

    if (joy->axes[5] < -0.9) //third no
    {
        TT = false; 
        flag = 6;
    }

    
    ROS_INFO("flag: %d", flag);
      
 
}

// void callback_cmd(const geometry_msgs::Twist &cmd_v)
// {

//     if(open_left){
//       time_open++;
//       ROS_INFO("time_open: %d ",time_open);
//     }
//     if(time_open > 50)
//     {
//       //close
//       current_target += step_size;
//       time_open = 0;
//       open_left = false;
//       ROS_INFO("time_open: %d ",time_open);
//     }
  

//    if (cmd_v.linear.x == 0 && cmd_v.angular.z == 0 && (!open_left)) //first yes
//     {
//         cmd_times++;
//         ROS_INFO("cmd_times: %d ",cmd_times);
    

//       if (cmd_times == 1 && FF == true) //first no
//       {
//           // open
//           open_left = true;
//           current_target -= step_size;
//           ROS_INFO("FF");
//           FF = false;

//       }

//       if (cmd_times == 2 && SS == true) //second yes
//       {
//           // open
//           open_left = true;
//           current_target -= step_size;
//           ROS_INFO("SS");
//           SS = false;
//       }

//       if (cmd_times == 3 && TT == true) //second no
//       {
//           // open
//           open_left = true;
//           current_target -= step_size;
//           ROS_INFO("TT");
//           TT = false;
//       }

//       if (cmd_times == 4) //second no
//       {
//         current_target = std::max(min_angle, std::min(current_target, max_angle));
//       }
      
//     }



    
//     // ROS_INFO("FF: %.3f | SS: %.3f | TT: %.1f ", 
//     //              FF, SS, TT);
      
 
// }


// void callback_cmd(const geometry_msgs::Twist &cmd_v)
// {

//     // if(open_left){
//     //   time_open++;
//     //   ROS_INFO("time_open: %d ",time_open);
//     // }
//     // if(time_open > 50)
//     // {
//     //   //close
//     //   current_target += step_size;
//     //   time_open = 0;
//     //   open_left = false;
//     //   ROS_INFO("time_open: %d ",time_open);
//     // }
  

//    if (cmd_v.linear.x == 0 && cmd_v.angular.z == 0) //first yes
//     {
//       if(first_time){
//         time_open++;
//         ROS_INFO("time_open: %d ",time_open);
//       }
      
//       if(time_open > 30)
//       {
//         first_time = false;
//         cmd_times++;
//         ROS_INFO("cmd_times: %d ",cmd_times);

        
//         if (cmd_times == 1 && FF == true) //first no
//         {
//             // open
//             ROS_INFO("FF"); 
//             time_open = 0;
//             current_target -= step_size;

//         }

//         if (cmd_times == 2 && SS == true) //second yes
//         {
//             // open
//             current_target -= step_size;
//             ROS_INFO("SS");
//             time_open = 0;
//         }

//         if (cmd_times == 3 && TT == true) //second no
//         {
//             // open
//             current_target -= step_size;
//             ROS_INFO("TT");
//             time_open = 0;
//         }

//         if (cmd_times == 4) //second no
//         {
//           current_target = std::max(min_angle, std::min(current_target, max_angle));
//           time_open = 0;
//         }
//       }
      
//     }else{
//       first_time = true;
//       // if(current_target < -1.0){
//       //   current_target = std::max(min_angle, std::min(current_target, max_angle));
//       // }
//       current_target += step_size;

//     }



    
//     // ROS_INFO("FF: %.3f | SS: %.3f | TT: %.1f ", 
//     //              FF, SS, TT);
      
 
// }


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
  ros::Subscriber sub_cmd;

  // 创建夹爪客户端
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> z1_gripper_client("z1_gripper", true);

  sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &callback);
  sub_cmd = nh.subscribe("cmd_vel", 1, &callback_cmd);
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
      
      // // 调整步长（速度）
      // if (key == 'w') step_size = std::min(step_size * 1.5, 1.2);
      // if (key == 's') step_size = std::max(step_size * 0.8, 0.02);
      
      // // 调整力矩
      // if (key == 'c') max_effort = std::min(max_effort + 1.0, 20.0);
      // if (key == 'v') max_effort = std::max(max_effort - 1.0, 3.0);
      
      // // 方向控制
      // if (key == 'd') current_target += step_size;  // 左键：打开夹爪
      // if (key == 'a') current_target -= step_size;  // 右键：闭合夹爪
      
    
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