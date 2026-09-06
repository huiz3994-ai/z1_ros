/********
要求 z1_ros/z1_hw/config/controllers.yaml 配置 joint1_position_controller，
并要求 z1_ros/z1_hw/launch/z1_hw.launch 中包含 <arg name="controllers" default="joint_state_controller joint1_position_controller" />
启动参数：
rosrun z1_hw high_freq_control _debug_init_from_current:=true #默认开启。防止cmd不总是从零开始，而是读取节点运行时的状态作为初值
rosrun z1_hw high_freq_control -1 2 #手动设置关节位置目标为 -1 rad，速度为 2
*********/
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cstdlib> // for atof（处理命令行参数）
#include <sensor_msgs/JointState.h> //用于订阅关节状态
#include <mutex>                    //用于订阅关节状态
#include <cmath>                    //用于订阅关节状态
#include <atomic>  //用于防止cmd总是从零开始

// 共享数据保护
std::mutex data_mutex;
double actual_joint_position = 0.0;  // 从/joint_state获取的实际位置（仅用于逻辑判断）
double current_value = 0.0;          // 程序内部控制值（受逻辑调节）
std::atomic<bool> position_initialized(false); // 原子标志位
// 新增调试参数
bool DEBUG_INIT_FROM_CURRENT = true; // 通过参数配置，默认防止cmd总是从零开始

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    static bool first_callback = true;

    if (!msg->position.empty()) {
        std::lock_guard<std::mutex> lock(data_mutex);
        actual_joint_position = msg->position[0]; // 更新joint1实际位置
        // 首次回调初始化
        if(first_callback && DEBUG_INIT_FROM_CURRENT) {
            position_initialized = true;
            first_callback = false;
            ROS_WARN_STREAM("[INIT] Initial position detected: " << actual_joint_position);
            // 初始化current_value
            current_value = actual_joint_position;
            ROS_INFO_STREAM("[INIT] Starting control from: " << current_value);
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "joint1_high_freq_controller");
    ros::NodeHandle nh;

    // 读取调试参数
    nh.param("debug_init_from_current", DEBUG_INIT_FROM_CURRENT, true);

    /******* 解析命令行参数（如果有）*******/
    const double DEFAULT_GOAL = 0.8;
    const double DEFAULT_SPEED = 0.6;
    double goal_joint1_position = (argc > 1) ? atof(argv[1]) : DEFAULT_GOAL;
    double current_jntSpeed = (argc > 2) ? atof(argv[2]) : DEFAULT_SPEED;

    // 创建300Hz发布者发布控制命令
    ros::Publisher pub = nh.advertise<std_msgs::Float64>("/joint1_position_controller/command", 1);
    //创建订阅者订阅关节实际状态
    ros::Subscriber sub = nh.subscribe("/joint_states", 1, jointStateCallback);
    ros::AsyncSpinner spinner(1);   // AsyncSpinner方式创建1个处理线程
    spinner.start();                // 后台自动处理回调，不阻塞控制循环

    // 控制参数
    const double CONTROL_STEP = 0.004; // 对应300Hz
    const double POS_TOLERANCE = 0.001; // 位置容差
    ros::Rate rate(300); // 严格300Hz

    while (ros::ok()) {
        std_msgs::Float64 cmd;

        // 线程安全读取
        double local_actual_pos;
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            local_actual_pos = actual_joint_position;
            // ROS_INFO_STREAM("local_actual_pos="<<local_actual_pos);//调试输出
            // ROS_INFO_STREAM("actual_joint_position="<<actual_joint_position);//调试输出
        }

        /* 核心控制逻辑 */
        if (fabs(current_value - goal_joint1_position) > POS_TOLERANCE) {
            double error = local_actual_pos - current_value;                    // 根据实际位置误差调整
            double step = current_jntSpeed * CONTROL_STEP * (1.0 + 0.5*error);  // 动态调整步长，0.5为误差增益系数；动态特性：当实际位置超前于指令（error>0）：增大步长加速追赶。当实际位置滞后于指令（error<0）：减小步长防止过冲。

            if(goal_joint1_position > current_value) {
                current_value += std::min(step, goal_joint1_position - current_value);// 正向限幅
            } else {
                current_value -= std::min(step, current_value - goal_joint1_position);// 反向限幅
            }
            /********原本控制逻辑********
            // // 根据实际位置误差调整current_value
            // double position_error = goal_joint1_position - local_actual_pos;
            // //ROS_INFO_STREAM("position_error="<<position_error);//调试输出

            // if (position_error > 0) {
            //     current_value += current_jntSpeed * CONTROL_STEP;
            // } else {
            //     current_value -= current_jntSpeed * CONTROL_STEP;
            // }
            ***************************/

            // 值域钳位
            //current_value = std::max(0.0, std::min(current_value, 3.14)); // 示例限制在0~π
           
            cmd.data = current_value;
            pub.publish(cmd);

            ROS_INFO_STREAM_THROTTLE(0.5, 
                "[Control] Target: " << goal_joint1_position 
                << " | Actual: " << local_actual_pos
                << " | Cmd: " << current_value);
        } else {
            ROS_INFO_ONCE("[SUCCESS] Position reached with tolerance: 0.001");
        }

        rate.sleep();
    }

    return 0;
}