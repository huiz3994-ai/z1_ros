/********
要求 z1_ros/z1_hw/config/controllers.yaml 配置 joint_group_position_controller
并要求 z1_ros/z1_hw/launch/z1_hw.launch 中包含 <arg name="controllers" default="joint_state_controller joint_group_position_controller" />

启动参数：
rosrun z1_hw high_freq_control _debug_init_from_current:=true #默认开启。防止cmd不总是从零开始，而是读取节点运行时的状态作为初值
rosrun z1_hw high_freq_control -0.3, 0.3, -0.3, -0.3 ,0.3, 0.3 1 #手动设置关节位置目标为 [-0.3, 0.3, -0.3, -0.3 ,0.3, 0.3] rad，速度为 1

Tips:
未对关节指令做限位，所给关节角度超出实际可达位置后，可能会卡死
*********/
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <cstdlib> // for atof（处理命令行参数）
#include <sensor_msgs/JointState.h> //用于订阅关节状态
#include <mutex>                    //用于订阅关节状态
#include <cmath>                    //用于订阅关节状态
#include <atomic>  //用于防止cmd总是从零开始
#include <vector>

// 共享数据保护
std::mutex data_mutex;
std::vector<double> actual_joint_positions(6, 0.0);  // 6个关节的实际位置
std::vector<double> current_values(6, 0.0);          // 内部控制值数组
std::atomic<bool> position_initialized(false); // 原子标志位
// 调试参数，默认开启
bool DEBUG_INIT_FROM_CURRENT = true; // 通过参数配置，默认防止cmd总是从零开始

void printDoubleArray(const std::vector<double>& Array, const std::string& StartWords, const std::int16_t mode) {
    std::ostringstream ss;
    ss << StartWords << "[";
    for (size_t i = 0; i < Array.size(); ++i) {
        ss << Array[i] << (i < Array.size() - 1 ? ", " : "]");
    }
    switch (mode)
    {
    case 1:ROS_INFO_STREAM(ss.str());break;
    case 2:ROS_WARN_STREAM(ss.str());break;
    default:ROS_INFO_STREAM(ss.str());break;
    }
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    static bool first_callback = true;

    if (msg->position.size() >= 6) {
        std::lock_guard<std::mutex> lock(data_mutex);
        for(int i=0; i<6; i++) {    //更新6关节实际位置
            actual_joint_positions[i] = msg->position[i];
        }
        // 首次回调初始化
        if(first_callback && DEBUG_INIT_FROM_CURRENT) {
            position_initialized = true;
            first_callback = false;
            //ROS_WARN_STREAM("[INIT] Initial position detected: " << actual_joint_positions[0] << actual_joint_positions[1] << actual_joint_positions[2] << actual_joint_positions[3] << actual_joint_positions[4] << actual_joint_positions[5]);
            //printJointPositions(actual_joint_positions, "[INIT] Initial position detected: ");
            printDoubleArray(actual_joint_positions, "[INIT] Initial position detected: ", 2);
            // 初始化所有关节的当前值
            current_values = actual_joint_positions;
            //ROS_INFO_STREAM("[INIT] Starting control from: " << current_values[0] << current_values[1] << current_values[2] << current_values[3] << current_values[4] << current_values[5]);
            //printCurrentValues(current_values, "[INIT] Starting control from: ");
            printDoubleArray(current_values, "[INIT] Starting control from: ", 1);
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "high_freq_controller");
    ros::NodeHandle nh;

    // 读取调试参数
    nh.param("debug_init_from_current", DEBUG_INIT_FROM_CURRENT, true);

    /******* 参数解析 ​*******/
    //命令行参数现在需要7个参数（6个目标位置 + 1个速度）
    const std::vector<double> DEFAULT_GOALS = {-0.3, 0.3, -0.3, -0.3 ,0.3, 0.3};
    //const std::vector<double> DEFAULT_SPEEDS(6, 0.6);
    const double DEFAULT_SPEED = 1;
    
    std::vector<double> goal_positions(6);
    //std::vector<double> joint_speeds(6);
    double joint_speed;

    // 简单参数解析示例，实际应用建议使用ROS参数服务器
    for(int i=0; i<6; i++) {
        goal_positions[i] = (argc > i+1) ? atof(argv[i+1]) : DEFAULT_GOALS[i];
        //joint_speeds[i] = (argc > i+7) ? atof(argv[i+6]) : DEFAULT_SPEEDS[i];
        // goal_positions[i] = DEFAULT_GOALS[i];
        // joint_speeds[i] = DEFAULT_SPEEDS[i];
    }
    joint_speed = (argc > 7) ? atof(argv[7]) : DEFAULT_SPEED;

    // 创建300Hz发布者发布控制命令
    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/joint_group_position_controller/command", 1);
    //创建订阅者订阅关节实际状态
    ros::Subscriber sub = nh.subscribe("/joint_states", 1, jointStateCallback);
    ros::AsyncSpinner spinner(1);   // AsyncSpinner方式创建1个处理线程
    spinner.start();                // 后台自动处理回调，不阻塞控制循环

    // 控制参数
    const double CONTROL_STEP = 0.004; // 对应300Hz
    const double POS_TOLERANCE = 0.001; // 位置容差
    ros::Rate rate(300); // 严格300Hz

    while (ros::ok()) {
        std_msgs::Float64MultiArray cmd;  //创建一个 std_msgs::Float64MultiArray 类型的消息对象 cmd。
        cmd.data.resize(6); //将 cmd.data 的大小调整为 6，并将所有元素初始化为 0.0（默认初始化值）；这种操作通常用于分配空间，确保在后续赋值时不会越界。

        // 线程安全读取，锁的作用域仅限于花括号 { } 内
        std::vector<double> local_actual_pos(6);
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            local_actual_pos = actual_joint_positions;
            // ROS_INFO_STREAM("local_actual_pos="<<local_actual_pos);//调试输出
            // ROS_INFO_STREAM("actual_joint_position="<<actual_joint_position);//调试输出
        }

        /* 核心控制逻辑 */
        bool all_reached = true;
        for(int i=0; i<6; i++) {
            if (fabs(current_values[i] - goal_positions[i]) > POS_TOLERANCE) {
                all_reached = false;
                
                double error = local_actual_pos[i] - current_values[i];            // 根据实际位置误差调整
                //double step = joint_speeds[i] * CONTROL_STEP * (1.0 + 0.5*error);  // 动态调整步长，0.5为误差增益系数；动态特性：当实际位置超前于指令（error>0）：增大步长加速追赶。当实际位置滞后于指令（error<0）：减小步长防止过冲。
                double step = joint_speed * CONTROL_STEP * (1.0 + 0.5*error);

                if(goal_positions[i] > current_values[i]) {
                    current_values[i] += std::min(step, goal_positions[i] - current_values[i]); // 正向限幅
                } else {
                    current_values[i] -= std::min(step, current_values[i] - goal_positions[i]); // 反向限幅
                }
                
                cmd.data[i] = current_values[i];  //写控制指令
            } else {
                cmd.data[i] = goal_positions[i];
                ROS_INFO_ONCE("[SUCCESS] Position reached with tolerance: 0.001");
            }
        }

        // 值域钳位(单关节，未测试)
        //current_values[0] = std::max(0.0, std::min(current_value, 3.14)); // 示例限制在0~π

        if(!all_reached) {
            pub.publish(cmd);      //发布控制指令
            std::stringstream ss;  //创建一个字符串流对象 ss，用于动态拼接字符串，效率优于 std::string 的多次拼接。
            ss << "[Control] Targets:";
            for(auto& g : goal_positions) ss << " " << g; //遍历 goal_positions（目标位置数组），将每个目标位置拼接到 ss 中。
            ss << " | Cmds:";
            for(auto& c : current_values) ss << " " << c; //遍历 current_values（当前值数组），将每个当前值拼接到 ss 中
            ROS_INFO_STREAM_THROTTLE(0.5, ss.str());      //每 0.5 秒输出一次日志
        } else {
            ROS_INFO_ONCE("[SUCCESS] All joints reached target positions");  //ROS_INFO_ONCE 仅在条件满足时输出一次日志，避免多次输出成功提示。
        }
           
        //     cmd.data = current_value;
        //     pub.publish(cmd);

        //     ROS_INFO_STREAM_THROTTLE(0.5, 
        //         "[Control] Target: " << goal_joint1_position 
        //         << " | Actual: " << local_actual_pos
        //         << " | Cmd: " << current_value);
        // } else {
        //     ROS_INFO_ONCE("[SUCCESS] Position reached with tolerance: 0.001");
        // }

        rate.sleep();
    }

    return 0;
}