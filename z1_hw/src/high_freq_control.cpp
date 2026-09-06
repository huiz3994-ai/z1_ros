/********
更新：
添加“初始位置ros参数控制”。
添加高频下发节点调试话题，便于 rosbag 记录后对“目标-下发-实际反馈”进行离线分析。

新增调试话题：
/debug/high_freq/goal_positions
/debug/high_freq/current_values
/debug/high_freq/actual_joint_positions
/debug/high_freq/all_reached
********/

/********
要求 z1_ros/z1_hw/config/controllers.yaml 配置 joint_group_position_controller
并要求 z1_ros/z1_hw/launch/z1_hw.launch 中包含 <arg name="controllers" default="joint_state_controller joint_group_position_controller" />

启动参数：
rosrun z1_hw high_freq_control _debug_init_from_current:=true #默认开启。防止cmd不总是从零开始，而是读取节点运行时的状态作为初值
rosrun z1_hw high_freq_control -0.3, 0.3, -0.3, -0.3 ,0.3, 0.3 1 #手动设置关节位置目标为 [-0.3, 0.3, -0.3, -0.3 ,0.3, 0.3] rad，速度为 1

Tips:
未对关节指令做限位，所给关节角度超出实际可达位置后，可能会卡死

使用说明：
1.通过命令行设置初始位置（可选）：
bash`
rosrun z1_hw high_freq_control -0.3 0.3 -0.3 -0.3 0.3 0.3 1
2.通过ROS话题实时更新目标位置：
bash`
rostopic pub /goal_joint_positions std_msgs/Float64MultiArray "data: [0.1, 0.2, -0.3, -0.4, 0.5, 0.6]"

补充说明：
各项宇树官方的机械臂展示位置量如下
forward, 0.0, 1.5, -1.0, -0.54, 0.0, 0.0, 
startFlat, 0.0, 0.0, -0.005, -0.074, 0.0, 0.0, 
show_left, -1.0, 0.9, -1., -0.3, 0.0, 0.0, 
show_mid, 0.0, 0.9, -1.2, 0.25, 0.0, 0.0, 
show_right, 1.0, 0.9, -1., -0.3, 0.0, 0.0, 
*********/
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <cstdlib>
#include <sensor_msgs/JointState.h>
#include <mutex>
#include <cmath>
#include <atomic>
#include <vector>
#include <map>
#include <sstream>
#include <cstdint>
#include <algorithm>

//机械臂关节速度统一控制，可取小数，设为1较为稳定
#define DEFAULT_JOINT_SPEED 1  

// 共享数据保护
std::mutex data_mutex;
std::vector<double> actual_joint_positions(6, 0.0);
std::vector<double> current_values(6, 0.0);
std::atomic<bool> position_initialized(false);

// 目标位置相关变量
std::vector<double> goal_positions(6, 0.0);
std::mutex goal_positions_mutex;

// 调试参数
bool DEBUG_INIT_FROM_CURRENT = true;

void printDoubleArray(const std::vector<double>& Array, const std::string& StartWords, const std::int16_t mode) {
    std::ostringstream ss;
    ss << StartWords << "[";
    for (size_t i = 0; i < Array.size(); ++i) {
        ss << Array[i] << (i < Array.size() - 1 ? ", " : "]");
    }
    switch (mode) {
        case 1: ROS_INFO_STREAM(ss.str()); break;
        case 2: ROS_WARN_STREAM(ss.str()); break;
        default: ROS_INFO_STREAM(ss.str()); break;
    }
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    static bool first_callback = true;

    if (msg->position.size() >= 6) {
        std::lock_guard<std::mutex> lock(data_mutex);
        for(int i=0; i<6; i++) {
            actual_joint_positions[i] = msg->position[i];
        }

        if(first_callback && DEBUG_INIT_FROM_CURRENT) {
            position_initialized = true;
            first_callback = false;
            current_values = actual_joint_positions;
            printDoubleArray(actual_joint_positions, "[INIT] Initial position detected: ", 2);
            printDoubleArray(current_values, "[INIT] Starting control from: ", 1);
        }
    }
}

void goalPositionsCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->data.size() == 6) {
        std::lock_guard<std::mutex> lock(goal_positions_mutex);
        for(int i=0; i<6; i++) {
            goal_positions[i] = msg->data[i];
        }
    } else {
        ROS_WARN_STREAM("Invalid goal size: " << msg->data.size() << ", expected 6");
    }
}

static std_msgs::Float64MultiArray toMultiArray(const std::vector<double>& values) {
    std_msgs::Float64MultiArray msg;
    msg.data = values;
    return msg;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "high_freq_controller");
    ros::NodeHandle nh;

    nh.param("debug_init_from_current", DEBUG_INIT_FROM_CURRENT, true);
    const double DEFAULT_SPEED = DEFAULT_JOINT_SPEED;

    std::map<std::string, std::vector<double>> PRESET_POSITIONS = {
        {"default",    {0.0, 0.0, -0.005, -0.074, 0.0, 0.0}},
        {"forward0",   {0.0, 1.5, -1.0, -0.54, 0.0, 0.0}},
        {"forward",    {0.0, 1.3, -1.0, -0.32, 0.0, 0.0}},
        {"startFlat",  {0.0, 0.0, -0.005, -0.074, 0.0, 0.0}},
        {"show_left",  {-1.0, 0.9, -1.0, -0.3, 0.0, 0.0}},
        {"show_mid",   {0.0, 0.9, -1.2, 0.25, 0.0, 0.0}},
        {"show_right", {1.0, 0.9, -1.0, -0.3, 0.0, 0.0}}
    };

    std::string preset_name;
    nh.param<std::string>("initial_position", preset_name, "default");

    std::vector<double> selected_preset = PRESET_POSITIONS["default"];
    if (PRESET_POSITIONS.find(preset_name) != PRESET_POSITIONS.end()) {
        selected_preset = PRESET_POSITIONS[preset_name];
    } else {
        ROS_WARN("Unknown initial_position: %s, using default preset.", preset_name.c_str());
    }

    std::vector<double> command_line_goals;
    for(int i = 0; i < 6 && (i+1) < argc; i++) {
        command_line_goals.push_back(atof(argv[i+1]));
    }

    std::vector<double> initial_goals = selected_preset;
    if (command_line_goals.size() == 6) {
        initial_goals = command_line_goals;
        ROS_WARN("Using command line goals instead of preset.");
    }

    {
        std::lock_guard<std::mutex> lock(goal_positions_mutex);
        goal_positions = initial_goals;
    }

    double joint_speed = (argc > 7) ? atof(argv[7]) : DEFAULT_SPEED;

    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/joint_group_position_controller/command", 1);
    ros::Publisher debug_goal_pub = nh.advertise<std_msgs::Float64MultiArray>("/debug/high_freq/goal_positions", 10);
    ros::Publisher debug_current_values_pub = nh.advertise<std_msgs::Float64MultiArray>("/debug/high_freq/current_values", 10);
    ros::Publisher debug_actual_joint_pub = nh.advertise<std_msgs::Float64MultiArray>("/debug/high_freq/actual_joint_positions", 10);
    ros::Publisher debug_all_reached_pub = nh.advertise<std_msgs::Bool>("/debug/high_freq/all_reached", 10);

    ros::Subscriber joint_sub = nh.subscribe("/joint_states", 1, jointStateCallback);
    ros::Subscriber goal_sub = nh.subscribe("/goal_joint_positions", 1, goalPositionsCallback);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    const double CONTROL_STEP = 0.004;
    const double POS_TOLERANCE = 0.001;
    ros::Rate rate(300);   

    while (ros::ok()) {
        std_msgs::Float64MultiArray cmd;
        cmd.data.resize(6);

        std::vector<double> local_actual_pos;
        std::vector<double> current_goals;
        {
            std::lock_guard<std::mutex> lock1(data_mutex);
            std::lock_guard<std::mutex> lock2(goal_positions_mutex);
            local_actual_pos = actual_joint_positions;
            current_goals = goal_positions;
        }

        bool all_reached = true;
        for(int i=0; i<6; i++) {
            if (fabs(current_values[i] - current_goals[i]) > POS_TOLERANCE) {
                all_reached = false;
                double error = local_actual_pos[i] - current_values[i];
                double step = joint_speed * CONTROL_STEP * (1.0 + 0.5*error);

                if(current_goals[i] > current_values[i]) {
                    current_values[i] += std::min(step, current_goals[i] - current_values[i]);
                } else {
                    current_values[i] -= std::min(step, current_values[i] - current_goals[i]);
                }

                cmd.data[i] = current_values[i];
            } else {
                cmd.data[i] = current_goals[i];
            }
        }

        debug_goal_pub.publish(toMultiArray(current_goals));
        debug_current_values_pub.publish(toMultiArray(current_values));
        debug_actual_joint_pub.publish(toMultiArray(local_actual_pos));

        std_msgs::Bool all_reached_msg;
        all_reached_msg.data = all_reached;
        debug_all_reached_pub.publish(all_reached_msg);

        if(!all_reached) {
            pub.publish(cmd);
            std::ostringstream ss;
            ss << "[Control] Targets:";
            for(auto& g : current_goals) ss << " " << g;
            ss << " | Cmds:";
            for(auto& c : current_values) ss << " " << c;
            ROS_INFO_STREAM_THROTTLE(0.5, ss.str());
        } else {
            ROS_INFO_ONCE("[SUCCESS] All joints reached target positions");
        }

        rate.sleep();
    }
    return 0;
}
