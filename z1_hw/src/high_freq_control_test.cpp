/********
更新：
本版本专用于性能测试。

最后编辑于2026.01.20 张耀华
********/


/********
更新：
添加“初始位置ros参数控制”。

最后编辑于2025.05.06 张耀华
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
/******** 
要求 z1_ros/z1_hw/config/controllers.yaml 配置 joint_group_position_controller
并要求 z1_ros/z1_hw/launch/z1_hw.launch 中包含 <arg name="controllers" default="joint_state_controller joint_group_position_controller" />

启动参数：
rosrun z1_hw high_freq_control _debug_init_from_current:=true
rosrun z1_hw high_freq_control -0.3, 0.3, -0.3, -0.3 ,0.3, 0.3 1

Tips:
未对关节指令做限位，所给关节角度超出实际可达位置后，可能会卡死

使用说明：
1.通过命令行设置初始位置（可选）：
rosrun z1_hw high_freq_control -0.3 0.3 -0.3 -0.3 0.3 0.3 1

2.通过ROS话题实时更新目标位置：
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
#include <sensor_msgs/JointState.h>

#include <cstdlib>
#include <mutex>
#include <cmath>
#include <atomic>
#include <vector>
#include <map>
#include <sstream>

// 机械臂关节速度统一控制，可取小数，设为1较为稳定
#define DEFAULT_JOINT_SPEED 1

// ====================== 采集埋点（图6） ======================
// 使用 ros::WallTime，避免 /use_sim_time 或时间源异常导致时间戳为 1970
ros::Publisher g_pub_tick;     // 每个控制周期一次，用于 jitter
ros::Publisher g_pub_goal_rx;  // 目标到达时刻
ros::Publisher g_pub_cmd_pub;  // 指令发布时刻
ros::Publisher g_pub_act_rx;   // 反馈到达时刻

static inline void publishWallTimeNow(ros::Publisher& pub) {
    std_msgs::Float64 m;
    m.data = ros::WallTime::now().toSec();
    pub.publish(m);
}
// ============================================================

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
        // 埋点：反馈到达时刻（WallTime）
        if (g_pub_act_rx) {
            publishWallTimeNow(g_pub_act_rx);
        }

        std::lock_guard<std::mutex> lock(data_mutex);
        for (int i = 0; i < 6; i++) {
            actual_joint_positions[i] = msg->position[i];
        }

        if (first_callback && DEBUG_INIT_FROM_CURRENT) {
            position_initialized = true;
            first_callback = false;
            current_values = actual_joint_positions;

            printDoubleArray(actual_joint_positions, "[INIT] Initial position detected: ", 2);
            printDoubleArray(current_values, "[INIT] Starting control from: ", 1);
        }
    }
}

void goalPositionsCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    // 埋点：目标到达时刻（WallTime）
    if (g_pub_goal_rx) {
        publishWallTimeNow(g_pub_goal_rx);
    }

    if (msg->data.size() == 6) {
        std::lock_guard<std::mutex> lock(goal_positions_mutex);
        for (int i = 0; i < 6; i++) {
            goal_positions[i] = msg->data[i];
        }
        // printDoubleArray(goal_positions, "[GOAL] New target received: ", 1);
    } else {
        ROS_WARN_STREAM("Invalid goal size: " << msg->data.size() << ", expected 6");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "high_freq_controller");
    ros::NodeHandle nh;

    // 初始化调试参数
    nh.param("debug_init_from_current", DEBUG_INIT_FROM_CURRENT, true);
    const double DEFAULT_SPEED = DEFAULT_JOINT_SPEED;

    // 预设位置集合
    std::map<std::string, std::vector<double>> PRESET_POSITIONS = {
        {"default",    {0.0, 0.0, -0.005, -0.074, 0.0, 0.0}},   // 零位
        {"forward0",   {0.0, 1.5, -1.0, -0.54, 0.0, 0.0}},     // 官方工作位置
        {"forward",    {0.0, 1.3, -1.0, -0.32, 0.0, 0.0}},     // 我的工作位置
        {"startFlat",  {0.0, 0.0, -0.005, -0.074, 0.0, 0.0}},  // 官方零位，同 "default"
        {"show_left",  {-1.0, 0.9, -1.0, -0.3, 0.0, 0.0}},
        {"show_mid",   {0.0, 0.9, -1.2, 0.25, 0.0, 0.0}},
        {"show_right", {1.0, 0.9, -1.0, -0.3, 0.0, 0.0}}
    };

    // 从参数服务器获取预设位置名称
    std::string preset_name;
    nh.param<std::string>("initial_position", preset_name, "default");

    // 获取对应的预设位置
    std::vector<double> selected_preset = PRESET_POSITIONS["default"];
    if (PRESET_POSITIONS.find(preset_name) != PRESET_POSITIONS.end()) {
        selected_preset = PRESET_POSITIONS[preset_name];
    } else {
        ROS_WARN("Unknown initial_position: %s, using default preset.", preset_name.c_str());
    }

    // 初始化目标位置：命令行优先
    std::vector<double> command_line_goals;
    for (int i = 0; i < 6 && (i + 1) < argc; i++) {
        command_line_goals.push_back(atof(argv[i + 1]));
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

    // ---------------------- ROS 通信 ----------------------
    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/joint_group_position_controller/command", 1);
    ros::Subscriber joint_sub = nh.subscribe("/joint_states", 1, jointStateCallback);
    ros::Subscriber goal_sub  = nh.subscribe("/goal_joint_positions", 1, goalPositionsCallback);

    // 图6埋点话题（WallTime, seconds）
    g_pub_tick    = nh.advertise<std_msgs::Float64>("/hf_tick", 10);
    g_pub_goal_rx = nh.advertise<std_msgs::Float64>("/hf_goal_rx", 10);
    g_pub_cmd_pub = nh.advertise<std_msgs::Float64>("/hf_cmd_pub", 10);
    g_pub_act_rx  = nh.advertise<std_msgs::Float64>("/hf_act_rx", 10);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    // -----------------------------------------------------

    // 控制参数
    const double CONTROL_STEP   = 0.004;
    const double POS_TOLERANCE  = 0.001;
    ros::Rate rate(300);

    while (ros::ok()) {
        // 埋点：控制周期 tick（用于 jitter）
        if (g_pub_tick) {
            publishWallTimeNow(g_pub_tick);
        }

        std_msgs::Float64MultiArray cmd;
        cmd.data.resize(6);

        // 获取当前实际位置和目标位置
        std::vector<double> local_actual_pos;
        std::vector<double> current_goals;
        {
            std::lock_guard<std::mutex> lock1(data_mutex);
            std::lock_guard<std::mutex> lock2(goal_positions_mutex);
            local_actual_pos = actual_joint_positions;
            current_goals = goal_positions;
        }

        // 核心控制逻辑
        bool all_reached = true;
        for (int i = 0; i < 6; i++) {
            if (fabs(current_values[i] - current_goals[i]) > POS_TOLERANCE) {
                all_reached = false;

                double error = local_actual_pos[i] - current_values[i];
                double step = joint_speed * CONTROL_STEP * (1.0 + 0.5 * error);

                // 注意：step 理论上可能为负（当 error < -2rad），工程上可进一步夹紧
                if (current_goals[i] > current_values[i]) {
                    current_values[i] += std::min(step, current_goals[i] - current_values[i]);
                } else {
                    current_values[i] -= std::min(step, current_values[i] - current_goals[i]);
                }

                cmd.data[i] = current_values[i];
            } else {
                // 到位时：同步内部状态，保证下一周期一致
                current_values[i] = current_goals[i];
                cmd.data[i] = current_goals[i];
            }
        }

        // 埋点：指令发布时刻（publish 前）
        if (g_pub_cmd_pub) {
            publishWallTimeNow(g_pub_cmd_pub);
        }

        // 改动要点：每周期都发布，保证 command 流连续（用于 jitter/时延统计）
        pub.publish(cmd);

        if (!all_reached) {
            std::ostringstream ss;
            ss << "[Control] Targets:";
            for (auto& g : current_goals) ss << " " << g;
            ss << " | Cmds:";
            for (auto& c : current_values) ss << " " << c;
            ROS_INFO_STREAM_THROTTLE(0.5, ss.str());
        } else {
            ROS_INFO_ONCE("[SUCCESS] All joints reached target positions");
        }

        rate.sleep();
    }

    return 0;
}
